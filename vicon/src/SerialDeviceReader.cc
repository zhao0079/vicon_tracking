/* Driver for reading data from a serial port

    Aleksandr Kushleyev <akushley(at)seas(dot)upenn(dot)edu>
    University of Pennsylvania, 2008

    BSD license.
    --------------------------------------------------------------------
    Copyright (c) 2008 Aleksandr Kushleyev
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    3. The name of the author may not be used to endorse or promote products
      derived from this software without specific prior written permission.

      THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
      IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
      OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
      IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
      INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
      NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
      THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "SerialDeviceReader.h"

SerialDeviceReader::SerialDeviceReader(int buffer_length, int num_buffers)
{
	m_thread_running=false;

  //allocate memory for the ring buffer
	m_data = new char[buffer_length*num_buffers];
  
  //allocate memory for array, containing number of chars read into a particular buffer
  m_data_length=new int[num_buffers];
  
  //memory for keeping track of whether a particular buffer has been read or not
  m_data_fresh = new int[num_buffers];

  //space for time stamps
  m_time_stamps = new double[num_buffers];

  //initialize timestamps to zero
  double t = m_timer0.Tic();
  for (int i=0; i< num_buffers; i++)
  {
    m_time_stamps[i] = t;
  }
  
  //zero out variables
  memset(m_data,0,buffer_length*num_buffers);
  memset(m_data_length,0,num_buffers*sizeof(int));
  memset(m_data_fresh,0,num_buffers*sizeof(int));
  
  //length of a single buffer
	m_buffer_length = buffer_length;
	
  //number of buffers in the ring buffer
  m_num_buffers = num_buffers;
	m_write_cntr=0;
	m_read_cntr=0;
  m_latest_cntr=-1;

	writing=false;
	reading=false;

  //init mutexes
	pthread_mutex_init(&m_data_mutex,NULL);
	pthread_cond_init(&m_data_cond,NULL);
}

SerialDeviceReader::~SerialDeviceReader()
{
	//free up memory
  if (m_data != NULL)	delete [] m_data;
  if (m_data_length != NULL) delete [] m_data_length;
  if (m_data_fresh != NULL) delete [] m_data_fresh;
  if (m_time_stamps != NULL) delete [] m_time_stamps;
  
  ///stop the thread and shutdown the device
  StopThread();
  StopDevice();

  //destroy mutexes
	pthread_mutex_destroy(&m_data_mutex);
	pthread_cond_destroy(&m_data_cond);
}

//dummy function to be implemented in child class
int SerialDeviceReader::StartDevice()
{
	return 0;
}

//dummy function to be implemented in child class
int SerialDeviceReader::StopDevice()
{
	return 0;
}

//start the main thread
int SerialDeviceReader::StartThread()
{
	
	std::cout<<"SerialDeviceReader:StartThread: Starting thread..."; fflush(stdout);

  if (pthread_create(&reader_thread, NULL, ReaderThreadFunc, (void *)this))
  {
    std::cout<<"SerialDeviceReader:StartThread: Could not start thread"<<std::endl;
    return -1;
  }
  std::cout<<"done"<<std::endl;

	m_thread_running=true;
	return 0;
}

int SerialDeviceReader::StopThread()
{
	if (m_thread_running)
  {
    std::cout<<"SerialDeviceReader::StopThread: Stopping thread..."; fflush(stdout); 
    pthread_cancel(reader_thread);
    pthread_join(reader_thread,NULL);
    std::cout<<" done"<<std::endl; fflush(stdout); 
    m_thread_running=false;
  }

	return 0;
}

//dummy function for running the main loop
void *SerialDeviceReader::ReaderThreadFunc(void * arg_in)
{
	sigset_t sigs;
	sigfillset(&sigs);
	pthread_sigmask(SIG_BLOCK,&sigs,NULL);

	SerialDeviceReader * sd_reader = (SerialDeviceReader *) arg_in;
  
  int n_errors_total=0;
  int n_errors_consecutive=0;

	while(1)
  {
    //see if we need to cancel the thread		
    pthread_testcancel();

    //run the update function
		if (sd_reader->UpdateFunction() != 0)
    {
      n_errors_total++;
      n_errors_consecutive++;
      if (n_errors_consecutive >= MAX_NUM_CONSECUTIVE_ERRORS)
      {
        std::cout<<"************************************************************************************"<<std::endl;        
        std::cout<<"SerialDeviceReader::ReaderThreadFunc: exiting because of too many consecutive errors"<<std::endl;
        std::cout<<"************************************************************************************"<<std::endl;
        pthread_exit(NULL);
      }
      usleep(UPDATE_FUNCTION_ERROR_SLEEP_US);
    }
    else
    {
      //reset the consecutive error count
      n_errors_consecutive=0;
    }
	}
}

//dummy function to be implemented in the child class
//read from the device and fill the buffers here
int SerialDeviceReader::UpdateFunction()
{
	std::cout<<"."; fflush(stdout);
	usleep(100000);
	return 0;
}

void SerialDeviceReader::LockDataMutex()
{
  pthread_mutex_lock(&m_data_mutex);
}

void SerialDeviceReader::UnlockDataMutex()
{
  pthread_mutex_unlock(&m_data_mutex);
}

//get a pointer to a buffer for writing
int SerialDeviceReader::GetWritePtr(char ** data_ptr)
{
	LockDataMutex();
  
  if (writing)
  {
    std::cout<<"SerialDeviceReader::GetReadPtr: ERROR: already requested a write pointer"<<std::endl;
    UnlockDataMutex();
    return -1;
  }
	
	if (m_write_cntr==m_read_cntr)
  {
		//std::cout<<"SerialDeviceReader::GetWritePtr: WARNING: write cntr has reached read cntr"<<std::endl;
		if (reading)
    {		
			m_write_cntr=(m_write_cntr+1)%m_num_buffers;
			std::cout<<"SerialDeviceReader::GetWritePtr: WARNING: write cntr jumped over read cntr"<<std::endl;
		}	
	}

	*data_ptr = (char *) (m_data + m_write_cntr*m_buffer_length);

	//std::cout<<"_"<<m_write_cntr<<"__"<<m_read_cntr<<"_"<<std::endl;
	writing=true;
	UnlockDataMutex();
	return 0;
}

//call this when done writing to a buffer to make the data available for reading
void SerialDeviceReader::DoneWriting(int data_length)
{
	LockDataMutex();
  if (writing)      //make sure that we were writing
  {
    m_data_length[m_write_cntr]=data_length;
    if (data_length > 0)
    {
      m_data_fresh[m_write_cntr] = 1;
      //m_time_stamps[m_write_cntr] = m_timer0.Toc();
      m_time_stamps[m_write_cntr] = m_timer0.GetAbsoluteTime();
      //std::cout<<"write timestamp = "<<m_time_stamps[m_write_cntr]<<std::endl;
      m_latest_cntr=m_write_cntr;
      m_write_cntr=(m_write_cntr+1)%m_num_buffers;
      pthread_cond_signal(&m_data_cond);
    }
    else 
    {
      m_data_fresh[m_write_cntr] = 0;
    }
    writing=false;
    pthread_cond_signal(&m_data_cond);
  }
	UnlockDataMutex();        
}

//get a pointer from the ring buffer for reading data
//the pointers will be returned in order of filling
//unless you read too slow, then some data will be lost
int SerialDeviceReader::GetReadPtr(const char ** data_ptr, int & data_length, double & time_stamp, double timeout_sec)
{
	LockDataMutex();		

  if (reading)
  {
    std::cout<<"SerialDeviceReader::GetReadPtr: ERROR: already requested a read pointer"<<std::endl;
    UnlockDataMutex();
    return -1;
  }

	if (m_read_cntr==m_write_cntr)
  {
		//std::cout<<"SerialDeviceReader::GetReadPtr: no fresh data available"<<std::endl;
		timeval timeStart;
  	timespec maxWaitTime;
  	gettimeofday(&timeStart,NULL);
  
		int seconds=(int)timeout_sec;
		int u_seconds = 1000000*(timeout_sec-seconds);

  	maxWaitTime.tv_sec=timeStart.tv_sec + seconds + ((timeStart.tv_usec < 1000000-u_seconds) ? 0:1);
  	maxWaitTime.tv_nsec=(timeStart.tv_usec*1000 + u_seconds*1000)%1000000000;

		int ret=pthread_cond_timedwait(&m_data_cond,&m_data_mutex,&maxWaitTime);
    if (ret==ETIMEDOUT)
    {
      std::cout<<"SerialDeviceReader::GetReadPtr: timeout!"<<std::endl;
			UnlockDataMutex();
			return -1;
    }
    
    if (m_read_cntr==m_write_cntr)
    {
      std::cout<<"SerialDeviceReader::GetReadPtr: m_read_cntr=m_write_cntr"<<std::endl;
			UnlockDataMutex();
			return -1;
    }

    if (m_data_fresh[m_read_cntr] == 0)
    {
      std::cout<<"SerialDeviceReader::GetReadPtr: m_data_fresh[m_read_cntr] == 0"<<std::endl;
			UnlockDataMutex();
			return -1;
    }
	}
	
	*data_ptr = (char *)(m_data + m_read_cntr*m_buffer_length);
  data_length=m_data_length[m_read_cntr];
  time_stamp = m_time_stamps[m_read_cntr];
  m_data_fresh[m_read_cntr] = 0;
	reading=true;
	UnlockDataMutex();
	return 0;
}

int SerialDeviceReader::GetReadPtrLatest(const char ** data_ptr, int & data_length, double timeout_sec)
{
	LockDataMutex();		

  if (reading)
  {
    std::cout<<"SerialDeviceReader::GetReadPtrLatest: ERROR: already requested a read pointer"<<std::endl;
    UnlockDataMutex();
    return -1;
  }
    
	if ( (m_latest_cntr < 0) || (m_data_fresh[m_latest_cntr] == 0) )
  {
    //std::cout<<"SerialDeviceReader::GetReadPtrLatest: no fresh data available"<<std::endl;
		timeval timeStart;
  	timespec maxWaitTime;
  	gettimeofday(&timeStart,NULL);
  
		int seconds=(int)timeout_sec;
		int u_seconds = 1000000*(timeout_sec-seconds);

  	maxWaitTime.tv_sec=timeStart.tv_sec + seconds + ((timeStart.tv_usec < 1000000-u_seconds) ? 0:1);
  	maxWaitTime.tv_nsec=(timeStart.tv_usec*1000 + u_seconds*1000)%1000000000;

		int ret=pthread_cond_timedwait(&m_data_cond,&m_data_mutex,&maxWaitTime);
    if ((ret==ETIMEDOUT) || (m_latest_cntr < 0) || (m_data_fresh[m_latest_cntr] == 0) )
    {
      std::cout<<"SerialDeviceReader::GetReadPtrLatest: timeout!"<<std::endl;
			UnlockDataMutex();
			return -1;
    }
	}
	
  //update the read counter because we are not following the order but reading the latest
  m_read_cntr=m_latest_cntr;
  
	*data_ptr = (char *)(m_data + m_read_cntr*m_buffer_length);
  data_length=m_data_length[m_read_cntr];
  m_data_fresh[m_read_cntr] = 0;
	reading=true;
	UnlockDataMutex();
	return 0;
}

//call this when done reading to make memory available for writing new data
int SerialDeviceReader::DoneReading(int * num_packets_remaining)
{
	LockDataMutex();
  if (reading)    //make sure that we were reading
  {
    if (num_packets_remaining != NULL)
    {
      *num_packets_remaining = m_write_cntr - m_read_cntr - 1;
      if (*num_packets_remaining < 0)
        *num_packets_remaining = *num_packets_remaining + m_num_buffers;
    }
    m_read_cntr=(m_read_cntr+1)%m_num_buffers;
    reading=false;
  }

	UnlockDataMutex();
	return 0;
}

int SerialDeviceReader::GetBufferLength() { return m_buffer_length; }
bool SerialDeviceReader::IsThreadRunning() { return m_thread_running; }

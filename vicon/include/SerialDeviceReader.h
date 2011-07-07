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

#ifndef SERIAL_DEVICE_READER_H
#define SERIAL_DEVICE_READER_H
#include <pthread.h>

#include "SerialDevice.h"
#include "Timer.h"


#include <csignal>


#define UPDATE_FUNCTION_ERROR_SLEEP_US 10000
#define MAX_NUM_CONSECUTIVE_ERRORS 10

class SerialDeviceReader : public SerialDevice 
{

	public:
		SerialDeviceReader(int buffer_length, int num_buffers);
		~SerialDeviceReader();

		int StartThread();
		int StopThread();
		virtual int StartDevice();
		virtual int StopDevice();

		int GetWritePtr(char ** data_ptr);
		void DoneWriting(int data_length);

		int GetReadPtr(const char ** data_ptr,int & data_length, double & time_stamp, double timeout_sec);
    int GetReadPtrLatest(const char ** data_ptr, int & data_length ,double timeout_sec);
		int DoneReading(int * num_packets_remaining = NULL);

		void LockDataMutex();
		void UnlockDataMutex();
    
    int GetBufferLength();
    bool IsThreadRunning();

		
	private:

		pthread_mutex_t m_data_mutex;
		pthread_cond_t m_data_cond;
		pthread_t reader_thread;
  	int thread_id;
  	static void *ReaderThreadFunc(void * input);

    Timer m_timer0;		
  
		bool m_thread_running;
		virtual int UpdateFunction();

		char * m_data;
    int * m_data_length;
    int * m_data_fresh;
    double * m_time_stamps;
		int m_buffer_length;
		int m_num_buffers;

		int m_read_cntr;
		int m_write_cntr;
    int m_latest_cntr;

		bool reading;
		bool writing;

};

#endif //SERIAL_DEVICE_READER_H

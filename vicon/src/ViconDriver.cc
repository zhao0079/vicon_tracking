/* C++ driver for interfacing to a Vicon tracking system
   Requires SerialDevice and SerialDeviceReader classes

   Aleksandr Kushleyev
   akushley (at) seas (dot) upenn (dot) edu
   University of Pennsylvania
   December 2008

   Note: If you stop the vicon stream while the driver is
   reading, the driver will block forever and will use
   100% cpu. This happens because of some strange condition
   when the tcp socket is closed.

*/

#include "ViconDriver.h"

ViconDriver::ViconDriver(unsigned int buffer_length,
                         unsigned int num_buffers, bool stream) : SerialDeviceReader(buffer_length,
                                                                                     num_buffers)
{
  m_synchronized   = false;
  m_streaming      = false;
  m_stream_mode    = stream;
  m_got_names      = false;
  m_num_data       = 0;
  m_names.resize(0);
}

ViconDriver::~ViconDriver()
{
  StopThread();   //stop the driver thead
  StopDevice();   //send the stop command to the device
  Disconnect();   //disconnect from the device
}

int ViconDriver::GetNames(vector<string> & names)
{
  if (!m_got_names)  //the names have not been requested before (device probably not started)
    {
      if (RequestAndGetNames(m_names) != 0)
        {
          cout<< "ViconDriver::GetNames: could not request and get names" << endl;
          return -1;
        }
    }
  
  //copy the names into the output vector
  names = m_names;
  return 0;
}

int ViconDriver::StartDevice()
{
  //make sure that we are not streaming
  if (m_streaming)
    {
      cout<< "ViconDriver::StartDevice: the device should not be streaming before it is started.." << endl;
      return -1;
    }
  
  //get the names from the device
  if (RequestAndGetNames(m_names) != 0)
    {
      cout<< "ViconDriver::StartDevice: could not get names" << endl;
      return -1;
    }
  
  //start streaming mode if needed
  if (m_stream_mode)
    {
      if (StartStreaming() != 0)
        {
          cout << "ViconDriver::StartDevice: Could not start streaming" <<endl;
          return -1;
        }
    }
  return 0;
}

int ViconDriver::StopDevice()
{
  StopStreaming();
  return 0;
}

int ViconDriver::StartStreaming()
{
  char out_buff[2*sizeof(int)];
  int * p_buff = (int *)out_buff;

  //make the info request message
  p_buff[0] = ClientCodes::EStreamOn;
  p_buff[1] = ClientCodes::ERequest;

  FlushInputBuffer();

  //send the info request packet
  if (WriteChars(out_buff,2*sizeof(int)) != 2*sizeof(int))
    {
      cout << "ViconDriver::StartStreaming: Error writing info request bytes!!!" <<endl;
      return -1;
    }

  //try getting values to make sure that the streaming started
  vector<double> values;  
  if (GetValues(values) != 0)
    {
      cout << "ViconDriver::StartStreaming: could not get values to confirm that streaming started" <<endl;
      return -1;
    }

  cout << "ViconDriver::StartStreaming: device set to streaming mode" <<endl;
  m_streaming = true;
  return 0;
}

int ViconDriver::StopStreaming()
{
  if (!m_streaming)
    {
      //cout << "ViconDriver::StopStreaming: not sending stop streaming cmd because not streaming." <<endl;
      return -1;
    }
  
  char out_buff[2*sizeof(int)];
  int * p_buff = (int *)out_buff;

  //make the info request message
  p_buff[0] = ClientCodes::EStreamOff;
  p_buff[1] = ClientCodes::ERequest;

  FlushInputBuffer();

  //send the info request packet
  if (WriteChars(out_buff,2*sizeof(int)) != 2*sizeof(int))
    {
      cout << "ViconDriver::StopStreaming: Error writing info request bytes!!!" <<endl;
      return -1;
    }
  cout << "ViconDriver::StopStreaming: streaming stopped." <<endl;
  m_streaming = false;
  return 0;
}

int ViconDriver::UpdateFunction()
{
  if (!m_stream_mode)
    {
      cout<<"ViconDriver::UpdateFunction: ERROR: please set the stream mode on if"<< 
        "you would like to use the threaded driver. Exiting."<<endl;
      exit(1);
    }

  if (!m_streaming)
    {
      cout<<"ViconDriver::UpdateFunction: Warning: streaming not turned on. Attempting to start streaming."<<endl;
      if (StartStreaming() != 0)
        {
          cout<<"ViconDriver::UpdateFunction: ERROR: could not start streaming"<<endl;
          return -1;
        }
    }
  
  //TODO: synchronization stuff

  //read back the packet type
  if (ReadLongIntAndCompare(ClientCodes::EData,VICON_READ_TIMEOUT_US,"packet type"))
    {
      cout <<"ViconDriver::UpdateFunction: Bad response"<<endl;
      return -1;
    }

  //read back the contents type (request/reply)
  if (ReadLongIntAndCompare(ClientCodes::EReply,VICON_READ_TIMEOUT_US,"contents type"))
    {
      cout <<"ViconDriver::UpdateFunction: Bad response"<<endl;
      return -1;
    }
  
  char * data_ptr;
  
  //read the number of items (values)
  int num_data;
  int num_read = ReadChars((char*)&num_data,sizeof(int),VICON_READ_TIMEOUT_US);
  if (num_read != sizeof(int))
    {
      cout <<"ViconDriver::UpdateFunction: could not read the size of the data packet" << endl;
      return -1;
    }
  
  //sanity check on num_data
  if ( (num_data < 0) || (num_data > VICON_MAX_NUM_DATA))
    {
      cout<<"ViconDriver::UpdateFunction: ERROR: bad number of expected data values: "<<num_data<<endl;
      return -1;
    }

  //request a pointer to a free buffer
  if (GetWritePtr(&data_ptr)){
    cout<<"ViconDriver::UpdateFunction: ERROR: could not get write pointer"<<endl;
    return -1;
  }

  //read the values and verify the amount of data read
  num_read = ReadChars(data_ptr,num_data*sizeof(double),VICON_READ_TIMEOUT_US);
  if (num_read != (int)(num_data*sizeof(double)))
    {
      cout <<"ViconDriver::UpdateFunction: could not read values. Read "<<num_read
           <<" chars. Expected:"<< num_data*sizeof(double) << endl;
    
      //release the pointer and set an error for number of bytes read
      DoneWriting(-1);
      return -1;
    }
  
  //release the pointer and set the number of bytes read
  DoneWriting(num_read);
  
  return 0;
}

int ViconDriver::ReadLongIntAndCompare(const int & val_exp, 
                                       int timeout_us, const char * val_type)
{
  int val=0;
  int num_read;

  //read the value
  num_read = ReadChars((char*)&val,sizeof(int),timeout_us);
  if (num_read != sizeof(int))
    {
      cout << "ViconDriver::ReadLongIntAndCompare: could not read value ("<<val_type<<")" << endl;
      return -1;
    }

  //check value
  if (val != val_exp)
    {
      cout << "ViconDriver::ReadLongIntAndCompare: value does not match ("<<val_type<<")"<< endl;
      cout << "expected: "<< val_exp <<" got: "<< val <<endl;
      return -1;
    }
  return 0;
}

int ViconDriver::RequestAndGetNames(vector<string> & names)
{
  if (m_streaming)
    {
      cout << "ViconDriver::RequestAndGetNames: Vicon is streaming - please stop the stream first" <<endl;
      return -1;
    }
  char out_buff[2*sizeof(int)];
  int * p_buff = (int *)out_buff;

  //clear out the names   
  names.clear();
  
  //make the info request message
  p_buff[0] = ClientCodes::EInfo;
  p_buff[1] = ClientCodes::ERequest;

  FlushInputBuffer();

  //send the info request packet
  if (WriteChars(out_buff,2*sizeof(int)) != 2*sizeof(int))
    {
      cout << "ViconDriver::RequestAndGetNames: Error writing info request bytes!!!" <<endl; fflush(stdout);
      return -1;
    }

  //read back the packet type
  if (ReadLongIntAndCompare(ClientCodes::EInfo,VICON_READ_TIMEOUT_US,"packet type"))
    {
      cout <<"ViconDriver::RequestAndGetNames: Bad response"<<endl;
      return -1;
    }

  //read back the contents type (request/reply)
  if (ReadLongIntAndCompare(ClientCodes::EReply,VICON_READ_TIMEOUT_US,"contents type"))
    {
      cout <<"ViconDriver::RequestAndGetNames: Bad response"<<endl;
      return -1;
    }

  int num_items;
  int num_read = ReadChars((char*)&num_items,sizeof(int),VICON_READ_TIMEOUT_US);
  if (num_read != sizeof(int))
    {
      cout << "ViconDriver::RequestAndGetNames: could not read the size of the packet" << endl;
      return -1;
    }

  int slen;
  char in_buff[255];
  for (int i =0; i < num_items; i++)
    {
      //read off the string length
      num_read = ReadChars((char*)&slen,sizeof(int),VICON_READ_TIMEOUT_US);
      if (num_read != sizeof(int))
        {
          cout << "ViconDriver::RequestAndGetNames: could not read string length #"<<i<< endl;
          return -1;
        }

      //read the actual string
      num_read = ReadChars(in_buff,slen,VICON_READ_TIMEOUT_US);
      if (num_read != slen)
        {
          cout << "ViconDriver::RequestAndGetNames: could not read string #"<<i<< endl;
          return -1;
        }
    
      //null-terminate the string
      in_buff[slen] = 0;
      names.push_back(string(in_buff));
    }

  m_num_data=num_items;
  m_got_names=true;
  return 0;
}


int ViconDriver::RequestValues()
{
  if (m_stream_mode)
    {
      cout << "ViconDriver::RequestValues: this function cannot be used in streaming mode" <<endl;
      return -1;
    }
  char out_buff[2*sizeof(int)];
  int * p_buff = (int *)out_buff;

  //make the info request message
  p_buff[0] = ClientCodes::EData;
  p_buff[1] = ClientCodes::ERequest;

  FlushInputBuffer();

  //send the info request packet
  if (WriteChars(out_buff,2*sizeof(int)) != 2*sizeof(int))
    {
      cout << "ViconDriver::RequestValues: Error writing info request bytes!!!" <<endl;
      return -1;
    }

  return 0;
}

int ViconDriver::SynchronizeWithStream()
{
  if ( (!m_stream_mode) || (!m_streaming) )
    {
      cout << "ViconDriver::RequestValues: not streaming or not in stream mode" <<endl;
      return -1;
    }
  return 0;
}


int ViconDriver::GetValues(vector<double> & values)
{
  if (IsThreadRunning())
    {
      cout << "ViconDriver::GetValues: this function cannot be used in multi-threaded mode" <<endl;
      exit(1);
    }  

  //clear out the values   
  values.clear();

  //read back the packet type
  if (ReadLongIntAndCompare(ClientCodes::EData,VICON_READ_TIMEOUT_US,"packet type"))
    {
      cout <<"ViconDriver::GetValues: Bad response"<<endl;
      return -1;
    }

  //read back the contents type (request/reply)
  if (ReadLongIntAndCompare(ClientCodes::EReply,VICON_READ_TIMEOUT_US,"contents type"))
    {
      cout <<"ViconDriver::GetValues: Bad response"<<endl;
      return -1;
    }

  //read the number of items (values)
  int num_data;
  int num_read = ReadChars((char*)&num_data,sizeof(int),VICON_READ_TIMEOUT_US);
  if (num_read != sizeof(int))
    {
      cout <<"ViconDriver::GetValues: could not read the size of the data packet" << endl;
      return -1;
    }

  values.resize(num_data);
  num_read = ReadChars((char*)&values[0],num_data*sizeof(double),VICON_READ_TIMEOUT_US);
  if (num_read != (int)(num_data*sizeof(double)))
    {
      cout <<"ViconDriver::GetValues: could not read values. Read "<<num_read
           <<" chars. Expected:"<< num_data*sizeof(double) << endl;
      return -1;
    }

  return 0;
}

int ViconDriver::GetValuesThreaded(vector< vector<double> > & values, 
                                   vector<double> & time_stamps,
                                   double timeout_sec)
{
  if (!IsThreadRunning())
    {
      cout << "ViconDriver::GetValuesThreaded: ERROR: main thread is not running." <<endl;
      return -1;
    } 

  char * read_ptr;
  int data_length;
  double time_stamp;
  int n_packets_remaining=1;
  int n_packets_got = 0;

  //clear out output variables
  values.clear();
  time_stamps.clear();
  
  vector<double> temp_values;

#ifdef VICON_DRIVER_DEBUG
  cout <<"ViconDriver::GetValuesThreaded: about to read data from the buffer" <<endl;
#endif
    
  //read off all the data from the ring buffers
  while (n_packets_remaining > 0)
    {
      if (GetReadPtr((const char **)(&read_ptr),data_length,time_stamp,timeout_sec)==0)
        {
          if (data_length % sizeof(double) != 0)
            {
              DoneReading(&n_packets_remaining);
              cout << "ViconDriver::GetValuesThreaded: bad data length (not a multiple of sizeof(double):" <<data_length<<endl;
              continue;
            }
      
          //calculate the number of variables
          int n_data = data_length / sizeof(double);
        
#ifdef VICON_DRIVER_DEBUG
          cout << "ViconDriver::GetValuesThreaded: got n_data "<<n_data<<endl;
#endif 
      
          //store the data into temp array
          values.push_back(temp_values);
          values[n_packets_got].resize(n_data);
        
#ifdef VICON_DRIVER_DEBUG
          cout << "ViconDriver::GetValuesThreaded: resized container" << endl;
#endif
          memcpy(&((values[n_packets_got])[0]),read_ptr,data_length);

#ifdef VICON_DRIVER_DEBUG        
          cout << "ViconDriver::GetValuesThreaded: copied data" <<endl;
#endif        
          //store the timestamp
          time_stamps.push_back(time_stamp);
          n_packets_got++;

          //let the driver know that we are done with data and 
          //receive number of available packets in the buffer
          DoneReading(&n_packets_remaining);
        }
      else
        {
          cout << "ViconDriver::GetValuesThreaded: could not read a packet" << endl;
          return -1;
        }
    }

  return 0;
}

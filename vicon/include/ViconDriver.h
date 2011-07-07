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

#ifndef VICON_DRIVER_H
#define VICON_DRIVER_H

#include <string>
#include <iostream>
#include <vector>

#include "SerialDeviceReader.h"
#include "ClientCodes.h"
#include "Timer.h"

#define VICON_READ_TIMEOUT_US 1000000
#define VICON_DEFAULT_GET_VALUES_TIMEOUT_SEC 0.1
#define VICON_MAX_NUM_DATA 1024  //maximum number of variables to expect from vicon

using namespace std;

class ViconDriver : public SerialDeviceReader
{
  public:
    ViconDriver(unsigned int buffer_length, 
                unsigned int num_buffers, bool stream);
    ~ViconDriver();

    virtual int StartDevice();
    virtual int StopDevice();

    int RequestAndGetNames(vector<string> & names);
    int GetNames(vector<string> & names);
    int GetValues(vector<double> & values);
    int RequestValues();
    int StartStreaming();
    int StopStreaming();
    int ParseValues(vector<double> & values);
    int GetValuesThreaded(vector< vector<double> > & values, 
                           vector<double> & time_stamps,
                           double timeout_sec = VICON_DEFAULT_GET_VALUES_TIMEOUT_SEC); 

  private:
    bool m_synchronized;
    bool m_streaming;
    bool m_stream_mode;
    bool m_got_names;
    long int m_num_data;
    vector<string> m_names;

    virtual int UpdateFunction();
    int SynchronizeWithStream();

    int ReadLongIntAndCompare(const int & val_exp, 
                              int timeout_us, const char * val_type);


};

#endif //VICON_DRIVER_H

#include <string>
#include <vector>
#include <ros/ros.h>

#include <vicon/Names.h>
#include <vicon/Values.h>
#include "ViconDriver.h"

ros::Publisher pub_names;
vicon::Names names_msg;

void names_callback(const ros::TimerEvent& e)
{
  pub_names.publish(names_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vicon");

  ros::NodeHandle n("~");

  pub_names = n.advertise<vicon::Names>("names", 100, true);

  ros::Publisher pub_values
    = n.advertise<vicon::Values>("values", 100);

  vicon::Values values_msg;
  n.param("frame_id", values_msg.header.frame_id,
          string("vicon"));

  const int buff_len = 1024;
  const int num_buff = 10;
  const bool stream = true;
  ViconDriver vd(buff_len, num_buff, stream);

  vector<double> values;
  vector<string> names;

  string server;
  n.param("server", server, string("alkaline"));
  int port;
  n.param("port", port, 800);

  // Connect
  if (vd.ConnectTCP(server.c_str(), port))
    {
      ROS_FATAL("%s: failed to connect to %s:%i",
                ros::this_node::getName().c_str(),
                server.c_str(), port);
      return -1;
    }

  // Start device
  if (vd.StartDevice() != 0)
    {
      ROS_FATAL("%s: could not start device",
                ros::this_node::getName().c_str());
      return -1;
    }

  //get names (buffered in the driver)
  if (vd.GetNames(names) != 0)
    {
      ROS_ERROR("%s: could not get names",
                ros::this_node::getName().c_str());
      return -1;
    }

  names_msg.names.resize(names.size());
  std::copy(names.begin(), names.end(),
            names_msg.names.begin());

  string fps = names[0];
  float nfps;
  sscanf(fps.c_str(),"%*s %f %*s %*s", &nfps);
  double vicon_dt = 1.0/nfps;

  values_msg.values.resize(names.size());

  ros::Timer timer = n.createTimer(ros::Duration(1),
                                   names_callback);

  bool init = false;

  unsigned int start_seq = 0;
  double start_time = ros::Time::now().toSec();

  while (n.ok())
    {
      //get values
      if (vd.GetValues(values) != 0)
        {
          ROS_INFO("%s: could not get values",
                   ros::this_node::getName().c_str());
          continue;
        }

      if (names.size() != values.size())
        {
          ROS_INFO("%s: names size does not match values size",
                   ros::this_node::getName().c_str());
          continue;
        }

      std::copy(values.begin(), values.end(),
                values_msg.values.begin());

      if (!init)
        {
          start_seq = (unsigned int)values[0];
          start_time = ros::Time::now().toSec();

          init = true;
        }

      values_msg.header.stamp = ros::Time(start_time + (values[0] - start_seq)*vicon_dt);

      double drift = (ros::Time::now() - values_msg.header.stamp).toSec();
      if (drift > 0.1)
        ROS_INFO("Drift = %f", (ros::Time::now() - values_msg.header.stamp).toSec());

      pub_values.publish(values_msg);

      ros::spinOnce();
    }

  if (vd.StopDevice() != 0)
    {
      ROS_FATAL("%s: could not stop device",
                ros::this_node::getName().c_str());
      return -1;
    }

  if (vd.Disconnect())
    {
      ROS_FATAL("%s: failed to disconnect",
                ros::this_node::getName().c_str());
    }

  return 0;
}

#include <string>
#include <armadillo>
#include <vector>
#include <fstream>
#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/Pose.h>
#include <vicon/Values.h>
#include <vicon/Names.h>
#include <XMLConfig.h>
#include <vicon_calibrate/Calibrate.h>

using namespace std;

int ParseVSK(string &file,
             vector<string> &names,
             vector<float> &values)
{
  XMLConfig config;

  if (config.Load(file) != 0)
    {
      ROS_ERROR("%s: Failed to load vsk file: %s",
                ros::this_node::getName().c_str(),
                file.c_str());
      return -1;
    }

  if (!config.HasElement("/KinematicModel/MarkerSet/Markers/Marker"))
    {
      ROS_ERROR("%s: Failed to locate Markers in vsk file %s",
                ros::this_node::getName().c_str(),
                file.c_str());
      return -1;
    }
  string model_name;

  int i = 0;
  bool model_set = false;
  ROS_DEBUG("%s: Finding markers", ros::this_node::getName().c_str());
  while (true)
    {
      XMLConfig* c =
        config.GetChildrenAsRoot("/KinematicModel/MarkerSet/Markers", i);

      if (c == NULL)
        break;

      if (!model_set)
        {
          if (!c->HasAttribute("SEGMENT"))
            {
              ROS_ERROR("%s: Failed to locate model name in vsk file %s",
                        ros::this_node::getName().c_str(),
                        file.c_str());
              return -1;
            }
          else
            {
              c->GetAttributeString("SEGMENT", model_name);
              model_set = true;
            }
        }

      if (c->HasAttribute("STATUS"))
        {
          if ((!c->HasAttribute("NAME")) &&
              (!c->HasAttribute("POSITION")))
            {
              ROS_ERROR("%s: Improperly formatted vsk file %s",
                        ros::this_node::getName().c_str(),
                        file.c_str());
              return -1;
            }

          string name;
          c->GetAttributeString(string("NAME"), name);
          names.push_back(model_name+string(":")+name);

          float x = c->GetAttributeTupleFloat(string("POSITION"), 0, 0);
          float y = c->GetAttributeTupleFloat(string("POSITION"), 1, 0);
          float z = c->GetAttributeTupleFloat(string("POSITION"), 2, 0);

          values.push_back(x);
          values.push_back(y);
          values.push_back(z);

          ROS_DEBUG("Name: %s, xyz: %f, %f, %f", name.c_str(), x, y, z);
        }

      delete c;
      i++;
    }

  return 0;
}

geometry_msgs::Pose pose_msg;

int ReadCalibration(string &file)
{
  XMLConfig calib;

  if (calib.Load(file) != 0)
    {
      ROS_FATAL("%s: Failed to load calibration file: %s",
                ros::this_node::getName().c_str(),
                file.c_str());
      return -1;
    }

  if (!calib.HasElement("/calibration/orientation"))
    {
      ROS_ERROR("%s: Failed to locate orientation in calibration file %s",
                ros::this_node::getName().c_str(),
                file.c_str());
      return -1;
    }

  if (!calib.HasElement("/calibration/position"))
    {
      ROS_ERROR("%s: Failed to locate position in calibration file %s",
                ros::this_node::getName().c_str(),
                file.c_str());
      return -1;
    }

  pose_msg.position.x = calib.GetFloat("/calibration/position/x", 0);
  pose_msg.position.y = calib.GetFloat("/calibration/position/y", 0);
  pose_msg.position.z = calib.GetFloat("/calibration/position/z", 0);

  btQuaternion q(calib.GetFloat("/calibration/orientation/x", 0),
                 calib.GetFloat("/calibration/orientation/y", 0),
                 calib.GetFloat("/calibration/orientation/z", 0),
                 calib.GetFloat("/calibration/orientation/w", 1));

  q.normalize();
  tf::quaternionTFToMsg(q, pose_msg.orientation);

  return 0;
}

vector<string> reference_names;
vector<unsigned int> reference_index;

int ReadReference(string &file,
                  vector<string> &names,
                  vector<float> &values)
{
  XMLConfig config;

  if (config.Load(file) != 0)
    {
      ROS_FATAL("%s: Failed to load reference file: %s",
                ros::this_node::getName().c_str(),
                file.c_str());
      return -1;
    }

  if (!config.HasElement("/reference/markers"))
    {
      ROS_ERROR("%s: Failed to locate markers in calibration file %s",
                ros::this_node::getName().c_str(),
                file.c_str());
      return -1;
    }

  int i = 0;

  ROS_DEBUG("%s: Finding markers", ros::this_node::getName().c_str());
  while (true)
    {
      XMLConfig* c =
        config.GetChildrenAsRoot("/reference/markers", i);

      if (c == NULL)
        break;

      if ((!c->HasElement("name")) &&
          (!c->HasElement("position")))
        {
          ROS_ERROR("%s: Improperly formatted reference file %s",
                    ros::this_node::getName().c_str(),
                    file.c_str());
          return -1;
        }

      string name;
      c->GetString(string("name"), name);
      names.push_back(name);

      float x = c->GetTupleFloat(string("position"), 0, 0);
      float y = c->GetTupleFloat(string("position"), 1, 0);
      float z = c->GetTupleFloat(string("position"), 2, 0);

      values.push_back(x);
      values.push_back(y);
      values.push_back(z);

      ROS_DEBUG("Name: %s, xyz: %f, %f, %f", name.c_str(), x, y, z);

      delete c;
      i++;
    }

  return 0;
}

bool names_set = false;
bool reference_available = false;
vector<string> vsk_names;
vector<unsigned int> vsk_index;

void names_callback(const vicon::Names::ConstPtr &msg)
{
  if (names_set)
    return;

  for (vector<string>::iterator i = vsk_names.begin();
       i != vsk_names.end(); ++i)
    for (unsigned int j = 0; j < msg->names.size(); j++) {
      string match_name = *i + string(" ");
      if (msg->names[j].find(match_name) != string::npos) {
        vsk_index.push_back(j);
      }
    }

  if (4*vsk_names.size() != vsk_index.size())
    {
      ROS_FATAL("Failed to extract names in vsk file from data");
      ros::shutdown();
    }

  if (reference_available)
    {
      for (vector<string>::iterator i = reference_names.begin();
           i != reference_names.end(); ++i)
        for (unsigned int j = 0; j < msg->names.size(); j++) {
	  string match_name = *i + string(" ");
          if (msg->names[j].find(match_name) != string::npos) {
            reference_index.push_back(j);
	  }
	}

      if (4*reference_names.size() != reference_index.size())
        {
          ROS_FATAL("Failed to extract reference names in vsk file from data");
          ros::shutdown();
        }
    }

  names_set = true;

  return;
}

void compute_transformation(vector<double> markers,
                            vector<float> reference,
                            vector<unsigned int> index,
                            arma::mat &R, arma::colvec &T)
{
  vector<unsigned int> visible;

  for (unsigned int i = 0; i < index.size()/4; i++)
    if (markers[index[4*i + 3]] < 1e-6)
      visible.push_back(i);

  unsigned int N = visible.size();

  arma::mat w(3, N);
  arma::mat b(3, N);

  for (unsigned int i = 0; i < visible.size(); i++)
    {
      w(0, i) = markers[index[4*visible[i]]];
      w(1, i) = markers[index[4*visible[i] + 1]];
      w(2, i) = markers[index[4*visible[i] + 2]];

      b(0, i) = reference[3*visible[i]];
      b(1, i) = reference[3*visible[i] + 1];
      b(2, i) = reference[3*visible[i] + 2];
    }

  arma::colvec wmean = arma::mean(w, 1);
  arma::colvec bmean = arma::mean(b, 1);

  arma::mat wc = w - wmean*arma::ones(1, N);
  arma::mat bc = b - bmean*arma::ones(1, N);

  arma::mat H = arma::zeros(3, 3);

  for (unsigned int i = 0; i < N; i++)
    H += bc.col(i)*arma::trans(wc.col(i));

  arma::mat U, V;
  arma::colvec s;

  arma::svd(U, s, V, H);

  R = V*arma::trans(U);
  T = wmean - R*bmean;

  return;
}

bool calibrating = false;
bool vsk_available = false;

vector<float> vsk_values;
vector<float> reference_values;

ros::Publisher pose_pub;

arma::running_stat_vec<arma::colvec::elem_type> Tstats;
arma::running_stat<double> qxstats;
arma::running_stat<double> qystats;
arma::running_stat<double> qzstats;
arma::running_stat<double> qwstats;

void values_callback(const vicon::Values::ConstPtr &msg)
{
  if (!calibrating)
    return;

  if ((!reference_available) || (!vsk_available))
    return;

  if (!names_set)
    return;

  arma::mat wRbv(3, 3);
  arma::colvec wTbv(3);

  compute_transformation(msg->values, vsk_values,
                         vsk_index, wRbv, wTbv);

  arma::mat wRb(3, 3);
  arma::colvec wTb(3);

  compute_transformation(msg->values, reference_values,
                         reference_index, wRb, wTb);

  arma::mat bRbv = arma::trans(wRb)*wRbv;
  arma::colvec bTbv = arma::trans(wRbv)*(wTbv - wTb);

  Tstats(bTbv);
  arma::colvec mean = Tstats.mean();

  pose_msg.position.x = mean(0)/1e3;
  pose_msg.position.y = mean(1)/1e3;
  pose_msg.position.z = mean(2)/1e3;

  btMatrix3x3 rot(bRbv(0, 0), bRbv(0, 1), bRbv(0, 2),
                  bRbv(1, 0), bRbv(1, 1), bRbv(1, 2),
                  bRbv(2, 0), bRbv(2, 1), bRbv(2, 2));

  btQuaternion quat;
  rot.getRotation(quat);
  quat.normalize();

  qxstats(quat.x());
  qystats(quat.y());
  qzstats(quat.z());
  qwstats(quat.w());

  btQuaternion mean_quat(qxstats.mean(),
                         qystats.mean(),
                         qzstats.mean(),
                         qwstats.mean());
  mean_quat.normalize();

  pose_msg.orientation.x = mean_quat.x();
  pose_msg.orientation.y = mean_quat.y();
  pose_msg.orientation.z = mean_quat.z();
  pose_msg.orientation.w = mean_quat.w();

  pose_pub.publish(pose_msg);

  return;
}

string calibration_file;

int WriteCalibrationFile()
{
  ofstream out(calibration_file.c_str(), ios_base::trunc);

  out << "<calibration>" << endl;
  out << "\t<position>" << endl;
  out << "\t\t<x>" << pose_msg.position.x << "</x>" << endl;
  out << "\t\t<y>" << pose_msg.position.y << "</y>" << endl;
  out << "\t\t<z>" << pose_msg.position.z << "</z>" << endl;
  out << "\t</position>" << endl;
  out << "\t<orientation>" << endl;
  out << "\t\t<x>" << pose_msg.orientation.x << "</x>" << endl;
  out << "\t\t<y>" << pose_msg.orientation.y << "</y>" << endl;
  out << "\t\t<z>" << pose_msg.orientation.z << "</z>" << endl;
  out << "\t\t<w>" << pose_msg.orientation.w << "</w>" << endl;
  out << "\t</orientation>" << endl;
  out << "</calibration>" << endl;

  return 0;
}

bool toggle_calibration(vicon_calibrate::Calibrate::Request& req,
                        vicon_calibrate::Calibrate::Response& res)
{
  if ((!reference_available) || (!vsk_available))
    return false;

  if (calibrating)
    {
      calibrating = false;
      if (WriteCalibrationFile() != 0)
        return false;
    }
  else
    calibrating = true;

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vicon_calibrate");
  ros::NodeHandle n("~");

  pose_pub = n.advertise<geometry_msgs::Pose>("pose", 100, true);

  if (!n.getParam("calibration", calibration_file))
    {
      ROS_FATAL("%s: Calibration file must be defined",
                ros::this_node::getName().c_str());
      return -1;
    }

  if (ReadCalibration(calibration_file) != 0)
    {
      ROS_FATAL("%s: Failed to read calibration file: %s",
                ros::this_node::getName().c_str(),
                calibration_file.c_str());
      return -1;
    }
  else
    pose_pub.publish(pose_msg);

  string vsk_file;
  if (n.getParam("vsk", vsk_file))
    {
      if (ParseVSK(vsk_file, vsk_names, vsk_values) != 0)
        {
          ROS_ERROR("%s: Failed to read vsk file: %s",
                    ros::this_node::getName().c_str(),
                    vsk_file.c_str());
        }
      else
        vsk_available = true;
    }

  string reference;
  if (n.getParam("reference", reference))
    {
      if (ReadReference(reference, reference_names,
                        reference_values) != 0)
        {
          ROS_ERROR("%s: Failed to read reference file: %s",
                    ros::this_node::getName().c_str(),
                    reference.c_str());
        }
      else
        reference_available = true;
    }

  ros::Subscriber values_sub;
  ros::Subscriber names_sub;
  ros::ServiceServer service;

  if (vsk_available && reference_available)
    {
      values_sub = n.subscribe("values", 10, values_callback);
      names_sub = n.subscribe("names", 1, names_callback);

      service = n.advertiseService("toggle_calibration", toggle_calibration);
    }

  ros::spin();

  return 0;
}

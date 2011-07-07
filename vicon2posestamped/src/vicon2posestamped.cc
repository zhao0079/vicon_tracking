#include <string>
#include <vector>
#include <numeric>
#include <armadillo>
#include <ros/ros.h>
#include <tf/tf.h>

#include <vicon/Names.h>
#include <vicon/Values.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <XMLConfig.h>

using namespace std;

arma::mat bRbv(3, 3);
arma::colvec bTbv(3);
bool calib_set = false;

void calib_callback(const geometry_msgs::Pose::ConstPtr &msg)
{
  bTbv(0) = 1e3*msg->position.x;
  bTbv(1) = 1e3*msg->position.y;
  bTbv(2) = 1e3*msg->position.z;

  btQuaternion quat;
  tf::quaternionMsgToTF(msg->orientation, quat);
  quat.normalize();

  btMatrix3x3 rot(quat);

  for (int i = 0; i < 3; i++)
    {
      bRbv(i, 0) = rot.getRow(i).x();
      bRbv(i, 1) = rot.getRow(i).y();
      bRbv(i, 2) = rot.getRow(i).z();
    }

  calib_set = true;

  return;
}

bool names_set = false;
vector<string> vsk_names;
vector<unsigned int> names_index;
double vicon_dt = 0.01;

void names_callback(const vicon::Names::ConstPtr &msg)
{
  if (names_set)
    return;

  string fps = msg->names[0];
  float nfps;
  sscanf(fps.c_str(),"%*s %f %*s %*s", &nfps);
  vicon_dt = 1.0/nfps;

  for (vector<string>::iterator i = vsk_names.begin();
       i != vsk_names.end(); ++i)
    for (unsigned int j = 0; j < msg->names.size(); j++) {
      string match_name = *i + string(" ");
      if (msg->names[j].find(match_name) != string::npos)
        names_index.push_back(j);
    }

  if (4*vsk_names.size() != names_index.size())
    {
      ROS_FATAL("Failed to extract names in vsk file from data");
      ros::shutdown();
    }

  names_set = true;

  return;
}

int compute_transformation(vector<double> markers,
                           vector<float> reference,
                           vector<unsigned int> index,
                           arma::mat &R, arma::colvec &T)
{
  vector<unsigned int> visible;

  for (unsigned int i = 0; i < index.size()/4; i++)
    if (markers[index[4*i + 3]] < 1e-6)
      visible.push_back(i);

  unsigned int N = visible.size();

  if (N == 0)
    return -1;

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

  return 0;
}

ros::Publisher pose_pub;
geometry_msgs::PoseStamped pose_msg;

btQuaternion quat;
vector<float> vsk_values;

arma::mat wRbv(3, 3);
arma::mat wRb(3, 3);

arma::colvec wTbv(3);
arma::colvec wTb(3);

void values_callback(const vicon::Values::ConstPtr &msg)
{
  if ((!names_set) || (!calib_set))
    return;

  if (compute_transformation(msg->values, vsk_values,
                             names_index, wRbv, wTbv) != 0)
    {
      return;
    }

  wRb = wRbv*arma::trans(bRbv);
  wTb = wTbv - wRbv*bTbv;

  pose_msg.pose.position.x = wTb(0)/1e3;
  pose_msg.pose.position.y = wTb(1)/1e3;
  pose_msg.pose.position.z = wTb(2)/1e3;

  btMatrix3x3 rot(wRb(0, 0), wRb(0, 1), wRb(0, 2),
                  wRb(1, 0), wRb(1, 1), wRb(1, 2),
                  wRb(2, 0), wRb(2, 1), wRb(2, 2));
  rot.getRotation(quat);
  quat.normalize();

  tf::quaternionTFToMsg(quat, pose_msg.pose.orientation);

  pose_msg.header.stamp = msg->header.stamp;
  pose_pub.publish(pose_msg);

  return;
}

int ParseVSK(string &file, string &model,
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

  int i = 0;
  bool model_set = false;
  ROS_DEBUG("%s: Finding markers", ros::this_node::getName().c_str());
  while (true)
    {
      XMLConfig *c =
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
              c->GetAttributeString("SEGMENT", model);
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
          names.push_back(model + string(":") + name);

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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vicon2posestamped");
  ros::NodeHandle n("~");

  pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 100);

  ros::Subscriber values_sub = n.subscribe("values", 1, values_callback);
  ros::Subscriber names_sub = n.subscribe("names", 1, names_callback);
  ros::Subscriber calibration_sub = n.subscribe("calibration", 1, calib_callback);

  string vsk_file;
  n.param("vsk", vsk_file, string("file.vsk"));

  string model;
  if (ParseVSK(vsk_file, model, vsk_names, vsk_values) != 0)
    {
      ROS_FATAL("%s: Failed to parse vsk file: %s",
                ros::this_node::getName().c_str(),
                vsk_file.c_str());
      return -1;
    }

  n.param("frame_id", pose_msg.header.frame_id, string("vicon"));

  ros::spin();

  return 0;
}

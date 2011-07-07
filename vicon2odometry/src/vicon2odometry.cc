#include <string>
#include <vector>
#include <numeric>
#include <armadillo>
#include <ros/ros.h>
#include <tf/tf.h>
#include <boost/tuple/tuple.hpp>

#include <vicon/Names.h>
#include <vicon/Values.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
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

ros::Publisher odometry_pub;
nav_msgs::Odometry odom_msg;

double told;
bool deriv_init = false;
btQuaternion quat;
vector<float> vsk_values;

arma::mat wRbv(3, 3);
arma::mat wRb(3, 3);
arma::mat wRb_prior(3, 3);
arma::mat dR(3, 3);
arma::mat dRvee(3, 3);

arma::colvec wTbv(3);
arma::colvec wTb(3);
arma::colvec wTb_prior(3);
arma::colvec dT(3);

int history = 5;

typedef boost::tuple<double, double, double> tuple_t;

std::list< tuple_t > pos_hist;
std::list< tuple_t >::iterator piter;

std::list< btQuaternion > quat_hist;
std::list< btQuaternion >::iterator qiter;

ros::Time tprior;
double t_threshold = 3;

void values_callback(const vicon::Values::ConstPtr &msg)
{
  if ((!names_set) || (!calib_set))
    return;

  if (compute_transformation(msg->values, vsk_values,
                             names_index, wRbv, wTbv) != 0)
    {
      ROS_WARN("%s: failed to compute transformation",
               ros::this_node::getName().c_str());
      return;
    }

  wRb = wRbv*arma::trans(bRbv);
  wTb = wTbv - wRbv*bTbv;

  // Update the pose as a moving weighted average
  // 1. Perform a moving weighted average on the latest n history measurements
  // 2. Perform a mean average with the current measurement.
  // The goal is to have a smoothing effect with minimal lag.

  double x = wTb(0)/1e3;
  double y = wTb(1)/1e3;
  double z = wTb(2)/1e3;

  ros::Time tnow(msg->header.stamp);

  if (deriv_init && ((tnow - tprior).toSec() > t_threshold*vicon_dt))
    {
      pos_hist.clear();
      quat_hist.clear();
      deriv_init = false;
    }

  if (pos_hist.size() > (unsigned int)history)
    pos_hist.pop_front();

  if (quat_hist.size() > (unsigned int)history)
    quat_hist.pop_front();

  if (pos_hist.size() == 0)
    {
      odom_msg.pose.pose.position.x = x;
      odom_msg.pose.pose.position.y = y;
      odom_msg.pose.pose.position.z = z;
    }
  else
    {
      std::vector<double> sums(3, 0.0);
      double j = 1.0, denom = 0.0;

      for (piter = pos_hist.begin();
           piter != pos_hist.end(); ++piter, j += 1.0)
        {
          sums[0] += ((*piter).get<0>())*j;
          sums[1] += ((*piter).get<1>())*j;
          sums[2] += ((*piter).get<2>())*j;

          denom += j;
        }

      odom_msg.pose.pose.position.x = 0.5*(sums[0]/denom + x);
      odom_msg.pose.pose.position.y = 0.5*(sums[1]/denom + y);
      odom_msg.pose.pose.position.z = 0.5*(sums[2]/denom + z);
    }

  // Get the latest orientation observation
  btMatrix3x3 rot(wRb(0, 0), wRb(0, 1), wRb(0, 2),
                  wRb(1, 0), wRb(1, 1), wRb(1, 2),
                  wRb(2, 0), wRb(2, 1), wRb(2, 2));
  rot.getRotation(quat);
  quat.normalize();

  if (quat_hist.size() == 0)
    {
      tf::quaternionTFToMsg(quat, odom_msg.pose.pose.orientation);
    }
  else
    {
      double j = 1.0;
      double mult = 0.0;
      arma::mat::fixed<4, 4> qavg;
      qavg.zeros();

      // Weighted quaternion averaging
      // Source:
      // F. L. Markley et al, Quaternion Averaging, 2007
      for (qiter = quat_hist.begin();
           qiter != quat_hist.end(); ++qiter)
        {
          arma::rowvec::fixed<4> qi;
          qi(0) = (*qiter).x();
          qi(1) = (*qiter).y();
          qi(2) = (*qiter).z();
          qi(3) = (*qiter).w();

          qavg += j*arma::trans(qi)*qi;
          j += 1.0;
          mult += j;
        }

      arma::rowvec::fixed<4> qlast;
      qlast(0) = quat.x();
      qlast(1) = quat.y();
      qlast(2) = quat.z();
      qlast(3) = quat.w();

      qavg += mult*arma::trans(qlast)*qlast;

      arma::colvec eigval;
      arma::mat eigvec;
      arma::eig_sym(eigval, eigvec, qavg);

      unsigned int index;
      eigval.max(index);
      // The average quaternion is the eigenvector associated with
      // the largest eigenvalue
      btQuaternion qout(eigvec(0, index),
                        eigvec(1, index),
                        eigvec(2, index),
                        eigvec(3, index));
      qout.normalize();

      tf::quaternionTFToMsg(qout, odom_msg.pose.pose.orientation);
    }

  // Whatever happens above, the current rotation is held in the msg
  btQuaternion q;
  tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, q);
  btMatrix3x3 Ravg(q);

  arma::mat::fixed<3, 3> R;
  for (unsigned int i = 0; i < 3; i++)
    {
      R(i, 0) = Ravg.getRow(i).x();
      R(i, 1) = Ravg.getRow(i).y();
      R(i, 2) = Ravg.getRow(i).z();
    }

  odom_msg.header.stamp = tnow;

  if (deriv_init)
    {
      double dt = (tnow - tprior).toSec();

      odom_msg.twist.twist.linear.x =
        (odom_msg.pose.pose.position.x - pos_hist.back().get<0>())/dt;
      odom_msg.twist.twist.linear.y =
        (odom_msg.pose.pose.position.y - pos_hist.back().get<1>())/dt;
      odom_msg.twist.twist.linear.z =
        (odom_msg.pose.pose.position.z - pos_hist.back().get<2>())/dt;

      dR = (R - wRb_prior)/dt;
      dRvee = -arma::trans(R)*dR;

      odom_msg.twist.twist.angular.x = dRvee(2, 1);
      odom_msg.twist.twist.angular.y = dRvee(0, 2);
      odom_msg.twist.twist.angular.z = dRvee(1, 0);
    }

  tprior = ros::Time(msg->header.stamp);

  pos_hist.push_back(boost::make_tuple(odom_msg.pose.pose.position.x,
                                       odom_msg.pose.pose.position.y,
                                       odom_msg.pose.pose.position.z));
  quat_hist.push_back(q);

  wRb_prior = R;

  deriv_init = true;

  odometry_pub.publish(odom_msg);

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
  ros::init(argc, argv, "vicon2odometry");
  ros::NodeHandle n("~");

  odometry_pub = n.advertise<nav_msgs::Odometry>("odom", 100);

  ros::Subscriber values_sub = n.subscribe("values", 1, values_callback);
  ros::Subscriber names_sub = n.subscribe("names", 1, names_callback);
  ros::Subscriber calibration_sub = n.subscribe("pose", 1, calib_callback);

  string vsk_file;
  n.param("vsk", vsk_file, string("file.vsk"));

  n.param("history", history, 5);

  string model;
  if (ParseVSK(vsk_file, model, vsk_names, vsk_values) != 0)
    {
      ROS_FATAL("%s: Failed to parse vsk file: %s",
                ros::this_node::getName().c_str(),
                vsk_file.c_str());
      return -1;
    }

  n.param("child_frame_id", odom_msg.child_frame_id, string(""));

  if(odom_msg.child_frame_id == "")
    odom_msg.child_frame_id = model;

  n.param("frame_id", odom_msg.header.frame_id, string("vicon"));
  n.param("threshold", t_threshold, 3.0);

  ros::spin();

  return 0;
}

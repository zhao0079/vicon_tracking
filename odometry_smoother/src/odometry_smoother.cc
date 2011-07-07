// Smoothes odometry input, used with vicon input

// N. Michael, UPenn

#include <deque>
#include <numeric>
#include <cmath>
#include <ros/ros.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <armadillo>

using namespace arma;

void QuatToRPY(const geometry_msgs::Quaternion &quat,
               double &r, double &p, double &y)
{
  double qx = quat.x;
  double qy = quat.y;
  double qz = quat.z;
  double qw = quat.w;

  double R12 = 2.0*(qx*qy - qw*qz);
  double R22 = qw*qw - qx*qx + qy*qy - qz*qz;
  double R31 = 2.0*(qx*qz - qw*qy);
  double R32 = 2.0*(qy*qz + qw*qx);
  double R33 = qw*qw - qx*qx - qy*qy + qz*qz;

  r = asin(R32);
  double cr = cos(r);

  if (fabs(cr) > 1e-6)
    {
      p = atan2(-R31/cr, R33/cr);
      y = atan2(-R12/cr, R22/cr);
    }
  else
    {
      p = 0;
      y = 0;
    }

  r = angles::normalize_angle(r);
  p = angles::normalize_angle(p);
  y = angles::normalize_angle(y);

  return;
}

void RPYToQuat(double r, double p, double y,
               geometry_msgs::Quaternion &quat)
{
  // Rot RZ(y)*RX(r)*RY(p)
  btMatrix3x3 rot(cos(p)*cos(y) - sin(p)*sin(r)*sin(y),
                  -(cos(r)*sin(y)),
                  cos(y)*sin(p) + cos(p)*sin(r)*sin(y),
                  cos(y)*sin(p)*sin(r) + cos(p)*sin(y),
                  cos(r)*cos(y),
                  -(cos(p)*cos(y)*sin(r)) + sin(p)*sin(y),
                  -(cos(r)*sin(p)),
                  sin(r),
                  cos(p)*cos(r));
  btQuaternion q;
  rot.getRotation(q);
  q.normalize();
  tf::quaternionTFToMsg(q, quat);
}

vec Q(6);
vec R(6);
bool initialized = false;
ros::Publisher pub;
nav_msgs::Odometry est;
double told;

std::deque<double> vx;
std::deque<double> vy;
std::deque<double> vz;
std::deque<double> wx;
std::deque<double> wy;
std::deque<double> wz;

void handle_odom(const nav_msgs::Odometry::ConstPtr &msg)
{
  if (!initialized)
    {
      est = *msg;

      for (unsigned int i = 0; i < 36; i++)
        {
          est.pose.covariance[i] = 0;
          est.twist.covariance[i] = 0;
        }

      for (unsigned int i = 0; i < 6; i++)
        est.pose.covariance[i*6 + i] = 1000;
      for (unsigned int i = 0; i < 6; i++)
        est.twist.covariance[i*6 + i] = 1000;

      initialized = true;
      told = msg->header.stamp.toSec();

      return;
    }

  // A couple comments:
  // 1. This is approach is flawed. The vicon is providing
  //    point clouds and the pose and velocity information is based
  //    on that point cloud and how it is changing.
  // 2. Using an average velocity on the data will
  //    a. Smooth noise (which is what I want)
  //    b. Introduce latency
  // 3. The process and measurement are coupled (and not i.i.d., see 1).
  //    Keep this fact in mind when using this. In other words, this will
  //    smooth the estimate but at a cost and is pretty much a hack!
  // 4. This is generic and only looks at Vicon. If you can access local
  //    process data, do it. Don't use this.

  // Prior
  vec state(6);
  state(0) = est.pose.pose.position.x;
  state(1) = est.pose.pose.position.y;
  state(2) = est.pose.pose.position.z;
  QuatToRPY(est.pose.pose.orientation,
            state(3), state(4), state(5));
  //state.print("state = ");

  vx.push_back(msg->twist.twist.linear.x);
  vy.push_back(msg->twist.twist.linear.y);
  vz.push_back(msg->twist.twist.linear.z);
  wx.push_back(msg->twist.twist.angular.x);
  wy.push_back(msg->twist.twist.angular.y);
  wz.push_back(msg->twist.twist.angular.z);

  if (vx.size() > 5) vx.pop_front();
  if (vy.size() > 5) vy.pop_front();
  if (vz.size() > 5) vz.pop_front();
  if (wx.size() > 5) wx.pop_front();
  if (wy.size() > 5) wy.pop_front();
  if (wz.size() > 5) wz.pop_front();

  double v_x = std::accumulate(vx.begin(), vx.end(), 0.0)/vx.size();
  double v_y = std::accumulate(vy.begin(), vy.end(), 0.0)/vy.size();
  double v_z = std::accumulate(vz.begin(), vz.end(), 0.0)/vz.size();
  double w_x = std::accumulate(wx.begin(), wx.end(), 0.0)/wx.size();
  double w_y = std::accumulate(wy.begin(), wy.end(), 0.0)/wy.size();
  double w_z = std::accumulate(wz.begin(), wz.end(), 0.0)/wz.size();

  vec obs(6);
  obs(0) = v_x;
  obs(1) = v_y;
  obs(2) = v_z;
  obs(3) = w_x;
  obs(4) = w_y;
  obs(5) = w_z;

  double tnow = msg->header.stamp.toSec();
  double dt = tnow - told;
  vec prior = state + dt*obs;
  told = tnow;
  //prior.print("prior = ");

  vec P = zeros(6);
  for (unsigned int i = 0; i < 6; i++)
    P(i) = est.pose.covariance[i*6 + i];
  P += Q;
  //P.print("P = ");

  mat diagP = diagmat(P);

  // Measurement
  vec measurement(6);
  measurement(0) = msg->pose.pose.position.x;
  measurement(1) = msg->pose.pose.position.y;
  measurement(2) = msg->pose.pose.position.z;
  QuatToRPY(msg->pose.pose.orientation,
            measurement(3),
            measurement(4),
            measurement(5));
  //measurement.print("measurement = ");

  vec innovation = measurement - prior;
  // Check angle innovation doesn't exceed great arc
  for (unsigned int i = 3; i < 6; i++)
    innovation(i) = angles::shortest_angular_distance(prior(i),
                                                      measurement(i));
  //innovation.print("innovation = ");

  // Update
  mat K = diagmat(P/(P + R));
  //K.print("gain = ");
  vec posterior = prior + K*innovation;
  //posterior.print("posterior = ");
  P = diagvec((eye(6, 6) - K)*diagP);
  //P.print("residual = ");

  // Posterior (store in est msg)
  est.pose.pose.position.x = posterior(0);
  est.pose.pose.position.y = posterior(1);
  est.pose.pose.position.z = posterior(2);
  RPYToQuat(posterior(3), posterior(4), posterior(5),
            est.pose.pose.orientation);

  est.twist.twist.linear.x = obs(0);
  est.twist.twist.linear.y = obs(1);
  est.twist.twist.linear.z = obs(2);
  est.twist.twist.angular.x = obs(3);
  est.twist.twist.angular.y = obs(4);
  est.twist.twist.angular.z = obs(5);

  for (unsigned int i = 0; i < 6; i++)
    est.pose.covariance[i*6 + i] = P(i);

  est.header.stamp = ros::Time(tnow);

  pub.publish(est);

  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_smoother");
  ros::NodeHandle n("~");

  ros::Subscriber sub = n.subscribe("odom_in", 100, handle_odom);
  pub = n.advertise<nav_msgs::Odometry>("odom_out", 1);

  R.zeros();
  Q.zeros();
  n.param("noise/x", R(0), 0.1);
  n.param("noise/y", R(1), 0.1);
  n.param("noise/z", R(2), 0.1);
  n.param("noise/r", R(3), 0.1);
  n.param("noise/p", R(4), 0.1);
  n.param("noise/y", R(5), 0.1);
  n.param("noise/vx", Q(0), 0.1);
  n.param("noise/vy", Q(1), 0.1);
  n.param("noise/vz", Q(2), 0.1);
  n.param("noise/wx", Q(3), 0.1);
  n.param("noise/wy", Q(4), 0.1);
  n.param("noise/wz", Q(5), 0.1);

  ros::spin();

  return 0;
}

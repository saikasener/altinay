#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>
#include <boost/array.hpp>

/*
  Calculates odometry and publishes.
*/

double linear_speed, angular_speed;
double pose_x, pose_y, theta;

boost::array<double, 36> ODOM_POSE_COVARIANCE = {1e-3, 0, 0, 0, 0, 0,
                                                 0, 1e-3, 0, 0, 0, 0,
                                                 0, 0, 1e6, 0, 0, 0,
                                                 0, 0, 0, 1e6, 0, 0,
                                                 0, 0, 0, 0, 1e6, 0,
                                                 0, 0, 0, 0, 0, 1e3};

boost::array<double, 36> ODOM_TWIST_COVARIANCE = {1e-3, 0, 0, 0, 0, 0,
                                                  0, 1e-3, 0, 0, 0, 0,
                                                  0, 0, 1e6, 0, 0, 0,
                                                  0, 0, 0, 1e6, 0, 0,
                                                  0, 0, 0, 0, 1e6, 0,
                                                  0, 0, 0, 0, 0, 1e3};

void getVelocity(const geometry_msgs::Twist &vel_msg)
{
  linear_speed = vel_msg.linear.x;
  angular_speed = vel_msg.angular.z;
}

void getInitialPose(const geometry_msgs::PoseWithCovarianceStamped &initial_pose)
{
  pose_x = initial_pose.pose.pose.position.x;
  pose_y = initial_pose.pose.pose.position.y;
  theta = initial_pose.pose.pose.orientation.z;
}

ros::Time t_old, t_new;
nav_msgs::Odometry odom_msg;

void calOdometry()
{
  t_new = ros::Time::now();
  double delta_t = (t_new - t_old).toSec();
  pose_x = pose_x + linear_speed * delta_t * cos(theta);
  pose_y = pose_y + linear_speed * delta_t * sin(theta);
  theta = theta + angular_speed * delta_t;

  // since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

  //publish the odometry message over ROS
  odom_msg.header.stamp = t_new;
  odom_msg.header.frame_id = "odom";

  // set the position
  odom_msg.pose.pose.position.x = pose_x;
  odom_msg.pose.pose.position.y = pose_y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  // set the velocity
  odom_msg.child_frame_id = "base_footprint";
  odom_msg.twist.twist.linear.x = linear_speed;
  odom_msg.twist.twist.angular.z = angular_speed;

  // set covariances
  odom_msg.pose.covariance = ODOM_POSE_COVARIANCE;

  odom_msg.twist.covariance = ODOM_TWIST_COVARIANCE;
  t_old = t_new;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moobot_odometry");
  pose_x = pose_y = theta = 0.0;

  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 500);
  ros::Subscriber vel_sub = nh.subscribe("/cmd_vel_act", 500, getVelocity);
  ros::Subscriber initalpose_sub = nh.subscribe("/initial_pose", 500, getInitialPose);

  ros::Rate loop_rate(50);

  t_old = ros::Time::now();
  t_new = ros::Time::now();

  while (ros::ok())
  {

    calOdometry();
    odom_pub.publish(odom_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

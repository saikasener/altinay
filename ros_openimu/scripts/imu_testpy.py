#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler
from math import cos, sin
from sensor_msgs.msg import Imu, MagneticField

import pandas as pd
import numpy as np 
import scipy.integrate 

linear_speed = 0.0
angular_speed = 0.0
pose_x = 0.0
pose_y = 0.0
theta = 0.0

imu_linear_acceleration_x= 0.0
imu_linear_acceleration_y=0.0
imu_linear_acceleration_z= 0.0
imu_angular_velocity_x=  0.0
imu_angular_velocity_y= 0.0
imu_angular_velocity_z=0.0


def get_imu(imu_msg):
    global imu_linear_acceleration_x,imu_linear_acceleration_y,imu_linear_acceleration_z,imu_angular_velocity_x,imu_angular_velocity_y,imu_angular_velocity_z
    imu_linear_acceleration_x= imu_msg.linear_acceleration.x 
    imu_linear_acceleration_y=imu_msg.linear_acceleration.y 
    imu_linear_acceleration_z= imu_msg.linear_acceleration.z
    imu_angular_velocity_x= imu_msg.angular_velocity.x 
    imu_angular_velocity_y= imu_msg.angular_velocity.y 
    imu_angular_velocity_z= imu_msg.angular_velocity.z 


imu_msg = Imu()           
mag_msg = MagneticField() 

t_old = rospy.Time.now()
t_new = rospy.Time.now()

def get_acc_data():
    global pose_x, pose_y, theta, odom_msg, t_old, t_new

    t_new = rospy.Time.now()
    delta_t = (t_new - t_old).to_sec()
    pose_x = pose_x + linear_speed * delta_t * cos(theta)
    pose_y = pose_y + linear_speed * delta_t * sin(theta)
    theta = theta + angular_speed * delta_t

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = Quaternion(*quaternion_from_euler(0, 0, theta))

    # publish the odometry message over ROS
    odom_msg.header.stamp = t_new
    odom_msg.header.frame_id = "odom"

    # set the position
    odom_msg.pose.pose.position.x = pose_x
    odom_msg.pose.pose.position.y = pose_y
    odom_msg.pose.pose.position.z = 0.0
    odom_msg.pose.pose.orientation = odom_quat

    # set the velocity
    odom_msg.child_frame_id = "base_footprint"
    odom_msg.twist.twist.linear.x = linear_speed
    odom_msg.twist.twist.angular.z = angular_speed

    # # set covariances
    # odom_msg.pose.covariance```python
    # odom_msg.pose.covariance = ODOM_POSE_COVARIANCE
    # odom_msg.twist.covariance = ODOM_TWIST_COVARIANCE
    t_old = t_new

def get_velocity(vel_msg):
    linear_speed=vel_msg.linear.x
    angular_speed = vel_msg.angular.z

def main():
    global pose_x, pose_y, theta, odom_msg, t_old, t_new

    rospy.init_node("imu_test")
    pose_x = pose_y = theta = 0.0

    rospy.Subscriber("/imu_acc_ar", Imu, get_imu)
    rospy.Subscriber("/cmd_vel", Twist, get_velocity)

    imu_pub = rospy.Publisher("/imu_test", Odometry, queue_size=500)

    loop_rate = rospy.Rate(50)

    t_old = rospy.Time.now()
    t_new = rospy.Time.now()

    while not rospy.is_shutdown():
        get_acc_data()
        # imu_pub.publish(odom_msg)
        loop_rate.sleep()

if __name__ == "__main__":
    main()

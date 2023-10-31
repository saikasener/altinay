#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
import math
import numpy as np
import signal
import sys

baselink_pose_x=0
baselink_pose_y=0
baselink_pose_z=0
qr_pose_x=0
qr_pose_y=0
qr_pose_z=0
qr_angle_y=0

def euler_from_quaternion(x, y, z, w):
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    y_deg = np.degrees(pitch_y)

    return y_deg

def poseCallback(msg):
    global qr_pose_x,qr_pose_y,qr_pose_z,qr_angle_y
    qr_pose_x= msg.pose.position.x
    qr_pose_y= msg.pose.position.y
    qr_pose_z= msg.pose.position.z
    qr_angle_y = euler_from_quaternion(msg.pose.orientation.x, msg.pose.orientation.y,
                                       msg.pose.orientation.z, msg.pose.orientation.w)

def poseBaseCallback(msg):
    global baselink_pose_x,baselink_pose_y,baselink_pose_z
    baselink_pose_x= msg.pose.pose.position.x
    baselink_pose_y= msg.pose.pose.position.y
    baselink_pose_z= msg.pose.pose.position.z

def set_robot_pose():
    global baselink_pose_x,baselink_pose_y,baselink_pose_z, qr_pose_x,qr_pose_y,qr_pose_z,qr_angle_y
    if (abs(qr_pose_z)) > 0.4:
        cmd_vel_msg.angular.z = 0.0
        cmd_vel_msg.linear.x = 0.2
        pubTwist_.publish(cmd_vel_msg)
    # while (abs(qr_angle_y)>15):
    #     cmd_vel_msg.angular.z = 0.2
    #     cmd_vel_msg.angular.y = 0.2
    #     cmd_vel_msg.angular.x = 0.2
    #     pubTwist_.publish(cmd_vel_msg)
    elif (abs(qr_pose_z)) <= 0.4:
        cmd_vel_msg.angular.z = 0.0
        cmd_vel_msg.linear.x = 0.0
        pubTwist_.publish(cmd_vel_msg)  
       

def signal_handler(sig, frame):
    global is_shutdown
    is_shutdown = True
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    
    rospy.init_node('qr_code')
    pubTwist_ = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    subPose_ = rospy.Subscriber("/visp_auto_tracker/object_position", PoseStamped, poseCallback)
    # subPoseAmcl = rospy.Subscriber("/amcl/pose", PoseWithCovarianceStamped, poseBaseCallback)

    cmd_vel_msg = Twist()

    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        set_robot_pose()
        rate.sleep()

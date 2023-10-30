#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
baselink_pose_x=0
baselink_pose_y=0
baselink_pose_z=0
qr_pose_x=0
qr_pose_y=0
qr_pose_z=0
def poseCallback(msg):
    global qr_pose_x,qr_pose_y,qr_pose_z
    qr_pose_x= msg.pose.pose.position.x
    qr_pose_y= msg.pose.pose.position.y
    qr_pose_z= msg.pose.pose.position.z

def poseBaseCallback(msg):
    global baselink_pose_x,baselink_pose_y,baselink_pose_z
    baselink_pose_x= msg.pose.pose.position.x
    baselink_pose_y= msg.pose.pose.position.y
    baselink_pose_z= msg.pose.pose.position.z

def set_robot_pose():
    global baselink_pose_x,baselink_pose_y,baselink_pose_z, qr_pose_x,qr_pose_y,qr_pose_z
    while (abs(baselink_pose_x - qr_pose_z)) > 0.2: 
        cmd_vel_msg.angular.z = 0.0
        cmd_vel_msg.linear.x = 0.4
        pubTwist_.publish(cmd_vel_msg)
    cmd_vel_msg.angular.z = 0.0
    cmd_vel_msg.linear.x = 0.0
    pubTwist_.publish(cmd_vel_msg)  



if __name__ == '__main__':
    
    rospy.init_node('qr_code')
    pubTwist_ = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)
    subPose_ = rospy.Subscriber("/visp_auto_tracker/object_position", PoseStamped, poseCallback)
    subPoseAmcl = rospy.Subscriber("/amcl/pose", PoseWithCovarianceStamped, poseBaseCallback)

    cmd_vel_msg = Twist()

    rate = rospy.Rate(150) 
    while not rospy.is_shutdown():
        set_robot_pose()
        rate.sleep()
   
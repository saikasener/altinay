#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from moobot_msgs.msg import moobot_status
import math
import numpy as np
import signal
import sys
import time

baselink_pose_x=0
baselink_pose_y=0
baselink_pose_z=0
qr_pose_x=0
qr_pose_y=0
qr_pose_z=0
qr_angle_y=0

IDLE=0
EMG=100
FAULT = 101
current_pos = IDLE
old_pos = IDLE
goal_reached = False
emg_time =0
agv_start_time = time.time()
total_emg_time=0
temp_time=0 

def check_status(moobot_status):
    global current_pos
    global old_pos

    if moobot_status.emergency == True:
        current_pos = EMG

    else:
        if current_pos == EMG or current_pos == FAULT:  # emg den çıktıysa eski pos a dönmesi için
            current_pos = old_pos
        

def set_robot_pose():
    global current_pos,goal_reached, emg_time, total_emg_time, temp_time
    while current_pos == IDLE and current_pos != EMG:
        anlik_time= time.time()
        total_emg_time= total_emg_time + emg_time
        temp_time = 0
        emg_time = 0
        if (anlik_time - total_emg_time- agv_start_time ) < 20:
            print("While Time:", anlik_time - total_emg_time- agv_start_time, "tot:",total_emg_time)
            cmd_vel_msg.angular.z = 0.0
            cmd_vel_msg.linear.x = 0.03
            return cmd_vel_msg
            #pubTwist_.publish(cmd_vel_msg)
          
        elif (anlik_time - total_emg_time- agv_start_time ) >= 20:
            print("elif " + str(anlik_time - total_emg_time- agv_start_time ), "tot:",total_emg_time)
            cmd_vel_msg.angular.z = 0.0
            cmd_vel_msg.linear.x = 0.0
            return cmd_vel_msg
            #pubTwist_.publish(cmd_vel_msg)
    if current_pos == EMG:
        if temp_time==0:
            temp_time = time.time()
        emg_time= time.time() - temp_time
        print("emg time:", emg_time)
        cmd_vel_msg.angular.z = 0.0
        cmd_vel_msg.linear.x = 0.0
        return cmd_vel_msg
     
def publisher(cmd_vel_msg):
    pubTwist_.publish(cmd_vel_msg)

def follow_line(goal_x_tolerance, goal_orientation, goal_pos_x):
    global cmd_vel_angular_sp
    global cmd_vel_linear_sp
    global follow_line_reached
    if current_pos != EMG:
        print("Reached the end of the line")
        follow_line_reached = True
        cmd_vel_linear_sp = 0.0
        cmd_vel_angular_sp = 0.0
        cmd_vel_msg.angular.z = cmd_vel_angular_sp
        cmd_vel_msg.linear.x = cmd_vel_linear_sp
        pubTwist_.publish(cmd_vel_msg)
        pubTwist_.publish(cmd_vel_msg)

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




def signal_handler(sig, frame):
    global is_shutdown
    is_shutdown = True
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    
    rospy.init_node('qr_code')
    pubTwist_ = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # subPose_ = rospy.Subscriber("/visp_auto_tracker/object_position", PoseStamped, poseCallback)
    moobot_status_sub_c_plc = rospy.Subscriber("/agv_status", moobot_status, check_status)
    cmd_vel_msg = Twist()

    # agv_start_time = time.time()

    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        publisher(set_robot_pose())
        # if current_pos == EMG:
        #     if time.time() - emg_start_time > 2:
        #         agv_job_status = "Logos is in emergency situtation"
        #         cmd_vel_msg.angular.z = 0.0
        #         cmd_vel_msg.linear.x = 0.0
        #         pubTwist_.publish(cmd_vel_msg)
        #         emg_start_time = time.time()

        rate.sleep()

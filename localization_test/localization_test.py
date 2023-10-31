#!/usr/bin/env python3

# import rospy
# from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
# from nav_msgs.msg import Odometry
# from moobot_msgs.msg import moobot_status
# from std_msgs.msg import Int16
# import math
# import numpy as np
# import signal
# import sys
# import time


# def set_robot_pose():
#     start_time= time.time()
    
#     while time.time() - start_time < 5:
#         cmd_vel_msg.angular.z = 0.0
#         cmd_vel_msg.linear.x = 0.3
#         # return cmd_vel_msg
#         pubTwist_.publish(cmd_vel_msg)
#         time.sleep(0.1)        
    
#     # if (anlik_time - total_emg_time- agv_start_time ) < 20:
#     #     print("While Time:", anlik_time - total_emg_time- agv_start_time, "tot:",total_emg_time)
#     #     cmd_vel_msg.angular.z = 0.0
#     #     cmd_vel_msg.linear.x = 0.03
#     #     return cmd_vel_msg
#     #     #pubTwist_.publish(cmd_vel_msg)
        
#     # elif (anlik_time - total_emg_time- agv_start_time ) >= 20:
#     #     print("elif " + str(anlik_time - total_emg_time- agv_start_time ), "tot:",total_emg_time)
#     #     cmd_vel_msg.angular.z = 0.0
#     #     cmd_vel_msg.linear.x = 0.0
#     #     return cmd_vel_msg
#     #     #pubTwist_.publish(cmd_vel_msg)

     
# # def publisher(cmd_vel_msg):
# #     pubTwist_.publish(cmd_vel_msg)


# def euler_from_quaternion(x, y, z, w):
#     t2 = +2.0 * (w * y - z * x)
#     t2 = +1.0 if t2 > +1.0 else t2
#     t2 = -1.0 if t2 < -1.0 else t2
#     pitch_y = math.asin(t2)
#     angle_degree = np.degrees(pitch_y)
#     return angle_degree

# def poseCallback(odom_msg):
#     global qr_pose_x,qr_pose_y,qr_pose_z,angle_quat
#     # odom_msg.pose.pose.position.x = pose_x;
#     # odom_msg.pose.pose.position.y = pose_y;
#     odom_msg.pose.pose.position.z = 0.0;
#     angle_quat = euler_from_quaternion(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
#                                        odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)


# def poseBaseCallback(msg):
#     global baselink_pose_x,baselink_pose_y,baselink_pose_z
#     baselink_pose_x= msg.pose.pose.position.x
#     baselink_pose_y= msg.pose.pose.position.y
#     baselink_pose_z= msg.pose.pose.position.z


# def signal_handler(sig, frame):
#     global is_shutdown
#     is_shutdown = True
#     sys.exit(0)
# signal.signal(signal.SIGINT, signal_handler)

# if __name__ == '__main__':
    
#     rospy.init_node('localization')
#     pubTwist_ = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
#     subPose_ = rospy.Subscriber("/odometry/filtered", Odometry, poseCallback)
#     cmd_vel_msg = Twist()
#     # agv_start_time = time.time()

#     rate = rospy.Rate(50) 
#     while not rospy.is_shutdown():
#         set_robot_pose()
#         # if current_pos == EMG:
#         #     if time.time() - emg_start_time > 2:
#         #         agv_job_status = "Logos is in emergency situtation"
#         #         cmd_vel_msg.angular.z = 0.0
#         #         cmd_vel_msg.linear.x = 0.0
#         #         pubTwist_.publish(cmd_vel_msg)
#         #         emg_start_time = time.time()

#         rate.sleep()


#=============================================================================================
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, Vector3
from math import atan2

x = 0.0
y = 0.0 
theta = 0.0

def getPose(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    vector_ang.roll
    vector_ang.pitch
    vector_ang.ang
    pub_angle.publish(vector_ang)

rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odometry/filtered", Odometry, getPose)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
pub_angle= rospy.Publisher("/orientation_angle", Vector3 ,queue_size=1)
speed = Twist()
vector_ang= Vector3()
r = rospy.Rate(4)

goal = Point()
goal.x = 5
goal.y = 5

while not rospy.is_shutdown():
    inc_x = goal.x -x
    inc_y = goal.y -y

    angle_to_goal = atan2(inc_y, inc_x)

    if abs(angle_to_goal - theta) > 0.1:
        speed.linear.x = 0.0
        speed.angular.z = 0.3
    else:
        speed.linear.x = 0.5
        speed.angular.z = 0.0

    pub.publish(speed)
    r.sleep() 
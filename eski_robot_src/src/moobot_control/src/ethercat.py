#!/usr/bin/env python3

# ----------------------------------------------------------------
# -------------------- Import Module -----------------------------
# ----------------------------------------------------------------
import serial
import time as t
import serial.tools.list_ports as port_list
import math
import os
import signal
import sys
import numpy as np
import rospy
import time
from moobot_msgs.msg import moobot_status, lift_status
from geometry_msgs.msg import Twist

########  STATES ########
HOME = 1
MOVE = 2
STOP = 3 
LIFT_UP = 4
LIFT_DOWN = 5
LIFT_UNKNOWN = 6
EMG = 100
FAULT = 101


def speeedUpdate():
    print("Speed updated")
    global cmd_vel_linear,cmd_vel_angular
    cmd_vel_linear = 1.5
    cmd_vel_angular = 0.5
    if current_velocity != EMG:
        # while (current_pos_x - goal_pos_x) > goal_x_tolerance:  # hız ver
        #     cmd_vel_linear_sp = (current_pos_x - goal_pos_x) * max_linear_vel / \
        #         reference_x * (1 - abs(current_pos_y)/reference_y)
        #     if current_pos_y > 10.0 or current_pos_y < -10.0:
        #         cmd_vel_angular_sp = current_pos_y * max_angular_vel / reference_y
        #     else:
        #         cmd_vel_angular_sp = current_pos_theta * max_angular_vel / reference_y
        cmd_vel_msg.linear.x = cmd_vel_linear * -1
        cmd_vel_msg.angular.z = cmd_vel_angular
        cmd_vel_act_pub.publish(cmd_vel_msg)

def speedup(self,Twist):
    # rate = rospy.Rate(5) # 200 ms -> 5Hz
    global desired_velocity,current_velocity
    AgvLinearVel= ((Twist.linear.x))
    AgvAngVel= ((Twist.angular.z))
    # AgvLinearVel= 1.2
    # AgvAngVel= 0
    Rdir = 1
    RMR = WheelVelocity(self,Rdir, AgvLinearVel,AgvAngVel) # RightMotorRpm (self,Rdir,LinVel,AngVel)
    # --------------- Actual Velocity Publisher ------
    r = .105   # Wheel Radius m
    b = .4775  # AGV Wheel Offset m
    GearRatio = 24.685 # Const Value
    ##---- Right Actual---- 
    WR_actual= (((GetVelActual_Right(self)/GearRatio)*math.pi/180)*6)  # rad/s actual right 
    VR_actual=WR_actual*r # m/s actual right 
    ##---- Left Actual---- 
    WL_actual= (((GetVelActual_Left(self)/GearRatio)*math.pi/180)*6)  # rad/s actual left 
    VL_actual=WL_actual*r # m/s actual left 
    ##---- AGV Actual---- 
    linear_actual_agv = (VR_actual + VL_actual) / 2
    angular_actual_agv = (VR_actual - VL_actual ) / b
    cmd_vel_msg.linear.x= linear_actual_agv
    cmd_vel_msg.angular.z = angular_actual_agv
    cmd_vel_act_pub.publish(cmd_vel_msg)
    # rate.sleep()
def GetVelActual_Right(self):
    actual_velocity_rpm=3100 
    return actual_velocity_rpm
def GetVelActual_Left(self):
    actual_velocity_rpm=3100 
    return actual_velocity_rpm
def WheelVelocity(self,Rdir,LinVel,AngVel):
    r = .105   # Wheel Radius m
    b = .4775  # AGV Wheel Offset m
    GearRatio = 24.685 # Const Value
    Rmrpm = int(((((LinVel+((AngVel*b)/2))/r)*180/math.pi)/6)*GearRatio) # Right Motor Rpm
    # print(f'----------------- Rmrpm value : {Rmrpm}')
    self.processImage.variables.RightMotor.RxPDO_Mapping_1.Target_velocity.set(Rdir*Rmrpm)
    return Rmrpm
def check_status(moobot_status):
    global current_velocity
    global old_velocity
    if moobot_status.emergency == True:
        current_velocity = EMG
    else:
        if current_velocity == EMG or current_velocity == FAULT:  # ??? emg den çıktıysa başlangıç hız profiline dönmeli
            current_velocity = old_velocity

def runLift(lift_status):
    if lift_status.up_lift == True:
        LiftUp()
    else:
        LiftDown()

def LiftUp ():
    print("lift up")
def LiftDown ():
    print("lift down")


def Main(self):  # Main Function
    global cmd_vel_sub, moobot_status, cmd_vel_act_pub, cmd_vel_msg, moobot_status_sub
    rospy.init_node('ethercat_master')
    rospy.loginfo('Node Begin!')
    cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, speedup)
    moobot_status_sub = rospy.Subscriber("/agv_status", moobot_status, check_status)
    cmd_vel_act_pub = rospy.Publisher("/cmd_vel_act", Twist)
    cmd_vel_msg = Twist()
    while not rospy.is_shutdown():  # While True Until Shutdown Node
        time.sleep(.001)
        rospy.Subscriber("/cmd_vel", Twist, self.speedup)
        rospy.Subscriber("/agv_status", moobot_status, self.check_status)
        rospy.Subscriber("/change_lift_status", lift_status, self.runLift)
        cmd_vel_act_pub = rospy.Publisher("/cmd_vel_act", Twist)
        cmd_vel_msg = Twist()
        rospy.spin()


if __name__ == "__main__":
    Main()
else:
    print("XXXXXX")

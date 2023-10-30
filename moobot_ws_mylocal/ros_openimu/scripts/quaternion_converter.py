#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.integrate import cumtrapz,trapz
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import math

class IMUAccuracyTester:
    def __init__(self):
        rospy.init_node('euler_from_quat')
        self.orientation = Vector3()
        # self.loop_rate = rospy.Rate(50)         
        self.final_position_msg=Vector3()                


        rospy.Subscriber('/imu_data', Imu, self.imu_callback)
        self.orientation_pub = rospy.Publisher('/euler', Vector3, queue_size=10)
        # self.orientation_pub = rospy.Publisher('/imu/orientation', Vector3, queue_size=10)


    def imu_callback(self, data):
        self.x = data.orientation.x
        self.y = data.orientation.y
        self.z = data.orientation.z
        self.w = data.orientation.w
        self.euler_from_quaternion(self.x, self.y, self.z, self.w)

    def euler_from_quaternion(self,x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        roll_x= math.degrees(roll_x)
        pitch_y= math.degrees(pitch_y)
        yaw_z= math.degrees(yaw_z)

        self.final_position_msg.x= roll_x
        self.final_position_msg.y= pitch_y
        self.final_position_msg.z= yaw_z
        self.orientation_pub.publish(self.final_position_msg) 
        # return roll_x, pitch_y, yaw_z # in radians


    def run(self):
        # self.previous_time = rospy.get_time()
        # self.loop_rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    imu_accuracy_tester = IMUAccuracyTester()
    imu_accuracy_tester.run()

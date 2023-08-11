#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.integrate import cumtrapz,trapz
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats


class IMUAccuracyTester:
    def __init__(self):
        rospy.init_node('imu_accuracy_tester')
        self.linear_acceleration = []
        self.angular_velocity = Vector3()
        self.orientation = Vector3()
        self.previous_time = rospy.get_time()
        # self.loop_rate = rospy.Rate(50)
        self.final_position = []              
        self.acceleration=[]
        self.final_position_msg=Vector3()                
        self.sayac=0
        self.velocity =[] 

        rospy.Subscriber('/imu_acc_ar', Imu, self.imu_callback)
        self.position_pub = rospy.Publisher('/imu/position', Vector3, queue_size=10)
        # self.orientation_pub = rospy.Publisher('/imu/orientation', Vector3, queue_size=10)


    def imu_callback(self, data):
              
        self.linear_acceleration.append ([data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z])
        # self.acceleration = np.asarray(self.linear_acceleration) # for 1st Method      
        self.integrate_acc(self.linear_acceleration)
        
        # self.angular_velocity.x = data.angular_velocity.x
        # self.angular_velocity.y = data.angular_velocity.y
        # self.angular_velocity.z = data.angular_velocity.z
        # self.update_orientation(dt)

    def integrate_acc(self,acceleration):
        dt = rospy.get_time() - self.previous_time      #TODO: dt time kontrol edilecek her loop dönme süresi   
# 1st Method: Python function cumtrapz 
        # velocity = cumtrapz(acceleration, axis=0, dx=dt,initial=0)
        # self.final_position = cumtrapz(velocity, axis=0, dx=dt,initial=0)
# 2nd Method: Integral formula for discrete numbers 
        v=0.5*(np.array(acceleration[-1])-np.array(acceleration[-2]))*dt
        self.velocity.append(v) 
        dx=0.5*(np.array(self.velocity[-1])-np.array(self.velocity[-2]))*dt  
        self.final_position.append(dx)
        
        # print(self.final_position[self.sayac])
        ax= self.final_position[self.sayac][0]
        ay= self.final_position[self.sayac][1]
        az= self.final_position[self.sayac][2]
        self.update_position(ax,ay,az)
        self.sayac += 1
    
    def update_position(self,ax,ay,az):
        self.final_position_msg.x= ax
        self.final_position_msg.y= ay
        self.final_position_msg.z= az
        self.position_pub.publish(self.final_position_msg)

    def run(self):
        # self.previous_time = rospy.get_time()
        # self.loop_rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    imu_accuracy_tester = IMUAccuracyTester()
    imu_accuracy_tester.run()

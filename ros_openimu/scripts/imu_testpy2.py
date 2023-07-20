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
        self.position = Vector3()
        self.orientation = Vector3()
        self.previous_time = rospy.get_time()
        self.loop_rate = rospy.Rate(50)
        self.final_position = []              
        self.acceleration=[]
        self.position.x = 0
        self.position.y = 0
        self.position.z = 0
        self.final_position_msg=Vector3()
        
        
        self.sayac=0

        rospy.Subscriber('/imu_acc_ar', Imu, self.imu_callback)
        self.position_pub = rospy.Publisher('/imu/position', Vector3, queue_size=10)
        # self.orientation_pub = rospy.Publisher('/imu/orientation', Vector3, queue_size=10)


    def imu_callback(self, data):
              
        self.linear_acceleration.append ([data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z])
        self.acceleration = np.asarray(self.linear_acceleration)
        self.integrate_acc(self.acceleration)
        
        # self.angular_velocity.x = data.angular_velocity.x
        # self.angular_velocity.y = data.angular_velocity.y
        # self.angular_velocity.z = data.angular_velocity.z
        # self.update_orientation(dt)

    def integrate_acc(self,acceleration):
        dt = rospy.get_time() - self.previous_time        

        velocity = cumtrapz(acceleration, axis=0, dx=dt,initial=0)
        self.final_position = cumtrapz(velocity, axis=0, dx=dt,initial=0)
        
        ax= self.final_position[self.sayac,0]
        ay= self.final_position[self.sayac,1]
        az= self.final_position[self.sayac,2]
        self.update_position(ax,ay,az)
        # if ax > 0.01 and ay< 0.01:
        #     self.update_position(ax,0,0)
        #     print("ax > 0.001 and ay< 0.001")
        # elif ay > 0.001 and ax< 0.001:
        #     self.update_position(0,ay,0)
        # elif ax< 0.001 and ay < 0.001:
        #     self.update_position(0,0,az)
        # elif ax > 0.001 and ay > 0.001:
        #     self.update_position(ax,ay,0)                        
        # else:
        #     print("-"*20)

        self.sayac += 1
        # self.update_position(self.acceleration,dt)

    def position_formula(self):
        #trying to use distance formula  
        dt = rospy.get_time() - self.previous_time
        self.linear_velocity.x += self.linear_acceleration.x * dt
        self.linear_velocity.y += self.linear_acceleration.y * dt
        self.linear_velocity.z += self.linear_acceleration.z * dt

        self.linear_position.x += self.linear_velocity.x * dt 
        self.linear_position.y += self.linear_velocity.y * dt 
        self.linear_position.z += self.linear_velocity.z * dt 
        self.distance_lin_pub.publish(self.linear_position)

        self.angular_position.x += self.angular_velocity.x * dt 
        self.angular_position.y += self.angular_velocity.y * dt 
        self.angular_position.z += self.angular_velocity.z * dt 
        self.previous_time = rospy.get_time()
        self.distance_ang_pub.publish(self.angular_position)

    
    def update_position(self,ax,ay,az):
        self.final_position_msg.x= ax
        self.final_position_msg.y= ay
        self.final_position_msg.z= az
        self.position_pub.publish(self.final_position_msg)


       
    def update_orientation(self, dt):
        self.orientation.x += self.angular_velocity.x * dt
        self.orientation.y += self.angular_velocity.y * dt
        self.orientation.z += self.angular_velocity.z * dt

        self.orientation_pub.publish(self.orientation)


    def run(self):
        # self.previous_time = rospy.get_time()
        # self.loop_rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    imu_accuracy_tester = IMUAccuracyTester()
    imu_accuracy_tester.run()

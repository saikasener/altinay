#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Twist
from scipy.integrate import cumtrapz
import time
import numpy as np


class IMUGyro:
    def __init__(self):
        rospy.init_node('imu_accuracy_tester')
        self.linear_acceleration = Vector3()
        self.angular_velocity = Vector3()
        self.position = Vector3()
        self.orientation = Vector3()
        self.previous_time = rospy.get_time()

        self.integ1_array_x=0
        self.integ1_array_y=0
        self.integ1_array_z=0 

        self.integ1_array =[]
        self.imu_array = []

        self.ax = 0
        self.ay = 0
        self.az = 0
        self.dif=0
        self.cal_size = 500  # points to use for calibration
        rospy.Subscriber('/imu_acc_ar', Imu, self.get_gyro)
        self.orientation_pub = rospy.Publisher('/imu/orientation', Vector3, queue_size=1)


    def get_gyro(self, data):
        dt = rospy.get_time() - self.previous_time      
        self.ax = data.linear_acceleration.x
        self.ay = data.linear_acceleration.y
        self.az = data.linear_acceleration.z
        self.gyro_cal()
        return self.ax,self.ay,self.az
        # self.update_angular_position(dt)

    def gyro_cal(self):
        # [self.get_gyro() for ii in range(0,self.cal_size)] # clear buffer before calibration
        # mpu_array = []
        # gyro_offsets = [0.0,0.0,0.0]
        # while True:
        #     try:
        #         self.wx,self.wy,self.wz = self.get_gyro() # get gyro vals
        #     except:
        #         continue

        #     mpu_array.append([self.wx,self.wy,self.wz])

        #     if np.shape(mpu_array)[0]==self.cal_size:
        #         for qq in range(0,3):
        #             gyro_offsets[qq] = np.mean(np.array(mpu_array)[:,qq]) # average
        #         break
        # print('Gyro Calibration Complete')
        # return gyro_offsets

       
        # self.imu_array.append([self.ax,self.ay,self.az])
        if self.ax > 0.001 and self.ay< 0.001:
            self.integrate_acc(self.ax,0,0)
        elif self.ay > 0.001 and self.ax< 0.001:
            self.integrate_acc(0,self.ay,0)
        elif self.ax< 0.001 and self.ay < 0.001:
            self.integrate_acc(0,0,self.az)
        elif self.ax > 0.001 and self.ay > 0.001:
            self.integrate_acc(self.ax,self.ay,0)  
                      
        else:
            print("-"*20)

    def integrate_acc(self,ax,ay,az):
        # print ("ax : ",ax, "             ay : ", ay)
# -------------------------------------------------
        x = np.arange(0, 10)
        y = np.arange(0, 10)
        

    def update_angular_position(self,dt):
        gyro_offsets = self.gyro_cal() # calculate gyro offsets
        data,t_vec = [],[]
        t0 = time.time()
        while time.time()-t0<dt:
            data.append(self.get_gyro())
            t_vec.append(time.time()-t0)

        rot_axis_x = 0 
        data_offseted_x = np.array(data)[:,rot_axis_x]-gyro_offsets[rot_axis_x]
        self.integ1_array_x = cumtrapz(data_offseted_x,x=t_vec) 

        rot_axis_y = 1 
        data_offseted_y = np.array(data)[:,rot_axis_y]-gyro_offsets[rot_axis_y]
        self.integ1_array_y = cumtrapz(data_offseted_y,x=t_vec) 
        
        rot_axis_z = 2 
        data_offseted_z = np.array(data)[:,rot_axis_z]-gyro_offsets[rot_axis_z]
        self.integ1_array_z = cumtrapz(data_offseted_z,x=t_vec)

        self.orientation = [self.integ1_array_x,self.integ1_array_y,self.integ1_array_z]
        self.orientation_pub.publish(self.orientation)

    #     dt = rospy.get_time() - self.previous_time
        # self.angular_position.x += self.angular_velocity.x * dt 
        # self.angular_position.y += self.angular_velocity.y * dt 
        # self.angular_position.z += self.angular_velocity.z * dt 
        # self.previous_time = rospy.get_time()


    



    def run(self):
        # self.previous_time = rospy.get_time()
        # self.loop_rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    imu_accuracy_tester = IMUGyro()
    imu_accuracy_tester.run()

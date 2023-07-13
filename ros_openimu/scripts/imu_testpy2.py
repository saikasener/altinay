#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

class IMUAccuracyTester:
    def __init__(self):
        rospy.init_node('imu_accuracy_tester')
        self.linear_acceleration = Vector3()
        self.angular_velocity = Vector3()
        self.position = Vector3()
        self.orientation = Vector3()
        self.previous_time = rospy.get_time()
        self.loop_rate = rospy.Rate(50)
        

        self.position.x = 0
        self.position.y = 0
        self.position.z = 0

        rospy.Subscriber('/imu_acc_ar', Imu, self.imu_callback)
        self.position_pub = rospy.Publisher('/imu/position', Vector3, queue_size=10)
        self.orientation_pub = rospy.Publisher('/imu/orientation', Vector3, queue_size=10)


    def imu_callback(self, data):
        dt = rospy.get_time() - self.previous_time      
        self.linear_acceleration.x = data.linear_acceleration.x
        self.linear_acceleration.y = data.linear_acceleration.y
        self.linear_acceleration.z = data.linear_acceleration.z
        
        self.update_position(dt)
        
        self.angular_velocity.x = data.angular_velocity.x
        self.angular_velocity.y = data.angular_velocity.y
        self.angular_velocity.z = data.angular_velocity.z

        self.update_orientation(dt)

    def update_linear_position(self):
        dt = rospy.get_time() - self.previous_time
        self.linear_velocity.x += self.linear_acceleration.x * dt
        self.linear_velocity.y += self.linear_acceleration.y * dt
        self.linear_velocity.z += self.linear_acceleration.z * dt

        self.linear_position.x += self.linear_velocity.x * dt 
        self.linear_position.y += self.linear_velocity.y * dt 
        self.linear_position.z += self.linear_velocity.z * dt 
        # self.previous_time = rospy.get_time()        
        self.distance_lin_pub.publish(self.linear_position)

    # def update_angular_position(self):
    #     dt = rospy.get_time() - self.previous_time
        self.angular_position.x += self.angular_velocity.x * dt 
        self.angular_position.y += self.angular_velocity.y * dt 
        self.angular_position.z += self.angular_velocity.z * dt 
        self.previous_time = rospy.get_time()

        self.distance_ang_pub.publish(self.angular_position)

    
    def update_position(self, dt):
        
        self.position.x += self.linear_acceleration.x * dt**2 / 2.0
        self.position.y += self.linear_acceleration.y * dt**2 / 2.0
        self.position.z += self.linear_acceleration.z * dt**2 / 2.0
        self.position_pub.publish(self.position)

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
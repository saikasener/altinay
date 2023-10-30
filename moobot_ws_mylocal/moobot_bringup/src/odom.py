import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class OdometryCalculator:
    def __init__(self):
        rospy.init_node('odometry_calculator')
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.velocity_sub = rospy.Subscriber('/cmd_vel_act', Twist, self.velocity_callback)
        self.last_time = rospy.Time.now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def velocity_callback(self, msg):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        delta_theta = angular_velocity * dt
        delta_x = linear_velocity * dt * math.cos(self.theta + delta_theta / 2.0)
        delta_y = linear_velocity * dt * math.sin(self.theta + delta_theta / 2.0)

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        self.publish_odometry()

        self.last_time = current_time

    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set the position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Set the orientation
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        calculator = OdometryCalculator()
        calculator.run()
    except rospy.ROSInterruptException:
        pass

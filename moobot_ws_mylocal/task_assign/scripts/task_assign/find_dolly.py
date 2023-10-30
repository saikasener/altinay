import rospy
from dynamic_reconfigure.client import Client


def say_it_works():
    rospy.loginfo("Success! You just have imported a Python module in another package.")

#!/usr/bin/env python3

import rospy
from ds4_driver.msg import Feedback, Status
from moobot_msgs.msg import lift_status

lift_status_msg = lift_status()
# publisher for lift status
lift_status_pub = rospy.Publisher(
    "/change_lift_status", lift_status, queue_size=1)


class Handler(object):
    def __init__(self, status_topic="status", feedback_topic="set_feedback"):
        self._min_interval = 0.1
        self._last_pub_time = rospy.Time()
        self._prev = Status()

        rospy.Subscriber("status", Status, self.cb_status, queue_size=1)

    def cb_status(self, msg):
        """
        :type msg: Status
        """
        now = rospy.Time.now()
        if (now - self._last_pub_time).to_sec() < self._min_interval:
            return

        button_x = msg.button_cross
        button_trio = msg.button_triangle
        if button_x == 1:
            lift_status_msg.up_lift = True
            lift_status_msg.down_lift = False
            lift_status_pub.publish(lift_status_msg)

        if button_trio == 1:
            lift_status_msg.up_lift = False
            lift_status_msg.down_lift = True
            lift_status_pub.publish(lift_status_msg)

        self._prev = msg
        self._last_pub_time = now


def main():
    rospy.init_node("sample")

    Handler()

    rospy.spin()


if __name__ == "__main__":
    main()

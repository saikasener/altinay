#!/usr/bin/env python3

# ################### OPEN RMF KODU 

import sys
import uuid
import argparse
import json
# import asyncio
import math

import rospy
# from rclpy.node import Node
# from rclpy.parameter import Parameter
# from rclpy.qos import qos_profile_system_default
# from rclpy.qos import QoSProfile
# from rclpy.qos import QoSHistoryPolicy as History
# from rclpy.qos import QoSDurabilityPolicy as Durability
# from rclpy.qos import QoSReliabilityPolicy as Reliability

from moobot_msgs.msg import ApiRequest, ApiResponse



###############################################################################

class TaskRequester():

    def __init__(self, argv=sys.argv):
        # rospy.init_node('task_requester')
        parser = argparse.ArgumentParser()
        parser.add_argument('-F', '--fleet', type=str, help='Fleet name')
        parser.add_argument('-R', '--robot', type=str, help='Robot name')
        parser.add_argument('-p', '--place', required=True, type=str, help='Place to go to')
        parser.add_argument('-o', '--orientation', required=False, type=float,help='Orientation to face in degrees (optional)')
        parser.add_argument('-st', '--start_time',help='Start time from now in secs, default: 0',type=int, default=0)
        parser.add_argument('-pt', '--priority',help='Priority value for this request',type=int, default=0)
        parser.add_argument("--use_sim_time", action="store_true",help='Use sim time, default: false')

        self.args = parser.parse_args(argv[1:])


        self.response = asyncio.Future()

        transient_qos= rospy.QoSProfile(
                depth=1,  # Depth of 1 message in the queue
                reliability=rospy.QoSProfile.RELIABLE,  # Reliable message delivery
                durability=rospy.QoSProfile.TRANSIENT_LOCAL,  # Transient-local durability
                history=rospy.QoSProfile.KEEP_LAST )

        # self.pub = self.create_publisher(ApiRequest, 'task_api_requests', transient_qos)
        self.pub = rospy.Publisher('task_api_requests', ApiRequest, queue_size=1)

        # enable ros sim time
        # if self.args.use_sim_time:
        #     self.get_logger().info("Using Sim Time")
        #     param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        #     self.set_parameters([param])

        # Construct task
        msg = ApiRequest()
        msg.request_id = "direct_" + str(uuid.uuid4())
        payload = {}

        if self.args.robot and self.args.fleet:
            self.get_logger().info("Using 'robot_task_request'")
            payload["type"] = "robot_task_request"
            payload["robot"] = self.args.robot
            payload["fleet"] = self.args.fleet
        else:
            self.get_logger().info("Using 'dispatch_task_request'")
            payload["type"] = "dispatch_task_request"

        # Set task request start time
        now = self.get_clock().now().to_msg()
        now.sec = now.sec + self.args.start_time
        start_time = now.sec * 1000 + round(now.nanosec/10**6)
        # todo(YV): Fill priority after schema is added

        # Define task request description
        go_to_description = {'waypoint': self.args.place}
        if self.args.orientation is not None:
            go_to_description['orientation'] = (
                self.args.orientation*math.pi/180.0
            )

        go_to_activity = {
            'category': 'go_to_place',
            'description': go_to_description
        }

        rmf_task_request = {
            'category': 'compose',
            'description': {
                'category': 'go_to_place',
                'phases': [{'activity': go_to_activity}]
            },
            'unix_millis_earliest_start_time': start_time
        }

        payload["request"] = rmf_task_request

        msg.json_msg = json.dumps(payload)

        def receive_response(response_msg: ApiResponse):
            if response_msg.request_id == msg.request_id:
                self.response.set_result(json.loads(response_msg.json_msg))

        # self.sub = self.create_subscription(
        #     ApiResponse, 'task_api_responses', receive_response, 10
        # )

        sub = rospy.Subscriber('task_api_responses', ApiResponse, receive_response)

        print(f"Json msg payload: \n{json.dumps(payload, indent=2)}")

        self.pub.publish(msg)


###############################################################################


# def main(argv=sys.argv):
#     rclpy.init(args=sys.argv)
#     args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)

#     task_requester = TaskRequester(args_without_ros)
#     rclpy.spin_until_future_complete(
#         task_requester, task_requester.response, timeout_sec=5.0)
#     if task_requester.response.done():
#         print(f'Got response:\n{task_requester.response.result()}')
#     else:
#         print('Did not get a response')
#     rclpy.shutdown()

###############################################################################


def load_task_data(task_file_path):
    with open(task_file_path, 'r') as json_file:
        return json.load(json_file)    

with open('/home/rnd/moobot_ws/src/moobot_plc/src/task_assign.json', 'w') as json_file:
                json.dump(task_data, json_file, indent=4)        

if __name__ == '__main__':
    move_base_client = SimpleActionClient('move_base', MoveBaseAction)
    move_base_client.wait_for_server()
    try:
        task_data = load_task_data('/home/rnd/moobot_ws/src/moobot_plc/src/task_assign.json')
        execute_tasks_for_robot('Robot1', task_data, move_base_client) #TODO farklı robotlar için düzenle
    except rospy.ROSInterruptException:
        pass

###############################################################################


def main(argv=sys.argv):
    rospy.init_node('task_requester')
    args_without_ros = rospy.myargv(sys.argv)

    task_requester = TaskRequester(args_without_ros)
    task_requester.response()
    
    # Custom logic for handling the asynchronous response
    rate = rospy.Rate(10)  # Adjust the rate as needed
    timeout = rospy.Duration(5.0)  # Timeout duration

    start_time = rospy.get_rostime()
    while (rospy.get_rostime() - start_time) < timeout:
        if task_requester.response.done():
            print(f'Got response:\n{task_requester.response.result()}')
            break
        rate.sleep()
    
    if not task_requester.response.done():
        print('Did not get a response')

    rospy.signal_shutdown('Shutting down...')
    rospy.spin()  # Spin to handle any remaining callbacks


if __name__ == '__main__':
    main(sys.argv)

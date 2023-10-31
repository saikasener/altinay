#!/usr/bin/env python3
import sys
import uuid
import argparse
import json

import rospy
from std_msgs.msg import String

###############################################################################

class TaskRequester:

    def __init__(self, argv=sys.argv):
        rospy.init_node('task_requester', anonymous=True)
        parser = argparse.ArgumentParser()
        parser.add_argument('-s', '--starts', default=[],
                            type=str, nargs='+', help='Action start waypoints')
        parser.add_argument('-st', '--start_time',
                            help='Start time from now in secs, default: 0',
                            type=int, default=0)
        parser.add_argument('-pt', '--priority',
                            help='Priority value for this request',
                            type=int, default=0)
        parser.add_argument('-a', '--action', required=True,
                            type=str, help='action name')
        parser.add_argument('-ad', '--action_desc', required=False,
                            default='{}',
                            type=str, help='action description in json str')
        parser.add_argument('-F', '--fleet', type=str,
                            help='Fleet name, should define tgt with robot')
        parser.add_argument('-R', '--robot', type=str,
                            help='Robot name, should define tgt with fleet')
        parser.add_argument("--use_sim_time", action="store_true",
                            help='Use sim time, default: false')
        parser.add_argument("--use_tool_sink", action="store_true",
                            help='Use tool sink during perform action, \
                                default: false')

        self.args = parser.parse_args(argv[1:])
        self.response = None

        # enable ros sim time
        if self.args.use_sim_time:
            rospy.loginfo("Using Sim Time")
            rospy.set_param("use_sim_time", True)

        # Construct task
        request_id = "custom_action_" + str(uuid.uuid4())
        payload = {}
        if self.args.fleet and self.args.robot:
            rospy.loginfo("Using 'robot_task_request'")
            payload["type"] = "robot_task_request"
            payload["robot"] = self.args.robot
            payload["fleet"] = self.args.fleet
        else:
            rospy.loginfo("Using 'dispatch_task_request'")
            payload["type"] = "dispatch_task_request"
        request = {}

        # Set task request start time
        now = rospy.Time.now()
        now.secs = now.secs + self.args.start_time
        start_time = now.secs * 1000 + round(now.nsecs/10**6)
        request["unix_millis_earliest_start_time"] = start_time

        # Define task request category
        request["category"] = "compose"

        # Define task request description with phases
        description = {}  # task_description_Compose.json
        description["category"] = self.args.action
        description["phases"] = []
        activities = []

        def _add_action():
            activities.append(
                {
                    "category": "perform_action",
                    "description":
                    {
                        "unix_millis_action_duration_estimate": 60000,
                        "category": self.args.action,
                        "description": json.loads(self.args.action_desc),
                        "use_tool_sink": self.args.use_tool_sink
                    }
                })

        if not self.args.starts:
            _add_action()
        else:
            # Add action activities
            for start in self.args.starts:
                activities.append({
                        "category": "go_to_place",
                        "description": start
                    })
                _add_action()

        # Add activities to phases
        description["phases"].append({
                "activity": {
                    "category": "sequence",
                    "description": {"activities": activities}}
            })

        request["description"] = description
        payload["request"] = request

        # Create publisher
        self.pub = rospy.Publisher('task_api_requests', String, queue_size=1)

        # Create subscriber
        self.sub = rospy.Subscriber('task_api_responses', String, self.receive_response)

        rospy.loginfo(f"msg: \n{json.dumps(payload, indent=2)}")

        # Publish message
        self.pub.publish(json.dumps(payload))

    def receive_response(self, response_msg):
        response_json = json.loads(response_msg.data)
        if response_json["request_id"] == self.response:
            self.response = response_json

def main(argv=sys.argv):
    task_requester = TaskRequester(argv[1:])
    rospy.spin()
    if task_requester.response:
        rospy.loginfo(f'Got response:\n{task_requester.response}')
    else:
        rospy.loginfo('Did not get a response')

if __name__ == '__main__':
    main(sys.argv)

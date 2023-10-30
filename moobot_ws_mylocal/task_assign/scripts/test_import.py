#!/usr/bin/env python3

import rospy
import json

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction

from task_assign.pick_up import lift_up
from task_assign.drop_off import lift_down
from task_assign.go_to_goal import execute_move_base_goal


def load_task_data(task_file_path):
    with open(task_file_path, 'r') as json_file:
        return json.load(json_file)


def execute_tasks_for_robot(robot_name, task_data, move_base_client):
    tasks = task_data.get(robot_name, [])
    for task in tasks:
        if task["target_executed"] == 0:
            execute_move_base_goal(move_base_client, task)
            for task_name in task["task"]:
                execute_task(task_name)
            task["target_executed"] = 1   
            with open('/home/rnd/moobot_ws/src/task_assign/schemas/task_assign.json', 'w') as json_file:
                json.dump(task_data, json_file, indent=4)
        else:
            rospy.logwarn("This task is already done")


def execute_task(task_name):
    if task_name == "lift_up":
        lift_up()
    elif task_name == "lift_down":
        lift_down()
    # elif task_name == "wait":
    #     wait()
    # elif task_name == "non_stop":
    #     non_stop()
    else:
        rospy.logwarn(f"Unknown task: {task_name}")


if __name__ == '__main__':
    rospy.init_node('robot_task_executor')
    move_base_client = SimpleActionClient('move_base', MoveBaseAction)
    move_base_client.wait_for_server()

    while not rospy.is_shutdown():
        try:
            task_data = load_task_data('/home/rnd/moobot_ws/src/moobot_plc/src/task_assign.json')
            execute_tasks_for_robot('Robot1', task_data, move_base_client) #TODO farklı robotlar için düzenle
        except rospy.ROSInterruptException:
            pass

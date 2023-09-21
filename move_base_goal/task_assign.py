#!/usr/bin/env python3
import rospy
import json
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient,GoalStatus
from dynamic_reconfigure.client import Client


def load_task_data(task_file_path):
    with open(task_file_path, 'r') as json_file:
        return json.load(json_file)

def execute_move_base_goal(move_base_client, task):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  
    goal.target_pose.pose.position.x = task["X"]
    goal.target_pose.pose.position.y = task["Y"]
    goal.target_pose.pose.position.z = task["Z"]
    goal.target_pose.pose.orientation.x = task["Orientation"][0]
    goal.target_pose.pose.orientation.y = task["Orientation"][1]
    goal.target_pose.pose.orientation.z = task["Orientation"][2]
    goal.target_pose.pose.orientation.w = task["Orientation"][3]
    move_base_client.send_goal(goal)
    move_base_client.wait_for_result()
    state = move_base_client.get_state()
    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully!")
    else:
        rospy.logwarn("Failed to reach the goal!")

def update_footprint(new_footprint):
    new_config = {"footprint": new_footprint}
    # ------- GLOBAL COSTMAP FOOTPRINT
    try:
        rospy.wait_for_service("/move_base/global_costmap/set_parameters")
        global_client = Client("/move_base/global_costmap")
        global_client.update_configuration(new_config)
    except rospy.ServiceException as e:
        rospy.logwarn(f"Failed to update 'footprint' parameter for global costmap: {e}")
    # ------- LOCAL COSTMAP FOOTPRINT
    try:
        rospy.wait_for_service("/move_base/local_costmap/set_parameters")
        local_client = Client("/move_base/local_costmap")
        local_client.update_configuration(new_config) 
    except rospy.ServiceException as e:
        rospy.logwarn(f"Failed to update 'footprint' parameter for local costmap: {e}")

def update_scanner_view(new_angle_start, new_angle_end):
    new_config_start = {"angle_start": new_angle_start}
    new_config_end = {"angle_end": new_angle_end}
    # ------- REAR SCANNER
    try:
        rospy.wait_for_service("/rear_laser_scanner/rear_laser_scanner/set_parameters")
        rear_client = Client("/rear_laser_scanner/rear_laser_scanner/")
        rear_client.update_configuration(new_config_start)
        rear_client.update_configuration(new_config_end)
    except rospy.ServiceException as e:
        rospy.logwarn(f"Failed to update 'rear_laser_scanner' parameter: {e}")
    # ------- FRONT SCANNER
    try:
        rospy.wait_for_service("/front_laser_scanner/front_laser_scanner/set_parameters")
        front_client = Client("/front_laser_scanner/front_laser_scanner")
        front_client.update_configuration(new_config_start) 
        front_client.update_configuration(new_config_end) 
    except rospy.ServiceException as e:
        rospy.logwarn(f"Failed to update 'front_laser_scanner' parameter: {e}")

def lift_up():
    rospy.loginfo("Lift Up")

    # ------- publishing lift command
    #TODO Naci'deki custom lift service alınacak 
    rospy.loginfo("lift status published -> up")

    # ------- increasing footprint
    wide_footprint = [[-1.1,-0.8],[-1.1,0.8],[1.1,0.8],[1.1,-0.8]]
    update_footprint(wide_footprint)  

    # ------- decreasing scanner area 
    angle_start = -0.5
    angle_end = 2.3
    # update_scanner_view(angle_start,angle_end) 
    rospy.loginfo("scanner view changed -> narrow")

def lift_down():
    rospy.loginfo("Lift Down")
    
    # ------- publishing lift command
    #TODO Naci'deki custom lift service alınacak
    rospy.loginfo("lift status published -> down")

    # ------- decreasing footprint
    narrow_footprint = [[-1, -0.6], [-1, 0.6], [1, 0.6], [1, -0.6]] 
    update_footprint(narrow_footprint)

    # ------- increasing scanner area  
    angle_start = -2.25
    angle_end = 2.25
    # update_scanner_view(angle_start,angle_end)
    rospy.loginfo("scanner view changed -> wider")
        

def wait():
    rospy.loginfo("Wait 10 seconds")
    time.sleep(10)

def non_stop():
    rospy.loginfo("Non-stop")
    pass

def execute_task(task_name):
    if task_name == "lift_up":
        lift_up()
    elif task_name == "lift_down":
        lift_down()
    elif task_name == "wait":
        wait()
    elif task_name == "non_stop":
        non_stop()
    else:
        rospy.logwarn(f"Unknown task: {task_name}")
   

def execute_tasks_for_robot(robot_name, task_data, move_base_client):
    tasks = task_data.get(robot_name, [])
    for task in tasks:
        if task["target_executed"] == 0:
            execute_move_base_goal(move_base_client, task)
            for task_name in task["task"]:
                execute_task(task_name)
            task["target_executed"] = 1   # task executed succesfully 
            with open('/home/rnd/moobot_ws/src/moobot_plc/src/assigned_tasks.json', 'w') as json_file:
                json.dump(task_data, json_file, indent=4)
        else:
            rospy.logwarn("This task is already done")

            

if __name__ == '__main__':
    rospy.init_node('robot_task_executor')
    # lift_status_pub = rospy.Publisher("/change_lift_status", lift_status, queue_size=1)
    move_base_client = SimpleActionClient('move_base', MoveBaseAction)
    move_base_client.wait_for_server()
    try:
        task_data = load_task_data('/home/rnd/moobot_ws/src/moobot_plc/src/assigned_tasks.json')
        execute_tasks_for_robot('Robot1', task_data, move_base_client) #TODO farklı robotlar için düzenle
    except rospy.ROSInterruptException:
        pass

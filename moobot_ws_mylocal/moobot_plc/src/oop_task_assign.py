#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient,GoalStatus
import json

from dynamic_reconfigure.client import Client
from rospy.exceptions import ROSException


class RobotTaskExecutor:
    def __init__(self, task_file_path):
        rospy.init_node('robot_task_executor')
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.task_data = self.load_task_data(task_file_path)

    def load_task_data(self, task_file_path):
        with open(task_file_path, 'r') as json_file:
            return json.load(json_file)

    def execute_move_base_goal(self, task):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  
        goal.target_pose.pose.position.x = task["X"]
        goal.target_pose.pose.position.y = task["Y"]
        goal.target_pose.pose.position.z = task["Z"]
        goal.target_pose.pose.orientation.x = task["Orientation"][0]
        goal.target_pose.pose.orientation.y = task["Orientation"][1]
        goal.target_pose.pose.orientation.z = task["Orientation"][2]
        goal.target_pose.pose.orientation.w = task["Orientation"][3]

        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

        state = self.move_base_client.get_state()

        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully!")
        else:
            rospy.logwarn("Failed to reach the goal!")

    def update_footprint(self, new_footprint):
        new_config = {"footprint": new_footprint}
        #GLOBAL COSTMAP FOOTPRINT
        try:
            rospy.wait_for_service("/move_base/global_costmap/set_parameters")
            global_client = Client("/move_base/global_costmap")
            global_client.update_configuration(new_config)
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to update 'footprint' parameter for global costmap: {e}")
        #LOCAL COSTMAP FOOTPRINT
        try:
            rospy.wait_for_service("/move_base/local_costmap/set_parameters")
            local_client = Client("/move_base/local_costmap")
            local_client.update_configuration(new_config) 
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to update 'footprint' parameter for local costmap: {e}")

    def update_scanner_view(self, new_angle_start, new_angle_end):
        new_config_start = {"angle_start": new_angle_start}
        new_config_end = {"angle_end": new_angle_end}
        #REAR SCANNER
        try:
            rospy.wait_for_service("/rear_laser_scanner/rear_laser_scanner/set_parameters")
            rear_client = Client("/rear_laser_scanner/rear_laser_scanner/")
            rear_client.update_configuration(new_config_start)
            rear_client.update_configuration(new_config_end)
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to update 'rear_laser_scanner' parameter: {e}")
        #local_costmap footprint
        try:
            rospy.wait_for_service("/front_laser_scanner/front_laser_scanner/set_parameters")
            front_client = Client("/front_laser_scanner/front_laser_scanner")
            front_client.update_configuration(new_config_start) 
            front_client.update_configuration(new_config_end) 
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to update 'front_laser_scanner' parameter: {e}")


    def lift_up(self):
        rospy.loginfo("Lift Up")
        wide_footprint = [[-1.1,-0.8],[-1.1,0.8],[1.1,0.8],[1.1,-0.8]]
        self.update_footprint(wide_footprint) # increasing footprint 
        angle_start = -0.5
        angle_end = 2.3
        # self.update_scanner_view(angle_start,angle_end) # decreasing scanner area 
        

    def lift_down(self):
        rospy.loginfo("Lift Down")
        narrow_footprint = [[-1, -0.6], [-1, 0.6], [1, 0.6], [1, -0.6]] 
        self.update_footprint(narrow_footprint) # decreasing footprint
        angle_start = -2.25
        angle_end = 2.25
        # self.update_scanner_view(angle_start,angle_end) # increasing scanner area         

    def wait(self):
        rospy.loginfo("Wait")
        

    def non_stop(self):
        rospy.loginfo("Non-stop")
        pass

    def execute_task(self, task_name):
        if task_name == "lift_up":
            self.lift_up()
        elif task_name == "lift_down":
            self.lift_down()
        elif task_name == "wait":
            self.wait()
        elif task_name == "non_stop":
            self.non_stop()
        else:
            rospy.logwarn(f"Unknown task: {task_name}")

    def execute_tasks_for_robot(self, robot_name):
        tasks = self.task_data.get(robot_name, [])
        for task in tasks:
            self.execute_move_base_goal(task)
            for task_name in task["task"]:
                self.execute_task(task_name)

if __name__ == '__main__':
    try:
        task_executor = RobotTaskExecutor('/home/rnd/moobot_ws/src/moobot_plc/src/coordinates.json')
        task_executor.execute_tasks_for_robot('Robot1')
    except rospy.ROSInterruptException:
        pass

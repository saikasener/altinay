#!/usr/bin/env python3
import json
import rospy
import actionlib
from geometry_msgs.msg import Point, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def send_goal(x_goal, y_goal, quaternion):
    move_base_client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(
        Point(x_goal, y_goal, 0.000), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
    print(goal)
    move_base_client.send_goal(goal)

    move_base_client.wait_for_result()

    state = move_base_client.get_state()

    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully!")
    else:
        rospy.logwarn("Failed to reach the goal!")


if __name__ == "__main__":
    rospy.init_node("moobot_task")
    
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
      
    while not rospy.is_shutdown():  
        json_file_path = "/home/rnd/moobot_ws/src/moobot_plc/src/coordinates.json"  

        try:
            with open(json_file_path, "r") as json_file:
                data = json.load(json_file)
                stations = data["Goals"]
                for station in stations:
                    station_x = station.get("X")
                    station_y = station.get("Y")
                    orientation = station.get("Orientation") #quaternion data
                    send_goal(station_x, station_y, orientation)
        except FileNotFoundError:
            print(f"File not found: {json_file_path}")
        except json.JSONDecodeError as e:
            print(f"JSON decoding error: {e}")

        rospy.spin()

    
    

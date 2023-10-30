import rospy
from move_base_msgs.msg import MoveBaseGoal
from actionlib import GoalStatus



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
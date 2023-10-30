import rospy
from dynamic_reconfigure.client import Client


def say_it_works():
    rospy.loginfo("Success! You just have imported a Python module in another package.")


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
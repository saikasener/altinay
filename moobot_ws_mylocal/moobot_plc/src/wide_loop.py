#!/usr/bin/env python3
import sys
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion
from actionlib_msgs.msg import GoalID
from timeit import default_timer as timer
from opcua import Client
from opcua import ua
import time
import yaml
import signal
import math
from moobot_msgs.msg import moobot_scanner, moobot_status, conveyor_status
from std_msgs.msg import Bool, Int16
from moobot_pgv.msg import pgv_scan_data
import pickle
import os
from geometry_msgs.msg import Twist
from threading import Thread
import time

IDLE = 0
EMG = 100
FAULT = 101
# STATIONS
STATION1 = 1
STATION2 = 2
STATION3 = 3
STATION4 = 4
STATION5 = 5

GOING_STATION1 = 6
GOING_STATION2 = 7
GOING_STATION3 = 8
GOING_STATION4 = 9
GOING_STATION5 = 10
GOING = 999

#TODO: orientation'lar belirlenecek
orientations = [3.1415, 1.57, None, None, -3.12]


sys.path.insert(0, "..")


cmd_vel_pub_line_follower = rospy.Publisher("/cmd_vel", Twist, queue_size=2)
cmd_vel_msg = Twist()

is_shutdown = False
def signal_handler(sig, frame):
    global is_shutdown
    is_shutdown = True
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


class Station:
    def __init__(self, number, x, y, z):
        self.number = number
        self.x = x
        self.y = y
        self.z = z


class ScannerData:
    def __init__(self, fs_ossd, fs_w1, fs_w2, bs_ossd, bs_w1, bs_w2):
        self.fs_ossd = fs_ossd
        self.fs_w1 = fs_w1
        self.fs_w2 = fs_w2
        self.bs_ossd = bs_ossd
        self.bs_w1 = bs_w1
        self.bs_w2 = bs_w2


stations_list = []

scanner_data = ScannerData(0, 0, 0, 0, 0, 0)
a_yaml_file = open(
    "/home/rnd/moobot_ws/src/moobot_navigation/moobot_navigation.rviz")
parsed_yaml_file = yaml.load(a_yaml_file, Loader=yaml.FullLoader)
vm = parsed_yaml_file["Visualization Manager"]
for x in vm:
    if x == "Tools":
        tools = vm[x]
        break
for tool in tools:
    if tool.get("Class") == "moobot_ui/Station":
        stations = tool.get("Stations")

for station in stations:

    station_name = station.get("Name")
    s = str.split(station_name)
    number = int(s[1])

    station_x = station.get("X")
    station_y = station.get("Y")
    station_z = station.get("Z")

    new_station = Station(number, station_x, station_y, station_z)
    stations_list.append(new_station)


print("Stations ")
for s in stations_list:
    print("Station:", s.number, "x:", s.x, "y:", s.y, "z:", s.z)


goal_station = 0
next_goal_station = 0
NARROW = 1
NORMAL = 0
scanner_area = NORMAL


def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
        math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
        math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
        math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
        math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    return [qx, qy, qz, qw]

goal_sended = False
def send_goal(x_goal, y_goal, orientation, station_num):
    global current_pos
    global goal_sended
    goal_pos = IDLE
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    q = euler_to_quaternion(0.0, 0.0, orientation)
    goal.target_pose.pose = Pose(
        Point(x_goal, y_goal, 0.000), Quaternion(q[0], q[1], q[2], q[3]))
    move_base_client.send_goal(goal)
    # TODO: bunu kapatıp yine dene
    # time.sleep(2)
    goal_pos = current_pos
    current_pos = current_pos + 5

    # TODO: Hedef değiştirilirse ne olacağını dene
    
    success = move_base_client.wait_for_result()
    state = move_base_client.get_state()
    if success and state == 3:
        print(current_pos)
        goal_sended = False
    elif success and state != 3:  # goal'a gidemiyor
        print("Goal state is", state)
        #if state == 4 or state == 5:
        os.system("rosservice call /move_base/clear_costmaps \"{}\"")
        time.sleep(2)
        goal_sended = False
        current_pos = goal_pos
                


def send_station(goal_station):
    global goal_sended
    goal_sended = True
    for s in stations_list:
        if s.number == goal_station:
            x_goal = s.x
            y_goal = s.y
            orientation = orientations[s.number-1]
            print("Set goal on station ", s.number)
            send_goal(x_goal, y_goal, orientation, goal_station)
            break


def check_status(moobot_status):
    global current_pos
    global old_pos
    if moobot_status.emergency == True:
        current_pos = EMG
    else:
        if current_pos == EMG or current_pos == FAULT:  # emg den çıktıysa eski pos a dönmesi için
            current_pos = old_pos


conveyor_transfer_done = False
scanner_area = 0
ossd_active = False


def check_scanner_area(Int16):
    global scanner_area
    scanner_area = Int16.data


def check_scanner_status(moobot_scanner):
    global ossd_active
    ossd_active = (moobot_scanner.fs_ossd == 1 or moobot_scanner.bs_ossd == 1)


conveyor_transfer_start = False
conveyor_running = False
conveyor_loaded = False
pallet_loaded = False



# goal_x_tolerance = 5.0
goal_y_tolerance = 2.0
goal_theta_tolerance = 0.5
max_linear_vel = 0.15
max_angular_vel = 0.05
min_angular_vel = 0.01
reference_x = 680.0
reference_y = 63.0

cmd_vel_linear_sp = 0.0
cmd_vel_angular_sp = 0.0


if os.path.getsize('/home/rnd/moobot_ws/src/moobot_plc/src/current_pos.pickle') > 0:
    with open('/home/rnd/moobot_ws/src/moobot_plc/src/current_pos.pickle', 'rb') as f:
        current_pos = pickle.load(f)
else:
    current_pos = IDLE

# AGV poses
current_pos = IDLE
old_pos = IDLE
source_pos = IDLE
print("Current pos", current_pos)

PGV_data = 0
following_line = False
goal_sended = False
time_t = 0.0
old_time_t = 0.0
live_bit = False
if __name__ == "__main__":
    rospy.init_node("controller_plc")

    # moobot_status subscriber ı olacak emg gelirse buna da emg yollayacak veya fault yollayacak
    moobot_status_sub_c_plc = rospy.Subscriber(
        "/agv_status", moobot_status, check_status)
    move_base_client = actionlib.SimpleActionClient(
        'move_base', MoveBaseAction)
    cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
    cancel_msg = GoalID()
   
    c_plc_emergency_pub = rospy.Publisher(
        "/controller_plc_emergency", Bool, queue_size=2)
    scannner_area_sub = rospy.Subscriber(
        '/scanner_area', Int16, check_scanner_area)
    scanner_area_change_pub = rospy.Publisher(
        "/change_scanner_area", Int16, queue_size=2)
    scanner_status_sub = rospy.Subscriber(
        "/scanner_status", moobot_scanner, check_scanner_status)

    if current_pos >= GOING_STATION1 and current_pos <= GOING_STATION2:
        thread_send_station = Thread(
            target=send_station, args=[current_pos-12])
        thread_send_station.start()
    time.sleep(2)
    while is_shutdown == False:
        if current_pos == EMG:
            print("Emergency")
            cmd_vel_msg.angular.z = 0.0
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_pub_line_follower.publish(cmd_vel_msg)
            cancel_pub.publish(cancel_msg)
        else:
            if current_pos == IDLE:
                if goal_sended == False:
                    print("idle")
                    thread_send_station = Thread(
                        target=send_station, args=[STATION1], daemon=True)
                    thread_send_station.start()

            elif current_pos == STATION1:
                time.sleep(2)
                if goal_sended == False:
                    print("station1 geldim.")
                    thread_send_station = Thread(
                        target=send_station, args=[STATION2], daemon=True)
                    thread_send_station.start()

            elif current_pos == STATION2:
                time.sleep(2)
                if goal_sended == False:
                    print("station2 geldim.")
                    thread_send_station = Thread(
                        target=send_station, args=[STATION3], daemon=True)
                    thread_send_station.start()
                

            elif current_pos == STATION3:
                time.sleep(2)
                if goal_sended == False:
                    print("station3 geldim.")
                    thread_send_station = Thread(
                        target=send_station, args=[STATION4], daemon=True)
                    thread_send_station.start()
                    
            elif current_pos == STATION4:
                time.sleep(2)
                if goal_sended == False:
                    print("station4 geldim.")
                    thread_send_station = Thread(
                        target=send_station, args=[STATION5], daemon=True)
                    thread_send_station.start()

            elif current_pos == STATION5:
                time.sleep(2)
                if goal_sended == False:
                    print("station5 geldim.")
                    thread_send_station = Thread(
                        target=send_station, args=[STATION1], daemon=True)
                    thread_send_station.start()
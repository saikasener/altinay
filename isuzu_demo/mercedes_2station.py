#!/usr/bin/env python3
'''
MERCEDES -> 2 istasyon arası sürüş (lift - boş )
'''

import sys
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion
from actionlib_msgs.msg import GoalID
from timeit import default_timer as timer
import time
import yaml
import signal
import math
from moobot_msgs.msg import moobot_scanner, moobot_status, lift_status
from std_msgs.msg import Bool, Int16
from moobot_pgv.msg import pgv_scan_data
import pickle
import os
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from threading import Thread, currentThread
import time
msg = Int16()

pallet_loaded = False
pallet_loaded_two = True
pallet_loaded_three = False
load_turn = 2
IDLE = 0
EMG = 100
FAULT = 101


GOING_HOME = 34
GOING_LIFT_STATION = 35

# STATIONS and states
A_station = 1
# A_station_EK = 2
B_station = 2
# B_station_EK = 4


MAKE_LIFT_UP = 24
MAKE_LIFT_DOWN = 25
MAKE_LIFT_UP_D1 = 26
MAKE_LIFT_DOWN_D1 = 27
MAKE_LIFT_UP_D2 = 28
MAKE_LIFT_DOWN_D2 = 29
MAKE_LIFT_UP_D3 = 30
MAKE_LIFT_DOWN_D3 = 31

A_station_GO = 41
# A_station_EK_GO = 42
B_station_GO = 42
# B_station_EK_GO = 44


A_station_WAIT = 71
# A_station_EK_WAIT = 72
B_station_WAIT = 72
# B_station_EK_WAIT = 74


orientations = [-1.57,
                -1.57,
                None]

# SCANNER AREAS
DEFAULT_SCANNER_AREA = 4
LIFT_SCANNER_AREA = 4

is_shutdown = False
lane_detected = 1

sys.path.insert(0, "..")


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

# Read yaml file and get stations
a_yaml_file = open(
    "/home/rnd/moobot_ws/src/moobot_navigation/moobot_navigation_mercedes.rviz")
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

    if number == 1:
        new_station = Station(A_station,
                              station_x, station_y, station_z)
        stations_list.append(new_station)
    elif number == 2:
        new_station = Station(B_station,
                              station_x, station_y, station_z)
        stations_list.append(new_station)


print("Stations ")
for s in stations_list:
    print("Station:", s.number, "x:", s.x, "y:", s.y, "z:", s.z)


goal_station = 0
next_goal_station = 0
NARROW = 1
NORMAL = 0
scanner_area = NORMAL

# AGV poses
current_pos = IDLE
old_pos = IDLE
source_pos = IDLE


following_line = False
goal_sended = False
time_t = 0.0
old_time_t = 0.0
live_bit = False


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


def send_goal(x_goal, y_goal, orientation, station_num, current_station):
    global current_pos
    global goal_sended
    goal_sended = True

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    q = euler_to_quaternion(0.0, 0.0, orientation)
    goal.target_pose.pose = Pose(
        Point(x_goal, y_goal, 0.000), Quaternion(q[0], q[1], q[2], q[3]))
    move_base_client.send_goal(goal)
    success = move_base_client.wait_for_result()
    state = move_base_client.get_state()
    if state != 3:
        print(state)
    if success and state == 3:  # goal a gidebilmiş
        current_pos = station_num
        print(current_pos)
        goal_sended = False
    elif success and state != 3:  # goal'a gidemiyor
        print("Goal state is", state)
        os.system("rosservice call /move_base/clear_costmaps \"{}\"")
        time.sleep(2)
        goal_sended = False
        current_pos = current_station


def send_station(goal_station, current_station):
    global current_pos
    # current_pos = current_pos + 8
    for s in stations_list:
        if s.number == goal_station:
            x_goal = s.x
            y_goal = s.y
            orientation = orientations[s.number-1]
            print("Set goal on station ", s.number)
            send_goal(x_goal, y_goal, orientation,
                      goal_station, current_station)
            break


def check_status(moobot_status):
    global current_pos
    global old_pos
    if moobot_status.emergency == True:
        current_pos = EMG
    else:
        if current_pos == EMG or current_pos == FAULT:  # emg den çıktıysa eski pos a dönmesi için
            current_pos = old_pos


def check_scanner_area(Int16):
    global scanner_area
    scanner_area = Int16.data


LIFT_UNKNOWN = 0
LIFT_UP = 1
LIFT_DOWN = 2

lift_status_val = IDLE

lift_status_msg = lift_status()

lift_last_time = time.time()


def check_lift_status(Int16):
    global lift_status_val
    global lift_last_time
    lift_status_val = Int16.data
    lift_last_time = time.time()
    # print(lift_status_val)


goal_theta_tolerance = 0.5
max_linear_vel = 0.1  # 0.05
max_angular_vel = 0.06  # 0.10
min_angular_vel = 0.01
reference_x = 680.0
reference_y = 63.0

cmd_vel_linear_sp = 0.0
cmd_vel_angular_sp = 0.0

current_pos_x = 0.0
current_pos_y = 0.0
current_pos_theta = 0.0
follow_line_reached = False

agv_job_status = "Logos starts in auto mode"


def calculate_agv_conveyor_pos(pgv_scan_data):
    global agv_conveyor_x_pos
    global agv_conveyor_y_pos
    global agv_conveyor_orientation
    global current_tag_num
    global lane_detected
    global current_pos_x
    global current_pos_y
    global current_pos_theta
    global read_barcode

    # TODO: burada aralığı check et
    lane_detected = pgv_scan_data.lane_detected
    if lane_detected == 0:
        current_pos_x = pgv_scan_data.x_pos
        current_pos_y = pgv_scan_data.y_pos
        current_pos_theta = pgv_scan_data.orientation


# line follower globalde tanımlandı çünkü follow_line kodu içinde publish ediliyor
cmd_vel_pub_line_follower = rospy.Publisher("/cmd_vel", Twist, queue_size=2)
cmd_vel_msg = Twist()

# Assume that goal is always in 0,0 and calculate speeds according to given x value of goal position
# PS: this function always works in different thread


def follow_line(goal_x_tolerance, goal_orientation, goal_pos_x):
    global cmd_vel_angular_sp
    global cmd_vel_linear_sp
    global follow_line_reached

    if current_pos != EMG:
        # print("whiledan cikti")
        '''while math.sqrt((current_pos_theta + goal_orientation)**2) > goal_theta_tolerance: #burası ne yapıyor?
            if (current_pos_theta + goal_orientation) < 0:
                cmd_vel_angular_sp = -1 * min_angular_vel
            else:
                cmd_vel_angular_sp = min_angular_vel
            cmd_vel_linear_sp = 0.0
            cmd_vel_msg.angular.z = cmd_vel_angular_sp
            cmd_vel_msg.linear.x = cmd_vel_linear_sp
            cmd_vel_pub_line_follower.publish(cmd_vel_msg)
            time.sleep(0.05)'''

        # while current_pos_x > 85.0:  # hız ver
        #     cmd_vel_msg.linear.x = -0.01
        #     cmd_vel_msg.angular.z = 0.0
        #     cmd_vel_pub_line_follower.publish(cmd_vel_msg)
        #     time.sleep(0.05)

        print("Reached the end of the line")
        follow_line_reached = True
        cmd_vel_linear_sp = 0.0
        cmd_vel_angular_sp = 0.0
        cmd_vel_msg.angular.z = cmd_vel_angular_sp
        cmd_vel_msg.linear.x = cmd_vel_linear_sp
        cmd_vel_pub_line_follower.publish(cmd_vel_msg)
        cmd_vel_pub_line_follower.publish(cmd_vel_msg)


if __name__ == "__main__":
    # Initialize pallet_loaded value before start since we don't have a sensor for checking
    # TODO: burası değişmeli başlangıçta paletin nerede olduğunu bilmeliyiz
    args = sys.argv[1:]
    if (len(args) > 0):
        print("Pallet loaded is set to " + args[0])
        if args[0] == "True" or args[0] == "true":
            pallet_loaded = True
        elif args[0] == "False" or args[0] == "false":
            pallet_loaded = False

    rospy.init_node("moobot_controller")

    # subscriber for checking emg or fault
    moobot_status_sub_c_plc = rospy.Subscriber(
        "/agv_status", moobot_status, check_status)

    # client for path following
    move_base_client = actionlib.SimpleActionClient(
        'move_base', MoveBaseAction)

    # publisher for cancelling goals
    cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
    cancel_msg = GoalID()

    # subscriber for checking scanner area type
    scannner_area_sub = rospy.Subscriber(
        '/scanner_area', Int16, check_scanner_area)

    # publisher for changing scanner area type
    scanner_area_change_pub = rospy.Publisher(
        "/change_scanner_area", Int16, queue_size=1)

    # publisher for lift status
    lift_status_pub = rospy.Publisher(
        "/change_lift_status", lift_status, queue_size=1)

    # subscriber for lift status
    lift_status_sub = rospy.Subscriber(
        '/lift_status', Int16, check_lift_status)

    # subscriber for pgv scan data
    pgv_scan_data_sub = rospy.Subscriber(
        "/pgv_scan", pgv_scan_data, calculate_agv_conveyor_pos)

    # publisher for agv job status
    agv_job_status_pub = rospy.Publisher(
        "/agv_job_status", String, queue_size=3)

    # Normally,current_pose info should read from a file
    # If code exists while agv running in auto mode and agv is in going state, it should reach the goal first
    if current_pos >= GOING_HOME and current_pos <= GOING_LIFT_STATION:
        thread_send_station = Thread(
            target=send_station, args=[current_pos-5, current_pos])
        thread_send_station.start()
    time.sleep(2)

    # init timers for agv status publisher
    start_time = time.time()
    end_time = time.time()
    lift_start_val = 0

    # If agv is in idle mode in begining, moves lift down
    if current_pos == IDLE:
        lift_status_msg.up_lift = False
        lift_status_msg.down_lift = True
        lift_start_val = LIFT_DOWN
        lift_status_pub.publish(lift_status_msg)

    is_shutdown = False
    change_scanner_publish_time = time.time()
    change_lift_status_time = time.time()
    emg_start_time = time.time()

# State Machine
# -------------
    while is_shutdown == False:
        end_time = time.time()
        if (end_time - start_time) > 2:
            agv_job_status_pub.publish(agv_job_status)
            start_time = end_time

        # if (time.time() - lift_last_time) > 10:
        #     os.system("rosnode kill /moobot_lift")
        #     print("killed moobot_lift node")
        #     lift_last_time = time.time()

        # If emg comes publishes stops agv and cancel goal in every two seconds
        if current_pos == EMG:
            if time.time() - emg_start_time > 2:
                agv_job_status = "Logos is in emergency situtation"
                cmd_vel_msg.angular.z = 0.0
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_pub_line_follower.publish(cmd_vel_msg)
                cancel_pub.publish(cancel_msg)
                emg_start_time = time.time()

        # If agv is not in emergency, state machine works
        else:
            if current_pos == IDLE:
                if goal_sended == False:
                    thread_send_station = Thread(
                        target=send_station, args=[A_station, current_pos], daemon=True)
                    thread_send_station.start()


# 1. İSTASYON (HOME)-> GİRİŞ DOLLY 1 İLE AYNI 
            elif current_pos == A_station:
                print("A_station'a geldim! ")
                if goal_sended == False:
                    current_pos = B_station_WAIT

# 2. İSTASYON - giriş1
            elif current_pos == B_station_WAIT:  # homeda bekliyor
                print("B_station_WAIT'a gitme için 1 saniye bekleniyor...")
                time.sleep(1.0)
                current_pos = B_station_GO

            elif current_pos == B_station_GO:
                if goal_sended == False:
                    next_station = B_station
                    thread_send_station = Thread(
                        target=send_station, args=[next_station, current_pos], daemon=True)
                    thread_send_station.start()

            elif current_pos == B_station:
                print("B_station'a geldim.")
                if goal_sended == False:
                    current_pos = A_station_WAIT

# 3. İSTASYONA - dolly_load
            elif current_pos == A_station_WAIT:
                print("A_station'a gitmek için 1 saniye bekleniyor...")
                time.sleep(1.0)
                current_pos = A_station_GO

            elif current_pos == A_station_GO:
                if goal_sended == False:
                    next_station = A_station
                    thread_send_station = Thread(
                        target=send_station, args=[next_station, current_pos], daemon=True)
                    thread_send_station.start()

            old_pos = current_pos

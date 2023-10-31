#!/usr/bin/env python3
'''
PS: State machine diagram can be found in documentation
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


pallet_loaded = False
pallet_loaded_two = True
pallet_loaded_three = False
load_turn = 2
IDLE = 0
EMG = 100
FAULT = 101

# STATIONS and states
HOME = 1
MIDDLE_STATION = 2  # TODO: 2 tane ayrı mid station gerekebilir
LIFT_STATION_BEFORE_TWO = 3  # load lift on agv
LIFT_STATION_OUT_TWO = 4
LIFT_STATION_BEFORE_THREE = 5  # unload lift on agv
LIFT_STATION_OUT_THREE = 6
LIFT_STATION = 7

GOING_HOME = 9
GOING_MIDDLE_STATION = 10
GOING_LIFT_STATION_BEFORE_TWO = 11
GOING_LIFT_STATION_OUT_TWO = 12
GOING_LIFT_STATION_BEFORE_THREE = 13
GOING_LIFT_STATION_OUT_THREE = 14
GOING_LIFT_STATION = 15

IN_FOLLOW_LINE = 16
CHANGE_AREA_LIFT = 17
CHANGE_AREA_DEFAULT = 18
LOAD_PALLET = 19
UNLOAD_PALLET = 20
LOAD_PALLET_DONE = 21
UNLOAD_PALLET_DONE = 22
LIFT_STATION_DONE = 23
MAKE_LIFT_UP = 24
MAKE_LIFT_DOWN = 25


#orientations = [0.0, 0.0334188, 0.0334188, 0.0334188, None]
# eskiden -0.03
orientations = [3.14, 3.14, 3.14, 3.14, 0.0, 0.0, None]
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

    #new_station = Station(number, station_x, station_y, station_z)
    if number == 1:
        new_station = Station(HOME,
                              station_x, station_y, station_z)
        stations_list.append(new_station)

        new_station = Station(MIDDLE_STATION,
                              station_x, station_y, station_z)
        stations_list.append(new_station)
    elif number == 2:
        new_station = Station(LIFT_STATION_BEFORE_TWO,
                              station_x, station_y, station_z)
        stations_list.append(new_station)

        new_station = Station(LIFT_STATION_OUT_TWO,
                              station_x, station_y, station_z)
        stations_list.append(new_station)

    elif number == 3:
        new_station = Station(LIFT_STATION_BEFORE_THREE,
                              station_x, station_y, station_z)
        stations_list.append(new_station)

        new_station = Station(LIFT_STATION_OUT_THREE,
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
    current_pos = current_pos + 8
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
        while (current_pos_x - goal_pos_x) > goal_x_tolerance:  # hız ver
            cmd_vel_linear_sp = (current_pos_x - goal_pos_x) * max_linear_vel / \
                reference_x * (1 - abs(current_pos_y)/reference_y)
            if current_pos_y > 10.0 or current_pos_y < -10.0:
                cmd_vel_angular_sp = current_pos_y * max_angular_vel / reference_y
            else:
                cmd_vel_angular_sp = current_pos_theta * max_angular_vel / reference_y
            cmd_vel_msg.linear.x = cmd_vel_linear_sp * -1
            cmd_vel_msg.angular.z = cmd_vel_angular_sp
            cmd_vel_pub_line_follower.publish(cmd_vel_msg)
            time.sleep(0.05)

        #print("whiledan cikti")
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
    #TODO: burası değişmeli başlangıçta paletin nerede olduğunu bilmeliyiz
    args = sys.argv[1:]
    if(len(args) > 0):
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

    # If agv is in idle mode in begining, moves lift up or down
    if current_pos == IDLE:
        if lift_status_val != LIFT_UP:
            lift_status_msg.up_lift = True
            lift_status_msg.down_lift = False
            lift_start_val = LIFT_UP
        else:
            lift_status_msg.up_lift = False
            lift_status_msg.down_lift = True
            lift_start_val = LIFT_DOWN
        lift_status_pub.publish(lift_status_msg)

        while lift_status_val != lift_start_val:
            is_shutdown = True
            if (time.time() - lift_last_time) > 10:
                os.system("rosnode kill /moobot_lift")
                print("killed moobot_lift node")
                lift_last_time = time.time()

    is_shutdown = False
    change_scanner_publish_time = time.time()
    change_lift_status_time = time.time()
    emg_start_time = time.time()

    while is_shutdown == False:
        end_time = time.time()
        if (end_time - start_time) > 2:
            agv_job_status_pub.publish(agv_job_status)
            start_time = end_time

        if (time.time() - lift_last_time) > 10:
            os.system("rosnode kill /moobot_lift")
            print("killed moobot_lift node")
            lift_last_time = time.time()

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
                agv_job_status = "Logos goes to home station"
                print("idle")
                if goal_sended == False:
                    thread_send_station = Thread(
                        target=send_station, args=[HOME, current_pos], daemon=True)
                    thread_send_station.start()

            elif current_pos == HOME:
                time.sleep(2)
                print("Home a geldim.")
                agv_job_status = "Logos is in home station"
                if goal_sended == False:
                    if pallet_loaded: # if pallet is loaded to agv, agv goes to the empty station according to the load turn
                        if not pallet_loaded_two and load_turn == 2:
                            next_station = LIFT_STATION_BEFORE_TWO
                        elif not pallet_loaded_three and load_turn == 3:
                            next_station = LIFT_STATION_BEFORE_THREE
                        agv_job_status = "Logos goes to unload"
                    else: # if pallet is not loaded to agv, agv goes to the loaded station
                        if pallet_loaded_two:
                            next_station = LIFT_STATION_BEFORE_TWO
                        elif pallet_loaded_three:
                            next_station = LIFT_STATION_BEFORE_THREE
                        agv_job_status = "Logos goes to load"
                    thread_send_station = Thread(
                        target=send_station, args=[next_station, current_pos], daemon=True)
                    thread_send_station.start()

            elif current_pos == LIFT_STATION_BEFORE_TWO:
                if lane_detected == 0:
                    if pallet_loaded_two:
                        agv_job_status = "Logos goes to load in station two"
                        current_pos = MAKE_LIFT_DOWN
                    else:
                        agv_job_status = "Logos goes to unload in station two"
                        current_pos = MAKE_LIFT_UP
                else:  # If lane is not detected, go to middle station and try again
                    if goal_sended == False:
                        thread_send_station = Thread(
                            target=send_station, args=[MIDDLE_STATION, current_pos], daemon=True)
                        thread_send_station.start()

            elif current_pos == LIFT_STATION_BEFORE_THREE:
                if lane_detected == 0:
                    if pallet_loaded_three:
                        agv_job_status = "Logos goes to load in station three"
                        current_pos = MAKE_LIFT_DOWN
                    else:
                        agv_job_status = "Logos goes to unload in station three"
                        current_pos = MAKE_LIFT_UP
                else:  # If lane is not detected, go to middle station and try again
                    if goal_sended == False:
                        thread_send_station = Thread(
                            target=send_station, args=[MIDDLE_STATION, current_pos], daemon=True)
                        thread_send_station.start()

            elif current_pos == MIDDLE_STATION: # if lane is not detected agv should try the steps in home station again 
                current_pos = HOME
                time.sleep(1.0)


            elif current_pos == MAKE_LIFT_DOWN:
                if lift_status_val != LIFT_DOWN:
                    if time.time() - change_lift_status_time > 2:
                        lift_status_msg.up_lift = False
                        lift_status_msg.down_lift = True
                        lift_status_pub.publish(lift_status_msg)
                        print("lift status published")
                        change_lift_status_time = time.time()
                else:
                    time.sleep(1.0)
                    current_pos = CHANGE_AREA_LIFT

            elif current_pos == MAKE_LIFT_UP:
                if lift_status_val != LIFT_UP:
                    if time.time() - change_lift_status_time > 2:
                        lift_status_msg.up_lift = True
                        lift_status_msg.down_lift = False
                        lift_status_pub.publish(lift_status_msg)
                        print("lift status published")
                        change_lift_status_time = time.time()
                else:
                    time.sleep(1.0)
                    current_pos = CHANGE_AREA_LIFT

            elif current_pos == CHANGE_AREA_LIFT:
                if scanner_area != LIFT_SCANNER_AREA:
                    if time.time() - change_scanner_publish_time > 2:
                        scanner_area_change_pub.publish(LIFT_SCANNER_AREA)
                        print("lift scanner area published")
                        change_scanner_publish_time = time.time()
                else:
                    current_pos = IN_FOLLOW_LINE

            elif current_pos == IN_FOLLOW_LINE:
                if lane_detected == 0 and following_line == False:
                    time.sleep(2.0)
                    print("Follow line")
                    time.sleep(0.5)
                    if not pallet_loaded: # If pallet not loaded agv should go the station with pallet 
                        if pallet_loaded_two:
                            goal_pos_x = 0.0
                        elif pallet_loaded_three:
                            goal_pos_x = 1800.0
                    else: # If pallet loaded agv should go the station which is not loaded 
                        if not pallet_loaded_two and load_turn == 2:
                            goal_pos_x = 0.0
                        elif not pallet_loaded_three and load_turn == 3:
                            goal_pos_x = 1800.0
                    thread_follow_line_con_load = Thread(
                        target=follow_line, args=[140.0, 1.5, goal_pos_x])  # TODO: change offset
                    thread_follow_line_con_load.start()
                    following_line = True
                if following_line == True and follow_line_reached == True:
                    current_pos = LIFT_STATION
                    following_line = False
                    follow_line_reached = False

            elif current_pos == LIFT_STATION:
                print("In lift station")
                time.sleep(2.0)
                if not pallet_loaded:
                    current_pos = LOAD_PALLET
                else:
                    current_pos = UNLOAD_PALLET

            elif current_pos == LOAD_PALLET:
                if lift_status_val != LIFT_UP:
                    if time.time() - change_lift_status_time > 2:
                        lift_status_msg.up_lift = True
                        lift_status_msg.down_lift = False
                        lift_status_pub.publish(lift_status_msg)
                        print("lift status published")
                        change_lift_status_time = time.time()
                else:
                    current_pos = LOAD_PALLET_DONE

            elif current_pos == UNLOAD_PALLET:
                if lift_status_val != LIFT_DOWN:
                    if time.time() - change_lift_status_time > 2:
                        lift_status_msg.up_lift = False
                        lift_status_msg.down_lift = True
                        lift_status_pub.publish(lift_status_msg)
                        print("lift status published")
                        change_lift_status_time = time.time()
                else:
                    current_pos = UNLOAD_PALLET_DONE

            elif current_pos == LOAD_PALLET_DONE:
                agv_job_status = "Logos has finished loading"
                time.sleep(5.0)
                if goal_sended == False:
                    if pallet_loaded_two:
                        pallet_loaded = True
                        pallet_loaded_two = False
                        next_station = LIFT_STATION_OUT_TWO
                        load_turn = 3
                    elif pallet_loaded_three:
                        pallet_loaded = True
                        pallet_loaded_three = False
                        next_station = LIFT_STATION_OUT_THREE
                        load_turn = 2
                    thread_send_station = Thread(
                        target=send_station, args=[next_station, current_pos], daemon=True)
                    thread_send_station.start()

            elif current_pos == UNLOAD_PALLET_DONE:
                agv_job_status = "Logos has finished unloading"
                time.sleep(3.0)
                if goal_sended == False:
                    if load_turn == 2:
                        pallet_loaded = False
                        pallet_loaded_two = True
                        next_station = LIFT_STATION_OUT_TWO
                    elif load_turn == 3:
                        pallet_loaded = False
                        pallet_loaded_three = True
                        next_station = LIFT_STATION_OUT_THREE
                    thread_send_station = Thread(
                        target=send_station, args=[next_station, current_pos], daemon=True)
                    thread_send_station.start()

            elif current_pos == LIFT_STATION_OUT_THREE or current_pos == LIFT_STATION_OUT_TWO:
                agv_job_status = "Logos goes to home station"
                current_pos = CHANGE_AREA_DEFAULT

            elif current_pos == CHANGE_AREA_DEFAULT:
                if scanner_area != DEFAULT_SCANNER_AREA:
                    if time.time() - change_scanner_publish_time > 2:
                        scanner_area_change_pub.publish(DEFAULT_SCANNER_AREA)
                        print("scanner are default published")
                        change_scanner_publish_time = time.time()
                else:
                    current_pos = LIFT_STATION_DONE

            elif current_pos == LIFT_STATION_DONE:
                if goal_sended == False:
                    thread_send_station = Thread(
                        target=send_station, args=[HOME, current_pos], daemon=True)
                    thread_send_station.start()

            old_pos = current_pos

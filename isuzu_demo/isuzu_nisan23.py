#!/usr/bin/env python3
'''
ISUZU -> 2 istasyon arası yük alıp bırakma senaryosu (lift)
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

# STATIONS and states
HOME = 1
GIRIS_DOLLY_1 = 2 # HOME = 1 İLE AYNI
DOLLY_1 = 3      
CIKIS_DOLLY_1 = 4 # YOLDAKİ DÖNÜŞ NOKTASI
KAMYON_ALANI = 5
CIKIS_KAMYON = 6      
BOS_DEPO_ALANI = 7 #DOLLY_1 İLE AYNI 
CIKIS_BOS_DEPO = 8
GIRIS_KAMYON_ALANI_DOLLY2  = 9 
KAMYON_ALANI_DOLLY2  = 10     # KAMYON_ALANI İLE AYNI
SON_DEPO = 11 # DOLLY_1 İLE AYNI 
FINISH = 12 


MAKE_LIFT_UP = 24
MAKE_LIFT_DOWN = 25
MAKE_LIFT_UP_D1 = 26
MAKE_LIFT_DOWN_D1 = 27
MAKE_LIFT_UP_D2 = 28
MAKE_LIFT_DOWN_D2 = 29
MAKE_LIFT_UP_D3 = 30
MAKE_LIFT_DOWN_D3 = 31


GOING_HOME = 34
GOING_LIFT_STATION = 35

HOME_STATION_GO = 41
GIRIS_DOLLY_1_GO = 42
DOLLY_1_GO = 43
CIKIS_DOLLY_1_GO = 44
KAMYON_ALANI_GO = 45
CIKIS_KAMYON_GO = 46
BOS_DEPO_ALANI_GO = 47
CIKIS_BOS_DEPO_GO = 48
GIRIS_KAMYON_ALANI_DOLLY2_GO = 49
KAMYON_ALANI_DOLLY2_GO = 50
SON_DEPO_GO = 51



HOME_STATION_WAIT = 71
GIRIS_DOLLY_1_WAIT = 72
DOLLY_1_WAIT = 73
CIKIS_DOLLY_1_WAIT = 74
KAMYON_ALANI_WAIT = 75
CIKIS_KAMYON_WAIT = 76
BOS_DEPO_ALANI_WAIT = 77
CIKIS_BOS_DEPO_WAIT = 78
GIRIS_KAMYON_ALANI_DOLLY2_WAIT = 79
KAMYON_ALANI_DOLLY2_WAIT = 80
SON_DEPO_WAIT = 81



# orientations = [0.0, 0.0334188, 0.0334188, 0.0334188, None]


orientations = [-1.57,
                -1.57,
                -1.57,
                3.14,
                0.0,
                1.57,
                -1.57,
                3.14,
                0.0,
                0.0,
                1.57,
                1.57,
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
    "/home/rnd/moobot_ws/src/moobot_navigation/moobot_navigation3.rviz")
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
        new_station = Station(HOME,
                              station_x, station_y, station_z)
        stations_list.append(new_station)
    elif number == 2:
        new_station = Station(GIRIS_DOLLY_1,
                              station_x, station_y, station_z)
        stations_list.append(new_station)
    elif number == 3:
        new_station = Station(DOLLY_1,
                              station_x, station_y, station_z)
        stations_list.append(new_station)
    elif number == 4:
        new_station = Station(CIKIS_DOLLY_1,
                              station_x, station_y, station_z)
        stations_list.append(new_station)
    elif number == 5:
        new_station = Station(KAMYON_ALANI,
                              station_x, station_y, station_z)
        stations_list.append(new_station)
    elif number == 6:
        new_station = Station(CIKIS_KAMYON,
                              station_x, station_y, station_z)
        stations_list.append(new_station)
    elif number == 7:
        new_station = Station(BOS_DEPO_ALANI,
                              station_x, station_y, station_z)
        stations_list.append(new_station)
    elif number == 8:
        new_station = Station(CIKIS_BOS_DEPO,
                              station_x, station_y, station_z)
        stations_list.append(new_station)
    elif number == 9:
        new_station = Station(GIRIS_KAMYON_ALANI_DOLLY2,
                              station_x, station_y, station_z)
        stations_list.append(new_station)
    elif number == 10:
        new_station = Station(KAMYON_ALANI_DOLLY2,
                              station_x, station_y, station_z)
        stations_list.append(new_station)
    elif number == 11:
        new_station = Station(SON_DEPO,
                              station_x, station_y, station_z)
        stations_list.append(new_station)
    elif number == 12:
        new_station = Station(FINISH,
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
        time.sleep(5)
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
                print("Alan değişimi 0 --> 1 ")
                msg.data = 1
                scanner_area_change_pub.publish(msg)  # ALAN 1'E GEÇİŞ PAYLAŞILDI
                if goal_sended == False:
                    thread_send_station = Thread(
                        target=send_station, args=[HOME, current_pos], daemon=True)
                    thread_send_station.start()
# 1. İSTASYON (HOME)-> GİRİŞ DOLLY 1 İLE AYNI 
            elif current_pos == HOME:
                print("Senaryo başlangıç --> Footprint (--) - Scanner Area(normal) - Tolerance (0.05) ")
                footprint_command_global = """rosservice call /move_base/global_costmap/set_parameters \"config:
                                            strs:
                                            - {name: 'footprint', value: '[[-1.1,-0.6],[-1.1,0.6],[1.1,0.6],[1.1,-0.6]]
'}\""""
                os.system(footprint_command_global)
                footprint_command_local = """rosservice call /move_base/local_costmap/set_parameters \"config:
                                            strs:
                                            - {name: 'footprint', value: '[[-1.1,-0.6],[-1.1,0.6],[1.1,0.6],[1.1,-0.6]]
'}\""""
                os.system(footprint_command_local)
                scanner_normal_area_front = """rosservice call /front_laser_scanner/front_laser_scanner/set_parameters "config:
                            bools:
                            - {name: '', value: false}
                            ints:
                            - {name: '', value: 0}
                            strs:
                            - {name: '', value: ''}
                            doubles:
                            - {name: "angle_start", value: "-2.5"}
                            - {name: "angle_end", value: "2.3"}
                            groups:
                            - {name: '', state: false, id: 0, parent: 0}\""""
                os.system(scanner_normal_area_front)
                scanner_normal_area_rear = """rosservice call /rear_laser_scanner/rear_laser_scanner/set_parameters "config:
                                            bools:
                                            - {name: '', value: false}
                                            ints:
                                            - {name: '', value: 0}
                                            strs:
                                            - {name: '', value: ''}
                                            doubles:
                                            - {name: "angle_start", value: "-2.5"}
                                            - {name: "angle_end", value: "2.3"}
                                            groups:
                                            - {name: '', state: false, id: 0, parent: 0}\""""
                os.system(scanner_normal_area_rear)
                tolerance_hassas = """rosservice call /move_base/DWAPlannerROS/set_parameters \"config:
                                    bools:
                                    - {name: '', value: false}
                                    ints:
                                    - {name: '', value: 0}
                                    strs:
                                    - {name: '', value: ''}
                                    doubles:
                                    - {name: 'xy_goal_tolerance', value: 0.05}
                                    - {name: 'yaw_goal_tolerance', value: 0.1}
                                    groups:
                                    - {name: '', state: false, id: 0, parent: 0}\""""
                os.system(tolerance_hassas)
                time.sleep(0.5)
                if goal_sended == False:
                    current_pos = GIRIS_DOLLY_1_WAIT
# 2. İSTASYON - giriş1
            elif current_pos == GIRIS_DOLLY_1_WAIT:  # homeda bekliyor
                print("GIRIS_DOLLY_1'a gitme için 0 saniye bekleniyor...")
                time.sleep(0.0)
                current_pos = GIRIS_DOLLY_1_GO

            elif current_pos == GIRIS_DOLLY_1_GO:
                if goal_sended == False:
                    next_station = GIRIS_DOLLY_1
                    thread_send_station = Thread(
                        target=send_station, args=[next_station, current_pos], daemon=True)
                    thread_send_station.start()

            elif current_pos == GIRIS_DOLLY_1:
                print("GIRIS_DOLLY_1'a geldim. Alan değişimi 0-->1 ")
                msg.data = 1
                scanner_area_change_pub.publish(msg)  # ALAN 1'E GEÇİŞ PAYLAŞILDI
                if goal_sended == False:
                    current_pos = DOLLY_1_WAIT
# 3. İSTASYONA - dolly_load
            elif current_pos == DOLLY_1_WAIT:
                print("DOLLY_1'a girmek için 5 saniye bekleniyor...")
                time.sleep(1.0)
                current_pos = DOLLY_1_GO

            elif current_pos == DOLLY_1_GO: # 5 saniye boyunca 0.4 lineer hızla ilerlemesi
                start_time = time.time() 
                while (time.time() - start_time) < 5: 
                    cmd_vel_msg.angular.z = 0.0
                    cmd_vel_msg.linear.x = 0.4
                    cmd_vel_pub_line_follower.publish(cmd_vel_msg)
                cmd_vel_msg.angular.z = 0.0
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_pub_line_follower.publish(cmd_vel_msg)                     
                current_pos = DOLLY_1    

            elif current_pos == DOLLY_1:
                if goal_sended == False:
                    current_pos = MAKE_LIFT_UP_D1

            elif current_pos == MAKE_LIFT_UP_D1:
                # print("Lift Up")
                if lift_status_val != LIFT_UP:
                    time.sleep(1.5)
                    if time.time() - change_lift_status_time > 2:
                        lift_status_msg.up_lift = True
                        lift_status_msg.down_lift = False
                        lift_status_pub.publish(lift_status_msg)
                        print("lift status published")
                        change_lift_status_time = time.time()
                else:
                    time.sleep(1.0)
                    if goal_sended == False:
                        current_pos = CIKIS_DOLLY_1_WAIT

# 4. İSTASYONA - giriş (2 ile aynı)
            elif current_pos == CIKIS_DOLLY_1_WAIT:
                print("CIKIS_DOLLY_1'a gitme için 5 saniye bekleniyor... Footprint ++ & Scanner -- & Tolerance-> 1.0")
                scanner_narrow_area_front = """rosservice call /front_laser_scanner/front_laser_scanner/set_parameters "config:
                            bools:
                            - {name: '', value: false}
                            ints:
                            - {name: '', value: 0}
                            strs:
                            - {name: '', value: ''}
                            doubles:
                            - {name: "angle_start", value: "-0.5"}
                            - {name: "angle_end", value: "2.3"}
                            groups:
                            - {name: '', state: false, id: 0, parent: 0}\""""
                os.system(scanner_narrow_area_front)
                scanner_narrow_area_rear = """rosservice call /rear_laser_scanner/rear_laser_scanner/set_parameters "config:
                                            bools:
                                            - {name: '', value: false}
                                            ints:
                                            - {name: '', value: 0}
                                            strs:
                                            - {name: '', value: ''}
                                            doubles:
                                            - {name: "angle_start", value: "-0.5"}
                                            - {name: "angle_end", value: "2.3"}
                                            groups:
                                            - {name: '', state: false, id: 0, parent: 0}\""""
                os.system(scanner_narrow_area_rear)
                footprint_command_global = """rosservice call /move_base/global_costmap/set_parameters \"config:
                                            strs:
                                            - {name: 'footprint', value: '[[-1.1,-0.8],[-1.1,0.8],[1.1,0.8],[1.1,-0.8]]
'}\""""
                os.system(footprint_command_global)
                footprint_command_local = """rosservice call /move_base/local_costmap/set_parameters \"config:
                                            strs:
                                            - {name: 'footprint', value: '[[-1.1,-0.8],[-1.1,0.8],[1.1,0.8],[1.1,-0.8]]
'}\""""
                os.system(footprint_command_local)
                tolerance_hassas_olmayan = """rosservice call /move_base/DWAPlannerROS/set_parameters \"config:
                                            bools:
                                            - {name: '', value: false}
                                            ints:
                                            - {name: '', value: 0}
                                            strs:
                                            - {name: '', value: ''}
                                            doubles:
                                            - {name: 'xy_goal_tolerance', value: 0.1}
                                            - {name: 'yaw_goal_tolerance', value: 0.1}
                                            groups:
                                            - {name: '', state: false, id: 0, parent: 0}\""""
                os.system(tolerance_hassas_olmayan) 
                time.sleep(5.0)
                current_pos = CIKIS_DOLLY_1_GO

            elif current_pos == CIKIS_DOLLY_1_GO:
                if goal_sended == False:
                    next_station = CIKIS_DOLLY_1
                    thread_send_station = Thread(
                        target=send_station, args=[next_station, current_pos], daemon=True)
                    thread_send_station.start()

            elif current_pos == CIKIS_DOLLY_1:
                print("CIKIS_DOLLY_1'a geldim. ")
                if goal_sended == False:
                    current_pos = KAMYON_ALANI_WAIT

# 6. İSTASYONA - KAMYON ALANI DOLLY BIRAKMA ALANI
            elif current_pos == KAMYON_ALANI_WAIT:
                print("KAMYON_ALANI'a gitme için 1 saniye bekleniyor... Tolerance-> 0.05 ")
                tolerance_hassas = """rosservice call /move_base/DWAPlannerROS/set_parameters \"config:
                                    bools:
                                    - {name: '', value: false}
                                    ints:
                                    - {name: '', value: 0}
                                    strs:
                                    - {name: '', value: ''}
                                    doubles:
                                    - {name: 'xy_goal_tolerance', value: 0.05}
                                    - {name: 'yaw_goal_tolerance', value: 0.1}
                                    groups:
                                    - {name: '', state: false, id: 0, parent: 0}\""""
                os.system(tolerance_hassas) 
                time.sleep(0.5)
                current_pos = KAMYON_ALANI_GO

            elif current_pos == KAMYON_ALANI_GO:
                if goal_sended == False:
                    next_station = KAMYON_ALANI
                    thread_send_station = Thread(
                        target=send_station, args=[next_station, current_pos], daemon=True)
                    thread_send_station.start()

            elif current_pos == KAMYON_ALANI:
                print("KAMYON_ALANI'a geldim. Footprint -- ")
                footprint_command_global = """rosservice call /move_base/global_costmap/set_parameters \"config:
                                            strs:
                                            - {name: 'footprint', value: '[[-1.1,-0.6],[-1.1,0.6],[1.1,0.6],[1.1,-0.6]]'}\""""
                os.system(footprint_command_global)
                footprint_command_local = """rosservice call /move_base/local_costmap/set_parameters \"config:
                                            strs:
                                            - {name: 'footprint', value: '[[-1.1,-0.6],[-1.1,0.6],[1.1,0.6],[1.1,-0.6]]'}\""""
                os.system(footprint_command_local)
                if goal_sended == False:
                    current_pos = MAKE_LIFT_DOWN_D1

            elif current_pos == MAKE_LIFT_DOWN_D1:
                # print("Lift Down")
                if lift_status_val != LIFT_DOWN:
                    time.sleep(1.5)
                    if time.time() - change_lift_status_time > 2:
                        lift_status_msg.up_lift = False
                        lift_status_msg.down_lift = True
                        lift_status_pub.publish(lift_status_msg)
                        print("lift status published")
                        change_lift_status_time = time.time()
                else:
                    time.sleep(1.0)
                    if goal_sended == False:
                        current_pos = CIKIS_KAMYON_WAIT

# 7. İSTASYONA giriş2 (5 ile aynı)
            elif current_pos == CIKIS_KAMYON_WAIT:
                print("CIKIS_KAMYON'a gitme için 5 saniye bekleniyor... Scanner ++ & Tolerance-> 0.1")
                scanner_normal_area_front = """rosservice call /front_laser_scanner/front_laser_scanner/set_parameters "config:
                                            bools:
                                            - {name: '', value: false}
                                            ints:
                                            - {name: '', value: 0}
                                            strs:
                                            - {name: '', value: ''}
                                            doubles:
                                            - {name: "angle_start",
                                                value: "-2.25"}
                                            - {name: "angle_end", value: "2.25"}
                                            groups:
                                            - {name: '', state: false, id: 0, parent: 0}\""""
                os.system(scanner_normal_area_front)
                scanner_normal_area_rear = """rosservice call /rear_laser_scanner/rear_laser_scanner/set_parameters "config:
                                            bools:
                                            - {name: '', value: false}
                                            ints:
                                            - {name: '', value: 0}
                                            strs:
                                            - {name: '', value: ''}
                                            doubles:
                                            - {name: "angle_start",
                                                value: "-2.25"}
                                            - {name: "angle_end", value: "2.25"}
                                            groups:
                                            - {name: '', state: false, id: 0, parent: 0}\""""
                os.system(scanner_normal_area_rear)
                tolerance_hassas_olmayan = """rosservice call /move_base/DWAPlannerROS/set_parameters \"config:
                                    bools:
                                    - {name: '', value: false}
                                    ints:
                                    - {name: '', value: 0}
                                    strs:
                                    - {name: '', value: ''}
                                    doubles:
                                    - {name: 'xy_goal_tolerance', value: 0.1}
                                    - {name: 'yaw_goal_tolerance', value: 0.2}
                                    groups:
                                    - {name: '', state: false, id: 0, parent: 0}\""""
                os.system(tolerance_hassas_olmayan)         
                time.sleep(5.0)
                current_pos = CIKIS_KAMYON_GO

            elif current_pos == CIKIS_KAMYON_GO:
                if goal_sended == False:
                    next_station = CIKIS_KAMYON
                    thread_send_station = Thread(
                        target=send_station, args=[next_station, current_pos], daemon=True)
                    thread_send_station.start()

            elif current_pos == CIKIS_KAMYON:
                print("CIKIS_KAMYON'a geldim. Alan değişimi 1-->0 Tolerance-> 0.05")
                msg.data = 0
                scanner_area_change_pub.publish(msg)  # ALAN 0'E GEÇİŞ PAYLAŞILDI
                # tolerance_hassas = """rosservice call /move_base/DWAPlannerROS/set_parameters \"config:
                #                     bools:
                #                     - {name: '', value: false}
                #                     ints:
                #                     - {name: '', value: 0}
                #                     strs:
                #                     - {name: '', value: ''}
                #                     doubles:
                #                     - {name: 'xy_goal_tolerance', value: 0.05}
                #                     - {name: 'yaw_goal_tolerance', value: 0.05}
                #                     groups:
                #                     - {name: '', state: false, id: 0, parent: 0}\""""
                # os.system(tolerance_hassas)
                if goal_sended == False:
                    current_pos = BOS_DEPO_ALANI_WAIT

            elif current_pos == BOS_DEPO_ALANI_WAIT:
                print(" BOS_DEPO_ALANI'a gitme için 1 saniye bekleniyor...")
                time.sleep(0.5)
                current_pos = BOS_DEPO_ALANI_GO

            elif current_pos == BOS_DEPO_ALANI_GO:
                if goal_sended == False:
                    next_station = BOS_DEPO_ALANI
                    thread_send_station = Thread(
                        target=send_station, args=[next_station, current_pos], daemon=True)
                    thread_send_station.start()

            elif current_pos == BOS_DEPO_ALANI:
                print("BOS_DEPO_ALANI'na geldim. ")
                if goal_sended == False:
                    current_pos = CIKIS_BOS_DEPO_WAIT
#############################
# 4. İSTASYONA - giriş (2 ile aynı)
            elif current_pos == CIKIS_BOS_DEPO_WAIT:
                print("kamyon_dolly'a gitme için 5 saniye bekleniyor... Tolerance-> 1.0")
                tolerance_hassas_olmayan = """rosservice call /move_base/DWAPlannerROS/set_parameters \"config:
                                    bools:
                                    - {name: '', value: false}
                                    ints:
                                    - {name: '', value: 0}
                                    strs:
                                    - {name: '', value: ''}
                                    doubles:
                                    - {name: 'xy_goal_tolerance', value: 0.1}
                                    - {name: 'yaw_goal_tolerance', value: 0.1}
                                    groups:
                                    - {name: '', state: false, id: 0, parent: 0}\""""
                os.system(tolerance_hassas_olmayan)             
                time.sleep(5.0)
                current_pos = CIKIS_BOS_DEPO_GO

            elif current_pos == CIKIS_BOS_DEPO_GO:
                if goal_sended == False:
                    next_station = CIKIS_BOS_DEPO
                    thread_send_station = Thread(
                        target=send_station, args=[next_station, current_pos], daemon=True)
                    thread_send_station.start()

            elif current_pos == CIKIS_BOS_DEPO:
                print("CIKIS_BOS_DEPO_GO'a geldim. Tolerance-> 0.05")
                tolerance_hassas = """rosservice call /move_base/DWAPlannerROS/set_parameters \"config:
                                    bools:
                                    - {name: '', value: false}
                                    ints:
                                    - {name: '', value: 0}
                                    strs:
                                    - {name: '', value: ''}
                                    doubles:
                                    - {name: 'xy_goal_tolerance', value: 0.05}
                                    - {name: 'yaw_goal_tolerance', value: 0.05}
                                    groups:
                                    - {name: '', state: false, id: 0, parent: 0}\""""
                os.system(tolerance_hassas)         
                if goal_sended == False:
                    current_pos = GIRIS_KAMYON_ALANI_DOLLY2_WAIT

# 2. İSTASYON - giriş1
            elif current_pos == GIRIS_KAMYON_ALANI_DOLLY2_WAIT:
                print("KAMYON_ALANI_DOLLY2_GIRIS'a gitme için 0.5 saniye bekleniyor...")
                time.sleep(0.5)
                current_pos = GIRIS_KAMYON_ALANI_DOLLY2_GO

            elif current_pos == GIRIS_KAMYON_ALANI_DOLLY2_GO:
                if goal_sended == False:
                    next_station = GIRIS_KAMYON_ALANI_DOLLY2
                    thread_send_station = Thread(
                        target=send_station, args=[next_station, current_pos], daemon=True)
                    thread_send_station.start()

            elif current_pos == GIRIS_KAMYON_ALANI_DOLLY2:
                print("KAMYON_ALANI_DOLLY2_GIRIS'a geldim. Alan değişimi 0-->1 ")
                msg.data = 1
                scanner_area_change_pub.publish(msg)  # ALAN 1'E GEÇİŞ PAYLAŞILDI
                if goal_sended == False:
                    current_pos = KAMYON_ALANI_DOLLY2_WAIT
# 3. İSTASYONA - dolly_load
            elif current_pos == KAMYON_ALANI_DOLLY2_WAIT:
                print("DOLLY_1'a girmek için 5 saniye bekleniyor...")
                time.sleep(1.0)
                current_pos = KAMYON_ALANI_DOLLY2_GO

            elif current_pos == KAMYON_ALANI_DOLLY2_GO:
                start_time = time.time() 
                while (time.time() - start_time) < 5: 
                    cmd_vel_msg.angular.z = 0.0
                    cmd_vel_msg.linear.x = 0.4
                    cmd_vel_pub_line_follower.publish(cmd_vel_msg)
                cmd_vel_msg.angular.z = 0.0
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_pub_line_follower.publish(cmd_vel_msg)                     
                current_pos = KAMYON_ALANI_DOLLY2    

            elif current_pos == KAMYON_ALANI_DOLLY2:
                if goal_sended == False:
                    current_pos = MAKE_LIFT_UP_D2

            elif current_pos == MAKE_LIFT_UP_D2:
                # print("Lift Up")
                if lift_status_val != LIFT_UP:
                    time.sleep(1.5)
                    if time.time() - change_lift_status_time > 2:
                        lift_status_msg.up_lift = True
                        lift_status_msg.down_lift = False
                        lift_status_pub.publish(lift_status_msg)
                        print("lift status published")
                        change_lift_status_time = time.time()
                else:
                    time.sleep(1.0)
                    if goal_sended == False:
                        current_pos = SON_DEPO_WAIT

# 4. İSTASYONA - giriş (2 ile aynı)
            elif current_pos == SON_DEPO_WAIT:
                print("SON_DEPO'a gitme için 5 saniye bekleniyor... Footprint ++ & Scanner --  ")
                scanner_narrow_area_front = """rosservice call /front_laser_scanner/front_laser_scanner/set_parameters "config:
                            bools:
                            - {name: '', value: false}
                            ints:
                            - {name: '', value: 0}
                            strs:
                            - {name: '', value: ''}
                            doubles:
                            - {name: "angle_start", value: "-0.5"}
                            - {name: "angle_end", value: "2.3"}
                            groups:
                            - {name: '', state: false, id: 0, parent: 0}\""""
                os.system(scanner_narrow_area_front)
                scanner_narrow_area_rear = """rosservice call /rear_laser_scanner/rear_laser_scanner/set_parameters "config:
                                            bools:
                                            - {name: '', value: false}
                                            ints:
                                            - {name: '', value: 0}
                                            strs:
                                            - {name: '', value: ''}
                                            doubles:
                                            - {name: "angle_start", value: "-0.5"}
                                            - {name: "angle_end", value: "2.3"}
                                            groups:
                                            - {name: '', state: false, id: 0, parent: 0}\""""
                os.system(scanner_narrow_area_rear)
                footprint_command_global = """rosservice call /move_base/global_costmap/set_parameters \"config:
                                            strs:
                                            - {name: 'footprint', value: '[[-1.1,-0.8],[-1.1,0.8],[1.1,0.8],[1.1,-0.8]]'}\""""
                os.system(footprint_command_global)
                footprint_command_local = """rosservice call /move_base/local_costmap/set_parameters \"config:
                                            strs:
                                            - {name: 'footprint', value: '[[-1.1,-0.8],[-1.1,0.8],[1.1,0.8],[1.1,-0.8]]'}\""""
                os.system(footprint_command_local)
                time.sleep(5.0)
                current_pos = SON_DEPO_GO

            elif current_pos == SON_DEPO_GO:
                if goal_sended == False:
                    next_station = SON_DEPO
                    thread_send_station = Thread(
                        target=send_station, args=[next_station, current_pos], daemon=True)
                    thread_send_station.start()
#######################

            elif current_pos == SON_DEPO:
                print("SON_DEPO'a geldim. FOOTPRINT -- ") 
                footprint_command_global = """rosservice call /move_base/global_costmap/set_parameters \"config:
                                        strs:
                                        - {name: 'footprint', value: '[[-1.1,-0.6],[-1.1,0.6],[1.1,0.6],[1.1,-0.6]]
'}\""""
                os.system(footprint_command_global)
                footprint_command_local = """rosservice call /move_base/local_costmap/set_parameters \"config:
                                        strs:
                                        - {name: 'footprint', value: '[[-1.1,-0.6],[-1.1,0.6],[1.1,0.6],[1.1,-0.6]]
'}\""""
                os.system(footprint_command_local)                 
                if goal_sended == False:
                    current_pos = MAKE_LIFT_DOWN_D2

            elif current_pos == MAKE_LIFT_DOWN_D2:
                # print("Lift Down")
                if lift_status_val != LIFT_DOWN:
                    time.sleep(1.5)
                    if time.time() - change_lift_status_time > 2:
                        lift_status_msg.up_lift = False
                        lift_status_msg.down_lift = True
                        lift_status_pub.publish(lift_status_msg)
                        print("lift status published")
                        change_lift_status_time = time.time()
                else:
                    time.sleep(1.0)
                    if goal_sended == False:
                        current_pos = HOME
            old_pos = current_pos

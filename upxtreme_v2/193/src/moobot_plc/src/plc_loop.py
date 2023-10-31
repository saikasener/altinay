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
from std_msgs.msg import Bool
from moobot_pgv.msg import pgv_scan_data
import pickle
import os
from geometry_msgs.msg import Twist
from threading import Thread
import time


sys.path.insert(0, "..")

IDLE = 0
EMG = 100
FAULT = 101
# STATIONS
HOME = 1
ROBOT_LOAD = 2
CONVEYOR_LOAD = 3
CONVEYOR_UNLOAD = 4
CONVEYOR_UNLOAD_BEFORE = 5
CONVEYOR_LOAD_BEFORE = 6
ROBOT_AREA = 7
CONVEYOR_AREA = 8
CONVEYOR_LOAD_BEFORE_OUT = 9
CONVEYOR_UNLOAD_BEFORE_OUT = 10
CONVEYOR_AREA_OUT = 11
ROBOT_AREA_OUT = 12

GOING_HOME = 13
GOING_ROBOT_LOAD = 14
GOING_CONVEYOR_LOAD = 15
GOING_CONVEYOR_UNLOAD = 16
GOING_CONVEYOR_UNLOAD_BEFORE = 17
GOING_CONVEYOR_LOAD_BEFORE = 18
GOING_ROBOT_AREA = 19
GOING_CONVEYOR_AREA = 20
GOING_CONVEYOR_LOAD_BEFORE_OUT = 21
GOING_CONVEYOR_UNLOAD_BEFORE_OUT = 22
GOING_CONVEYOR_AREA_OUT = 23
GOING_ROBOT_AREA_OUT = 24

IN_TRANSFER_CONVEYOR_LOAD = 25
IN_TRANSFER_CONVEYOR_UNLOAD = 26
CONVEYOR_LOAD_DONE = 27
CONVEYOR_UNLOAD_DONE = 28


GOING = 999


orientations = [3.1415, 1.57, None, None, -3.12,
    3.3, 1.57, -3.08, -1.57, -2.08, -1.57, 3.1415]

agv_conveyor_x_pos = 0.0
agv_conveyor_y_pos = 0.0
agv_conveyor_orientation = 0.0
read_barcode = True


pgv_x_from_base = 58.7  # mm
pgv_y_from_base = 297  # mm
pgv_a_from_base = 302.7  # mm
pgv_theta_from_base = 78.8  # deg

PGV_home_tag_num = 4
PGV_robot_load_tag_num = 1

current_tag_num = 0
lane_detected = 1

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
    if number == 6:
        new_station = Station(
            CONVEYOR_AREA_OUT, station_x, station_y, station_z)
        stations_list.append(new_station)

        new_station = Station(CONVEYOR_LOAD_BEFORE_OUT,
                              station_x, station_y, station_z)
        stations_list.append(new_station)

    elif number == 5:
        new_station = Station(CONVEYOR_UNLOAD_BEFORE_OUT,
                              station_x, station_y, station_z)
        stations_list.append(new_station)
    elif number == 7:
        new_station = Station(ROBOT_AREA_OUT, station_x, station_y, station_z)
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


def send_goal(x_goal, y_goal, orientation, station_num):
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

    time.sleep(2)
    current_pos = current_pos + 12

    # TODO: Hedef değiştirilirse ne olacağını dene
    ret = move_base_client.wait_for_result()
    print(ret)

    if ret != False:
        goal_sended = False
        # Home ve robot load için barcode'ları check eder.
        if station_num == HOME:
            current_pos = HOME
            try:
                go_home_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
            except:
                pass
            # if current_tag_num == PGV_home_tag_num:
            #    current_pos = station_num
            #    go_home_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
            # else:
            #    current_pos = FAULT
        elif station_num == ROBOT_LOAD:
            if current_tag_num == PGV_robot_load_tag_num:
                x = int(agv_conveyor_x_pos) - 20
                y = int(agv_conveyor_y_pos) + 40  # TODO: buna bakkkkkkkkkk
                a = int(agv_conveyor_orientation)

                if x > 126 or y > 126 or a > 10 or read_barcode == False:
                    print("Location error for robot load")
                    current_pos = FAULT
                else:
                    current_pos = station_num
                    agv_x_pos_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(
                        ua.Variant(x, ua.VariantType.Int16)))
                    agv_y_pos_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(
                        ua.Variant(y, ua.VariantType.Int16)))
                    agv_a_pos_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(
                        ua.Variant(a, ua.VariantType.Int16)))
                    go_robot_load_send.set_attribute(
                        ua.AttributeIds.Value, ua.DataValue(True))
            else:
                current_pos = FAULT
        else:
            current_pos = station_num
        print(current_pos)


def send_station(goal_station):
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


def check_conveyor_done(Bool):
    global conveyor_transfer_done
    conveyor_transfer_done = Bool.data


def check_conveyor_loaded_button(Bool):
    global conveyor_loaded
    global get_button_time
    conveyor_loaded = Bool.data and pallet_loaded


def check_conveyor_pallet_loaded(Bool):
    global pallet_loaded
    global conveyor_loaded
    pallet_loaded = Bool.data

    if Bool.data == False:
        conveyor_loaded = False


conveyor_transfer_start = False
conveyor_running = False
conveyor_loaded = False
pallet_loaded = False


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

    # Calculate position of agv conveyor according to pgv data
    read_barcode = pgv_scan_data.read_barcode
    if pgv_scan_data.read_barcode == True:
        current_tag_num = pgv_scan_data.tag_num
        agv_conveyor_orientation = pgv_scan_data.orientation
        agv_conveyor_y_pos = pgv_scan_data.x_pos + pgv_a_from_base * \
            math.cos((agv_conveyor_orientation + pgv_theta_from_base) /
                     180 * math.pi) - pgv_x_from_base
        agv_conveyor_x_pos = pgv_scan_data.y_pos + pgv_a_from_base * \
            math.sin((agv_conveyor_orientation + pgv_theta_from_base) /
                     180 * math.pi) - pgv_y_from_base
    # TODO: burada aralığı check et
    lane_detected = pgv_scan_data.lane_detected
    if lane_detected == 0:
        current_pos_x = pgv_scan_data.x_pos
        current_pos_y = pgv_scan_data.y_pos
        current_pos_theta = pgv_scan_data.orientation


nodes = []

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

current_pos_x = 0.0
current_pos_y = 0.0
current_pos_theta = 0.0
follow_line_reached = False


# goal un her zaman 0,0 olduğunu varsayıyorum
def follow_line(goal_x_tolerance, goal_orientation):
    global cmd_vel_angular_sp
    global cmd_vel_linear_sp
    global follow_line_reached

    while current_pos_x > goal_x_tolerance:  # hız ver
        cmd_vel_linear_sp = current_pos_x * max_linear_vel / \
            reference_x * (1 - abs(current_pos_y)/reference_y)
        cmd_vel_angular_sp = current_pos_y * max_angular_vel / reference_y

        cmd_vel_msg.linear.x = cmd_vel_linear_sp * -1
        cmd_vel_msg.angular.z = cmd_vel_angular_sp
        cmd_vel_pub_line_follower.publish(cmd_vel_msg)
        time.sleep(0.05)

    print("whiledan cikti")
    while abs(current_pos_theta + goal_orientation) > goal_theta_tolerance:
        if (current_pos_theta + goal_orientation) < 0:
            cmd_vel_angular_sp = -1 * min_angular_vel
        else:
            cmd_vel_angular_sp = min_angular_vel
        cmd_vel_linear_sp = 0.0
        cmd_vel_msg.angular.z = cmd_vel_angular_sp
        cmd_vel_msg.linear.x = cmd_vel_linear_sp
        cmd_vel_pub_line_follower.publish(cmd_vel_msg)
        time.sleep(0.05)

    print("Durması lazım")
    follow_line_reached = True
    cmd_vel_linear_sp = 0.0
    cmd_vel_angular_sp = 0.0
    cmd_vel_msg.angular.z = cmd_vel_angular_sp
    cmd_vel_msg.linear.x = cmd_vel_linear_sp
    cmd_vel_pub_line_follower.publish(cmd_vel_msg)
    cmd_vel_pub_line_follower.publish(cmd_vel_msg)


def set_attribute_for_nodes(current_nodes, val):
    try:
        for node in nodes:
            node.set_attribute(ua.AttributeIds.Value, ua.DataValue(val))
    except:
        pass
        



if os.path.getsize('/home/rnd/moobot_ws/src/moobot_plc/src/current_pos.pickle') > 0:
    with open('/home/rnd/moobot_ws/src/moobot_plc/src/current_pos.pickle', 'rb') as f:
        current_pos = pickle.load(f)
else:
    current_pos = IDLE
current_pos = IDLE
print("Current pos", current_pos)
is_connected = False

while is_connected == False:
    try:
        # if anonymous authentication is enabled
        client = Client("opc.tcp://192.168.1.1:4840")
        print(client.application_uri)
        client.connect()

        # Nodes from AGV to PLC
        go_home_send = client.get_node(
            "ns=3; s= \"DB_Agv\".\"AGV_To_PLC\".\"Home_Pos\"")
        go_robot_load_send = client.get_node(
            "ns=3; s= \"DB_Agv\".\"AGV_To_PLC\".\"Robot_Load_Pos\"")
        go_conveyor_load_send = client.get_node(
            "ns=3; s= \"DB_Agv\".\"AGV_To_PLC\".\"Conveyor_Load_Pos\"")
        go_conveyor_un_load_send = client.get_node(
            "ns=3; s= \"DB_Agv\".\"AGV_To_PLC\".\"Conveyor_Un_Load_Pos\"")
        transfer_start_send = client.get_node(
            "ns=3; s= \"DB_Agv\".\"AGV_To_PLC\".\"Transfer_Start\"")
        fault_send = client.get_node(
            "ns=3; s= \"DB_Agv\".\"AGV_To_PLC\".\"Fault\"")
        ready_send = client.get_node(
            "ns=3; s= \"DB_Agv\".\"AGV_To_PLC\".\"Ready\"")  # şu an kullanmıyor
        emg = client.get_node("ns=3; s= \"DB_Agv\".\"PLC_To_AGV\".\"Emg\"")
        emg_send = client.get_node(
            "ns=3; s= \"DB_Agv\".\"AGV_To_PLC\".\"Emg\"")
        zone_info_send = client.get_node(
            "ns=3; s= \"DB_Agv\".\"AGV_To_PLC\".\"Warning_Laser_Scanner\"")  # TODO
        out_from_conveyor_load_send = client.get_node(
            "ns=3; s= \"DB_Agv\".\"AGV_To_PLC\".\"Out_of_interference_to_Conveyor\"")  # TODO: söylenmesi lazım
        out_from_robot_load_send = client.get_node(
            "ns=3; s= \"DB_Agv\".\"AGV_To_PLC\".\"Out_of_interference_to_Robot\"")  # TODO: bunun yapılması lazım
        agv_x_pos_send = client.get_node(
            "ns=3; s= \"DB_Agv\".\"AGV_To_PLC\".\"Agv_offset_x\"")
        agv_y_pos_send = client.get_node(
            "ns=3; s= \"DB_Agv\".\"AGV_To_PLC\".\"Agv_offset_y\"")
        agv_a_pos_send = client.get_node(
            "ns=3; s= \"DB_Agv\".\"AGV_To_PLC\".\"Agv_offset_a\"")
        conveyor_load_done_send = client.get_node(
            "ns=3; s= \"DB_Agv\".\"AGV_To_PLC\".\"Installation_completed\"")
        life_bit_send = client.get_node(
            "ns=3; s= \"DB_Agv\".\"AGV_To_PLC\".\"Life_bit\"")

        # Nodes from PLC to AGV
        go_home = client.get_node(
            "ns=3; s= \"DB_Agv\".\"PLC_To_AGV\".\"Go_Home_Pos\"")
        go_robot_load = client.get_node(
            "ns=3; s= \"DB_Agv\".\"PLC_To_AGV\".\"Consent from_Robot_Load_Pos\"")  # SAMIR
        go_conveyor_load = client.get_node(
            "ns=3; s= \"DB_Agv\".\"PLC_To_AGV\".\"Consent from_Conveyor_Load_Pos\"")  # SAMIR
        go_conveyor_un_load = client.get_node(
            "ns=3; s= \"DB_Agv\".\"PLC_To_AGV\".\"Consent from_Conveyor_Un_Load_Pos\"")  # SAMIR
        transfer_start = client.get_node(
            "ns=3; s= \"DB_Agv\".\"PLC_To_AGV\".\"Trasnfer_Start\"")
        fault_reset = client.get_node(
            "ns=3; s= \"DB_Agv\".\"PLC_To_AGV\".\"Fault_Reset\"")
        emg = client.get_node(
            "ns=3; s= \"DB_Agv\".\"PLC_To_AGV\".\"Emg_Reset\"")  # Acili resetle
        out_from_conveyor_load_ok = client.get_node(
            "ns=3; s=\"DB_Agv\".\"PLC_To_AGV\".\"Consent from_Out_Conveyor_Load_Pos\"")  # SAMIR
        out_from_conveyor_un_load_ok = client.get_node(
            "ns=3; s=\"DB_Agv\".\"PLC_To_AGV\".\"Consent from_Out_Conveyor_Un_Load_Pos\"")  # SAMIR
        out_from_robot_load_ok = client.get_node(
            "ns=3; s=\"DB_Agv\".\"PLC_To_AGV\".\"Consent from_Out_Robot_Load_Pos\"")  # SAMIR

        # Append nodes to nodes list
        nodes.append(go_home)
        nodes.append(go_robot_load)
        nodes.append(go_conveyor_load)
        nodes.append(go_conveyor_un_load)
        nodes.append(out_from_robot_load_ok)
        nodes.append(out_from_conveyor_load_ok)
        nodes.append(out_from_conveyor_un_load_ok)
        nodes.append(emg)
        nodes.append(transfer_start)

        is_connected = True
        print("PLC connection established")
    except:
        is_connected = False
        # client.disconnect()
        print("Connection could not establish, retrying in 3 seconds...")
        time.sleep(3)


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
    pgv_scan_data_sub = rospy.Subscriber(
        "/pgv_scan", pgv_scan_data, calculate_agv_conveyor_pos)
    cancel_msg = GoalID()
    moobot_line_follower_pub = rospy.Publisher(
        "/moobot_follow_line_mode", Bool, queue_size=2)
    conveyor_pub = rospy.Publisher(
        "/conveyor_status", conveyor_status, queue_size=10)
    conveyor_done_sub = rospy.Subscriber(
        "/conveyor_transfer_done", Bool, check_conveyor_done)
    conveyor_loaded_sub = rospy.Subscriber(
        "/conveyor_loaded", Bool, check_conveyor_loaded_button)
    pallet_loaded_sub = rospy.Subscriber(
        "/conveyor_pallet_loaded", Bool, check_conveyor_pallet_loaded)
    conveyor_status_msg = conveyor_status()
    c_plc_emergency_pub = rospy.Publisher(
        "/controller_plc_emergency", Bool, queue_size=2)
    time.sleep(2)

    # if starts in IDLE pos set all nodes to false
    if current_pos == IDLE: 
        go_home_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(False))
        go_robot_load_send.set_attribute(
            ua.AttributeIds.Value, ua.DataValue(False))
        go_conveyor_un_load_send.set_attribute(
            ua.AttributeIds.Value, ua.DataValue(False))
        go_conveyor_load_send.set_attribute(
            ua.AttributeIds.Value, ua.DataValue(False))
        transfer_start_send.set_attribute(
            ua.AttributeIds.Value, ua.DataValue(False))
        conveyor_load_done_send.set_attribute(
            ua.AttributeIds.Value, ua.DataValue(False))

        #set_attribute_for_nodes([go_home_send, go_robot_load_send, go_conveyor_un_load_send, go_conveyor_load_send, transfer_start_send, conveyor_load_done_send], False)
        
    # if agv is stopped while going a station send the old goal
    elif current_pos >= GOING_HOME and current_pos <= GOING_ROBOT_AREA_OUT:
        thread_send_station=Thread(
            target = send_station, args = [current_pos-12])
        thread_send_station.start()

    while is_shutdown == False:
        try:
            values=client.get_values(nodes)
            go_home_val=values[0]
            go_robot_load_val=values[1]
            go_conveyor_load_val=values[2]
            go_conveyor_un_load_val=values[3]
            out_from_robot_load_ok_val=values[4]
            out_from_conveyor_load_ok_val=values[5]
            out_from_conveyor_un_load_ok_val=values[6]
            emg_val=values[7]
            transfer_start_val=values[8]
            time_t=time.time()
        except:
            pass
        if (time_t - old_time_t > 1.0):
            old_time_t=time_t
            live_bit=not live_bit
            #set_attribute_for_nodes([life_bit_send], live_bit)

        if emg_val == False:  # Emergency durumu yokken 1 gelir
            c_plc_emergency_pub.publish(True)  # TODO: in s-plc node
            current_pos=EMG
        if current_pos == EMG:
            print("Emergency")
            emg_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
            cmd_vel_msg.angular.z=0.0
            cmd_vel_msg.linear.x=0.0
            cmd_vel_pub_line_follower.publish(cmd_vel_msg)
            cancel_pub.publish(cancel_msg)

        elif current_pos == FAULT:
            # send fault
            fault_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
        else:
            #emg_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(False))
            if current_pos == IDLE:
                if goal_sended == False:
                    if go_home_val:
                        thread_send_station=Thread(
                            target = send_station, args = [HOME])
                        thread_send_station.start()
                    elif go_conveyor_load_val:
                        thread_send_station=Thread(
                            target = send_station, args = [CONVEYOR_LOAD_BEFORE])
                        thread_send_station.start()
                    elif go_robot_load_val:
                        thread_send_station=Thread(
                            target = send_station, args = [ROBOT_AREA])
                        thread_send_station.start()
                    elif go_conveyor_un_load_val:
                        thread_send_station=Thread(
                            target = send_station, args = [CONVEYOR_AREA_OUT])
                        thread_send_station.start()

            elif current_pos == HOME:
                # Before going anywhere checks load by operator

                if conveyor_loaded:
                    print("Conveyor loaded")
                    conveyor_load_done_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
                    # load_time = time.time()

                if goal_sended == False:  # butona bastıktan sonra iki saniye geçtiyse
                    if go_conveyor_un_load_val:
                        conveyor_load_done_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(False))
                        conveyor_loaded=False
                        thread_send_station=Thread(
                            target = send_station, args = [CONVEYOR_AREA_OUT])
                        thread_send_station.start()

                    elif go_conveyor_load_val:
                        conveyor_load_done_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(False))
                        conveyor_loaded=False
                        thread_send_station=Thread(
                            target = send_station, args = [CONVEYOR_LOAD_BEFORE])
                        thread_send_station.start()

                    elif go_robot_load_val:
                        conveyor_load_done_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(False))
                        conveyor_loaded=False
                        thread_send_station=Thread(
                            target = send_station, args = [ROBOT_AREA])
                        thread_send_station.start()

            elif current_pos == CONVEYOR_LOAD_BEFORE:
                if go_conveyor_load_val:
                    if lane_detected == 0 and following_line == False:
                        print("Follow line")
                        time.sleep(0.5)
                        thread_follow_line_con_load=Thread(
                            target = follow_line, args = [87.0, 1.5])
                        thread_follow_line_con_load.start()
                        following_line=True
                    if following_line == True and follow_line_reached == True:
                        current_pos=CONVEYOR_LOAD
                        following_line=False
                        follow_line_reached=False

                if goal_sended == False:
                    if go_conveyor_un_load_val:
                        thread_send_station=Thread(
                            target = send_station, args = [CONVEYOR_AREA_OUT])
                        thread_send_station.start()

                    elif go_robot_load_val:
                        thread_send_station=Thread(
                            target = send_station, args = [ROBOT_AREA])
                        thread_send_station.start()

                    elif go_home_val:
                        thread_send_station=Thread(
                            target = send_station, args = [HOME])
                        thread_send_station.start()

            elif current_pos == CONVEYOR_LOAD:
                # transfer_start_val = transfer_start.get_value()
                go_conveyor_load_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
                if transfer_start_val == True:
                    current_pos=IN_TRANSFER_CONVEYOR_LOAD
                    transfer_start_val=False

            elif current_pos == CONVEYOR_LOAD_BEFORE_OUT:
                if goal_sended == False:
                    if go_conveyor_un_load_val:
                        thread_send_station=Thread(
                            target = send_station, args = [CONVEYOR_AREA])
                        thread_send_station.start()

                    elif go_conveyor_load_val:
                        thread_send_station=Thread(
                            target = send_station, args = [CONVEYOR_LOAD_BEFORE])
                        thread_send_station.start()

                    elif go_robot_load_val:
                        thread_send_station=Thread(
                            target = send_station, args = [ROBOT_AREA])
                        thread_send_station.start()

                    elif go_home_val:
                        thread_send_station=Thread(
                            target = send_station, args = [HOME])
                        thread_send_station.start()

            elif current_pos == CONVEYOR_AREA:
                if goal_sended == False:
                    if go_conveyor_un_load_val:
                        thread_send_station=Thread(target = send_station, args = [
                                                   CONVEYOR_UNLOAD_BEFORE])
                        thread_send_station.start()

                    elif go_conveyor_load_val or go_robot_load_val or go_home_val:
                        thread_send_station=Thread(target = send_station, args = [
                                                   CONVEYOR_UNLOAD_BEFORE_OUT])
                        thread_send_station.start()

            elif current_pos == CONVEYOR_UNLOAD_BEFORE:
                if go_conveyor_un_load_val:
                    if lane_detected == 0 and following_line == False:
                        print("Follow line")
                        time.sleep(0.5)
                        thread_follow_line_con_un_load=Thread(
                            target = follow_line, args = [143.0, 0.0])
                        thread_follow_line_con_un_load.start()
                        following_line=True
                    if following_line == True and follow_line_reached == True:
                        current_pos=CONVEYOR_UNLOAD
                        following_line=False
                        follow_line_reached=False

                elif go_conveyor_load_val or go_robot_load_val or go_home_val:
                    if goal_sended == False:
                        thread_send_station=Thread(
                            target = send_station, args = [CONVEYOR_AREA_OUT])
                        thread_send_station.start()

            elif current_pos == CONVEYOR_UNLOAD:
                # transfer_start_val = transfer_start.get_value()
                go_conveyor_un_load_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
                if transfer_start_val == True:
                    current_pos=IN_TRANSFER_CONVEYOR_UNLOAD
                    transfer_start_val=False

            elif current_pos == CONVEYOR_UNLOAD_BEFORE_OUT:
                if goal_sended == False:
                    if go_conveyor_un_load_val:
                        thread_send_station=Thread(target = send_station, args = [
                                                     CONVEYOR_UNLOAD_BEFORE])
                        thread_send_station.start()

                    elif go_conveyor_load_val or go_robot_load_val:
                        thread_send_station=Thread(
                            target = send_station, args = [CONVEYOR_AREA_OUT])
                        thread_send_station.start()

            elif current_pos == CONVEYOR_AREA_OUT:
                if goal_sended == False:
                    if go_robot_load_val:
                        thread_send_station=Thread(
                            target = send_station, args = [ROBOT_AREA])
                        thread_send_station.start()

                    elif go_conveyor_un_load_val:
                        thread_send_station=Thread(
                            target = send_station, args = [CONVEYOR_AREA])
                        thread_send_station.start()

                    elif go_conveyor_load_val:
                        thread_send_station=Thread(
                            target = send_station, args = [CONVEYOR_LOAD_BEFORE])
                        thread_send_station.start()

            elif current_pos == ROBOT_AREA:
                if goal_sended == False:
                    if go_robot_load_val:
                        thread_send_station=Thread(
                            target = send_station, args = [ROBOT_LOAD])
                        thread_send_station.start()

                    elif go_conveyor_un_load_val:
                        thread_send_station=Thread(
                            target = send_station, args = [CONVEYOR_AREA_OUT])
                        thread_send_station.start()

                    elif go_conveyor_load_val:
                        thread_send_station=Thread(
                            target = send_station, args = [CONVEYOR_LOAD_BEFORE])
                        thread_send_station.start()

                    elif go_home_val:
                        thread_send_station=Thread(
                            target = send_station, args = [HOME])
                        thread_send_station.start()

            elif current_pos == ROBOT_LOAD:
                # out_from_robot_load_ok_val = out_from_robot_load_ok.get_value()
                if out_from_robot_load_ok_val:
                    if goal_sended == False:
                        if go_conveyor_un_load_val or go_conveyor_load_val or go_home_val:
                            thread_send_station=Thread(
                                target = send_station, args = [ROBOT_AREA_OUT])
                            thread_send_station.start()

            elif current_pos == ROBOT_AREA_OUT:
                if goal_sended == False:
                    if go_robot_load_val:
                        thread_send_station=Thread(
                            target = send_station, args = [ROBOT_AREA])
                        thread_send_station.start()

                    elif go_conveyor_load_val:
                        thread_send_station=Thread(
                            target = send_station, args = [CONVEYOR_LOAD_BEFORE])
                        thread_send_station.start()

                    elif go_conveyor_un_load_val:
                        thread_send_station=Thread(
                            target = send_station, args = [CONVEYOR_AREA_OUT])
                        thread_send_station.start()

                    elif go_home_val:
                        thread_send_station=Thread(
                            target = send_station, args = [HOME])
                        thread_send_station.start()

            elif current_pos == CONVEYOR_LOAD_DONE:
                if out_from_conveyor_load_ok_val and goal_sended == False:
                    thread_send_station=Thread(target = send_station, args = [
                                                 CONVEYOR_LOAD_BEFORE_OUT])
                    thread_send_station.start()

            elif current_pos == CONVEYOR_UNLOAD_DONE:
                if out_from_conveyor_un_load_ok_val and goal_sended == False:
                    thread_send_station=Thread(target = send_station, args = [
                                                 CONVEYOR_UNLOAD_BEFORE_OUT])
                    thread_send_station.start()

            elif current_pos == IN_TRANSFER_CONVEYOR_UNLOAD:
                if conveyor_running == False:
                    transfer_start_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
                    conveyor_status_msg.load_conveyor=True
                    conveyor_status_msg.unload_conveyor=False
                    conveyor_pub.publish(conveyor_status_msg)
                    print("Conveyor is working for loading to AGV")
                    conveyor_running=True
                    conveyor_transfer_done=False

                if conveyor_transfer_done == True:
                    print("Transfer done")
                    transfer_start_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(False))
                    conveyor_transfer_done=False
                    conveyor_running=False
                    conveyor_status_msg.load_conveyor=False
                    conveyor_status_msg.unload_conveyor=False
                    conveyor_pub.publish(conveyor_status_msg)
                    current_pos=CONVEYOR_UNLOAD_DONE

            elif current_pos == IN_TRANSFER_CONVEYOR_LOAD:
                # conveyorü çalıştır
                if conveyor_running == False:
                    transfer_start_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
                    conveyor_status_msg.unload_conveyor=True
                    conveyor_status_msg.load_conveyor=False
                    conveyor_pub.publish(conveyor_status_msg)
                    print("Conveyor is working for unloading from AGV")
                    conveyor_running=True
                    conveyor_transfer_done=False

                # conveyor load tamamlandıysa
                if conveyor_transfer_done == True:
                    print("Transfer done")
                    transfer_start_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(False))
                    conveyor_transfer_done=False
                    conveyor_running=False
                    conveyor_status_msg.load_conveyor=False
                    conveyor_status_msg.unload_conveyor=False
                    conveyor_pub.publish(conveyor_status_msg)
                    current_pos=CONVEYOR_LOAD_DONE

            elif current_pos >= GOING_HOME and current_pos <= GOING_ROBOT_AREA_OUT:
                go_home_send.set_attribute(
                    ua.AttributeIds.Value, ua.DataValue(False))
                go_robot_load_send.set_attribute(
                    ua.AttributeIds.Value, ua.DataValue(False))
                go_conveyor_un_load_send.set_attribute(
                    ua.AttributeIds.Value, ua.DataValue(False))
                go_conveyor_load_send.set_attribute(
                    ua.AttributeIds.Value, ua.DataValue(False))
                conveyor_load_done_send.set_attribute(
                    ua.AttributeIds.Value, ua.DataValue(False))

            old_pos=current_pos

        with open('/home/rnd/moobot_ws/src/moobot_plc/src/current_pos.pickle', 'wb') as f:
            pickle.dump(current_pos, f)

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
from moobot_msgs.msg import moobot_scanner, moobot_status, conveyor_status, plc_data
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
                3.3, 1.57, -3.08, -1.57, -1.65, -1.57, 3.1415]

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


is_shutdown = False

sys.path.insert(0, "..")


cmd_vel_pub_line_follower = rospy.Publisher("/cmd_vel", Twist, queue_size=2)
cmd_vel_msg = Twist()


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


class Pallet:
    def __init__(self, pallet_loaded, box_num):
        self.pallet_loaded = pallet_loaded
        self.box_num = box_num


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

# AGV poses
current_pos = IDLE
old_pos = IDLE
source_pos = IDLE


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
    goal_sended = True
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
    current_pos = current_pos + 12

    # TODO: Hedef değiştirilirse ne olacağını dene
    success = move_base_client.wait_for_result()
    state = move_base_client.get_state()
    print(success, state)

    if success:
        if state == 3:
            # Home ve robot load için barcode'ları check eder.
            if station_num == HOME:
                current_pos = HOME
                try:
                    go_home_send.set_attribute(
                        ua.AttributeIds.Value, ua.DataValue(True))
                    print("Set attribute home  geldim", True)
                except:
                    pass
                # if current_tag_num == PGV_home_tag_num:
                #    current_pos = station_num
                #    go_home_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
                # else:
                #    current_pos = FAULT
            elif station_num == ROBOT_LOAD:
                if current_tag_num == PGV_robot_load_tag_num:
                    x = round(agv_conveyor_x_pos) - 20
                    y = round(agv_conveyor_y_pos) + 40  # TODO: buna bakkkkkkkkkk
                    print(agv_conveyor_orientation)
                    a = round(agv_conveyor_orientation)

                    if x > 126 or y > 126 or a > 10 or read_barcode == False:
                        print("Location error for robot load")
                        current_pos = FAULT
                    else:
                        current_pos = station_num
                        try:
                            agv_x_pos_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(
                                ua.Variant(x, ua.VariantType.Int16)))
                            agv_y_pos_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(
                                ua.Variant(y, ua.VariantType.Int16)))
                            agv_a_pos_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(
                                ua.Variant(a, ua.VariantType.Int16)))
                            go_robot_load_send.set_attribute(
                                ua.AttributeIds.Value, ua.DataValue(True))
                            print("Set attribute robot load geldim", True)
                            print("Set pgv pos x:", x, "y: ", y, "a:", a)
                        except:
                            pass
                else:
                    current_pos = FAULT
            else:
                current_pos = station_num
            print(current_pos)
            goal_sended = False
        else:
            #TODO: recovery lazım şu anlık clear yapıyoruz
            os.system("rosservice call /move_base/clear_costmaps \"{}\"")
            time.sleep(1)
            goal_sended = False
            current_pos = goal_pos

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
    print("Emergency ", moobot_status.emergency)
    if moobot_status.emergency == True:
        current_pos = EMG
    else:
        if current_pos == EMG or current_pos == FAULT:  # emg den çıktıysa eski pos a dönmesi için
            current_pos = old_pos


conveyor_transfer_done = False
scanner_area = 0
ossd_active = False
conveyor_loaded_false = False


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

    if current_pos != EMG:
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


# şu an kullanılmıyor
def set_attribute_for_nodes(current_nodes, val):
    try:
        for node in nodes:
            node.set_attribute(ua.AttributeIds.Value, ua.DataValue(val))
    except:
        pass


if os.path.getsize('/home/rnd/moobot_ws/src/moobot_plc/src/current_pos.pickle') > 0:
    with open('/home/rnd/moobot_ws/src/moobot_plc/src/current_pos.pickle', 'rb') as f:
        current_pos = pickle.load(f)
        print("Current pos pickle", current_pos)
else:
    current_pos = IDLE

if os.path.getsize('/home/rnd/moobot_ws/src/moobot_plc/src/pallet_info.pickle') > 0:
    with open('/home/rnd/moobot_ws/src/moobot_plc/src/pallet_info.pickle', 'rb') as f:
        pallet_info = pickle.load(f)
        print("Pallet info pickle", pallet_info)
else:
    pallet_info = Pallet(False, 0)

pallet_info = Pallet(True, 0)
current_pos = ROBOT_AREA
print("Current pos", current_pos)
print("Pallet info\nPallet loaded:", pallet_info.pallet_loaded,
      "box num:", pallet_info.box_num)

is_connected = False

while is_connected == False:
    try:
        # if anonymous authentication is enabled
        print(1)
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
        emg_send = client.get_node(
            "ns=3; s= \"DB_Agv\".\"AGV_To_PLC\".\"Emg\"")
        zone_info_send = client.get_node(
            "ns=3; s= \"DB_Agv\".\"AGV_To_PLC\".\"Warning_Laser_Scaner\"")  # TODO
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



def save_pallet(pallet_loaded, box_num):
    global pallet_info
    pallet_info.pallet_loaded = pallet_loaded
    pallet_info.box_num = box_num


if __name__ == "__main__":
    rospy.init_node("controller_plc")

    # moobot_status subscriber ı olacak emg gelirse buna da emg yollayacak veya fault yollayacak
    moobot_status_sub_c_plc = rospy.Subscriber(
        "/agv_status", moobot_status, check_status)
    move_base_client = actionlib.SimpleActionClient(
        'move_base', MoveBaseAction)
    #cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
    pgv_scan_data_sub = rospy.Subscriber(
        "/pgv_scan", pgv_scan_data, calculate_agv_conveyor_pos)
    cancel_msg = GoalID()

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
    scannner_area_sub = rospy.Subscriber(
        '/scanner_area', Int16, check_scanner_area)
    scanner_area_change_pub = rospy.Publisher(
        "/change_scanner_area", Int16, queue_size=2)
    scanner_status_sub = rospy.Subscriber(
        "/scanner_status", moobot_scanner, check_scanner_status)
    plc_data_pub = rospy.Publisher("/plc_data", plc_data, queue_size=100)
    plc_data_msg = plc_data()
    time.sleep(2)
    check = 0
    while check == 0:  # checks whether start correctly or not
        try:
            fault_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
            zone_info_send.set_attribute(
                ua.AttributeIds.Value, ua.DataValue(True))
            if current_pos == IDLE:  # if starts in IDLE pos set all nodes to false
                go_home_send.set_attribute(
                    ua.AttributeIds.Value, ua.DataValue(False))
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
                out_from_robot_load_send.set_attribute(
                    ua.AttributeIds.Value, ua.DataValue(True))
                fault_send.set_attribute(
                    ua.AttributeIds.Value, ua.DataValue(True))  # fault yokken true
                emg_send.set_attribute(
                    ua.AttributeIds.Value, ua.DataValue(True))  # emg yokken true
                out_from_conveyor_load_send.set_attribute(
                       ua.AttributeIds.Value, ua.DataValue(True))
                
                

                #set_attribute_for_nodes([go_home_send, go_robot_load_send, go_conveyor_un_load_send, go_conveyor_load_send, transfer_start_send, conveyor_load_done_send], False)

            # if agv is stopped while going a station send the old goal
            elif current_pos >= GOING_HOME and current_pos <= GOING_ROBOT_AREA_OUT:
                thread_send_station = Thread(
                    target=send_station, args=[current_pos-12])
                thread_send_station.start()
            check = 1
        except:
            check = 0
    #go_robot_load_send.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))

    while is_shutdown == False:
        try:
            values = client.get_values(nodes)
            go_home_val = values[0]
            go_robot_load_val = values[1]
            go_conveyor_load_val = values[2]
            go_conveyor_un_load_val = values[3]
            out_from_robot_load_ok_val = values[4]
            out_from_conveyor_load_ok_val = values[5]
            out_from_conveyor_un_load_ok_val = values[6]
            emg_val = values[7]
            transfer_start_val = values[8]
            time_t = time.time()
            if len(values) != 9:
                print("Nodelar alınamadı!!!!")
            plc_data_msg.home = values[0]
            plc_data_msg.robot_load = values[1]
            plc_data_msg.conv_load = values[2]
            plc_data_msg.conv_unload = values[3]
            plc_data_msg.out_robot_load = values[4]
            plc_data_msg.out_conv_load = values[5]
            plc_data_msg.out_conv_unload = values[6]
            plc_data_msg.emg_val = values[7]
            plc_data_msg.transfer_start = values[8]
            plc_data_pub.publish(plc_data_msg)
            #rospy.loginfo("Home: %d, robot load: %d, conveyor load: %d, conveyor unload: %d, out from robot load: %d, out form conveyor load: %d, out from conveyor unload: %d, emg val: %d, transfer start %d", values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8])
        except:
            pass
        """if (time_t - old_time_t > 1.0):
            old_time_t = time_t
            live_bit = not live_bit
            #set_attribute_for_nodes([life_bit_send], live_bit)"""

        if emg_val == False:  # Emergency durumu yokken 1 gelir
            c_plc_emergency_pub.publish(True)  # TODO: in s-plc node
            current_pos = EMG

        if current_pos == EMG:
            try:
                emg_send.set_attribute(
                    ua.AttributeIds.Value, ua.DataValue(False))
                print("Set attribute emg ", False)
            except:
                pass
            cmd_vel_msg.angular.z = 0.0
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_pub_line_follower.publish(cmd_vel_msg)
            # cancel_pub.publish(cancel_msg)

        elif current_pos == FAULT:
            # send fault
            try:
                fault_send.set_attribute(
                    ua.AttributeIds.Value, ua.DataValue(False))
                print("Set attribute fault ", False)
            except:
                pass
        else:
            try:  # Emg yokken 1
                emg_send.set_attribute(
                    ua.AttributeIds.Value, ua.DataValue(True))
            except:
                pass
            allowed_conveyor_load = go_conveyor_load_val and pallet_info.pallet_loaded and pallet_info.box_num == 6
            allowed_conveyor_unload = go_conveyor_un_load_val and pallet_info.pallet_loaded == False
            allowed_robot_load = go_robot_load_val and pallet_info.pallet_loaded and pallet_info.box_num == 0
            if current_pos == IDLE:
                if goal_sended == False:
                    if go_home_val:
                        thread_send_station = Thread(
                            target=send_station, args=[HOME])
                        thread_send_station.start()
                    elif go_conveyor_load_val and pallet_info.pallet_loaded and pallet_info.box_num == 6:
                        thread_send_station = Thread(
                            target=send_station, args=[CONVEYOR_LOAD_BEFORE])
                        thread_send_station.start()
                    elif allowed_robot_load:
                        thread_send_station = Thread(
                            target=send_station, args=[ROBOT_AREA])
                        thread_send_station.start()
                    elif allowed_conveyor_unload:
                        thread_send_station = Thread(
                            target=send_station, args=[CONVEYOR_AREA_OUT])
                        thread_send_station.start()

            elif current_pos == HOME:
                # Before going anywhere checks load by operator
                #print(conveyor_loaded)
                if conveyor_loaded:
                    print("Pallet loaded")
                    try:
                        conveyor_load_done_send.set_attribute(
                            ua.AttributeIds.Value, ua.DataValue(True))
                        print("Set attribute conveyor load done", True)
                        pallet_info.pallet_loaded = True
                        pallet_info.box_num = 6
                    except:
                        pass
                    # load_time = time.time()

                if goal_sended == False:
                    if allowed_conveyor_unload:
                        try:
                            conveyor_load_done_send.set_attribute(
                                ua.AttributeIds.Value, ua.DataValue(False))
                            print("Set attribute conveyor load done ", False)
                            conveyor_loaded = False
                            thread_send_station = Thread(
                                target=send_station, args=[CONVEYOR_AREA_OUT])
                            thread_send_station.start()
                        except:
                            pass

                    elif go_conveyor_load_val and pallet_info.pallet_loaded and pallet_info.box_num == 6:
                        try:
                            conveyor_load_done_send.set_attribute(
                                ua.AttributeIds.Value, ua.DataValue(False))
                            print("Set attribute conveyor load done ", False)
                            conveyor_loaded = False
                            thread_send_station = Thread(
                                target=send_station, args=[CONVEYOR_LOAD_BEFORE])
                            thread_send_station.start()
                        except:
                            pass

                    elif allowed_robot_load:
                        try:
                            conveyor_load_done_send.set_attribute(
                                ua.AttributeIds.Value, ua.DataValue(False))
                            print("Set attribute conveyor load done ", False)
                            conveyor_loaded = False
                            thread_send_station = Thread(
                                target=send_station, args=[ROBOT_AREA])
                            thread_send_station.start()
                        except:
                            pass

            # Burada arada bir home geldiği için başka komut almıyorum.
            elif current_pos == CONVEYOR_LOAD_BEFORE:
                #print(pallet_info.box_num)
                if go_conveyor_load_val and pallet_info.pallet_loaded and pallet_info.box_num == 6:
                    if lane_detected == 0 and following_line == False:
                        print("Follow line")
                        time.sleep(0.5)
                        thread_follow_line_con_load = Thread(
                            target=follow_line, args=[87.0, 1.5])
                        thread_follow_line_con_load.start()
                        following_line = True
                    if following_line == True and follow_line_reached == True:
                        current_pos = CONVEYOR_LOAD
                        following_line = False
                        follow_line_reached = False

            elif current_pos == CONVEYOR_LOAD:
                try:
                    go_conveyor_load_send.set_attribute(
                        ua.AttributeIds.Value, ua.DataValue(True))
                    print("Set attribute conveyor load geldim", True)
                    rospy.loginfo("Home: %d, robot load: %d, conveyor load: %d, conveyor unload: %d, out from robot load: %d, out form conveyor load: %d, out from conveyor unload: %d, emg val: %d, transfer start %d",
                                  values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8])
                    if transfer_start_val == True:
                        current_pos = IN_TRANSFER_CONVEYOR_LOAD
                        transfer_start_val = False
                except:
                    pass

            elif current_pos == CONVEYOR_LOAD_BEFORE_OUT:
                if goal_sended == False:
                    if allowed_conveyor_unload:
                        thread_send_station = Thread(
                            target=send_station, args=[CONVEYOR_AREA])
                        thread_send_station.start()

                    elif go_conveyor_load_val and pallet_info.pallet_loaded and pallet_info.box_num == 6:
                        thread_send_station = Thread(
                            target=send_station, args=[CONVEYOR_LOAD_BEFORE])
                        thread_send_station.start()

                    elif allowed_robot_load:
                        thread_send_station = Thread(
                            target=send_station, args=[ROBOT_AREA])
                        thread_send_station.start()

                    elif go_home_val:
                        thread_send_station = Thread(
                            target=send_station, args=[HOME])
                        thread_send_station.start()

            elif current_pos == CONVEYOR_AREA:
                out_conveyor_send = False
                try:
                    out_from_conveyor_load_send.set_attribute(
                        ua.AttributeIds.Value, ua.DataValue(False))
                    out_conveyor_send = True
                except:
                    out_conveyor_send = False
                
                if goal_sended == False and out_conveyor_send:     
                    if allowed_conveyor_unload:
                        thread_send_station = Thread(target=send_station, args=[
                            CONVEYOR_UNLOAD_BEFORE])
                        thread_send_station.start()

                    elif allowed_conveyor_load or allowed_robot_load or go_home_val:
                        thread_send_station = Thread(target=send_station, args=[
                            CONVEYOR_UNLOAD_BEFORE_OUT])
                        thread_send_station.start()

            elif current_pos == CONVEYOR_UNLOAD_BEFORE:
                if allowed_conveyor_unload:
                    if lane_detected == 0 and following_line == False:
                        print("Follow line")
                        time.sleep(0.5)
                        thread_follow_line_con_un_load = Thread(
                            target=follow_line, args=[143.0, 0.0])
                        thread_follow_line_con_un_load.start()
                        following_line = True
                    if following_line == True and follow_line_reached == True:
                        current_pos = CONVEYOR_UNLOAD
                        following_line = False
                        follow_line_reached = False

                elif allowed_conveyor_load or allowed_robot_load or go_home_val:
                    if goal_sended == False:
                        thread_send_station = Thread(
                            target=send_station, args=[CONVEYOR_AREA_OUT])
                        thread_send_station.start()

            elif current_pos == CONVEYOR_UNLOAD:
                try:
                    go_conveyor_un_load_send.set_attribute(
                        ua.AttributeIds.Value, ua.DataValue(True))
                    print("Set attribute conveyor unload geldim", True)
                    if transfer_start_val == True:
                        current_pos = IN_TRANSFER_CONVEYOR_UNLOAD
                        transfer_start_val = False
                    if goal_sended == False:
                        if go_home_val:
                            thread_send_station = Thread(target=send_station, args=[
                                CONVEYOR_UNLOAD_BEFORE_OUT])
                            thread_send_station.start()
                except:
                    pass

            elif current_pos == CONVEYOR_UNLOAD_BEFORE_OUT:
                if goal_sended == False:
                    if allowed_conveyor_unload:
                        thread_send_station = Thread(target=send_station, args=[
                            CONVEYOR_UNLOAD_BEFORE])
                        thread_send_station.start()

                    elif allowed_conveyor_load or allowed_robot_load or go_home_val:
                        thread_send_station = Thread(
                            target=send_station, args=[CONVEYOR_AREA_OUT])
                        thread_send_station.start()

            elif current_pos == CONVEYOR_AREA_OUT:
                out_conveyor_send = False
                try:
                    out_from_conveyor_load_send.set_attribute(
                        ua.AttributeIds.Value, ua.DataValue(True))
                    out_conveyor_send = True
                except:
                    out_conveyor_send = False
                if goal_sended == False and out_conveyor_send:
                    if allowed_robot_load:
                        thread_send_station = Thread(
                            target=send_station, args=[ROBOT_AREA])
                        thread_send_station.start()

                    elif allowed_conveyor_unload:
                        thread_send_station = Thread(
                            target=send_station, args=[CONVEYOR_AREA])
                        thread_send_station.start()

                    elif allowed_conveyor_load:
                        thread_send_station = Thread(
                            target=send_station, args=[CONVEYOR_LOAD_BEFORE])
                        thread_send_station.start()
                    elif go_home_val:
                        thread_send_station = Thread(
                            target=send_station, args=[HOME])
                        thread_send_station.start()

            elif current_pos == ROBOT_AREA:
                if goal_sended == False:
                    if allowed_robot_load:
                        thread_send_station = Thread(
                            target=send_station, args=[ROBOT_LOAD])
                        thread_send_station.start()

                    elif allowed_conveyor_unload:
                        thread_send_station = Thread(
                            target=send_station, args=[CONVEYOR_AREA_OUT])
                        thread_send_station.start()

                    elif allowed_conveyor_load:
                        thread_send_station = Thread(
                            target=send_station, args=[CONVEYOR_LOAD_BEFORE])
                        thread_send_station.start()

                    elif go_home_val:
                        thread_send_station = Thread(
                            target=send_station, args=[HOME])
                        thread_send_station.start()

            elif current_pos == ROBOT_LOAD:
                scanner_area_change_pub.publish(1)  # 1 for robot area
                try:
                    if scanner_area == 1:
                        # print(ossd_active)
                        out_from_robot_load_send.set_attribute(
                            ua.AttributeIds.Value, ua.DataValue(False))
                        #print("Alan 1")
                        zone_info_send.set_attribute(
                            ua.AttributeIds.Value, ua.DataValue(not ossd_active))
                        #print("Set attribute zone info", not ossd_active)
                    else:  # alan değişti sinyali gelene kadar robota biri var datası yolluyorum
                        zone_info_send.set_attribute(
                            ua.AttributeIds.Value, ua.DataValue(False))
                        print("Set attribute zone info ", False)
                except:
                    pass
                if out_from_robot_load_ok_val:
                    scanner_area_change_pub.publish(0)  # 0 for normal area
                    if scanner_area == 0:  # alan değiştiyse çıkabilir
                        if goal_sended == False:
                            pallet_info.pallet_loaded = True
                            pallet_info.box_num = 6
                            if allowed_conveyor_unload or go_conveyor_load_val or go_home_val:
                                thread_send_station = Thread(
                                    target=send_station, args=[ROBOT_AREA_OUT])
                                thread_send_station.start()
                    else:  # alan değişti sinyali gelene kadar robota biri var datası yolluyorum
                        try:
                            zone_info_send.set_attribute(
                                ua.AttributeIds.Value, ua.DataValue(False))
                            print("Set attribute zone info 2 ", False)
                        except:
                            pass
                if go_home_val:
                    scanner_area_change_pub.publish(0)  # 0 for normal area
                    if scanner_area == 0:  # alan değiştiyse çıkabilir
                        if goal_sended == False:
                            thread_send_station = Thread(
                                target=send_station, args=[ROBOT_AREA_OUT])
                            thread_send_station.start()

            elif current_pos == ROBOT_AREA_OUT:
                if goal_sended == False:
                    if allowed_robot_load:
                        thread_send_station = Thread(
                            target=send_station, args=[ROBOT_AREA])
                        thread_send_station.start()

                    elif go_conveyor_load_val:
                        thread_send_station = Thread(
                            target=send_station, args=[CONVEYOR_LOAD_BEFORE])
                        thread_send_station.start()

                    elif go_conveyor_un_load_val:
                        thread_send_station = Thread(
                            target=send_station, args=[CONVEYOR_AREA_OUT])
                        thread_send_station.start()

                    elif go_home_val:
                        thread_send_station = Thread(
                            target=send_station, args=[HOME])
                        thread_send_station.start()

            elif current_pos == CONVEYOR_LOAD_DONE:
                pallet_info.pallet_loaded = False
                pallet_info.box_num = 0

                if out_from_conveyor_load_ok_val and goal_sended == False:
                    thread_send_station = Thread(target=send_station, args=[
                        CONVEYOR_LOAD_BEFORE_OUT])
                    thread_send_station.start()

                if go_home_val and goal_sended == False:
                    thread_send_station = Thread(target=send_station, args=[
                        CONVEYOR_LOAD_BEFORE_OUT])
                    thread_send_station.start()

            elif current_pos == CONVEYOR_UNLOAD_DONE:
                pallet_info.pallet_loaded = True
                pallet_info.box_num = 0
                if out_from_conveyor_un_load_ok_val and goal_sended == False:
                    thread_send_station = Thread(target=send_station, args=[
                        CONVEYOR_UNLOAD_BEFORE_OUT])
                    thread_send_station.start()

                if go_home_val and goal_sended == False:
                    thread_send_station = Thread(target=send_station, args=[
                        CONVEYOR_UNLOAD_BEFORE_OUT])
                    thread_send_station.start()

            elif current_pos == IN_TRANSFER_CONVEYOR_UNLOAD:  # TODO: burada home a git gelirse napıcam???
                if conveyor_running == False:
                    try:
                        transfer_start_send.set_attribute(
                            ua.AttributeIds.Value, ua.DataValue(True))
                        print("Set attribute transfer start info ", True)
                        conveyor_status_msg.load_conveyor = True
                        conveyor_status_msg.unload_conveyor = False
                        conveyor_pub.publish(conveyor_status_msg)
                        print("Conveyor is working for loading to AGV")
                        conveyor_running = True
                        conveyor_transfer_done = False
                    except:
                        pass
                if conveyor_transfer_done == True:
                    try:
                        print("Transfer done")
                        transfer_start_send.set_attribute(
                            ua.AttributeIds.Value, ua.DataValue(False))
                        print("Set attribute transfer start info ", False)
                        conveyor_transfer_done = False
                        conveyor_running = False
                        conveyor_status_msg.load_conveyor = False
                        conveyor_status_msg.unload_conveyor = False
                        conveyor_pub.publish(conveyor_status_msg)
                        current_pos = CONVEYOR_UNLOAD_DONE
                    except:
                        pass

            elif current_pos == IN_TRANSFER_CONVEYOR_LOAD:  # TODO: burada home a git gelirse napıcam???
                # conveyorü çalıştır
                if conveyor_running == False:
                    try:
                        transfer_start_send.set_attribute(
                            ua.AttributeIds.Value, ua.DataValue(True))
                        print("Set attribute transfer start info ", True)
                        conveyor_status_msg.unload_conveyor = True
                        conveyor_status_msg.load_conveyor = False
                        conveyor_pub.publish(conveyor_status_msg)
                        print("Conveyor is working for unloading from AGV")
                        conveyor_running = True
                        conveyor_transfer_done = False
                    except:
                        pass
                # conveyor load tamamlandıysa
                if conveyor_transfer_done == True:
                    try:
                        print("Transfer done")
                        transfer_start_send.set_attribute(
                            ua.AttributeIds.Value, ua.DataValue(False))
                        print("Set attribute transfer start info ", False)
                        conveyor_transfer_done = False
                        conveyor_running = False
                        conveyor_status_msg.load_conveyor = False
                        conveyor_status_msg.unload_conveyor = False
                        conveyor_pub.publish(conveyor_status_msg)
                        current_pos = CONVEYOR_LOAD_DONE

                    except:
                        pass

            elif current_pos >= GOING_HOME and current_pos <= GOING_ROBOT_AREA_OUT:
                try:
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
                    out_from_robot_load_send.set_attribute(
                        ua.AttributeIds.Value, ua.DataValue(True))
                except:
                    pass
                # WARNING :  BURADA GOING HOME OLDUĞUNDA SIKINTI YAŞAYABİLİRSİN AMA GOAL SENDED IN ÇÖZECEĞİNİ DÜŞÜNÜYORUM
                # ÇÖZMEZSE AŞAĞI GOING HOME DEĞİLSE KOŞULU DA KOY <3

                if go_home_val and goal_sended == False:  # Come back to source pos and go home
                    thread_send_station = Thread(target=send_station, args=[
                        source_pos])
                    thread_send_station.start()

            if current_pos < GOING_HOME and current_pos > GOING_ROBOT_AREA_OUT:
                source_pos = current_pos

            old_pos = current_pos
            with open('/home/rnd/moobot_ws/src/moobot_plc/src/current_pos.pickle', 'wb') as f:
                pickle.dump(current_pos, f)
            with open('/home/rnd/moobot_ws/src/moobot_plc/src/pallet_info.pickle', 'wb') as f:
                pickle.dump(pallet_info, f)

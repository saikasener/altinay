#!/usr/bin/env python3

# Run These Command in Bash
# roscore  -> Start Ros Core
# rosrun rqt_graph rqt_graph -> show rqt graph
# rosnode list -> optional
# /../ python3 first_node.py

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
import sensor_msgs.msg as sensor_msgs
import signal
import sys
import random
import os


def GetSerialData():
    global VoltageList,CurrentList,TempList
    # VoltageList = list()
    # CurrentList = list()
    # TempList = list()
    # VoltageList= ['4978', '4965', '0', '3316', '3319', '3319', '3318', '3316', '3316', '3318', '3318', '3318', '3318', '3317', '3320', '3319', '3319', '3318']
    rate = rospy.Rate(1)  # 200 ms -> 5Hz
    while not rospy.is_shutdown():
        scanner_narrow_area_front = """rosservice call /front_laser_scanner/front_laser_scanner/set_parameters "config:
                                    bools:
                                    - {name: '', value: false}
                                    ints:
                                    - {name: '', value: 0}
                                    strs:
                                    - {name: '', value: ''}
                                    doubles:
                                    - {name: "angle_start", value: "1.0"}
                                    - {name: "angle_end", value: "1.0"}
                                    groups:                                      
                                    - {name: '', state: false, id: 0, parent: 0}\""""
        os.system(scanner_narrow_area_front)  

        # a = random.random()
        # VoltageList.insert(0,a)
        # b = random.random()
        # CurrentList.insert(0,b)      
        # c = random.random()
        # TempList.insert(0,c)          
        # BMS_info()
        rate.sleep()

# def BMS_info():
#     # ros communications
#     battery_publisher = rospy.Publisher('~state', sensor_msgs.BatteryState, queue_size=1, latch=True)
#     rate = rospy.Rate(1)
#     battery = sensor_msgs.BatteryState()
#     battery.voltage = float(VoltageList[0])


#     battery_publisher.publish(battery)

#     # rate.sleep()
#     # GetSerialData()
#     # initialisations
#     # battery.header.stamp = rospy.Time.now()
#     # battery.voltage = 10
#     # battery.current = 100
#     # battery.charge = float('nan')
#     # battery.capacity = float('nan')
#     # battery.design_capacity = float('nan')
#     # battery.percentage = 100
#     # battery.power_supply_health = sensor_msgs.BatteryState.POWER_SUPPLY_HEALTH_GOOD
#     # battery.power_supply_technology = sensor_msgs.BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
#     # battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_FULL
#     # battery.present = True
#     # battery.location = ""
#     # battery.serial_number = ""

#     # while not rospy.is_shutdown():
#         # battery_msg = battery.voltage
    
# # ----------------------------------------------------------------


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    connected = False
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
# ----------------------------------------------------------------


if __name__ == '__main__':
    rospy.init_node('BMS_node1')
    try:
        GetSerialData()
    except rospy.ROSInterruptException:
        pass

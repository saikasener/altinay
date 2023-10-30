#!/usr/bin/env python3

# ----------------------------------------------------------------
# -------------------- Import Module -----------------------------
# ----------------------------------------------------------------
import serial
import time as t
import serial.tools.list_ports as port_list
import re
import os
import rospy
import signal
import sys
import numpy as np
from moobot_msgs.msg import bms_status


PortName = None
# ----------------------------------------------------------------
# -------------------- Function Definition -----------------------
# ----------------------------------------------------------------


def GetPortList():
    ports = list(port_list.comports())
    for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))
        return str(port)

# ----------------------------------------------------------------


def SetPortConfig():
    global PortName
    # ----------------- For Linux --------------------------------
    if os.name == 'posix':
        BashCommand = 'sudo chmod 666 ' + \
            str(PortName)     # sudo chmod 666 /dev/ttyACM0
        # Run this command in console for [Errno 13] Permission denied
        os.system(BashCommand)
        Ser = serial.Serial(PortName, 115200, timeout=.1)
    # ----------------- For Windows ------------------------------
    elif os.name == 'nt':
        Ser = serial.Serial(port='COM9', baudrate=115200, timeout=.1)

    if (Ser.isOpen()):
        return Ser
    else:
        return -1
# ----------------------------------------------------------------


def GetSerialData(Ser):
    rate = rospy.Rate(5) # 200 ms -> 5Hz
    while Ser.isOpen():
        # global VoltageList, CurrentList, TemperatureList
        VoltageList = []
        Cell_VoltageList = []
        CurrentList = []
        TemperatureList = []
        Cell_TemperatureList =[]          
        SerialData = str(Ser.readline())
        # print(SerialData)
        if SerialData[2] == 'V':
            TempVoltage = SerialData
            VoltageList = TextSplit("V=:\\rb'", TempVoltage)
            VoltageList = list(filter(None, VoltageList))     # listenin başındaki float olmayan elemanların silinmesi        
            for i in range(0,len(VoltageList)):                
                VoltageList[i] = float(VoltageList[i]) / 100
            Cell_VoltageList = VoltageList[3:]
            for i in range(0,len(Cell_VoltageList)):                
                Cell_VoltageList[i] = float(Cell_VoltageList[i]) / 10
            # --------- ROS Messages---------
            battery.battery_voltage = VoltageList[0] 
            battery.load_voltage = VoltageList[1]
            battery.charger_voltage = VoltageList [2] 
            battery.cell_voltage=Cell_VoltageList
            CleanList(VoltageList)
            print(f'Voltage List -> {VoltageList}\n')
            # rospy.loginfo('Voltage-> ' + str(VoltageList))
        elif SerialData[2] == 'A':
            TempCurrent = SerialData
            CurrentList = TextSplit("A=:\\rb'", TempCurrent)
            CurrentList = list(filter(None, CurrentList))
            for i in range(0,len(CurrentList)):                
                CurrentList[i] = float(CurrentList[i])
            # --------- ROS Messages---------
            battery.current_income = CurrentList[0] / 100    # Pile gelen akım (A)
            battery.capacity = CurrentList[1] / 10           # Pilin doluluk kapasitesi (Ah)
            battery.capacity_max = CurrentList[2] / 10       # Pilin total kapasitesi (Ah)
            battery.soc = CurrentList[3]                     # Charge percentage (%)
            battery.current_charger = CurrentList[4] / 100   # Charger dan gelen akım (A)           
            battery.current_load = CurrentList[5] / 100      # Sistemin Çektiği Akım (A)
            CleanList(CurrentList)
            print(f'Current List -> {CurrentList}\n')
            # rospy.loginfo('Current-> ' + str(CurrentList))

        elif SerialData[2] == 'T':
            TempTemperature = SerialData
            TemperatureList = TextSplit("T=:\\rb'", TempTemperature)
            TemperatureList = list(filter(None, TemperatureList))
            for i in range(0,len(TemperatureList)):                
                TemperatureList[i] = float(TemperatureList[i])
            Cell_TemperatureList = TemperatureList[1:]
            # --------- ROS Messages---------                        
            battery.temperature_bms = TemperatureList[0]
            battery.temperature_cell=Cell_TemperatureList
            CleanList(TemperatureList)
            print(f'Temperature List -> {TemperatureList}\n')
            # rospy.loginfo('Temperature-> ' + str(TemperatureList))

        # BMS_info()
        battery_publisher.publish(battery)       
        rate.sleep()
    else:
        Ser.close()
# ----------------------------------------------------------------


def WriteSerialData(Ser):
    Delay = 0.1  # Time Delay
    ReadMilis = 1000  # Data Reading Periode From BMS
    WriteList = ['# C\r', '?V \r', '?A \r', '?T \r', '# ' +
                 str(ReadMilis) + ' \r']  # BMS Initialize Command
    for i in range(0, len(WriteList)):
        Ser.write(str.encode(WriteList[i]))
        t.sleep(Delay)
    Ser.flushInput()  # Serial Buffer Input Clean
# ----------------------------------------------------------------


def TextSplit(delimiters, string, maxsplit=0):
    # BMS Data Split Using Regular Expression
    RegexPattern = '|'.join(map(re.escape, delimiters))
    return re.split(RegexPattern, string, maxsplit)
# ----------------------------------------------------------------


def CleanList(List):
    """
    Parameters
    List : List Object with ''
    ----------
    Returns
    List Object without '' token
    """
    while '' in List:
        List.remove('')
# ----------------------------------------------------------------


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    connected = False
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
# ----------------------------------------------------------------


def Main():  # Main Function
    global PortName, battery , battery_publisher
    PortName = GetPortList()
    if SetPortConfig() == -1:
        print("Cannot Open Serial Port...")
        exit()
    else:
        rospy.init_node('scan_run')
        rospy.loginfo('Node Begin!')
        battery_publisher = rospy.Subscriber('bms_status', bms_status, queue_size=1, latch=True)
        battery = bms_status()
        print("Serial Port Open Now...")
        SerObj = SetPortConfig()  # SerObj -> Serial Port Object
        WriteSerialData(SerObj)
        while not rospy.is_shutdown():  # While True Until Shutdown Node
            GetSerialData(SerObj)


# ----------------------------------------------------------------
# -------------------- Run Main Code -----------------------------
# ----------------------------------------------------------------
if __name__ == "__main__":
    Main()
else:
    print("XXXXXX")

# -*- coding: utf-8 -*-
"""
Written By: Rasit EVDUZEN
Created on Tue Oct 18 10:17:18 2022
"""


#-------------------- Import Module -----------------------------
#----------------------------------------------------------------
import serial, time as t
import serial.tools.list_ports as port_list
import re, os
PortName = None
#----------------------------------------------------------------
#----------------------------------------------------------------
#-------------------- Function Definition -----------------------
#----------------------------------------------------------------
def GetPortList():
    ports = list(port_list.comports())
    for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))
        return str(port)

#----------------------------------------------------------------
def SetPortConfig():
    #----------------- For Windows ----------------------
    #Ser = serial.Serial(port='COM9', baudrate=115200, timeout=.1) 
    #----------------- For Linux ----------------------
    global PortName
    BashCommand = 'sudo chmod 666 ' + str(PortName)
    os.system(BashCommand)
    Ser = serial.Serial(PortName, 115200, timeout = .1) 
    # Run this command in console for [Errno 13] Permission denied
    # sudo chmod 666 /dev/ttyACM0
    if(Ser.isOpen()):
        return Ser
    else:
        return -1
#----------------------------------------------------------------
def GetSerialData(Ser):
    while Ser.isOpen():
        VoltageList = []
        CurrentList = []
        TemperatureList = []
        SerialData = str(Ser.readline())
        # print(SerialData)
        if SerialData[2] == 'V':
            TempVoltage = SerialData
            VoltageList = TextSplit("V=:\\rb'",TempVoltage)
            CleanList(VoltageList)
            print(f'Voltage List -> {VoltageList}\n')
        elif SerialData[2] == 'A':
            TempCurrent = SerialData
            CurrentList = TextSplit("A=:\\rb'",TempCurrent)
            CleanList(CurrentList)
            print(f'Current List -> {CurrentList}\n')
        elif SerialData[2] == 'T':
            TempTemperature = SerialData
            TemperatureList = TextSplit("T=:\\rb'",TempTemperature)
            CleanList(TemperatureList)
            print(f'Temperature List -> {TemperatureList}\n')

    else:
        Ser.close()
#----------------------------------------------------------------        
def WriteSerialData(Ser):
    Delay = 0.1 
    ReadMilis = 100
    WriteList = ['# C\r', '?V \r', '?A \r', '?T \r', '# ' + str(ReadMilis) + ' \r']
    for i in range(0, len(WriteList)):
        Ser.write(str.encode(WriteList[i]))
        t.sleep(Delay)
    Ser.flushInput()
#----------------------------------------------------------------         
def TextSplit(delimiters, string, maxsplit=0):
    RegexPattern = '|'.join(map(re.escape, delimiters))
    return re.split(RegexPattern, string, maxsplit)
#----------------------------------------------------------------        
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
        
#----------------------------------------------------------------        
    
#----------------------------------------------------------------        
def Main(): # Main Function
    global PortName
    PortName = GetPortList()
    if SetPortConfig() == -1:
        print("Cannot Open Serial Port...")
        exit()
    else:
        print("Serial Port Open Now...")
        SerObj = SetPortConfig() # SerObj -> Serial Port Object
        WriteSerialData(SerObj)
        GetSerialData(SerObj)
        
#----------------------------------------------------------------



#----------------------------------------------------------------
#----------------------------------------------------------------



if __name__ == "__main__":
    Main()
else:
    print("XXXXXX")



    

# -*- coding: utf-8 -*-
"""
Written By: Rasit EVDUZEN
Created on Fri Oct 21 19:00:07 2022

"""
import re

"""TempVoltage = 'V=0:0:2440:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0'
TempCurrent = 'A=0:249:500:49:0:0:0:0:1032:16386:7:0:16:0:0'
TempTemperature = 'T=25:25:25:25'"""

TempVoltage = "b'V=0:0:2404:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0\r\r"
TempTemperature =  "b'T=25:25:25:25\r\r"
TempCurrent = "b'A=0:249:500:49:0:0:0:0:1032:16386:7:0:16:0:0\r\r"

SerialData = "b'V=0:0:2404:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0\r\r"

if SerialData[0] == 'V':
    TempVoltage = SerialData
elif SerialData[0] == 'A':
    TempCurrent = SerialData
elif SerialData[0] == 'T':
    TempTemperature = SerialData

def TextSplit(delimiters, string = '', maxsplit=0):
    RegexPattern = '|'.join(map(re.escape, delimiters))
    return re.split(RegexPattern, string, maxsplit)



    
def CleanList(VoltageList = None,CurrentList = None,TemperatureList = None):
    while '' in CurrentList:
        CurrentList.remove('')
        VoltageList.remove('')
        TemperatureList.remove('')


VoltageList = TextSplit("V=:\rb'",TempVoltage)
CurrentList = TextSplit("A=:\rb'",TempCurrent)
TemperatureList = TextSplit("T=:\rb'",TempTemperature)
CleanList(VoltageList,CurrentList,TemperatureList)


print(f'Voltage List -> {VoltageList}\nCurrent List -> {CurrentList}\nTemperature List -> {TemperatureList}')


#/*-----------------------------------------------------------------------------
# * EcDemoApp.py
# * Copyright                acontis technologies GmbH, Ravensburg, Germany
# * Description              EC-Master demo application for Python
# *---------------------------------------------------------------------------*/
# pylint: disable=unused-wildcard-import, wildcard-import, unused-argument
from EcWrapperPython import *
from EcWrapperPythonTypes import *
from enum import Enum
import sys, argparse
from threading import Thread
import threading
import time

EC_DEMO_APP_NAME = "EcMasterDemoPython"
EC_DEMO_APP_DESC = "EC-Master demo application for Python"

class RunMode(Enum):
    Unknown = 0
    Master = 1
    RasClient = 2
    MbxGateway = 3
    Simulator = 4

class EcLogging:
    def __init__(self):
        self.severity = DN_EC_LOG_LEVEL.INFO
        self.prefix = ""
        return

    def initLogger(self, dwSeverity, szLogFileprefix):
        self.severity = dwSeverity
        self.prefix = szLogFileprefix
        return True

    def logMsg(self, severity, msg, *args):
        if severity.value > self.severity.value:
            return
        if (len(args) > 0):
            msg = EcLogging.replaceFormatStr(msg)
            msg = msg.format(*args)
        print(self.prefix + msg.rstrip())
        return

    @staticmethod
    def replaceFormatStr(msg):
        msg = msg.replace("%d", "{}")
        msg = msg.replace("%lx", "{}")
        msg = msg.replace("%x", "{:02X}")
        msg = msg.replace("%02X", "{:02X}")
        msg = msg.replace("%04x", "{:04X}")
        msg = msg.replace("%04X", "{:04X}")
        msg = msg.replace("%4d", "{}")
        msg = msg.replace("%08X", "{:08X}")
        msg = msg.replace("%s", "{}")
        return msg

class EcProcessImageDynGroup(object):
    pass

class EcProcessImageGroup:
    def __init__(self):
        self._children = {}

    def hasChild(self, name):
        if name in self._children:
            return True
        return False

    def addChild(self, name, value):
        self._children[name] = value
        return

    def getChild(self, name):
        return self._children[name]

    def createDynObject(self):
        obj = EcProcessImageDynGroup()
        for name in self._children:
            value = self._children[name]
            # create dynamic object (escape special chars)
            name = name.replace("[", "_")
            name = name.replace("]", "_")
            name = name.replace(" ", "_")
            if isinstance(value, EcProcessImageGroup):
                value = value.createDynObject()
            setattr(obj, name, value)
        return obj

class EcProcessImageVariable:
    def __init__(self, demo2, rtMode, variable):
        self._demo = demo2
        self._rtMode = rtMode
        self._variable = variable

    def set(self, value):
        variable = self._variable
        if self._rtMode:
            pbyPDBuf = self._demo.oReg.pbyPDIn if variable.bIsInputData else self._demo.oReg.pbyPDOut
            dwRes = CEcWrapperPython.WriteValueToAddress(pbyPDBuf, variable.nBitOffs, variable.nBitSize, DN_EC_T_DEFTYPE(variable.wDataType), value)
            if dwRes != ECError.EC_NOERROR:
                return False
            return True

        out_valueAsBytes = CEcWrapperPythonOutParam()
        dwRes = CEcWrapperPython.ConvValueToBytes(DN_EC_T_DEFTYPE(variable.wDataType), value, out_valueAsBytes)
        if dwRes != ECError.EC_NOERROR:
            print("ConvValueToBytes failed! Error Text: " + self._demo.oEcWrapper.GetErrorText(dwRes))
            return False
        valueAsBytes = out_valueAsBytes.value
        dwRes = self._demo.oEcWrapper.SetProcessDataBits(not variable.bIsInputData, variable.nBitOffs, valueAsBytes, variable.nBitSize, 2000)
        if dwRes != ECError.EC_NOERROR:
            print("Write process data failed! Error Text: " + self._demo.oEcWrapper.GetErrorText(dwRes))
            return False

        print("Variable written.")
        return True

    def get(self):
        variable = self._variable

        if self._rtMode:
            pbyPDBuf = self._demo.oReg.pbyPDIn if variable.bIsInputData else self._demo.oReg.pbyPDOut
            out_dataRead = CEcWrapperPythonOutParam()
            dwRes = CEcWrapperPython.ReadValueFromAddress(pbyPDBuf, variable.nBitOffs, variable.nBitSize, DN_EC_T_DEFTYPE(variable.wDataType), out_dataRead)
            if dwRes != ECError.EC_NOERROR:
                return None
            dataRead = out_dataRead.value
            return dataRead

        pbyDataDst = [0] * ((variable.nBitSize + 8) // 8)
        dwRes = self._demo.oEcWrapper.GetProcessDataBits(not variable.bIsInputData, variable.nBitOffs, pbyDataDst, variable.nBitSize, 2000)
        if dwRes != ECError.EC_NOERROR:
            print("Read process data failed! Error Text: " + self._demo.oEcWrapper.GetErrorText(dwRes))
            return None

        out_dataRead = CEcWrapperPythonOutParam()
        dwRes = CEcWrapperPython.ReadValueFromBytes(pbyDataDst, 0, variable.nBitSize, DN_EC_T_DEFTYPE(variable.wDataType), out_dataRead)
        if dwRes != ECError.EC_NOERROR:
            print("ReadValueFromBytes failed! Error Text: " + self._demo.oEcWrapper.GetErrorText(dwRes))
            return None

        dataRead = out_dataRead.value
        return dataRead

    def dataType(self):
        return DN_EC_T_DEFTYPE(self._variable.wDataType)

    def size(self):
        return self._variable.nBitSize

    def dmp(self):
        variable = self._variable
        print("szName       = " + str(variable.szName))
        print("wDataType    = " + str(variable.wDataType))
        print("wFixedAddr   = " + str(variable.wFixedAddr))
        print("nBitSize     = " + str(variable.nBitSize))
        print("nBitOffs     = " + str(variable.nBitOffs))
        print("bIsInputData = " + str(variable.bIsInputData))
        return

class EcProcessImageSlave:
    def __init__(self):
        self.address = 0
        self.name = ""
        self.variables = []

class EcProcessImage:
    def __init__(self, demo2, rtMode):
        self.demo = demo2
        self.rtMode = rtMode
        self.slaves = []
        self.variables = []
        self.variableByName = {}
        return

    def getVariableByName(self, name):
        return self.variableByName[name]

    def reload(self):
        self.slaves.clear()

        # /* Read amount of slaves from Master */
        oStatusArr = CEcWrapperPythonOutParam()
        self.demo.oEcWrapper.GetScanBusStatus(oStatusArr)
        oStatus = oStatusArr.value
        if oStatus.dwResultCode == ECError.EC_BUSY: #ECScanBus.BUSY:
            return

        if oStatus.dwSlaveCount == 0:
            return

        ## /* Create slave nodes, if slaves are connected */
        for wIdx in range(0, oStatus.dwSlaveCount):
            out_oCfgSlaveInfo = CEcWrapperPythonOutParam()

            # /* Request information about slave object */
            eRetVal = self.demo.oEcWrapper.GetCfgSlaveInfo(False, 0 - wIdx, out_oCfgSlaveInfo)
            oCfgSlaveInfo = out_oCfgSlaveInfo.value
            if ECError.EC_NOERROR != eRetVal:
                self.demo.logMasterError("Reading slave info failed: ", eRetVal)
                continue

            slaveData = EcProcessImageSlave()
            slaveData.address = oCfgSlaveInfo.wStationAddress
            slaveData.name = oCfgSlaveInfo.abyDeviceName
            self.slaves.append(slaveData)

        for slaveData in self.slaves:
            inputVariables = self.ReadProcessVariablesFromSlave(slaveData.address, True)
            if inputVariables != None:
                slaveData.variables.extend(inputVariables)
            outputVariables = self.ReadProcessVariablesFromSlave(slaveData.address, False)
            if outputVariables != None:
                slaveData.variables.extend(outputVariables)

        self.variables = self.ReadProcessVariables()
        return

    #/// <summary>Reads all process variables of a slave</summary>
    def ReadProcessVariablesFromSlave(self, wSlaveAddress, input_):
        numOfVariablesArr = CEcWrapperPythonOutParam()
        eRes = ECError.EC_ERROR
        if input_:
            eRes = self.demo.oEcWrapper.GetSlaveInpVarInfoNumOf(True, wSlaveAddress, numOfVariablesArr)
        else:
            eRes = self.demo.oEcWrapper.GetSlaveOutpVarInfoNumOf(True, wSlaveAddress, numOfVariablesArr)
        numOfVariables = numOfVariablesArr.value
        if eRes != ECError.EC_NOERROR:
            self.demo.logMasterError("Reading number of process variables failed: ", eRes)
            return None

        if numOfVariables == 0:
            return None

        numOfReadVariablesArr = CEcWrapperPythonOutParam()
        variablesArr = CEcWrapperPythonOutParam()
        if input_:
            eRes = self.demo.oEcWrapper.GetSlaveInpVarInfo(True, wSlaveAddress, numOfVariables, variablesArr, numOfReadVariablesArr)
        else:
            eRes = self.demo.oEcWrapper.GetSlaveOutpVarInfo(True, wSlaveAddress, numOfVariables, variablesArr, numOfReadVariablesArr)
        variables = variablesArr.value
        if eRes != ECError.EC_NOERROR:
            self.demo.logMasterError("Reading process variables failed: ", eRes)
            return None

        return variables

    def ReadProcessVariables(self):
        variables = []
        for slave in self.slaves:
            variables.extend(slave.variables)
        tree = EcProcessImageGroup()
        for variable in variables:
            parts = variable.szName.split(".")
            cur = tree
            for i, part in enumerate(parts):
                if i == len(parts) - 1:
                    v = EcProcessImageVariable(self.demo, self.rtMode, variable)
                    self.variableByName[variable.szName] = v
                    cur.addChild(part, v)
                elif not cur.hasChild(part):
                    cur.addChild(part, EcProcessImageGroup())
                cur = cur.getChild(part)
        return tree.createDynObject()

class MyAppDesc:
    def __init__(self):
        self.wFlashSlaveAddr = 0         #/* flash slave address */
        self.dwFlashTimer = 0            #/* flash timer */
        self.dwFlashInterval = 20000     #/* flash every 20 msec */
        self.oFlashVariable = None       #/* flash variable */

class EcDemoFormatter(argparse.HelpFormatter):
    def _split_lines(self, text, width):
        if text.find('\n')!=-1:
            return text.splitlines()
        return argparse.HelpFormatter._split_lines(self, text, width)

class EcDemoAppParams:
    ATEMRAS_DEFAULT_PORT = 6000
    MBXGATEWAY_DEFAULT_PORT = 34980

    def __init__(self):

        #/* configuration */
        self.tRunMode = RunMode.Unknown         #/* Run mode */
        self.szENIFilename = ""                 #/* ENI filename string */
        #/* link layer */
        self.szLinkLayer = ""                   #/* Link layer settings */
        #/* timing */
        self.dwBusCycleTimeUsec = 4000          #/* bus cycle time in usec */
        self.dwDemoDuration = 0                 #/* demo duration in msec */
        #/* logging */
        self.nVerbose = 3                       #/* verbosity level */
        self.dwAppLogLevel = DN_EC_LOG_LEVEL.UNDEFINED #/* demo application log level (derived from verbosity level) */
        self.dwMasterLogLevel = DN_EC_LOG_LEVEL.UNDEFINED #/* master stack log level (derived from verbosity level) */
        self.szLogFileprefix = ""               #/* log file prefix string */
        #/* RAS */
        self.wRasServerPort = 0                 #/* remote access server port */
        self.abyRasServerIpAddress = "127.0.0.1" #/* remote access server IP address */
        #/* additional parameters for the different demos */
        self.wFlashSlaveAddr = 0                #/* flashing output slave station address */
        self.bPerfMeasEnabled = False           #/* performance measurement for jobs enabled */

class EcDemoApp:
    c_MASTER_CFG_ECAT_MAX_BUS_SLAVES = 256
    c_MASTER_CFG_MAX_ACYC_FRAMES_QUEUED = 32        #/* max number of acyc frames queued, 127 = the absolute maximum number */
    c_MASTER_CFG_MAX_ACYC_BYTES_PER_CYC = 4096      #/* max number of bytes sent during eUsrJob_SendAcycFrames within one cycle */
    c_MASTER_CFG_MAX_ACYC_CMD_RETRIES = 3
    ETHERCAT_STATE_CHANGE_TIMEOUT = 15000           #/* master state change timeout in ms */
    ETHERCAT_SCANBUS_TIMEOUT = 10000                #/* scanbus timeout in ms, see also EC_SB_DEFAULTTIMEOUT */
    PERF_myAppWorkpd = 0
    MAX_JOB_NUM = 1
    S_aPerfMeasInfos = [ "myAppWorkPd                " ]
    EC_PERF_MEAS_ALL = 0xFFFFFFFF

    def __init__(self):
        self.pAppParms = EcDemoAppParams()
        self.pMyAppDesc = MyAppDesc()
        self.logger = EcLogging()
        self.oEcWrapper = CEcWrapperPythonEx()
        self.onDbgMsgNotificationId = -1
        self.onMasterNotificationId = -1
        self.onRasNotificationId = -1
        self.processImage = EcProcessImage(self, False)   #/* process image (default) */
        self.processImageRt = EcProcessImage(self, True)  #/* process image with realtime support (access only from job task) */
        self.oReg = None
        self.m_oJobTask = None
        self.m_bJobTaskRunning = False
        self.m_bJobTaskShutDown = False
        self.m_nCycleTime = 4
        self.bPerfMeasEnabled = False
        self.pvPerfMeas = ctypes.c_void_p(None)

    def logMasterError(self, message, eRes):
        self.logMsg(DN_EC_LOG_LEVEL.ERROR, "{}{} (0x{:08X})".format(message, self.oEcWrapper.GetErrorText(eRes), eRes))

    def logInfo(self, message, *args):
        self.logMsg(DN_EC_LOG_LEVEL.INFO, message, *args)

    def logMsg(self, severity, message, *args):
        self.logger.logMsg(severity, message, *args)

    def parseCommandLine(self, argv):
        parser = argparse.ArgumentParser(description=EC_DEMO_APP_DESC, formatter_class=EcDemoFormatter)
        parser.add_argument("-f", metavar='file', help="Use given ENI file")
        parser.add_argument("-t", metavar='time', type=int, help="Demo duration\nTime in msec, 0 = forever (default = 120000)")
        parser.add_argument("-b", metavar='cycle time', type=int, help="Bus cycle time\nCycle time in usec")
        parser.add_argument("-v", metavar='lvl', type=int, help="Set verbosity level\nLevel: 0=off, 1...n=more messages, 3(default) generate dcmlog file")
        parser.add_argument("--log", metavar='prefix', help="Prefix for log files")
        parser.add_argument("--sp", metavar='port', type=int, help="Start RAS server\nport (default = {})".format(EcDemoAppParams.ATEMRAS_DEFAULT_PORT))
        parser.add_argument("-rem", metavar='remoteip:port', help="RAS server IP address and port (default = {})".format(EcDemoAppParams.ATEMRAS_DEFAULT_PORT))
        parser.add_argument("--mode", metavar='mode', type=int, required=True, help="Run mode\n1 = master, 2 = ras client, 3 = mailbox gateway, 4 = simulator")
        parser.add_argument("--link", metavar='name', help="Link layer\nname and link layer specific settings (see '--link help')\ne.g. 'winpcap 127.0.0.0 1' or 'sockraw eth0 1'")
        parser.add_argument("--flash", type=int, metavar='slave', help="Address of slave to flash")
        parser.add_argument('--perf', dest='perf', action='store_true', help="Enable performance measurement (printed cyclically if verbosity level >= 2)")
        args, unknown = parser.parse_known_args()
        if args.f:
            self.pAppParms.szENIFilename = args.f
        if args.t:
            self.pAppParms.dwDemoDuration = args.t
        if args.b:
            self.pAppParms.dwBusCycleTimeUsec = args.b
        if args.v:
            self.pAppParms.nVerbose = args.v
        if args.log:
            self.pAppParms.szLogFileprefix = args.log
        if args.sp:
            self.pAppParms.wRasServerPort = args.sp
        if args.rem:
            if self.pAppParms.abyRasServerIpAddress.find(':') != -1:
                parts = args.rem.split(':')
                self.pAppParms.abyRasServerIpAddress = parts[0]
                self.pAppParms.wRasServerPort = int(parts[1])
            else:
                self.pAppParms.abyRasServerIpAddress = args.rem
                self.pAppParms.wRasServerPort = 6000
        if args.mode:
            self.pAppParms.tRunMode = RunMode(args.mode)
        if args.link:
            self.pAppParms.szLinkLayer = args.link
        if args.flash:
            self.pAppParms.wFlashSlaveAddr = args.flash
        if args.perf:
            self.pAppParms.bPerfMeasEnabled = True
        return True

    def setAppAndMasterLogLevel(self):
        if self.pAppParms.nVerbose == 0:
            self.pAppParms.dwAppLogLevel   = DN_EC_LOG_LEVEL.SILENT
            self.pAppParms.dwMasterLogLevel = DN_EC_LOG_LEVEL.SILENT
        elif self.pAppParms.nVerbose == 1:
            self.pAppParms.dwAppLogLevel   = DN_EC_LOG_LEVEL.INFO
            self.pAppParms.dwMasterLogLevel = DN_EC_LOG_LEVEL.ERROR
        elif self.pAppParms.nVerbose == 2:
            self.pAppParms.dwAppLogLevel   = DN_EC_LOG_LEVEL.INFO
            self.pAppParms.dwMasterLogLevel = DN_EC_LOG_LEVEL.WARNING
        elif self.pAppParms.nVerbose == 3:
            self.pAppParms.dwAppLogLevel   = DN_EC_LOG_LEVEL.VERBOSE
            self.pAppParms.dwMasterLogLevel = DN_EC_LOG_LEVEL.WARNING
        elif self.pAppParms.nVerbose == 4:
            self.pAppParms.dwAppLogLevel   = DN_EC_LOG_LEVEL.VERBOSE
            self.pAppParms.dwMasterLogLevel = DN_EC_LOG_LEVEL.INFO
        elif self.pAppParms.nVerbose == 5:
            self.pAppParms.dwAppLogLevel   = DN_EC_LOG_LEVEL.VERBOSE
            self.pAppParms.dwMasterLogLevel = DN_EC_LOG_LEVEL.VERBOSE
        else:
            self.pAppParms.dwAppLogLevel   = DN_EC_LOG_LEVEL.VERBOSE_CYC
            self.pAppParms.dwMasterLogLevel = DN_EC_LOG_LEVEL.VERBOSE_CYC

    @staticmethod
    def str_to_ip_address(string):
        return [int(octet) for octet in string.split(".")[0:4]]

    def showSyntaxLinkLayer(self):
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "Link layer options:\n")

        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "   winpcap           Link layer = WinPcap/NPF\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     IpAddress       IP address of network adapter card, e.g. 192.168.157.2\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "                     NPF only: 255.255.255.x, x = network adapter number (1,2,...)\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     Mode            Interrupt (0) or Polling (1) mode\n")

        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "   sockraw           Link layer = raw socket\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     Device          network device (e.g. eth1)\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     Mode            Interrupt (0) or Polling (1) mode\n")

        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "   i8254x            Link layer = Intel 8254x\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     Instance        Device instance (1=first), e.g. 1\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     Mode            Interrupt (0) or Polling (1) mode\n")

        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "   i8255x            Link layer = Intel 8255x\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     Instance        Device instance (1=first), e.g. 1\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     Mode            Interrupt (0) or Polling (1) mode\n")

        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "   rtl8139           Link layer = Realtek RTL8139\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     Instance        Device instance (1=first), e.g. 1\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     Mode            Interrupt (0) or Polling (1) mode\n")

        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "   rtl8169           Link layer = Realtek RTL8169 / RTL8168 / RTL8111\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     Instance        Device instance (1=first), e.g. 1\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     Mode            Interrupt (0) or Polling (1) mode\n")

        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "   ccat              Link layer = Beckhoff CCAT\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     Instance        Device instance (1=first), e.g. 1\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     Mode            Interrupt (0) or Polling (1) mode\n")

        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "   udp               Link layer = UDP\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     IpAddress       IP address of network adapter card, e.g. 192.168.157.2\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "                     NPF only: 255.255.255.x, x = network adapter number (1,2,...)\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     Mode            Interrupt (0) or Polling (1) mode\n")

        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "   ndis              Link layer = NDIS\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     IpAddress       IP address of network adapter card, e.g. 192.168.157.2 or 0.0.0.0 if name given (optional)\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     Mode            Interrupt (0) or Polling (1) mode\n")

        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "   simulator         Link layer = EC-Simulator\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     EXI-file        simulated topology\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     Instance        Device instance (1=first), e.g. 1\n")
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "     Mode            Interrupt (0) or Polling (1) mode\n")

    def createLinkParms(self):
        try:
            if self.pAppParms.szLinkLayer == "help":
                self.showSyntaxLinkLayer()
                sys.exit(0)

            arLinkLayer = self.pAppParms.szLinkLayer.split(" ")
            if not arLinkLayer or len(arLinkLayer) < 1:
                return None

            name = arLinkLayer[0]
            oLinkParms = DN_EC_T_LINK_PARMS()

            if name == "winpcap":
                eLinkMode = int(arLinkLayer[2])
                oLinkParms.eLinkType = ELinkType.WINPCAP
                oLinkParms.eLinkMode = ELinkMode.INTERRUPT if eLinkMode == 0 else ELinkMode.POLLING
                oLinkParms.dwInstance = 0
                oLinkParms.oWinPcap = DN_EC_T_LINK_PARMS_WINPCAP()
                oLinkParms.oWinPcap.abyIpAddress = EcDemoApp.str_to_ip_address(arLinkLayer[1])
                return oLinkParms

            if name == "sockraw":
                szAdapterName = arLinkLayer[1]
                eLinkMode = int(arLinkLayer[2])
                oLinkParms.eLinkType = ELinkType.SOCKRAW
                oLinkParms.eLinkMode = ELinkMode.INTERRUPT if eLinkMode == 0 else ELinkMode.POLLING
                oLinkParms.dwInstance = 0
                oLinkParms.oSockRaw = DN_EC_T_LINK_PARMS_SOCKRAW()
                oLinkParms.oSockRaw.szAdapterName = szAdapterName
                return oLinkParms

            if name == "i8254x":
                dwInstance = int(arLinkLayer[1])
                eLinkMode = int(arLinkLayer[2])
                oLinkParms.eLinkType = ELinkType.I8254X
                oLinkParms.eLinkMode = ELinkMode.INTERRUPT if eLinkMode == 0 else ELinkMode.POLLING
                oLinkParms.dwInstance = dwInstance
                oLinkParms.oI8254x = DN_EC_T_LINK_PARMS_I8254X()
                return oLinkParms

            if name in {"i8255x", "rtl8139", "rtl8169", "ccat"}:
                dwInstance = int(arLinkLayer[1])
                eLinkMode = int(arLinkLayer[2])
                if name == "i8255x":
                    oLinkParms.eLinkType = ELinkType.I8255X
                elif name == "rtl8139":
                    oLinkParms.eLinkType = ELinkType.RTL8139
                elif name == "rtl8169":
                    oLinkParms.eLinkType = ELinkType.RTL8169
                elif name == "ccat":
                    oLinkParms.eLinkType = ELinkType.CCAT
                oLinkParms.eLinkMode = ELinkMode.INTERRUPT if eLinkMode == 0 else ELinkMode.POLLING
                oLinkParms.dwInstance = dwInstance
                oLinkParms.oDefault = DN_EC_T_LINK_PARMS_DEFAULT()
                return oLinkParms

            if name == "udp":
                eLinkMode = int(arLinkLayer[2])
                oLinkParms.eLinkType = ELinkType.UDP
                oLinkParms.eLinkMode = ELinkMode.INTERRUPT if eLinkMode == 0 else ELinkMode.POLLING
                oLinkParms.dwInstance = 0
                oLinkParms.oUdp = DN_EC_T_LINK_PARMS_UDP()
                oLinkParms.oUdp.abyIpAddress = EcDemoApp.str_to_ip_address(arLinkLayer[1])
                return oLinkParms

            if name == "ndis":
                eLinkMode = int(arLinkLayer[2])
                oLinkParms.eLinkType = ELinkType.NDIS
                oLinkParms.eLinkMode = ELinkMode.INTERRUPT if eLinkMode == 0 else ELinkMode.POLLING
                oLinkParms.dwInstance = 0
                oLinkParms.oNdis = DN_EC_T_LINK_PARMS_NDIS()
                oLinkParms.oNdis.abyIpAddress = EcDemoApp.str_to_ip_address(arLinkLayer[1])
                return oLinkParms

            if name == "simulator":
                szEniFilename = arLinkLayer[1]
                dwInstance = int(arLinkLayer[2])
                eLinkMode = int(arLinkLayer[3])
                oLinkParms.eLinkType = ELinkType.Simulator
                oLinkParms.eLinkMode = ELinkMode.INTERRUPT if eLinkMode == 0 else ELinkMode.POLLING
                oLinkParms.dwInstance = dwInstance
                oLinkParms.oSimulator = DN_EC_T_LINK_PARMS_SIMULATOR()
                oLinkParms.oSimulator.szEniFilename = szEniFilename
                oLinkParms.oSimulator.bJobsExecutedByApp = False
                oLinkParms.oSimulator.bConnectHcGroups = True
                return oLinkParms
        except Exception as error:
            raise ValueError("Invalid link layer settings '{}' ({})".format(self.pAppParms.szLinkLayer, error))

        raise ValueError("Unknown link layer settings: '{}'".format(self.pAppParms.szLinkLayer))

    def initializeEtherCATmaster(self):
        if self.pAppParms.tRunMode == RunMode.Master:
            initMasterParams = DN_EC_T_INIT_MASTER_PARMS()
            initMasterParams.oLinkParms = self.createLinkParms()
            initMasterParams.dwBusCycleTimeUsec = self.pAppParms.dwBusCycleTimeUsec
            initMasterParams.dwMaxBusSlaves = EcDemoApp.c_MASTER_CFG_ECAT_MAX_BUS_SLAVES
            initMasterParams.dwMaxAcycFramesQueued = EcDemoApp.c_MASTER_CFG_MAX_ACYC_FRAMES_QUEUED
            initMasterParams.dwMaxAcycBytesPerCycle = EcDemoApp.c_MASTER_CFG_MAX_ACYC_BYTES_PER_CYC
            initMasterParams.dwEcatCmdMaxRetries = EcDemoApp.c_MASTER_CFG_MAX_ACYC_CMD_RETRIES

            initMasterParams.dwLogLevel = DN_EC_LOG_LEVEL(self.pAppParms.dwMasterLogLevel)

            if self.pAppParms.bPerfMeasEnabled:
                initMasterParams.PerfMeasInternalParms = DN_EC_T_PERF_MEAS_INTERNAL_PARMS()
                initMasterParams.PerfMeasInternalParms.bEnabled = True

            oRasParms = None
            if self.pAppParms.wRasServerPort != 0:
                oRasParms = DN_EC_T_INITRASPARAMS()
                oRasParms.wPort = self.pAppParms.wRasServerPort

            self.oEcWrapper = CEcWrapperPythonEx()
            self.onDbgMsgNotificationId = self.oEcWrapper.AddNotificationHandler("onDbgMsg", self.onDbgMsgNotification)
            self.onMasterNotificationId = self.oEcWrapper.AddNotificationHandler("onMaster", self.onMasterNotification)
            self.onRasNotificationId = self.oEcWrapper.AddNotificationHandler("onRas", self.onRasNotification)
            self.oEcWrapper.ThrottleNotification(DN_NotifyCode.CYCCMD_WKC_ERROR, 1000)
            self.oEcWrapper.ThrottleNotification(DN_NotifyCode.NOT_ALL_DEVICES_OPERATIONAL, 1000)
            dwRes = self.oEcWrapper.InitWrapper(0, initMasterParams, oRasParms, None, False)
            if dwRes != ECError.EC_NOERROR:
                self.logMasterError("Cannot initialize EtherCAT-Master: ", dwRes)
                return dwRes
            return ECError.EC_NOERROR

        if self.pAppParms.tRunMode == RunMode.Simulator:
            initSimulatorParams = DN_EC_T_SIMULATOR_INIT_PARMS()
            initSimulatorParams.dwSimulatorAddress = 0
            initSimulatorParams.aoLinkParms[0] = self.createLinkParms()
            initSimulatorParams.dwBusCycleTimeUsec = self.pAppParms.dwBusCycleTimeUsec
            initSimulatorParams.dwLogLevel = DN_EC_LOG_LEVEL(self.pAppParms.dwMasterLogLevel)

            oRasParms = None
            if self.pAppParms.wRasServerPort != 0:
                oRasParms = DN_EC_T_INITRASPARAMS()
                oRasParms.wPort = self.pAppParms.wRasServerPort

            self.oEcWrapper = CEcWrapperPythonEx()
            self.onDbgMsgNotificationId = self.oEcWrapper.AddNotificationHandler("onDbgMsg", self.onDbgMsgNotification)
            self.onMasterNotificationId = self.oEcWrapper.AddNotificationHandler("onMaster", self.onMasterNotification)
            self.onRasNotificationId = self.oEcWrapper.AddNotificationHandler("onRas", self.onRasNotification)
            self.oEcWrapper.ThrottleNotification(DN_NotifyCode.CYCCMD_WKC_ERROR, 1000)
            self.oEcWrapper.ThrottleNotification(DN_NotifyCode.NOT_ALL_DEVICES_OPERATIONAL, 1000)
            dwRes = self.oEcWrapper.InitWrapper(0, None, oRasParms, None, False, True, initSimulatorParams)
            if dwRes != ECError.EC_NOERROR:
                self.logMasterError("Cannot initialize simulator: ", dwRes)
                return dwRes
            return ECError.EC_NOERROR

        if self.pAppParms.tRunMode == RunMode.RasClient:
            oRasParms = DN_EC_T_INITRASPARAMS()
            oRasParms.abyIpAddr = [int(octet) for octet in self.pAppParms.abyRasServerIpAddress.split(".")]
            oRasParms.wPort = self.pAppParms.wRasServerPort
            oRasParms.dwLogLevel = DN_EC_LOG_LEVEL(self.pAppParms.dwMasterLogLevel)

            self.oEcWrapper = CEcWrapperPythonEx()
            self.onDbgMsgNotificationId = self.oEcWrapper.AddNotificationHandler("onDbgMsg", self.onDbgMsgNotification)
            self.onMasterNotificationId = self.oEcWrapper.AddNotificationHandler("onMaster", self.onMasterNotification)
            self.onRasNotificationId = self.oEcWrapper.AddNotificationHandler("onRas", self.onRasNotification)
            dwRes = self.oEcWrapper.InitWrapper(0, None, oRasParms, None, False)
            if dwRes != ECError.EC_NOERROR:
                self.logMasterError("Cannot initialize RAS client: ", dwRes)
                return dwRes
            return ECError.EC_NOERROR

        if self.pAppParms.tRunMode == RunMode.MbxGateway:
            oMbxGatewayParms = DN_EC_T_INIT_MBXGATEWAY_PARMS()
            oMbxGatewayParms.abyIpAddr = [int(octet) for octet in self.pAppParms.abyRasServerIpAddress.split(".")]
            oMbxGatewayParms.wPort = self.pAppParms.wRasServerPort
            oMbxGatewayParms.dwLogLevel = DN_EC_LOG_LEVEL(self.pAppParms.dwMasterLogLevel)

            self.oEcWrapper = CEcWrapperPythonEx()
            self.onDbgMsgNotificationId = self.oEcWrapper.AddNotificationHandler("onDbgMsg", self.onDbgMsgNotification)
            self.onMasterNotificationId = self.oEcWrapper.AddNotificationHandler("onMaster", self.onMasterNotification)
            self.onRasNotificationId = self.oEcWrapper.AddNotificationHandler("onRas", self.onRasNotification)
            dwRes = self.oEcWrapper.InitWrapper(0, None, None, oMbxGatewayParms, False)
            if dwRes != ECError.EC_NOERROR:
                self.logMasterError("Cannot initialize mailbox gateway: ", dwRes)
                return dwRes
            return ECError.EC_NOERROR

        self.logMsg(DN_EC_LOG_LEVEL.ERROR, "Mode is not supported: {0}", self.pAppParms.tRunMode)
        return ECError.EC_INVALIDPARM

    def onDbgMsgNotification(self, type_, severity, msg):
        self.logger.logMsg(severity, "{0}".format(msg.rstrip()))
        return

    def onMasterNotification(self, type_, code, data, errMsgs):
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "Master: Type={0}, Code={1}, Msg={2}".format(type_, code, errMsgs))
        return

    def onRasNotification(self, type_, code, data, errMsgs):
        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "RAS: Type={0}, Code={1}".format(type_, code))
        return

    def deinitializeEtherCATmaster(self):
        if self.pAppParms.tRunMode != RunMode.Unknown:
            self.oReg = None
            dwRes = self.oEcWrapper.UnregisterClient()
            if dwRes != ECError.EC_NOERROR:
                self.logger.logMsg(DN_EC_LOG_LEVEL.ERROR, "Cannot unregister client: %s (0x%lx))\n", self.oEcWrapper.GetErrorText(dwRes), dwRes)
                return dwRes

        if self.pAppParms.tRunMode != RunMode.Unknown:
            if self.onDbgMsgNotificationId != -1:
                self.oEcWrapper.RemoveNotificationHandler(self.onDbgMsgNotificationId)
                self.onDbgMsgNotificationId = -1
            if self.onMasterNotificationId != -1:
                self.oEcWrapper.RemoveNotificationHandler(self.onMasterNotificationId)
                self.onMasterNotificationId = -1
            if self.onRasNotificationId != -1:
                self.oEcWrapper.RemoveNotificationHandler(self.onRasNotificationId)
                self.onRasNotificationId = -1

            dwRes = self.oEcWrapper.DeinitWrapper()
            if dwRes != ECError.EC_NOERROR:
                self.logger.logMsg(DN_EC_LOG_LEVEL.ERROR, "Cannot de-initialize EtherCAT-Master: %s (0x%lx)\n", self.oEcWrapper.GetErrorText(dwRes), dwRes)
                return ECError.EC_NOERROR
        return ECError.EC_NOERROR

    def tEcJobTask(self):
        self.m_bJobTaskRunning = True
        nOverloadCounter = 0

        while not self.m_bJobTaskShutDown:
            try:
                bPrevCycProcessedArr = CEcWrapperPythonOutParam()
                dwRes = self.oEcWrapper.ExecJobProcessAllRxFrames(bPrevCycProcessedArr)

                if ECError.EC_NOERROR != dwRes:
                    if not bPrevCycProcessedArr:
                        #/* it is not reasonable, that more than 5 continuous frames are lost */
                        nOverloadCounter += 10
                        if nOverloadCounter >= 50:
                            if ((self.bPerfMeasEnabled) and (nOverloadCounter < 60)):
                                self.PrintPerfMeas(0)
                            self.logger.logMsg(DN_EC_LOG_LEVEL.ERROR, "Error: System overload: Cycle time too short or huge jitter!\n")
                        else:
                            self.logger.logMsg(DN_EC_LOG_LEVEL.ERROR, "eUsrJob_ProcessAllRxFrames - not all previously sent frames are received/processed (frame loss)!\n")
                    else:
                        #/* everything o.k.? If yes, decrement overload counter */
                        if nOverloadCounter > 0: 
                            nOverloadCounter -= 1

                if self.pAppParms.tRunMode == RunMode.Simulator:
                    self.myAppWorkpd()

                    #/* run the simulator timer handler */
                    self.oEcWrapper.ExecJob(DN_EC_T_USER_JOB.SimulatorTimer)
                else:
                    if self.bPerfMeasEnabled:
                        self.oEcWrapper.PerfMeasAppStart(self.pvPerfMeas, EcDemoApp.PERF_myAppWorkpd)

                    eMasterState = self.oEcWrapper.GetMasterState()
                    if eMasterState in {DN_EC_T_STATE.SAFEOP, DN_EC_T_STATE.OP}:
                        self.myAppWorkpd()

                    if self.bPerfMeasEnabled:
                        self.oEcWrapper.PerfMeasAppEnd(self.pvPerfMeas, EcDemoApp.PERF_myAppWorkpd)

                    #/* write output values from current cycle, by sending all cyclic frames */
                    #/* send all cyclic frames (write new output values) */
                    self.oEcWrapper.ExecJob(DN_EC_T_USER_JOB.SendAllCycFrames)

                    #/* run the master timer handler */
                    self.oEcWrapper.ExecJob(DN_EC_T_USER_JOB.MasterTimer)

                    #/* send all queued acyclic EtherCAT frames */
                    self.oEcWrapper.ExecJob(DN_EC_T_USER_JOB.SendAcycFrames)
            except CEcWrapperPythonException:
                pass

            #/* Wait for the next cycle */
            time.sleep(self.m_nCycleTime / 1000)

        self.m_bJobTaskRunning = False


    def runDemo(self):
        endTimeInMs = 0
        if self.pAppParms.dwDemoDuration != 0:
            endTimeInMs = time.time()*1000.0 + self.pAppParms.dwDemoDuration
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "%s runtime: %ds ...\n", EC_DEMO_APP_NAME, self.pAppParms.dwDemoDuration / 1000)
        try:
            while True:
                if endTimeInMs > 0 and time.time() * 1000.0 >= endTimeInMs:
                    break
                self.myAppDiagnosis()
                time.sleep(5 / 1000)
        except KeyboardInterrupt:
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "Stopped by user")
        return ECError.EC_NOERROR


    @staticmethod
    def is_bit_set(value, bit):
        return (1 << bit) == (value & (1 << bit))


    def printSlaveInfos(self):
        #/* get information about all bus slaves */
        for i in range(65535, 0, -1):
            wAutoIncAddress = i + 1
            oBusSlaveInfoArr = CEcWrapperPythonOutParam()

            #/* get bus slave information */
            try:
                dwRes = self.oEcWrapper.GetBusSlaveInfo(False, wAutoIncAddress, oBusSlaveInfoArr)
                if dwRes != ECError.EC_NOERROR:
                    break
            except CEcWrapperPythonException as error:
                if error.code == ECError.EC_NOTFOUND:
                    break
                raise

            oBusSlaveInfo = oBusSlaveInfoArr.value
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "******************************************************************************\n")
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "Slave ID............: 0x%08X\n", oBusSlaveInfo.dwSlaveId)
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "Bus Index...........: %d\n", (0 - wAutoIncAddress))
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "Bus AutoInc Address.: 0x%04x (%4d)\n", oBusSlaveInfo.wAutoIncAddress, oBusSlaveInfo.wAutoIncAddress)
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "Bus Station Address.: 0x%04x (%4d)\n", oBusSlaveInfo.wStationAddress, oBusSlaveInfo.wStationAddress)
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "Bus Alias Address...: 0x%04x (%4d)\n", oBusSlaveInfo.wAliasAddress, oBusSlaveInfo.wAliasAddress)
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "Vendor ID...........: 0x%08X = %s\n", oBusSlaveInfo.dwVendorId, CEcWrapperPython.SlaveVendorText(oBusSlaveInfo.dwVendorId))
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "Product Code........: 0x%08X = %s\n", oBusSlaveInfo.dwProductCode, CEcWrapperPython.SlaveProdCodeText(oBusSlaveInfo.dwVendorId, oBusSlaveInfo.dwProductCode))
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "Revision............: 0x%08X   Serial Number: %d\n", oBusSlaveInfo.dwRevisionNumber, oBusSlaveInfo.dwSerialNumber)
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "ESC Type............: %s (0x%x)  Revision: %d  Build: %d\n", CEcWrapperPython.ESCTypeText(oBusSlaveInfo.byESCType, True), oBusSlaveInfo.byESCType, oBusSlaveInfo.byESCRevision, oBusSlaveInfo.wESCBuild)
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "Connection at Port A: %s (to 0x%08X)\n", "yes" if self.is_bit_set(oBusSlaveInfo.wPortState, 0) else "no", oBusSlaveInfo.adwPortSlaveIds[0])
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "Connection at Port D: %s (to 0x%08X)\n", "yes" if self.is_bit_set(oBusSlaveInfo.wPortState, 3) else "no", oBusSlaveInfo.adwPortSlaveIds[3])
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "Connection at Port B: %s (to 0x%08X)\n", "yes" if self.is_bit_set(oBusSlaveInfo.wPortState, 1) else "no", oBusSlaveInfo.adwPortSlaveIds[1])
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "Connection at Port C: %s (to 0x%08X)\n", "yes" if self.is_bit_set(oBusSlaveInfo.wPortState, 2) else "no", oBusSlaveInfo.adwPortSlaveIds[2])
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "Line Crossed........: %s\n", "yes" if oBusSlaveInfo.bLineCrossed else "no")
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "Line Crossed Flags..: 0x%x\n", oBusSlaveInfo.wLineCrossedFlags)

            oCfgSlaveInfoArr = CEcWrapperPythonOutParam()

            #/* get cfg slave information (matching bus slave) */
            dwRes = self.oEcWrapper.GetCfgSlaveInfo(True, oBusSlaveInfo.wStationAddress, oCfgSlaveInfoArr)
            if dwRes != ECError.EC_NOERROR:
                continue

            oCfgSlaveInfo = oCfgSlaveInfoArr.value

            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "Cfg Station Address.: 0x%04x (%4d)\n", oCfgSlaveInfo.wStationAddress, oCfgSlaveInfo.wStationAddress)
            if oCfgSlaveInfo.dwPdSizeIn:
                self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "PD IN    Byte.Bit offset: %d.%d   Size: %d bits\n", oCfgSlaveInfo.dwPdOffsIn // 8, oCfgSlaveInfo.dwPdOffsIn % 8, oCfgSlaveInfo.dwPdSizeIn)
            if oCfgSlaveInfo.dwPdSizeOut:
                self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "PD OUT   Byte.Bit offset: %d.%d   Size: %d bits\n", oCfgSlaveInfo.dwPdOffsOut // 8, oCfgSlaveInfo.dwPdOffsOut % 8, oCfgSlaveInfo.dwPdSizeOut)

            if oCfgSlaveInfo.dwPdSizeIn2:
                self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "PD IN  2 Byte.Bit offset: %d.%d   Size: %d bits\n", oCfgSlaveInfo.dwPdOffsIn2 // 8, oCfgSlaveInfo.dwPdOffsIn2 % 8, oCfgSlaveInfo.dwPdSizeIn2)
            if oCfgSlaveInfo.dwPdSizeOut2:
                self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "PD OUT 2 Byte.Bit offset: %d.%d   Size: %d bits\n", oCfgSlaveInfo.dwPdOffsOut2 // 8, oCfgSlaveInfo.dwPdOffsOut2 % 8, oCfgSlaveInfo.dwPdSizeOut2)

            if oCfgSlaveInfo.dwPdSizeIn3:
                self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "PD IN  3 Byte.Bit offset: %d.%d   Size: %d bits\n", oCfgSlaveInfo.dwPdOffsIn3 // 8, oCfgSlaveInfo.dwPdOffsIn3 % 8, oCfgSlaveInfo.dwPdSizeIn3)
            if oCfgSlaveInfo.dwPdSizeOut3:
                self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "PD OUT 3 Byte.Bit offset: %d.%d   Size: %d bits\n", oCfgSlaveInfo.dwPdOffsOut3 // 8, oCfgSlaveInfo.dwPdOffsOut3 % 8, oCfgSlaveInfo.dwPdSizeOut3)
            if oCfgSlaveInfo.dwPdSizeIn4:
                self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "PD IN  4 Byte.Bit offset: %d.%d   Size: %d bits\n", oCfgSlaveInfo.dwPdOffsIn4 // 8, oCfgSlaveInfo.dwPdOffsIn4 % 8, oCfgSlaveInfo.dwPdSizeIn4)
            if oCfgSlaveInfo.dwPdSizeOut4:
                self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "PD OUT 4 Byte.Bit offset: %d.%d   Size: %d bits\n", oCfgSlaveInfo.dwPdOffsOut4 // 8, oCfgSlaveInfo.dwPdOffsOut4 % 8, oCfgSlaveInfo.dwPdSizeOut4)

        self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "******************************************************************************\n")
        return True

    #/***************************************************************************************************/
    #/**
    #\brief  PrintPerfMeas
    #*/
    def PrintPerfMeas(self, dwPerfMeasInstanceId0):

        out_dwInternalNumOf = CEcWrapperPythonOutParam()    
        dwRes = self.oEcWrapper.PerfMeasInternalGetNumOfByTaskId(dwPerfMeasInstanceId0, out_dwInternalNumOf)
        if ECError.EC_NOERROR != dwRes:
            return

        dwInternalNumOf = out_dwInternalNumOf.value
        if dwInternalNumOf > 0:
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "============================================================================\n")

            #/* print internal benchmarks */
            if dwInternalNumOf > 0:
                aPerfMeasVal = [DN_EC_T_PERF_MEAS_VAL()] * dwInternalNumOf
                aPerfMeasInfo = [DN_EC_T_PERF_MEAS_INFO()] * dwInternalNumOf

                ref_aPerfMeasVal = CEcWrapperPythonRefParam(aPerfMeasVal)    
                dwRes = self.oEcWrapper.PerfMeasInternalGetRawByTaskId(0, EcDemoApp.EC_PERF_MEAS_ALL, ref_aPerfMeasVal, None, dwInternalNumOf)
                if ECError.EC_NOERROR != dwRes:
                    return
                aPerfMeasVal = ref_aPerfMeasVal.value
                out_aPerfMeasInfo = CEcWrapperPythonOutParam()    
                dwRes = self.oEcWrapper.PerfMeasInternalGetInfoByTaskId(0, EcDemoApp.EC_PERF_MEAS_ALL, out_aPerfMeasInfo, dwInternalNumOf)
                if ECError.EC_NOERROR != dwRes:
                    return
                aPerfMeasInfo = out_aPerfMeasInfo.value
                self.PrintPerfMeasInternal(dwInternalNumOf, aPerfMeasVal, aPerfMeasInfo)

            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "\n")

        out_dwAppNumOf = CEcWrapperPythonOutParam()    
        dwRes = self.oEcWrapper.PerfMeasAppGetNumOf(self.pvPerfMeas, out_dwAppNumOf)
        if ECError.EC_NOERROR != dwRes:
            return

        dwAppNumOf = out_dwAppNumOf.value
        if dwAppNumOf > 0:
            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "============================================================================\n")

            aPerfMeasVal = [DN_EC_T_PERF_MEAS_VAL()] * dwAppNumOf
            aPerfMeasInfo = [DN_EC_T_PERF_MEAS_INFO()] * dwAppNumOf

            ref_aPerfMeasVal = CEcWrapperPythonRefParam(aPerfMeasVal)    
            dwRes = self.oEcWrapper.PerfMeasAppGetRaw(self.pvPerfMeas, EcDemoApp.EC_PERF_MEAS_ALL, ref_aPerfMeasVal, None, dwAppNumOf)
            if ECError.EC_NOERROR != dwRes:
                return
            aPerfMeasVal = ref_aPerfMeasVal.value
            out_aPerfMeasInfo = CEcWrapperPythonOutParam()    
            dwRes = self.oEcWrapper.PerfMeasAppGetInfo(self.pvPerfMeas, EcDemoApp.EC_PERF_MEAS_ALL, out_aPerfMeasInfo, dwAppNumOf)
            if ECError.EC_NOERROR != dwRes:
                return
            aPerfMeasInfo = out_aPerfMeasInfo.value
            self.PrintPerfMeasInternal(dwAppNumOf, aPerfMeasVal, aPerfMeasInfo)

            self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "\n")

        return ECError.EC_NOERROR

    #/***************************************************************************************************/
    #/**
    #\brief  PrintPerfMeasInternal
    #*/
    def PrintPerfMeasInternal(self, dwNumOf, aPerfMeasVal, aPerfMeasInfo):
        for dwPerfMeasIdx in range(0, dwNumOf):
            pPerfMeasVal = aPerfMeasVal[dwPerfMeasIdx]
            pPerfMeasInfo = aPerfMeasInfo[dwPerfMeasIdx]
            qwFrequency = pPerfMeasInfo.qwFrequency / (10 * 1000) #/* 1/10 usec */
            if qwFrequency == 0:
                continue
            qwMin = pPerfMeasVal.qwMinTicks * 1000 / qwFrequency
            qwAvg = pPerfMeasVal.qwAvgTicks * 1000 / qwFrequency
            qwMax = pPerfMeasVal.qwMaxTicks * 1000 / qwFrequency
            if qwMax > 0:
                self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "PerfMsmt '%s' (min/avg/max) [usec]: %4d.%d/%4d.%d/%4d.%d\n",
                    pPerfMeasInfo.szName,
                    str(int(qwMin / 10)).rjust(4, ), int(qwMin % 10),
                    str(int(qwAvg / 10)).rjust(4, ), int(qwAvg % 10),
                    str(int(qwMax / 10)).rjust(4, ), int(qwMax % 10))

    #/***************************************************************************************************/
    #/**
    #\brief  Initialize Application
    #
    #\return EC_E_NOERROR on success, error code otherwise. todo update
    #*/
    def myAppInit(self):
        self.pMyAppDesc.wFlashSlaveAddr = self.pAppParms.wFlashSlaveAddr
        return ECError.EC_NOERROR

    #/***************************************************************************************************/
    #/**
    #\brief  Initialize Slave Instance.

    #Find slave parameters.
    #\return EC_E_NOERROR on success, error code otherwise. //todo
    #*/
    def myAppPrepare(self):

        if self.pMyAppDesc.wFlashSlaveAddr != 0xFFFF:
            #/* check if slave address is provided */
            if self.pMyAppDesc.wFlashSlaveAddr != 0:
                wFixedAddress = self.pMyAppDesc.wFlashSlaveAddr
                inputs = True if self.pAppParms.tRunMode == RunMode.Simulator else False

                # Flash first variable of slave with specific data type
                self.pMyAppDesc.oFlashVariable = None
                for slave in self.processImageRt.slaves:
                    if slave.address == wFixedAddress:
                        for variable in slave.variables:
                            wDataType = DN_EC_T_DEFTYPE(variable.wDataType)
                            if wDataType == DN_EC_T_DEFTYPE.BOOLEAN:
                                self.pMyAppDesc.oFlashVariable = self.processImageRt.getVariableByName(variable.szName)
                                break
                            if wDataType == DN_EC_T_DEFTYPE.UNSIGNED8 or wDataType == DN_EC_T_DEFTYPE.UNSIGNED16 or wDataType == DN_EC_T_DEFTYPE.UNSIGNED32:
                                self.pMyAppDesc.oFlashVariable = self.processImageRt.getVariableByName(variable.szName)
                                break
                            if wDataType == DN_EC_T_DEFTYPE.INTEGER8 or wDataType == DN_EC_T_DEFTYPE.INTEGER16 or wDataType == DN_EC_T_DEFTYPE.INTEGER32:
                                self.pMyAppDesc.oFlashVariable = self.processImageRt.getVariableByName(variable.szName)
                                break

                if self.pMyAppDesc.oFlashVariable == None:
                    self.logger.logMsg(DN_EC_LOG_LEVEL.ERROR, "Slave address=%d has no supported %s variable, therefore flashing not possible\n", wFixedAddress, "input" if inputs else "output")

    #/***************************************************************************************************/
    #/**
    #\brief  Setup slave parameters (normally done in PREOP state

    #  - SDO up- and Downloads
    #  - Read Object Dictionary

    #\return EC_E_NOERROR on success, error code otherwise.
    #*/
    def myAppSetup(self):
        pass

    def myAppWorkpd(self):
        """
        demo application working process data function.
        This function is called in every cycle after the the master stack is started.
        """
        #/* demo code flashing */
        if self.pMyAppDesc.oFlashVariable != None:

            self.pMyAppDesc.dwFlashTimer += self.pAppParms.dwBusCycleTimeUsec
            if self.pMyAppDesc.dwFlashTimer >= self.pMyAppDesc.dwFlashInterval:
                self.pMyAppDesc.dwFlashTimer = 0

                #/* flash / sawtooth wave */
                dwFlashVal = self.pMyAppDesc.oFlashVariable.get() + 1
                if dwFlashVal > (2 ** self.pMyAppDesc.oFlashVariable.size()) - 1:
                    dwFlashVal = 0

                #/* update process data image */
                self.pMyAppDesc.oFlashVariable.set(dwFlashVal)

        return ECError.EC_NOERROR

    #/***************************************************************************************************/
    #/**
    #\brief  demo application doing some diagnostic tasks

    #  This function is called in sometimes from the main demo task
    #*/
    def myAppDiagnosis(self):
        return ECError.EC_NOERROR

    def startDemo(self):
        dwRes = ECError.EC_NOERROR
        try:
            self.setAppAndMasterLogLevel()
            if self.logger.initLogger(self.pAppParms.dwAppLogLevel, self.pAppParms.szLogFileprefix) == False:
                return None

            dwRes = self.myAppInit()
            if dwRes != ECError.EC_NOERROR:
                self.logger.logMsg(DN_EC_LOG_LEVEL.ERROR, "myAppInit failed: %s (0x%lx))\n", self.oEcWrapper.GetErrorText(dwRes), dwRes)
                return dwRes
            dwRes = self.initializeEtherCATmaster()
            if dwRes != ECError.EC_NOERROR:
                return dwRes

            #/* initalize performance measurement */
            if self.pAppParms.bPerfMeasEnabled:
                oPerfMeasAppParms = DN_EC_T_PERF_MEAS_APP_PARMS()
                oPerfMeasAppParms.dwNumMeas = EcDemoApp.MAX_JOB_NUM
                oPerfMeasAppParms.aPerfMeasInfos = [ DN_EC_T_PERF_MEAS_INFO_PARMS() ] * 20
                for i in range(len(EcDemoApp.S_aPerfMeasInfos)):
                    oPerfMeasAppParms.aPerfMeasInfos[i].szName = EcDemoApp.S_aPerfMeasInfos[i]

                pvPerfMeas_out = CEcWrapperPythonOutParam()    
                dwRes = self.oEcWrapper.PerfMeasAppCreate(oPerfMeasAppParms, pvPerfMeas_out)
                if dwRes != ECError.EC_NOERROR:
                    self.logger.logMsg(DN_EC_LOG_LEVEL.ERROR, "ERROR: Cannot initialize app performance measurement: %s (0x%lx))\n", self.oEcWrapper.GetErrorText(dwRes), dwRes)
                    return dwRes
                self.bPerfMeasEnabled = True
                self.pvPerfMeas = pvPerfMeas_out.value

            # /* print MAC address */
            oSrcMacAddressArr = CEcWrapperPythonOutParam()
            dwRes = self.oEcWrapper.GetSrcMacAddress(oSrcMacAddressArr)
            if dwRes != ECError.EC_NOERROR:
                self.logMasterError("Cannot get MAC address: ", dwRes)
            else:
                oSrcMacAddress = oSrcMacAddressArr.value
                self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "EtherCAT network adapter MAC: %02X-%02X-%02X-%02X-%02X-%02X\n", *oSrcMacAddress.b)

            #/* create cyclic task to trigger jobs */
            if (self.pAppParms.tRunMode in {RunMode.Master, RunMode.Simulator}) and self.m_bJobTaskRunning == False:
                self.m_bJobTaskShutDown = False
                self.m_oJobTask = Thread(target=self.tEcJobTask)
                self.m_oJobTask.start()

            if self.pAppParms.szENIFilename:
                if self.pAppParms.tRunMode == RunMode.Master:
                    dwRes = self.oEcWrapper.ConfigureMaster(DN_EC_T_CNF_TYPE.Filename, self.pAppParms.szENIFilename, len(self.pAppParms.szENIFilename))
                    if dwRes != ECError.EC_NOERROR:
                        self.logMasterError("Cannot configure EtherCAT-Master: ", dwRes)
                        return dwRes

                if self.pAppParms.tRunMode == RunMode.Simulator:
                    dwRes = self.oEcWrapper.ConfigureNetwork(DN_EC_T_CNF_TYPE.Filename, self.pAppParms.szENIFilename, len(self.pAppParms.szENIFilename))
                    if dwRes != ECError.EC_NOERROR:
                        self.logMasterError("Cannot configure EtherCAT-Simulator: ", dwRes)
                        return dwRes

            out_oReg = CEcWrapperPythonOutParam()
            dwRes = self.oEcWrapper.RegisterClient(out_oReg)
            if dwRes != ECError.EC_NOERROR:
                self.logMasterError("Cannot register client: ", dwRes)
                return dwRes

            self.oReg = out_oReg.value

            #/* print found slaves */
            if self.pAppParms.dwAppLogLevel.value >= DN_EC_LOG_LEVEL.VERBOSE:
                if self.pAppParms.tRunMode == RunMode.Master:
                    dwRes = self.oEcWrapper.ScanBus(EcDemoApp.ETHERCAT_SCANBUS_TIMEOUT)
                if dwRes in {ECError.EC_NOERROR, ECError.EC_BUSCONFIG_MISMATCH, ECError.EC_LINE_CROSSED}:
                    self.printSlaveInfos()
                else:
                    self.logMasterError("Cannot scan bus: ", dwRes)

            self.processImage.reload()
            self.processImageRt.reload()

            if self.pAppParms.tRunMode == RunMode.Master and self.pAppParms.szENIFilename != "":
                dwRes = self.oEcWrapper.SetMasterState(EcDemoApp.ETHERCAT_STATE_CHANGE_TIMEOUT, DN_EC_T_STATE.INIT)
                if dwRes != ECError.EC_NOERROR:
                    self.logMasterError("Cannot set master state to INIT: ", dwRes)
                    return dwRes

                self.myAppPrepare()

                dwRes = self.oEcWrapper.SetMasterState(EcDemoApp.ETHERCAT_STATE_CHANGE_TIMEOUT, DN_EC_T_STATE.PREOP)
                if dwRes != ECError.EC_NOERROR:
                    self.logMasterError("Cannot set master state to PREOP: ", dwRes)
                    return dwRes

                self.myAppSetup()

                dwRes = self.oEcWrapper.SetMasterState(EcDemoApp.ETHERCAT_STATE_CHANGE_TIMEOUT, DN_EC_T_STATE.SAFEOP)
                if dwRes != ECError.EC_NOERROR:
                    self.logMasterError("Cannot set master state to SAFEOP: ", dwRes)
                    return dwRes

                dwRes = self.oEcWrapper.SetMasterState(EcDemoApp.ETHERCAT_STATE_CHANGE_TIMEOUT, DN_EC_T_STATE.OP)
                if dwRes != ECError.EC_NOERROR:
                    self.logMasterError("Cannot set master state to OP: ", dwRes)
                    return dwRes
            elif self.pAppParms.tRunMode == RunMode.Simulator:
                self.myAppPrepare()
                self.myAppSetup()

            if self.bPerfMeasEnabled:
                self.logMsg(DN_EC_LOG_LEVEL.INFO, "\nJob times during startup <INIT> to <%s>:\n", self.oEcWrapper.GetMasterState())
                self.PrintPerfMeas(0)
                self.logMsg(DN_EC_LOG_LEVEL.INFO, "\n")
                #/* clear job times of startup phase */
                self.oEcWrapper.PerfMeasAppReset(self.pvPerfMeas, EcDemoApp.EC_PERF_MEAS_ALL)
                self.oEcWrapper.PerfMeasInternalResetByTaskId(0, EcDemoApp.EC_PERF_MEAS_ALL)

            #self.processImage.variables.Slave_1005__EL2008_.Channel_1.Output.set(1)
            #self.processImage.variables.Slave_1005__EL2008_.Channel_1.Output.get()
        except CEcWrapperPythonException as e:
            self.logger.logMsg(DN_EC_LOG_LEVEL.ERROR, str(e))

        return dwRes


    def stopDemo(self):

        #/* set master state to INIT */
        if DN_EC_T_STATE.UNKNOWN != self.oEcWrapper.GetMasterState():
            if self.bPerfMeasEnabled:
                self.logger.logMsg(DN_EC_LOG_LEVEL.INFO, "\nJob times before shutdown\n")
                self.PrintPerfMeas(0)

        if self.pAppParms.tRunMode == RunMode.Master:
            dwRes = self.oEcWrapper.SetMasterState(EcDemoApp.ETHERCAT_STATE_CHANGE_TIMEOUT, DN_EC_T_STATE.INIT)
            if dwRes != ECError.EC_NOERROR:
                self.logger.logMsg(DN_EC_LOG_LEVEL.ERROR, "Cannot start set master state to INIT: %s (0x%lx))\n", self.oEcWrapper.GetErrorText(dwRes), dwRes)
                return dwRes

        #/* shutdown JobTask */
        if self.m_oJobTask != None:
            self.m_bJobTaskShutDown = True
            while self.m_bJobTaskRunning:
                time.sleep(50 / 1000)
            self.m_oJobTask.join()
            self.m_oJobTask = None
        #/* deinitialize master */
        dwRes = self.deinitializeEtherCATmaster()
        if dwRes != ECError.EC_NOERROR:
            return dwRes

        return ECError.EC_NOERROR

    def help(self):
        print("Interactive help:")
        print("- demo.startDemo(): start demo")
        print("- demo.stopDemo(): stop demo")
        print("- demo.pAppParms: demo parameters")
        print("- demo.processImage: process image")
        print("  - e.g. write variable")
        print("    demo.processImage.variables.Slave_1005__EL2008_.Channel_1.Output.set(1)")
        print("  - e.g. read variable")
        print("    demo.processImage.variables.Slave_1005__EL2008_.Channel_1.Output.get()")
        return

    def main(self, argv):
        if self.parseCommandLine(argv) == False:
            return
        dwRes = self.startDemo()
        if dwRes != ECError.EC_NOERROR:
            return
        dwRes = self.runDemo()
        if dwRes != ECError.EC_NOERROR:
            return
        dwRes = self.stopDemo()
        if dwRes != ECError.EC_NOERROR:
            return

class EcMasterDemoPython(EcDemoApp):
    def __init__(self):
        EcDemoApp.__init__(self)

if __name__ == "__main__":
    CEcWrapperPython.EnableExceptionHandling = False # True, to throw exception in case of error
    demo = EcMasterDemoPython()
    demo.main(sys.argv[1:])

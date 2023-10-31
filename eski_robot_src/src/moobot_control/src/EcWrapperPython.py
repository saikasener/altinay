#/*-----------------------------------------------------------------------------
# * EcWrapperPython.py
# * Copyright                acontis technologies GmbH, Ravensburg, Germany
# * Description              EC-Master Python Interface
# *---------------------------------------------------------------------------*/
# pylint: disable=anomalous-backslash-in-string, unused-wildcard-import, wildcard-import
from EcWrapperPythonTypes import *
from EcWrapperTypes import *
from EcWrapper import *
import os
import sys
import platform
import datetime
import threading
import inspect

# Native internal function pointers
NativeEcEventNotif = ctypes.CFUNCTYPE(ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p)
NativeCycFrameRxEvent = ctypes.CFUNCTYPE(None, ctypes.c_uint, ctypes.c_void_p)
NativeRapiNotif = ctypes.CFUNCTYPE(ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p)
NativePerfNotif = ctypes.CFUNCTYPE(ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint)
NativeTranslateNotif = ctypes.CFUNCTYPE(ctypes.c_void_p, ctypes.c_uint, SDN_EC_STRING_HLP)
NativeDbgMsgNotif = ctypes.CFUNCTYPE(ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint, ctypes.c_char_p)


class ThrottleElem:
    def __init__(self):
        self.tNotifyCode = DN_NotifyCode.STATECHANGED # DN_NotifyCode
        self.nLastSeen = 0 # int64
        self.dwTimeout = 0 # uint

class CEcWrapperPython:
    REMOTE_CYCLE_TIME = 2
    REMOTE_WD_TO_LIMIT = 10000
    REMOTE_RECV_THREAD_PRIO = 0 #//((EC_T_DWORD) THREAD_PRIORITY_NORMAL)
    MAIN_THREAD_PRIO = 0 #//((EC_T_DWORD) THREAD_PRIORITY_NORMAL)
    ECWRAPPER_EVAL_VERSION = True
    ECWRAPPER_API_VERSION = 1659626703
    EnableExceptionHandling = False
    m_dwRasConnectionCounter = 0
    m_dwMbxGatewayConnectionCounter = 0
    m_szInstallDir = ""
    DATE_SINCE_1970 = datetime.datetime(1970,1,1)
    m_oInstances = []
    m_oInstancesLock = threading.Lock()

    @staticmethod
    def s_ATECAT_VERSION():
        return CEcWrapper.Get().ecwGetDefaultValue(EC_T_DEFALT_VALUE.EC_T_DEFALT_VALUE_ATECAT_VERSION) if CEcWrapperPython.IsEcWrapperInstalled() else 0

    @staticmethod
    def s_ATECAT_SIGNATURE():
        return CEcWrapper.Get().ecwGetDefaultValue(EC_T_DEFALT_VALUE.EC_T_DEFALT_VALUE_ATECAT_SIGNATURE) if CEcWrapperPython.IsEcWrapperInstalled() else 0

    @staticmethod
    def s_INVALID_SLAVE_ID():
        return CEcWrapper.Get().ecwGetDefaultValue(EC_T_DEFALT_VALUE.EC_T_DEFALT_VALUE_INVALID_SLAVE_ID) if CEcWrapperPython.IsEcWrapperInstalled() else 0

    @staticmethod
    def s_MASTER_SLAVE_ID():
        return CEcWrapper.Get().ecwGetDefaultValue(EC_T_DEFALT_VALUE.EC_T_DEFALT_VALUE_MASTER_SLAVE_ID) if CEcWrapperPython.IsEcWrapperInstalled() else 0

    @staticmethod
    def s_MAX_NUMOF_MASTER_INSTANCES():
        return CEcWrapper.Get().ecwGetDefaultValue(EC_T_DEFALT_VALUE.EC_T_DEFALT_VALUE_MAX_NUMOF_MASTER_INSTANCES) if CEcWrapperPython.IsEcWrapperInstalled() else 0

    @staticmethod
    def s_MASTER_RED_SLAVE_ID():
        return CEcWrapper.Get().ecwGetDefaultValue(EC_T_DEFALT_VALUE.EC_T_DEFALT_VALUE_MASTER_RED_SLAVE_ID) if CEcWrapperPython.IsEcWrapperInstalled() else 0

    @staticmethod
    def s_EL9010_SLAVE_ID():
        return CEcWrapper.Get().ecwGetDefaultValue(EC_T_DEFALT_VALUE.EC_T_DEFALT_VALUE_EL9010_SLAVE_ID) if CEcWrapperPython.IsEcWrapperInstalled() else 0

    @staticmethod
    def s_FRAMELOSS_SLAVE_ID():
        return CEcWrapper.Get().ecwGetDefaultValue(EC_T_DEFALT_VALUE.EC_T_DEFALT_VALUE_FRAMELOSS_SLAVE_ID) if CEcWrapperPython.IsEcWrapperInstalled() else 0

    @staticmethod
    def s_JUNCTION_RED_FLAG():
        return CEcWrapper.Get().ecwGetDefaultValue(EC_T_DEFALT_VALUE.EC_T_DEFALT_VALUE_JUNCTION_RED_FLAG) if CEcWrapperPython.IsEcWrapperInstalled() else 0

    def __init__(self):
        self.notificationHandlers = []
        self.notificationHandlerId = 0
        self.m_bRasClient = False

        self.m_dwClientId = 0
        self.m_dwRasCookie = 0
        self.m_dwMasterInstanceId = ctypes.c_uint(0)
        self.m_eLastScanBusRes = ECError.EC_NOTFOUND
        self.m_tRunMode = EcRunMode.None_

        self.m_pRemoteApiSrvHandle = ctypes.c_void_p(None)
        self.m_pMbxGatewaySrvHandle = ctypes.c_void_p(None)
        self.m_pvRasConnectionHandle = ctypes.c_void_p(None)
        self.m_pSimulatorRemoteApiSrvHandle = ctypes.c_void_p(None)
        self.m_pMonitorRemoteApiSrvHandle = ctypes.c_void_p(None)

        self.m_pvTimingEvent = None
        self.m_pEoe = ctypes.c_void_p(None)
        self.m_pTscMeasDesc = ctypes.c_void_p(None)

        self.onEcNotificationId = self.AddNotificationHandler("onMaster", self.OnHandleEcNotification)
        self.onRasNotificationId = self.AddNotificationHandler("onRas", self.OnHandleRasNotification)

        self.m_pfNativEcEvent = ctypes.cast(NativeEcEventNotif(self.ThrowEcEvent), ctypes.c_void_p)
        self.m_pfNativCycFrameRxEvent = ctypes.cast(NativeCycFrameRxEvent(self.ThrowCycFrameRxEvent), ctypes.c_void_p)
        self.m_pfnCycFrameRxCallback = None
        self.m_pfNativRasEvent = ctypes.cast(NativeRapiNotif(self.ThrowRasEvent), ctypes.c_void_p)
        self.m_pfNativTranslateEvent = ctypes.cast(NativeTranslateNotif(self.ThrowTranslateEvent), ctypes.c_void_p)
        self.m_pfNativPerfEvent = ctypes.cast(NativePerfNotif(self.ThrowPerfEvent), ctypes.c_void_p)
        self.m_pfNativDbgMsgEvent = ctypes.cast(NativeDbgMsgNotif(self.ThrowDbgMsgEvent), ctypes.c_void_p)

        self.m_cThrottleQueue = [] #List<ThrottleElem>()
        #self.IntPtr pfTranslationCallback = Marshal.GetFunctionPointerForDelegate(m_pfNativTranslateEvent)
        #self.CEcWrapper.Get().ecwEnableTranslation(pfTranslationCallback)

    def AddNotificationHandler(self, name, cb):
        if name not in {"onMaster", "onRas", "onTranslate", "onPerf", "onDbgMsg", "onApp"}:
            return -1
        self.notificationHandlerId = self.notificationHandlerId + 1
        notificationHandler = {
          "id": self.notificationHandlerId,
          "name": name,
          "cb": cb,
        }
        self.notificationHandlers.append(notificationHandler)
        return self.notificationHandlerId

    def RemoveNotificationHandler(self, id_):
        for i in range(len(self.notificationHandlers)):
            if self.notificationHandlers[i]["id"] == id_:
                self.notificationHandlers.pop(i)
                return True
        return False

    def OnNotificationHandler(self, name, *args):
        for notificationHandler in self.notificationHandlers:
            if notificationHandler["name"] == name:
                notificationHandler["cb"](*args)

    def HasNotificationHandler(self, name):
        for notificationHandler in self.notificationHandlers:
            if notificationHandler["name"] == name:
                return True
        return False


    def ThrowEcEvent(self, dwCode, unmParms):
        """
        Throws the EtherCAT-Notifications. The Type of the event depends on the notification code.
        """
        try:
            EC_NOTIFY_APP = 0x00080000 # const uint
            EC_NOTIFY_APP_MAX_CODE = 0x0000FFFF # const uint
            code = DN_NotifyCode.UNDEFINED if dwCode >= EC_NOTIFY_APP and dwCode <= EC_NOTIFY_APP + EC_NOTIFY_APP_MAX_CODE else DN_NotifyCode(dwCode)
            if self.IsThrottledNotification(code):
                return 0

            unmParamType = NotificationDataType(CEcWrapperTypes.GetNotificationDataType(code)) #CEcWrapperTypes.NotificationDataType

            pbyInBuf = ctypes.c_void_p(None) # IntPtr
            dwInBufSize = ctypes.c_uint(0)
            eRes = CEcWrapper.Get().ecwGetNotificationData(dwCode, unmParamType.value, ctypes.c_void_p(unmParms), ctypes.byref(pbyInBuf), ctypes.byref(dwInBufSize))
            if eRes != ECError.EC_NOERROR:
                return 0

            if dwCode >= EC_NOTIFY_APP and dwCode <= EC_NOTIFY_APP + EC_NOTIFY_APP_MAX_CODE:
                if self.HasNotificationHandler("onApp"):
                    out_inData = CEcWrapperPythonOutParam()
                    CEcWrapperTypes.ReadPdByteFromAddress(pbyInBuf, 0, dwInBufSize, out_inData)
                    inData = out_inData.value

                    out_outData = CEcWrapperPythonOutParam()

                    #// Notify application synchronously!
                    self.OnNotificationHandler("onApp", dwCode - EC_NOTIFY_APP, inData, out_outData)

                    outData = out_outData.value # byte[]
                    if outData is not None:
                        pOutData = CEcWrapperTypes.Conv_IntArrayToBytePtr(outData) # IntPtr
                        CEcWrapper.Get().ecwSetNotificationData(unmParms, pOutData, len(outData))

                return 0

            obj = CEcWrapperTypes.ConvNotificationData(code, pbyInBuf) # object
            errMsgs = self.GetNotificationErrMsg(False, dwCode, unmParms)

            if self.HasNotificationHandler("onMaster"):
                notifyType = DN_NotifyType(CEcWrapper.Get().ecwParseNotificationType(dwCode))
                self.OnNotificationHandler("onMaster", notifyType, code, obj, errMsgs)
        except:
            self.ReportException("ThrowEcEvent(dwCode={0})".format(DN_NotifyCode(dwCode)), DN_EC_LOG_TYPE.MASTER, sys.exc_info()[0])
        return 0

    def SetCycFrameRxCallback(self, pfnCallback):
        """
        Set cyclic frame RX callback function
        
        This function will be called after the cyclic frame is received, if there is more than
        one cyclic frame after the last frame. The application has to assure that these functions
        will not block.

        Args:
            pfnCallback: Callback function

        Returns:
            EC_E_NOERROR or error code
        """
        self.m_pfnCycFrameRxCallback = pfnCallback

        pbyInBuf = SDN_EC_T_CYCFRAME_RX_CBDESC()
        pbyInBuf.pCallbackContext = ctypes.c_void_p(0)

        if pfnCallback == None:
            pbyInBuf.pfnCallback = ctypes.c_void_p(0)
        else:
            pbyInBuf.pfnCallback = self.m_pfNativCycFrameRxEvent

        dwCode = DN_EC_T_IOCTL.REGISTER_CYCFRAME_RX_CB
        pbyInBufPin = ctypes.byref(pbyInBuf)
        dwInBufSize = ctypes.sizeof(pbyInBuf)
        pbyOutBufPin = None
        dwOutBufSize = ctypes.c_uint(0)
        pdwNumOutData = ctypes.c_uint(0)

        eRes = self.ConvResAsError(CEcWrapper.Get().ecwIoControl2(self.m_dwMasterInstanceId, dwCode, pbyInBufPin, dwInBufSize, pbyOutBufPin, dwOutBufSize, ctypes.byref(pdwNumOutData)))
        return self.ReportErrorCode(eRes)

    def ThrowCycFrameRxEvent(self, dwTaskId, _pCallbackContext):
        try:
            if None != self.m_pfnCycFrameRxCallback:
                self.m_pfnCycFrameRxCallback(dwTaskId)

        except:
            self.ReportException("ThrowCycFrameRxEvent(dwTaskId={0})".format(dwTaskId), DN_EC_LOG_TYPE.MASTER, sys.exc_info()[0])
        return 0

    def ThrowRasEvent(self, dwCode, unmParms):
        """
        Throws the RAS Events. The Type of the event depends on the notification code.
        """
        try:
            pbyInBuf = ctypes.c_void_p(None) # IntPtr
            dwInBufSize = ctypes.c_uint(0)
            eRes = CEcWrapper.Get().ecwGetNotificationData(dwCode, NotificationDataType.Default.value, ctypes.c_void_p(unmParms), ctypes.byref(pbyInBuf), ctypes.byref(dwInBufSize))
            if eRes != ECError.EC_NOERROR:
                return 0

            code = DN_RasNotifyCode(dwCode)
            obj = CEcWrapperTypes.ConvRasNotificationData(code, pbyInBuf)
            errMsgs = self.GetNotificationErrMsg(True, dwCode, unmParms)

            notifyType = DN_NotifyType(CEcWrapper.Get().ecwParseNotificationType(dwCode))

            with CEcWrapperPython.m_oInstancesLock:
                dwRasCookie = self.GetRasNotificationCookie(code, obj)
                for instance in CEcWrapperPython.m_oInstances:
                    if instance.HasNotificationHandler("onRas"):
                        if not dwRasCookie:
                            instance.OnNotificationHandler("onRas", notifyType, code, obj, errMsgs)
                            continue

                        if not self.m_dwRasCookie and code == DN_RasNotifyCode.CONNECTION:
                            self.m_dwRasCookie = dwRasCookie

                        elif self.m_dwRasCookie and self.m_dwRasCookie != dwRasCookie:
                            continue

                        instance.OnNotificationHandler("onRas", notifyType, code, obj, errMsgs)

        except:
            self.ReportException("ThrowRasEvent(dwCode={0})".format(DN_RasNotifyCode(dwCode)), DN_EC_LOG_TYPE.RASCLIENT, sys.exc_info()[0])
        return 0


    def GetRasNotificationCookie(self, code, obj):
        if obj is None:
            return 0

        if code == DN_RasNotifyCode.CONNECTION:
            return obj.dwCookie #DN_ATEMRAS_T_CONNOTIFYDESC
        if code in {DN_RasNotifyCode.CONNECTION, DN_RasNotifyCode.UNREGISTER}:
            return obj.dwCookie # DN_ATEMRAS_T_REGNOTIFYDESC
        if code == DN_RasNotifyCode.MARSHALERROR:
            return obj.dwCookie # DN_ATEMRAS_T_MARSHALERRORDESC
        if code in {DN_RasNotifyCode.NONOTIFYMEMORY, DN_RasNotifyCode.STDNOTIFYMEMORYSMALL, DN_RasNotifyCode.MBXNOTIFYMEMORYSMALL}:
            return obj.dwCookie # DN_ATEMRAS_T_NONOTIFYMEMORYDESC

        return 0

    def GetNotificationErrMsg(self, bRas, dwCode, unmParms):
        msg = [] # List<string>
        data = ctypes.c_void_p(None) #IntPtr
        eRes = ECError.EC_NOERROR
        if bRas:
            eRes = CEcWrapper.Get().ecwParseRasNotificationErrMsg(self.m_dwMasterInstanceId, dwCode, ctypes.c_void_p(unmParms), ctypes.byref(data))
        else:
            eRes = CEcWrapper.Get().ecwParseNotificationErrMsg(self.m_dwMasterInstanceId, dwCode, ctypes.c_void_p(unmParms), ctypes.byref(data))

        if eRes == ECError.EC_NOERROR:
            i = 0 # uint
            m = SDN_EC_STRING_HLP()
            eRes2 = self.ConvResAsError(CEcWrapper.Get().ecwGetNotificationErrMsg(data, i, ctypes.byref(m)), True)
            while eRes2 == ECError.EC_NOERROR:
                msg.append(CEcWrapperTypes.Conv_StrFromCharPtr(m.Data))
                i += 1
                eRes2 = self.ConvResAsError(CEcWrapper.Get().ecwGetNotificationErrMsg(data, i, ctypes.byref(m)), True)

            CEcWrapper.Get().ecwFreeNotificationErrMsg(data)
        return msg

    def ThrowPerfEvent(self, pszFktName, dwTime):
        """
        Throws the performance monitoring events.
        """
        if self.HasNotificationHandler("onPerf"):
            self.OnNotificationHandler("onPerf", CEcWrapperTypes.Conv_StrFromCharPtr(pszFktName), dwTime)


    def ThrowTranslateEvent(self, _code, msg):
        """
        Throws the performance monitoring events.
        """
        if self.HasNotificationHandler("onTranslate"):
            self.OnNotificationHandler("onTranslate", CEcWrapperTypes.Conv_StrFromCharPtr(msg))


    def ThrowDbgMsgEvent(self, type_, severity, msg):
        """
        Throws the debug message events.
        """
        if self.HasNotificationHandler("onDbgMsg"):
            self.OnNotificationHandler("onDbgMsg", DN_EC_LOG_TYPE(type_), DN_EC_LOG_LEVEL(severity), CEcWrapperTypes.Conv_StrFromCharPtr(msg))


    def ReportException(self, fname, type_, e):
        if self.HasNotificationHandler("onDbgMsg"):
            self.OnNotificationHandler("onDbgMsg", type_, DN_EC_LOG_LEVEL.CRITICAL, "Exception in '{0}':\n  StackTrace: ".format(fname) + str(e))


    def GetRunMode(self):
        """
        Return run mode

        Returns:
            Run mode
        """
        return self.m_tRunMode


    def OnHandleEcNotification(self, _type_, code, data, _errMsgs):
        """
        Callback function that will be called after the scan bus has been finished.
        The scan bus result will be stored in m_eLastScanBusRes.

        Args:
            type_: Type
            code: Code
            data: Data
            errMsgs: Error messages

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        if code == DN_NotifyCode.SB_STATUS:
            self.m_eLastScanBusRes = ECError(data.dwResultCode)


    def OnHandleRasNotification(self, _type_, code, data, _errMsgs):
        """
        Callback function that will be called after the scan bus has been finished.
        The scan bus result will be stored in m_eLastScanBusRes.

        Args:
            type_: Type
            code: Code
            data: Data
            errMsgs: Error messages

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        if code == DN_RasNotifyCode.CONNECTION:
            if self.m_pvRasConnectionHandle is not None:
                if data.dwCause == ECError.EMRAS_SERVERSTOPPED:
                    self.RasClntRemoveConnection(1000)


    def RegisterClient(self, out_pRegRes):
        """
        Register a client with the EtherCAT Master

        Args:
            out_pRegRes: out Registration results, a pointer to a structure of type REGISTERRESULTS

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        out_pRegRes.value = None
        self.m_dwClientId = 0

        pRegParams = SDN_EC_T_REGISTERPARMS(
            pCallerData=None,
            pfnNotify=self.m_pfNativEcEvent
        )

        gen_pRegRes = SDN_EC_T_REGISTERRESULTS()
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwRegisterClient2(self.m_dwMasterInstanceId, ctypes.byref(pRegParams), ctypes.byref(gen_pRegRes)))
        out_pRegRes.value = CEcWrapperTypes.Conv(gen_pRegRes)

        if gen_dwRetVal == ECError.EC_NOERROR:
            self.m_dwClientId = out_pRegRes.value.dwClntId

        return self.ReportErrorCode(gen_dwRetVal)


    def UnregisterClient(self):
        """
        Unregister a client from the EtherCAT master

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        return self.ConvResAsError(CEcWrapper.Get().ecwUnregisterClient2(self.m_dwMasterInstanceId, self.m_dwClientId))


    def CreateRasSrvParams(self, oParms):
        oRemoteApiConfig = SDN_ATEMRAS_T_SRVPARMS(
            oAddr=(ctypes.c_ubyte * 4)(*oParms.abyIpAddr),
            wPort=oParms.wPort,
            wMaxClientCnt=0,
            dwCycleTime=oParms.dwCycleTime or CEcWrapperPython.REMOTE_CYCLE_TIME,    #/* 100 msec for the begin */
            dwCommunicationTimeout=oParms.dwWDTOLimit or CEcWrapperPython.REMOTE_WD_TO_LIMIT,
            oAcceptorThreadCpuAffinityMask=0,
            dwAcceptorThreadPrio=CEcWrapperPython.MAIN_THREAD_PRIO,
            dwAcceptorThreadStackSize=0x1000,
            oClientWorkerThreadCpuAffinityMask=0,
            dwClientWorkerThreadPrio=CEcWrapperPython.MAIN_THREAD_PRIO,
            dwClientWorkerThreadStackSize=0x1000,
            dwMaxQueuedNotificationCnt=100,                          # for the first pre-allocate 100 Notification spaces
            dwMaxParallelMbxTferCnt=50,                            # for the first pre-allocate 50 Notification spaces
            pfnRasNotify=None,                             # Notification function for emras Layer
            pvRasNotifyCtxt=None,                                # Notification context
            dwCycErrInterval=500,                            # span between to consecutive cyclic notifications of same type
            dwLogLevel=oParms.dwLogLevel.value,
            pfLogMsgCallBack=self.m_pfNativDbgMsgEvent
        )

        return oRemoteApiConfig


    def RasSrvStart(self, oParms):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def RasSrvStop(self, dwTimeout):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def SimulatorRasSrvStart(self, oParms):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def SimulatorRasSrvStop(self, dwTimeout):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def MonitorRasSrvStart(self, oParms):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def MonitorRasSrvStop(self, dwTimeout):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def MbxGatewaySrvStart(self, dwMasterInstanceId, oParms):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def MbxGatewaySrvStop(self, dwTimeout):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def RasClntAddConnection(self, oRasParms):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def RasClntRemoveConnection(self, dwTimeout):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def RasGetConnectionInfo(self, out_pConInfo):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    @staticmethod
    def IsRemoteServerUp(abyIpAddr, wPort):
        """
        Checks if remote server is up by sending a "ping"

        Args:
            abyIpAddr: IPAddress
            wPort:     Port

        Returns:
            bool: True, if server is up, False otherwise
        """
        #// Special: For CeWin, we must skip Port 3, because they use not a "real" IP address!
        if wPort == 3:
            return True

        try:
            hostname = "{}.{}.{}.{}".format(abyIpAddr[0], abyIpAddr[1], abyIpAddr[2], abyIpAddr[3])

            if platform.system() == "Windows":
                response = os.system("ping " + hostname + " -n 1")
            else:
                response = os.system("ping -c 1 " + hostname)

            return response == 0
        except:
            return False


    def MbxGatewayClntAddConnection(self, oMbxGatewayParms):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def MbxGatewayClntRemoveConnection(self):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def MbxGatewayCoeSdoDownload(self, wAddress, wObIndex, byObSubIndex, pbyData, dwDataLen, dwTimeout, dwFlags):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def MbxGatewayCoeSdoUpload(self, wAddress, wObIndex, byObSubIndex, pbyData, dwDataLen, pdwOutDataLen, dwTimeout, dwFlags):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def EnablePerformanceMeasuring(self, bEnable):
        """
        Enables performance monitoring. For every master function, the notification "PerfNotification" will be called.

        Args:
            bEnable: True = Enables performance monitoring
        """
        pfPerfCallback = self.m_pfNativPerfEvent
        CEcWrapper.Get().ecwEnablePerformanceMeasuring(pfPerfCallback if bEnable else ctypes.c_void_p(None))


    def EnableTranslation(self, bEnable):
        """
        Enables translation. For every master string, the notification "TranslateNotification" will be called.

        Args:
            bEnable: True = Enables translation
        """
        pfTranslationCallback = self.m_pfNativTranslateEvent
        CEcWrapper.Get().ecwEnableTranslation(pfTranslationCallback if bEnable else ctypes.c_void_p(None))


    @classmethod
    def GetErrorText(cls, eErrorCode):
        """
        Return text tokens by Error code from master stack.

        Args:
            eErrorCode: Error code

        Returns:
            str: Error text for supplied error code
        """
        if CEcWrapper.IsInitialized():
            data = SDN_EC_STRING_HLP()
            eRes = ECError(CEcWrapper.Get().ecwGetText2(eErrorCode.value, ctypes.byref(data)))
            if eRes == ECError.EC_NOERROR:
                return CEcWrapperTypes.Conv_StrFromCharPtr(data.Data)
        return "Unknown Error 0x{:08X}".format(eErrorCode)


    def ExecJob(self, eUserJob):
        """
        Execute or initiate the requested master job.

        Args:
            eUserJob: Requested job.

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        data = ctypes.c_uint(0)
        return self.ConvResAsError(CEcWrapper.Get().ecwExecDefaultJob(self.m_dwMasterInstanceId, eUserJob.value, ctypes.byref(data)), True)


    def ExecJobProcessAllRxFrames(self, out_bPrevCycProcessed):
        """
        calls the process all rx frame job

        Args:
            bPrevCycProcessed: True: previous send frame was received and processed, False: otherwise

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        bPrevCycProcessedUnm = ctypes.c_uint(0)
        dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwExecDefaultJob(self.m_dwMasterInstanceId, DN_EC_T_USER_JOB.ProcessAllRxFrames, ctypes.byref(bPrevCycProcessedUnm)), True)
        out_bPrevCycProcessed.value = bPrevCycProcessedUnm.value == 1
        return dwRetVal


    def ExecJobSendCycFramesByTaskId(self, dwTaskId):
        """
        sends a cycle frame by its task id

        Args:
            dwTaskId: task id of the cyclic frame

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        dwTaskIdUnm = ctypes.c_uint(dwTaskId)
        dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwExecDefaultJob(self.m_dwMasterInstanceId, DN_EC_T_USER_JOB.SendCycFramesByTaskId, ctypes.byref(dwTaskIdUnm)), True)
        return dwRetVal


    @staticmethod
    def ConvertMasterStringFromBytes(byData, dwOffset, dwLen):
        """
        Converts a master string into UTF8 format

        Args:
            abyData: Byte array to convert
            dwOffset: Offset
            dwLen: Length

        Returns:
            str: Converted string
        """
        return CEcWrapperTypes.ConvertMasterStringFromBytes(byData, dwOffset, dwLen)


    @staticmethod
    def ConvertMasterString(val):
        return CEcWrapperTypes.ConvertMasterString(val)


    def WaitForAuxClock(self):
        """
        Waits for AuxClock

        Returns:
            bool: Wait event was triggered
        """
        if self.m_pvTimingEvent is None:
            return False

        #// Wait for next cycle (event from scheduler task)
        return CEcWrapper.Get().ecwOsWaitForEvent2(self.m_pvTimingEvent)


    def IsAuxClockEnabled(self):
        """
        Checks if AuxClock is enabled

        Returns:
            bool: AuxClock enabled or disabled
        """
        return self.m_pvTimingEvent is not None


    def EoeInstallEndpoint(self, bCreateTxEvent, strInterfaceName, strInterfaceGuid):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def EoeUninstallEndpoint(self):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def EoeTriggerTxEvent(self):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    @classmethod
    def ESCTypeText(cls, byEscType, bLong):
        """
        Gets the text of corresponding ESC type

        Args:
            byEscType: ESC type
            bLong: True: long text version, False: short text version
            strInterfaceGuid: Interface guid (optional)

        Returns:
            str: Text of corresponding ESC type
        """
        data = SDN_EC_STRING_HLP()
        eRes = cls.ConvResAsError(CEcWrapper.Get().ecwESCTypeText(byEscType, bLong, ctypes.pointer(data)))
        if eRes == ECError.EC_NOERROR:
            return CEcWrapperTypes.Conv_StrFromCharPtr(data.Data)
        return ""


    @classmethod
    def SlaveVendorText(cls, dwVendorId):
        """
        Gets the text of slave vendor

        Args:
            dwVendorId: Vendor ID

        Returns:
            str: Text of slave vendor
        """
        data = SDN_EC_STRING_HLP()
        eRes = cls.ConvResAsError(CEcWrapper.Get().ecwSlaveVendorText(dwVendorId, ctypes.pointer(data)))
        if eRes == ECError.EC_NOERROR:
            return CEcWrapperTypes.Conv_StrFromCharPtr(data.Data)
        return ""


    @classmethod
    def SlaveProdCodeText(cls, dwVendorId, dwProductCode):
        """
        Gets the text of slave product code

        Args:
            dwVendorId: Vendor ID
            dwProductCode: Product code

        Returns:
            str: Text of slave product code
        """
        data = SDN_EC_STRING_HLP()
        eRes = cls.ConvResAsError(CEcWrapper.Get().ecwSlaveProdCodeText(dwVendorId, dwProductCode, ctypes.pointer(data)))
        if eRes == ECError.EC_NOERROR:
            return CEcWrapperTypes.Conv_StrFromCharPtr(data.Data)
        return ""


    def SetBusCnfReadProp(self, eEscSiiReg, dwTimeout):
        """
        Sets read property for bus configuration

        Args:
            eEscSiiReg: SII register
            dwTimeout: Time out of bus scan

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        gen_eEscSiiReg = CEcWrapperTypes.Conv(eEscSiiReg, "DN_ESC_SII_REG")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        return self.ConvResAsError(CEcWrapper.Get().ecwSetBusCnfReadProp(self.m_dwMasterInstanceId, gen_eEscSiiReg, gen_dwTimeout))


    def RestartScanBus(self, dwTimeout, bReadRevisionNo, bReadSerialNo):
        """
        Trigger Bus Scan

        Args:
            dwTimeout: Time out of bus scan
            bReadRevisionNo: Read revision number
            bReadSerialNo: Read serial number

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_bReadRevisionNo = CEcWrapperTypes.Conv(bReadRevisionNo, "bool")
        gen_bReadSerialNo = CEcWrapperTypes.Conv(bReadSerialNo, "bool")
        return self.ConvResAsError(CEcWrapper.Get().ecwRestartScanBus(self.m_dwMasterInstanceId, gen_dwTimeout, gen_bReadRevisionNo, gen_bReadSerialNo))


    def GetBusScanSlaveInfoDesc(self, wAutoIncAddr, out_oSlaveInfoDesc):
        """
        This call will return the basic slave info determined in the last bus scan

        Args:
            wAutoIncAddr: Auto increment address of the slave
            oSlaveInfoDesc: Out parameter that contains different slave information after the call

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        out_oSlaveInfoDesc.value = None
        oSlaveInfoDesc = DN_EC_T_SB_SLAVEINFO_DESC()
        gen_wAutoIncAddr = CEcWrapperTypes.Conv(wAutoIncAddr, "short")
        gen_oSlaveInfoDesc = CEcWrapperTypes.Conv(oSlaveInfoDesc, "SDN_EC_T_SB_SLAVEINFO_DESC")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetBusScanSlaveInfoDesc(self.m_dwMasterInstanceId, gen_wAutoIncAddr, ctypes.byref(gen_oSlaveInfoDesc)))
        out_oSlaveInfoDesc.value = CEcWrapperTypes.Conv(gen_oSlaveInfoDesc, "DN_EC_T_SB_SLAVEINFO_DESC")
        return self.ReportErrorCode(gen_dwRetVal)


    def GetScanBusStatus(self, out_oSbStatus):
        """
        Gets the status of the last bus scan.

        Args:
            oSbStatus: The last bus scan status

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        out_oSbStatus.value = None
        gen_oSbStatus = SDN_EC_T_SB_STATUS_NTFY_DESC()
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetScanBusStatus(self.m_dwMasterInstanceId, ctypes.byref(gen_oSbStatus)))
        out_oSbStatus.value = CEcWrapperTypes.Conv(gen_oSbStatus)
        return self.ReportErrorCode(gen_dwRetVal)


    def CoeGetODList2(self, dwSlaveId, eListType, out_pList, dwLen, out_pdwOutLen, dwTimeout):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def CoeGetObjectDesc2(self, dwSlaveId, wIndex, out_oObjDesc, dwTimeout):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def CoeGetObjectDescReq(self, pMbxTfer, dwSlaveId, wIndex, out_oObjDesc, dwTimeout):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def CoeGetEntryDesc2(self, dwSlaveId, wIndex, bySubIndex, byValueInfoType, out_oEntryDesc, dwTimeout):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def CoeGetEntryDescReq(self, pMbxTfer, dwSlaveId, wIndex, bySubIndex, byValueInfoType, out_oEntryDesc, dwTimeout):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def MbxTferCreate(self, dwTferId, dwBufferSize, out_pMbxTfer):
        """
        Creates a mailbox transfer object

        Args:
            dwTferId: transfer ID (optional, can be 0)
            dwBufferSize: buffer size
            out_pMbxTfer: out mailbox transfer object

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        gen_dwTferId = CEcWrapperTypes.Conv(dwTferId, "uint", "uint")
        gen_dwBufferSize = CEcWrapperTypes.Conv(dwBufferSize, "uint", "uint")
        pMbxTfer = None
        gen_pMbxTfer = CEcWrapperTypes.Conv(pMbxTfer, "IntPtr")
        dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwMbxTferCreate2(self.m_dwMasterInstanceId, self.m_dwClientId, gen_dwTferId, gen_dwBufferSize, ctypes.pointer(gen_pMbxTfer)))
        out_pMbxTfer.value = CEcWrapperTypes.Conv(gen_pMbxTfer, "IntPtr")
        return dwRetVal


    def MbxTferWait(self, pMbxTfer):
        """
        Waits until mailbox transfer is finished

        Args:
            pMbxTfer: mailbox transfer object

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        gen_pMbxTfer = CEcWrapperTypes.Conv(pMbxTfer, "IntPtr")
        return self.ConvResAsError(CEcWrapper.Get().ecwMbxTferWait(self.m_dwMasterInstanceId, gen_pMbxTfer))


    def MbxTferCopyTo(self, pMbxTfer, abyData, dwDataLen):
        """
        Copies data to the mailbox transfer buffer

        Args:
            pMbxTfer: mailbox transfer object
            abyData: abyData
            dwDataLen: dwDataLen

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        gen_pMbxTfer = CEcWrapperTypes.Conv(pMbxTfer, "IntPtr")
        gen_abyData = CEcWrapperTypes.Conv_IntArrayToBytePtr(abyData)
        gen_dwDataLen = CEcWrapperTypes.Conv(dwDataLen, "uint")
        return self.ConvResAsError(CEcWrapper.Get().ecwMbxTferCopyTo(self.m_dwMasterInstanceId, gen_pMbxTfer, gen_abyData, gen_dwDataLen))


    def MbxTferCopyFrom(self, pMbxTfer, abyData, dwDataLen, out_pdwOutDataLen):
        """
        Copies data from the mailbox transfer buffer

        Args:
            pMbxTfer: mailbox transfer object
            abyData: abyData
            dwDataLen: dwDataLen
            out_pdwOutDataLen: pdwOutDataLen

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        out_pdwOutDataLen.value = 0
        gen_pMbxTfer = CEcWrapperTypes.Conv(pMbxTfer, "IntPtr")
        gen_abyData = CEcWrapperTypes.Conv_IntArrayToBytePtr(abyData)
        gen_dwDataLen = CEcWrapperTypes.Conv(dwDataLen, "uint")
        gen_pdwOutDataLen = ctypes.c_uint(0)
        dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwMbxTferCopyFrom(self.m_dwMasterInstanceId, gen_pMbxTfer, gen_abyData, gen_dwDataLen, ctypes.byref(gen_pdwOutDataLen)))
        out_pdwOutDataLen.value = gen_pdwOutDataLen.value
        for i in range(out_pdwOutDataLen.value):
            abyData[i] = CEcWrapperTypes.Conv_IntFromBytes(gen_abyData[i])
        return dwRetVal


    def NotifyApp(self, dwCode, pInData, wDataLen, out_pOutData, wOutLen, out_pdwOutDataLen):
        """
        Notifies the master application

        Args:
            dwCode: Code
            pInData: Input data
            wDataLen: Length of in data
            out_pOutData: Output data
            wOutLen: Length of output data field
            out_pdwOutDataLen: Length of actual out data

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        gen_dwCode = CEcWrapperTypes.Conv(dwCode, "uint")
        gen_pInData = CEcWrapperTypes.Conv(pInData, "byte[]")
        gen_wDataLen = CEcWrapperTypes.Conv(wDataLen, "ushort")
        pOutData = [0] * wOutLen
        gen_pOutData = CEcWrapperTypes.Conv(pOutData, "byte[]")
        gen_wOutLen = CEcWrapperTypes.Conv(wOutLen, "ushort")
        gen_pdwOutDataLen = ctypes.c_uint(0)
        dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwNotifyApp2(self.m_dwMasterInstanceId, gen_dwCode, gen_pInData, gen_wDataLen, gen_pOutData, gen_wOutLen, gen_pdwOutDataLen))
        out_pdwOutDataLen.value = gen_pdwOutDataLen.value
        for i in range(out_pdwOutDataLen.value):
            pOutData[i] = CEcWrapperTypes.Conv_IntFromBytes(gen_pOutData[i])
        out_pOutData.value = pOutData
        return dwRetVal


    def ReadIdentifyObj(self, wFixedAddr):
        """
        Reads the identify object of a slave

        Args:
            wFixedAddr: Fixed station address
        """
        CEcWrapper.Get().ecwReadIdentifyObj(self.m_dwMasterInstanceId, wFixedAddr)


    def AddDbgMsg(self, strDbgMsg):
        """
        Adds a debug message to the print message queue

        Args:
            strDbgMsg: debug message

        Returns:
            returns True: if success; otherwise False
        """
        gen_strDbgMsg = CEcWrapperTypes.Conv(strDbgMsg, "string")
        CEcWrapper.Get().ecwOsDbgMsg(gen_strDbgMsg)
        return True


    def GetSlaveInfoEx(self, oReq, out_oRes):
        """
        Gets the extended slave info determined in the last bus scan.

        Args:
            oReq: Request parameter
            out_oRes: The extended slave information structure

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        oRes = DN_EC_T_SB_SLAVEINFO_RES_DESC()
        gen_oReq = CEcWrapperTypes.Conv(oReq, "DN_EC_T_SB_SLAVEINFO_REQ_DESC")
        gen_oRes = CEcWrapperTypes.Conv(oRes, "DN_EC_T_SB_SLAVEINFO_RES_DESC")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetSlaveInfoEx(self.m_dwMasterInstanceId, ctypes.byref(gen_oReq), ctypes.byref(gen_oRes)))
        out_oRes.value = CEcWrapperTypes.Conv(gen_oRes, "SDN_EC_T_SB_SLAVEINFO_RES_DESC")
        return self.ReportErrorCode(gen_dwRetVal)


    def GetMasterParms(self, out_pParms): # ret: ECError
        """
        Get current Master initialization parameters.
        
        If the given buffer is larger than the actual size of structure EC_T_INIT_MASTER_PARMS, the parameters of EC_T_INIT_MASTER_PARMS.pOsParms,
        EC_T_INIT_MASTER_PARMS.pLinkParms and EC_T_INIT_MASTER_PARMS.pLinkParmsRed are appended.

        Args:
            pParms: Buffer to store Master parameters

        Returns:
            - EC_E_NOERROR on success
            - EC_E_INVALIDSTATE if master isn't initialized
            - EC_E_INVALIDPARM if buffer pParms is too small
        """
        pParms = DN_EC_T_INIT_MASTER_PARMS()
        gen_pParms = CEcWrapperTypes.Conv(pParms, "EC_T_INIT_MASTER_PARMS")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetMasterParms2(self.m_dwMasterInstanceId, ctypes.pointer(gen_pParms)))
        out_pParms.value = CEcWrapperTypes.Conv(gen_pParms, "DN_EC_T_INIT_MASTER_PARMS")
        return gen_dwRetVal


    def SetMasterParms(self, pParms): # ret: ECError
        """
        Change Master initialization parameters.
        
        Currently the following parameters cannot be changed:
        - EC_T_INIT_MASTER_PARMS.pOsParms
        - EC_T_INIT_MASTER_PARMS.pLinkParms
        - EC_T_INIT_MASTER_PARMS.pLinkParmsRed
        - EC_T_INIT_MASTER_PARMS.dwMaxBusSlaves
        - EC_T_INIT_MASTER_PARMS.dwMaxAcycFramesQueued
        - EC_T_INIT_MASTER_PARMS.dwAdditionalEoEEndpoints
        - EC_T_INIT_MASTER_PARMS.bVLANEnable
        - EC_T_INIT_MASTER_PARMS.wVLANId
        - EC_T_INIT_MASTER_PARMS.byVLANPrio

        Args:
            pParms: New Master parameters

        Returns:
            - EC_E_NOERROR on success
            - EC_E_INVALIDSTATE if master isn't initialized
        """
        gen_pParms = CEcWrapperTypes.Conv(pParms, "EC_T_INIT_MASTER_PARMS")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwSetMasterParms2(self.m_dwMasterInstanceId, ctypes.pointer(gen_pParms)))
        return gen_dwRetVal


    def ForceSlvStatCollection(self):
        """
        Sends datagrams to collect slave statistics counters.

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        return self.ConvResAsError(CEcWrapper.Get().ecwForceSlvStatCollection(self.m_dwMasterInstanceId))


    def GetSlvStatistics(self, dwSlaveId, out_oStatistics):
        """
        Returns slave statistics counters.

        Args:
            dwSlaveId: Slave ID
            oStatistics: out Statistics counters

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        oStatistics = DN_EC_T_SLVSTATISTICS_DESC()
        gen_oStatistics = CEcWrapperTypes.Conv(oStatistics, "SDN_EC_T_SLVSTATISTICS_DESC")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetSlvStatistics(self.m_dwMasterInstanceId, dwSlaveId, ctypes.byref(gen_oStatistics)))
        out_oStatistics.value = CEcWrapperTypes.Conv(gen_oStatistics, "DN_EC_T_SLVSTATISTICS_DESC")
        return self.ReportErrorCode(gen_dwRetVal)


    def GetCyclicConfigInfo(self, out_oCyclicConfigInfo):
        """
        Returns an array of cyclic tasks.

        Args:
            out_oCyclicConfigInfo: out Array of cyclic tasks

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        out_oCyclicConfigInfo.value = None

        oCyclicConfigInfo = None
        eRes = ECError.EC_INVALIDPARM # ECError
        dwCycEntryIndex = 0 # uint
        while True:
            tConfigDesc = SDN_EC_T_CYC_CONFIG_DESC()

            eRes = self.ConvResAsError(CEcWrapper.Get().ecwGetCyclicConfigInfo(self.m_dwMasterInstanceId, dwCycEntryIndex, ctypes.byref(tConfigDesc)), True)
            if eRes == ECError.EC_NOERROR:
                if oCyclicConfigInfo is None:
                    oCyclicConfigInfo = (DN_EC_T_CYC_CONFIG_DESC * tConfigDesc.dwNumCycEntries)()
                oCyclicConfigInfo[dwCycEntryIndex] = CEcWrapperTypes.Conv(tConfigDesc, "DN_EC_T_CYC_CONFIG_DESC")
                dwCycEntryIndex = dwCycEntryIndex + 1

            if eRes == ECError.EC_INVALIDINDEX:
                out_oCyclicConfigInfo.value = oCyclicConfigInfo
                return ECError.EC_NOERROR

            if eRes != ECError.EC_NOERROR:
                break

        return self.ReportErrorCode(eRes)


    def SetAllSlavesMustReachState(self, bAllSlavesMustReachState):
        """
        Sets flag that all slaves must reach the requested master state

        Args:
            bAllSlavesMustReachState: True: All slaves must reach the requested master state

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        return self.ConvResAsError(CEcWrapper.Get().ecwSetAllSlavesMustReachState(self.m_dwMasterInstanceId, bAllSlavesMustReachState))

    def SetAdditionalVariablesForSpecificDataTypesEnabled(self, bAdditionalVariablesForSpecificDataTypes):
        return self.ConvResAsError(CEcWrapper.Get().ecwSetAdditionalVariablesForSpecificDataTypesEnabled(self.m_dwMasterInstanceId, bAdditionalVariablesForSpecificDataTypes))


    def EnableNotification(self, dwClientId, tNotifyCode, bEnable):
        """
        Enables notification

        Args:
            tNotifyCode: Code of notification, which should be enabled
            bEnable: True: Enables notification

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        return self.ConvResAsError(CEcWrapper.Get().ecwEnableNotification(self.m_dwMasterInstanceId, dwClientId, tNotifyCode.value, bEnable))


    def ThrottleNotification(self, tNotifyCode, dwTimeout):
        """
        Throttles notification

        Args:
            tNotifyCode: Code of notification, which should be throttled
            dwTimeout: 0 = Not throttled, > 0 = Throttle timeout in ms

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        for i in range(len(self.m_cThrottleQueue)):
            elem = self.m_cThrottleQueue[i] # ThrottleElem
            if elem.tNotifyCode == tNotifyCode:
                self.m_cThrottleQueue.pop(elem)
                break

        e = ThrottleElem()
        e.tNotifyCode = tNotifyCode
        e.nLastSeen = 0
        e.dwTimeout = dwTimeout
        self.m_cThrottleQueue.append(e)
        return ECError.EC_NOERROR


    def IsThrottledNotification(self, tNotifyCode):
        """
        Checks if notification is throttled

        Returns:
            True, if notification is throttled
        """
        for elem in self.m_cThrottleQueue:
            if elem.tNotifyCode != tNotifyCode:
                continue

            now = int((datetime.datetime.now() - CEcWrapperPython.DATE_SINCE_1970).total_seconds() * 1000) # int64
            last = elem.nLastSeen # int64

            if last + elem.dwTimeout > now:
                return True

            elem.nLastSeen = now
            break

        return False


    def IoControl(self, dwCode, pbyInBuf, dwInBufSize, out_pbyOutBuf, dwOutBufSize, out_pdwNumOutData):
        """
        Executes an IO control

        Args:
            dwCode: Control code
            pbyInBuf: input data buffer
            dwInBufSize: size of input data buffer in byte
            out_pbyOutBuf: output data buffer
            dwOutBufSize: size of output data buffer in byte
            out_pdwNumOutData: out number of output data bytes stored in output data buffer

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        out_pbyOutBuf.value = [0] * dwOutBufSize
        out_pdwNumOutData.value = 0
        gen_dwCode = CEcWrapperTypes.Conv(dwCode, "uint")
        gen_pbyInBuf = CEcWrapperTypes.Conv_IntArrayToBytePtr(pbyInBuf)
        gen_dwInBufSize = CEcWrapperTypes.Conv(dwInBufSize, "uint")
        gen_pbyOutBuf = CEcWrapperTypes.Conv_IntArrayToBytePtr(out_pbyOutBuf.value)
        gen_dwOutBufSize = CEcWrapperTypes.Conv(dwOutBufSize, "uint")
        gen_pdwNumOutData = CEcWrapperTypes.Conv(out_pdwNumOutData.value, "uint")
        res = self.ConvResAsError(CEcWrapper.Get().ecwIoControl2(self.m_dwMasterInstanceId, gen_dwCode, ctypes.byref(gen_pbyInBuf), gen_dwInBufSize, ctypes.byref(gen_pbyOutBuf), gen_dwOutBufSize, ctypes.byref(gen_pdwNumOutData)))
        out_pbyOutBuf.value = CEcWrapperTypes.Conv(gen_pbyOutBuf)
        out_pdwNumOutData.value = CEcWrapperTypes.Conv(gen_pdwNumOutData)
        return res


    def SdoUploadMasterOd(self, wObIndex, dwTimeout, out_pobjMasterOd):
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    @staticmethod
    def CreateObjectFromBytes(bytes_, ref_obj):
        """
        Creates object from byte array

        Args:
            bytes_: Object data as byte array
            ref_obj: Object which should be filled

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        obj = ref_obj.value

        if not obj.__class__.__name__.startswith("DN_"):
            return ECError.EC_INVALIDPARM

        sobj = CEcWrapperTypes.Conv(obj)
        if sobj is None:
            return ECError.EC_NOTFOUND

        size = CEcWrapperTypes.GetSizeOfStructure(sobj)
        if len(bytes_) < size:
            return ECError.EC_NOMEMORY

        bytes2 = (ctypes.c_uint8 * len(bytes_))()
        for i, b in enumerate(bytes_):
            if isinstance(b, bytes):
                bytes2[i] = int.from_bytes(b, "little")
            else:
                bytes2[i] = b
        sobj2 = CEcWrapperTypes.ConvBytesToStructure(bytes2, sobj)
        if sobj2 is None:
            return ECError.EC_INVALIDPARM

        res = CEcWrapperTypes.Conv(sobj2)
        if res is None:
            return ECError.EC_NOTFOUND

        ref_obj.value = res
        return ECError.EC_NOERROR


    @staticmethod
    def GetSizeOfObject(obj, out_size):
        """
        Get byte size of object (as required from CoeSdoUpload)

        Args:
            obj: Object
            out_size: Byte size of object

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        out_size.value = 0

        if not obj.__class__.__name__.startswith("DN_"):
            return ECError.EC_INVALIDPARM

        sobj = CEcWrapperTypes.Conv(obj)
        if sobj is None:
            return ECError.EC_NOTFOUND

        out_size.value = CEcWrapperTypes.GetSizeOfStructure(sobj)
        return ECError.EC_NOERROR

    @staticmethod
    def GetBitLenOfDataType(type_):
        if type_ == DN_EC_T_DEFTYPE.BOOLEAN:
            return 1
        if DN_EC_T_DEFTYPE.BIT1 <= type_.value <= DN_EC_T_DEFTYPE.BIT8:
            return 8
        if DN_EC_T_DEFTYPE.BIT9 <= type_.value <= DN_EC_T_DEFTYPE.BIT16:
            return 16
        if type_ == DN_EC_T_DEFTYPE.INTEGER8:
            return 8
        if type_ == DN_EC_T_DEFTYPE.INTEGER16:
            return 16
        if type_ in {DN_EC_T_DEFTYPE.INTEGER24, DN_EC_T_DEFTYPE.INTEGER32}:
            return 32
        if type_ in {DN_EC_T_DEFTYPE.INTEGER40, DN_EC_T_DEFTYPE.INTEGER48, DN_EC_T_DEFTYPE.INTEGER56, DN_EC_T_DEFTYPE.INTEGER64}:
            return 64
        if type_ in {DN_EC_T_DEFTYPE.UNSIGNED8, DN_EC_T_DEFTYPE.BYTE, DN_EC_T_DEFTYPE.BITARR8}:
            return 8
        if type_ in {DN_EC_T_DEFTYPE.UNSIGNED16, DN_EC_T_DEFTYPE.WORD, DN_EC_T_DEFTYPE.BITARR16}:
            return 16
        if type_ in {DN_EC_T_DEFTYPE.UNSIGNED24, DN_EC_T_DEFTYPE.UNSIGNED32, DN_EC_T_DEFTYPE.DWORD, DN_EC_T_DEFTYPE.BITARR32}:
            return 32
        if type_ in {DN_EC_T_DEFTYPE.UNSIGNED40, DN_EC_T_DEFTYPE.UNSIGNED48, DN_EC_T_DEFTYPE.UNSIGNED56, DN_EC_T_DEFTYPE.UNSIGNED64}:
            return 64
        if type_ == DN_EC_T_DEFTYPE.REAL32:
            return 32
        if type_ == DN_EC_T_DEFTYPE.REAL64:
            return 64
        return SDN_EC_T_VARIANT.MaxBufferSize * 8


    @staticmethod
    def IsBitDataType(type_):
        if (type_ == DN_EC_T_DEFTYPE.BOOLEAN):
            return True
        if (type_ >= DN_EC_T_DEFTYPE.BIT1 and type_ <= DN_EC_T_DEFTYPE.BIT8):
            return True
        if (type_ >= DN_EC_T_DEFTYPE.BIT9 and type_ <= DN_EC_T_DEFTYPE.BIT16):
            return True
        return False


    @staticmethod
    def ConvValueToBytes(type_, value, out_bytes):
        """
        Converts value to bytes

        Args:
            type_: Data type
            value: Value
            out_bytes: Bytes

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        out_bytes.value = None

        if (type == DN_EC_T_DEFTYPE.NULL or type_ == DN_EC_T_DEFTYPE.ARRAY_OF_BYTE):
            if (not isinstance(value, list)):
                return ECError.EC_NOTSUPPORTED

            out_bytes.value = value
            return ECError.EC_NOERROR

        variant = SDN_EC_T_VARIANT()
        variant.nBufferSize = (CEcWrapperPython.GetBitLenOfDataType(type_) + 7) // 8

        if type_ == DN_EC_T_DEFTYPE.BOOLEAN:
            variant.uVariant.nUnsigned8 = 1 if (str(value) == "1" or str(value).lower() == "true") else 0
        elif DN_EC_T_DEFTYPE.BIT1 <= type_.value <= DN_EC_T_DEFTYPE.BIT8:
            variant.uVariant.nUnsigned8 = int(str(value))
        elif DN_EC_T_DEFTYPE.BIT9 <= type_.value <= DN_EC_T_DEFTYPE.BIT16:
            variant.uVariant.nUnsigned16 = int(str(value))
        elif type_ == DN_EC_T_DEFTYPE.INTEGER8:
            variant.uVariant.nInteger8 = int(str(value))
        elif type_ == DN_EC_T_DEFTYPE.INTEGER16:
            variant.uVariant.nInteger16 = int(str(value))
        elif type_ in {DN_EC_T_DEFTYPE.INTEGER24, DN_EC_T_DEFTYPE.INTEGER32}:
            variant.uVariant.nInteger32 = int(str(value))
        elif type_ in {DN_EC_T_DEFTYPE.INTEGER40, DN_EC_T_DEFTYPE.INTEGER48, DN_EC_T_DEFTYPE.INTEGER56, DN_EC_T_DEFTYPE.INTEGER64}:
            variant.uVariant.nInteger64 = int(str(value))
        elif type_ in {DN_EC_T_DEFTYPE.UNSIGNED8, DN_EC_T_DEFTYPE.BYTE, DN_EC_T_DEFTYPE.BITARR8}:
            variant.uVariant.nUnsigned8 = int(str(value))
        elif type_ in {DN_EC_T_DEFTYPE.UNSIGNED16, DN_EC_T_DEFTYPE.WORD, DN_EC_T_DEFTYPE.BITARR16}:
            variant.uVariant.nUnsigned16 = int(str(value))
        elif type_ in {DN_EC_T_DEFTYPE.UNSIGNED24, DN_EC_T_DEFTYPE.UNSIGNED32, DN_EC_T_DEFTYPE.DWORD, DN_EC_T_DEFTYPE.BITARR32}:
            variant.uVariant.nUnsigned32 = int(str(value))
        elif type_ in {DN_EC_T_DEFTYPE.UNSIGNED40, DN_EC_T_DEFTYPE.UNSIGNED48, DN_EC_T_DEFTYPE.UNSIGNED56, DN_EC_T_DEFTYPE.UNSIGNED64}:
            variant.uVariant.nUnsigned64 = int(str(value))
        elif type_ == DN_EC_T_DEFTYPE.REAL32:
            variant.uVariant.nReal32 = float(str(value))
        elif type_ == DN_EC_T_DEFTYPE.REAL64:
            variant.uVariant.nReal64 = float(str(value))
        elif type_ == DN_EC_T_DEFTYPE.VISIBLESTRING:
            bytes2 = str.encode(value, 'utf-8')
            if (len(bytes2) >= SDN_EC_T_VARIANT.MaxBufferSize * 8):
                return ECError.EC_NOMEMORY
            variant.SetBuffer(bytes2)
        elif type_ == DN_EC_T_DEFTYPE.ARRAY_OF_BYTE:
            bytes2 = value
            if len(bytes2) >= SDN_EC_T_VARIANT.MaxBufferSize * 8:
                return ECError.EC_NOMEMORY
            variant.SetBuffer(bytes2)
        elif type_ == DN_EC_T_DEFTYPE.ARRAY_OF_UINT:
            bytes2 = CEcWrapperTypes.Conv_IntArrayFromChunks(value, 2)
            if len(bytes2) >= SDN_EC_T_VARIANT.MaxBufferSize * 8:
                return ECError.EC_NOMEMORY
            variant.SetBuffer(bytes2)
        elif type_ == DN_EC_T_DEFTYPE.ARRAY_OF_INT:
            bytes2 = CEcWrapperTypes.Conv_IntArrayFromChunks(value, 2)
            if len(bytes2) >= SDN_EC_T_VARIANT.MaxBufferSize * 8:
                return ECError.EC_NOMEMORY
            variant.SetBuffer(bytes2)
        elif type_ == DN_EC_T_DEFTYPE.ARRAY_OF_SINT:
            bytes2 = value
            if len(bytes2) >= SDN_EC_T_VARIANT.MaxBufferSize * 8:
                return ECError.EC_NOMEMORY
            variant.SetBuffer(bytes2)
        elif type_ == DN_EC_T_DEFTYPE.ARRAY_OF_DINT:
            bytes2 = CEcWrapperTypes.Conv_IntArrayFromChunks(value, 4)
            if len(bytes2) >= SDN_EC_T_VARIANT.MaxBufferSize * 8:
                return ECError.EC_NOMEMORY
            variant.SetBuffer(bytes2)
        elif type_ == DN_EC_T_DEFTYPE.ARRAY_OF_UDINT:
            bytes2 = CEcWrapperTypes.Conv_IntArrayFromChunks(value, 4)
            if len(bytes2) >= SDN_EC_T_VARIANT.MaxBufferSize * 8:
                return ECError.EC_NOMEMORY
            variant.SetBuffer(bytes2)
        elif type_ == DN_EC_T_DEFTYPE.GUID:
            bytes2 = value
            if not len(bytes2) == 16:
                return ECError.EC_INVALIDPARM
            if len(bytes2) >= SDN_EC_T_VARIANT.MaxBufferSize * 8:
                return ECError.EC_NOMEMORY
            variant.SetBuffer(bytes2)
        else:
            return ECError.EC_NOTSUPPORTED

        out_bytes.value = variant.GetBuffer()
        return ECError.EC_NOERROR


    @staticmethod
    def ConvValueFromBytes(bytes_, type_, out_value):
        """
        Converts value to bytes

        Args:
            bytes_: Bytes
            type_: Data type
            out_value: Value

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        out_value.value = None
        value = None

        if (type_ == DN_EC_T_DEFTYPE.NULL or type_ == DN_EC_T_DEFTYPE.ARRAY_OF_BYTE):
            out_value.value = bytes_
            return ECError.EC_NOERROR

        if (len(bytes_) >= SDN_EC_T_VARIANT.MaxBufferSize*8):
            return ECError.EC_NOMEMORY

        variant = SDN_EC_T_VARIANT()
        variant.nBufferSize = (CEcWrapperPython.GetBitLenOfDataType(type_) + 7) // 8
        variant.SetBuffer(bytes_)

        if type_ == DN_EC_T_DEFTYPE.BOOLEAN:
            value = variant.uVariant.nUnsigned8 == 1
        elif DN_EC_T_DEFTYPE.BIT1 <= type_.value <= DN_EC_T_DEFTYPE.BIT8:
            value = variant.uVariant.nUnsigned8
        elif DN_EC_T_DEFTYPE.BIT9 <= type_.value <= DN_EC_T_DEFTYPE.BIT16:
            value = variant.uVariant.nUnsigned16
        elif type_ == DN_EC_T_DEFTYPE.INTEGER8:
            value = variant.uVariant.nInteger8
        elif type_ == DN_EC_T_DEFTYPE.INTEGER16:
            value = variant.uVariant.nInteger16
        elif type_ in {DN_EC_T_DEFTYPE.INTEGER24, DN_EC_T_DEFTYPE.INTEGER32}:
            value = variant.uVariant.nInteger32
        elif type_ in {DN_EC_T_DEFTYPE.INTEGER40, DN_EC_T_DEFTYPE.INTEGER48, DN_EC_T_DEFTYPE.INTEGER56, DN_EC_T_DEFTYPE.INTEGER64}:
            value = variant.uVariant.nInteger64
        elif type_ in {DN_EC_T_DEFTYPE.UNSIGNED8, DN_EC_T_DEFTYPE.BYTE, DN_EC_T_DEFTYPE.BITARR8}:
            value = variant.uVariant.nUnsigned8
        elif type_ in {DN_EC_T_DEFTYPE.UNSIGNED16, DN_EC_T_DEFTYPE.WORD, DN_EC_T_DEFTYPE.BITARR16}:
            value = variant.uVariant.nUnsigned16
        elif type_ in {DN_EC_T_DEFTYPE.UNSIGNED24, DN_EC_T_DEFTYPE.UNSIGNED32, DN_EC_T_DEFTYPE.DWORD, DN_EC_T_DEFTYPE.BITARR32}:
            value = variant.uVariant.nUnsigned32
        elif type_ in {DN_EC_T_DEFTYPE.UNSIGNED40, DN_EC_T_DEFTYPE.UNSIGNED48, DN_EC_T_DEFTYPE.UNSIGNED56, DN_EC_T_DEFTYPE.UNSIGNED64}:
            value = variant.uVariant.nUnsigned64
        elif type_ == DN_EC_T_DEFTYPE.REAL32:
            value = variant.uVariant.nReal32
        elif type_ == DN_EC_T_DEFTYPE.REAL64:
            value = variant.uVariant.nReal64
        elif type_ == DN_EC_T_DEFTYPE.VISIBLESTRING:
            bytes1 = variant.GetBuffer()
            value = CEcWrapperTypes.PatchString(bytearray(bytes1).decode('utf-8'))
        elif type_ == DN_EC_T_DEFTYPE.ARRAY_OF_BYTE:
            value = variant.GetBuffer()
        elif type_ == DN_EC_T_DEFTYPE.ARRAY_OF_UINT:
            bytes1 = variant.GetBuffer()
            value = CEcWrapperTypes.Conv_IntArrayToChunks(bytes1, 2)
        elif type_ == DN_EC_T_DEFTYPE.ARRAY_OF_INT:
            bytes1 = variant.GetBuffer()
            value = CEcWrapperTypes.Conv_IntArrayToChunks(bytes1, 2)
        elif type_ == DN_EC_T_DEFTYPE.ARRAY_OF_SINT:
            value = variant.GetBuffer()
        elif type_ == DN_EC_T_DEFTYPE.ARRAY_OF_DINT:
            bytes1 = variant.GetBuffer()
            value = CEcWrapperTypes.Conv_IntArrayToChunks(bytes1, 4)
        elif type_ == DN_EC_T_DEFTYPE.ARRAY_OF_UDINT:
            bytes1 = variant.GetBuffer()
            value = CEcWrapperTypes.Conv_IntArrayToChunks(bytes1, 4)
        elif type_ == DN_EC_T_DEFTYPE.GUID:
            value = variant.GetBuffer()
        else:
            return ECError.EC_NOTSUPPORTED

        out_value.value = value
        return ECError.EC_NOERROR


    @staticmethod
    def ReadValueFromBytes(bytes_, bitOffset, bitLength, type_, out_value):
        """
        Read value from bytes

        Args:
            bytes_: Bytes
            bitOffset: Bit offset
            bitLength: Bit length
            type_: Data type
            out_value: Value

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        out_value.value = None

        isBitDataType = CEcWrapperPython.IsBitDataType(type_)
        if not isBitDataType and (bitOffset % 8 > 0 or bitLength % 8 > 0):
            return ECError.EC_INVALIDPARM

        bytes2 = []
        if isBitDataType:
            out_bytes2 = CEcWrapperPythonOutParam()
            CEcWrapperTypes.ReadPdBitsFromBytes(bytes_, bitOffset, bitLength, out_bytes2)
            bytes2 = out_bytes2.value
        else:
            byteOffset = (bitOffset + 7) // 8
            byteLength = (bitLength + 7) // 8
            out_bytes2 = CEcWrapperPythonOutParam()
            CEcWrapperTypes.ReadPdByteFromBytes(bytes_, byteOffset, byteLength, out_bytes2)
            bytes2 = out_bytes2.value

        return CEcWrapperPython.ConvValueFromBytes(bytes2, type_, out_value)


    @staticmethod
    def ReadValueFromAddress(address, bitOffset, bitLength, type_, out_value):
        """
        Read value from address

        Args:
            address: Address
            bitOffset: Bit offset
            bitLength: Bit length
            type_: Data type
            out_value: Value

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        out_value.value = None

        isBitDataType = CEcWrapperPython.IsBitDataType(type_)
        if not isBitDataType and (bitOffset % 8 > 0 or bitLength % 8 > 0):
            return ECError.EC_INVALIDPARM

        bytes2 = []
        if isBitDataType:
            out_bytes2 = CEcWrapperPythonOutParam()
            CEcWrapperTypes.ReadPdBitsFromAddress(address, bitOffset, bitLength, out_bytes2)
            bytes2 = out_bytes2.value
        else:
            byteOffset = (bitOffset + 7) // 8
            byteLength = (bitLength + 7) // 8
            out_bytes2 = CEcWrapperPythonOutParam()
            CEcWrapperTypes.ReadPdByteFromAddress(address, byteOffset, byteLength, out_bytes2)
            bytes2 = out_bytes2.value

        return CEcWrapperPython.ConvValueFromBytes(bytes2, type_, out_value)


    @staticmethod
    def WriteValueToBytes(bytes_, bitOffset, bitLength, type_, value):
        """
        Write value to bytes

        Args:
            bytes_: Bytes
            bitOffset: Bit offset
            bitLength: Bit length
            type_: Data type
            value: Value

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        isBitDataType = CEcWrapperPython.IsBitDataType(type_)
        if not isBitDataType and (bitOffset % 8 > 0 or bitLength % 8 > 0):
            return ECError.EC_INVALIDPARM

        out_bytes2 = CEcWrapperPythonOutParam()
        eRes = CEcWrapperPython.ConvValueToBytes(type_, value, out_bytes2)
        if eRes != ECError.EC_NOERROR:
            return eRes

        bytes2 = out_bytes2.value
        if isBitDataType:
            CEcWrapperTypes.WritePdBitsToBytes(bytes_, bitOffset, bytes2, bitLength)
        else:
            byteOffset = (bitOffset + 7) // 8
            CEcWrapperTypes.WritePdByteToBytes(bytes_, byteOffset, bytes2)

        return ECError.EC_NOERROR


    @staticmethod
    def WriteValueToAddress(address, bitOffset, bitLength, type_, value):
        """
        Write value to address

        Args:
            address: Address
            bitOffset: Bit offset
            bitLength: Bit length
            type_: Data type
            value: Value

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        isBitDataType = CEcWrapperPython.IsBitDataType(type_)
        if not isBitDataType and (bitOffset % 8 > 0 or bitLength % 8 > 0):
            return ECError.EC_INVALIDPARM

        out_bytes2 = CEcWrapperPythonOutParam()
        eRes = CEcWrapperPython.ConvValueToBytes(type_, value, out_bytes2)
        if eRes != ECError.EC_NOERROR:
            return eRes

        bytes2 = out_bytes2.value
        if isBitDataType:
            CEcWrapperTypes.WritePdBitsToAddress(address, bitOffset, bytes2, bitLength)
        else:
            byteOffset = (bitOffset + 7) // 8
            CEcWrapperTypes.WritePdByteToAddress(address, byteOffset, bytes2)

        return ECError.EC_NOERROR


    @staticmethod
    def ConvValueFromString(type_, value):
        """
        Converts value from string

        Args:
            type_: Type
            value: Value

        Returns:
            Value as object
        """

        if type_ == DN_EC_T_DEFTYPE.BOOLEAN:
            return True if (value == "1" or value.lower() == "true") else False
        if DN_EC_T_DEFTYPE.BIT1 <= type_.value <= DN_EC_T_DEFTYPE.BIT8:
            return int(value)
        if DN_EC_T_DEFTYPE.BIT9 <= type_.value <= DN_EC_T_DEFTYPE.BIT16:
            return int(value)
        if type_ == DN_EC_T_DEFTYPE.INTEGER8:
            return int(value)
        if type_ == DN_EC_T_DEFTYPE.INTEGER16:
            return int(value)
        if type_ in {DN_EC_T_DEFTYPE.INTEGER24, DN_EC_T_DEFTYPE.INTEGER32}:
            return int(value)
        if type_ in {DN_EC_T_DEFTYPE.INTEGER40, DN_EC_T_DEFTYPE.INTEGER48, DN_EC_T_DEFTYPE.INTEGER56, DN_EC_T_DEFTYPE.INTEGER64}:
            return int(value)
        if type_ in {DN_EC_T_DEFTYPE.UNSIGNED8, DN_EC_T_DEFTYPE.BYTE, DN_EC_T_DEFTYPE.BITARR8}:
            return int(value)
        if type_ in {DN_EC_T_DEFTYPE.UNSIGNED16, DN_EC_T_DEFTYPE.WORD, DN_EC_T_DEFTYPE.BITARR16}:
            return int(value)
        if type_ in {DN_EC_T_DEFTYPE.UNSIGNED24, DN_EC_T_DEFTYPE.UNSIGNED32, DN_EC_T_DEFTYPE.DWORD, DN_EC_T_DEFTYPE.BITARR32}:
            return int(value)
        if type_ in {DN_EC_T_DEFTYPE.UNSIGNED40, DN_EC_T_DEFTYPE.UNSIGNED48, DN_EC_T_DEFTYPE.UNSIGNED56, DN_EC_T_DEFTYPE.UNSIGNED64}:
            return int(value)
        if type_ == DN_EC_T_DEFTYPE.REAL32:
            return float(value)
        if type_ == DN_EC_T_DEFTYPE.REAL64:
            return float(value)
        if type_ == DN_EC_T_DEFTYPE.VISIBLESTRING:
            return value
        if type_ == DN_EC_T_DEFTYPE.ARRAY_OF_BYTE:
            return CEcWrapperPython.ConvStringToByteArray(value)
        if type_ == DN_EC_T_DEFTYPE.ARRAY_OF_UINT:
            bytes2 = CEcWrapperPython.ConvStringToByteArray(value)
            if bytes2 == None: return None
            return CEcWrapperTypes.Conv_IntArrayToChunks(bytes2, 2)
        if type_ == DN_EC_T_DEFTYPE.ARRAY_OF_INT:
            bytes2 = CEcWrapperPython.ConvStringToByteArray(value)
            if bytes2 == None: return None
            return CEcWrapperTypes.Conv_IntArrayToChunks(bytes2, 2)
        if type_ == DN_EC_T_DEFTYPE.ARRAY_OF_SINT:
            return CEcWrapperPython.ConvStringToByteArray(value)
        if type_ == DN_EC_T_DEFTYPE.ARRAY_OF_DINT:
            bytes2 = CEcWrapperPython.ConvStringToByteArray(value)
            if bytes2 == None: return None
            return CEcWrapperTypes.Conv_IntArrayToChunks(bytes2, 4)
        if type_ == DN_EC_T_DEFTYPE.ARRAY_OF_UDINT:
            bytes2 = CEcWrapperPython.ConvStringToByteArray(value)
            if bytes2 == None: return None
            return CEcWrapperTypes.Conv_IntArrayToChunks(bytes2, 4)
        if type_ == DN_EC_T_DEFTYPE.GUID:
            value = value[1:len(value) - 2]
            value = value.replace("-", "")
            valueWithBlanks = ""
            for i in range(0, len(value), 2):
                valueWithBlanks += value[i:i + 2] + " "
            return CEcWrapperPython.ConvStringToByteArray(valueWithBlanks.strip())

        return None


    @staticmethod
    def ConvValueToString(type_, value):
        """
        Converts value to string

        Args:
            type: Type
            value: Value

        Returns:
            Value as string
        """
        if (value == None):
            return ""
        if type_ == DN_EC_T_DEFTYPE.REAL32:
            return "{:.2f}".format(value)
        if type_ == DN_EC_T_DEFTYPE.REAL64:
            return "{:.2f}".format(value)
        if type_ == DN_EC_T_DEFTYPE.ARRAY_OF_BYTE:
            return CEcWrapperPython.ConvStringFromByteArray(value)
        if type_ == DN_EC_T_DEFTYPE.ARRAY_OF_UINT:
            bytes2 = CEcWrapperTypes.Conv_IntArrayFromChunks(value, 2)
            return CEcWrapperPython.ConvStringFromByteArray(bytes2)
        if type_ == DN_EC_T_DEFTYPE.ARRAY_OF_INT:
            bytes2 = CEcWrapperTypes.Conv_IntArrayFromChunks(value, 2)
            return CEcWrapperPython.ConvStringFromByteArray(bytes2)
        if type_ == DN_EC_T_DEFTYPE.ARRAY_OF_SINT:
            return CEcWrapperPython.ConvStringFromByteArray(value)
        if type_ == DN_EC_T_DEFTYPE.ARRAY_OF_DINT:
            bytes2 = CEcWrapperTypes.Conv_IntArrayFromChunks(value, 4)
            return CEcWrapperPython.ConvStringFromByteArray(bytes2)
        if type_ == DN_EC_T_DEFTYPE.ARRAY_OF_UDINT:
            bytes2 = CEcWrapperTypes.Conv_IntArrayFromChunks(value, 4)
            return CEcWrapperPython.ConvStringFromByteArray(bytes2)
        if type_ == DN_EC_T_DEFTYPE.GUID:
            bytesAsStr = "".join("{:02X}".format(x) for x in value)
            indices = [ 9, 14, 19, 24]
            for indice in indices:
                s1 = bytesAsStr[0:indice-1]
                s2 = bytesAsStr[indice:]
                bytesAsStr = s1 + "-" + s2
            return "{" + bytesAsStr.lower() + "}"
        return str(value)


    @staticmethod
    def ConvStringFromByteArray(data):
        """
        Converts byte array to string (e.g. 01 02)

        Args:
            data: Data

        Returns:
            Data as string
        """
        if data == None or len(data) == 0:
            return ""

        bytesAsStr = " ".join("{:02X}".format(x) for x in data)
        if bytesAsStr.startswith("FF FE "):
            bytesAsStr = bytesAsStr[6:]
        return bytesAsStr


    @staticmethod
    def ConvStringToByteArray(data):
        """
        Converts byte array from string (e.g. 01 02)

        Args:
            data: Data

        Returns:
            Data as byte array
        """

        if data == None or len(data) == 0:
            return None

        bytes_ = data.strip().split(' ')
        if len(bytes_) == 0:
            return None

        ret = []
        for b in bytes_:
            val = 0

            try:
                val = int(b, 16)
            except:
                return None

            if val < 0 or val > 255:
                return None

            ret.append(val)

        return ret


    def ReadSlaveEEPRom(self, bFixedAddressing, wSlaveAddress, wEEPRomStartOffset, pwReadData, dwReadLen, out_pdwNumOutData, dwTimeout):
        """
        Read EEPRom data from slave

        Args:
            bFixedAddressing: True: use station addressing, False: use auto increment addressing
            wSlaveAddress: Slave Address, station or auto increment address depending on bFixedAddressing
            wEEPRomStartOffset: Address to start EEPRom Read from
            pwReadData: Pointer to ushort array to carry the read data
            dwReadLen: Size of the ushort array provided at pwReadData (in ushorts)
            out_pdwNumOutData: out Pointer to uint carrying actually read data (in ushorts) after completion
            dwTimeout: Timeout in milliseconds. The function will block at most for this time.
              The timeout value must not be set to EC_NOWAIT

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        out_pdwNumOutData.value = 0
        if pwReadData is None or len(pwReadData) == 0 or len(pwReadData) < dwReadLen:
            return ECError.EC_INVALIDPARM

        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_wEEPRomStartOffset = CEcWrapperTypes.Conv(wEEPRomStartOffset, "ushort")
        gen_pwReadData = CEcWrapperTypes.Conv_IntArrayToBytePtr(pwReadData)
        gen_dwReadLen = CEcWrapperTypes.Conv((dwReadLen + 1) // 2, "uint")
        gen_pdwNumOutData = CEcWrapperTypes.Conv(out_pdwNumOutData.value, "uint")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")

        dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwReadSlaveEEPRom(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, gen_wEEPRomStartOffset, gen_pwReadData, gen_dwReadLen, ctypes.byref(gen_pdwNumOutData), gen_dwTimeout))
        out_pdwNumOutData.value = gen_pdwNumOutData.value * 2 if gen_pdwNumOutData.value * 2 < dwReadLen else dwReadLen
        for i in range(out_pdwNumOutData.value):
            pwReadData[i] = CEcWrapperTypes.Conv_IntFromBytes(gen_pwReadData[i])
        return dwRetVal


    def WriteSlaveEEPRom(self, bFixedAddressing, wSlaveAddress, wEEPRomStartOffset, pwWriteData, dwWriteLen, dwTimeout):
        """
        Write EEPRom data from slave

        Args:
            bFixedAddressing: True: use station addressing, False: use auto increment addressing
            wSlaveAddress: Slave Address, station or auto increment address depending on bFixedAddressing
            wEEPRomStartOffset: Address to start EEPRom Read from
            pwWriteData: Pointer to WORD array carrying the write data.
            dwWriteLen: Sizeof Write Data WORD array (in WORDS)
            dwTimeout: Timeout in milliseconds. The function will block at most for this time.
              The timeout value must not be set to EC_NOWAIT

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        if pwWriteData is None or len(pwWriteData) == 0:
            return ECError.EC_INVALIDPARM

        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_wEEPRomStartOffset = CEcWrapperTypes.Conv(wEEPRomStartOffset, "ushort")
        gen_pwWriteData = CEcWrapperTypes.Conv_IntArrayToBytePtr(pwWriteData)
        gen_dwWriteLen = CEcWrapperTypes.Conv((dwWriteLen + 1) // 2, "uint")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwWriteSlaveEEPRom(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, gen_wEEPRomStartOffset, gen_pwWriteData, gen_dwWriteLen, gen_dwTimeout))
        return dwRetVal


    @classmethod
    def GetInstallDir(cls):
        if cls.m_szInstallDir == "":
            cls.m_szInstallDir = os.path.dirname(__file__) + os.path.sep
            pathEntries = os.environ["PATH"].split(os.pathsep)
            for pathEntry in pathEntries:
                pathEntry = pathEntry.rstrip(os.path.sep) + os.path.sep
                libPath = pathEntry + CEcWrapper.GetEcWrapperName()
                if os.path.isfile(libPath):
                    cls.m_szInstallDir = pathEntry
                    break
        return cls.m_szInstallDir

    @classmethod
    def SetInstallDir(cls, path):
        cls.m_szInstallDir = path
        return True


    def InitInstance(self, oParms):
        """
        Initializes EtherCAT wrapper

        Args:
            oParms: Parameters

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        with CEcWrapperPython.m_oInstancesLock:
            CEcWrapperPython.m_oInstances.append(self)

        eRetVal = self.ReportErrorCode(self.InitInstanceInt(oParms))
        if eRetVal != ECError.EC_NOERROR:
            with CEcWrapperPython.m_oInstancesLock:
                CEcWrapperPython.m_oInstances.remove(self)

        return self.ReportErrorCode(eRetVal)

    @staticmethod
    def IsEcWrapperInstalled():
        if CEcWrapper.IsInitialized():
            return True
        installDir = CEcWrapperPython.GetInstallDir()
        libPath = installDir + CEcWrapper.GetEcWrapperName()
        if os.path.isfile(libPath):
            return True
        return False

    def InitInstanceInt(self, oParms):
        if CEcWrapperPython.IsEcWrapperInstalled() == False:
            raise CEcWrapperPythonException(ECError.EC_NOTFOUND, "ERROR: Not found", "ERROR: EcWrapper library not found.")

        installDir = CEcWrapperPython.GetInstallDir()
        CEcWrapper.ecSetInstallDir(installDir)

        apiVer = CEcWrapper.Get().ecwGetApiVer()
        if CEcWrapperPython.ECWRAPPER_API_VERSION != apiVer:
            return ECError.EC_INVALIDPARM

        # Necessary, because without EcMaster is not able to find link layer
        os.environ["PATH"] += os.pathsep + installDir

        if self.m_tRunMode != EcRunMode.None_:
            self.DeinitInstance()

        dwMasterInstanceId = 0
        oInitMaster = None #DN_EC_T_INIT_MASTER_PARMS
        oRasParmsClient = None #DN_EC_T_INITRASPARAMS
        oMasterRasParmsServer = None #DN_EC_T_INITRASPARAMS
        oSimulatorRasParmsServer = None #DN_EC_T_INITRASPARAMS
        oMonitorRasParmsServer = None #DN_EC_T_INITRASPARAMS
        oMbxGatewayParmsClient = None #DN_EC_T_INIT_MBXGATEWAY_PARMS
        oMbxGatewayParmsServer = None #DN_EC_T_INIT_MBXGATEWAY_PARMS
        bUseAuxClock = False
        bSimulator = False
        oSimulatorParams = None #DN_EC_T_SIMULATOR_INIT_PARMS
        oMonitorParams = None #DN_EC_T_MONITOR_INIT_PARMS
        oDaqParams = None #DN_EC_T_DAQ_INIT_PARMS
        oDaqReaderParams = None #DN_EC_T_DAQ_READER_PARMS

        if isinstance(oParms, DN_EC_T_INIT_PARMS_MASTER):
            dwMasterInstanceId = oParms.dwMasterInstanceId
            oInitMaster = oParms.oMaster
            bUseAuxClock = oParms.bUseAuxClock

        if isinstance(oParms, DN_EC_T_INIT_PARMS_MASTER_RAS_SERVER):
            oMasterRasParmsServer = oParms.oRas

        if isinstance(oParms, DN_EC_T_INIT_PARMS_RAS_CLIENT):
            dwMasterInstanceId = oParms.dwMasterInstanceId
            oRasParmsClient = oParms.oRas

        if isinstance(oParms, DN_EC_T_INIT_PARMS_MBXGATEWAY_CLIENT):
            oMbxGatewayParmsClient = oParms.oMbxGateway

        if isinstance(oParms, DN_EC_T_INIT_PARMS_MBXGATEWAY_SERVER):
            dwMasterInstanceId = oParms.dwMasterInstanceId
            oMbxGatewayParmsServer = oParms.oMbxGateway

        if isinstance(oParms, DN_EC_T_INIT_PARMS_SIMULATOR):
            bSimulator = True
            dwMasterInstanceId = oParms.dwSimulatorInstanceId
            oSimulatorParams = oParms.oSimulator

        if isinstance(oParms, DN_EC_T_INIT_PARMS_SIMULATOR_RAS_SERVER):
            oSimulatorRasParmsServer = oParms.oRas

        if isinstance(oParms, DN_EC_T_INIT_PARMS_MONITOR):
            dwMasterInstanceId = oParms.dwMonitorInstanceId
            oMonitorParams = oParms.oMonitor

        if isinstance(oParms, DN_EC_T_INIT_PARMS_MONITOR_RAS_SERVER):
            oMonitorRasParmsServer = oParms.oRas

        if isinstance(oParms, DN_EC_T_INIT_PARMS_DAQ):
            oDaqParams = oParms.oDaq

        if isinstance(oParms, DN_EC_T_INIT_PARMS_DAQ_READER):
            oDaqReaderParams = oParms.oDaqReader

        if bSimulator:
            dwInternalID = ctypes.c_uint(0)
            eErrCode = self.ConvResAsError(CEcWrapper.Get().ecwSimulatorInit(dwMasterInstanceId, ctypes.byref(dwInternalID)))
            if eErrCode != ECError.EC_NOERROR:
                return eErrCode

            if oSimulatorParams is not None:
                gen_oSimulatorParams = CEcWrapperTypes.Conv(oSimulatorParams)
                gen_oSimulatorParams.pfLogMsgCallBack = self.m_pfNativDbgMsgEvent
                eErrCode = self.ConvResAsError(CEcWrapper.Get().ecwInitSimulator2(dwInternalID, ctypes.byref(gen_oSimulatorParams)))
                if eErrCode != ECError.EC_NOERROR:
                    self.ConvResAsError(CEcWrapper.Get().ecwSimulatorDeinit(dwInternalID))
                    return eErrCode

                self.m_tRunMode = EcRunMode.SimulatorHil
            else:
                self.m_tRunMode = EcRunMode.SimulatorSil

            self.m_dwMasterInstanceId = dwInternalID
            return ECError.EC_NOERROR

        if oMonitorParams is not None:
            dwInternalID = ctypes.c_uint(0)
            eErrCode = self.ConvResAsError(CEcWrapper.Get().ecwMonitorInit(dwMasterInstanceId, ctypes.byref(dwInternalID)))
            if eErrCode != ECError.EC_NOERROR:
                return eErrCode

            gen_oMonitorParams = CEcWrapperTypes.Conv(oMonitorParams)
            gen_oMonitorParams.pfLogMsgCallBack = self.m_pfNativDbgMsgEvent
            eErrCode = self.ConvResAsError(CEcWrapper.Get().ecwInitMonitor2(dwInternalID, ctypes.byref(gen_oMonitorParams)))
            if eErrCode != ECError.EC_NOERROR:
                self.ConvResAsError(CEcWrapper.Get().ecwMonitorDeinit(dwInternalID))
                return eErrCode

            self.m_tRunMode = EcRunMode.Monitor
            self.m_dwMasterInstanceId = dwInternalID
            return ECError.EC_NOERROR

        if oMasterRasParmsServer is not None:
            self.m_tRunMode = EcRunMode.RasServer
            return self.RasSrvStart(oMasterRasParmsServer)

        if oSimulatorRasParmsServer is not None:
            self.m_tRunMode = EcRunMode.SimulatorRasServer
            return self.SimulatorRasSrvStart(oSimulatorRasParmsServer)

        if oMonitorRasParmsServer is not None:
            self.m_tRunMode = EcRunMode.MonitorRasServer
            return self.MonitorRasSrvStart(oMonitorRasParmsServer)

        if oMbxGatewayParmsServer is not None:
            self.m_tRunMode = EcRunMode.MbxGatewaySrv
            return self.MbxGatewaySrvStart(dwMasterInstanceId, oMbxGatewayParmsServer)

        if oDaqParams is not None:
            dwInternalID = ctypes.c_uint(0)
            eErrCode = self.ConvResAsError(CEcWrapper.Get().ecwDaqInit(ctypes.byref(dwInternalID)))
            if eErrCode != ECError.EC_NOERROR:
                return eErrCode

            gen_oDaqParams = CEcWrapperTypes.Conv(oDaqParams)
            gen_oDaqParams.pfLogMsgCallBack = self.m_pfNativDbgMsgEvent
            eErrCode = self.ConvResAsError(CEcWrapper.Get().ecwDaqRecCreate2(dwInternalID, ctypes.byref(gen_oDaqParams)))
            if eErrCode != ECError.EC_NOERROR:
                self.ConvResAsError(CEcWrapper.Get().ecwDaqDeinit(dwInternalID))
                return eErrCode

            self.m_tRunMode = EcRunMode.Daq
            self.m_dwMasterInstanceId = dwInternalID
            return ECError.EC_NOERROR

        if oDaqReaderParams is not None:
            dwInternalID = ctypes.c_uint(0)
            eErrCode = self.ConvResAsError(CEcWrapper.Get().ecwDaqInit(ctypes.byref(dwInternalID)))
            if eErrCode != ECError.EC_NOERROR:
                return eErrCode

            gen_oDaqReaderParams = CEcWrapperTypes.Conv(oDaqReaderParams)
            gen_oDaqReaderParams.pfLogMsgCallBack = self.m_pfNativDbgMsgEvent
            eErrCode = self.ConvResAsError(CEcWrapper.Get().ecwDaqReaderCreate2(dwInternalID, ctypes.byref(gen_oDaqReaderParams)))
            if eErrCode != ECError.EC_NOERROR:
                self.ConvResAsError(CEcWrapper.Get().ecwDaqDeinit(dwInternalID))
                return eErrCode

            self.m_tRunMode = EcRunMode.DaqReader
            self.m_dwMasterInstanceId = dwInternalID
            return ECError.EC_NOERROR

        if oInitMaster is not None:
            self.m_tRunMode = EcRunMode.Master

            dwInternalID = ctypes.c_uint(0)
            self.m_dwMasterInstanceId = dwMasterInstanceId
            eRetVal = self.ConvResAsError(CEcWrapper.Get().ecwInit(self.m_dwMasterInstanceId, False, ctypes.byref(dwInternalID)))
            if eRetVal != ECError.EC_NOERROR:
                return eRetVal

            self.m_dwMasterInstanceId = dwInternalID

            if bUseAuxClock:
                dwCpuIndex = 0
                dwBusCycleTimeUsec = oInitMaster.dwBusCycleTimeUsec
                if dwBusCycleTimeUsec < 10:
                    dwBusCycleTimeUsec = 10

                #// Create timing event to trigger the job task
                self.m_pvTimingEvent = CEcWrapper.Get().ecwOsCreateEvent()
                if self.m_pvTimingEvent is None:
                    CEcWrapper.Get().ecwOsDbgMsg("ERROR: insufficient memory to create timing event!\n")
                    CEcWrapper.Get().ecwDone(self.m_dwMasterInstanceId)
                    return ECError.EC_NOMEMORY

                eErrCode = self.ConvResAsError(CEcWrapper.Get().ecwOsAuxClkInit(dwCpuIndex, 1000000 / dwBusCycleTimeUsec, self.m_pvTimingEvent))
                if ECError.EC_NOERROR != eErrCode:
                    CEcWrapper.Get().ecwOsDbgMsg("ERROR at auxiliary clock initialization!\n")
                    CEcWrapper.Get().ecwDone(self.m_dwMasterInstanceId)
                    return eErrCode

            gen_oInitMaster = CEcWrapperTypes.Conv(oInitMaster)
            gen_oInitMaster.pfLogMsgCallBack = self.m_pfNativDbgMsgEvent
            eRetVal = self.ConvResAsError(CEcWrapper.Get().ecwInitMaster2(self.m_dwMasterInstanceId, ctypes.byref(gen_oInitMaster)))

            return eRetVal

        if oRasParmsClient is not None:
            self.m_tRunMode = EcRunMode.RasClient
            self.m_dwMasterInstanceId = dwMasterInstanceId

            #// Initialize the native Remote API
            CEcWrapperPython.m_dwRasConnectionCounter = CEcWrapperPython.m_dwRasConnectionCounter + 1
            if CEcWrapperPython.m_dwRasConnectionCounter == 1:
                umClntParms = SDN_ATEMRAS_T_CLNTPARMS(
                    cpuAffinityMask=0,
                    dwAdmPrio=1,
                    dwAdmStackSize=0x1000,
                    pvNotifCtxt=None,
                    pfNotification=self.m_pfNativRasEvent,
                    dwLogLevel=oRasParmsClient.dwLogLevel.value,
                    pfLogMsgCallBack=self.m_pfNativDbgMsgEvent
                )

                eRetVal2 = self.ConvResAsError(CEcWrapper.Get().ecwRasClntInit(umClntParms, False))
                if eRetVal2 != ECError.EC_NOERROR:
                    return eRetVal2

            dwInternalID = ctypes.c_uint(0)
            self.m_dwMasterInstanceId = dwMasterInstanceId
            eRetVal = self.ConvResAsError(CEcWrapper.Get().ecwInit(self.m_dwMasterInstanceId, True, ctypes.byref(dwInternalID)))
            if eRetVal != ECError.EC_NOERROR:
                return eRetVal

            self.m_dwMasterInstanceId = dwInternalID

            #/* Establish a RAS connection to the remote EtherCAT master */
            return self.RasClntAddConnection(oRasParmsClient)

        if oMbxGatewayParmsClient is not None:
            self.m_tRunMode = EcRunMode.MbxGateway
            self.m_dwMasterInstanceId = dwMasterInstanceId

            #// Initialize the mailbox gateway Remote API
            CEcWrapperPython.m_dwMbxGatewayConnectionCounter = CEcWrapperPython.m_dwMbxGatewayConnectionCounter + 1
            if CEcWrapperPython.m_dwMbxGatewayConnectionCounter == 1:
                umClntParms = SDN_EC_T_MBX_GATEWAY_CLNT_PARMS(
                    dwLogLevel=oMbxGatewayParmsClient.dwLogLevel.value,
                    pfLogMsgCallBack=self.m_pfNativDbgMsgEvent
                )

                eRetVal2 = self.ConvResAsError(CEcWrapper.Get().ecwMbxGatewayClntInit(umClntParms))
                if eRetVal2 != ECError.EC_NOERROR:
                    return eRetVal2

            dwInternalID = ctypes.c_uint(0)
            self.m_dwMasterInstanceId = dwMasterInstanceId
            eRetVal = self.ConvResAsError(CEcWrapper.Get().ecwInit2(self.m_dwMasterInstanceId, False, True, ctypes.byref(dwInternalID)))
            if eRetVal != ECError.EC_NOERROR:
                return eRetVal

            self.m_dwMasterInstanceId = dwInternalID

            #/* Establish a mailbox gateway connection */
            return self.MbxGatewayClntAddConnection(oMbxGatewayParmsClient)

        return ECError.EC_INVALIDPARM


    def DeinitInstance(self):
        """
        Deinitialize EtherCAT wrapper

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        eRetVal = ECError.EC_NOERROR

        if self.m_tRunMode == EcRunMode.RasClient:
            #// RasClient

            #/* remove the ras client connection */
            eRetVal = self.RasClntRemoveConnection(2000)
            if eRetVal != ECError.EC_NOERROR:
                return self.ReportErrorCode(eRetVal)

            if CEcWrapperPython.m_dwRasConnectionCounter > 0:
                CEcWrapperPython.m_dwRasConnectionCounter = CEcWrapperPython.m_dwRasConnectionCounter - 1

                if CEcWrapperPython.m_dwRasConnectionCounter == 0:
                    CEcWrapper.Get().ecwRasClntClose(2000)

        if self.m_tRunMode == EcRunMode.MbxGateway:
            #// MbxGateway

            #/* remove the mailbox gateway client connection */
            eRetVal = self.MbxGatewayClntRemoveConnection()
            if eRetVal != ECError.EC_NOERROR:
                return self.ReportErrorCode(eRetVal)

            if CEcWrapperPython.m_dwMbxGatewayConnectionCounter > 0:
                CEcWrapperPython.m_dwMbxGatewayConnectionCounter = CEcWrapperPython.m_dwMbxGatewayConnectionCounter - 1

                if CEcWrapperPython.m_dwMbxGatewayConnectionCounter == 0:
                    CEcWrapper.Get().ecwMbxGatewayClntDeinit(2000)

        if self.m_tRunMode == EcRunMode.MbxGatewaySrv:
            #// MbxGateway Server
            eRetVal = self.MbxGatewaySrvStop(3000)
            if eRetVal != ECError.EC_NOERROR:
                return self.ReportErrorCode(eRetVal)

        if self.m_tRunMode == EcRunMode.Master:
            #// Local
            eRetVal = self.ConvResAsError(
                CEcWrapper.Get().ecwDeinitMaster(self.m_dwMasterInstanceId))

            if self.m_pvTimingEvent is not None:
                CEcWrapper.Get().ecwOsAuxClkDeinit()

                CEcWrapper.Get().ecwOsDeleteEvent(self.m_pvTimingEvent)
                self.m_pvTimingEvent = None

        if self.m_tRunMode == EcRunMode.SimulatorSil or self.m_tRunMode == EcRunMode.SimulatorHil:
            #// Simulator

            if self.m_tRunMode == EcRunMode.SimulatorHil:
                eRetVal = self.ConvResAsError(
                    CEcWrapper.Get().ecwDeinitSimulator(self.m_dwMasterInstanceId))
                if eRetVal != ECError.EC_NOERROR:
                    return self.ReportErrorCode(eRetVal)

            eRetVal = self.ConvResAsError(
                CEcWrapper.Get().ecwSimulatorDeinit(self.m_dwMasterInstanceId))
            if eRetVal != ECError.EC_NOERROR:
                return self.ReportErrorCode(eRetVal)

        if self.m_tRunMode == EcRunMode.Monitor:
            #// Monitor

            eRetVal = self.ConvResAsError(
                CEcWrapper.Get().ecwDeinitMonitor(self.m_dwMasterInstanceId))
            if eRetVal != ECError.EC_NOERROR:
                return self.ReportErrorCode(eRetVal)

            eRetVal = self.ConvResAsError(
                CEcWrapper.Get().ecwMonitorDeinit(self.m_dwMasterInstanceId))
            if eRetVal != ECError.EC_NOERROR:
                return self.ReportErrorCode(eRetVal)

        if self.m_tRunMode == EcRunMode.Daq:
            #// Daq

            eRetVal = self.ConvResAsError(
                CEcWrapper.Get().ecwDaqRecDelete(self.m_dwMasterInstanceId))
            if eRetVal != ECError.EC_NOERROR:
                return self.ReportErrorCode(eRetVal)

            eRetVal = self.ConvResAsError(
                CEcWrapper.Get().ecwDaqDeinit(self.m_dwMasterInstanceId))
            if eRetVal != ECError.EC_NOERROR:
                return self.ReportErrorCode(eRetVal)

        if self.m_tRunMode == EcRunMode.DaqReader:
            #// DaqReader

            eRetVal = self.ConvResAsError(
                CEcWrapper.Get().ecwDaqReaderDelete(self.m_dwMasterInstanceId))
            if eRetVal != ECError.EC_NOERROR:
                return self.ReportErrorCode(eRetVal)

            eRetVal = self.ConvResAsError(
                CEcWrapper.Get().ecwDaqDeinit(self.m_dwMasterInstanceId))
            if eRetVal != ECError.EC_NOERROR:
                return self.ReportErrorCode(eRetVal)

        if self.m_tRunMode == EcRunMode.RasServer:
            #// RasServer
            eRetVal = self.RasSrvStop(3000)
            if eRetVal != ECError.EC_NOERROR:
                return self.ReportErrorCode(eRetVal)

        if self.m_tRunMode == EcRunMode.SimulatorRasServer:
            #// SimulatorRasServer
            eRetVal = self.SimulatorRasSrvStop(3000)
            if eRetVal != ECError.EC_NOERROR:
                return self.ReportErrorCode(eRetVal)

        if self.m_tRunMode == EcRunMode.MonitorRasServer:
            #// MonitorRasServer
            eRetVal = self.MonitorRasSrvStop(3000)
            if eRetVal != ECError.EC_NOERROR:
                return self.ReportErrorCode(eRetVal)

        CEcWrapper.Get().ecwDone(self.m_dwMasterInstanceId)

        with CEcWrapperPython.m_oInstancesLock:
            CEcWrapperPython.m_oInstances.remove(self)

        self.m_tRunMode = EcRunMode.None_

        #//delete this
        return self.ReportErrorCode(eRetVal)


    @classmethod
    def ConvResAsError(cls, obj, skipThrow = False):
        return cls.ReportErrorCode(ECError(obj), skipThrow)


    @classmethod
    def ReportErrorCode(cls, eErrCode, skipThrow = False):
        if cls.EnableExceptionHandling and eErrCode != ECError.EC_NOERROR and skipThrow == False:
            text = cls.GetErrorText(eErrCode)
            raise CEcWrapperPythonException(eErrCode, text, cls.FormatExceptionMessage(eErrCode, text))
        return eErrCode


    @classmethod
    def FormatExceptionMessage(cls, eErrCode, text):
        frame = inspect.stack()[2][0]
        if frame.f_code.co_name == "ConvResAsError":
            frame = inspect.stack()[3][0]
        func = frame.f_code.co_name
        args, _, _, values = inspect.getargvalues(frame)

        errMsg = "{} (0x{:08X})".format(text, eErrCode)
        if func == "InitInstance" and text == "Unknown Error 0x9811000B":
            apiVer = CEcWrapper.Get().ecwGetApiVer()
            return "Python wrapper and native wrapper are incompatible ({} != {}).".format(CEcWrapperPython.ECWRAPPER_API_VERSION, apiVer)
        if func == "SetMasterState":
            return "Cannot set master state to {}: {}.".format(str(values["eReqState"]), errMsg)
        if func == "RegisterClient":
            return "Cannot register client: {}.".format(errMsg)
        if func == "ConfigureMaster":
            return "Cannot configure EtherCAT-Master: {}.".format(errMsg)
        if func == "ConfigureNetwork":
            return "Cannot configure EtherCAT-Simulator: {}.".format(errMsg)

        return "{} failed: {}.".format(func, errMsg)


    def PerfMeasInit(self, dwlFreqSet, dwNumMeas):
        """
        Initialize performance measurement

        Args:
            dwlFreqSet: TSC frequency, 0: auto-calibrate
            dwNumMeas: Number of elements to be allocated in in pTscMeasDesc->aTscTime
        """
        if self.m_pTscMeasDesc != ctypes.c_void_p(None): return
        CEcWrapper.Get().ecwPerfMeasInit(ctypes.byref(self.m_pTscMeasDesc), dwlFreqSet, dwNumMeas)


    def PerfMeasDeinit(self):
        """
        Deinitialize performance measurement
        """
        if self.m_pTscMeasDesc != ctypes.c_void_p(None): return
        CEcWrapper.Get().ecwPerfMeasDeinit(self.m_pTscMeasDesc)
        self.m_pTscMeasDesc = ctypes.c_void_p(None)


    def PerfMeasEnable(self):
        """
        Enable performance measurement
        """
        if self.m_pTscMeasDesc == ctypes.c_void_p(None): return
        CEcWrapper.Get().ecwPerfMeasEnable(self.m_pTscMeasDesc)


    def PerfMeasStart(self, dwIndex):
        """
        Start measurement

        Args:
            dwIndex: Measurement index, 0xFFFFFFFF: all indexes
        """
        if self.m_pTscMeasDesc == ctypes.c_void_p(None): return
        CEcWrapper.Get().ecwPerfMeasStart(self.m_pTscMeasDesc, dwIndex)


    def PerfMeasEnd(self, dwIndex):
        """
        End measurement

        Args:
            dwIndex: Measurement index, 0xFFFFFFFF: all indexes
        """
        if self.m_pTscMeasDesc == ctypes.c_void_p(None): return
        CEcWrapper.Get().ecwPerfMeasEnd(self.m_pTscMeasDesc, dwIndex)


    def PerfMeasReset(self, dwIndex):
        """
        Reset measurement

        Args:
            dwIndex: Measurement index, 0xFFFFFFFF: all indexes
        """
        if self.m_pTscMeasDesc == ctypes.c_void_p(None): return
        CEcWrapper.Get().ecwPerfMeasReset(self.m_pTscMeasDesc, dwIndex)


    def PerfMeasDisable(self):
        """
        Disable performance measurement
        """
        if self.m_pTscMeasDesc == ctypes.c_void_p(None): return
        CEcWrapper.Get().ecwPerfMeasDisable(self.m_pTscMeasDesc)


    def PerfMeasShow(self, dwIndex, aszMeasCaption):
        """
        Show measurement results

        Args:
            dwIndex: Measurement index, 0xFFFFFFFF: all indexes
            aszMeasCaption: Measurement caption
        """
        if self.m_pTscMeasDesc == ctypes.c_void_p(None): return
        gen_aszMeasCaption = CEcWrapperTypes.Conv(aszMeasCaption, "string")
        CEcWrapper.Get().ecwPerfMeasShow(self.m_pTscMeasDesc, dwIndex, gen_aszMeasCaption)


    def PerfMeasSetIrqCtlEnabled(self, bEnabled):
        """
        PerfMeasSetIrqCtlEnabled

        Args:
            bEnabled: True, to enable
        """
        if self.m_pTscMeasDesc == ctypes.c_void_p(None): return
        CEcWrapper.Get().ecwPerfMeasSetIrqCtlEnabled(bEnabled)


    def ConnectPorts(self, wCfgFixedAddress1, byPort1, wCfgFixedAddress2, byPort2):
        """
        Connect a slave ESC port to another slave ESC port or to a network adapter.

        Args:
            wCfgFixedAddress1:
            byPort1:
            wCfgFixedAddress2:
            byPort2:

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        gen_wCfgFixedAddress1 =  CEcWrapperTypes.Conv(wCfgFixedAddress1, "ushort")
        gen_byPort1 =  CEcWrapperTypes.Conv(byPort1, "byte")
        gen_wCfgFixedAddress2 =  CEcWrapperTypes.Conv(wCfgFixedAddress2, "ushort")
        gen_byPort2 = CEcWrapperTypes.Conv(byPort2, "byte")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwConnectPorts2(self.m_dwMasterInstanceId, gen_wCfgFixedAddress1, gen_byPort1, gen_wCfgFixedAddress2, gen_byPort2))
        return self.ReportErrorCode(gen_dwRetVal)


    def PerfMeasAppCreate(self, pPerfMeasAppParms, out_ppvPerfMeas): # ret: ECError
        """
        Create PerfMeas object and bind it to the master instance

        This API can be called multiple times to create PerfMeas objects. The perfomance
        counters in each of the objects can be accessed in the following two ways:
        - by passing the PerfMeas object and the index of the performance measurement.
        The index ranges from [0-pPerfMeasAppParms->dwNumAppMeas]
        - by passing EC_NULL instead of a PerfMeas object and an index.
        In this case the index works across all PerfMeas objects bound to the master instance.

        Args:
            pPerfMeasAppParms: Pointer to parameter definitions
            ppvPerfMeas: Created PerfMeas object.

        Returns:
            EC_E_NOERROR or an error code
        """
        out_ppvPerfMeas.value = None
        ppvPerfMeas = ctypes.c_void_p(None)
        gen_pPerfMeasAppParms = CEcWrapperTypes.Conv(pPerfMeasAppParms, "EC_T_PERF_MEAS_APP_PARMS")
        gen_ppvPerfMeas = CEcWrapperTypes.Conv(ppvPerfMeas, "IntPtr")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwPerfMeasAppCreate(self.m_dwMasterInstanceId, ctypes.byref(gen_pPerfMeasAppParms), gen_ppvPerfMeas))
        out_ppvPerfMeas.value = CEcWrapperTypes.Conv(ppvPerfMeas, "IntPtr")
        return gen_dwRetVal

    def PerfMeasAppDelete(self, pvPerfMeas): # ret: ECError
        """
        Delete application performance measurement and unbind it from the master instance

        Objects which are not deleted using PerfMeasAppDelete are automatically deleted when calling
        DeinitMaster.

        \note This invalidates the global index used when passing EC_NULL into the other PerfMeasApp functions

        Args:
            pvPerfMeas: PerfMeas object to delete

        Returns:
            EC_E_NOERROR or an error code
        """
        gen_pvPerfMeas = CEcWrapperTypes.Conv(pvPerfMeas, "IntPtr")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwPerfMeasAppDelete(self.m_dwMasterInstanceId, gen_pvPerfMeas))
        return gen_dwRetVal

    def PerfMeasAppStart(self, pvPerfMeas, dwIndex): # ret: ECError
        """
        Start application performance measurement

        Args:
            pvPerfMeas: PerfMeas object or EC_NULL to use continuous index
            dwIndex: Index of the performance measurement

        Returns:
            EC_E_NOERROR or an error code
        """
        gen_pvPerfMeas = CEcWrapperTypes.Conv(pvPerfMeas, "IntPtr")
        gen_dwIndex = CEcWrapperTypes.Conv(dwIndex, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwPerfMeasAppStart(self.m_dwMasterInstanceId, gen_pvPerfMeas, gen_dwIndex))
        return gen_dwRetVal

    def PerfMeasAppEnd(self, pvPerfMeas, dwIndex): # ret: ECError
        """
        Stop application performance measurement

        Args:
            pvPerfMeas: PerfMeas object or EC_NULL to use continuous index
            dwIndex: Index of the performance measurement

        Returns:
            EC_E_NOERROR or an error code
        """
        gen_pvPerfMeas = CEcWrapperTypes.Conv(pvPerfMeas, "IntPtr")
        gen_dwIndex = CEcWrapperTypes.Conv(dwIndex, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwPerfMeasAppEnd(self.m_dwMasterInstanceId, gen_pvPerfMeas, gen_dwIndex))
        return gen_dwRetVal

    def PerfMeasAppReset(self, pvPerfMeas, dwIndex): # ret: ECError
        """
        Reset application performance measurement

        Args:
            pvPerfMeas: PerfMeas object or EC_NULL to use continuous index
            dwIndex: Index of the performance measurement, use 0xFFFFFFFF to reset all

        Returns:
            EC_E_NOERROR or an error code
        """
        gen_pvPerfMeas = CEcWrapperTypes.Conv(pvPerfMeas, "IntPtr")
        gen_dwIndex = CEcWrapperTypes.Conv(dwIndex, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwPerfMeasAppReset(self.m_dwMasterInstanceId, gen_pvPerfMeas, gen_dwIndex))
        return gen_dwRetVal

    def PerfMeasAppGetRaw(self, pvPerfMeas, dwIndex, ref_pPerfMeasVal, ref_pPerfMeasHistogram, dwPerfMeasNumOf): # ret: ECError
        """
        Get raw data of one/all application performance measurement

        Args:
            pvPerfMeas: PerfMeas object or EC_NULL to use continuous index
            dwIndex: Index of the performance measurement, use 0xFFFFFFFF to get all
            pPerfMeasVal: Pointer to a buffer receiving one/all performance measurement values or EC_NULL
            pPerfMeasHistogram: Pointer to a buffer receiving one/all performance measurement histograms or EC_NULL
            dwPerfMeasNumOf: Number of elements allocated in pPerfMeasVal and pPerfMeasHistogram

        Returns:
            EC_E_NOERROR or an error code
        """
        gen_pvPerfMeas = CEcWrapperTypes.Conv(pvPerfMeas, "IntPtr")
        gen_dwIndex = CEcWrapperTypes.Conv(dwIndex, "uint")
        gen_pPerfMeasVal = None
        if ref_pPerfMeasVal is not None:
            gen_pPerfMeasVal = (SDN_EC_T_PERF_MEAS_VAL * dwPerfMeasNumOf)()
        gen_pPerfMeasHistogram = None
        if ref_pPerfMeasHistogram is not None:
            gen_pPerfMeasHistogram = (SDN_EC_T_PERF_MEAS_HISTOGRAM * dwPerfMeasNumOf)()
        gen_dwPerfMeasNumOf = CEcWrapperTypes.Conv(dwPerfMeasNumOf, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwPerfMeasAppGetRaw(self.m_dwMasterInstanceId, gen_pvPerfMeas, gen_dwIndex, gen_pPerfMeasVal, gen_pPerfMeasHistogram, gen_dwPerfMeasNumOf))
        if gen_dwRetVal == ECError.EC_NOERROR:
            if ref_pPerfMeasVal is not None:
                pPerfMeasVal = [CEcWrapperTypes.Conv(gen_pPerfMeasVal[i]) for i in range(dwPerfMeasNumOf)]
                ref_pPerfMeasVal.value = pPerfMeasVal
            if ref_pPerfMeasHistogram is not None:
                pPerfMeasHistogram = [0] * dwPerfMeasNumOf
                for i in range(dwPerfMeasNumOf):
                    pPerfMeasHistogram[i] = CEcWrapperTypes.Conv(gen_pPerfMeasHistogram[i])
                    if pPerfMeasHistogram[i].aBins is not None:
                        pPerfMeasHistogram[i].aBinsObj = [0] * pPerfMeasHistogram[i].dwBinCount
                        for j in range(pPerfMeasHistogram[i].dwBinCount):
                            outData2 =  ctypes.c_uint(0)
                            CEcWrapper.Get().ecwGetObjectByIdx(pPerfMeasHistogram[i].aBins, j, 4, ctypes.pointer(outData2))
                            pPerfMeasHistogram[i].aBinsObj[j] = outData2.value
                        CEcWrapper.Get().ecwFreeObject(pPerfMeasHistogram[i].aBins)
                ref_pPerfMeasHistogram.value = pPerfMeasHistogram
        return gen_dwRetVal

    def PerfMeasAppGetInfo(self, pvPerfMeas, dwIndex, out_pPerfMeasInfo, dwPerfMeasNumOf): # ret: ECError
        """
        Get general info about one/all application performance measurement

        Args:
            pvPerfMeas: PerfMeas object or EC_NULL to use continous index
            dwIndex: Index of the performance measurement information, use 0xFFFFFFFF to get all
            pPerfMeasInfo: Pointer to a buffer receiving one/all performance measurement information
            dwPerfMeasNumOf: Number of elements allocated in pPerfMeasInfo

        Returns:
            EC_E_NOERROR or an error code
        """
        out_pPerfMeasInfo.value = None
        gen_pvPerfMeas = CEcWrapperTypes.Conv(pvPerfMeas, "IntPtr")
        gen_dwIndex = CEcWrapperTypes.Conv(dwIndex, "uint")
        gen_pPerfMeasInfo = (SDN_EC_T_PERF_MEAS_INFO * dwPerfMeasNumOf)()
        gen_dwPerfMeasNumOf = CEcWrapperTypes.Conv(dwPerfMeasNumOf, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwPerfMeasAppGetInfo(self.m_dwMasterInstanceId, gen_pvPerfMeas, gen_dwIndex, gen_pPerfMeasInfo, gen_dwPerfMeasNumOf))
        if gen_dwRetVal == ECError.EC_NOERROR:
            pPerfMeasInfo = [CEcWrapperTypes.Conv(gen_pPerfMeasInfo[i]) for i in range(dwPerfMeasNumOf)]
            out_pPerfMeasInfo.value = pPerfMeasInfo
        return gen_dwRetVal

    def PerfMeasAppGetNumOf(self, pvPerfMeas, out_pdwNumOf): # ret: ECError
        """
        Reset number of application performance measurement

        Args:
            pvPerfMeas: PerfMeas object or EC_NULL to get the number of performance measurements in all PerfMeas objects
            pdwNumOf: Number of performance measurements

        Returns:
            EC_E_NOERROR or an error code
        """
        out_pdwNumOf.value = 0
        pdwNumOf = 0
        gen_pvPerfMeas = CEcWrapperTypes.Conv(pvPerfMeas, "IntPtr")
        gen_pdwNumOf = CEcWrapperTypes.Conv(pdwNumOf, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwPerfMeasAppGetNumOf(self.m_dwMasterInstanceId, gen_pvPerfMeas, ctypes.pointer(gen_pdwNumOf)))
        out_pdwNumOf.value = CEcWrapperTypes.Conv(gen_pdwNumOf, "uint")
        return gen_dwRetVal

    def PerfMeasInternalResetByTaskId(self, dwTaskId, dwIndex): # ret: ECError
        """
        Reset internal performance measurement

        Args:
            dwTaskId: Task Job ID
            dwIndex: Index of the performance measurement, use 0xFFFFFFFF to reset all

        Returns:
            EC_E_NOERROR or an error code
        """
        gen_dwTaskId = CEcWrapperTypes.Conv(dwTaskId, "uint")
        gen_dwIndex = CEcWrapperTypes.Conv(dwIndex, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwPerfMeasInternalResetByTaskId(self.m_dwMasterInstanceId, gen_dwTaskId, gen_dwIndex))
        return gen_dwRetVal

    def PerfMeasInternalGetRawByTaskId(self, dwTaskId, dwIndex, ref_pPerfMeasVal, ref_pPerfMeasHistogram, dwPerfMeasNumOf): # ret: ECError
        """
        Get raw data of one/all application performance measurement

        Args:
            dwTaskId: Task Job ID
            dwIndex: Index of the performance measurement, use 0xFFFFFFFF to get all
            pPerfMeasVal: Pointer to a buffer receiving one/all performance measurement values or EC_NULL
            pPerfMeasHistogram: Pointer to a buffer receiving one/all performance measurement histograms or EC_NULL
            dwPerfMeasNumOf: Number of elements allocated in pPerfMeasVal and pPerfMeasHistogram

        Returns:
            EC_E_NOERROR or an error code
        """
        gen_dwTaskId = CEcWrapperTypes.Conv(dwTaskId, "uint")
        gen_dwIndex = CEcWrapperTypes.Conv(dwIndex, "uint")
        gen_pPerfMeasVal = None
        if ref_pPerfMeasVal is not None:
            gen_pPerfMeasVal = (SDN_EC_T_PERF_MEAS_VAL * dwPerfMeasNumOf)()
        gen_pPerfMeasHistogram = None
        if ref_pPerfMeasHistogram is not None:
            gen_pPerfMeasHistogram = (SDN_EC_T_PERF_MEAS_HISTOGRAM * dwPerfMeasNumOf)()
        gen_dwPerfMeasNumOf = CEcWrapperTypes.Conv(dwPerfMeasNumOf, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwPerfMeasInternalGetRawByTaskId(self.m_dwMasterInstanceId, gen_dwTaskId, gen_dwIndex, gen_pPerfMeasVal, gen_pPerfMeasHistogram, gen_dwPerfMeasNumOf))
        if gen_dwRetVal == ECError.EC_NOERROR:
            if ref_pPerfMeasVal is not None:
                pPerfMeasVal = [CEcWrapperTypes.Conv(gen_pPerfMeasVal[i]) for i in range(dwPerfMeasNumOf)]
                ref_pPerfMeasVal.value = pPerfMeasVal
            if ref_pPerfMeasHistogram is not None:
                pPerfMeasHistogram = [0] * dwPerfMeasNumOf
                for i in range(dwPerfMeasNumOf):
                    pPerfMeasHistogram[i] = CEcWrapperTypes.Conv(gen_pPerfMeasHistogram[i])
                    if pPerfMeasHistogram[i].aBins is not None:
                        pPerfMeasHistogram[i].aBinsObj = [0] * pPerfMeasHistogram[i].dwBinCount
                        for j in range(pPerfMeasHistogram[i].dwBinCount):
                            outData2 =  ctypes.c_uint(0)
                            CEcWrapper.Get().ecwGetObjectByIdx(pPerfMeasHistogram[i].aBins, j, 4, ctypes.pointer(outData2))
                            pPerfMeasHistogram[i].aBinsObj[j] = outData2.value
                        CEcWrapper.Get().ecwFreeObject(pPerfMeasHistogram[i].aBins)
                ref_pPerfMeasHistogram.value = pPerfMeasHistogram
        return gen_dwRetVal

    def PerfMeasInternalGetInfoByTaskId(self, dwTaskId, dwIndex, out_pPerfMeasInfo, dwPerfMeasNumOf): # ret: ECError
        """
        Get general info about one/all internal performance measurement

        Args:
            dwTaskId: Task Job ID
            dwIndex: Index of the performance measurement, use 0xFFFFFFFF to get all
            pPerfMeasInfo: Pointer to a buffer receiving one/all performance measurement infos
            dwPerfMeasNumOf: Number of elements allocated in pPerfMeasInfo

        Returns:
            EC_E_NOERROR or an error code
        """
        out_pPerfMeasInfo.value = None
        gen_dwTaskId = CEcWrapperTypes.Conv(dwTaskId, "uint")
        gen_dwIndex = CEcWrapperTypes.Conv(dwIndex, "uint")
        gen_pPerfMeasInfo = (SDN_EC_T_PERF_MEAS_INFO * dwPerfMeasNumOf)()
        gen_dwPerfMeasNumOf = CEcWrapperTypes.Conv(dwPerfMeasNumOf, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwPerfMeasInternalGetInfoByTaskId(self.m_dwMasterInstanceId, gen_dwTaskId, gen_dwIndex, gen_pPerfMeasInfo, gen_dwPerfMeasNumOf))
        if gen_dwRetVal == ECError.EC_NOERROR:
            pPerfMeasInfo = [CEcWrapperTypes.Conv(gen_pPerfMeasInfo[i]) for i in range(dwPerfMeasNumOf)]
            out_pPerfMeasInfo.value = pPerfMeasInfo
        return gen_dwRetVal


    def PerfMeasInternalGetNumOfByTaskId(self, dwTaskId, out_pdwNumOf): # ret: ECError
        """
        Reset number of internal performance measurement

        Args:
            dwTaskId: Task Job ID
            pdwNumOf: Number of performance measurements

        Returns:
            EC_E_NOERROR or an error code
        """
        out_pdwNumOf.value = 0
        pdwNumOf = 0
        gen_dwTaskId = CEcWrapperTypes.Conv(dwTaskId, "uint")
        gen_pdwNumOf = CEcWrapperTypes.Conv(pdwNumOf, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwPerfMeasInternalGetNumOfByTaskId(self.m_dwMasterInstanceId, gen_dwTaskId, ctypes.pointer(gen_pdwNumOf)))
        out_pdwNumOf.value = CEcWrapperTypes.Conv(gen_pdwNumOf, "uint")
        return gen_dwRetVal

    #//////////////////////////////////////////////////////////////////////////
    #// @CODEGENERATOR_IMPL_BEGIN@
    def SetMasterRedStateReq(self, bActive): # ret: ECError
        """
        Requests Master Redundancy State ACTIVE / INACTIVE.

        Args:
            bActive: 

        Returns:
            - EC_E_NOERROR or error code
            - EC_E_INVALIDSTATE if MasterRedParms.bEnabled = EC_FALSE or Master not initialized
            - EC_E_NOTSUPPORTED if EC-Master stack does not include Master Redundancy support
        """
        gen_bActive = CEcWrapperTypes.Conv(bActive, "bool")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwSetMasterRedStateReq(self.m_dwMasterInstanceId, gen_bActive))
        return gen_dwRetVal

    def GetMasterRedState(self, pbActive): # ret: ECError
        """
        Gets Master Redundancy State (ACTIVE / INACTIVE).

        Args:
            pbActive: Pointer to variable of type EC_T_BOOL. Contains Master Redundancy State on success.

        Returns:
            - EC_E_NOERROR or error code
            - EC_E_INVALIDSTATE if MasterRedParms.bEnabled = EC_FALSE or Master not initialized
            - EC_E_NOTSUPPORTED if EC-Master stack does not include Master Redundancy support
        """
        gen_pbActive = CEcWrapperTypes.Conv(pbActive, "bool")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetMasterRedState(self.m_dwMasterInstanceId, ctypes.pointer(gen_pbActive)))
        return gen_dwRetVal

    def GetMasterRedProcessImageInputPtr(self): # ret: byte[]
        """
        Gets the Master Redundancy process data input image pointer

        Args:

        Returns:
            Master Redundancy process data input image pointer
        """
        gen_dwRetVal = CEcWrapperTypes.ConvRes(CEcWrapper.Get().ecwGetMasterRedProcessImageInputPtr(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def GetMasterRedProcessImageOutputPtr(self): # ret: byte[]
        """
        Gets the Master Redundancy process data output image pointer

        Args:

        Returns:
            Master Redundancy process data output image pointer
        """
        gen_dwRetVal = CEcWrapperTypes.ConvRes(CEcWrapper.Get().ecwGetMasterRedProcessImageOutputPtr(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def ScanBus(self, dwTimeout): # ret: ECError
        """
        Scans all connected slaves.
        
        Scans all connected slaves connected to EC-Master. If a configuration has been loaded, a validation between the configuration and the connected slaves is done.
        This function should not be called from within the JobTask's context.

        Args:
            dwTimeout: Timeout [ms]

        Returns:
            - EC_E_NOERROR on success
            - EC_E_INVALIDSTATE if master isn't initialized
            - EC_E_MAX_BUS_SLAVES_EXCEEDED if the amount of slaves found exceeds EC_T_INIT_MASTER_PARMS.dwMaxBusSlaves
            - EC_E_BUSCONFIG_MISMATCH if the slaves found are not matching the configured ones
            - EC_E_LINE_CROSSED if a line crossed (cabling wrong) condition has been detected
            - EC_E_REDLINEBREAK if cable redundancy is configured and a line break condition has been detected
            - EC_E_JUNCTION_RED_LINE_BREAK if junction redundancy is configured and a line break condition has been detected
        """
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwScanBus(self.m_dwMasterInstanceId, gen_dwTimeout))
        return gen_dwRetVal

    def RescueScan(self, dwTimeout): # ret: ECError
        """
        Recovers the bus from permanent frame loss situations
        
        Scans all connected slaves. Closes and open ports on the network to rule out slaves which permanently discard frames.
        The Master notifies every slave port which permanently discard frames with EC_NOTIFY_FRAMELOSS_AFTER_SLAVE.
        Due to port opening and closing the scanning time is increased about 2 seconds per slave.
        The Master will not automatically re-open this port. The application can force to open the port again.
        This function may not be called from within the JobTask's context.

        Args:
            dwTimeout: Timeout [ms]

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwRescueScan(self.m_dwMasterInstanceId, gen_dwTimeout))
        return gen_dwRetVal

    def GetMasterInfo(self, out_pMasterInfo): # ret: ECError
        """
        Get generic information about the Master

        Args:
            pMasterInfo: Master information

        Returns:
            EC_E_NOERROR or error code
        """
        pMasterInfo = DN_EC_T_MASTER_INFO()
        gen_pMasterInfo = CEcWrapperTypes.Conv(pMasterInfo, "EC_T_MASTER_INFO")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetMasterInfo(self.m_dwMasterInstanceId, ctypes.pointer(gen_pMasterInfo)))
        out_pMasterInfo.value = CEcWrapperTypes.Conv(gen_pMasterInfo, "DN_EC_T_MASTER_INFO")
        return gen_dwRetVal

    def ConfigureMaster(self, eCnfType, pbyCnfData, dwCnfDataLen): # ret: ECError
        """
        Configure the Master.
        
        This function must be called after the master has been initialized. Among others the EtherCAT topology defined in the given XML configuration file will be stored internally.
        Analyzing the network including mailbox communication can be done without specifying an ENI file using eCnfType_GenPreopENI.
        
        \note A client must not be registered prior to calling this function. Existing client registrations will be dropped.

        Args:
            eCnfType: Type of configuration data provided
            pbyCnfData: Filename / configuration data, or EC_NULL if eCnfType is eCnfType_GenPreopENI
            dwCnfDataLen: Length of configuration data in byte, or zero if eCnfType is eCnfType_GenPreopENI

        Returns:
            EC_E_NOERROR or error code
        """
        gen_eCnfType = CEcWrapperTypes.Conv(eCnfType, "EC_T_CNF_TYPE")
        gen_pbyCnfData = CEcWrapperTypes.Conv(pbyCnfData, "byte[]")
        gen_dwCnfDataLen = CEcWrapperTypes.Conv(dwCnfDataLen, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwConfigureMaster(self.m_dwMasterInstanceId, gen_eCnfType, gen_pbyCnfData, gen_dwCnfDataLen))
        return gen_dwRetVal

    def ConfigLoad(self, eCnfType, pbyCnfData, dwCnfDataLen): # ret: ECError
        """
        Load the master configuration.
        
        In combination with emConfigApply, this function replaces emConfigureMaster and must be called after the master has been initialized.
        Among others the EtherCAT topology defined in the given XML configuration file will be stored internally.
        
        \note A client must not be registered prior to calling this function. Existing client registrations will be dropped.

        Args:
            eCnfType: Type of configuration data provided
            pbyCnfData: Configuration data
            dwCnfDataLen: Length of configuration data in byte

        Returns:
            EC_E_NOERROR or error code
        """
        gen_eCnfType = CEcWrapperTypes.Conv(eCnfType, "EC_T_CNF_TYPE")
        gen_pbyCnfData = CEcWrapperTypes.Conv(pbyCnfData, "byte[]")
        gen_dwCnfDataLen = CEcWrapperTypes.Conv(dwCnfDataLen, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwConfigLoad(self.m_dwMasterInstanceId, gen_eCnfType, gen_pbyCnfData, gen_dwCnfDataLen))
        return gen_dwRetVal

    def ConfigExcludeSlave(self, wStationAddress): # ret: ECError
        """
        Exclude a slave from the master configuration.
        
        It has to be called after emConfigLoad and prior to calling emConfigApply.

        Args:
            wStationAddress: Station address of the slave to be excluded. A value of 0 excludes all slaves.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_wStationAddress = CEcWrapperTypes.Conv(wStationAddress, "ushort")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwConfigExcludeSlave(self.m_dwMasterInstanceId, gen_wStationAddress))
        return gen_dwRetVal

    def ConfigIncludeSlave(self, wStationAddress): # ret: ECError
        """
        Include a slave in the master configuration
        
        Slaves that were previously excluded with emConfigSlaveExclude can be added again. It has to be called after emConfigLoad and prior to calling emConfigApply.

        Args:
            wStationAddress: Station address of the slave to be included. A value of 0 includes all slaves.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_wStationAddress = CEcWrapperTypes.Conv(wStationAddress, "ushort")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwConfigIncludeSlave(self.m_dwMasterInstanceId, gen_wStationAddress))
        return gen_dwRetVal

    def ConfigSetPreviousPort(self, wStationAddress, wStationAddressPrev, wPortPrev): # ret: ECError
        """
        Set previous port information of a slave
        
        It has to be called after emConfigLoad and prior to calling emConfigApply.

        Args:
            wStationAddress: Station address of the slave
            wStationAddressPrev: Previous slave station address
            wPortPrev: Previous port

        Returns:
            EC_E_NOERROR or error code
        """
        gen_wStationAddress = CEcWrapperTypes.Conv(wStationAddress, "ushort")
        gen_wStationAddressPrev = CEcWrapperTypes.Conv(wStationAddressPrev, "ushort")
        gen_wPortPrev = CEcWrapperTypes.Conv(wPortPrev, "ushort")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwConfigSetPreviousPort(self.m_dwMasterInstanceId, gen_wStationAddress, gen_wStationAddressPrev, gen_wPortPrev))
        return gen_dwRetVal

    def ConfigAddJunctionRedundancyConnection(self, wHeadStationAddress, wHeadRedPort, wTailStationAddress, wTailRedPort): # ret: ECError
        """
        Add a junction redundancy connection
        
        Since there is no mechanism to configure junction redundancy in the ENI, this API allows adding junction redundancy connections which will be validated by the EC-Master stack.
        It has to be called after emConfigLoad and prior to calling emConfigApply. Calling this API enables junction redundancy support implicitly.

        Args:
            wHeadStationAddress: Station address of the junction redundancy head slave. Typically this is an EtherCAT junction.
            wHeadRedPort: Port at head slave to which the junction redundancy cable is connected. Must be ESC_PORT_B
            wTailStationAddress: Station address of the junction redundancy tail slave
            wTailRedPort: Port at tail slave to which the junction redundancy cable is connected. May not be ESC_PORT_A.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_wHeadStationAddress = CEcWrapperTypes.Conv(wHeadStationAddress, "ushort")
        gen_wHeadRedPort = CEcWrapperTypes.Conv(wHeadRedPort, "ushort")
        gen_wTailStationAddress = CEcWrapperTypes.Conv(wTailStationAddress, "ushort")
        gen_wTailRedPort = CEcWrapperTypes.Conv(wTailRedPort, "ushort")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwConfigAddJunctionRedundancyConnection(self.m_dwMasterInstanceId, gen_wHeadStationAddress, gen_wHeadRedPort, gen_wTailStationAddress, gen_wTailRedPort))
        return gen_dwRetVal

    def ConfigApply(self): # ret: ECError
        """
        Apply the master configuration.
        
        It has to be called after emConfigLoad.

        Args:

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwConfigApply(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def ConfigExtend(self, bResetConfig, dwTimeout): # ret: ECError
        """
        Extends the existing network configuration
        
        This function extends the existing configuration described in the ENI to allow mailbox communication with unexpected slaves. After this function was called, unexpected slaves can reach PREOP state.
        After the configuration was extended, disconnecting any slave will generate a bus mismatch, because all the slaves are part of the configuration.
        Recalling this function with bResetConfig set to EC_FALSE will extend the configuration again by any new connected unexpected slaves. The previous extension is not deleted.
        Calling the function with bResetConfig set to EC_TRUE, reset all the previous extensions.
        
        \note This function may not be called from within the JobTask's context.

        Args:
            bResetConfig: EC_TRUE: Extended configuration will be removed
            dwTimeout: Timeout [ms]

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bResetConfig = CEcWrapperTypes.Conv(bResetConfig, "bool")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwConfigExtend(self.m_dwMasterInstanceId, gen_bResetConfig, gen_dwTimeout))
        return gen_dwRetVal

    def IsConfigured(self, pbIsConfigured): # ret: ECError
        """
        Returns if configuration has been applied

        Args:
            pbIsConfigured: EC_TRUE if configuration has been applied

        Returns:
            EC_E_NOERROR or error code
        """
        gen_pbIsConfigured = CEcWrapperTypes.Conv(pbIsConfigured, "bool")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwIsConfigured(self.m_dwMasterInstanceId, ctypes.pointer(gen_pbIsConfigured)))
        return gen_dwRetVal

    def SetMasterState(self, dwTimeout, eReqState): # ret: ECError
        """
        Set the EtherCAT master (and all slaves) into the requested state.
        
        If the function is called with EC_NOWAIT, the client may wait for reaching the requested state using the notification callback (EC_NOTIFY_STATECHANGED).\n
        Master by default will just change to a higher state, if all slaves have reached the requested state. It may happen that some slaves are in higher state at network than Master, e.g.:
        - Master and all slaves are in PREOP
        - Application requests SAFEOP
        - Master starts transition for all slaves
        - Some slaves changed to SAFEOP, but some fail and therefore stay in PREOP
        - Master state stays in PREOP, function returns with error
        
        The application can request SAFEOP again to re-request state of previously failed slaves.
        Transition to lower state: The master changes to lower state, even if one slave is not able to follow.
        This function may not be called from within the JobTask's context with dwTimeout other than EC_NOWAIT.

        Args:
            dwTimeout: Timeout [ms] This function will block until the requested state is reached or the timeout elapsed. If the timeout value is set to EC_NOWAIT the function will return immediately.
            eReqState: Requested System state

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_eReqState = CEcWrapperTypes.Conv(eReqState, "EC_T_STATE")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwSetMasterState(self.m_dwMasterInstanceId, gen_dwTimeout, gen_eReqState))
        return gen_dwRetVal

    def GetMasterState(self): # ret: DN_EC_T_STATE
        """
        Get the EtherCAT master current state.

        Args:

        Returns:
            EtherCAT master state
        """
        gen_dwRetVal = DN_EC_T_STATE(CEcWrapperTypes.ConvRes(CEcWrapper.Get().ecwGetMasterState(self.m_dwMasterInstanceId)))
        return gen_dwRetVal

    def GetMasterStateEx(self, pwCurrState, pwReqState): # ret: ECError
        """
        Get the EtherCAT master current and requested state.
        Possible return values for current and requested state:
        - #DEVICE_STATE_UNKNOWN
        - #DEVICE_STATE_INIT
        - #DEVICE_STATE_PREOP
        - #DEVICE_STATE_SAFEOP
        - #DEVICE_STATE_OP

        Args:
            pwCurrState: Current master state.
            pwReqState: Requested master state

        Returns:
            - EC_E_NOERROR on success
            - EC_E_INVALIDSTATE if master isn't initialized
            - EC_E_INVALIDPARM if the output pointers are EC_NULL
        """
        gen_pwCurrState = CEcWrapperTypes.Conv(pwCurrState, "ushort")
        gen_pwReqState = CEcWrapperTypes.Conv(pwReqState, "ushort")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetMasterStateEx(self.m_dwMasterInstanceId, ctypes.pointer(gen_pwCurrState), ctypes.pointer(gen_pwReqState)))
        return gen_dwRetVal

    def Start(self, dwTimeout): # ret: ECError
        """
        The EtherCAT master and all slaves will be set into the OPERATIONAL state
        
        \deprecated Use emSetMasterState() instead
        
        \note If the function is called with EC_NOWAIT, the client may wait for reaching the OPERATIONAL state using the notification callback (EC_NOTIFY_STATECHANGED).
        This function may not be called from within the JobTask's context.

        Args:
            dwTimeout: Timeout [ms] This function will block until the OPERATIONAL state is reached or the timeout elapsed. If the timeout value is set to EC_NOWAIT the function will return immediately.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwStart(self.m_dwMasterInstanceId, gen_dwTimeout))
        return gen_dwRetVal

    def Stop(self, dwTimeout): # ret: ECError
        """
        The EtherCAT master and all slaves will be set back into the INIT state.
        
        \deprecated Use emSetMasterState() instead
        
        \note If the function is called with EC_NOWAIT, the client may wait for reaching the INIT state using the notification callback (ECAT_NOTIFY_STATECHANGE).
        This function may not be called from within the JobTask's context.

        Args:
            dwTimeout: Timeout [ms] This function will block until the INIT state is reached or the timeout elapsed. If the timeout value is set to EC_NOWAIT the function will return immediately.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwStop(self.m_dwMasterInstanceId, gen_dwTimeout))
        return gen_dwRetVal

    def GetSlaveId(self, wStationAddress): # ret: uint
        """
        Determines the slave ID using the slave station address.

        Args:
            wStationAddress: Station address of the slave

        Returns:
            Slave ID or INVALID_SLAVE_ID if the slave could not be found or stack is not initialized
        """
        gen_wStationAddress = CEcWrapperTypes.Conv(wStationAddress, "ushort")
        gen_dwRetVal = CEcWrapperTypes.ConvRes(CEcWrapper.Get().ecwGetSlaveId(self.m_dwMasterInstanceId, gen_wStationAddress))
        return gen_dwRetVal

    def GetSlaveFixedAddr(self, dwSlaveId, out_pwFixedAddr): # ret: ECError
        """
        Determine slave station address according to its slave ID.

        Args:
            dwSlaveId: Slave ID
            pwFixedAddr: Corresponding fixed address

        Returns:
            - EC_E_NOERROR or error code
            - EC_E_NOTFOUND if slave not found
        """
        gen_dwSlaveId = CEcWrapperTypes.Conv(dwSlaveId, "uint")
        pwFixedAddr = 0
        gen_pwFixedAddr = CEcWrapperTypes.Conv(pwFixedAddr, "ushort")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetSlaveFixedAddr(self.m_dwMasterInstanceId, gen_dwSlaveId, ctypes.pointer(gen_pwFixedAddr)))
        out_pwFixedAddr.value = CEcWrapperTypes.Conv(gen_pwFixedAddr, "ushort")
        return gen_dwRetVal

    def GetSlaveIdAtPosition(self, wAutoIncAddress): # ret: uint
        """
        Determines the slave ID using the slave auto increment address.

        Args:
            wAutoIncAddress: Auto increment address of the slave

        Returns:
            Slave ID or INVALID_SLAVE_ID if the slave could not be found
        """
        gen_wAutoIncAddress = CEcWrapperTypes.Conv(wAutoIncAddress, "ushort")
        gen_dwRetVal = CEcWrapperTypes.ConvRes(CEcWrapper.Get().ecwGetSlaveIdAtPosition(self.m_dwMasterInstanceId, gen_wAutoIncAddress))
        return gen_dwRetVal

    def GetSlaveProp(self, dwSlaveId, out_pSlaveProp): # ret: bool
        """
        Determines the properties of the slave device.
        
        \deprecated Use emGetCfgSlaveInfo instead

        Args:
            dwSlaveId: Slave ID
            pSlaveProp: Slave properties

        Returns:
            EC_TRUE if the slave exists, EC_FALSE if the slave id is invalid
        """
        gen_dwSlaveId = CEcWrapperTypes.Conv(dwSlaveId, "uint")
        pSlaveProp = DN_EC_T_SLAVE_PROP()
        gen_pSlaveProp = CEcWrapperTypes.Conv(pSlaveProp, "EC_T_SLAVE_PROP")
        gen_dwRetVal = CEcWrapperTypes.ConvRes(CEcWrapper.Get().ecwGetSlaveProp(self.m_dwMasterInstanceId, gen_dwSlaveId, ctypes.pointer(gen_pSlaveProp)))
        out_pSlaveProp.value = CEcWrapperTypes.Conv(gen_pSlaveProp, "DN_EC_T_SLAVE_PROP")
        return gen_dwRetVal

    def GetSlavePortState(self, dwSlaveId, pwPortState): # ret: ECError
        """
        Returns the state of the slave ports.

        Args:
            dwSlaveId: Slave ID
            pwPortState: Slave port state.\n Format: wwww xxxx yyyy zzzz (each nibble : port 3210)\n

        Returns:
            - EC_E_NOERROR if successful
            - EC_E_INVALIDSTATE if master is not initialized
            - EC_E_NOTFOUND if the slave with ID dwSlaveId does not exist
        """
        gen_dwSlaveId = CEcWrapperTypes.Conv(dwSlaveId, "uint")
        gen_pwPortState = CEcWrapperTypes.Conv(pwPortState, "ushort")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetSlavePortState(self.m_dwMasterInstanceId, gen_dwSlaveId, ctypes.pointer(gen_pwPortState)))
        return gen_dwRetVal

    def GetSlaveState(self, dwSlaveId, out_pwCurrDevState, out_pwReqDevState): # ret: ECError
        """
        Get the slave state.
        
        The slave state is always read automatically from the AL_STATUS register whenever necessary. It is not forced by calling this function.
        This function may be called from within the JobTask's context.

        Args:
            dwSlaveId: Slave ID
            pwCurrDevState: Current slave state.
            pwReqDevState: Requested slave state

        Returns:
            - EC_E_NOERROR on success.
            - EC_E_SLAVE_NOT_PRESENT if slave not present.
            - EC_E_NOTFOUND if the slave with ID dwSlaveId does not exist in the XML file.
        """
        gen_dwSlaveId = CEcWrapperTypes.Conv(dwSlaveId, "uint")
        pwCurrDevState = 0
        gen_pwCurrDevState = CEcWrapperTypes.Conv(pwCurrDevState, "ushort")
        pwReqDevState = 0
        gen_pwReqDevState = CEcWrapperTypes.Conv(pwReqDevState, "ushort")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetSlaveState(self.m_dwMasterInstanceId, gen_dwSlaveId, ctypes.pointer(gen_pwCurrDevState), ctypes.pointer(gen_pwReqDevState)))
        out_pwReqDevState.value = CEcWrapperTypes.Conv(gen_pwReqDevState, "ushort")
        out_pwCurrDevState.value = CEcWrapperTypes.Conv(gen_pwCurrDevState, "ushort")
        return gen_dwRetVal

    def SetSlaveState(self, dwSlaveId, wNewReqDevState, dwTimeout): # ret: ECError
        """
        Set a specified slave into the requested state.
        
        The requested state shall not be higher than the overall operational state. DEVICE_STATE_BOOTSTRAP can only be requested if the slave's state is INIT.
        This function may not be called from within the JobTask's context.

        Args:
            dwSlaveId: Slave ID
            wNewReqDevState: Requested state
            dwTimeout: Timeout [ms] May not be EC_NOWAIT!

        Returns:
            - EC_E_NOERROR if successful
            - EC_E_BUSY if the master cannot execute the request at this time, the function has to be called at a later time
            - EC_E_NOTFOUND if the slave does not exist
            - EC_E_NOTREADY if the working counter was not set when requesting the slave's state (slave may not be connected or did not respond)
            - EC_E_TIMEOUT if the slave did not enter the requested state in time
            - EC_E_INVALIDSTATE if the master denies the requested state, see comments below
            - EC_E_INVALIDPARM if BOOTSTRAP was requested for a slave that does not support it
        """
        gen_dwSlaveId = CEcWrapperTypes.Conv(dwSlaveId, "uint")
        gen_wNewReqDevState = CEcWrapperTypes.Conv(wNewReqDevState, "ushort")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwSetSlaveState(self.m_dwMasterInstanceId, gen_dwSlaveId, gen_wNewReqDevState, gen_dwTimeout))
        return gen_dwRetVal

    def TferSingleRawCmd(self, byCmd, dwMemoryAddress, pbyData, wLen, dwTimeout): # ret: ECError
        """
        Transfers a single raw EtherCAT command to one or multiple slaves and waits for the result.
        
        Using this function it is possible exchange arbitrary data between the master and the slaves. When the master receives the response to the queued frame it raises EC_NOTIFY_RAWCMD_DONE to all clients.
        This function blocks until the command is completely processed. In case of read commands the slave data will be written back into the given memory area.
        If a timeout occurs (e.g. due to a bad line quality) the corresponding frame will be sent again. The timeout value and retry counter can be set using the master configuration parameters dwEcatCmdTimeout and dwEcatCmdMaxRetries.
        The call will return in any case (without waiting for the number of retries specified in dwEcatCmdMaxRetries) if the time determined with the dwTimeout parameter elapsed.
        Caveat: Using auto increment addressing (APRD, APWR, APRW) may lead to unexpected results in case the selected slave does not increment the working counter. In such cases the EtherCAT command would be handled by the slave directly behind the selected one.
        This function may not be called from within the JobTask's context.

        Args:
            byCmd: EtherCAT command type. EC_CMD_TYPE_...
            dwMemoryAddress: Slave memory address, depending on the command to be sent this is either a physical or logical address.
            pbyData: [in, out] Buffer containing or receiving transfered data
            wLen: Number of bytes to transfer
            dwTimeout: Timeout [ms]

        Returns:
            - EC_E_NOERROR if successful
            - EC_E_BUSY another transfer request is already pending
            - EC_E_NOTFOUND if the slave with ID dwSlaveId does not exist
            - EC_E_NOTREADY if the working counter was not set when sending the command (slave may not be connected or did not respond)
            - EC_E_TIMEOUT if the slave did not respond to the command
            - EC_E_BUSY if the master or the corresponding slave is currently changing its operational state
            - EC_E_INVALIDPARM if the command is not supported or the timeout value is set to EC_NOWAIT
            - EC_E_INVALIDSIZE if the size of the complete command does not fit into a single Ethernet frame. The maximum amount of data to transfer must not exceed 1486 bytes
        """
        gen_byCmd = CEcWrapperTypes.Conv(byCmd, "byte")
        gen_dwMemoryAddress = CEcWrapperTypes.Conv(dwMemoryAddress, "uint")
        gen_pbyData = CEcWrapperTypes.Conv(pbyData, "byte[]")
        gen_wLen = CEcWrapperTypes.Conv(wLen, "ushort")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwTferSingleRawCmd(self.m_dwMasterInstanceId, gen_byCmd, gen_dwMemoryAddress, gen_pbyData, gen_wLen, gen_dwTimeout))
        CEcWrapperTypes.Conv_Array(gen_pbyData, pbyData)
        return gen_dwRetVal

    def ReadSlaveRegister(self, bFixedAddressing, wSlaveAddress, wRegisterOffset, pbyData, wLen, dwTimeout): # ret: ECError
        """
        Reads data from the ESC memory of a specified slave.
        
        This function may not be called from within the JobTask's context.

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            wRegisterOffset: Register offset. I.e. use 0x0130 to read the AL Status register.
            pbyData: Buffer receiving transfered data
            wLen: Number of bytes to receive
            dwTimeout: Timeout [ms]

        Returns:
            - EC_E_NOERROR if successful
            - EC_E_SLAVE_NOT_PRESENT if slave not present
            - EC_E_BUSY another transfer request is already pending
            - EC_E_NOTFOUND if the slave with the given address does not exist
            - EC_E_NOTREADY if the working counter was not set when sending the command (slave may not be connected or did not respond)
            - EC_E_TIMEOUT if the slave did not respond to the command
            - EC_E_BUSY if the master or the corresponding slave is currently changing its operational state
            - EC_E_INVALIDPARM if the command is not supported or the timeout value is set to EC_NOWAIT
            - EC_E_INVALIDSIZE if the size of the complete command does not fit into a single Ethernet frame. The maximum amount of data to transfer must not exceed 1486 bytes
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_wRegisterOffset = CEcWrapperTypes.Conv(wRegisterOffset, "ushort")
        gen_pbyData = CEcWrapperTypes.Conv(pbyData, "byte[]")
        gen_wLen = CEcWrapperTypes.Conv(wLen, "ushort")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwReadSlaveRegister(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, gen_wRegisterOffset, gen_pbyData, gen_wLen, gen_dwTimeout))
        CEcWrapperTypes.Conv_Array(gen_pbyData, pbyData)
        return gen_dwRetVal

    def ReadSlaveRegisterReq(self, dwClientId, dwTferId, bFixedAddressing, wSlaveAddress, wRegisterOffset, pbyData, wLen): # ret: ECError
        """
        Requests data read transfer from the ESC memory of a specified slave and returns immediately.
        
        A notification EC_NOTIFY_SLAVE_REGISTER_TRANSFER is given on completion.
        This function may be called from within the JobTask's context.

        Args:
            dwClientId: Client ID returned by RegisterClient (0 if all registered clients shall be notified).
            dwTferId: Transfer ID. The application can set this ID to identify the transfer. It will be passed back to the application within EC_T_SLAVEREGISTER_TRANSFER_NTFY_DESC
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            wRegisterOffset: Register offset, e.g. use 0x0130 to read the AL Status register.
            pbyData: Buffer receiving transfered data
            wLen: Number of bytes to receive

        Returns:
            - EC_E_NOERROR if successful.
            - EC_E_SLAVE_NOT_PRESENT if slave not present.
            - EC_E_NOTFOUND if the slave with the given address does not exist.
            - EC_E_INVALIDPARM if the command is not supported or the timeout value is set to EC_NOWAIT.
            - EC_E_INVALIDSIZE if the size of the complete command does not fit into a single Ethernet frame. The maximum amount of data to transfer must not exceed 1486 bytes.
        """
        gen_dwClientId = CEcWrapperTypes.Conv(dwClientId, "uint")
        gen_dwTferId = CEcWrapperTypes.Conv(dwTferId, "uint")
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_wRegisterOffset = CEcWrapperTypes.Conv(wRegisterOffset, "ushort")
        gen_pbyData = CEcWrapperTypes.Conv(pbyData, "byte[]")
        gen_wLen = CEcWrapperTypes.Conv(wLen, "ushort")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwReadSlaveRegisterReq(self.m_dwMasterInstanceId, gen_dwClientId, gen_dwTferId, gen_bFixedAddressing, gen_wSlaveAddress, gen_wRegisterOffset, gen_pbyData, gen_wLen))
        CEcWrapperTypes.Conv_Array(gen_pbyData, pbyData)
        return gen_dwRetVal

    def WriteSlaveRegister(self, bFixedAddressing, wSlaveAddress, wRegisterOffset, pbyData, wLen, dwTimeout): # ret: ECError
        """
        Writes data into the ESC memory of a specified slave.
        
        This function may not be called from within the JobTask's context
        
        \warning Changing contents of ESC registers may lead to unpredictable behavior of the slaves and/or the master.

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            wRegisterOffset: Register offset, e.g. use 0x0120 to write to the AL Control register.
            pbyData: Buffer containing transfered data
            wLen: Number of bytes to send
            dwTimeout: Timeout [ms]

        Returns:
            - EC_E_NOERROR if successful
            - EC_E_SLAVE_NOT_PRESENT if slave not present
            - EC_E_BUSY another transfer request is already pending
            - EC_E_NOTFOUND if the slave with the given address does not exist
            - EC_E_NOTREADY if the working counter was not set when sending the command (slave may not be connected or did not respond)
            - EC_E_TIMEOUT if the slave did not respond to the command
            - EC_E_BUSY if the master or the corresponding slave is currently changing its operational state
            - EC_E_INVALIDPARM if the command is not supported or the timeout value is set to EC_NOWAIT
            - EC_E_INVALIDSIZE if the size of the complete command does not fit into a single Ethernet frame. The maximum amount of data to transfer must not exceed 1486 bytes
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_wRegisterOffset = CEcWrapperTypes.Conv(wRegisterOffset, "ushort")
        gen_pbyData = CEcWrapperTypes.Conv(pbyData, "byte[]")
        gen_wLen = CEcWrapperTypes.Conv(wLen, "ushort")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwWriteSlaveRegister(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, gen_wRegisterOffset, gen_pbyData, gen_wLen, gen_dwTimeout))
        CEcWrapperTypes.Conv_Array(gen_pbyData, pbyData)
        return gen_dwRetVal

    def WriteSlaveRegisterReq(self, dwClientId, dwTferId, bFixedAddressing, wSlaveAddress, wRegisterOffset, pbyData, wLen): # ret: ECError
        """
        Requests a data write transfer into the ESC memory of a specified slave and returns immediately.
        
        A notification EC_NOTIFY_SLAVE_REGISTER_TRANSFER is given on completion.
        This function may be called from within the JobTask's context.
        
        \warning Changing contents of ESC registers may lead to unpredictable behavior of the slaves and/or the master.

        Args:
            dwClientId: Client ID returned by RegisterClient (0 if all registered clients shall be notified).
            dwTferId: Transfer ID. The application can set this ID to identify the transfer. It will be passed back to the application within EC_T_SLAVEREGISTER_TRANSFER_NTFY_DESC
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            wRegisterOffset: Register offset. I.e. use 0x0120 to write to the AL Control register
            pbyData: Buffer containing transfered data
            wLen: Number of bytes to send

        Returns:
            - EC_E_NOERROR if successful
            - EC_E_SLAVE_NOT_PRESENT if slave not present
            - EC_E_NOTFOUND if the slave with the given address does not exist
            - EC_E_INVALIDPARM if the command is not supported or the timeout value is set to EC_NOWAIT
            - EC_E_INVALIDSIZE if the size of the complete command does not fit into a single Ethernet frame. The maximum amount of data to transfer must not exceed 1486 bytes
        """
        gen_dwClientId = CEcWrapperTypes.Conv(dwClientId, "uint")
        gen_dwTferId = CEcWrapperTypes.Conv(dwTferId, "uint")
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_wRegisterOffset = CEcWrapperTypes.Conv(wRegisterOffset, "ushort")
        gen_pbyData = CEcWrapperTypes.Conv(pbyData, "byte[]")
        gen_wLen = CEcWrapperTypes.Conv(wLen, "ushort")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwWriteSlaveRegisterReq(self.m_dwMasterInstanceId, gen_dwClientId, gen_dwTferId, gen_bFixedAddressing, gen_wSlaveAddress, gen_wRegisterOffset, gen_pbyData, gen_wLen))
        CEcWrapperTypes.Conv_Array(gen_pbyData, pbyData)
        return gen_dwRetVal

    def QueueRawCmd(self, wInvokeId, byCmd, dwMemoryAddress, pbyData, wLen): # ret: ECError
        """
        Transfers a raw EtherCAT command to one or multiple slaves.
        
        All registered clients will be notified.
        This function may not be called from within the JobTask's context.

        Args:
            wInvokeId: Invoke ID to reassign the results to the sent CMD
            byCmd: EtherCAT command
            dwMemoryAddress: Slave memory address, depending on the command to be sent this is either a physical or logical address.
            pbyData: [in, out] Buffer containing or receiving transfered data In case a read-only command is queued (e.g. APRD) this pointer should be set to a value of EC_NULL
            wLen: Number of bytes to transfer.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_wInvokeId = CEcWrapperTypes.Conv(wInvokeId, "ushort")
        gen_byCmd = CEcWrapperTypes.Conv(byCmd, "byte")
        gen_dwMemoryAddress = CEcWrapperTypes.Conv(dwMemoryAddress, "uint")
        gen_pbyData = CEcWrapperTypes.Conv(pbyData, "byte[]")
        gen_wLen = CEcWrapperTypes.Conv(wLen, "ushort")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwQueueRawCmd(self.m_dwMasterInstanceId, gen_wInvokeId, gen_byCmd, gen_dwMemoryAddress, gen_pbyData, gen_wLen))
        CEcWrapperTypes.Conv_Array(gen_pbyData, pbyData)
        return gen_dwRetVal

    def ClntQueueRawCmd(self, dwClntId, wInvokeId, byCmd, dwMemoryAddress, pbyData, wLen): # ret: ECError
        """
        Transfers a raw EtherCAT command to one or multiple slaves.
        
        Using this function it is possible to exchange data between the master and the slaves. When the response to the queued frame is received, the notification EC_NOTIFY_RAWCMD_DONE is given for the appropriate client.
        This function queues a single EtherCAT command. Queued raw commands will be sent after sending cyclic process data values.
        If a timeout occurs the corresponding frame will be sent again, the timeout value and retry counter can be set using the master configuration parameters EC_T_INIT_MASTER_PARMS.dwEcatCmdTimeout and EC_T_INIT_MASTER_PARMS.dwEcatCmdMaxRetries.\n
        Using auto increment addressing (APRD, APWR, APRW) may lead to unexpected results in case the selected slave does not increment the working counter. In such cases the EtherCAT command would be handled by the slave directly behind the selected one.
        This function may not be called from within the JobTask's context.

        Args:
            dwClntId: Client ID to be notified (0 if all registered clients shall be notified).
            wInvokeId: Invoke ID to reassign the results to the sent CMD
            byCmd: EtherCAT command
            dwMemoryAddress: Slave memory address, depending on the command to be sent this is either a physical or logical address
            pbyData: [in, out] Buffer containing or receiving transfered data. In case a read-only command is queued (e.g. APRD) this pointer should be set to a value of EC_NULL.
            wLen: Number of bytes to transfer.

        Returns:
            - EC_E_NOERROR if successful
            - EC_E_NOTFOUND if the slave with ID dwSlaveId does not exist
            - EC_E_BUSY if the master or the corresponding slave is currently changing its operational state
            - EC_E_INVALIDPARM  if the command is not supported
            - EC_E_INVALIDSIZE if the size of the complete command does not fit into a single Ethernet frame. The maximum amount of data to transfer must not exceed 1486 bytes
        """
        gen_dwClntId = CEcWrapperTypes.Conv(dwClntId, "uint")
        gen_wInvokeId = CEcWrapperTypes.Conv(wInvokeId, "ushort")
        gen_byCmd = CEcWrapperTypes.Conv(byCmd, "byte")
        gen_dwMemoryAddress = CEcWrapperTypes.Conv(dwMemoryAddress, "uint")
        gen_pbyData = CEcWrapperTypes.Conv(pbyData, "byte[]")
        gen_wLen = CEcWrapperTypes.Conv(wLen, "ushort")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwClntQueueRawCmd(self.m_dwMasterInstanceId, gen_dwClntId, gen_wInvokeId, gen_byCmd, gen_dwMemoryAddress, gen_pbyData, gen_wLen))
        CEcWrapperTypes.Conv_Array(gen_pbyData, pbyData)
        return gen_dwRetVal

    def GetNumConfiguredSlaves(self): # ret: uint
        """
        Returns number of slaves which are configured in the ENI.

        Args:

        Returns:
            Number of slaves
        """
        gen_dwRetVal = CEcWrapperTypes.ConvRes(CEcWrapper.Get().ecwGetNumConfiguredSlaves(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def MbxTferAbort(self, pMbxTfer): # ret: ECError
        """
        Abort a running mailbox transfer.
        
        This function may not be called from within the JobTask's context.

        Args:
            pMbxTfer: Mailbox transfer object created with emMbxTferCreate

        Returns:
            EC_E_NOERROR if successful
        """
        gen_pMbxTfer = CEcWrapperTypes.Conv(pMbxTfer, "IntPtr")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwMbxTferAbort(self.m_dwMasterInstanceId, gen_pMbxTfer))
        return gen_dwRetVal

    def MbxTferDelete(self, pMbxTfer): # ret: void
        """
        Deletes a mailbox transfer object.
        
        A transfer object may only be deleted if it is in the Idle state.

        Args:
            pMbxTfer: Mailbox transfer object created with emMbxTferCreate
        """
        gen_pMbxTfer = CEcWrapperTypes.Conv(pMbxTfer, "IntPtr")
        CEcWrapper.Get().ecwMbxTferDelete(self.m_dwMasterInstanceId, gen_pMbxTfer)
        return

    def ClntSendRawMbx(self, dwClntId, pbyMbxCmd, dwMbxCmdLen, dwTimeout): # ret: ECError
        """
        Send a raw mailbox command

        Args:
            dwClntId: Client ID
            pbyMbxCmd: Buffer containing the raw mailbox command starting with mailbox header
            dwMbxCmdLen: Length of pbyMbxCmd buffer
            dwTimeout: Timeout [ms]

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwClntId = CEcWrapperTypes.Conv(dwClntId, "uint")
        gen_pbyMbxCmd = CEcWrapperTypes.Conv(pbyMbxCmd, "byte[]")
        gen_dwMbxCmdLen = CEcWrapperTypes.Conv(dwMbxCmdLen, "uint")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwClntSendRawMbx(self.m_dwMasterInstanceId, gen_dwClntId, gen_pbyMbxCmd, gen_dwMbxCmdLen, gen_dwTimeout))
        CEcWrapperTypes.Conv_Array(gen_pbyMbxCmd, pbyMbxCmd)
        return gen_dwRetVal

    def CoeSdoDownloadReq(self, pMbxTfer, dwSlaveId, wObIndex, byObSubIndex, dwTimeout, dwFlags): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def CoeSdoDownload(self, dwSlaveId, wObIndex, byObSubIndex, pbyData, dwDataLen, dwTimeout, dwFlags): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def CoeSdoUploadReq(self, pMbxTfer, dwSlaveId, wObIndex, byObSubIndex, dwTimeout, dwFlags): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def CoeSdoUpload(self, dwSlaveId, wObIndex, byObSubIndex, pbyData, dwDataLen, out_pdwOutDataLen, dwTimeout, dwFlags): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def CoeGetODList(self, pMbxTfer, dwSlaveId, eListType, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def CoeGetObjectDesc(self, pMbxTfer, dwSlaveId, wObIndex, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def CoeGetEntryDesc(self, pMbxTfer, dwSlaveId, wObIndex, byObSubIndex, byValueInfo, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def CoeRxPdoTfer(self, pMbxTfer, dwSlaveId, dwNumber, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def CoeProfileGetChannelInfo(self, bStationAddress, wSlaveAddress, dwChannel, pInfo): # ret: ECError
        """
        Return information about a configured CoE profile channel from the ENI file

        Args:
            bStationAddress: 
            wSlaveAddress: Slave address according bFixedAddressing
            dwChannel: Channel
            pInfo: Channel info

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bStationAddress = CEcWrapperTypes.Conv(bStationAddress, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_dwChannel = CEcWrapperTypes.Conv(dwChannel, "uint")
        gen_pInfo = CEcWrapperTypes.Conv(pInfo, "EC_T_PROFILE_CHANNEL_INFO")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwCoeProfileGetChannelInfo(self.m_dwMasterInstanceId, gen_bStationAddress, gen_wSlaveAddress, gen_dwChannel, gen_pInfo))
        return gen_dwRetVal

    def FoeFileUpload(self, dwSlaveId, achFileName, dwFileNameLen, pbyData, dwDataLen, out_pdwOutDataLen, dwPassword, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def FoeFileDownload(self, dwSlaveId, achFileName, dwFileNameLen, pbyData, dwDataLen, dwPassword, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def FoeUploadReq(self, pMbxTfer, dwSlaveId, achFileName, dwFileNameLen, dwPassword, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def FoeSegmentedUploadReq(self, pMbxTfer, dwSlaveId, szFileName, dwFileNameLen, dwFileSize, dwPassword, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def FoeDownloadReq(self, pMbxTfer, dwSlaveId, achFileName, dwFileNameLen, dwPassword, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def FoeSegmentedDownloadReq(self, pMbxTfer, dwSlaveId, szFileName, dwFileNameLen, dwFileSize, dwPassword, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def SoeWrite(self, dwSlaveId, byDriveNo, pbyElementFlags, wIDN, pbyData, dwDataLen, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def SoeRead(self, dwSlaveId, byDriveNo, pbyElementFlags, wIDN, pbyData, dwDataLen, out_pdwOutDataLen, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def SoeAbortProcCmd(self, dwSlaveId, byDriveNo, pbyElementFlags, wIDN, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def SoeWriteReq(self, pMbxTfer, dwSlaveId, byDriveNo, pbyElementFlags, wIDN, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def SoeReadReq(self, pMbxTfer, dwSlaveId, byDriveNo, pbyElementFlags, wIDN, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def AoeGetSlaveNetId(self, dwSlaveId, out_poAoeNetId): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def AoeRead(self, dwSlaveId, poTargetNetId, wTargetPort, dwIndexGroup, dwIndexOffset, dwDataLen, pbyData, out_pdwDataOutLen, out_pdwErrorCode, out_pdwCmdResult, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def AoeReadReq(self, pMbxTfer, dwSlaveId, poTargetNetId, wTargetPort, dwIndexGroup, dwIndexOffset, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def AoeWrite(self, dwSlaveId, poTargetNetId, wTargetPort, dwIndexGroup, dwIndexOffset, dwDataLen, pbyData, out_pdwErrorCode, out_pdwCmdResult, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def AoeWriteReq(self, pMbxTfer, dwSlaveId, poTargetNetId, wTargetPort, dwIndexGroup, dwIndexOffset, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def AoeReadWrite(self, dwSlaveId, poTargetNetId, wTargetPort, dwIndexGroup, dwIndexOffset, dwReadDataLen, dwWriteDataLen, pbyData, out_pdwDataOutLen, out_pdwErrorCode, out_pdwCmdResult, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def AoeWriteControl(self, dwSlaveId, poTargetNetId, wTargetPort, wAoEState, wDeviceState, dwDataLen, pbyData, pdwErrorCode, pdwCmdResult, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def VoeRead(self, dwSlaveId, pbyData, dwDataLen, out_pdwOutDataLen, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def VoeWrite(self, dwSlaveId, pbyData, dwDataLen, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def VoeWriteReq(self, pMbxTfer, dwSlaveId, dwTimeout): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def GetProcessData(self, bOutputData, dwOffset, pbyData, dwLength, dwTimeout): # ret: ECError
        """
        Retrieve Process data synchronized.
        
        If process data are required outside the cyclic master job task (which is calling ecatExecJob), direct access to the process data is not recommended as data consistency cannot be guaranteed.
        A call to this function will send a data read request to the master stack and then check every millisecond whether new data are provided.
        The master stack will provide new data after calling ecatExecJob(eUsrJob_ MasterTimer) within the job task. This function is usually only called remotely (using the Remote API).
        
        \note This function may not be called from within the JobTask's context.

        Args:
            bOutputData: EC_TRUE: read output data, EC_FALSE: read input data.
            dwOffset: Byte offset in Process data to read from.
            pbyData: Buffer receiving transfered data
            dwLength: 
            dwTimeout: Timeout [ms]

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bOutputData = CEcWrapperTypes.Conv(bOutputData, "bool")
        gen_dwOffset = CEcWrapperTypes.Conv(dwOffset, "uint")
        gen_pbyData = CEcWrapperTypes.Conv(pbyData, "byte[]")
        gen_dwLength = CEcWrapperTypes.Conv(dwLength, "uint")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetProcessData(self.m_dwMasterInstanceId, gen_bOutputData, gen_dwOffset, gen_pbyData, gen_dwLength, gen_dwTimeout))
        CEcWrapperTypes.Conv_Array(gen_pbyData, pbyData)
        return gen_dwRetVal

    def SetProcessData(self, bOutputData, dwOffset, pbyData, dwLength, dwTimeout): # ret: ECError
        """
        Write Process data synchronized.
        
        If process data shall be set outside the cyclic master job task (which is calling ecatExecJob), direct access to the process data is not recommended as data consistency cannot be guaranteed.
        A call to this function will send a data write request to the master stack and then check every millisecond whether new data is written.
        The master stack will copy the data after calling ecatExecJob(eUsrJob_ MasterTimer) within the job task. This function is usually only called remotely (using the Remote API).
        
        \note This function may not be called from within the JobTask's context.

        Args:
            bOutputData: EC_TRUE: write output data, EC_FALSE: write input data.
            dwOffset: Byte offset in Process data to write to.
            pbyData: Buffer containing transfered data
            dwLength: 
            dwTimeout: Timeout [ms]

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bOutputData = CEcWrapperTypes.Conv(bOutputData, "bool")
        gen_dwOffset = CEcWrapperTypes.Conv(dwOffset, "uint")
        gen_pbyData = CEcWrapperTypes.Conv(pbyData, "byte[]")
        gen_dwLength = CEcWrapperTypes.Conv(dwLength, "uint")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwSetProcessData(self.m_dwMasterInstanceId, gen_bOutputData, gen_dwOffset, gen_pbyData, gen_dwLength, gen_dwTimeout))
        CEcWrapperTypes.Conv_Array(gen_pbyData, pbyData)
        return gen_dwRetVal

    def SetProcessDataBits(self, bOutputData, dwBitOffsetPd, pbyDataSrc, dwBitLengthSrc, dwTimeout): # ret: ECError
        """
        Writes a specific number of bits from a given buffer to the process image with a bit offset (synchronized).
        
        This function may not be called from within the JobTask's context.

        Args:
            bOutputData: EC_TRUE: write output data, EC_FALSE: write input data.
            dwBitOffsetPd: Bit offset in Process data image.
            pbyDataSrc: 
            dwBitLengthSrc: 
            dwTimeout: Timeout [ms] The timeout value must not be set to EC_NOWAIT.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bOutputData = CEcWrapperTypes.Conv(bOutputData, "bool")
        gen_dwBitOffsetPd = CEcWrapperTypes.Conv(dwBitOffsetPd, "uint")
        gen_pbyDataSrc = CEcWrapperTypes.Conv(pbyDataSrc, "byte[]")
        gen_dwBitLengthSrc = CEcWrapperTypes.Conv(dwBitLengthSrc, "uint")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwSetProcessDataBits(self.m_dwMasterInstanceId, gen_bOutputData, gen_dwBitOffsetPd, gen_pbyDataSrc, gen_dwBitLengthSrc, gen_dwTimeout))
        CEcWrapperTypes.Conv_Array(gen_pbyDataSrc, pbyDataSrc)
        return gen_dwRetVal

    def GetProcessDataBits(self, bOutputData, dwBitOffsetPd, pbyDataDst, dwBitLengthDst, dwTimeout): # ret: ECError
        """
        Reads a specific number of bits from the process image to the given buffer with a bit offset (synchronized).
        
        This function may not be called from within the JobTask's context.

        Args:
            bOutputData: EC_TRUE: read output data, EC_FALSE: write input data.
            dwBitOffsetPd: Bit offset in Process data image.
            pbyDataDst: 
            dwBitLengthDst: 
            dwTimeout: Timeout [ms] The timeout value must not be set to EC_NOWAIT.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bOutputData = CEcWrapperTypes.Conv(bOutputData, "bool")
        gen_dwBitOffsetPd = CEcWrapperTypes.Conv(dwBitOffsetPd, "uint")
        gen_pbyDataDst = CEcWrapperTypes.Conv(pbyDataDst, "byte[]")
        gen_dwBitLengthDst = CEcWrapperTypes.Conv(dwBitLengthDst, "uint")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetProcessDataBits(self.m_dwMasterInstanceId, gen_bOutputData, gen_dwBitOffsetPd, gen_pbyDataDst, gen_dwBitLengthDst, gen_dwTimeout))
        CEcWrapperTypes.Conv_Array(gen_pbyDataDst, pbyDataDst)
        return gen_dwRetVal

    def ForceProcessDataBits(self, dwClientId, bOutputData, dwBitOffsetPd, wBitLength, pbyData, dwTimeout): # ret: ECError
        """
        Force a specific number of bits from a given buffer to the process image with a bit offset.
        
        All output data set by this API are overwriting the values set by the application. All input data set by this API are overwriting the values read from the slaves.
        Forcing will be terminated by calling the corresponding functions.
        This function may not be called from within the JobTask's context.

        Args:
            dwClientId: Client ID returned by RegisterClient (0 if all registered clients shall be notified).
            bOutputData: EC_TRUE: write output data, EC_FALSE: write input data.
            dwBitOffsetPd: Bit offset in Process data image
            wBitLength: 
            pbyData: Buffer containing transfered data
            dwTimeout: Timeout [ms] The timeout value must not be set to EC_NOWAIT.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwClientId = CEcWrapperTypes.Conv(dwClientId, "uint")
        gen_bOutputData = CEcWrapperTypes.Conv(bOutputData, "bool")
        gen_dwBitOffsetPd = CEcWrapperTypes.Conv(dwBitOffsetPd, "uint")
        gen_wBitLength = CEcWrapperTypes.Conv(wBitLength, "ushort")
        gen_pbyData = CEcWrapperTypes.Conv(pbyData, "byte[]")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwForceProcessDataBits(self.m_dwMasterInstanceId, gen_dwClientId, gen_bOutputData, gen_dwBitOffsetPd, gen_wBitLength, gen_pbyData, gen_dwTimeout))
        CEcWrapperTypes.Conv_Array(gen_pbyData, pbyData)
        return gen_dwRetVal

    def ReleaseProcessDataBits(self, dwClientId, bOutputData, dwBitOffsetPd, wBitLength, dwTimeout): # ret: ECError
        """
        Release previously forced process data.
        
        - Forced output: Value set by application become valid again. Because forced process data bits are written directly into the process output image, the application has to update the process image with the required value, otherwise the forced value is still valid.
        - Forced input: Value read from the slaves become valid again.
        
        This function may not be called from within the JobTask's context.

        Args:
            dwClientId: Client ID returned by RegisterClient (0 if all registered clients shall be notified).
            bOutputData: EC_TRUE: write output data, EC_FALSE: write input data
            dwBitOffsetPd: Bit offset in Process data image
            wBitLength: Number of bits that shall be written to the process image.
            dwTimeout: Timeout [ms] The timeout value must not be set to EC_NOWAIT.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwClientId = CEcWrapperTypes.Conv(dwClientId, "uint")
        gen_bOutputData = CEcWrapperTypes.Conv(bOutputData, "bool")
        gen_dwBitOffsetPd = CEcWrapperTypes.Conv(dwBitOffsetPd, "uint")
        gen_wBitLength = CEcWrapperTypes.Conv(wBitLength, "ushort")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwReleaseProcessDataBits(self.m_dwMasterInstanceId, gen_dwClientId, gen_bOutputData, gen_dwBitOffsetPd, gen_wBitLength, gen_dwTimeout))
        return gen_dwRetVal

    def ReleaseAllProcessDataBits(self, dwClientId, dwTimeout): # ret: ECError
        """
        Release all previously forced process data for a dedicated client.
        
        - Forced output: Value set by application become valid again. Because forced process data bits are written directly into the process output image, the application has to update the process image with the required value, otherwise the forced value is still valid.
        - Forced input: Value read from the slaves become valid again.
        
        This function may not be called from within the JobTask's context.

        Args:
            dwClientId: Client ID returned by RegisterClient (0 if all registered clients shall be notified).
            dwTimeout: Timeout [ms] The timeout value must not be set to EC_NOWAIT.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwClientId = CEcWrapperTypes.Conv(dwClientId, "uint")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwReleaseAllProcessDataBits(self.m_dwMasterInstanceId, gen_dwClientId, gen_dwTimeout))
        return gen_dwRetVal

    def GetNumConnectedSlaves(self): # ret: uint
        """
        Get amount of currently connected slaves.

        Args:

        Returns:
            Number of connected slaves
        """
        gen_dwRetVal = CEcWrapperTypes.ConvRes(CEcWrapper.Get().ecwGetNumConnectedSlaves(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def GetNumConnectedSlavesMain(self): # ret: uint
        """
        Get the amount of currently connected Slaves to main interface.

        Args:

        Returns:
            Number of connected slaves at main interface
        """
        gen_dwRetVal = CEcWrapperTypes.ConvRes(CEcWrapper.Get().ecwGetNumConnectedSlavesMain(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def GetNumConnectedSlavesRed(self): # ret: uint
        """
        Get the amount of currently connected Slaves to redundancy interface.

        Args:

        Returns:
            Number of connected slaves at redundancy interface
        """
        gen_dwRetVal = CEcWrapperTypes.ConvRes(CEcWrapper.Get().ecwGetNumConnectedSlavesRed(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def ReadSlaveEEPRomReq(self, dwClientId, dwTferId, bFixedAddressing, wSlaveAddress, wEEPRomStartOffset, pwReadData, dwReadLen, pdwNumOutData, dwTimeout): # ret: ECError
        """
        Requests a EEPROM data read operation from slave and returns immediately.
        
        A EC_NOTIFY_EEPROM_OPERATION is given on completion or timeout.
        This function may be called from within the JobTask's context.

        Args:
            dwClientId: Client ID returned by RegisterClient (0 if all registered clients shall be notified).
            dwTferId: Transfer ID. The application can set this ID to identify the transfer. It will be passed back to the application within EC_T_EEPROM_OPERATION_NTFY_DESC
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            wEEPRomStartOffset: Word address to start EEPROM read from
            pwReadData: Pointer to EC_T_WORD array to carry the read data, must be valid until the operation complete
            dwReadLen: Size of the EC_T_WORD array provided at pwReadData (in EC_T_WORDs)
            pdwNumOutData: Pointer to EC_T_DWORD carrying actually read data (in EC_T_WORDs) after completion
            dwTimeout: Timeout [ms]

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwClientId = CEcWrapperTypes.Conv(dwClientId, "uint")
        gen_dwTferId = CEcWrapperTypes.Conv(dwTferId, "uint")
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_wEEPRomStartOffset = CEcWrapperTypes.Conv(wEEPRomStartOffset, "ushort")
        gen_pwReadData = CEcWrapperTypes.Conv(pwReadData, "ushort")
        gen_dwReadLen = CEcWrapperTypes.Conv(dwReadLen, "uint")
        gen_pdwNumOutData = CEcWrapperTypes.Conv(pdwNumOutData, "uint")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwReadSlaveEEPRomReq(self.m_dwMasterInstanceId, gen_dwClientId, gen_dwTferId, gen_bFixedAddressing, gen_wSlaveAddress, gen_wEEPRomStartOffset, ctypes.pointer(gen_pwReadData), gen_dwReadLen, ctypes.pointer(gen_pdwNumOutData), gen_dwTimeout))
        return gen_dwRetVal

    def WriteSlaveEEPRomReq(self, dwClientId, dwTferId, bFixedAddressing, wSlaveAddress, wEEPRomStartOffset, pwWriteData, dwWriteLen, dwTimeout): # ret: ECError
        """
        Requests a EEPROM data write operation from slave and returns immediately.
        
        The EEPROM's CRC is updated automatically. A reset of the slave controller is needed to reload the alias address in register 0x12.
        A EC_NOTIFY_EEPROM_OPERATION is given on completion or timeout. This function may be called from within the JobTask's context.

        Args:
            dwClientId: Client ID returned by RegisterClient (0 if all registered clients shall be notified).
            dwTferId: Transfer ID. The application can set this ID to identify the transfer. It will be passed back to the application within EC_T_EEPROM_OPERATION_NTFY_DESC
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            wEEPRomStartOffset: Word address to start EEPROM Write from.
            pwWriteData: Pointer to WORD array carrying the write data, must be valid until operation complete
            dwWriteLen: Size of Write Data WORD array (in WORDS)
            dwTimeout: Timeout [ms]

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwClientId = CEcWrapperTypes.Conv(dwClientId, "uint")
        gen_dwTferId = CEcWrapperTypes.Conv(dwTferId, "uint")
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_wEEPRomStartOffset = CEcWrapperTypes.Conv(wEEPRomStartOffset, "ushort")
        gen_pwWriteData = CEcWrapperTypes.Conv(pwWriteData, "ushort")
        gen_dwWriteLen = CEcWrapperTypes.Conv(dwWriteLen, "uint")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwWriteSlaveEEPRomReq(self.m_dwMasterInstanceId, gen_dwClientId, gen_dwTferId, gen_bFixedAddressing, gen_wSlaveAddress, gen_wEEPRomStartOffset, ctypes.pointer(gen_pwWriteData), gen_dwWriteLen, gen_dwTimeout))
        return gen_dwRetVal

    def ReloadSlaveEEPRom(self, bFixedAddressing, wSlaveAddress, dwTimeout): # ret: ECError
        """
        Causes a slave to reload its EEPROM values to ESC registers.
        
        Alias address at 0x12 is not reloaded through this command, this is prevented by the slave hardware. The slave controller must be reset to reload the alias address.
        This function may not be called from within the JobTask's context.

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            dwTimeout: Timeout [ms] The function will block at most for this time. The timeout value must not be set to EC_NOWAIT

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwReloadSlaveEEPRom(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, gen_dwTimeout))
        return gen_dwRetVal

    def ReloadSlaveEEPRomReq(self, dwClientId, dwTferId, bFixedAddressing, wSlaveAddress, dwTimeout): # ret: ECError
        """
        Request a slave to reload its EEPROM values to ESC registers, and returns immediately.
        
        Alias address at 0x12 is not reloaded through this command, this is prevented by the slave hardware. The slave controller must be reset to reload the alias address.
        A EC_NOTIFY_EEPROM_OPERATION is given on completion or timeout. This function may be called from within the JobTask's context.

        Args:
            dwClientId: Client ID returned by RegisterClient (0 if all registered clients shall be notified).
            dwTferId: Transfer ID. The application can set this ID to identify the transfer. It will be passed back to the application within EC_T_EEPROM_OPERATION_NTFY_DESC
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            dwTimeout: Timeout [ms]

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwClientId = CEcWrapperTypes.Conv(dwClientId, "uint")
        gen_dwTferId = CEcWrapperTypes.Conv(dwTferId, "uint")
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwReloadSlaveEEPRomReq(self.m_dwMasterInstanceId, gen_dwClientId, gen_dwTferId, gen_bFixedAddressing, gen_wSlaveAddress, gen_dwTimeout))
        return gen_dwRetVal

    def ResetSlaveController(self, bFixedAddressing, wSlaveAddress, dwTimeout): # ret: ECError
        """
        Reset EtherCAT slave controller (ESC)
        
        A special sequence of three independent and consecutive frames/commands is sent to the slave (reset register ECAT 0x0040 or PDI 0x0041), after which the slave resets.
        If that fails, the reset sequence is repeated until it succeeds or the timeout expires.
        The ESC must support resetting and the slave state should be INIT when calling this function.
        The number of acyclic frames per cycle EC_T_INIT_MASTER_PARMS.dwMaxAcycFramesPerCycle must be at least 3, otherwise an error is returned.
        This function may not be called from within the JobTask's context.

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            dwTimeout: Timeout [ms] The function will block at most for this time. The timeout value must not be set to EC_NOWAIT

        Returns:
            - EC_E_NOERROR or error code
            - EC_E_NOTSUPPORTED if EC_T_INIT_MASTER_PARMS.dwMaxAcycFramesPerCycle is less than 3
            - EC_E_SLAVE_NOT_PRESENT if slave not present
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwResetSlaveController(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, gen_dwTimeout))
        return gen_dwRetVal

    def AssignSlaveEEPRom(self, bFixedAddressing, wSlaveAddress, bSlavePDIAccessEnable, bForceAssign, dwTimeout): # ret: ECError
        """
        Set EEPROM Assignment to PDI or EtherCAT Master.
        
        This function may not be called from within the JobTask's context.

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            bSlavePDIAccessEnable: EC_TRUE: EEPROM assigned to slave PDI application,
            bForceAssign: Force Assignment of EEPROM (only for ECat Master Assignment)
            dwTimeout: Timeout [ms] The function will block at most for this time. The timeout value must not be set to EC_NOWAIT.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_bSlavePDIAccessEnable = CEcWrapperTypes.Conv(bSlavePDIAccessEnable, "bool")
        gen_bForceAssign = CEcWrapperTypes.Conv(bForceAssign, "bool")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwAssignSlaveEEPRom(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, gen_bSlavePDIAccessEnable, gen_bForceAssign, gen_dwTimeout))
        return gen_dwRetVal

    def AssignSlaveEEPRomReq(self, dwClientId, dwTferId, bFixedAddressing, wSlaveAddress, bSlavePDIAccessEnable, bForceAssign, dwTimeout): # ret: ECError
        """
        Requests EEPROM Assignment to PDI or EtherCAT Master operation and return immediately
        
        EC_NOTIFY_EEPROM_OPERATION is given on completion or timeout.
        This function may be called from within the JobTask's context.

        Args:
            dwClientId: Client ID returned by RegisterClient (0 if all registered clients shall be notified).
            dwTferId: Transfer ID. The application can set this ID to identify the transfer. It will be passed back to the application within EC_T_EEPROM_OPERATION_NTFY_DESC
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            bSlavePDIAccessEnable: EC_TRUE: EEPROM assigned to slave PDI application, EC_FALSE: EEPROM assigned to EC-Master
            bForceAssign: Force Assignment of EEPROM (only for ECat Master Assignment)
            dwTimeout: Timeout [ms]

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwClientId = CEcWrapperTypes.Conv(dwClientId, "uint")
        gen_dwTferId = CEcWrapperTypes.Conv(dwTferId, "uint")
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_bSlavePDIAccessEnable = CEcWrapperTypes.Conv(bSlavePDIAccessEnable, "bool")
        gen_bForceAssign = CEcWrapperTypes.Conv(bForceAssign, "bool")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwAssignSlaveEEPRomReq(self.m_dwMasterInstanceId, gen_dwClientId, gen_dwTferId, gen_bFixedAddressing, gen_wSlaveAddress, gen_bSlavePDIAccessEnable, gen_bForceAssign, gen_dwTimeout))
        return gen_dwRetVal

    def ActiveSlaveEEPRom(self, bFixedAddressing, wSlaveAddress, pbSlavePDIAccessActive, dwTimeout): # ret: ECError
        """
        Check whether EEPROM is marked access active by Slave PDI application.
        
        This function may not be called from within the JobTask's context.

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            pbSlavePDIAccessActive: Pointer to Boolean value: EC_TRUE: EEPROM active by PDI application, EC_FALSE: EEPROM not active
            dwTimeout: Timeout [ms] The function will block at most for this time. The timeout value must not be set to EC_NOWAIT

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_pbSlavePDIAccessActive = CEcWrapperTypes.Conv(pbSlavePDIAccessActive, "bool")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwActiveSlaveEEPRom(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, ctypes.pointer(gen_pbSlavePDIAccessActive), gen_dwTimeout))
        return gen_dwRetVal

    def ActiveSlaveEEPRomReq(self, dwClientId, dwTferId, bFixedAddressing, wSlaveAddress, pbSlavePDIAccessActive, dwTimeout): # ret: ECError
        """
        Requests EEPROM is marked access active by Slave PDI application check and returns immediately.
        
        A EC_NOTIFY_EEPROM_OPERATION is given on completion or timeout.
        This function may be called from within the JobTask's context.

        Args:
            dwClientId: Client ID returned by RegisterClient (0 if all registered clients shall be notified).
            dwTferId: Transfer ID. The application can set this ID to identify the transfer. It will be passed back to the application within EC_T_EEPROM_OPERATION_NTFY_DESC
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            pbSlavePDIAccessActive: Pointer to Boolean value: EC_TRUE: EEPROM active by PDI application, EC_FALSE: EEPROM not active. Must be valid until operation complete
            dwTimeout: Timeout [ms] The function will block at most for this time. The timeout value must not be set to EC_NOWAIT.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwClientId = CEcWrapperTypes.Conv(dwClientId, "uint")
        gen_dwTferId = CEcWrapperTypes.Conv(dwTferId, "uint")
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_pbSlavePDIAccessActive = CEcWrapperTypes.Conv(pbSlavePDIAccessActive, "bool")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwActiveSlaveEEPRomReq(self.m_dwMasterInstanceId, gen_dwClientId, gen_dwTferId, gen_bFixedAddressing, gen_wSlaveAddress, ctypes.pointer(gen_pbSlavePDIAccessActive), gen_dwTimeout))
        return gen_dwRetVal

    def HCAcceptTopoChange(self): # ret: ECError
        """
        Accept last detected topology change
        
        If Hot connect is configured in manual mode by EC_IOCTL_HC_SETMODE, the master will generate the notifications EC_NOTIFY_HC_PROBEALLGROUPS or EC_NOTIFY_HC_DETECTADDGROUPS after a topology change was detected.
        This function will set all new detected slaves to the current master state.

        Args:

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwHCAcceptTopoChange(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def HCGetNumGroupMembers(self, dwGroupIndex): # ret: ECError
        """
        Get number of slaves belonging to a specific HotConnect group.

        Args:
            dwGroupIndex: Index of HotConnect group, 0 is the mandatory group

        Returns:
            Number of slaves belonging to specified HotConnect group
        """
        gen_dwGroupIndex = CEcWrapperTypes.Conv(dwGroupIndex, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwHCGetNumGroupMembers(self.m_dwMasterInstanceId, gen_dwGroupIndex))
        return gen_dwRetVal

    def HCGetSlaveIdsOfGroup(self, dwGroupIndex, adwSlaveId, dwMaxNumSlaveIds): # ret: ECError
        """
        Return the list of ID referencing slaves belonging to a specific HotConnect group.

        Args:
            dwGroupIndex: Index of HotConnect group, 0 is the mandatory group
            adwSlaveId: DWORD array to carry slave ids of specified HotConncet group
            dwMaxNumSlaveIds: size of adwSlaveId array

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwGroupIndex = CEcWrapperTypes.Conv(dwGroupIndex, "uint")
        gen_adwSlaveId = CEcWrapperTypes.Conv(adwSlaveId, "uint")
        gen_dwMaxNumSlaveIds = CEcWrapperTypes.Conv(dwMaxNumSlaveIds, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwHCGetSlaveIdsOfGroup(self.m_dwMasterInstanceId, gen_dwGroupIndex, ctypes.pointer(gen_adwSlaveId), gen_dwMaxNumSlaveIds))
        return gen_dwRetVal

    def SetSlavePortState(self, dwSlaveId, wPort, bClose, bForce, dwTimeout): # ret: ECError
        """
        Open or close slave port
        
        This function can be called to re-open ports closed by a rescue scan.

        Args:
            dwSlaveId: Slave ID
            wPort: Port to open or close. Can be ESC_PORT_A, ESC_PORT_B, ESC_PORT_C, ESC_PORT_D
            bClose: EC_TRUE: close port, EC_FALSE: open port
            bForce: EC_TRUE: port will be closed or open, EC_FALSE: port will be set in AutoClose mode
            dwTimeout: Timeout [ms]

        Returns:
            - EC_E_NOERROR on success
            - EC_E_SLAVE_NOT_PRESENT if slave not present
            - EC_E_NOTFOUND if the slave with ID dwSlaveId does not exist
        """
        gen_dwSlaveId = CEcWrapperTypes.Conv(dwSlaveId, "uint")
        gen_wPort = CEcWrapperTypes.Conv(wPort, "ushort")
        gen_bClose = CEcWrapperTypes.Conv(bClose, "bool")
        gen_bForce = CEcWrapperTypes.Conv(bForce, "bool")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwSetSlavePortState(self.m_dwMasterInstanceId, gen_dwSlaveId, gen_wPort, gen_bClose, gen_bForce, gen_dwTimeout))
        return gen_dwRetVal

    def SetSlavePortStateReq(self, dwClientId, dwTferId, dwSlaveId, wPort, bClose, bForce, dwTimeout): # ret: ECError
        """
        Requests Open or close slave port operation and returns immediately.
        
        A EC_T_PORT_OPERATION_NTFY_DESC is given on completion. This function can be called to re-open ports closed by a rescue scan.

        Args:
            dwClientId: Client ID returned by RegisterClient (0 if all registered clients shall be notified).
            dwTferId: Transfer ID. The application can set this ID to identify the transfer. It will be passed back to the application within EC_T_PORT_OPERATION_NTFY_DESC
            dwSlaveId: Slave ID
            wPort: Port to open or close. Can be ESC_PORT_A, ESC_PORT_B, ESC_PORT_C, ESC_PORT_D
            bClose: EC_TRUE: close port, EC_FALSE: open port
            bForce: EC_TRUE: port will be closed or open, EC_FALSE: port will be set in AutoClose mode
            dwTimeout: Timeout [ms]

        Returns:
            - EC_E_NOERROR on success
            - EC_E_SLAVE_NOT_PRESENT if slave not present
            - EC_E_NOTFOUND if the slave with ID dwSlaveId does not exist
        """
        gen_dwClientId = CEcWrapperTypes.Conv(dwClientId, "uint")
        gen_dwTferId = CEcWrapperTypes.Conv(dwTferId, "uint")
        gen_dwSlaveId = CEcWrapperTypes.Conv(dwSlaveId, "uint")
        gen_wPort = CEcWrapperTypes.Conv(wPort, "ushort")
        gen_bClose = CEcWrapperTypes.Conv(bClose, "bool")
        gen_bForce = CEcWrapperTypes.Conv(bForce, "bool")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwSetSlavePortStateReq(self.m_dwMasterInstanceId, gen_dwClientId, gen_dwTferId, gen_dwSlaveId, gen_wPort, gen_bClose, gen_bForce, gen_dwTimeout))
        return gen_dwRetVal

    def SlaveSerializeMbxTfers(self, dwSlaveId): # ret: ECError
        """
        Serializes all mailbox transfers to the specified slave
        
        The parallel (overlapped) usage of more than one protocol (CoE, EoE, FoE, etc.) will be disabled. By default parallel mailbox transfers are enabled.

        Args:
            dwSlaveId: Slave ID

        Returns:
            - EC_E_NOERROR if successful
            - EC_E_INVALIDPARM if master is not initialized
            - EC_E_NOTFOUND if the slave with given ID does not exist
            - EC_E_NO_MBX_SUPPORT if slave does not support mailbox transfers
        """
        gen_dwSlaveId = CEcWrapperTypes.Conv(dwSlaveId, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwSlaveSerializeMbxTfers(self.m_dwMasterInstanceId, gen_dwSlaveId))
        return gen_dwRetVal

    def SlaveParallelMbxTfers(self, dwSlaveId): # ret: ECError
        """
        Re-enable the parallel mailbox transfers to the specified slave
        
        Allows parallel (overlapped) usage of more than one protocol (CoE, EoE, FoE, etc.).
        By default parallel mailbox transfers are enabled.

        Args:
            dwSlaveId: Slave ID

        Returns:
            - EC_E_NOERROR if successful
            - EC_E_INVALIDPARM if master is not initialized
            - EC_E_NOTFOUND if the slave with given ID does not exist
            - EC_E_NO_MBX_SUPPORT if slave does not support mailbox transfers
        """
        gen_dwSlaveId = CEcWrapperTypes.Conv(dwSlaveId, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwSlaveParallelMbxTfers(self.m_dwMasterInstanceId, gen_dwSlaveId))
        return gen_dwRetVal

    def DcEnable(self): # ret: ECError
        """
        Enable DC Support

        Args:

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDcEnable(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def DcDisable(self): # ret: ECError
        """
        Disable DC Support

        Args:

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDcDisable(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def DcIsEnabled(self, pbDcIsEnabled): # ret: ECError
        """
        Determines if DC is enabled and used.

        Args:
            pbDcIsEnabled: EC_TRUE if DC is enabled

        Returns:
            EC_E_NOERROR or error code
        """
        gen_pbDcIsEnabled = CEcWrapperTypes.Conv(pbDcIsEnabled, "bool")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDcIsEnabled(self.m_dwMasterInstanceId, ctypes.pointer(gen_pbDcIsEnabled)))
        return gen_dwRetVal

    def DcConfigure(self, pDcConfigure): # ret: ECError
        """
        Configure the distributed clocks.
        
        - Set the DC synchronization settling time ([ms]).
        - Set the DC slave limit for the wire or'ed clock deviation value. This value determines whether the slave clocks are synchronized or not.
        - Configure the ARMW burst frames to compensate the static deviations of the clock speeds.

        Args:
            pDcConfigure: Configuration parameter a pointer to a structure of type EC_T_DC_CONFIGURE.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_pDcConfigure = CEcWrapperTypes.Conv(pDcConfigure, "EC_T_DC_CONFIGURE")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDcConfigure(self.m_dwMasterInstanceId, gen_pDcConfigure))
        return gen_dwRetVal

    def DcContDelayCompEnable(self): # ret: ECError
        """
        Enable the continuous propagation delay compensation.
        
        Calling this function generate a propagation delay measurement every 30s. The result of the measurement is used to correct the propagation delay values on the bus.

        Args:

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDcContDelayCompEnable(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def DcContDelayCompDisable(self): # ret: ECError
        """
        Disable the continuous propagation delay compensation.

        Args:

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDcContDelayCompDisable(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def DcmConfigure(self, pDcmConfig, dwInSyncTimeout): # ret: ECError
        """
        Configure DC master synchronization

        Args:
            pDcmConfig: Configuration information, a pointer to a structure of type EC_T_DCM_CONFIG.
            dwInSyncTimeout: Currently not implemented.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_pDcmConfig = CEcWrapperTypes.Conv(pDcmConfig, "EC_T_DCM_CONFIG")
        gen_dwInSyncTimeout = CEcWrapperTypes.Conv(dwInSyncTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDcmConfigure(self.m_dwMasterInstanceId, ctypes.pointer(gen_pDcmConfig), gen_dwInSyncTimeout))
        return gen_dwRetVal

    def DcmGetStatus(self, pdwErrorCode, pnDiffCur, pnDiffAvg, pnDiffMax): # ret: ECError
        """
        Get DC master synchronization controller status.

        Args:
            pdwErrorCode: Pointer to current error code of the DCM controller. Possible values are:
            pnDiffCur: Pointer to current difference between set value and actual value of controller in nanoseconds.
            pnDiffAvg: Pointer to average difference between set value and actual value of controller in nanoseconds
            pnDiffMax: Pointer to maximum difference between set value and actual value of controller in nanoseconds

        Returns:
            - EC_E_NOERROR if status retrival was successful
            - EC_E_NOTSUPPORTED the DC feature is not supported/switched off. EC master stack has to be compiled with DC support see #define INCLUDE_DC_SUPPORT
            - EC_NULL does not appear in normal flow, if EC_NULL is returned we are in a very exceptional case where m_poDcm == NULL (SW error detection)
        """
        gen_pdwErrorCode = CEcWrapperTypes.Conv(pdwErrorCode, "uint")
        gen_pnDiffCur = CEcWrapperTypes.Conv(pnDiffCur, "int")
        gen_pnDiffAvg = CEcWrapperTypes.Conv(pnDiffAvg, "int")
        gen_pnDiffMax = CEcWrapperTypes.Conv(pnDiffMax, "int")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDcmGetStatus(self.m_dwMasterInstanceId, ctypes.pointer(gen_pdwErrorCode), ctypes.pointer(gen_pnDiffCur), ctypes.pointer(gen_pnDiffAvg), ctypes.pointer(gen_pnDiffMax)))
        return gen_dwRetVal

    def DcxGetStatus(self, pdwErrorCode, pnDiffCur, pnDiffAvg, pnDiffMax, pnTimeStampDiff): # ret: ECError
        """
        Get DC master external synchronization controller status.

        Args:
            pdwErrorCode: DCX controller error code
            pnDiffCur: Current difference between set value and actual value of controller in nanoseconds.
            pnDiffAvg: Average difference between set value and actual value of controller in nanoseconds.
            pnDiffMax: Maximum difference between set value and actual value of controller in nanoseconds.
            pnTimeStampDiff: Difference between external and internal timestamps

        Returns:
            EC_E_NOERROR or error code
        """
        gen_pdwErrorCode = CEcWrapperTypes.Conv(pdwErrorCode, "uint")
        gen_pnDiffCur = CEcWrapperTypes.Conv(pnDiffCur, "int")
        gen_pnDiffAvg = CEcWrapperTypes.Conv(pnDiffAvg, "int")
        gen_pnDiffMax = CEcWrapperTypes.Conv(pnDiffMax, "int")
        gen_pnTimeStampDiff = CEcWrapperTypes.Conv(pnTimeStampDiff, "int64")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDcxGetStatus(self.m_dwMasterInstanceId, ctypes.pointer(gen_pdwErrorCode), ctypes.pointer(gen_pnDiffCur), ctypes.pointer(gen_pnDiffAvg), ctypes.pointer(gen_pnDiffMax), ctypes.pointer(gen_pnTimeStampDiff)))
        return gen_dwRetVal

    def DcmResetStatus(self): # ret: ECError
        """
        Reset DC master synchronization controller status, average and maximum difference between set value and actual value

        Args:

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDcmResetStatus(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def DcmGetBusShiftConfigured(self, pbBusShiftConfigured): # ret: ECError
        """
        Determines if DCM Bus Shift is configured/possible in configuration (ENI file)

        Args:
            pbBusShiftConfigured: EC_TRUE if DCM bus shift mode is supported by the current configuration

        Returns:
            EC_E_NOERROR or error code
        """
        gen_pbBusShiftConfigured = CEcWrapperTypes.Conv(pbBusShiftConfigured, "bool")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDcmGetBusShiftConfigured(self.m_dwMasterInstanceId, ctypes.pointer(gen_pbBusShiftConfigured)))
        return gen_dwRetVal

    def DcmShowStatus(self): # ret: ECError
        """
        Show DC master synchronization status as DbgMsg (for development purposes only).

        Args:

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDcmShowStatus(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def DcmGetAdjust(self, pnAdjustPermil): # ret: ECError
        """
        Returns the current adjustment value for the timer.
        
        bCtlOff must be set to EC_TRUE in EC_T_DCM_CONFIG to enable external adjustment.

        Args:
            pnAdjustPermil: Current adjustment value of the timer.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_pnAdjustPermil = CEcWrapperTypes.Conv(pnAdjustPermil, "int")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDcmGetAdjust(self.m_dwMasterInstanceId, ctypes.pointer(gen_pnAdjustPermil)))
        return gen_dwRetVal

    def GetSlaveInfo(self, bFixedAddressing, wSlaveAddress, out_pGetSlaveInfo): # ret: ECError
        """
        Get Slave Info
        
        \deprecated Use emGetCfgSlaveInfo or emGetBusSlaveInfo instead

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            pGetSlaveInfo: Slave information

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        pGetSlaveInfo = DN_EC_T_GET_SLAVE_INFO()
        gen_pGetSlaveInfo = CEcWrapperTypes.Conv(pGetSlaveInfo, "EC_T_GET_SLAVE_INFO")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetSlaveInfo(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, ctypes.pointer(gen_pGetSlaveInfo)))
        out_pGetSlaveInfo.value = CEcWrapperTypes.Conv(gen_pGetSlaveInfo, "DN_EC_T_GET_SLAVE_INFO")
        return gen_dwRetVal

    def GetCfgSlaveInfo(self, bStationAddress, wSlaveAddress, out_pSlaveInfo): # ret: ECError
        """
        Return information about a configured slave from the ENI file

        Args:
            bStationAddress: 
            wSlaveAddress: Slave address according bFixedAddressing
            pSlaveInfo: Information about the slave.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bStationAddress = CEcWrapperTypes.Conv(bStationAddress, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        pSlaveInfo = DN_EC_T_CFG_SLAVE_INFO()
        gen_pSlaveInfo = CEcWrapperTypes.Conv(pSlaveInfo, "EC_T_CFG_SLAVE_INFO")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetCfgSlaveInfo(self.m_dwMasterInstanceId, gen_bStationAddress, gen_wSlaveAddress, ctypes.pointer(gen_pSlaveInfo)))
        out_pSlaveInfo.value = CEcWrapperTypes.Conv(gen_pSlaveInfo, "DN_EC_T_CFG_SLAVE_INFO")
        return gen_dwRetVal

    def GetCfgSlaveEoeInfo(self, bStationAddress, wSlaveAddress, out_pSlaveEoeInfo): # ret: ECError
        """
        Return EoE information about a configured slave from the ENI file

        Args:
            bStationAddress: 
            wSlaveAddress: Slave address according bFixedAddressing
            pSlaveEoeInfo: Information about the slave

        Returns:
            - EC_E_NOERROR if successful
            - EC_E_NOTFOUND if the slave with the given address does not exist
            - EC_E_NO_MBX_SUPPORT if the slave does not support mailbox communication
            - EC_E_NO_EOE_SUPPORT if the slave supports mailbox communication, but not EoE
        """
        gen_bStationAddress = CEcWrapperTypes.Conv(bStationAddress, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        pSlaveEoeInfo = DN_EC_T_CFG_SLAVE_EOE_INFO()
        gen_pSlaveEoeInfo = CEcWrapperTypes.Conv(pSlaveEoeInfo, "EC_T_CFG_SLAVE_EOE_INFO")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetCfgSlaveEoeInfo(self.m_dwMasterInstanceId, gen_bStationAddress, gen_wSlaveAddress, ctypes.pointer(gen_pSlaveEoeInfo)))
        out_pSlaveEoeInfo.value = CEcWrapperTypes.Conv(gen_pSlaveEoeInfo, "DN_EC_T_CFG_SLAVE_EOE_INFO")
        return gen_dwRetVal

    def GetBusSlaveInfo(self, bStationAddress, wSlaveAddress, out_pSlaveInfo): # ret: ECError
        """
        Return information about a slave connected to the EtherCAT bus

        Args:
            bStationAddress: 
            wSlaveAddress: Slave address according bFixedAddressing
            pSlaveInfo: Information from the slave.

        Returns:
            - EC_E_NOERROR if successful
            - EC_E_NOTFOUND if the slave with the given address does not exist
        """
        gen_bStationAddress = CEcWrapperTypes.Conv(bStationAddress, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        pSlaveInfo = DN_EC_T_BUS_SLAVE_INFO()
        gen_pSlaveInfo = CEcWrapperTypes.Conv(pSlaveInfo, "EC_T_BUS_SLAVE_INFO")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetBusSlaveInfo(self.m_dwMasterInstanceId, gen_bStationAddress, gen_wSlaveAddress, ctypes.pointer(gen_pSlaveInfo)))
        out_pSlaveInfo.value = CEcWrapperTypes.Conv(gen_pSlaveInfo, "DN_EC_T_BUS_SLAVE_INFO")
        return gen_dwRetVal

    def GetSlaveInpVarInfoNumOf(self, bFixedAddressing, wSlaveAddress, out_pwSlaveInpVarInfoNumOf): # ret: ECError
        """
        Gets the number of input variables of a specific slave.

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            pwSlaveInpVarInfoNumOf: Number of found process variable entries

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        pwSlaveInpVarInfoNumOf = 0
        gen_pwSlaveInpVarInfoNumOf = CEcWrapperTypes.Conv(pwSlaveInpVarInfoNumOf, "ushort")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetSlaveInpVarInfoNumOf(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, ctypes.pointer(gen_pwSlaveInpVarInfoNumOf)))
        out_pwSlaveInpVarInfoNumOf.value = CEcWrapperTypes.Conv(gen_pwSlaveInpVarInfoNumOf, "ushort")
        return gen_dwRetVal

    def GetSlaveOutpVarInfoNumOf(self, bFixedAddressing, wSlaveAddress, out_pwSlaveOutpVarInfoNumOf): # ret: ECError
        """
        Gets the number of output variables of a specific slave.

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            pwSlaveOutpVarInfoNumOf: Number of found process variables

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        pwSlaveOutpVarInfoNumOf = 0
        gen_pwSlaveOutpVarInfoNumOf = CEcWrapperTypes.Conv(pwSlaveOutpVarInfoNumOf, "ushort")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetSlaveOutpVarInfoNumOf(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, ctypes.pointer(gen_pwSlaveOutpVarInfoNumOf)))
        out_pwSlaveOutpVarInfoNumOf.value = CEcWrapperTypes.Conv(gen_pwSlaveOutpVarInfoNumOf, "ushort")
        return gen_dwRetVal

    def GetSlaveInpVarInfo(self, bFixedAddressing, wSlaveAddress, wNumOfVarsToRead, out_pSlaveProcVarInfoEntries, out_pwReadEntries): # ret: ECError
        """
        Gets the number of input variables of a specific slave.

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            wNumOfVarsToRead: 
            pSlaveProcVarInfoEntries: 
            pwReadEntries: 

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_wNumOfVarsToRead = CEcWrapperTypes.Conv(wNumOfVarsToRead, "ushort")
        pSlaveProcVarInfoEntries = CEcWrapperTypes.CreateStructArray(DN_EC_T_PROCESS_VAR_INFO(), wNumOfVarsToRead)
        gen_pSlaveProcVarInfoEntries = CEcWrapperTypes.Conv(pSlaveProcVarInfoEntries, "EC_T_PROCESS_VAR_INFO")
        gen_pSlaveProcVarInfoEntries_bytes = CEcWrapperTypes.ConvStructArrayToBytes(gen_pSlaveProcVarInfoEntries)
        pwReadEntries = 0
        gen_pwReadEntries = CEcWrapperTypes.Conv(pwReadEntries, "ushort")
        if wNumOfVarsToRead == 0: return ECError.EC_NOERROR
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetSlaveInpVarInfo(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, gen_wNumOfVarsToRead, gen_pSlaveProcVarInfoEntries_bytes, ctypes.pointer(gen_pwReadEntries)))
        out_pwReadEntries.value = CEcWrapperTypes.Conv(gen_pwReadEntries, "ushort")
        gen_pSlaveProcVarInfoEntries = CEcWrapperTypes.ConvStructArrayFromBytes(gen_pSlaveProcVarInfoEntries_bytes, gen_pSlaveProcVarInfoEntries)
        out_pSlaveProcVarInfoEntries.value = CEcWrapperTypes.Conv(gen_pSlaveProcVarInfoEntries, "DN_EC_T_PROCESS_VAR_INFO")
        return gen_dwRetVal

    def GetSlaveInpVarInfoEx(self, bFixedAddressing, wSlaveAddress, wNumOfVarsToRead, out_pSlaveProcVarInfoEntries, out_pwReadEntries): # ret: ECError
        """
        Gets the input process variable extended information entries of a specific slave.

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            wNumOfVarsToRead: Number process variable entries that have been stored in pSlaveProcVarInfoEntries
            pSlaveProcVarInfoEntries: Number process variable entries that have been stored in pSlaveProcVarInfoEntries
            pwReadEntries: The number of read process variable information entries

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_wNumOfVarsToRead = CEcWrapperTypes.Conv(wNumOfVarsToRead, "ushort")
        pSlaveProcVarInfoEntries = CEcWrapperTypes.CreateStructArray(DN_EC_T_PROCESS_VAR_INFO_EX(), wNumOfVarsToRead)
        gen_pSlaveProcVarInfoEntries = CEcWrapperTypes.Conv(pSlaveProcVarInfoEntries, "EC_T_PROCESS_VAR_INFO_EX")
        gen_pSlaveProcVarInfoEntries_bytes = CEcWrapperTypes.ConvStructArrayToBytes(gen_pSlaveProcVarInfoEntries)
        pwReadEntries = 0
        gen_pwReadEntries = CEcWrapperTypes.Conv(pwReadEntries, "ushort")
        if wNumOfVarsToRead == 0: return ECError.EC_NOERROR
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetSlaveInpVarInfoEx(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, gen_wNumOfVarsToRead, gen_pSlaveProcVarInfoEntries_bytes, ctypes.pointer(gen_pwReadEntries)))
        out_pwReadEntries.value = CEcWrapperTypes.Conv(gen_pwReadEntries, "ushort")
        gen_pSlaveProcVarInfoEntries = CEcWrapperTypes.ConvStructArrayFromBytes(gen_pSlaveProcVarInfoEntries_bytes, gen_pSlaveProcVarInfoEntries)
        out_pSlaveProcVarInfoEntries.value = CEcWrapperTypes.Conv(gen_pSlaveProcVarInfoEntries, "DN_EC_T_PROCESS_VAR_INFO_EX")
        return gen_dwRetVal

    def GetSlaveOutpVarInfo(self, bFixedAddressing, wSlaveAddress, wNumOfVarsToRead, out_pSlaveProcVarInfoEntries, out_pwReadEntries): # ret: ECError
        """
        Gets the number of output variables of a specific slave.

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            wNumOfVarsToRead: 
            pSlaveProcVarInfoEntries: 
            pwReadEntries: 

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_wNumOfVarsToRead = CEcWrapperTypes.Conv(wNumOfVarsToRead, "ushort")
        pSlaveProcVarInfoEntries = CEcWrapperTypes.CreateStructArray(DN_EC_T_PROCESS_VAR_INFO(), wNumOfVarsToRead)
        gen_pSlaveProcVarInfoEntries = CEcWrapperTypes.Conv(pSlaveProcVarInfoEntries, "EC_T_PROCESS_VAR_INFO")
        gen_pSlaveProcVarInfoEntries_bytes = CEcWrapperTypes.ConvStructArrayToBytes(gen_pSlaveProcVarInfoEntries)
        pwReadEntries = 0
        gen_pwReadEntries = CEcWrapperTypes.Conv(pwReadEntries, "ushort")
        if wNumOfVarsToRead == 0: return ECError.EC_NOERROR
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetSlaveOutpVarInfo(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, gen_wNumOfVarsToRead, gen_pSlaveProcVarInfoEntries_bytes, ctypes.pointer(gen_pwReadEntries)))
        out_pwReadEntries.value = CEcWrapperTypes.Conv(gen_pwReadEntries, "ushort")
        gen_pSlaveProcVarInfoEntries = CEcWrapperTypes.ConvStructArrayFromBytes(gen_pSlaveProcVarInfoEntries_bytes, gen_pSlaveProcVarInfoEntries)
        out_pSlaveProcVarInfoEntries.value = CEcWrapperTypes.Conv(gen_pSlaveProcVarInfoEntries, "DN_EC_T_PROCESS_VAR_INFO")
        return gen_dwRetVal

    def GetSlaveOutpVarInfoEx(self, bFixedAddressing, wSlaveAddress, wNumOfVarsToRead, out_pSlaveProcVarInfoEntries, out_pwReadEntries): # ret: ECError
        """
        Gets the output process variable extended information entries of a specific slave.

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            wNumOfVarsToRead: Number of process variable information entries
            pSlaveProcVarInfoEntries: The read process extended variable entries
            pwReadEntries: The number of read process variable information entries

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_wNumOfVarsToRead = CEcWrapperTypes.Conv(wNumOfVarsToRead, "ushort")
        pSlaveProcVarInfoEntries = CEcWrapperTypes.CreateStructArray(DN_EC_T_PROCESS_VAR_INFO_EX(), wNumOfVarsToRead)
        gen_pSlaveProcVarInfoEntries = CEcWrapperTypes.Conv(pSlaveProcVarInfoEntries, "EC_T_PROCESS_VAR_INFO_EX")
        gen_pSlaveProcVarInfoEntries_bytes = CEcWrapperTypes.ConvStructArrayToBytes(gen_pSlaveProcVarInfoEntries)
        pwReadEntries = 0
        gen_pwReadEntries = CEcWrapperTypes.Conv(pwReadEntries, "ushort")
        if wNumOfVarsToRead == 0: return ECError.EC_NOERROR
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetSlaveOutpVarInfoEx(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, gen_wNumOfVarsToRead, gen_pSlaveProcVarInfoEntries_bytes, ctypes.pointer(gen_pwReadEntries)))
        out_pwReadEntries.value = CEcWrapperTypes.Conv(gen_pwReadEntries, "ushort")
        gen_pSlaveProcVarInfoEntries = CEcWrapperTypes.ConvStructArrayFromBytes(gen_pSlaveProcVarInfoEntries_bytes, gen_pSlaveProcVarInfoEntries)
        out_pSlaveProcVarInfoEntries.value = CEcWrapperTypes.Conv(gen_pSlaveProcVarInfoEntries, "DN_EC_T_PROCESS_VAR_INFO_EX")
        return gen_dwRetVal

    def GetSlaveOutpVarByObjectEx(self, bFixedAddressing, wSlaveAddress, wIndex, wSubIndex, out_pProcessVarInfoEntry): # ret: ECError
        """
        Gets the input process variable extended information entry by object index, subindex of a specific slave.

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            wIndex: Object index
            wSubIndex: Object sub index
            pProcessVarInfoEntry: Process variable extended information entry

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_wIndex = CEcWrapperTypes.Conv(wIndex, "ushort")
        gen_wSubIndex = CEcWrapperTypes.Conv(wSubIndex, "ushort")
        pProcessVarInfoEntry = DN_EC_T_PROCESS_VAR_INFO_EX()
        gen_pProcessVarInfoEntry = CEcWrapperTypes.Conv(pProcessVarInfoEntry, "EC_T_PROCESS_VAR_INFO_EX")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetSlaveOutpVarByObjectEx(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, gen_wIndex, gen_wSubIndex, ctypes.pointer(gen_pProcessVarInfoEntry)))
        out_pProcessVarInfoEntry.value = CEcWrapperTypes.Conv(gen_pProcessVarInfoEntry, "DN_EC_T_PROCESS_VAR_INFO_EX")
        return gen_dwRetVal

    def GetSlaveInpVarByObjectEx(self, bFixedAddressing, wSlaveAddress, wIndex, wSubIndex, out_pProcessVarInfoEntry): # ret: ECError
        """
        Gets the input process variable extended information entry by object index, subindex of a specific slave.

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            wIndex: Object index
            wSubIndex: Object sub index
            pProcessVarInfoEntry: Process variable extended information entry

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_wIndex = CEcWrapperTypes.Conv(wIndex, "ushort")
        gen_wSubIndex = CEcWrapperTypes.Conv(wSubIndex, "ushort")
        pProcessVarInfoEntry = DN_EC_T_PROCESS_VAR_INFO_EX()
        gen_pProcessVarInfoEntry = CEcWrapperTypes.Conv(pProcessVarInfoEntry, "EC_T_PROCESS_VAR_INFO_EX")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetSlaveInpVarByObjectEx(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, gen_wIndex, gen_wSubIndex, ctypes.pointer(gen_pProcessVarInfoEntry)))
        out_pProcessVarInfoEntry.value = CEcWrapperTypes.Conv(gen_pProcessVarInfoEntry, "DN_EC_T_PROCESS_VAR_INFO_EX")
        return gen_dwRetVal

    def FindOutpVarByName(self, szVariableName, out_pSlaveOutpVarInfo): # ret: ECError
        """
        Finds an output process variable information entry by the variable name.

        Args:
            szVariableName: Variable name
            pSlaveOutpVarInfo: 

        Returns:
            EC_E_NOERROR or error code
        """
        gen_szVariableName = CEcWrapperTypes.Conv(szVariableName, "string")
        pSlaveOutpVarInfo = DN_EC_T_PROCESS_VAR_INFO()
        gen_pSlaveOutpVarInfo = CEcWrapperTypes.Conv(pSlaveOutpVarInfo, "EC_T_PROCESS_VAR_INFO")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwFindOutpVarByName(self.m_dwMasterInstanceId, gen_szVariableName, ctypes.pointer(gen_pSlaveOutpVarInfo)))
        out_pSlaveOutpVarInfo.value = CEcWrapperTypes.Conv(gen_pSlaveOutpVarInfo, "DN_EC_T_PROCESS_VAR_INFO")
        return gen_dwRetVal

    def FindOutpVarByNameEx(self, szVariableName, out_pProcessVarInfoEntry): # ret: ECError
        """
        Finds an output process variable extended information entry by the variable name.

        Args:
            szVariableName: Variable name
            pProcessVarInfoEntry: Process variable extended information entry

        Returns:
            EC_E_NOERROR or error code
        """
        gen_szVariableName = CEcWrapperTypes.Conv(szVariableName, "string")
        pProcessVarInfoEntry = DN_EC_T_PROCESS_VAR_INFO_EX()
        gen_pProcessVarInfoEntry = CEcWrapperTypes.Conv(pProcessVarInfoEntry, "EC_T_PROCESS_VAR_INFO_EX")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwFindOutpVarByNameEx(self.m_dwMasterInstanceId, gen_szVariableName, ctypes.pointer(gen_pProcessVarInfoEntry)))
        out_pProcessVarInfoEntry.value = CEcWrapperTypes.Conv(gen_pProcessVarInfoEntry, "DN_EC_T_PROCESS_VAR_INFO_EX")
        return gen_dwRetVal

    def FindInpVarByName(self, szVariableName, out_pProcessVarInfoEntry): # ret: ECError
        """
        Finds an input process variable information entry by the variable name.

        Args:
            szVariableName: Variable name
            pProcessVarInfoEntry: Process variable information entry

        Returns:
            EC_E_NOERROR or error code
        """
        gen_szVariableName = CEcWrapperTypes.Conv(szVariableName, "string")
        pProcessVarInfoEntry = DN_EC_T_PROCESS_VAR_INFO()
        gen_pProcessVarInfoEntry = CEcWrapperTypes.Conv(pProcessVarInfoEntry, "EC_T_PROCESS_VAR_INFO")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwFindInpVarByName(self.m_dwMasterInstanceId, gen_szVariableName, ctypes.pointer(gen_pProcessVarInfoEntry)))
        out_pProcessVarInfoEntry.value = CEcWrapperTypes.Conv(gen_pProcessVarInfoEntry, "DN_EC_T_PROCESS_VAR_INFO")
        return gen_dwRetVal

    def FindInpVarByNameEx(self, szVariableName, out_pProcessVarInfoEntry): # ret: ECError
        """
        Finds an input process variable extended information entry by the variable name.

        Args:
            szVariableName: Variable name
            pProcessVarInfoEntry: Process variable extended information entry

        Returns:
            EC_E_NOERROR or error code
        """
        gen_szVariableName = CEcWrapperTypes.Conv(szVariableName, "string")
        pProcessVarInfoEntry = DN_EC_T_PROCESS_VAR_INFO_EX()
        gen_pProcessVarInfoEntry = CEcWrapperTypes.Conv(pProcessVarInfoEntry, "EC_T_PROCESS_VAR_INFO_EX")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwFindInpVarByNameEx(self.m_dwMasterInstanceId, gen_szVariableName, ctypes.pointer(gen_pProcessVarInfoEntry)))
        out_pProcessVarInfoEntry.value = CEcWrapperTypes.Conv(gen_pProcessVarInfoEntry, "DN_EC_T_PROCESS_VAR_INFO_EX")
        return gen_dwRetVal

    def EthDbgMsg(self, byEthTypeByte0, byEthTypeByte1, szMsg): # ret: ECError
        """
        Send a debug message to the EtherCAT Link Layer.
        
        This feature can be used for debugging purposes.

        Args:
            byEthTypeByte0: Ethernet type byte 0
            byEthTypeByte1: Ethernet type byte 1
            szMsg: Message to send to link layer

        Returns:
            EC_E_NOERROR or error code
        """
        gen_byEthTypeByte0 = CEcWrapperTypes.Conv(byEthTypeByte0, "byte")
        gen_byEthTypeByte1 = CEcWrapperTypes.Conv(byEthTypeByte1, "byte")
        gen_szMsg = CEcWrapperTypes.Conv(szMsg, "string")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwEthDbgMsg(self.m_dwMasterInstanceId, gen_byEthTypeByte0, gen_byEthTypeByte1, gen_szMsg))
        return gen_dwRetVal

    def BlockNode(self, pMisMatch, dwTimeout): # ret: ECError
        """
        Blocks a slaves on a specific port
        
        If an invalid slave node is connected, which happens bus topology scan to fail, the previous port, where this node is connected to can be shut down with this call.
        This allows a HotConnect system to be not disturbed, if unknown nodes are connected.
        If this function will be executed on a HotConnect member (a slave which is part of a hot connect group) the complete hot connect group will be excluded from the bus.
        This function may only be called from within the JobTask's context with parameter dwTimeout set to EC_NOWAIT.

        Args:
            pMisMatch: Pointer to EC_T_SB_MISMATCH_DESC carrying mismatch descriptor.
            dwTimeout: Timeout [ms]. The function will block at most for this time.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_pMisMatch = CEcWrapperTypes.Conv(pMisMatch, "EC_T_SB_MISMATCH_DESC")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwBlockNode(self.m_dwMasterInstanceId, gen_pMisMatch, gen_dwTimeout))
        return gen_dwRetVal

    def OpenBlockedPorts(self, dwTimeout): # ret: ECError
        """
        Opens all blocked ports
        
        This call allows re-opening all blocked ports to check whether mismatch cause is removed from bus.
        This function may only be called from within the JobTask's context with parameter dwTimeout set to EC_NOWAIT.

        Args:
            dwTimeout: Timeout [ms]. The function will block at most for this time.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwOpenBlockedPorts(self.m_dwMasterInstanceId, gen_dwTimeout))
        return gen_dwRetVal

    def ForceTopologyChange(self): # ret: ECError
        """
        Force changed topology
        
        Trigger HC State Machine

        Args:

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwForceTopologyChange(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def IsTopologyChangeDetected(self, pbTopologyChangeDetected): # ret: ECError
        """
        Returns whether topology change detected.

        Args:
            pbTopologyChangeDetected: Pointer to EC_T_BOOL value: EC_TRUE if Topology Change Detected, EC_FALSE if not.

        Returns:
            - EC_E_NOERROR if successful
            - EC_E_INVALIDSTATE if the master is not initialized
        """
        gen_pbTopologyChangeDetected = CEcWrapperTypes.Conv(pbTopologyChangeDetected, "bool")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwIsTopologyChangeDetected(self.m_dwMasterInstanceId, ctypes.pointer(gen_pbTopologyChangeDetected)))
        return gen_dwRetVal

    def IsTopologyKnown(self, out_pbTopologyKnown): # ret: ECError
        """
        Returns whether topology known

        Args:
            pbTopologyKnown: Topology known

        Returns:
            EC_E_NOERROR or error code
        """
        pbTopologyKnown = False
        gen_pbTopologyKnown = CEcWrapperTypes.Conv(pbTopologyKnown, "bool")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwIsTopologyKnown(self.m_dwMasterInstanceId, ctypes.pointer(gen_pbTopologyKnown)))
        out_pbTopologyKnown.value = CEcWrapperTypes.Conv(gen_pbTopologyKnown, "bool")
        return gen_dwRetVal

    def GetBusTime(self, out_pqwBusTime): # ret: ECError
        """
        This function returns the actual bus time in nanoseconds.

        Args:
            pqwBusTime: Bus time [ns]

        Returns:
            EC_E_NOERROR or error code
        """
        pqwBusTime = 0
        gen_pqwBusTime = CEcWrapperTypes.Conv(pqwBusTime, "uint64")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetBusTime(self.m_dwMasterInstanceId, ctypes.pointer(gen_pqwBusTime)))
        out_pqwBusTime.value = CEcWrapperTypes.Conv(gen_pqwBusTime, "uint64")
        return gen_dwRetVal

    def IsSlavePresent(self, dwSlaveId, out_pbPresence): # ret: ECError
        """
        Returns whether a specific slave is currently connected to the Bus.
        
        This function may be called from within the JobTask.

        Args:
            dwSlaveId: Slave ID
            pbPresence: EC_TRUE if slave is currently connected to the bus, EC_FALSE if not.

        Returns:
            - EC_E_NOERROR if successful
            - EC_E_INVALIDSTATE if the master is not initialized
            - EC_E_NOTFOUND if the slave with ID dwSlaveId does not exist or no ENI File was loaded
        """
        gen_dwSlaveId = CEcWrapperTypes.Conv(dwSlaveId, "uint")
        pbPresence = False
        gen_pbPresence = CEcWrapperTypes.Conv(pbPresence, "bool")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwIsSlavePresent(self.m_dwMasterInstanceId, gen_dwSlaveId, ctypes.pointer(gen_pbPresence)))
        out_pbPresence.value = CEcWrapperTypes.Conv(gen_pbPresence, "bool")
        return gen_dwRetVal

    def PassThroughSrvGetStatus(self): # ret: DN_EC_PTS_STATE
        """
        Gets the status of the Pass-through server.

        Args:

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwRetVal = DN_EC_PTS_STATE(CEcWrapperTypes.ConvRes(CEcWrapper.Get().ecwPassThroughSrvGetStatus(self.m_dwMasterInstanceId)))
        return gen_dwRetVal

    def PassThroughSrvStart(self, poPtsStartParams, dwTimeout): # ret: ECError
        """
        Starts the Pass Through Server

        Args:
            poPtsStartParams: Pass through server start parameter
            dwTimeout: Timeout

        Returns:
            EC_E_NOERROR or error code
        """
        gen_poPtsStartParams = CEcWrapperTypes.Conv(poPtsStartParams, "EC_T_PTS_SRV_START_PARMS")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwPassThroughSrvStart(self.m_dwMasterInstanceId, gen_poPtsStartParams, gen_dwTimeout))
        return gen_dwRetVal

    def PassThroughSrvStop(self, dwTimeout): # ret: ECError
        """
        Stops the Pass Through Server

        Args:
            dwTimeout: Timeout

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwPassThroughSrvStop(self.m_dwMasterInstanceId, gen_dwTimeout))
        return gen_dwRetVal

    def PassThroughSrvEnable(self, dwTimeout): # ret: ECError
        """
        Enables the Pass-through server.

        Args:
            dwTimeout: Timeout [ms]

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwPassThroughSrvEnable(self.m_dwMasterInstanceId, gen_dwTimeout))
        return gen_dwRetVal

    def PassThroughSrvDisable(self, dwTimeout): # ret: ECError
        """
        Disables the Pass-through server.

        Args:
            dwTimeout: Timeout [ms]

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwPassThroughSrvDisable(self.m_dwMasterInstanceId, gen_dwTimeout))
        return gen_dwRetVal

    def AdsAdapterStart(self, poStartParams, dwTimeout): # ret: ECError
        """
        Starts the ADS Pass Through Server

        Args:
            poStartParams: Pass through server start parameter
            dwTimeout: Timeout

        Returns:
            EC_E_NOERROR or error code
        """
        gen_poStartParams = CEcWrapperTypes.Conv(poStartParams, "EC_T_ADS_ADAPTER_START_PARMS")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwAdsAdapterStart(self.m_dwMasterInstanceId, gen_poStartParams, gen_dwTimeout))
        return gen_dwRetVal

    def AdsAdapterStop(self, dwTimeout): # ret: ECError
        """
        Stops the ADS Pass Through Server

        Args:
            dwTimeout: Timeout

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwAdsAdapterStop(self.m_dwMasterInstanceId, gen_dwTimeout))
        return gen_dwRetVal

    def GetSrcMacAddress(self, out_pMacSrc): # ret: ECError
        """
        Gets the source MAC address

        Args:
            pMacSrc: 6-byte buffer to write source MAC address to.

        Returns:
            EC_E_NOERROR or error code
        """
        pMacSrc = DN_ETHERNET_ADDRESS()
        gen_pMacSrc = CEcWrapperTypes.Conv(pMacSrc, "ETHERNET_ADDRESS")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetSrcMacAddress(self.m_dwMasterInstanceId, ctypes.pointer(gen_pMacSrc)))
        out_pMacSrc.value = CEcWrapperTypes.Conv(gen_pMacSrc, "DN_ETHERNET_ADDRESS")
        return gen_dwRetVal

    def SetLicenseKey(self, pszLicenseKey): # ret: ECError
        """
        Sets the license key for the protected version of EC-Master.
        
        Must be called after initialization and before configuration. This function may not be called if a non protected version is used.

        Args:
            pszLicenseKey: License key as zero terminated string with 26 characters.

        Returns:
            - EC_E_NOERROR or error code
            - EC_E_INVALIDSIZE the format of the license key is wrong. The correct length is 26 characters
            - EC_E_LICENSE_MISSING the license key doesn't match to the MAC Address
        """
        gen_pszLicenseKey = CEcWrapperTypes.Conv(pszLicenseKey, "string")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwSetLicenseKey(self.m_dwMasterInstanceId, gen_pszLicenseKey))
        return gen_dwRetVal

    def GetVersion(self, out_pdwVersion): # ret: ECError
        """
        Gets the version number as a 32-bit value

        Args:
            pdwVersion: Pointer to EC_T_DWORD to carry out version number

        Returns:
            - EC_E_NOERROR
            - EC_E_INVALIDPARM
        """
        pdwVersion = 0
        gen_pdwVersion = CEcWrapperTypes.Conv(pdwVersion, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetVersion(self.m_dwMasterInstanceId, ctypes.pointer(gen_pdwVersion)))
        out_pdwVersion.value = CEcWrapperTypes.Conv(gen_pdwVersion, "uint")
        return gen_dwRetVal

    def TraceDataConfig(self, wTraceDataSize): # ret: ECError
        """
        Configures a trace data buffer and enables it for transmission
        
        Must be called after initialization and before configuration.
        
        \note If wTraceDataSize is too large, configuration will fail with return code #EC_E_XML_CYCCMDS_SIZEMISMATCH.

        Args:
            wTraceDataSize: Size of Trace Data in bytes

        Returns:
            - EC_E_NOERROR or error code
            - EC_E_NOTSUPPORTED if eCycFrameLayout_FIXED is configured
        """
        gen_wTraceDataSize = CEcWrapperTypes.Conv(wTraceDataSize, "ushort")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwTraceDataConfig(self.m_dwMasterInstanceId, gen_wTraceDataSize))
        return gen_dwRetVal

    def TraceDataGetInfo(self, pTraceDataInfo): # ret: ECError
        """
        Get information about the offset and size of trace data.
        
        The trace data buffer is locate in EC_T_TRACE_DATA_INFO.pbyData at the byte offset EC_T_TRACE_DATA_INFO.dwOffset.

        Args:
            pTraceDataInfo: Information about trace data

        Returns:
            EC_E_NOERROR or error code
        """
        gen_pTraceDataInfo = CEcWrapperTypes.Conv(pTraceDataInfo, "EC_T_TRACE_DATA_INFO")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwTraceDataGetInfo(self.m_dwMasterInstanceId, gen_pTraceDataInfo))
        return gen_dwRetVal

    def FastModeInit(self): # ret: ECError
        """
        

        Args:

        Returns:
            
        """
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwFastModeInit(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def FastSendAllCycFrames(self): # ret: ECError
        """
        

        Args:

        Returns:
            
        """
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwFastSendAllCycFrames(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def FastProcessAllRxFrames(self, pbAreAllCycFramesProcessed): # ret: ECError
        """
        

        Args:
            pbAreAllCycFramesProcessed: 

        Returns:
            
        """
        gen_pbAreAllCycFramesProcessed = CEcWrapperTypes.Conv(pbAreAllCycFramesProcessed, "bool")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwFastProcessAllRxFrames(self.m_dwMasterInstanceId, ctypes.pointer(gen_pbAreAllCycFramesProcessed)))
        return gen_dwRetVal

    def ReadSlaveIdentification(self, bFixedAddressing, wSlaveAddress, wAdo, pwValue, dwTimeout): # ret: ECError
        """
        Read identification value from slave.
        
        This function may not be called from within the JobTask's context.

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            wAdo: ADO used for identification command
            pwValue: Pointer to Word value containing the Identification value
            dwTimeout: Timeout [ms]

        Returns:
            - EC_E_NOERROR if successful
            - EC_E_SLAVE_NOT_PRESENT if slave not present
            - EC_E_BUSY another transfer request is already pending
            - EC_E_NOTFOUND if the slave with the given address does not exist
            - EC_E_NOTREADY if the working counter was not set when sending the command (slave may not be connected or did not respond)
            - EC_E_TIMEOUT if the slave did not respond to the command
            - EC_E_BUSY if the master or the corresponding slave is currently changing its operational state
            - EC_E_INVALIDPARM if the command is not supported or the timeout value is set to EC_NOWAIT
            - EC_E_ADO_NOT_SUPPORTED if the slave does not support requesting ID mechanism
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_wAdo = CEcWrapperTypes.Conv(wAdo, "ushort")
        gen_pwValue = CEcWrapperTypes.Conv(pwValue, "ushort")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwReadSlaveIdentification(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, gen_wAdo, ctypes.pointer(gen_pwValue), gen_dwTimeout))
        return gen_dwRetVal

    def ReadSlaveIdentificationReq(self, dwClientId, dwTferId, bFixedAddressing, wSlaveAddress, wAdo, pwValue, dwTimeout): # ret: ECError
        """
        Request the identification value from a slave and returns immediately.
        
        A notification EC_NOTIFY_SLAVE_IDENTIFICATION is given on completion or timeout.
        This function may be called from within the JobTask's context.

        Args:
            dwClientId: Client ID returned by RegisterClient (0 if all registered clients shall be notified).
            dwTferId: Transfer ID. The application can set this ID to identify the transfer. It will be passed back to the application within EC_T_SLAVE_IDENTIFICATION_NTFY_DESC
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            wAdo: ADO used for identification command
            pwValue: Pointer to Word value containing the Identification value, must be valid until the request complete.
            dwTimeout: Timeout [ms]

        Returns:
            - EC_E_NOERROR if successful
            - EC_E_SLAVE_NOT_PRESENT if slave not present
            - EC_E_NOTFOUND if the slave with the given address does not exist
            - EC_E_INVALIDPARM if the command is not supported or the timeout value is set to EC_NOWAIT
            - EC_E_ADO_NOT_SUPPORTED if the slave does not support requesting ID mechanism
        """
        gen_dwClientId = CEcWrapperTypes.Conv(dwClientId, "uint")
        gen_dwTferId = CEcWrapperTypes.Conv(dwTferId, "uint")
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_wAdo = CEcWrapperTypes.Conv(wAdo, "ushort")
        gen_pwValue = CEcWrapperTypes.Conv(pwValue, "ushort")
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwReadSlaveIdentificationReq(self.m_dwMasterInstanceId, gen_dwClientId, gen_dwTferId, gen_bFixedAddressing, gen_wSlaveAddress, gen_wAdo, ctypes.pointer(gen_pwValue), gen_dwTimeout))
        return gen_dwRetVal

    def SetSlaveDisabled(self, bFixedAddressing, wSlaveAddress, bDisabled): # ret: ECError
        """
        Enable or disable a specific slave
        
        The EtherCAT state of disabled slaves can not be set higher than PREOP. If the state is higher than PREOP at the time this function is called.
        The state will be automatically change to PREOP. The information about the last requested state is lost and is set to PREOP too.

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            bDisabled: EC_TRUE: Disable slave, EC_FALSE: Enable slave

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_bDisabled = CEcWrapperTypes.Conv(bDisabled, "bool")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwSetSlaveDisabled(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, gen_bDisabled))
        return gen_dwRetVal

    def SetSlavesDisabled(self, bFixedAddressing, wSlaveAddress, eSlaveSelection, bDisabled): # ret: ECError
        """
        Enable or disable a specific group of slaves
        
        The EtherCAT state of disabled slaves can not be set higher than PREOP. If the state is higher than PREOP at the time this function is called, the state will be automatically change to PREOP.
        The information about the last requested state is lost and is set to PREOP too.

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            eSlaveSelection: Slave selection criteria for following slaves
            bDisabled: EC_TRUE: Disable slaves, EC_FALSE: Enable slaves.

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_eSlaveSelection = CEcWrapperTypes.Conv(eSlaveSelection, "EC_T_SLAVE_SELECTION")
        gen_bDisabled = CEcWrapperTypes.Conv(bDisabled, "bool")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwSetSlavesDisabled(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, gen_eSlaveSelection, gen_bDisabled))
        return gen_dwRetVal

    def SetSlaveDisconnected(self, bFixedAddressing, wSlaveAddress, bDisconnected): # ret: ECError
        """
        Mark specific slave for connection or disconnection
        
        The EtherCAT state of disconnected slaves can not be set higher than INIT. If the state is higher than INIT at the time this function is called, the state will be automatically change to INIT.
        The information about the last requested state is lost and is set to INIT too.

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            bDisconnected: EC_TRUE: Mark slave for disconnection, EC_FALSE: Mark slave for (re-)connection

        Returns:
            EC_E_NOERROR or error code
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_bDisconnected = CEcWrapperTypes.Conv(bDisconnected, "bool")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwSetSlaveDisconnected(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, gen_bDisconnected))
        return gen_dwRetVal

    def SetSlavesDisconnected(self, bFixedAddressing, wSlaveAddress, eSlaveSelection, bDisconnected): # ret: ECError
        """
        Mark a specific group of slaves for connection or disconnection

        Args:
            bFixedAddressing: EC_TRUE: use station address, EC_FALSE: use AutoInc address
            wSlaveAddress: Slave address according bFixedAddressing
            eSlaveSelection: Slave selection criteria
            bDisconnected: EC_TRUE: mark slaves for disconnection, EC_FALSE: mark slaves for connection

        Returns:
            - EC_E_NOERROR or error code
            - EC_E_NOTFOUND if the slave does not exist
        """
        gen_bFixedAddressing = CEcWrapperTypes.Conv(bFixedAddressing, "bool")
        gen_wSlaveAddress = CEcWrapperTypes.Conv(wSlaveAddress, "ushort")
        gen_eSlaveSelection = CEcWrapperTypes.Conv(eSlaveSelection, "EC_T_SLAVE_SELECTION")
        gen_bDisconnected = CEcWrapperTypes.Conv(bDisconnected, "bool")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwSetSlavesDisconnected(self.m_dwMasterInstanceId, gen_bFixedAddressing, gen_wSlaveAddress, gen_eSlaveSelection, gen_bDisconnected))
        return gen_dwRetVal

    def GetMemoryUsage(self, out_pdwCurrentUsage, out_pdwMaxUsage): # ret: ECError
        """
        Returns information about memory usage.
        
        All calls to malloc/free and new/delete are monitored.

        Args:
            pdwCurrentUsage: Current memory usage in Bytes at the time where this function is called
            pdwMaxUsage: Maximum memory usage in Bytes since initialization at the time where this function is called

        Returns:
            EC_E_NOERROR or error code
        """
        pdwCurrentUsage = 0
        gen_pdwCurrentUsage = CEcWrapperTypes.Conv(pdwCurrentUsage, "uint")
        pdwMaxUsage = 0
        gen_pdwMaxUsage = CEcWrapperTypes.Conv(pdwMaxUsage, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetMemoryUsage(self.m_dwMasterInstanceId, ctypes.pointer(gen_pdwCurrentUsage), ctypes.pointer(gen_pdwMaxUsage)))
        out_pdwMaxUsage.value = CEcWrapperTypes.Conv(gen_pdwMaxUsage, "uint")
        out_pdwCurrentUsage.value = CEcWrapperTypes.Conv(gen_pdwCurrentUsage, "uint")
        return gen_dwRetVal

    def GetSlaveStatistics(self, dwSlaveId, out_pSlaveStatisticsDesc): # ret: ECError
        """
        Get Slave's statistics counter.

        Args:
            dwSlaveId: Slave id
            pSlaveStatisticsDesc: Pointer to structure EC_T_SLVSTATISTICS_DESC

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwSlaveId = CEcWrapperTypes.Conv(dwSlaveId, "uint")
        pSlaveStatisticsDesc = DN_EC_T_SLVSTATISTICS_DESC()
        gen_pSlaveStatisticsDesc = CEcWrapperTypes.Conv(pSlaveStatisticsDesc, "EC_T_SLVSTATISTICS_DESC")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetSlaveStatistics(self.m_dwMasterInstanceId, gen_dwSlaveId, ctypes.pointer(gen_pSlaveStatisticsDesc)))
        out_pSlaveStatisticsDesc.value = CEcWrapperTypes.Conv(gen_pSlaveStatisticsDesc, "DN_EC_T_SLVSTATISTICS_DESC")
        return gen_dwRetVal

    def ClearSlaveStatistics(self, dwSlaveId): # ret: ECError
        """
        Clears all error registers of a slave.

        Args:
            dwSlaveId: Slave Id, INVALID_SLAVE_ID clears all slaves

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwSlaveId = CEcWrapperTypes.Conv(dwSlaveId, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwClearSlaveStatistics(self.m_dwMasterInstanceId, gen_dwSlaveId))
        return gen_dwRetVal

    def GetMasterSyncUnitInfoNumOf(self): # ret: uint
        """
        Get number of Master Sync Units info entries.

        Args:

        Returns:
            Number of Master Sync Units info entries
        """
        gen_dwRetVal = CEcWrapperTypes.ConvRes(CEcWrapper.Get().ecwGetMasterSyncUnitInfoNumOf(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def GetMasterSyncUnitInfo(self, wMsuId, out_pMsuInfo): # ret: ECError
        """
        Get number of Master Sync Units info entries.

        Args:
            wMsuId: 
            pMsuInfo: 

        Returns:
            Number of Master Sync Units info entries
        """
        gen_wMsuId = CEcWrapperTypes.Conv(wMsuId, "ushort")
        pMsuInfo = DN_EC_T_MSU_INFO()
        gen_pMsuInfo = CEcWrapperTypes.Conv(pMsuInfo, "EC_T_MSU_INFO")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetMasterSyncUnitInfo(self.m_dwMasterInstanceId, gen_wMsuId, ctypes.pointer(gen_pMsuInfo)))
        out_pMsuInfo.value = CEcWrapperTypes.Conv(gen_pMsuInfo, "DN_EC_T_MSU_INFO")
        return gen_dwRetVal

    def GetMasterDump(self, pbyBuffer, dwBufferSize, pdwDumpSize): # ret: ECError
        """
        The dump contains relevant information about the master and slave status.
        
        The dump is only intended for internal troubleshooting at acontis Amongst others it contains the following descriptors:
        - EC_T_INIT_MASTER_PARMS
        - EC_T_BUS_DIAGNOSIS_INFO
        - EC_T_MAILBOX_STATISTICS
        - EC_T_CFG_SLAVE_INFO
        - EC_T_BUS_SLAVE_INFO
        - EC_T_SLVSTATISTICS_DESC
        
        The buffer is written until all relevant data have been dumped or buffer size has been exceeded.

        Args:
            pbyBuffer: Preallocated buffer to dump log data
            dwBufferSize: Size of preallocated buffer
            pdwDumpSize: Size of master dump

        Returns:
            - EC_E_NOERROR
            - EC_E_NOMEMORY if buffer too small
        """
        gen_pbyBuffer = CEcWrapperTypes.Conv(pbyBuffer, "byte[]")
        gen_dwBufferSize = CEcWrapperTypes.Conv(dwBufferSize, "uint")
        gen_pdwDumpSize = CEcWrapperTypes.Conv(pdwDumpSize, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetMasterDump(self.m_dwMasterInstanceId, gen_pbyBuffer, gen_dwBufferSize, ctypes.pointer(gen_pdwDumpSize)))
        CEcWrapperTypes.Conv_Array(gen_pbyBuffer, pbyBuffer)
        return gen_dwRetVal

    def BadConnectionsDetect(self, dwTimeout): # ret: ECError
        """
        Detects bad connections
        
        Reads the error counters of all slaves and analyzes the slave ESC error counters:
        - Invalid Frame Counter (0x0300),
        - RX Error Counter (0x0301),
        - Lost Link Counter (0x0310),
        
        whether there is a problem in the area PHY - connector - cable - connector - PHY.
        If one of the above error counters shows a value not equal to zero, an EC_NOTIFY_BAD_CONNECTION is generated, which contains the exact position of the faulty connection.
        It is recommended to call emBadConnectionsReset() on startup of EC-Master to ensure that all error counters of all slaves are in a defined state.

        Args:
            dwTimeout: Timeout [ms] May not be EC_NOWAIT!

        Returns:
            EC_E_NOERROR or error code
        """
        gen_dwTimeout = CEcWrapperTypes.Conv(dwTimeout, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwBadConnectionsDetect(self.m_dwMasterInstanceId, gen_dwTimeout))
        return gen_dwRetVal

    def SelfTestScan(self, pParms): # ret: ECError
        """
        Self test scan
        
        Send a burst of numerous frames and analyze the slave connections.
        After deactivating the job task, frames will be sent as fast as the LinkLayer can send them.
        The size of the frames increases and decreases between the defined limits.
        Dependent on the parameters the BadConnectionsDetect API will analyze the slave connections.

        Args:
            pParms: Self-test scan parameters

        Returns:
            - EC_E_NOERROR or error code
            - EC_E_BAD_CONNECTION if bad connection was detected
            - EC_E_FRAME_LOST if frame(s) lost during self-test
        """
        gen_pParms = CEcWrapperTypes.Conv(pParms, "EC_T_SELFTESTSCAN_PARMS")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwSelfTestScan(self.m_dwMasterInstanceId, ctypes.pointer(gen_pParms)))
        return gen_dwRetVal

    def DisconnectPort(self, wCfgFixedAddress, byPort): # ret: ECError
        """
        

        Args:
            wCfgFixedAddress: 
            byPort: 

        Returns:
            
        """
        gen_wCfgFixedAddress = CEcWrapperTypes.Conv(wCfgFixedAddress, "ushort")
        gen_byPort = CEcWrapperTypes.Conv(byPort, "byte")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDisconnectPort(self.m_dwMasterInstanceId, gen_wCfgFixedAddress, gen_byPort))
        return gen_dwRetVal

    def PowerSlave(self, wCfgFixedAddress, bOn): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def DeleteSlaveCoeObject(self, wCfgFixedAddress, wIndex): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def ClearSlaveCoeObjectDictionary(self, wCfgFixedAddress): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def ResetSlaveCoeObjectDictionary(self, wCfgFixedAddress): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def ConfigureNetwork(self, eCnfType, pbyCnfData, dwCnfDataLen): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def SetErrorAtSlavePort(self, wCfgFixedAddress, byPort, bOutgoing): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def SetErrorGenerationAtSlavePort(self, wCfgFixedAddress, byPort, bOutgoing, dwLikelihoodPpm, dwFixedGoodFramesCnt, dwFixedErroneousFramesCnt): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def ResetErrorGenerationAtSlavePorts(self, wCfgFixedAddress): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def SetLinkDownAtSlavePort(self, wCfgFixedAddress, byPort, bDown, dwLinkDownTimeMs): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def SetLinkDownGenerationAtSlavePort(self, wCfgFixedAddress, byPort, dwLikelihoodPpm, dwFixedLinkDownTimeMs, dwFixedLinkUpTimeMs): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def ResetLinkDownGenerationAtSlavePorts(self, wCfgFixedAddress): # ret: ECError
        # eval limitation
        # pylint: disable=unused-argument
        return self.ReportErrorCode(ECError.EC_NOTSUPPORTED)


    def VoeSend(self, wCfgFixedAddress, wDstFixedAddress, pvData, dwDataLen): # ret: ECError
        """
        

        Args:
            wCfgFixedAddress: 
            wDstFixedAddress: 
            pvData: 
            dwDataLen: 

        Returns:
            
        """
        gen_wCfgFixedAddress = CEcWrapperTypes.Conv(wCfgFixedAddress, "ushort")
        gen_wDstFixedAddress = CEcWrapperTypes.Conv(wDstFixedAddress, "ushort")
        gen_pvData = CEcWrapperTypes.Conv(pvData, "byte[]")
        gen_dwDataLen = CEcWrapperTypes.Conv(dwDataLen, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwVoeSend(self.m_dwMasterInstanceId, gen_wCfgFixedAddress, gen_wDstFixedAddress, gen_pvData, gen_dwDataLen))
        return gen_dwRetVal

    def GetMonitorStatus(self, out_pStatus): # ret: ECError
        """
        

        Args:
            pStatus: 

        Returns:
            
        """
        pStatus = DN_EC_T_MONITOR_STATUS()
        gen_pStatus = CEcWrapperTypes.Conv(pStatus, "EC_T_MONITOR_STATUS")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetMonitorStatus(self.m_dwMasterInstanceId, ctypes.pointer(gen_pStatus)))
        out_pStatus.value = CEcWrapperTypes.Conv(gen_pStatus, "DN_EC_T_MONITOR_STATUS")
        return gen_dwRetVal

    def OpenPacketCapture(self, pParms): # ret: ECError
        """
        

        Args:
            pParms: 

        Returns:
            
        """
        gen_pParms = CEcWrapperTypes.Conv(pParms, "EC_T_PACKETCAPTURE_PARMS")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwOpenPacketCapture(self.m_dwMasterInstanceId, ctypes.pointer(gen_pParms)))
        return gen_dwRetVal

    def ClosePacketCapture(self): # ret: ECError
        """
        

        Args:

        Returns:
            
        """
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwClosePacketCapture(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def GetPacketCaptureInfo(self, out_pInfo): # ret: ECError
        """
        

        Args:
            pInfo: 

        Returns:
            
        """
        pInfo = DN_EC_T_PACKETCAPTURE_INFO()
        gen_pInfo = CEcWrapperTypes.Conv(pInfo, "EC_T_PACKETCAPTURE_INFO")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwGetPacketCaptureInfo(self.m_dwMasterInstanceId, ctypes.pointer(gen_pInfo)))
        out_pInfo.value = CEcWrapperTypes.Conv(gen_pInfo, "DN_EC_T_PACKETCAPTURE_INFO")
        return gen_dwRetVal

    def StartLivePacketCapture(self, pParms): # ret: ECError
        """
        

        Args:
            pParms: 

        Returns:
            
        """
        gen_pParms = CEcWrapperTypes.Conv(pParms, "EC_T_PACKETCAPTURE_PARMS")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwStartLivePacketCapture(self.m_dwMasterInstanceId, ctypes.pointer(gen_pParms)))
        return gen_dwRetVal

    def StopLivePacketCapture(self): # ret: ECError
        """
        

        Args:

        Returns:
            
        """
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwStopLivePacketCapture(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def BacktracePacketCapture(self, pParms): # ret: ECError
        """
        

        Args:
            pParms: 

        Returns:
            
        """
        gen_pParms = CEcWrapperTypes.Conv(pParms, "EC_T_PACKETCAPTURE_PARMS")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwBacktracePacketCapture(self.m_dwMasterInstanceId, ctypes.pointer(gen_pParms)))
        return gen_dwRetVal

    def DaqConfigAddDataSlave(self, wAddress): # ret: ECError
        """
        Add slave to logging.

        Args:
            wAddress: Station address of slave

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_wAddress = CEcWrapperTypes.Conv(wAddress, "ushort")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqConfigAddDataSlave(self.m_dwMasterInstanceId, gen_wAddress))
        return gen_dwRetVal

    def DaqConfigAddDataVariable(self, pszName): # ret: ECError
        """
        Add variable to logging.

        Args:
            pszName: Name of slave

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_pszName = CEcWrapperTypes.Conv(pszName, "string")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqConfigAddDataVariable(self.m_dwMasterInstanceId, gen_pszName))
        return gen_dwRetVal

    def DaqConfigAddDataOversamplingVariable(self, pszName, dwCount): # ret: ECError
        """
        Add oversampling variable to logging.

        Args:
            pszName: Name of slave
            dwCount: Count of oversampling

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_pszName = CEcWrapperTypes.Conv(pszName, "string")
        gen_dwCount = CEcWrapperTypes.Conv(dwCount, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqConfigAddDataOversamplingVariable(self.m_dwMasterInstanceId, gen_pszName, gen_dwCount))
        return gen_dwRetVal

    def DaqConfigAddDataRange(self, dwOffset, dwSize, bInput): # ret: ECError
        """
        Add process data range to logging.

        Args:
            dwOffset: Offset of data
            dwSize: Size of data
            bInput: EC_TRUE: input data, EC_FALSE: output data

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_dwOffset = CEcWrapperTypes.Conv(dwOffset, "uint")
        gen_dwSize = CEcWrapperTypes.Conv(dwSize, "uint")
        gen_bInput = CEcWrapperTypes.Conv(bInput, "bool")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqConfigAddDataRange(self.m_dwMasterInstanceId, gen_dwOffset, gen_dwSize, gen_bInput))
        return gen_dwRetVal

    def DaqConfigRegisterAppVariable(self, pszName): # ret: ECError
        """
        Register application variable to logging.

        Args:
            pszName: 

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_pszName = CEcWrapperTypes.Conv(pszName, "string")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqConfigRegisterAppVariable(self.m_dwMasterInstanceId, gen_pszName))
        return gen_dwRetVal

    def DaqConfigLoad(self, pszFile): # ret: ECError
        """
        Loads configuration from file.

        Args:
            pszFile: Path to config file

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_pszFile = CEcWrapperTypes.Conv(pszFile, "string")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqConfigLoad(self.m_dwMasterInstanceId, gen_pszFile))
        return gen_dwRetVal

    def DaqConfigLoadFromMemory(self, pbyData, wLen): # ret: ECError
        """
        Loads configuration from memory.

        Args:
            pbyData: Data pointer
            wLen: Data length

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_pbyData = CEcWrapperTypes.Conv(pbyData, "byte[]")
        gen_wLen = CEcWrapperTypes.Conv(wLen, "ushort")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqConfigLoadFromMemory(self.m_dwMasterInstanceId, gen_pbyData, gen_wLen))
        CEcWrapperTypes.Conv_Array(gen_pbyData, pbyData)
        return gen_dwRetVal

    def DaqConfigAddTriggerByValue(self, phTrigger, pszName, pszValue, eOperator, bEnable, bStart, dwDuration, dwCount): # ret: ECError
        """
        Add trigger for starting/stopping logging by comparing a variable with a static value (e.g. variable is greater than 5)

        Args:
            phTrigger: Trigger handle
            pszName: Name of variable
            pszValue: Value of variable
            eOperator: Operator
            bEnable: EC_TRUE: enable trigger, EC_FALSE: disable trigger
            bStart: EC_TRUE: start trigger, EC_FALSE: stop trigger
            dwDuration: Duration in msec (0 = infinite)
            dwCount: Count (0 = infinite)

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_phTrigger = CEcWrapperTypes.Conv(phTrigger, "IntPtr")
        gen_pszName = CEcWrapperTypes.Conv(pszName, "string")
        gen_pszValue = CEcWrapperTypes.Conv(pszValue, "string")
        gen_eOperator = CEcWrapperTypes.Conv(eOperator, "EC_T_DAQ_OPERATOR")
        gen_bEnable = CEcWrapperTypes.Conv(bEnable, "bool")
        gen_bStart = CEcWrapperTypes.Conv(bStart, "bool")
        gen_dwDuration = CEcWrapperTypes.Conv(dwDuration, "uint")
        gen_dwCount = CEcWrapperTypes.Conv(dwCount, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqConfigAddTriggerByValue(self.m_dwMasterInstanceId, ctypes.pointer(gen_phTrigger), gen_pszName, gen_pszValue, gen_eOperator, gen_bEnable, gen_bStart, gen_dwDuration, gen_dwCount))
        return gen_dwRetVal

    def DaqConfigAddTriggerByVariable(self, phTrigger, pszName1, pszName2, eOperator, bEnable, bStart, dwDuration, dwCount): # ret: ECError
        """
        Add trigger for starting/stopping logging by comparing a variable with another variable (e.g. variable 1 is greater than variable 2).

        Args:
            phTrigger: Trigger handle
            pszName1: Name of variable 1
            pszName2: Name of variable 2
            eOperator: Operator
            bEnable: EC_TRUE: enable trigger, EC_FALSE: disable trigger
            bStart: EC_TRUE: start trigger, EC_FALSE: stop trigger
            dwDuration: Duration in msec (0 = infinite)
            dwCount: Count (0 = infinite)

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_phTrigger = CEcWrapperTypes.Conv(phTrigger, "IntPtr")
        gen_pszName1 = CEcWrapperTypes.Conv(pszName1, "string")
        gen_pszName2 = CEcWrapperTypes.Conv(pszName2, "string")
        gen_eOperator = CEcWrapperTypes.Conv(eOperator, "EC_T_DAQ_OPERATOR")
        gen_bEnable = CEcWrapperTypes.Conv(bEnable, "bool")
        gen_bStart = CEcWrapperTypes.Conv(bStart, "bool")
        gen_dwDuration = CEcWrapperTypes.Conv(dwDuration, "uint")
        gen_dwCount = CEcWrapperTypes.Conv(dwCount, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqConfigAddTriggerByVariable(self.m_dwMasterInstanceId, ctypes.pointer(gen_phTrigger), gen_pszName1, gen_pszName2, gen_eOperator, gen_bEnable, gen_bStart, gen_dwDuration, gen_dwCount))
        return gen_dwRetVal

    def DaqTriggerEnable(self, hTrigger): # ret: ECError
        """
        Enable trigger

        Args:
            hTrigger: Trigger handle

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_hTrigger = CEcWrapperTypes.Conv(hTrigger, "IntPtr")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqTriggerEnable(self.m_dwMasterInstanceId, gen_hTrigger))
        return gen_dwRetVal

    def DaqTriggerDisable(self, hTrigger): # ret: ECError
        """
        Disable trigger

        Args:
            hTrigger: Trigger handle

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_hTrigger = CEcWrapperTypes.Conv(hTrigger, "IntPtr")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqTriggerDisable(self.m_dwMasterInstanceId, gen_hTrigger))
        return gen_dwRetVal

    def DaqTriggerGetCount(self, hTrigger, pdwCount): # ret: ECError
        """
        Returns trigger count

        Args:
            hTrigger: Trigger handle
            pdwCount: Trigger count

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_hTrigger = CEcWrapperTypes.Conv(hTrigger, "IntPtr")
        gen_pdwCount = CEcWrapperTypes.Conv(pdwCount, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqTriggerGetCount(self.m_dwMasterInstanceId, gen_hTrigger, ctypes.pointer(gen_pdwCount)))
        return gen_dwRetVal

    def DaqConfigApply(self): # ret: ECError
        """
        Applies the configuration and prepares everything for start/stop logging.

        Args:

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqConfigApply(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def DaqGetStatistic(self, out_pStatistic): # ret: ECError
        """
        Returns recorder statistic.

        Args:
            pStatistic: Statistic pointer

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        pStatistic = DN_EC_T_DAQ_REC_STATISTIC()
        gen_pStatistic = CEcWrapperTypes.Conv(pStatistic, "EC_T_DAQ_REC_STATISTIC")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqGetStatistic(self.m_dwMasterInstanceId, ctypes.pointer(gen_pStatistic)))
        out_pStatistic.value = CEcWrapperTypes.Conv(gen_pStatistic, "DN_EC_T_DAQ_REC_STATISTIC")
        return gen_dwRetVal

    def DaqMemoryGetInfo(self, out_pInfo): # ret: ECError
        """
        Returns recorder memory information.

        Args:
            pInfo: Info pointer

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        pInfo = DN_EC_T_DAQ_MEMORY_INFO()
        gen_pInfo = CEcWrapperTypes.Conv(pInfo, "EC_T_DAQ_MEMORY_INFO")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqMemoryGetInfo(self.m_dwMasterInstanceId, ctypes.pointer(gen_pInfo)))
        out_pInfo.value = CEcWrapperTypes.Conv(gen_pInfo, "DN_EC_T_DAQ_MEMORY_INFO")
        return gen_dwRetVal

    def DaqMemoryGetVariables(self, out_pVariables, dwVariablesCount): # ret: ECError
        """
        Returns recorder memory variables.

        Args:
            pVariables: Variables pointer
            dwVariablesCount: Variables count

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        pVariables = CEcWrapperTypes.CreateStructArray(DN_EC_T_DAQ_MEMORY_VARIABLE(), dwVariablesCount)
        gen_pVariables = CEcWrapperTypes.Conv(pVariables, "EC_T_DAQ_MEMORY_VARIABLE")
        gen_pVariables_bytes = CEcWrapperTypes.ConvStructArrayToBytes(gen_pVariables)
        gen_dwVariablesCount = CEcWrapperTypes.Conv(dwVariablesCount, "uint")
        if dwVariablesCount == 0: return ECError.EC_NOERROR
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqMemoryGetVariables(self.m_dwMasterInstanceId, gen_pVariables_bytes, gen_dwVariablesCount))
        gen_pVariables = CEcWrapperTypes.ConvStructArrayFromBytes(gen_pVariables_bytes, gen_pVariables)
        out_pVariables.value = CEcWrapperTypes.Conv(gen_pVariables, "DN_EC_T_DAQ_MEMORY_VARIABLE")
        return gen_dwRetVal

    def DaqMemoryGetData(self, dwIndex, pbyData, dwDataSize): # ret: ECError
        """
        Returns recorder memory data.

        Args:
            dwIndex: Data index or -1 for all
            pbyData: Data pointer
            dwDataSize: Data size

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_dwIndex = CEcWrapperTypes.Conv(dwIndex, "uint")
        gen_pbyData = CEcWrapperTypes.Conv(pbyData, "byte[]")
        gen_dwDataSize = CEcWrapperTypes.Conv(dwDataSize, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqMemoryGetData(self.m_dwMasterInstanceId, gen_dwIndex, gen_pbyData, gen_dwDataSize))
        CEcWrapperTypes.Conv_Array(gen_pbyData, pbyData)
        return gen_dwRetVal

    def DaqMemoryGetVariableValue(self, pVariable, pbyData, out_pszBuf, dwBufLen): # ret: ECError
        """
        Returns value of recorder variable as string.

        Args:
            pVariable: Variable pointer
            pbyData: Data pointer
            pszBuf: String buffer
            dwBufLen: String buffer length

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_pVariable = CEcWrapperTypes.Conv(pVariable, "EC_T_DAQ_MEMORY_VARIABLE")
        gen_pbyData = CEcWrapperTypes.Conv(pbyData, "byte[]")
        pszBuf = CEcWrapperTypes.CreateCharArray()
        gen_pszBuf = CEcWrapperTypes.Conv(pszBuf, "string")
        gen_pszBuf_bytes = CEcWrapperTypes.ConvCharArrayToBytes(gen_pszBuf, dwBufLen)
        gen_dwBufLen = CEcWrapperTypes.Conv(dwBufLen, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqMemoryGetVariableValue(self.m_dwMasterInstanceId, ctypes.pointer(gen_pVariable), gen_pbyData, gen_pszBuf_bytes, gen_dwBufLen))
        gen_pszBuf = CEcWrapperTypes.ConvCharArrayFromBytes(gen_pszBuf_bytes)
        out_pszBuf.value = CEcWrapperTypes.Conv(gen_pszBuf, "EC_T_CHAR")
        CEcWrapperTypes.Conv_Array(gen_pbyData, pbyData)
        return gen_dwRetVal

    def DaqRecStart(self, dwDuration): # ret: ECError
        """
        Start logging of recorder.

        Args:
            dwDuration: Duration in msec (0 = infinite)

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_dwDuration = CEcWrapperTypes.Conv(dwDuration, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqRecStart(self.m_dwMasterInstanceId, gen_dwDuration))
        return gen_dwRetVal

    def DaqRecStop(self, dwDuration): # ret: ECError
        """
        Stop logging of recorder.

        Args:
            dwDuration: Duration in msec (0 = infinite)

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_dwDuration = CEcWrapperTypes.Conv(dwDuration, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqRecStop(self.m_dwMasterInstanceId, gen_dwDuration))
        return gen_dwRetVal

    def DaqProcessRt(self): # ret: ECError
        """
        Process logging, called within cyclic task.

        Args:

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqProcessRt(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def DaqRecStartRt(self, dwDuration): # ret: ECError
        """
        Start logging of recorder (must be called only from cyclic task).

        Args:
            dwDuration: Duration in msec (0 = infinite)

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_dwDuration = CEcWrapperTypes.Conv(dwDuration, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqRecStartRt(self.m_dwMasterInstanceId, gen_dwDuration))
        return gen_dwRetVal

    def DaqRecStopRt(self, dwDuration): # ret: ECError
        """
        Stop logging of recorder (must be called only from cyclic task).

        Args:
            dwDuration: Duration in msec (0 = infinite)

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_dwDuration = CEcWrapperTypes.Conv(dwDuration, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqRecStopRt(self.m_dwMasterInstanceId, gen_dwDuration))
        return gen_dwRetVal

    def DaqReaderOpen(self, out_pInfo): # ret: ECError
        """
        Open DAQ reader.

        Args:
            pInfo: Info pointer

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        pInfo = DN_EC_T_DAQ_READER_INFO()
        gen_pInfo = CEcWrapperTypes.Conv(pInfo, "EC_T_DAQ_READER_INFO")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqReaderOpen(self.m_dwMasterInstanceId, ctypes.pointer(gen_pInfo)))
        out_pInfo.value = CEcWrapperTypes.Conv(gen_pInfo, "DN_EC_T_DAQ_READER_INFO")
        return gen_dwRetVal

    def DaqReaderClose(self): # ret: ECError
        """
        Close DAQ reader file.

        Args:

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqReaderClose(self.m_dwMasterInstanceId))
        return gen_dwRetVal

    def DaqReaderGetGroup(self, dwIndex, out_pGroup): # ret: ECError
        """
        Returns DAQ reader group.

        Args:
            dwIndex: Group index
            pGroup: Group pointer

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_dwIndex = CEcWrapperTypes.Conv(dwIndex, "uint")
        pGroup = DN_EC_T_DAQ_READER_GROUP()
        gen_pGroup = CEcWrapperTypes.Conv(pGroup, "EC_T_DAQ_READER_GROUP")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqReaderGetGroup(self.m_dwMasterInstanceId, gen_dwIndex, ctypes.pointer(gen_pGroup)))
        out_pGroup.value = CEcWrapperTypes.Conv(gen_pGroup, "DN_EC_T_DAQ_READER_GROUP")
        return gen_dwRetVal

    def DaqReaderGetVariable(self, dwIndex, out_pVariable): # ret: ECError
        """
        Returns DAQ reader variable.

        Args:
            dwIndex: Variable index
            pVariable: Variable pointer

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_dwIndex = CEcWrapperTypes.Conv(dwIndex, "uint")
        pVariable = DN_EC_T_DAQ_READER_VARIABLE()
        gen_pVariable = CEcWrapperTypes.Conv(pVariable, "EC_T_DAQ_READER_VARIABLE")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqReaderGetVariable(self.m_dwMasterInstanceId, gen_dwIndex, ctypes.pointer(gen_pVariable)))
        out_pVariable.value = CEcWrapperTypes.Conv(gen_pVariable, "DN_EC_T_DAQ_READER_VARIABLE")
        return gen_dwRetVal

    def DaqReaderGetRecordData(self, dwIndex, pbyRecordData, dwRecordDataSize): # ret: ECError
        """
        Returns DAQ reader record data.

        Args:
            dwIndex: Record data index
            pbyRecordData: Record data pointer
            dwRecordDataSize: Record data size

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_dwIndex = CEcWrapperTypes.Conv(dwIndex, "uint")
        gen_pbyRecordData = CEcWrapperTypes.Conv(pbyRecordData, "byte[]")
        gen_dwRecordDataSize = CEcWrapperTypes.Conv(dwRecordDataSize, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqReaderGetRecordData(self.m_dwMasterInstanceId, gen_dwIndex, gen_pbyRecordData, gen_dwRecordDataSize))
        CEcWrapperTypes.Conv_Array(gen_pbyRecordData, pbyRecordData)
        return gen_dwRetVal

    def DaqReaderGetVariableValue(self, pVariable, pbyData, out_pszBuf, dwBufLen): # ret: ECError
        """
        Returns value of DAQ reader variable as string.

        Args:
            pVariable: Variable pointer
            pbyData: Data pointer
            pszBuf: String buffer
            dwBufLen: String buffer length

        Returns:
            EC_E_NOERROR on success, error code otherwise.
        """
        gen_pVariable = CEcWrapperTypes.Conv(pVariable, "EC_T_DAQ_READER_VARIABLE")
        gen_pbyData = CEcWrapperTypes.Conv(pbyData, "byte[]")
        pszBuf = CEcWrapperTypes.CreateCharArray()
        gen_pszBuf = CEcWrapperTypes.Conv(pszBuf, "string")
        gen_pszBuf_bytes = CEcWrapperTypes.ConvCharArrayToBytes(gen_pszBuf, dwBufLen)
        gen_dwBufLen = CEcWrapperTypes.Conv(dwBufLen, "uint")
        gen_dwRetVal = self.ConvResAsError(CEcWrapper.Get().ecwDaqReaderGetVariableValue(self.m_dwMasterInstanceId, ctypes.pointer(gen_pVariable), gen_pbyData, gen_pszBuf_bytes, gen_dwBufLen))
        gen_pszBuf = CEcWrapperTypes.ConvCharArrayFromBytes(gen_pszBuf_bytes)
        out_pszBuf.value = CEcWrapperTypes.Conv(gen_pszBuf, "EC_T_CHAR")
        CEcWrapperTypes.Conv_Array(gen_pbyData, pbyData)
        return gen_dwRetVal

    #// @CODEGENERATOR_IMPL_END@


class CEcWrapperPythonRefParam:
    """
    'ref' Parameter
    """
    def __init__(self, value):
        self.value = value


class CEcWrapperPythonOutParam:
    """
    'out' Parameter
    """
    def __init__(self):
        self.value =  None


class CEcWrapperPythonException(Exception):
    """
    EcWrapper Python Exception

    EnableExceptionHandling = true, functions to throw an exception instead of returning an error code
    """
    def __init__(self, code, text, message = None):
        message = message if message else "{} (0x{:08X})".format(text, code)
        super(CEcWrapperPythonException, self).__init__(message)
        self.code = code
        self.text = text
        self.message = message
        return

    def __str__(self):
        return self.message


class CEcWrapperPythonEx(CEcWrapperPython):
    """
    Extended EcWrapper for Python (combines EcWrapper with RAS server and Mailbox gateway server)
    """
    def __init__(self):
        super().__init__()
        self._rasServer = None
        self._mbxGatewayServer = None


    def InitWrapper(self, dwMasterInstanceId, oInitMaster, oRasParms, oMbxGatewayParms, bUseAuxClock, bSimulator = False, oSimulatorParms = None, oMonitorParms = None, oDaqParms = None, oDaqReaderParms = None):
        """
        Initializes the EtherCAT wrapper

        Args:
            dwMasterInstanceId: Master instance
            oInitMaster: Pointer to parameter definitions
            oRasParms: Pointer to ras parameter definitions (null = Ras server will not be started)
            oMbxGatewayParms: Pointer to mailbox gateway parameter definitions
            bUseAuxClock: True, to use the soft real-time timer
            bSimulator: True, to activate simulator mode
            oSimulatorParms: Pointer to simulator HiL parameter definitions
            oMonitorParms: Pointer to monitor parameter definitions
            oDaqParms: Pointer to DAQ parameter definitions
            oDaqReaderParms: Pointer to DAQ reader parameter definitions

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        if bSimulator and oMbxGatewayParms is not None:
            return ECError.EC_INVALIDPARM
        if oMonitorParms is not None and oMbxGatewayParms is not None:
            return ECError.EC_INVALIDPARM

        if oInitMaster is not None:
            eRetVal = ECError.EC_ERROR

            if oRasParms is not None:
                oParamsMasterRas = DN_EC_T_INIT_PARMS_MASTER_RAS_SERVER()
                oParamsMasterRas.oRas = oRasParms
                self._rasServer = CEcWrapperPython()
                self.RegisterNotificationHandlers(self._rasServer)
                eRetVal = self._rasServer.InitInstance(oParamsMasterRas)
                if eRetVal != ECError.EC_NOERROR:
                    self.UnregisterNotificationHandlers(self._rasServer)
                    return eRetVal

            oParams = DN_EC_T_INIT_PARMS_MASTER()
            oParams.dwMasterInstanceId = dwMasterInstanceId
            oParams.oMaster = oInitMaster
            oParams.bUseAuxClock = bUseAuxClock
            eRetVal = self.InitInstance(oParams)
            if eRetVal != ECError.EC_NOERROR:
                if self._rasServer is not None:
                    self._rasServer.DeinitInstance()
                    self.UnregisterNotificationHandlers(self._rasServer)
                    self._rasServer = None

                return eRetVal

            if oMbxGatewayParms is not None:
                oParamsMbxGateway = DN_EC_T_INIT_PARMS_MBXGATEWAY_SERVER()
                oParamsMbxGateway.dwMasterInstanceId = dwMasterInstanceId
                oParamsMbxGateway.oMbxGateway = oMbxGatewayParms
                self._mbxGatewayServer = CEcWrapperPython()
                self.RegisterNotificationHandlers(self._mbxGatewayServer)
                eRetVal = self._mbxGatewayServer.InitInstance(oParamsMbxGateway)
                if eRetVal != ECError.EC_NOERROR:
                    self.UnregisterNotificationHandlers(self._mbxGatewayServer)
                    if self._rasServer is not None:
                        self._rasServer.DeinitInstance()
                        self.UnregisterNotificationHandlers(self._rasServer)
                        self._rasServer = None

                    self.DeinitInstance()
                    return eRetVal

            return ECError.EC_NOERROR

        if bSimulator:
            eRetVal = ECError.EC_ERROR

            if oRasParms is not None:
                oParamsSimulatorRas = DN_EC_T_INIT_PARMS_SIMULATOR_RAS_SERVER()
                oParamsSimulatorRas.oRas = oRasParms
                self._rasServer = CEcWrapperPython()
                self.RegisterNotificationHandlers(self._rasServer)
                eRetVal = self._rasServer.InitInstance(oParamsSimulatorRas)
                if eRetVal != ECError.EC_NOERROR:
                    self.UnregisterNotificationHandlers(self._rasServer)
                    return eRetVal

            oParams = DN_EC_T_INIT_PARMS_SIMULATOR()
            oParams.dwSimulatorInstanceId = dwMasterInstanceId
            oParams.oSimulator = oSimulatorParms
            eRetVal = self.InitInstance(oParams)
            if eRetVal != ECError.EC_NOERROR:
                if self._rasServer is not None:
                    self._rasServer.DeinitInstance()
                    self.UnregisterNotificationHandlers(self._rasServer)
                    self._rasServer = None

                return eRetVal

            return ECError.EC_NOERROR

        if oMonitorParms is not None:
            eRetVal = ECError.EC_ERROR

            if oRasParms is not None:
                oParamsMonitorRas = DN_EC_T_INIT_PARMS_MONITOR_RAS_SERVER()
                oParamsMonitorRas.oRas = oRasParms
                self._rasServer = CEcWrapperPython()
                self.RegisterNotificationHandlers(self._rasServer)
                eRetVal = self._rasServer.InitInstance(oParamsMonitorRas)
                if eRetVal != ECError.EC_NOERROR:
                    self.UnregisterNotificationHandlers(self._rasServer)
                    return eRetVal

            oParams = DN_EC_T_INIT_PARMS_MONITOR()
            oParams.dwMonitorInstanceId = dwMasterInstanceId
            oParams.oMonitor = oMonitorParms
            eRetVal = self.InitInstance(oParams)
            if eRetVal != ECError.EC_NOERROR:
                if self._rasServer is not None:
                    self._rasServer.DeinitInstance()
                    self.UnregisterNotificationHandlers(self._rasServer)
                    self._rasServer = None

                return eRetVal

            return ECError.EC_NOERROR

        if oRasParms is not None:
            oParams = DN_EC_T_INIT_PARMS_RAS_CLIENT()
            oParams.dwMasterInstanceId = dwMasterInstanceId
            oParams.oRas = oRasParms
            return self.InitInstance(oParams)

        if oMbxGatewayParms is not None:
            oParams = DN_EC_T_INIT_PARMS_MBXGATEWAY_CLIENT()
            oParams.oMbxGateway = oMbxGatewayParms
            return self.InitInstance(oParams)

        if oDaqParms is not None:
            oParams = DN_EC_T_INIT_PARMS_DAQ()
            oParams.oDaq = oDaqParms
            return self.InitInstance(oParams)

        if oDaqReaderParms is not None:
            oParams = DN_EC_T_INIT_PARMS_DAQ_READER()
            oParams.oDaqReader = oDaqReaderParms
            return self.InitInstance(oParams)

        return ECError.EC_NOTSUPPORTED


    def DeinitWrapper(self):
        """
        Terminates the EtherCAT wrapper and releases all resources

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        eRetVal = self.DeinitInstance()
        if eRetVal != ECError.EC_NOERROR:
            return eRetVal

        if self._rasServer is not None:
            eRetVal = self._rasServer.DeinitInstance()
            if eRetVal != ECError.EC_NOERROR:
                return eRetVal

            self.UnregisterNotificationHandlers(self._rasServer)
            self._rasServer = None

        if self._mbxGatewayServer is not None:
            eRetVal = self._mbxGatewayServer.DeinitInstance()
            if eRetVal != ECError.EC_NOERROR:
                return eRetVal

            self.UnregisterNotificationHandlers(self._mbxGatewayServer)
            self._mbxGatewayServer = None

        return eRetVal


    def RegisterNotificationHandlers(self, instance):
        instance.ecNotificationHandlerId = instance.AddNotificationHandler("onMaster", self.EcNotificationHandler)
        instance.rasNotificationHandlerId = instance.AddNotificationHandler("onRas", self.RasNotificationHandler)
        instance.dbgMsgNotificationHandlerId = instance.AddNotificationHandler("onDbgMsg", self.DbgMsgNotificationHandler)

    def UnregisterNotificationHandlers(self, instance):
        if instance.ecNotificationHandlerId is not None and instance.ecNotificationHandlerId != -1:
            instance.RemoveNotificationHandler(instance.ecNotificationHandlerId)
            instance.ecNotificationHandlerId = -1
        if instance.rasNotificationHandlerId is not None and instance.rasNotificationHandlerId != -1:
            instance.RemoveNotificationHandler(instance.rasNotificationHandlerId)
            instance.rasNotificationHandlerId = -1
        if instance.dbgMsgNotificationHandlerId is not None and instance.dbgMsgNotificationHandlerId != -1:
            instance.RemoveNotificationHandler(instance.dbgMsgNotificationHandlerId)
            instance.dbgMsgNotificationHandlerId = -1

    def EcNotificationHandler(self, type_, code, data, _errMsgs):
        if self.HasNotificationHandler("onMaster"):
            self.OnNotificationHandler("onMaster", type_, code, data, _errMsgs)

    def RasNotificationHandler(self, type_, code, data, _errMsgs):
        if self.HasNotificationHandler("onRas"):
            self.OnNotificationHandler("onRas", type_, code, data, _errMsgs)

    def DbgMsgNotificationHandler(self, type_, severity, msg):
        if self.HasNotificationHandler("onDbgMsg"):
            self.OnNotificationHandler("onDbgMsg", type_, severity, msg)

class CEcMasterPython(CEcWrapperPython):
    """
    CEcMaster for Python
    """

    @staticmethod
    def Open(oParms):
        """
        Initialize EcMaster and return instance

        Args:
            oParms: Parameter definitions

        Returns:
            Instance on success, otherwise Null.
        """
        oObj = CEcMasterPython()
        eRes = oObj.DoOpen(oParms)
        if eRes != ECError.EC_NOERROR:
            return None
        return oObj


    def DoOpen(self, oParms):
        """
        Initialize EcMaster

        Args:
            oParms: Parameter definitions

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        return self.InitInstance(oParms)


    def Close(self):
        """
        Deinitialize EcMaster

        Returns:
            ECError: EC_E_NOERROR on success, otherwise an error code.
        """
        return self.DeinitInstance()

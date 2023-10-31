#/*-----------------------------------------------------------------------------
# * EcWrapper.py
# * Copyright                acontis technologies GmbH, Ravensburg, Germany
# * Description              EC-Wrapper CPython Interface (internal)
# *---------------------------------------------------------------------------*/
# pylint: disable=unused-wildcard-import, wildcard-import
import ctypes
import platform
import collections
from EcWrapperTypes import *

class CEcWrapper:
    _instance = None
    _installDir = ""

    @staticmethod
    def GetEcWrapperName():
        name = platform.system()
        if name == "Windows":
            return "EcWrapper.dll"
        return "libEcWrapper.so"

    @classmethod
    def IsInitialized(cls):
        if cls._instance is None:
            return False
        return True

    @classmethod
    def Get(cls):
        if cls._instance is None:
            # fix: TapEdInitParams undefined
            RTLD_LAZY = 0x00001 # https://code.woboq.org/userspace/glibc/bits/dlfcn.h.html
            so_file = cls._installDir + cls.GetEcWrapperName()
            #ctypes.set_conversion_mode('utf-8', 'strict')
            cls._instance = ctypes.CDLL(so_file, mode = RTLD_LAZY)
            for function in CEcWrapperInterface:
                try:
                    func = getattr(cls._instance, function.name)
                    if function.args:
                        func.argtypes = function.args
                    func.restype = function.ret
                except AttributeError as error:
                    print("CEcWrapper.Get: ", str(error))
        return cls._instance

# General functions

    @staticmethod
    def ecSetInstallDir(pszInstallDir):
        CEcWrapper._installDir = pszInstallDir
        CEcWrapper.Get().ecwSetInstallDir(pszInstallDir.encode('utf8'))

# Generated functions

CFunction = collections.namedtuple('CFunction', 'name ret args')

# argtypes are not set here, since this only restricts how objects
# can be passed. e.g. ctypes.byref can not be used when a
# ctypes.POINTER(<type>) is expected
CEcWrapperInterface = [
    CFunction(name="ecwGetApiVer", ret=ctypes.c_uint, args=None),
    CFunction(name="ecwSetInstallDir", ret=None, args=[ctypes.c_char_p]),
    CFunction(name="ecwInit", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.POINTER(ctypes.c_uint)]),
    CFunction(name="ecwInit2", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_bool, ctypes.POINTER(ctypes.c_uint)]),
    CFunction(name="ecwDone", ret=None, args=[ctypes.c_uint]),
    CFunction(name="ecwGetMasterByID", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwOsDbgMsg", ret=None, args=[ctypes.c_char_p]),
    CFunction(name="ecwOsQueryMsecCount", ret=ctypes.c_uint, args=None),
    CFunction(name="ecwGetText2", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_STRING_HLP)]),
    CFunction(name="ecwOsAuxClkInit", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p]),
    CFunction(name="ecwOsAuxClkDeinit", ret=ctypes.c_uint, args=None),
    CFunction(name="ecwOsCreateEvent", ret=ctypes.c_void_p, args=None),
    CFunction(name="ecwOsDeleteEvent", ret=None, args=[ctypes.c_void_p]),
    CFunction(name="ecwOsWaitForEvent", ret=ctypes.c_uint, args=[ctypes.c_void_p, ctypes.c_uint]),
    CFunction(name="ecwOsDeleteThreadHandle", ret=None, args=[ctypes.c_void_p]),
    CFunction(name="ecwEnablePerformanceMeasuring", ret=None, args=[ctypes.c_void_p]),
    CFunction(name="ecwEnableTranslation", ret=None, args=[ctypes.c_void_p]),
    CFunction(name="ecwGetDefaultValue", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwIoControl", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(SDN_EC_T_IOCTLOPARMS)]),
    CFunction(name="ecwExecDefaultJob", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint)]),
    CFunction(name="ecwEoeInstallEndpoint", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_void_p]),
    CFunction(name="ecwEoeUninstallEndpoint", ret=ctypes.c_uint, args=[ctypes.c_void_p]),
    CFunction(name="ecwEoeTriggerTxEvent", ret=ctypes.c_uint, args=[ctypes.c_void_p]),
    CFunction(name="ecwESCTypeText", ret=ctypes.c_uint, args=[ctypes.c_ubyte, ctypes.c_bool, ctypes.POINTER(SDN_EC_STRING_HLP)]),
    CFunction(name="ecwSlaveVendorText", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_STRING_HLP)]),
    CFunction(name="ecwSlaveProdCodeText", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(SDN_EC_STRING_HLP)]),
    CFunction(name="ecwRestartScanBus", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_bool, ctypes.c_bool]),
    CFunction(name="ecwSetBusCnfReadProp", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwIoControl2", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint)]),
    CFunction(name="ecwGetBusScanSlaveInfoDesc", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_short, ctypes.POINTER(SDN_EC_T_SB_SLAVEINFO_DESC)]),
    CFunction(name="ecwGetScanBusStatus", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_SB_STATUS_NTFY_DESC)]),
    CFunction(name="ecwCoeGetObjectDescReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_ushort, ctypes.POINTER(SDN_EC_T_COE_OBDESC), ctypes.c_uint]),
    CFunction(name="ecwCoeGetEntryDescReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_ushort, ctypes.c_ubyte, ctypes.c_ubyte, ctypes.POINTER(SDN_EC_T_COE_ENTRYDESC), ctypes.c_uint]),
    CFunction(name="ecwCoeGetODList2", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint), ctypes.c_uint]),
    CFunction(name="ecwMbxTferCreate2", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p]),
    CFunction(name="ecwMbxTferWait", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p]),
    CFunction(name="ecwMbxTferCopyTo", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint]),
    CFunction(name="ecwMbxTferCopyFrom", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint)]),
    CFunction(name="ecwNotifyApp2", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.c_ushort, ctypes.c_void_p, ctypes.c_ushort, ctypes.POINTER(ctypes.c_uint)]),
    CFunction(name="ecwGetNotificationData", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint)]),
    CFunction(name="ecwSetNotificationData", ret=ctypes.c_uint, args=[ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint]),
    CFunction(name="ecwParseNotificationType", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwParseNotificationErrMsg", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p]),
    CFunction(name="ecwParseRasNotificationErrMsg", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p]),
    CFunction(name="ecwGetNotificationErrMsg", ret=ctypes.c_uint, args=[ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(SDN_EC_STRING_HLP)]),
    CFunction(name="ecwFreeNotificationErrMsg", ret=None, args=[ctypes.c_void_p]),
    CFunction(name="ecwParseMbxTransferData", ret=None, args=[ctypes.c_void_p, ctypes.POINTER(SDN_EC_T_MBXTFER), ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint), ctypes.c_void_p]),
    CFunction(name="ecwOsWaitForEvent2", ret=ctypes.c_bool, args=[ctypes.c_void_p]),
    CFunction(name="ecwGetMasterParms2", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_INIT_MASTER_PARMS)]),
    CFunction(name="ecwSetMasterParms2", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_INIT_MASTER_PARMS)]),
    CFunction(name="ecwForceSlvStatCollection", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwSetAllSlavesMustReachState", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool]),
    CFunction(name="ecwSetAdditionalVariablesForSpecificDataTypesEnabled", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool]),
    CFunction(name="ecwEnableNotification", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_bool]),
    CFunction(name="ecwGetSlvStatistics", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(SDN_EC_T_SLVSTATISTICS_DESC)]),
    CFunction(name="ecwGetCyclicConfigInfo", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(SDN_EC_T_CYC_CONFIG_DESC)]),
    CFunction(name="ecwSetScanBusEnable", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwSetScanBusStatus", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwReadIdentifyObj", ret=None, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwGetSlaveInfoEx", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_SB_SLAVEINFO_REQ_DESC), ctypes.POINTER(SDN_EC_T_SB_SLAVEINFO_RES_DESC)]),

#//////////////////////////////////////////////////////////////////////////
#// RasServer functions
    CFunction(name="ecwRasSrvStart", ret=ctypes.c_uint, args=[ctypes.POINTER(SDN_ATEMRAS_T_SRVPARMS), ctypes.c_void_p, ctypes.c_bool]),
    CFunction(name="ecwRasSrvStop", ret=ctypes.c_uint, args=[ctypes.c_void_p, ctypes.c_uint]),

#//////////////////////////////////////////////////////////////////////////
#// RasClient functions
    CFunction(name="ecwRasClntGetVersion", ret=ctypes.c_uint, args=None),
    CFunction(name="ecwRasClntInit", ret=ctypes.c_uint, args=[ctypes.POINTER(SDN_ATEMRAS_T_CLNTPARMS), ctypes.c_bool]),
    CFunction(name="ecwRasClntClose", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwRasClntAddConnection", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_ATEMRAS_T_CLNTCONDESC), ctypes.c_void_p]),
    CFunction(name="ecwRasClntRemoveConnection", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint]),
    CFunction(name="ecwRasGetConnectionInfo", ret=ctypes.c_uint, args=[ctypes.c_void_p, ctypes.POINTER(SDN_EC_T_RAS_CONNECTION_INFO)]),

#//////////////////////////////////////////////////////////////////////////
#// MbxGateway functions
    CFunction(name="ecwMbxGatewayClntGetVersion", ret=ctypes.c_uint, args=None),
    CFunction(name="ecwMbxGatewayClntInit", ret=ctypes.c_uint, args=[ctypes.POINTER(SDN_EC_T_MBX_GATEWAY_CLNT_PARMS)]),
    CFunction(name="ecwMbxGatewayClntDeinit", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwMbxGatewayClntAddConnection", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_MBX_GATEWAY_CLNT_CONDESC)]),
    CFunction(name="ecwMbxGatewayClntRemoveConnection", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwMbxGatewayClntCoeSdoDownload", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_ubyte, ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwMbxGatewayClntCoeSdoUpload", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_ubyte, ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint), ctypes.c_uint, ctypes.c_uint]),

#//////////////////////////////////////////////////////////////////////////
#// MbxGatewaySrv functions
    CFunction(name="ecwMbxGatewaySrvGetVersion", ret=ctypes.c_uint, args=None),
    CFunction(name="ecwMbxGatewaySrvStart", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_MBX_GATEWAY_SRV_PARMS), ctypes.c_void_p]),
    CFunction(name="ecwMbxGatewaySrvStop", ret=ctypes.c_uint, args=[ctypes.c_void_p, ctypes.c_uint]),

#//////////////////////////////////////////////////////////////////////////
#// Simulator functions
    CFunction(name="ecwSimulatorInit", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_uint)]),
    CFunction(name="ecwSimulatorDeinit", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwInitSimulator2", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_SIMULATOR_INIT_PARMS)]),
    CFunction(name="ecwDeinitSimulator", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwConnectPorts2", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort, ctypes.c_ubyte, ctypes.c_ushort, ctypes.c_ubyte]),

#//////////////////////////////////////////////////////////////////////////
#// Simulator RasServer functions
    CFunction(name="ecwSimulatorRasSrvStart", ret=ctypes.c_uint, args=[ctypes.POINTER(SDN_ATEMRAS_T_SRVPARMS), ctypes.c_void_p, ctypes.c_bool]),
    CFunction(name="ecwSimulatorRasSrvStop", ret=ctypes.c_uint, args=[ctypes.c_void_p, ctypes.c_uint]),

#//////////////////////////////////////////////////////////////////////////
#// Monitor functions
    CFunction(name="ecwMonitorInit", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_uint)]),
    CFunction(name="ecwMonitorDeinit", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwInitMonitor2", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_MONITOR_INIT_PARMS)]),
    CFunction(name="ecwDeinitMonitor", ret=ctypes.c_uint, args=[ctypes.c_uint]),

#//////////////////////////////////////////////////////////////////////////
#// Monitor RasServer functions
    CFunction(name="ecwMonitorRasSrvStart", ret=ctypes.c_uint, args=[ctypes.POINTER(SDN_ATEMRAS_T_SRVPARMS), ctypes.c_void_p, ctypes.c_bool]),
    CFunction(name="ecwMonitorRasSrvStop", ret=ctypes.c_uint, args=[ctypes.c_void_p, ctypes.c_uint]),

#//////////////////////////////////////////////////////////////////////////
#// Daq functions
    CFunction(name="ecwDaqInit", ret=ctypes.c_uint, args=[ctypes.POINTER(ctypes.c_uint)]),
    CFunction(name="ecwDaqDeinit", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwDaqRecCreate2", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_DAQ_INIT_PARMS)]),
    CFunction(name="ecwDaqRecDelete", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwDaqReaderCreate2", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_DAQ_READER_PARMS)]),
    CFunction(name="ecwDaqReaderDelete", ret=ctypes.c_uint, args=[ctypes.c_uint]),

#//////////////////////////////////////////////////////////////////////////
#// Performance Measurement functions (deprecated)
    CFunction(name="ecwPerfMeasInit", ret=None, args=[ctypes.c_void_p, ctypes.c_uint64, ctypes.c_uint]),
    CFunction(name="ecwPerfMeasDeinit", ret=None, args=[ctypes.c_void_p]),
    CFunction(name="ecwPerfMeasEnable", ret=None, args=[ctypes.c_void_p]),
    CFunction(name="ecwPerfMeasStart", ret=None, args=[ctypes.c_void_p, ctypes.c_uint]),
    CFunction(name="ecwPerfMeasEnd", ret=None, args=[ctypes.c_void_p, ctypes.c_uint]),
    CFunction(name="ecwPerfMeasReset", ret=None, args=[ctypes.c_void_p, ctypes.c_uint]),
    CFunction(name="ecwPerfMeasDisable", ret=None, args=[ctypes.c_void_p]),
    CFunction(name="ecwPerfMeasShow", ret=None, args=[ctypes.c_void_p, ctypes.c_uint, ctypes.c_void_p]),
    CFunction(name="ecwPerfMeasSetIrqCtlEnabled", ret=None, args=[ctypes.c_bool]),

#//////////////////////////////////////////////////////////////////////////
#// PerfMeas functions
    CFunction(name="ecwPerfMeasAppCreate", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_PERF_MEAS_APP_PARMS), ctypes.POINTER(ctypes.c_void_p)]),
    CFunction(name="ecwPerfMeasAppDelete", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p]),
    CFunction(name="ecwPerfMeasAppStart", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint]),
    CFunction(name="ecwPerfMeasAppEnd", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint]),
    CFunction(name="ecwPerfMeasAppReset", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint]),
    CFunction(name="ecwPerfMeasAppGetRaw", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(SDN_EC_T_PERF_MEAS_VAL), ctypes.POINTER(SDN_EC_T_PERF_MEAS_HISTOGRAM), ctypes.c_uint]),
    CFunction(name="ecwPerfMeasAppGetInfo", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(SDN_EC_T_PERF_MEAS_INFO), ctypes.c_uint]),
    CFunction(name="ecwPerfMeasAppGetNumOf", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint)]),
    CFunction(name="ecwPerfMeasInternalResetByTaskId", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwPerfMeasInternalGetRawByTaskId", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(SDN_EC_T_PERF_MEAS_VAL), ctypes.POINTER(SDN_EC_T_PERF_MEAS_HISTOGRAM), ctypes.c_uint]),
    CFunction(name="ecwPerfMeasInternalGetInfoByTaskId", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(SDN_EC_T_PERF_MEAS_INFO), ctypes.c_uint]),
    CFunction(name="ecwPerfMeasInternalGetNumOfByTaskId", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint)]),

#//////////////////////////////////////////////////////////////////////////
#// DotNetWrapper functions
#//
    CFunction(name="ecwBitCopy", ret=None, args=[ctypes.c_void_p, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwIntPtrAdd", ret=ctypes.c_void_p, args=[ctypes.c_void_p, ctypes.c_uint]),
    CFunction(name="ecwRegisterClient2", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_REGISTERPARMS), ctypes.POINTER(SDN_EC_T_REGISTERRESULTS)]),
    CFunction(name="ecwUnregisterClient2", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwInitMaster2", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_INIT_MASTER_PARMS)]),
    CFunction(name="ecwDeinitMaster", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwReadSlaveEEPRom", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint), ctypes.c_uint]),
    CFunction(name="ecwWriteSlaveEEPRom", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwGetObjectByIdx", ret=ctypes.c_uint, args=[ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p]),
    CFunction(name="ecwFreeObject", ret=ctypes.c_uint, args=[ctypes.c_void_p]),
    CFunction(name="ecwSdoUploadMasterOd", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint]),

#//////////////////////////////////////////////////////////////////////////
#// Generated functions

#// @CODEGENERATOR_IMPL_BEGIN@
    CFunction(name="ecwSetMasterRedStateReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool]),
    CFunction(name="ecwGetMasterRedState", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_bool)]),
    CFunction(name="ecwGetMasterRedProcessImageInputPtr", ret=ctypes.c_void_p, args=[ctypes.c_uint]),
    CFunction(name="ecwGetMasterRedProcessImageOutputPtr", ret=ctypes.c_void_p, args=[ctypes.c_uint]),
    CFunction(name="ecwScanBus", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwRescueScan", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwGetMasterInfo", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_MASTER_INFO)]),
    CFunction(name="ecwConfigureMaster", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint]),
    CFunction(name="ecwConfigLoad", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint]),
    CFunction(name="ecwConfigExcludeSlave", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort]),
    CFunction(name="ecwConfigIncludeSlave", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort]),
    CFunction(name="ecwConfigSetPreviousPort", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_ushort]),
    CFunction(name="ecwConfigAddJunctionRedundancyConnection", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_ushort]),
    CFunction(name="ecwConfigApply", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwConfigExtend", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_uint]),
    CFunction(name="ecwIsConfigured", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_bool)]),
    CFunction(name="ecwSetMasterState", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwGetMasterState", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwGetMasterStateEx", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_ushort), ctypes.POINTER(ctypes.c_ushort)]),
    CFunction(name="ecwStart", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwStop", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwGetSlaveId", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort]),
    CFunction(name="ecwGetSlaveFixedAddr", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(ctypes.c_ushort)]),
    CFunction(name="ecwGetSlaveIdAtPosition", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort]),
    CFunction(name="ecwGetSlaveProp", ret=ctypes.c_bool, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(SDN_EC_T_SLAVE_PROP)]),
    CFunction(name="ecwGetSlavePortState", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(ctypes.c_ushort)]),
    CFunction(name="ecwGetSlaveState", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(ctypes.c_ushort), ctypes.POINTER(ctypes.c_ushort)]),
    CFunction(name="ecwSetSlaveState", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_ushort, ctypes.c_uint]),
    CFunction(name="ecwTferSingleRawCmd", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ubyte, ctypes.c_uint, ctypes.c_void_p, ctypes.c_ushort, ctypes.c_uint]),
    CFunction(name="ecwReadSlaveRegister", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_void_p, ctypes.c_ushort, ctypes.c_uint]),
    CFunction(name="ecwReadSlaveRegisterReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_void_p, ctypes.c_ushort]),
    CFunction(name="ecwWriteSlaveRegister", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_void_p, ctypes.c_ushort, ctypes.c_uint]),
    CFunction(name="ecwWriteSlaveRegisterReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_void_p, ctypes.c_ushort]),
    CFunction(name="ecwQueueRawCmd", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort, ctypes.c_ubyte, ctypes.c_uint, ctypes.c_void_p, ctypes.c_ushort]),
    CFunction(name="ecwClntQueueRawCmd", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_ushort, ctypes.c_ubyte, ctypes.c_uint, ctypes.c_void_p, ctypes.c_ushort]),
    CFunction(name="ecwGetNumConfiguredSlaves", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwMbxTferAbort", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p]),
    CFunction(name="ecwMbxTferDelete", ret=None, args=[ctypes.c_uint, ctypes.c_void_p]),
    CFunction(name="ecwClntSendRawMbx", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwCoeSdoDownloadReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_ushort, ctypes.c_ubyte, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwCoeSdoDownload", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_ushort, ctypes.c_ubyte, ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwCoeSdoUploadReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_ushort, ctypes.c_ubyte, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwCoeSdoUpload", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_ushort, ctypes.c_ubyte, ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint), ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwCoeGetODList", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwCoeGetObjectDesc", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_ushort, ctypes.c_uint]),
    CFunction(name="ecwCoeGetEntryDesc", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_ushort, ctypes.c_ubyte, ctypes.c_ubyte, ctypes.c_uint]),
    CFunction(name="ecwCoeRxPdoTfer", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwCoeProfileGetChannelInfo", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_uint, SDN_EC_T_PROFILE_CHANNEL_INFO]),
    CFunction(name="ecwFoeFileUpload", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_char_p, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint), ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwFoeFileDownload", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_char_p, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwFoeUploadReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_char_p, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwFoeSegmentedUploadReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_char_p, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwFoeDownloadReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_char_p, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwFoeSegmentedDownloadReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_char_p, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwSoeWrite", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_ubyte, ctypes.c_void_p, ctypes.c_ushort, ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwSoeRead", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_ubyte, ctypes.c_void_p, ctypes.c_ushort, ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint), ctypes.c_uint]),
    CFunction(name="ecwSoeAbortProcCmd", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_ubyte, ctypes.c_void_p, ctypes.c_ushort, ctypes.c_uint]),
    CFunction(name="ecwSoeWriteReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_ubyte, ctypes.c_void_p, ctypes.c_ushort, ctypes.c_uint]),
    CFunction(name="ecwSoeReadReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_ubyte, ctypes.c_void_p, ctypes.c_ushort, ctypes.c_uint]),
    CFunction(name="ecwAoeGetSlaveNetId", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(SDN_EC_T_AOE_NETID)]),
    CFunction(name="ecwAoeRead", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(SDN_EC_T_AOE_NETID), ctypes.c_ushort, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint), ctypes.c_uint]),
    CFunction(name="ecwAoeReadReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, SDN_EC_T_AOE_NETID, ctypes.c_ushort, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwAoeWrite", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(SDN_EC_T_AOE_NETID), ctypes.c_ushort, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint), ctypes.c_uint]),
    CFunction(name="ecwAoeWriteReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, SDN_EC_T_AOE_NETID, ctypes.c_ushort, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwAoeReadWrite", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(SDN_EC_T_AOE_NETID), ctypes.c_ushort, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint), ctypes.c_uint]),
    CFunction(name="ecwAoeWriteControl", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, SDN_EC_T_AOE_NETID, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_uint, ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint), ctypes.c_uint]),
    CFunction(name="ecwVoeRead", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint), ctypes.c_uint]),
    CFunction(name="ecwVoeWrite", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwVoeWriteReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwGetProcessData", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwSetProcessData", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwSetProcessDataBits", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwGetProcessDataBits", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwForceProcessDataBits", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_bool, ctypes.c_uint, ctypes.c_ushort, ctypes.c_void_p, ctypes.c_uint]),
    CFunction(name="ecwReleaseProcessDataBits", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_bool, ctypes.c_uint, ctypes.c_ushort, ctypes.c_uint]),
    CFunction(name="ecwReleaseAllProcessDataBits", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwGetNumConnectedSlaves", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwGetNumConnectedSlavesMain", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwGetNumConnectedSlavesRed", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwReadSlaveEEPRomReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_ushort, ctypes.POINTER(ctypes.c_ushort), ctypes.c_uint, ctypes.POINTER(ctypes.c_uint), ctypes.c_uint]),
    CFunction(name="ecwWriteSlaveEEPRomReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_ushort, ctypes.POINTER(ctypes.c_ushort), ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwReloadSlaveEEPRom", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_uint]),
    CFunction(name="ecwReloadSlaveEEPRomReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_uint]),
    CFunction(name="ecwResetSlaveController", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_uint]),
    CFunction(name="ecwAssignSlaveEEPRom", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_bool, ctypes.c_bool, ctypes.c_uint]),
    CFunction(name="ecwAssignSlaveEEPRomReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_bool, ctypes.c_bool, ctypes.c_uint]),
    CFunction(name="ecwActiveSlaveEEPRom", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.POINTER(ctypes.c_bool), ctypes.c_uint]),
    CFunction(name="ecwActiveSlaveEEPRomReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.POINTER(ctypes.c_bool), ctypes.c_uint]),
    CFunction(name="ecwHCAcceptTopoChange", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwHCGetNumGroupMembers", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwHCGetSlaveIdsOfGroup", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint), ctypes.c_uint]),
    CFunction(name="ecwSetSlavePortState", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_ushort, ctypes.c_bool, ctypes.c_bool, ctypes.c_uint]),
    CFunction(name="ecwSetSlavePortStateReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_ushort, ctypes.c_bool, ctypes.c_bool, ctypes.c_uint]),
    CFunction(name="ecwSlaveSerializeMbxTfers", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwSlaveParallelMbxTfers", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwDcEnable", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwDcDisable", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwDcIsEnabled", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_bool)]),
    CFunction(name="ecwDcConfigure", ret=ctypes.c_uint, args=[ctypes.c_uint, SDN_EC_T_DC_CONFIGURE]),
    CFunction(name="ecwDcContDelayCompEnable", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwDcContDelayCompDisable", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwDcmConfigure", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_DCM_CONFIG), ctypes.c_uint]),
    CFunction(name="ecwDcmGetStatus", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]),
    CFunction(name="ecwDcxGetStatus", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int64)]),
    CFunction(name="ecwDcmResetStatus", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwDcmGetBusShiftConfigured", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_bool)]),
    CFunction(name="ecwDcmShowStatus", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwDcmGetAdjust", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_int)]),
    CFunction(name="ecwGetSlaveInfo", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.POINTER(SDN_EC_T_GET_SLAVE_INFO)]),
    CFunction(name="ecwGetCfgSlaveInfo", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.POINTER(SDN_EC_T_CFG_SLAVE_INFO)]),
    CFunction(name="ecwGetCfgSlaveEoeInfo", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.POINTER(SDN_EC_T_CFG_SLAVE_EOE_INFO)]),
    CFunction(name="ecwGetBusSlaveInfo", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.POINTER(SDN_EC_T_BUS_SLAVE_INFO)]),
    CFunction(name="ecwGetSlaveInpVarInfoNumOf", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.POINTER(ctypes.c_ushort)]),
    CFunction(name="ecwGetSlaveOutpVarInfoNumOf", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.POINTER(ctypes.c_ushort)]),
    CFunction(name="ecwGetSlaveInpVarInfo", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_ushort, ctypes.POINTER(ctypes.c_ubyte), ctypes.POINTER(ctypes.c_ushort)]),
    CFunction(name="ecwGetSlaveInpVarInfoEx", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_ushort, ctypes.POINTER(ctypes.c_ubyte), ctypes.POINTER(ctypes.c_ushort)]),
    CFunction(name="ecwGetSlaveOutpVarInfo", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_ushort, ctypes.POINTER(ctypes.c_ubyte), ctypes.POINTER(ctypes.c_ushort)]),
    CFunction(name="ecwGetSlaveOutpVarInfoEx", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_ushort, ctypes.POINTER(ctypes.c_ubyte), ctypes.POINTER(ctypes.c_ushort)]),
    CFunction(name="ecwGetSlaveOutpVarByObjectEx", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_ushort, ctypes.POINTER(SDN_EC_T_PROCESS_VAR_INFO_EX)]),
    CFunction(name="ecwGetSlaveInpVarByObjectEx", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_ushort, ctypes.POINTER(SDN_EC_T_PROCESS_VAR_INFO_EX)]),
    CFunction(name="ecwFindOutpVarByName", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_char_p, ctypes.POINTER(SDN_EC_T_PROCESS_VAR_INFO)]),
    CFunction(name="ecwFindOutpVarByNameEx", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_char_p, ctypes.POINTER(SDN_EC_T_PROCESS_VAR_INFO_EX)]),
    CFunction(name="ecwFindInpVarByName", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_char_p, ctypes.POINTER(SDN_EC_T_PROCESS_VAR_INFO)]),
    CFunction(name="ecwFindInpVarByNameEx", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_char_p, ctypes.POINTER(SDN_EC_T_PROCESS_VAR_INFO_EX)]),
    CFunction(name="ecwEthDbgMsg", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ubyte, ctypes.c_ubyte, ctypes.c_char_p]),
    CFunction(name="ecwBlockNode", ret=ctypes.c_uint, args=[ctypes.c_uint, SDN_EC_T_SB_MISMATCH_DESC, ctypes.c_uint]),
    CFunction(name="ecwOpenBlockedPorts", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwForceTopologyChange", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwIsTopologyChangeDetected", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_bool)]),
    CFunction(name="ecwIsTopologyKnown", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_bool)]),
    CFunction(name="ecwGetBusTime", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_uint64)]),
    CFunction(name="ecwIsSlavePresent", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(ctypes.c_bool)]),
    CFunction(name="ecwPassThroughSrvGetStatus", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwPassThroughSrvStart", ret=ctypes.c_uint, args=[ctypes.c_uint, SDN_EC_T_PTS_SRV_START_PARMS, ctypes.c_uint]),
    CFunction(name="ecwPassThroughSrvStop", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwPassThroughSrvEnable", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwPassThroughSrvDisable", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwAdsAdapterStart", ret=ctypes.c_uint, args=[ctypes.c_uint, SDN_EC_T_ADS_ADAPTER_START_PARMS, ctypes.c_uint]),
    CFunction(name="ecwAdsAdapterStop", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwGetSrcMacAddress", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_ETHERNET_ADDRESS)]),
    CFunction(name="ecwSetLicenseKey", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_char_p]),
    CFunction(name="ecwGetVersion", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_uint)]),
    CFunction(name="ecwTraceDataConfig", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort]),
    CFunction(name="ecwTraceDataGetInfo", ret=ctypes.c_uint, args=[ctypes.c_uint, SDN_EC_T_TRACE_DATA_INFO]),
    CFunction(name="ecwFastModeInit", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwFastSendAllCycFrames", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwFastProcessAllRxFrames", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_bool)]),
    CFunction(name="ecwReadSlaveIdentification", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_ushort, ctypes.POINTER(ctypes.c_ushort), ctypes.c_uint]),
    CFunction(name="ecwReadSlaveIdentificationReq", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_ushort, ctypes.POINTER(ctypes.c_ushort), ctypes.c_uint]),
    CFunction(name="ecwSetSlaveDisabled", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_bool]),
    CFunction(name="ecwSetSlavesDisabled", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_uint, ctypes.c_bool]),
    CFunction(name="ecwSetSlaveDisconnected", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_bool]),
    CFunction(name="ecwSetSlavesDisconnected", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_bool, ctypes.c_ushort, ctypes.c_uint, ctypes.c_bool]),
    CFunction(name="ecwGetMemoryUsage", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint)]),
    CFunction(name="ecwGetSlaveStatistics", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(SDN_EC_T_SLVSTATISTICS_DESC)]),
    CFunction(name="ecwClearSlaveStatistics", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwGetMasterSyncUnitInfoNumOf", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwGetMasterSyncUnitInfo", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort, ctypes.POINTER(SDN_EC_T_MSU_INFO)]),
    CFunction(name="ecwGetMasterDump", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint)]),
    CFunction(name="ecwBadConnectionsDetect", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwSelfTestScan", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_SELFTESTSCAN_PARMS)]),
    CFunction(name="ecwDisconnectPort", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort, ctypes.c_ubyte]),
    CFunction(name="ecwPowerSlave", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort, ctypes.c_bool]),
    CFunction(name="ecwDeleteSlaveCoeObject", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort, ctypes.c_ushort]),
    CFunction(name="ecwClearSlaveCoeObjectDictionary", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort]),
    CFunction(name="ecwResetSlaveCoeObjectDictionary", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort]),
    CFunction(name="ecwConfigureNetwork", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint]),
    CFunction(name="ecwSetErrorAtSlavePort", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort, ctypes.c_ubyte, ctypes.c_bool]),
    CFunction(name="ecwSetErrorGenerationAtSlavePort", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort, ctypes.c_ubyte, ctypes.c_bool, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwResetErrorGenerationAtSlavePorts", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort]),
    CFunction(name="ecwSetLinkDownAtSlavePort", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort, ctypes.c_ubyte, ctypes.c_bool, ctypes.c_uint]),
    CFunction(name="ecwSetLinkDownGenerationAtSlavePort", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort, ctypes.c_ubyte, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwResetLinkDownGenerationAtSlavePorts", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort]),
    CFunction(name="ecwVoeSend", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort, ctypes.c_ushort, ctypes.c_void_p, ctypes.c_uint]),
    CFunction(name="ecwGetMonitorStatus", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_MONITOR_STATUS)]),
    CFunction(name="ecwOpenPacketCapture", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_PACKETCAPTURE_PARMS)]),
    CFunction(name="ecwClosePacketCapture", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwGetPacketCaptureInfo", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_PACKETCAPTURE_INFO)]),
    CFunction(name="ecwStartLivePacketCapture", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_PACKETCAPTURE_PARMS)]),
    CFunction(name="ecwStopLivePacketCapture", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwBacktracePacketCapture", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_PACKETCAPTURE_PARMS)]),
    CFunction(name="ecwDaqConfigAddDataSlave", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_ushort]),
    CFunction(name="ecwDaqConfigAddDataVariable", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_char_p]),
    CFunction(name="ecwDaqConfigAddDataOversamplingVariable", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_char_p, ctypes.c_uint]),
    CFunction(name="ecwDaqConfigAddDataRange", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_bool]),
    CFunction(name="ecwDaqConfigRegisterAppVariable", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_char_p]),
    CFunction(name="ecwDaqConfigLoad", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_char_p]),
    CFunction(name="ecwDaqConfigLoadFromMemory", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.c_ushort]),
    CFunction(name="ecwDaqConfigAddTriggerByValue", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_void_p), ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint, ctypes.c_bool, ctypes.c_bool, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwDaqConfigAddTriggerByVariable", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_void_p), ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint, ctypes.c_bool, ctypes.c_bool, ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwDaqTriggerEnable", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p]),
    CFunction(name="ecwDaqTriggerDisable", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p]),
    CFunction(name="ecwDaqTriggerGetCount", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint)]),
    CFunction(name="ecwDaqConfigApply", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwDaqGetStatistic", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_DAQ_REC_STATISTIC)]),
    CFunction(name="ecwDaqMemoryGetInfo", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_DAQ_MEMORY_INFO)]),
    CFunction(name="ecwDaqMemoryGetVariables", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_uint]),
    CFunction(name="ecwDaqMemoryGetData", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint]),
    CFunction(name="ecwDaqMemoryGetVariableValue", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_DAQ_MEMORY_VARIABLE), ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint]),
    CFunction(name="ecwDaqRecStart", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwDaqRecStop", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwDaqProcessRt", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwDaqRecStartRt", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwDaqRecStopRt", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint]),
    CFunction(name="ecwDaqReaderOpen", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_DAQ_READER_INFO)]),
    CFunction(name="ecwDaqReaderClose", ret=ctypes.c_uint, args=[ctypes.c_uint]),
    CFunction(name="ecwDaqReaderGetGroup", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(SDN_EC_T_DAQ_READER_GROUP)]),
    CFunction(name="ecwDaqReaderGetVariable", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(SDN_EC_T_DAQ_READER_VARIABLE)]),
    CFunction(name="ecwDaqReaderGetRecordData", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint]),
    CFunction(name="ecwDaqReaderGetVariableValue", ret=ctypes.c_uint, args=[ctypes.c_uint, ctypes.POINTER(SDN_EC_T_DAQ_READER_VARIABLE), ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint]),
#// @CODEGENERATOR_IMPL_END@
]

class CEcWrapperTypes:

    @staticmethod
    def Conv_IntToBytes(val, size):
        return val.to_bytes(size, byteorder='little')

    @staticmethod
    def Conv_IntFromBytes(val):
        return int.from_bytes(val, 'little')

    @staticmethod
    def Conv_IntArrayFromChunks(arr, size):
        bytes2 = []
        for b in arr:
            bytes3 = CEcWrapperTypes.Conv_IntToBytes(b, size)
            for b2 in bytes3:
                bytes2.append(int(b2))
        return bytes2

    @staticmethod
    def Conv_IntArrayToChunks(arr, size):
        arr2 = []
        for i in range(0, len(arr), size):
            arr3 = []
            for j in range(i, i + size):
                arr3.append(arr[j])
            val = CEcWrapperTypes.Conv_IntFromBytes(arr3)
            arr2.append(val)
        return arr2

    @staticmethod
    def Conv_IntArrayToBytePtr(val):
        size = len(val)
        buff = (ctypes.c_ubyte * size)(*val)
        return ctypes.cast(buff, ctypes.POINTER(ctypes.c_char * len(buff)))[0]

    @staticmethod
    def Conv_IntArrayFromBytePtr(val):
        # https://www.delftstack.com/de/howto/python/how-to-convert-bytes-to-integers/#int-from-bytes-beispiele
        ar = []
        for a in val:
            ar.append(CEcWrapperTypes.Conv_IntFromBytes(a))
        return ar

    @staticmethod
    def Conv_IntArrayFromBytePtrWithSize(val,size):
        # cast void*
        arsize = 0
        if isinstance(size, ctypes.c_uint):
            arsize = size.value
        else:
            arsize = size
        val = ctypes.cast(val, ctypes.POINTER(ctypes.c_char))
        ar = [0] * arsize
        for i in range(arsize):
            ar[i] = val[i]
        return CEcWrapperTypes.Conv_IntArrayFromBytePtr(ar)

    @staticmethod
    def Conv_IntArrayFromBytePtr2(pProcessData, dwOffset, pData):
        for i in range(len(pData)):
            if isinstance(pProcessData[dwOffset + i], bytes):
                pData[i] = CEcWrapperTypes.Conv_IntFromBytes(pProcessData[dwOffset + i])
            else:
                pData[i] = pProcessData[dwOffset + i]

    @staticmethod
    def Create_ArrayByType(type_, size):
        defval = 0
        if type_ == "bool":
            defval = False
        elif type_.startswith("DN_"):
            defval = None
        return [defval] * size

    @staticmethod
    def Conv_IntArrayToCTypesArray(ctype, val):
        return (ctype * len(val))(*val)

    @staticmethod
    def Conv_IntArrayFromCTypesArray(val):
        return [x for x in val]

    @staticmethod
    def Conv_Array(src, dst, size=None):
        if size is None:
            size = len(src)

        # workaround for bytes/char_array does not support item assignment
        if isinstance(src, bytes) or isinstance(src, ctypes.Array):
            src = ctypes.cast(src, ctypes.POINTER(ctypes.c_uint8))
        if isinstance(dst, bytes) or isinstance(src, ctypes.Array):
            dst = ctypes.cast(dst, ctypes.POINTER(ctypes.c_uint8))

        for i in range(size):
            dst[i] = src[i]

    @staticmethod
    def Conv_StrToCharPtr(str_):
        return ctypes.c_char_p(str_.encode('utf-8'))

    @staticmethod
    def Conv_StrFromCharPtr(str_):
        return str_.decode()

    @staticmethod
    def Pack(ctype_instance):
        buf = ctypes.string_at(ctypes.byref(ctype_instance), ctypes.sizeof(ctype_instance))
        return buf

    @staticmethod
    def Unpack(ctype, buf):
        cstring = ctypes.create_string_buffer(buf)
        ctype_instance = ctypes.cast(ctypes.pointer(cstring), ctypes.POINTER(ctype)).contents
        return ctype_instance

    @staticmethod
    def Conv(obj1, type_="", pack=False):
        name = obj1.__class__.__name__

        if isinstance(obj1, Enum):
            return obj1.value
        if isinstance(obj1, str):
            return ctypes.c_char_p(obj1.encode('utf-8'))
        if type_ == "byte[]":
            return CEcWrapperTypes.Conv_IntArrayToBytePtr(obj1)

        if isinstance(obj1, list) and len(obj1) > 0:
            arr = []
            for elem in obj1:
                elem2 = CEcWrapperTypes.Conv(elem)
                if elem2 == None:
                    continue
                arr.append(elem2)
            return arr

        if name != "" and not name.startswith("DN_") and not name.startswith("SDN_"):
            obj2 = None
            if name.startswith("c_"):
                obj2 = CEcWrapperTypes.ConvSimpleDataTypeFromCTypes(obj1, type_)
            else:
                obj2 = CEcWrapperTypes.ConvSimpleDataTypeToCTypes(obj1, type_)
            if obj2 is not None:
                return obj2

        name2 = name

        if name.startswith("SDN_"):
            name2 = name.replace("SDN_", "DN_")
            t = eval(name2)
            obj2 = t()
            CEcWrapperTypes.CopyFieldsFromStruct(obj1, obj2)
            return obj2

        if name.startswith("DN_"):
            name2 = name.replace("DN_", "SDN_")
            t = eval(name2)
            obj2 = t()
            CEcWrapperTypes.CopyFieldsToStruct(obj1, obj2)
            if pack:
                obj2 = CEcWrapperTypes.Pack(obj2)
            return obj2

        return obj1

    @staticmethod
    def CopyFieldsToStruct(obj1, obj2):
        for attr, value in obj1.__dict__.items():
            if value is None:
                continue
            for attr2 in obj2._fields_:
                if attr == attr2[0]:
                    try:
                        value2 = getattr(obj2, attr)
                        if isinstance(value, Enum):
                            setattr(obj2, attr, value.value)
                        elif isinstance(value, list) and isinstance(value2, ctypes.Array):
                            if len(value) == len(value2):
                                for i, elem in enumerate(value):
                                    if elem.__class__.__name__.startswith("DN_"):
                                        value2[i] = CEcWrapperTypes.Conv(elem, False)
                                    elif value2[i].__class__.__name__.startswith("SDN_"):
                                        continue  # elem = None
                                    else:
                                        value2[i] = elem
                                setattr(obj2, attr, value2)
                        elif value is not None and value.__class__.__name__.startswith("DN_"):
                            value21 = CEcWrapperTypes.Conv(value, False)
                            if value2 is None:
                                value22 = ctypes.py_object(value21)
                                value2 = ctypes.cast(ctypes.pointer(value22), ctypes.c_void_p)
                            else:
                                value2 = value21
                            setattr(obj2, attr, value2)
                        elif isinstance(value, str):
                            if len(value)> 0:
                                value2 = str.encode(value)
                            #for i in range(len(value)):
                            #    value2[i] = value[i]
                            #value2 = ctypes.create_string_buffer(value)
                            #value2 = ctypes.cast(value, ctypes.POINTER(ctypes.c_char * 20))[0]
                            setattr(obj2, attr, value2)
                        else:
                            setattr(obj2, attr, value)
                    except Exception as error:
                        print("CEcWrapperTypes.CopyFieldsToStruct: " + attr + " " + str(error))
                    break

    @staticmethod
    def CopyFieldsFromStruct(obj1, obj2):
        for attr in obj1._fields_:
            for attr2, value2 in obj2.__dict__.items():
                if attr[0] == attr2:
                    v = getattr(obj1, attr2)
                    try:
                        if isinstance(value2, str):
                            setattr(obj2, attr2, str(v.decode('utf-8')))
                        else:
                            setattr(obj2, attr2, v)
                    except Exception as error:
                        print("CEcWrapperTypes.CopyFieldsFromStruct: " + attr2 + " " + str(error))
                    break

    @staticmethod
    def ConvSimpleDataTypeFromCTypes(obj1, _type_):
        return obj1.value

    @staticmethod
    def ConvSimpleDataTypeToCTypes(obj1, type_):
        if type_ == "bool":
            return ctypes.c_uint32(1) if obj1 == True else ctypes.c_uint32(0)
        if type_ == "byte":
            return ctypes.c_uint8(obj1)
        if type_ == "ushort":
            return ctypes.c_uint16(obj1)
        if type_ == "uint":
            return ctypes.c_uint32(obj1)
        if type_ == "uint64":
            return ctypes.c_uint64(obj1)
        if type_ == "sbyte":
            return ctypes.c_int8(obj1)
        if type_ == "short":
            return ctypes.c_int16(obj1)
        if type_ == "int":
            return ctypes.c_int32(obj1)
        if type_ == "int64":
            return ctypes.c_int64(obj1)
        if type_ == "IntPtr":
            return ctypes.c_void_p(obj1)
        return None

    @staticmethod
    def ConvRes(obj):
        return obj

    @staticmethod
    def CreateStructArray(obj, length):
        objType = eval(obj.__class__.__name__)
        arr = []
        for _ in range(length):
            elem = objType()
            arr.append(elem)
        return arr

    @staticmethod
    def ConvStructArrayToBytes(obj):
        collection = obj
        if not isinstance(obj, list) or len(obj) == 0:
            return None
        structObj = collection[0]
        size = CEcWrapperTypes.GetSizeOfStructure(structObj)
        bytes_ = [0] * (size * len(collection))
        for i in range(len(collection)):
            bytesElem = bytearray(collection[i])
            for j in range(size):
                bytes_[(i * size) + j] = bytesElem[j]
        return CEcWrapperTypes.Conv_IntArrayToCTypesArray(ctypes.c_ubyte, bytes_)

    @staticmethod
    def ConvStructArrayFromBytes(bytes_, obj):
        collection = obj
        if not isinstance(obj, list) or len(obj) == 0:
            return None
        structObj = collection[0]
        structType = eval(structObj.__class__.__name__)
        size = CEcWrapperTypes.GetSizeOfStructure(structObj)
        arr = []
        for i in range(len(collection)):
            elem = structType.from_buffer(bytes_, size * i)
            arr.append(elem)
        return arr

    @staticmethod
    def CreateCharArray():
        return ""

    @staticmethod
    def ConvCharArrayToBytes(obj, length):
        bytes_ = [0] * length
        if len(obj) > 0:
            utf8Bytes = obj.encode('utf-8')
            for i in range(len(utf8Bytes)):
                bytes_[i] = utf8Bytes[i]
        return bytes_

    @staticmethod
    def ConvCharArrayFromBytes(bytes_):
        return CEcWrapperTypes.ConvertMasterStringFromBytes(bytes_, 0, bytes_.Length)

    @staticmethod
    def ConvertMasterStringFromBytes(abyData, dwOffset, dwLen):
        out_value = object()
        out_value.value = ""
        CEcWrapperTypes.ReadPdByteFromBytes(abyData, dwOffset, dwLen, out_value)
        return CEcWrapperTypes.ConvertMasterString(str(out_value.value))

    @staticmethod
    def ConvertMasterString(val):
        return CEcWrapperTypes.PatchString(val)

    @staticmethod
    def PatchString(str_):
        #//str->Replace("\0", String::Empty) // Doesn't work e.g. for BK1120, Coe-OD-Index 0x4000
        if not str_:
            return str_

        idx = str_.find('\0')
        if idx != -1:
            str_ = str_[0:idx]

        return str_

    @staticmethod
    def GetNotificationDataType(code):
        #// @CODEGENERATOR_IMPL_NOTIFYTYPE_BEGIN@
        if code == DN_NotifyCode.STATECHANGED:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.ETH_LINK_CONNECTED:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.SB_STATUS:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.DC_STATUS:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.DC_SLV_SYNC:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.DCL_STATUS:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.DCM_SYNC:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.DCX_SYNC:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.SLAVE_STATECHANGED:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.SLAVES_STATECHANGED:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.RAWCMD_DONE:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.COE_TX_PDO:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.SLAVE_PRESENCE:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.SLAVES_PRESENCE:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.REFCLOCK_PRESENCE:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.MASTER_RED_STATECHANGED:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.MASTER_RED_FOREIGN_SRC_MAC:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.SLAVE_REGISTER_TRANSFER:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.PORT_OPERATION:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.SLAVE_IDENTIFICATION:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.RELEASE_FORCED_PROCESSDATA:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.CYCCMD_WKC_ERROR:
            return NotificationDataType.Error
        if code == DN_NotifyCode.MASTER_INITCMD_WKC_ERROR:
            return NotificationDataType.Error
        if code == DN_NotifyCode.SLAVE_INITCMD_WKC_ERROR:
            return NotificationDataType.Error
        if code == DN_NotifyCode.EOE_MBXSND_WKC_ERROR:
            return NotificationDataType.Error
        if code == DN_NotifyCode.COE_MBXSND_WKC_ERROR:
            return NotificationDataType.Error
        if code == DN_NotifyCode.FOE_MBXSND_WKC_ERROR:
            return NotificationDataType.Error
        if code == DN_NotifyCode.FRAME_RESPONSE_ERROR:
            return NotificationDataType.Error
        if code == DN_NotifyCode.SLAVE_INITCMD_RESPONSE_ERROR:
            return NotificationDataType.Error
        if code == DN_NotifyCode.MASTER_INITCMD_RESPONSE_ERROR:
            return NotificationDataType.Error
        if code == DN_NotifyCode.MBSLAVE_INITCMD_TIMEOUT:
            return NotificationDataType.Error
        if code == DN_NotifyCode.NOT_ALL_DEVICES_OPERATIONAL:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.ETH_LINK_NOT_CONNECTED:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.RED_LINEBRK:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.STATUS_SLAVE_ERROR:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.SLAVE_ERROR_STATUS_INFO:
            return NotificationDataType.Error
        if code == DN_NotifyCode.SLAVE_NOT_ADDRESSABLE:
            return NotificationDataType.Error
        if code == DN_NotifyCode.SOE_MBXSND_WKC_ERROR:
            return NotificationDataType.Error
        if code == DN_NotifyCode.SOE_WRITE_ERROR:
            return NotificationDataType.Error
        if code == DN_NotifyCode.MBSLAVE_COE_SDO_ABORT:
            return NotificationDataType.Error
        if code == DN_NotifyCode.CLIENTREGISTRATION_DROPPED:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.RED_LINEFIXED:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.FOE_MBSLAVE_ERROR:
            return NotificationDataType.Error
        if code == DN_NotifyCode.MBXRCV_INVALID_DATA:
            return NotificationDataType.Error
        if code == DN_NotifyCode.PDIWATCHDOG:
            return NotificationDataType.Error
        if code == DN_NotifyCode.SLAVE_NOTSUPPORTED:
            return NotificationDataType.Error
        if code == DN_NotifyCode.SLAVE_UNEXPECTED_STATE:
            return NotificationDataType.Error
        if code == DN_NotifyCode.ALL_DEVICES_OPERATIONAL:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.VOE_MBXSND_WKC_ERROR:
            return NotificationDataType.Error
        if code == DN_NotifyCode.EEPROM_CHECKSUM_ERROR:
            return NotificationDataType.Error
        if code == DN_NotifyCode.JUNCTION_RED_CHANGE:
            return NotificationDataType.Error
        if code == DN_NotifyCode.SLAVES_UNEXPECTED_STATE:
            return NotificationDataType.Error
        if code == DN_NotifyCode.LINE_CROSSED:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.SLAVES_ERROR_STATUS:
            return NotificationDataType.Error
        if code == DN_NotifyCode.FRAMELOSS_AFTER_SLAVE:
            return NotificationDataType.Error
        if code == DN_NotifyCode.S2SMBX_ERROR:
            return NotificationDataType.Error
        if code == DN_NotifyCode.BAD_CONNECTION:
            return NotificationDataType.Error
        if code == DN_NotifyCode.COMMUNICATION_TIMEOUT:
            return NotificationDataType.Error
        if code == DN_NotifyCode.TAP_LINK_STATUS:
            return NotificationDataType.Error
        if code == DN_NotifyCode.SB_MISMATCH:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.SB_DUPLICATE_HC_NODE:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.HC_DETECTADDGROUPS:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.HC_PROBEALLGROUPS:
            return NotificationDataType.Notify
        if code == DN_NotifyCode.HC_TOPOCHGDONE:
            return NotificationDataType.Notify
        #// @CODEGENERATOR_IMPL_NOTIFYTYPE_END@
        return NotificationDataType.Default

    @staticmethod
    def ConvNotificationData(code, pbyInBuf):
        #// @CODEGENERATOR_IMPL_NOTIFY_BEGIN@
        if code == DN_NotifyCode.STATECHANGED:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_STATECHANGE))[0])
        if code == DN_NotifyCode.ETH_LINK_CONNECTED:
            return None
        if code == DN_NotifyCode.SB_STATUS:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_SB_STATUS_NTFY_DESC))[0])
        if code == DN_NotifyCode.DC_STATUS:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(ctypes.c_uint))[0])
        if code == DN_NotifyCode.DC_SLV_SYNC:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_DC_SYNC_NTFY_DESC))[0])
        if code == DN_NotifyCode.DCL_STATUS:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(ctypes.c_uint))[0])
        if code == DN_NotifyCode.DCM_SYNC:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_DCM_SYNC_NTFY_DESC))[0])
        if code == DN_NotifyCode.DCX_SYNC:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_DCX_SYNC_NTFY_DESC))[0])
        if code == DN_NotifyCode.SLAVE_STATECHANGED:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_SLAVE_STATECHANGED_NTFY_DESC))[0])
        if code == DN_NotifyCode.SLAVES_STATECHANGED:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_SLAVES_STATECHANGED_NTFY_DESC))[0])
        if code == DN_NotifyCode.RAWCMD_DONE:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_RAWCMDRESPONSE_NTFY_DESC))[0])
        if code == DN_NotifyCode.COE_TX_PDO:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_TX_PDO_NTFY_DESC))[0])
        if code == DN_NotifyCode.SLAVE_PRESENCE:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_SLAVE_PRESENCE_NTFY_DESC))[0])
        if code == DN_NotifyCode.SLAVES_PRESENCE:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_SLAVES_PRESENCE_NTFY_DESC))[0])
        if code == DN_NotifyCode.REFCLOCK_PRESENCE:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_REFCLOCK_PRESENCE_NTFY_DESC))[0])
        if code == DN_NotifyCode.MASTER_RED_STATECHANGED:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(ctypes.c_uint))[0])
        if code == DN_NotifyCode.MASTER_RED_FOREIGN_SRC_MAC:
            return None
        if code == DN_NotifyCode.SLAVE_REGISTER_TRANSFER:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_SLAVEREGISTER_TRANSFER_NTFY_DESC))[0])
        if code == DN_NotifyCode.PORT_OPERATION:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_PORT_OPERATION_NTFY_DESC))[0])
        if code == DN_NotifyCode.SLAVE_IDENTIFICATION:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_SLAVE_IDENTIFICATION_NTFY_DESC))[0])
        if code == DN_NotifyCode.RELEASE_FORCED_PROCESSDATA:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_RELEASE_FORCED_PROCESSDATA_NTFY_DESC))[0])
        if code == DN_NotifyCode.CYCCMD_WKC_ERROR:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_WKCERR_DESC))[0])
        if code == DN_NotifyCode.MASTER_INITCMD_WKC_ERROR:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_WKCERR_DESC))[0])
        if code == DN_NotifyCode.SLAVE_INITCMD_WKC_ERROR:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_WKCERR_DESC))[0])
        if code == DN_NotifyCode.EOE_MBXSND_WKC_ERROR:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_WKCERR_DESC))[0])
        if code == DN_NotifyCode.COE_MBXSND_WKC_ERROR:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_WKCERR_DESC))[0])
        if code == DN_NotifyCode.FOE_MBXSND_WKC_ERROR:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_WKCERR_DESC))[0])
        if code == DN_NotifyCode.FRAME_RESPONSE_ERROR:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_FRAME_RSPERR_DESC))[0])
        if code == DN_NotifyCode.SLAVE_INITCMD_RESPONSE_ERROR:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_INITCMD_ERR_DESC))[0])
        if code == DN_NotifyCode.MASTER_INITCMD_RESPONSE_ERROR:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_INITCMD_ERR_DESC))[0])
        if code == DN_NotifyCode.MBSLAVE_INITCMD_TIMEOUT:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_INITCMD_ERR_DESC))[0])
        if code == DN_NotifyCode.NOT_ALL_DEVICES_OPERATIONAL:
            return None
        if code == DN_NotifyCode.ETH_LINK_NOT_CONNECTED:
            return None
        if code == DN_NotifyCode.RED_LINEBRK:
            return None
        if code == DN_NotifyCode.STATUS_SLAVE_ERROR:
            return None
        if code == DN_NotifyCode.SLAVE_ERROR_STATUS_INFO:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_SLAVE_ERROR_INFO_DESC))[0])
        if code == DN_NotifyCode.SLAVE_NOT_ADDRESSABLE:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_WKCERR_DESC))[0])
        if code == DN_NotifyCode.SOE_MBXSND_WKC_ERROR:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_WKCERR_DESC))[0])
        if code == DN_NotifyCode.SOE_WRITE_ERROR:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_INITCMD_ERR_DESC))[0])
        if code == DN_NotifyCode.MBSLAVE_COE_SDO_ABORT:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_MBOX_SDO_ABORT_DESC))[0])
        if code == DN_NotifyCode.CLIENTREGISTRATION_DROPPED:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(ctypes.c_uint))[0])
        if code == DN_NotifyCode.RED_LINEFIXED:
            return None
        if code == DN_NotifyCode.FOE_MBSLAVE_ERROR:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_MBOX_FOE_ABORT_DESC))[0])
        if code == DN_NotifyCode.MBXRCV_INVALID_DATA:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_MBXRCV_INVALID_DATA_DESC))[0])
        if code == DN_NotifyCode.PDIWATCHDOG:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_PDIWATCHDOG_DESC))[0])
        if code == DN_NotifyCode.SLAVE_NOTSUPPORTED:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_SLAVE_NOTSUPPORTED_DESC))[0])
        if code == DN_NotifyCode.SLAVE_UNEXPECTED_STATE:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_SLAVE_UNEXPECTED_STATE_DESC))[0])
        if code == DN_NotifyCode.ALL_DEVICES_OPERATIONAL:
            return None
        if code == DN_NotifyCode.VOE_MBXSND_WKC_ERROR:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_WKCERR_DESC))[0])
        if code == DN_NotifyCode.EEPROM_CHECKSUM_ERROR:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_EEPROM_CHECKSUM_ERROR_DESC))[0])
        if code == DN_NotifyCode.JUNCTION_RED_CHANGE:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_JUNCTION_RED_CHANGE_DESC))[0])
        if code == DN_NotifyCode.SLAVES_UNEXPECTED_STATE:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_SLAVES_UNEXPECTED_STATE_DESC))[0])
        if code == DN_NotifyCode.LINE_CROSSED:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_LINE_CROSSED_DESC))[0])
        if code == DN_NotifyCode.SLAVES_ERROR_STATUS:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_SLAVES_ERROR_DESC))[0])
        if code == DN_NotifyCode.FRAMELOSS_AFTER_SLAVE:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_FRAMELOSS_AFTER_SLAVE_NTFY_DESC))[0])
        if code == DN_NotifyCode.S2SMBX_ERROR:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_S2SMBX_ERROR_DESC))[0])
        if code == DN_NotifyCode.BAD_CONNECTION:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_BAD_CONNECTION_NTFY_DESC))[0])
        if code == DN_NotifyCode.COMMUNICATION_TIMEOUT:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_COMMUNICATION_TIMEOUT_NTFY_DESC))[0])
        if code == DN_NotifyCode.TAP_LINK_STATUS:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_TAP_LINK_STATUS_NTFY_DESC))[0])
        if code == DN_NotifyCode.SB_MISMATCH:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_SB_MISMATCH_DESC))[0])
        if code == DN_NotifyCode.SB_DUPLICATE_HC_NODE:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_SB_MISMATCH_DESC))[0])
        if code == DN_NotifyCode.HC_DETECTADDGROUPS:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_HC_DETECTALLGROUP_NTFY_DESC))[0])
        if code == DN_NotifyCode.HC_PROBEALLGROUPS:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_EC_T_HC_DETECTALLGROUP_NTFY_DESC))[0])
        if code == DN_NotifyCode.HC_TOPOCHGDONE:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(ctypes.c_uint))[0])
        if code == DN_NotifyCode.MBOXRCV:
            pMbxTfer = SDN_EC_T_MBXTFER()
            pMbxTferData = ctypes.c_void_p(None) # IntPtr
            dwMaxDataLen = ctypes.c_uint(0) # uint
            pbyMbxTferDescData = ctypes.c_void_p(None) # IntPtr
            CEcWrapper.Get().ecwParseMbxTransferData(pbyInBuf, ctypes.pointer(pMbxTfer), ctypes.byref(pMbxTferData), ctypes.byref(dwMaxDataLen), ctypes.byref(pbyMbxTferDescData))
            eMbxTferType = DN_EC_T_MBXTFER_TYPE(pMbxTfer.eMbxTferType)

            if eMbxTferType == DN_EC_T_MBXTFER_TYPE.COE_SDO_DOWNLOAD:
                mboxRcv = CEcWrapperTypes.ConvMbxRcv(pMbxTfer)
                mboxRcv.MbxTferData = CEcWrapperTypes.ConvMbxTferData(pbyMbxTferDescData, dwMaxDataLen)
                mboxRcv.MbxData = CEcWrapperTypes.Conv(ctypes.cast(pMbxTferData, ctypes.POINTER(SDN_EC_T_MBX_DATA_COE))[0])
                return mboxRcv
            if eMbxTferType == DN_EC_T_MBXTFER_TYPE.COE_SDO_UPLOAD:
                mboxRcv = CEcWrapperTypes.ConvMbxRcv(pMbxTfer)
                mboxRcv.MbxTferData = CEcWrapperTypes.ConvMbxTferData(pbyMbxTferDescData, dwMaxDataLen)
                mboxRcv.MbxData = CEcWrapperTypes.Conv(ctypes.cast(pMbxTferData, ctypes.POINTER(SDN_EC_T_MBX_DATA_COE))[0])
                return mboxRcv
            if eMbxTferType == DN_EC_T_MBXTFER_TYPE.COE_GETODLIST:
                mboxRcv = CEcWrapperTypes.ConvMbxRcv(pMbxTfer)
                mboxRcv.MbxTferData = CEcWrapperTypes.ConvMbxTferData(pbyMbxTferDescData, dwMaxDataLen)
                return mboxRcv
            if eMbxTferType == DN_EC_T_MBXTFER_TYPE.COE_GETOBDESC:
                mboxRcv = CEcWrapperTypes.ConvMbxRcv(pMbxTfer)
                mboxRcv.MbxTferData = CEcWrapperTypes.ConvMbxTferData(pbyMbxTferDescData, dwMaxDataLen)
                return mboxRcv
            if eMbxTferType == DN_EC_T_MBXTFER_TYPE.COE_GETENTRYDESC:
                mboxRcv = CEcWrapperTypes.ConvMbxRcv(pMbxTfer)
                mboxRcv.MbxTferData = CEcWrapperTypes.ConvMbxTferData(pbyMbxTferDescData, dwMaxDataLen)
                return mboxRcv
            if eMbxTferType == DN_EC_T_MBXTFER_TYPE.COE_EMERGENCY:
                mboxRcv = CEcWrapperTypes.ConvMbxRcv(pMbxTfer)
                mboxRcv.MbxTferData = CEcWrapperTypes.ConvMbxTferData(pbyMbxTferDescData, dwMaxDataLen)
                mboxRcv.MbxData = CEcWrapperTypes.Conv(ctypes.cast(pMbxTferData, ctypes.POINTER(SDN_EC_T_COE_EMERGENCY))[0])
                return mboxRcv
            if eMbxTferType == DN_EC_T_MBXTFER_TYPE.FOE_FILE_DOWNLOAD:
                mboxRcv = CEcWrapperTypes.ConvMbxRcv(pMbxTfer)
                mboxRcv.MbxTferData = CEcWrapperTypes.ConvMbxTferData(pbyMbxTferDescData, dwMaxDataLen)
                mboxRcv.MbxData = CEcWrapperTypes.Conv(ctypes.cast(pMbxTferData, ctypes.POINTER(SDN_EC_T_MBX_DATA_FOE))[0])
                return mboxRcv
            if eMbxTferType == DN_EC_T_MBXTFER_TYPE.FOE_FILE_UPLOAD:
                mboxRcv = CEcWrapperTypes.ConvMbxRcv(pMbxTfer)
                mboxRcv.MbxTferData = CEcWrapperTypes.ConvMbxTferData(pbyMbxTferDescData, dwMaxDataLen)
                mboxRcv.MbxData = CEcWrapperTypes.Conv(ctypes.cast(pMbxTferData, ctypes.POINTER(SDN_EC_T_MBX_DATA_FOE))[0])
                return mboxRcv
            if eMbxTferType == DN_EC_T_MBXTFER_TYPE.SOE_WRITEREQUEST:
                mboxRcv = CEcWrapperTypes.ConvMbxRcv(pMbxTfer)
                mboxRcv.MbxTferData = CEcWrapperTypes.ConvMbxTferData(pbyMbxTferDescData, dwMaxDataLen)
                return mboxRcv
            if eMbxTferType == DN_EC_T_MBXTFER_TYPE.SOE_READREQUEST:
                mboxRcv = CEcWrapperTypes.ConvMbxRcv(pMbxTfer)
                mboxRcv.MbxTferData = CEcWrapperTypes.ConvMbxTferData(pbyMbxTferDescData, dwMaxDataLen)
                return mboxRcv
            if eMbxTferType == DN_EC_T_MBXTFER_TYPE.SOE_EMERGENCY:
                mboxRcv = CEcWrapperTypes.ConvMbxRcv(pMbxTfer)
                mboxRcv.MbxTferData = CEcWrapperTypes.ConvMbxTferData(pbyMbxTferDescData, dwMaxDataLen)
                mboxRcv.MbxData = CEcWrapperTypes.Conv(ctypes.cast(pMbxTferData, ctypes.POINTER(SDN_EC_T_SOE_EMERGENCY))[0])
                return mboxRcv
            if eMbxTferType == DN_EC_T_MBXTFER_TYPE.SOE_NOTIFICATION:
                mboxRcv = CEcWrapperTypes.ConvMbxRcv(pMbxTfer)
                mboxRcv.MbxTferData = CEcWrapperTypes.ConvMbxTferData(pbyMbxTferDescData, dwMaxDataLen)
                mboxRcv.MbxData = CEcWrapperTypes.Conv(ctypes.cast(pMbxTferData, ctypes.POINTER(SDN_EC_T_SOE_NOTIFICATION))[0])
                return mboxRcv
            if eMbxTferType == DN_EC_T_MBXTFER_TYPE.VOE_MBX_WRITE:
                mboxRcv = CEcWrapperTypes.ConvMbxRcv(pMbxTfer)
                mboxRcv.MbxTferData = CEcWrapperTypes.ConvMbxTferData(pbyMbxTferDescData, dwMaxDataLen)
                return mboxRcv
            if eMbxTferType == DN_EC_T_MBXTFER_TYPE.VOE_MBX_READ:
                mboxRcv = CEcWrapperTypes.ConvMbxRcv(pMbxTfer)
                mboxRcv.MbxTferData = CEcWrapperTypes.ConvMbxTferData(pbyMbxTferDescData, dwMaxDataLen)
                return mboxRcv
            if eMbxTferType == DN_EC_T_MBXTFER_TYPE.AOE_WRITE:
                mboxRcv = CEcWrapperTypes.ConvMbxRcv(pMbxTfer)
                mboxRcv.MbxTferData = CEcWrapperTypes.ConvMbxTferData(pbyMbxTferDescData, dwMaxDataLen)
                mboxRcv.MbxData = CEcWrapperTypes.Conv(ctypes.cast(pMbxTferData, ctypes.POINTER(SDN_EC_T_AOE_CMD_RESPONSE))[0])
                return mboxRcv
            if eMbxTferType == DN_EC_T_MBXTFER_TYPE.AOE_READ:
                mboxRcv = CEcWrapperTypes.ConvMbxRcv(pMbxTfer)
                mboxRcv.MbxTferData = CEcWrapperTypes.ConvMbxTferData(pbyMbxTferDescData, dwMaxDataLen)
                mboxRcv.MbxData = CEcWrapperTypes.Conv(ctypes.cast(pMbxTferData, ctypes.POINTER(SDN_EC_T_AOE_CMD_RESPONSE))[0])
                return mboxRcv
            return None
        #// @CODEGENERATOR_IMPL_NOTIFY_END@
        return None

    @staticmethod
    def ConvMbxRcv(pMbxTfer):
        mboxRcv = DN_EC_T_MBOXRCV()
        mboxRcv.eMbxTferType = DN_EC_T_MBXTFER_TYPE(pMbxTfer.eMbxTferType)
        mboxRcv.eTferStatus = DN_EC_T_MBXTFER_STATUS(pMbxTfer.eTferStatus)
        mboxRcv.dwErrorCode = pMbxTfer.dwErrorCode
        mboxRcv.dwTferId = pMbxTfer.dwTferId
        return mboxRcv

    @staticmethod
    def ConvMbxTferData(pbyMbxTferDescData, dwMaxDataLen):
        if pbyMbxTferDescData == ctypes.c_void_p(None) or dwMaxDataLen == ctypes.c_uint(0):
            return None
        try:
            return CEcWrapperTypes.Conv_IntArrayFromBytePtrWithSize(pbyMbxTferDescData, dwMaxDataLen)
        except:
            return None

    @staticmethod
    def ConvRasNotificationData(code, pbyInBuf):
        if code == DN_RasNotifyCode.CONNECTION:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_ATEMRAS_T_CONNOTIFYDESC))[0])
        if code == DN_RasNotifyCode.REGISTER:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_ATEMRAS_T_REGNOTIFYDESC))[0])
        if code == DN_RasNotifyCode.UNREGISTER:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_ATEMRAS_T_REGNOTIFYDESC))[0])
        if code == DN_RasNotifyCode.MARSHALERROR:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_ATEMRAS_T_MARSHALERRORDESC))[0])
        if code == DN_RasNotifyCode.NONOTIFYMEMORY:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_ATEMRAS_T_NONOTIFYMEMORYDESC))[0])
        if code == DN_RasNotifyCode.STDNOTIFYMEMORYSMALL:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_ATEMRAS_T_NONOTIFYMEMORYDESC))[0])
        if code == DN_RasNotifyCode.MBXNOTIFYMEMORYSMALL:
            return CEcWrapperTypes.Conv(ctypes.cast(pbyInBuf, ctypes.POINTER(SDN_ATEMRAS_T_NONOTIFYMEMORYDESC))[0])
        return None

    @staticmethod
    def ConvBytesToStructure(bytes_, type2):
        return ctypes.cast(bytes_, ctypes.POINTER(type(type2)))[0]

    @staticmethod
    def GetSizeOfStructure(type_):
        return ctypes.sizeof(type_)

    @staticmethod
    def ConvMasterOdData(wObIndex):
        #// @CODEGENERATOR_IMPL_MASTEROD_BEGIN@
        if wObIndex == 0x2002:
            return SDN_EC_T_OBJ2002()
        if wObIndex == 0x2003:
            return SDN_EC_T_OBJ2003()
        if wObIndex == 0x2005:
            return SDN_EC_T_OBJ2005()
        if wObIndex == 0x2020:
            return SDN_EC_T_OBJ2020()
        if wObIndex == 0x2200:
            return SDN_EC_T_OBJ2200()
        if wObIndex >= 0x3000 and wObIndex <= 0x3FFF:
            return SDN_EC_T_OBJ3XXX()
        if wObIndex >= 0x8000 and wObIndex <= 0x8FFF:
            return SDN_EC_T_OBJ8XXX()
        if wObIndex >= 0x9000 and wObIndex <= 0x9FFF:
            return SDN_EC_T_OBJ9XXX()
        if wObIndex >= 0xA000 and wObIndex <= 0xAFFF:
            return SDN_EC_T_OBJAXXX()
        if wObIndex == 0xF000:
            return SDN_EC_T_OBJF000()
        if wObIndex >= 0xF020 and wObIndex <= 0xF02F:
            return SDN_EC_T_OBJF02X()
        if wObIndex >= 0xF040 and wObIndex <= 0xF04F:
            return SDN_EC_T_OBJF04X()
        #// @CODEGENERATOR_IMPL_MASTEROD_END@
        return None

    @staticmethod
    def ReadPdBitsFromAddress(pProcessData, dwBitOffset, dwBitLen, out_pData):
        pData = [0] * dwBitLen
        pDataPin = CEcWrapperTypes.Conv_IntArrayToBytePtr(pData)
        CEcWrapper.Get().ecwBitCopy(pDataPin, 0, pProcessData, dwBitOffset, dwBitLen)
        CEcWrapperTypes.Conv_IntArrayFromBytePtr2(pDataPin, 0, pData)
        out_pData.value = pData

    @staticmethod
    def ReadPdBitsFromBytes(pProcessData, dwBitOffset, dwBitLen, out_pData):
        pProcessDataPin = CEcWrapperTypes.Conv_IntArrayToBytePtr(pProcessData)
        CEcWrapperTypes.ReadPdBitsFromAddress(pProcessDataPin, dwBitOffset, dwBitLen, out_pData)

    @staticmethod
    def ReadPdByteFromAddress(pProcessData, dwOffset, dwLength, out_pData):
        src = CEcWrapper.Get().ecwIntPtrAdd(pProcessData, dwOffset)
        out_pData.value = CEcWrapperTypes.Conv_IntArrayFromBytePtrWithSize(src, dwLength)

    @staticmethod
    def ReadPdByteFromBytes(pProcessData, dwOffset, dwLength, out_pData):
        pData = [0] * dwLength
        CEcWrapperTypes.Conv_IntArrayFromBytePtr2(pProcessData, dwOffset, pData)
        out_pData.value = pData

    @staticmethod
    def WritePdBitsToAddress(pProcessData, dwBitOffset, pData, dwBitLen):
        pDataPin = CEcWrapperTypes.Conv_IntArrayToBytePtr(pData)
        CEcWrapper.Get().ecwBitCopy(pProcessData, dwBitOffset, pDataPin, 0, dwBitLen)

    @staticmethod
    def WritePdBitsToBytes(pProcessData, dwBitOffset, pData, dwBitLen):
        pProcessDataPin = CEcWrapperTypes.Conv_IntArrayToBytePtr(pProcessData)
        CEcWrapperTypes.WritePdBitsToAddress(pProcessDataPin, dwBitOffset, pData, dwBitLen)
        CEcWrapperTypes.Conv_IntArrayFromBytePtr2(pProcessDataPin, 0, pProcessData)

    @staticmethod
    def WritePdByteToAddress(pProcessData, dwOffset, pData):
        pDataPin = CEcWrapperTypes.Conv_IntArrayToBytePtr(pData)
        CEcWrapper.Get().ecwBitCopy(pProcessData, dwOffset * 8, pDataPin, 0, len(pData) * 8)

    @staticmethod
    def WritePdByteToBytes(pProcessData, dwOffset, pData):
        for i, elem in enumerate(pData):
            pProcessData[dwOffset + i] = elem

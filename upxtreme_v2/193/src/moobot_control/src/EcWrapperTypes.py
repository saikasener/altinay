#/*-----------------------------------------------------------------------------
# * EcWrapperTypes.py
# * Copyright                acontis technologies GmbH, Ravensburg, Germany
# * Description              EC-Wrapper CPython Types (internal)
# *---------------------------------------------------------------------------*/
# pylint: disable=unused-wildcard-import, wildcard-import
from enum import Enum
from EcWrapperPythonTypes import *
import ctypes

class SDN_EC_T_PERF_MEAS_COUNTER_PARMS(ctypes.Structure):
    _fields_ = [
        ("qwFrequency", ctypes.c_uint64), # uint64
    ]

class SDN_EC_T_PERF_MEAS_HISTOGRAM_PARMS(ctypes.Structure):
    _fields_ = [
        ("dwBinCount", ctypes.c_uint), # uint
        ("qwMinTicks", ctypes.c_uint64), # uint64
        ("qwMaxTicks", ctypes.c_uint64), # uint64
    ]

class SDN_EC_T_PERF_MEAS_INTERNAL_PARMS(ctypes.Structure):
    _fields_ = [
        ("bEnabled", ctypes.c_uint), # bool
        ("CounterParms", SDN_EC_T_PERF_MEAS_COUNTER_PARMS), # SDN_EC_T_PERF_MEAS_COUNTER_PARMS
        ("HistogramParms", SDN_EC_T_PERF_MEAS_HISTOGRAM_PARMS), # SDN_EC_T_PERF_MEAS_HISTOGRAM_PARMS
    ]

#// @CODEGENERATOR_IMPL_BEGIN@
class SDN_EC_T_OBJ2002(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_OBJ2003(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_OBJ2005(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_OBJ2020(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_OBJ2200(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_OBJ3XXX(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_OBJ8XXX(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_OBJ9XXX(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_OBJAXXX(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_OBJF000(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_OBJF02X(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_OBJF04X(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_SLAVE_PROP(ctypes.Structure):
    _fields_ = [
        ("wStationAddress", ctypes.c_ushort), # ushort
        ("wAutoIncAddr", ctypes.c_ushort), # ushort
        ("achName", ctypes.c_char * 80), # string[80]
    ]

class SDN_EC_T_NOTIFYPARMS(ctypes.Structure):
    _fields_ = [
        ("pCallerData", ctypes.c_void_p), # IntPtr
        ("pbyInBuf", ctypes.c_void_p), # IntPtr
        ("dwInBufSize", ctypes.c_uint), # uint
        ("pbyOutBuf", ctypes.c_void_p), # IntPtr
        ("dwOutBufSize", ctypes.c_uint), # uint
        ("pdwNumOutData", ctypes.c_uint), # uint
    ]

class SDN_EC_T_WKCERR_DESC(ctypes.Structure):
    _fields_ = [
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
        ("byCmd", ctypes.c_ubyte), # byte
        ("byRsvd", ctypes.c_uint8 * 3), # byte[3]
        ("dwAddr", ctypes.c_uint), # uint
        ("wWkcSet", ctypes.c_ushort), # ushort
        ("wWkcAct", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_FRAME_RSPERR_DESC(ctypes.Structure):
    _fields_ = [
        ("bIsCyclicFrame", ctypes.c_uint), # bool
        ("EErrorType", ctypes.c_uint), # DN_EC_T_FRAME_RSPERR_TYPE
        ("byEcCmdHeaderIdxSet", ctypes.c_ubyte), # byte
        ("byEcCmdHeaderIdxAct", ctypes.c_ubyte), # byte
        ("wCycFrameNum", ctypes.c_ushort), # ushort
        ("dwTaskId", ctypes.c_uint), # uint
    ]

class SDN_EC_T_INITCMD_ERR_DESC(ctypes.Structure):
    _fields_ = [
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
        ("achStateChangeName", ctypes.c_char * 20), # string[20]
        ("EErrorType", ctypes.c_uint), # DN_EC_T_INITCMD_ERR_TYPE
        ("szComment", ctypes.c_char * 80), # string[80]
    ]

class SDN_EC_T_SLAVE_ERROR_INFO_DESC(ctypes.Structure):
    _fields_ = [
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
        ("wStatus", ctypes.c_ushort), # ushort
        ("wStatusCode", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_SLAVES_ERROR_DESC_ENTRY(ctypes.Structure):
    _fields_ = [
        ("wStationAddress", ctypes.c_ushort), # ushort
        ("wStatus", ctypes.c_ushort), # ushort
        ("wStatusCode", ctypes.c_ushort), # ushort
        ("wRes", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_SLAVES_ERROR_DESC(ctypes.Structure):
    _fields_ = [
        ("wCount", ctypes.c_ushort), # ushort
        ("wRes", ctypes.c_ushort), # ushort
        ("SlaveError", SDN_EC_T_SLAVES_ERROR_DESC_ENTRY * 128), # SDN_EC_T_SLAVES_ERROR_DESC_ENTRY[128]
    ]

class SDN_EC_T_MBOX_SDO_ABORT_DESC(ctypes.Structure):
    _fields_ = [
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
        ("dwErrorCode", ctypes.c_uint), # uint
        ("wObjIndex", ctypes.c_ushort), # ushort
        ("bySubIndex", ctypes.c_ubyte), # byte
    ]

class SDN_EC_T_MBOX_FOE_ABORT_DESC(ctypes.Structure):
    _fields_ = [
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
        ("dwErrorCode", ctypes.c_uint), # uint
        ("achErrorString", ctypes.c_char * 80), # string[80]
    ]

class SDN_EC_T_MBXRCV_INVALID_DATA_DESC(ctypes.Structure):
    _fields_ = [
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
    ]

class SDN_EC_T_PDIWATCHDOG_DESC(ctypes.Structure):
    _fields_ = [
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
    ]

class SDN_EC_T_SLAVE_NOTSUPPORTED_DESC(ctypes.Structure):
    _fields_ = [
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
    ]

class SDN_EC_T_SLAVE_UNEXPECTED_STATE_DESC(ctypes.Structure):
    _fields_ = [
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
        ("curState", ctypes.c_uint), # DN_EC_T_STATE
        ("expState", ctypes.c_uint), # DN_EC_T_STATE
    ]

class SDN_EC_T_SLAVES_UNEXPECTED_STATE_DESC_ENTRY(ctypes.Structure):
    _fields_ = [
        ("wStationAddress", ctypes.c_ushort), # ushort
        ("curState", ctypes.c_uint), # DN_EC_T_STATE
        ("expState", ctypes.c_uint), # DN_EC_T_STATE
    ]

class SDN_EC_T_SLAVES_UNEXPECTED_STATE_DESC(ctypes.Structure):
    _fields_ = [
        ("wCount", ctypes.c_ushort), # ushort
        ("wRes", ctypes.c_ushort), # ushort
        ("SlaveStates", SDN_EC_T_SLAVES_UNEXPECTED_STATE_DESC_ENTRY * 128), # SDN_EC_T_SLAVES_UNEXPECTED_STATE_DESC_ENTRY[128]
    ]

class SDN_EC_T_EEPROM_CHECKSUM_ERROR_DESC(ctypes.Structure):
    _fields_ = [
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
    ]

class SDN_EC_T_JUNCTION_RED_CHANGE_DESC(ctypes.Structure):
    _fields_ = [
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
        ("bLineBreak", ctypes.c_uint), # bool
        ("wPort", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_REFCLOCK_PRESENCE_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("bPresent", ctypes.c_uint), # bool
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
    ]

class SDN_EC_T_DC_SYNC_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("IsInSync", ctypes.c_uint), # uint
        ("IsNegative", ctypes.c_uint), # uint
        ("dwDeviation", ctypes.c_uint), # uint
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
    ]

class SDN_EC_T_DCM_SYNC_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("IsInSync", ctypes.c_uint), # uint
        ("nCtlErrorNsecCur", ctypes.c_int), # int
        ("nCtlErrorNsecAvg", ctypes.c_int), # int
        ("nCtlErrorNsecMax", ctypes.c_int), # int
    ]

class SDN_EC_T_DCX_SYNC_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("IsInSync", ctypes.c_uint), # uint
        ("nCtlErrorNsecCur", ctypes.c_int), # int
        ("nCtlErrorNsecAvg", ctypes.c_int), # int
        ("nCtlErrorNsecMax", ctypes.c_int), # int
        ("nTimeStampDiff", ctypes.c_int64), # int64
        ("dwErrorCode", ctypes.c_uint), # uint
    ]

class SDN_EC_T_SLAVE_STATECHANGED_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
        ("newState", ctypes.c_uint), # DN_EC_T_STATE
    ]

class SDN_EC_T_SLAVES_STATECHANGED_NTFY_DESC_ENTRY(ctypes.Structure):
    _fields_ = [
        ("wStationAddress", ctypes.c_ushort), # ushort
        ("byState", ctypes.c_ubyte), # byte
    ]

class SDN_EC_T_SLAVES_STATECHANGED_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("wCount", ctypes.c_ushort), # ushort
        ("SlaveStates", SDN_EC_T_SLAVES_STATECHANGED_NTFY_DESC_ENTRY * 128), # SDN_EC_T_SLAVES_STATECHANGED_NTFY_DESC_ENTRY[128]
    ]

class SDN_EC_T_FRAMELOSS_AFTER_SLAVE_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
        ("wPort", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_BAD_CONNECTION_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("SlavePropParent", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
        ("wPortAtParent", ctypes.c_ushort), # ushort
        ("SlavePropChild", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
        ("wPortAtChild", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_COMMUNICATION_TIMEOUT_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("bMainTapPortIn", ctypes.c_uint), # bool
        ("bMainTapPortOut", ctypes.c_uint), # bool
    ]

class SDN_EC_T_TAP_LINK_STATUS_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("bLinkConnected", ctypes.c_uint), # bool
    ]

class SDN_EC_T_SB_STATUS_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("dwResultCode", ctypes.c_uint), # uint
        ("dwSlaveCount", ctypes.c_uint), # uint
    ]

class SDN_EC_T_SB_MISMATCH_DESC(ctypes.Structure):
    _fields_ = [
        ("wPrevFixedAddress", ctypes.c_ushort), # ushort
        ("wPrevPort", ctypes.c_ushort), # ushort
        ("wPrevAIncAddress", ctypes.c_ushort), # ushort
        ("wBusAIncAddress", ctypes.c_ushort), # ushort
        ("dwBusVendorId", ctypes.c_uint), # uint
        ("dwBusProdCode", ctypes.c_uint), # uint
        ("dwBusRevisionNo", ctypes.c_uint), # uint
        ("dwBusSerialNo", ctypes.c_uint), # uint
        ("wBusFixedAddress", ctypes.c_ushort), # ushort
        ("wIdentificationVal", ctypes.c_ushort), # ushort
        ("wCfgFixedAddress", ctypes.c_ushort), # ushort
        ("wCfgAIncAddress", ctypes.c_ushort), # ushort
        ("dwCfgVendorId", ctypes.c_uint), # uint
        ("dwCfgProdCode", ctypes.c_uint), # uint
        ("dwCfgRevisionNo", ctypes.c_uint), # uint
        ("dwCfgSerialNo", ctypes.c_uint), # uint
        ("bIdentValidationError", ctypes.c_uint), # bool
        ("oIdentCmdHdr", ctypes.c_uint16 * 5), # ushort[5]
        ("dwCmdData", ctypes.c_uint), # uint
        ("dwCmdVMask", ctypes.c_uint), # uint
        ("dwCmdVData", ctypes.c_uint), # uint
    ]

class SDN_EC_T_LINE_CROSSED_DESC(ctypes.Structure):
    _fields_ = [
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
        ("wInputPort", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_HC_DETECTALLGROUP_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("dwResultCode", ctypes.c_uint), # uint
        ("dwGroupCount", ctypes.c_uint), # uint
        ("dwGroupsPresent", ctypes.c_uint), # uint
        ("dwGroupMask", ctypes.c_uint), # uint
        ("adwGroupMask", ctypes.c_uint * 100), # uint[100]
    ]

class SDN_EC_T_RAWCMDRESPONSE_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("dwInvokeId", ctypes.c_uint), # uint
        ("dwResult", ctypes.c_uint), # uint
        ("dwWkc", ctypes.c_uint), # uint
        ("dwCmdIdx", ctypes.c_uint), # uint
        ("dwAddr", ctypes.c_uint), # uint
        ("dwLength", ctypes.c_uint), # uint
        ("pbyData", ctypes.c_void_p), # IntPtr
    ]

class SDN_EC_T_TX_PDO_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("wPhysAddr", ctypes.c_uint), # uint
        ("dwNumber", ctypes.c_uint), # uint
        ("wLen", ctypes.c_uint), # uint
        ("pbyData", ctypes.c_void_p), # IntPtr
    ]

class SDN_EC_T_STATECHANGE(ctypes.Structure):
    _fields_ = [
        ("oldState", ctypes.c_uint), # DN_EC_T_STATE
        ("newState", ctypes.c_uint), # DN_EC_T_STATE
    ]

class SDN_EC_T_SLAVEREGISTER_TRANSFER_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("dwTferId", ctypes.c_uint), # uint
        ("dwResult", ctypes.c_uint), # uint
        ("bRead", ctypes.c_uint), # bool
        ("wFixedAddr", ctypes.c_ushort), # ushort
        ("wRegisterOffset", ctypes.c_ushort), # ushort
        ("wLen", ctypes.c_ushort), # ushort
        ("pbyData", ctypes.c_void_p), # IntPtr
        ("wWkc", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_PORT_OPERATION_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("dwTferId", ctypes.c_uint), # uint
        ("dwResult", ctypes.c_uint), # uint
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
        ("wPortStateOld", ctypes.c_ushort), # ushort
        ("wPortStateNew", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_SLAVE_IDENTIFICATION_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("dwTferId", ctypes.c_uint), # uint
        ("dwResult", ctypes.c_uint), # uint
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
        ("wAdo", ctypes.c_ushort), # ushort
        ("wValue", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_RELEASE_FORCED_PROCESSDATA_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("bOutput", ctypes.c_uint), # bool
        ("dwOffset", ctypes.c_uint), # uint
        ("wBitLength", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_SLAVE_PRESENCE_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("wStationAddress", ctypes.c_ushort), # ushort
        ("bPresent", ctypes.c_ubyte), # byte
    ]

class SDN_EC_T_SLAVES_PRESENCE_NTFY_DESC(ctypes.Structure):
    _fields_ = [
        ("wCount", ctypes.c_ushort), # ushort
        ("SlavePresence", SDN_EC_T_SLAVE_PRESENCE_NTFY_DESC * 128), # SDN_EC_T_SLAVE_PRESENCE_NTFY_DESC[128]
    ]

class SDN_EC_T_S2SMBX_ERROR_DESC(ctypes.Structure):
    _fields_ = [
        ("SlaveProp", SDN_EC_T_SLAVE_PROP), # SDN_EC_T_SLAVE_PROP
        ("wTargetFixedAddress", ctypes.c_ushort), # ushort
        ("dwErrorCode", ctypes.c_uint), # uint
    ]

class SDN_EC_T_SB_SLAVEINFO_REQ_DESC(ctypes.Structure):
    _fields_ = [
        ("eInfoEntry", ctypes.c_uint), # DN_EC_T_eINFOENTRY
        ("wAutoIncAddress", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_SB_SLAVEINFO_RES_DESC(ctypes.Structure):
    _fields_ = [
        ("eInfoEntry", ctypes.c_uint), # DN_EC_T_eINFOENTRY
        ("dwInfoLength", ctypes.c_uint), # uint
        ("pbyInfo", ctypes.c_void_p), # IntPtr
    ]

class SDN_EC_T_SLVSTATISTICS_DESC(ctypes.Structure):
    _fields_ = [
        ("abyInvalidFrameCnt", ctypes.c_uint8 * 4), # byte[4]
        ("abyRxErrorCnt", ctypes.c_uint8 * 4), # byte[4]
        ("abyFwdRxErrorCnt", ctypes.c_uint8 * 4), # byte[4]
        ("byProcessingUnitErrorCnt", ctypes.c_ubyte), # byte
        ("byPdiErrorCnt", ctypes.c_ubyte), # byte
        ("wAlStatusCode", ctypes.c_ushort), # ushort
        ("abyLostLinkCnt", ctypes.c_uint8 * 4), # byte[4]
        ("qwReadTime", ctypes.c_uint64), # uint64
        ("qwChangeTime", ctypes.c_uint64), # uint64
    ]

class SDN_EC_T_CFG_SLAVE_INFO(ctypes.Structure):
    _fields_ = [
        ("dwSlaveId", ctypes.c_uint), # uint
        ("abyDeviceName", ctypes.c_char * 80), # string[80]
        ("dwHCGroupIdx", ctypes.c_uint), # uint
        ("bIsPresent", ctypes.c_uint), # bool
        ("bIsHCGroupPresent", ctypes.c_uint), # bool
        ("dwVendorId", ctypes.c_uint), # uint
        ("dwProductCode", ctypes.c_uint), # uint
        ("dwRevisionNumber", ctypes.c_uint), # uint
        ("dwSerialNumber", ctypes.c_uint), # uint
        ("wStationAddress", ctypes.c_ushort), # ushort
        ("wAutoIncAddress", ctypes.c_ushort), # ushort
        ("dwPdOffsIn", ctypes.c_uint), # uint
        ("dwPdSizeIn", ctypes.c_uint), # uint
        ("dwPdOffsOut", ctypes.c_uint), # uint
        ("dwPdSizeOut", ctypes.c_uint), # uint
        ("dwPdOffsIn2", ctypes.c_uint), # uint
        ("dwPdSizeIn2", ctypes.c_uint), # uint
        ("dwPdOffsOut2", ctypes.c_uint), # uint
        ("dwPdSizeOut2", ctypes.c_uint), # uint
        ("dwPdOffsIn3", ctypes.c_uint), # uint
        ("dwPdSizeIn3", ctypes.c_uint), # uint
        ("dwPdOffsOut3", ctypes.c_uint), # uint
        ("dwPdSizeOut3", ctypes.c_uint), # uint
        ("dwPdOffsIn4", ctypes.c_uint), # uint
        ("dwPdSizeIn4", ctypes.c_uint), # uint
        ("dwPdOffsOut4", ctypes.c_uint), # uint
        ("dwPdSizeOut4", ctypes.c_uint), # uint
        ("dwMbxSupportedProtocols", ctypes.c_uint), # uint
        ("dwMbxOutSize", ctypes.c_uint), # uint
        ("dwMbxInSize", ctypes.c_uint), # uint
        ("dwMbxOutSize2", ctypes.c_uint), # uint
        ("dwMbxInSize2", ctypes.c_uint), # uint
        ("bDcSupport", ctypes.c_uint), # bool
        ("wNumProcessVarsInp", ctypes.c_ushort), # ushort
        ("wNumProcessVarsOutp", ctypes.c_ushort), # ushort
        ("wPrevStationAddress", ctypes.c_ushort), # ushort
        ("wPrevPort", ctypes.c_ushort), # ushort
        ("wIdentifyAdo", ctypes.c_ushort), # ushort
        ("wIdentifyData", ctypes.c_ushort), # ushort
        ("byPortDescriptor", ctypes.c_ubyte), # byte
        ("wWkcStateDiagOffsIn", ctypes.c_uint16 * 4), # ushort[4]
        ("wWkcStateDiagOffsOut", ctypes.c_uint16 * 4), # ushort[4]
        ("awMasterSyncUnitIn", ctypes.c_uint16 * 4), # ushort[4]
        ("awMasterSyncUnitOut", ctypes.c_uint16 * 4), # ushort[4]
        ("bDisabled", ctypes.c_uint), # bool
        ("bDisconnected", ctypes.c_uint), # bool
        ("bExtended", ctypes.c_uint), # bool
    ]

class SDN_EC_T_PROFILE_CHANNEL_INFO(ctypes.Structure):
    _fields_ = [
        ("wProfileNo", ctypes.c_ushort), # ushort
        ("wAddInfo", ctypes.c_ushort), # ushort
        ("szDisplayName", ctypes.c_char * 80), # string[80]
    ]

class SDN_EC_T_CFG_SLAVE_EOE_INFO(ctypes.Structure):
    _fields_ = [
        ("dwSlaveId", ctypes.c_uint), # uint
        ("bMacAddr", ctypes.c_uint), # bool
        ("abyMacAddr", ctypes.c_uint8 * 6), # byte[6]
        ("bIpAddr", ctypes.c_uint), # bool
        ("abyIpAddr", ctypes.c_uint8 * 4), # byte[4]
        ("bSubnetMask", ctypes.c_uint), # bool
        ("abySubnetMask", ctypes.c_uint8 * 4), # byte[4]
        ("bDefaultGateway", ctypes.c_uint), # bool
        ("abyDefaultGateway", ctypes.c_uint8 * 4), # byte[4]
        ("bDnsServer", ctypes.c_uint), # bool
        ("abyDnsServer", ctypes.c_uint8 * 4), # byte[4]
        ("bDnsName", ctypes.c_uint), # bool
        ("szDnsName", ctypes.c_char * 32), # string[32]
    ]

class SDN_EC_T_BUS_SLAVE_INFO(ctypes.Structure):
    _fields_ = [
        ("dwSlaveId", ctypes.c_uint), # uint
        ("adwPortSlaveIds", ctypes.c_uint * 4), # uint[4]
        ("wPortState", ctypes.c_ushort), # ushort
        ("wAutoIncAddress", ctypes.c_ushort), # ushort
        ("bDcSupport", ctypes.c_uint), # bool
        ("bDc64Support", ctypes.c_uint), # bool
        ("dwVendorId", ctypes.c_uint), # uint
        ("dwProductCode", ctypes.c_uint), # uint
        ("dwRevisionNumber", ctypes.c_uint), # uint
        ("dwSerialNumber", ctypes.c_uint), # uint
        ("byESCType", ctypes.c_ubyte), # byte
        ("byESCRevision", ctypes.c_ubyte), # byte
        ("wESCBuild", ctypes.c_ushort), # ushort
        ("byPortDescriptor", ctypes.c_ubyte), # byte
        ("wFeaturesSupported", ctypes.c_ushort), # ushort
        ("wStationAddress", ctypes.c_ushort), # ushort
        ("wAliasAddress", ctypes.c_ushort), # ushort
        ("wAlStatus", ctypes.c_ushort), # ushort
        ("wAlStatusCode", ctypes.c_ushort), # ushort
        ("dwSystemTimeDifference", ctypes.c_uint), # uint
        ("wMbxSupportedProtocols", ctypes.c_ushort), # ushort
        ("wDlStatus", ctypes.c_ushort), # ushort
        ("wPrevPort", ctypes.c_ushort), # ushort
        ("wIdentifyData", ctypes.c_ushort), # ushort
        ("bLineCrossed", ctypes.c_uint), # bool
        ("dwSlaveDelay", ctypes.c_uint), # uint
        ("dwPropagDelay", ctypes.c_uint), # uint
        ("bIsRefClock", ctypes.c_uint), # bool
        ("bIsDeviceEmulation", ctypes.c_uint), # bool
        ("wLineCrossedFlags", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_TRACE_DATA_INFO(ctypes.Structure):
    _fields_ = [
        ("pbyData", ctypes.c_void_p), # IntPtr
        ("dwOffset", ctypes.c_uint), # uint
        ("wSize", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_BUS_DIAGNOSIS_INFO(ctypes.Structure):
    _fields_ = [
        ("dwCRC32ConfigCheckSum", ctypes.c_uint), # uint
        ("dwNumSlavesFound", ctypes.c_uint), # uint
        ("dwNumDCSlavesFound", ctypes.c_uint), # uint
        ("dwNumCfgSlaves", ctypes.c_uint), # uint
        ("dwNumMbxSlaves", ctypes.c_uint), # uint
        ("dwTXFrames", ctypes.c_uint), # uint
        ("dwRXFrames", ctypes.c_uint), # uint
        ("dwLostFrames", ctypes.c_uint), # uint
        ("dwCyclicFrames", ctypes.c_uint), # uint
        ("dwCyclicDatagrams", ctypes.c_uint), # uint
        ("dwAcyclicFrames", ctypes.c_uint), # uint
        ("dwAcyclicDatagrams", ctypes.c_uint), # uint
        ("dwClearCounters", ctypes.c_uint), # uint
        ("dwCyclicLostFrames", ctypes.c_uint), # uint
        ("dwAcyclicLostFrames", ctypes.c_uint), # uint
        ("dwRes", ctypes.c_uint * 2), # uint[2]
    ]

class SDN_EC_T_REDUNDANCY_DIAGNOSIS_INFO(ctypes.Structure):
    _fields_ = [
        ("bRedEnabled", ctypes.c_uint), # bool
        ("dwMainSlaveCnt", ctypes.c_uint), # uint
        ("dwRedSlaveCnt", ctypes.c_uint), # uint
        ("bLineBreakDetected", ctypes.c_uint), # bool
        ("dwRes", ctypes.c_uint * 4), # uint[4]
    ]

class SDN_EC_T_STATISTIC(ctypes.Structure):
    _fields_ = [
        ("dwTotal", ctypes.c_uint), # uint
        ("dwLast", ctypes.c_uint), # uint
    ]

class SDN_EC_T_STATISTIC_TRANSFER(ctypes.Structure):
    _fields_ = [
        ("Cnt", SDN_EC_T_STATISTIC), # SDN_EC_T_STATISTIC
        ("Bytes", SDN_EC_T_STATISTIC), # SDN_EC_T_STATISTIC
    ]

class SDN_EC_T_STATISTIC_TRANSFER_DUPLEX(ctypes.Structure):
    _fields_ = [
        ("Read", SDN_EC_T_STATISTIC_TRANSFER), # SDN_EC_T_STATISTIC_TRANSFER
        ("Write", SDN_EC_T_STATISTIC_TRANSFER), # SDN_EC_T_STATISTIC_TRANSFER
    ]

class SDN_EC_T_MAILBOX_STATISTICS(ctypes.Structure):
    _fields_ = [
        ("Aoe", SDN_EC_T_STATISTIC_TRANSFER_DUPLEX), # SDN_EC_T_STATISTIC_TRANSFER_DUPLEX
        ("Coe", SDN_EC_T_STATISTIC_TRANSFER_DUPLEX), # SDN_EC_T_STATISTIC_TRANSFER_DUPLEX
        ("Eoe", SDN_EC_T_STATISTIC_TRANSFER_DUPLEX), # SDN_EC_T_STATISTIC_TRANSFER_DUPLEX
        ("Foe", SDN_EC_T_STATISTIC_TRANSFER_DUPLEX), # SDN_EC_T_STATISTIC_TRANSFER_DUPLEX
        ("Soe", SDN_EC_T_STATISTIC_TRANSFER_DUPLEX), # SDN_EC_T_STATISTIC_TRANSFER_DUPLEX
        ("Voe", SDN_EC_T_STATISTIC_TRANSFER_DUPLEX), # SDN_EC_T_STATISTIC_TRANSFER_DUPLEX
        ("RawMbx", SDN_EC_T_STATISTIC_TRANSFER_DUPLEX), # SDN_EC_T_STATISTIC_TRANSFER_DUPLEX
        ("aRes", SDN_EC_T_STATISTIC_TRANSFER_DUPLEX), # SDN_EC_T_STATISTIC_TRANSFER_DUPLEX
    ]

class SDN_EC_T_MASTER_INFO(ctypes.Structure):
    _fields_ = [
        ("dwMasterVersion", ctypes.c_uint), # uint
        ("BusDiagnosisInfo", SDN_EC_T_BUS_DIAGNOSIS_INFO), # SDN_EC_T_BUS_DIAGNOSIS_INFO
        ("MailboxStatistics", SDN_EC_T_MAILBOX_STATISTICS), # SDN_EC_T_MAILBOX_STATISTICS
        ("RedundancyDiagnosisInfo", SDN_EC_T_REDUNDANCY_DIAGNOSIS_INFO), # SDN_EC_T_REDUNDANCY_DIAGNOSIS_INFO
        ("dwMasterStateSummary", ctypes.c_uint), # uint
    ]

class SDN_EC_T_RAS_CONNECTION_INFO(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_MSU_INFO(ctypes.Structure):
    _fields_ = [
        ("wMsuId", ctypes.c_ushort), # ushort
        ("dwBitOffsIn", ctypes.c_uint), # uint
        ("dwBitSizeIn", ctypes.c_uint), # uint
        ("dwBitOffsOut", ctypes.c_uint), # uint
        ("dwBitSizeOut", ctypes.c_uint), # uint
        ("wWkcStateDiagOffsIn", ctypes.c_ushort), # ushort
        ("wWkcStateDiagOffsOut", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_REGISTERRESULTS(ctypes.Structure):
    _fields_ = [
        ("dwClntId", ctypes.c_uint), # uint
        ("pbyPDIn", ctypes.c_void_p), # IntPtr
        ("dwPDInSize", ctypes.c_uint), # uint
        ("pbyPDOut", ctypes.c_void_p), # IntPtr
        ("dwPDOutSize", ctypes.c_uint), # uint
    ]

class SDN_EC_T_MASTER_RED_PARMS(ctypes.Structure):
    _fields_ = [
        ("bEnabled", ctypes.c_uint), # bool
        ("wMasterPdOutSize", ctypes.c_ushort), # ushort
        ("wMasterPdInSize", ctypes.c_ushort), # ushort
        ("dwMaxAcycFramesPerCycle", ctypes.c_uint), # uint
        ("bUpdateSlavePdOut", ctypes.c_uint), # bool
        ("bUpdateSlavePdIn", ctypes.c_uint), # bool
    ]

class SDN_EC_T_DC_CONFIGURE(ctypes.Structure):
    _fields_ = [
        ("dwClntId", ctypes.c_uint), # uint
        ("dwTimeout", ctypes.c_uint), # uint
        ("dwDevLimit", ctypes.c_uint), # uint
        ("dwSettleTime", ctypes.c_uint), # uint
        ("dwTotalBurstLength", ctypes.c_uint), # uint
        ("dwBurstBulk", ctypes.c_uint), # uint
        ("bBulkInLinkLayer", ctypes.c_uint), # bool
        ("bAcycDistributionDisabled", ctypes.c_uint), # bool
        ("dwDcStartTimeGrid", ctypes.c_uint), # uint
        ("bDcInitBeforeSlaveStateChange", ctypes.c_uint), # bool
    ]

class SDN_EC_T_DC_STARTTIME_CB_DESC(ctypes.Structure):
    _fields_ = [
        ("pvContext", ctypes.c_void_p), # IntPtr
    ]

class SDN_EC_T_DCM_CONFIG_BUSSHIFT(ctypes.Structure):
    _fields_ = [
        ("nCtlSetVal", ctypes.c_int), # int
        ("nCtlGain", ctypes.c_int), # int
        ("nCtlDriftErrorGain", ctypes.c_int), # int
        ("nMaxValidVal", ctypes.c_int), # int
        ("bLogEnabled", ctypes.c_uint), # bool
        ("dwInSyncLimit", ctypes.c_uint), # uint
        ("dwInSyncSettleTime", ctypes.c_uint), # uint
        ("bCtlOff", ctypes.c_uint), # bool
        ("bUseDcLoopCtlStdValues", ctypes.c_uint), # bool
        ("dwInSyncStartDelayCycle", ctypes.c_uint), # uint
    ]

class SDN_EC_T_DCM_CONFIG_MASTERSHIFT(ctypes.Structure):
    _fields_ = [
        ("nCtlSetVal", ctypes.c_int), # int
        ("nCtlGain", ctypes.c_int), # int
        ("nCtlDriftErrorGain", ctypes.c_int), # int
        ("nMaxValidVal", ctypes.c_int), # int
        ("bLogEnabled", ctypes.c_uint), # bool
        ("dwInSyncLimit", ctypes.c_uint), # uint
        ("dwInSyncSettleTime", ctypes.c_uint), # uint
        ("bCtlOff", ctypes.c_uint), # bool
        ("dwInSyncStartDelayCycle", ctypes.c_uint), # uint
    ]

class SDN_EC_T_DCM_CONFIG_LINKLAYERREFCLOCK(ctypes.Structure):
    _fields_ = [
        ("nCtlSetVal", ctypes.c_int), # int
        ("bLogEnabled", ctypes.c_uint), # bool
        ("DcStartTimeCallbackDesc", SDN_EC_T_DC_STARTTIME_CB_DESC), # SDN_EC_T_DC_STARTTIME_CB_DESC
    ]

class SDN_EC_T_DCM_CONFIG_MASTERREFCLOCK(ctypes.Structure):
    _fields_ = [
        ("nCtlSetVal", ctypes.c_int), # int
        ("bLogEnabled", ctypes.c_uint), # bool
        ("dwInSyncLimit", ctypes.c_uint), # uint
        ("dwInSyncSettleTime", ctypes.c_uint), # uint
        ("dwInSyncStartDelayCycle", ctypes.c_uint), # uint
    ]

class SDN_EC_T_DCM_CONFIG_DCX(ctypes.Structure):
    _fields_ = [
        ("MasterShift", SDN_EC_T_DCM_CONFIG_MASTERSHIFT), # SDN_EC_T_DCM_CONFIG_MASTERSHIFT
        ("nCtlSetVal", ctypes.c_int), # int
        ("nCtlGain", ctypes.c_int), # int
        ("nCtlDriftErrorGain", ctypes.c_int), # int
        ("nMaxValidVal", ctypes.c_int), # int
        ("bLogEnabled", ctypes.c_uint), # bool
        ("dwInSyncLimit", ctypes.c_uint), # uint
        ("dwInSyncSettleTime", ctypes.c_uint), # uint
        ("bCtlOff", ctypes.c_uint), # bool
        ("wExtClockFixedAddr", ctypes.c_ushort), # ushort
        ("dwExtClockTimeout", ctypes.c_uint), # uint
        ("dwInSyncStartDelayCycle", ctypes.c_uint), # uint
        ("dwMaxErrCompensableOnExtClockReconnect", ctypes.c_uint), # uint
    ]

class SDN_EC_T_DCM_CONFIG(ctypes.Structure):
    _fields_ = [
        ("eMode", ctypes.c_uint), # DN_EC_T_DCM_MODE
        ("BusShift", SDN_EC_T_DCM_CONFIG_BUSSHIFT), # SDN_EC_T_DCM_CONFIG_BUSSHIFT
        ("MasterShift", SDN_EC_T_DCM_CONFIG_MASTERSHIFT), # SDN_EC_T_DCM_CONFIG_MASTERSHIFT
        ("LinkLayerRefClock", SDN_EC_T_DCM_CONFIG_LINKLAYERREFCLOCK), # SDN_EC_T_DCM_CONFIG_LINKLAYERREFCLOCK
        ("MasterRefClock", SDN_EC_T_DCM_CONFIG_MASTERREFCLOCK), # SDN_EC_T_DCM_CONFIG_MASTERREFCLOCK
        ("Dcx", SDN_EC_T_DCM_CONFIG_DCX), # SDN_EC_T_DCM_CONFIG_DCX
    ]

class SDN_EC_T_GET_SLAVE_INFO(ctypes.Structure):
    _fields_ = [
        ("dwScanBusStatus", ctypes.c_uint), # uint
        ("dwVendorId", ctypes.c_uint), # uint
        ("dwProductCode", ctypes.c_uint), # uint
        ("dwRevisionNumber", ctypes.c_uint), # uint
        ("dwSerialNumber", ctypes.c_uint), # uint
        ("wPortState", ctypes.c_ushort), # ushort
        ("bDcSupport", ctypes.c_uint), # bool
        ("bDc64Support", ctypes.c_uint), # bool
        ("wAliasAddress", ctypes.c_ushort), # ushort
        ("wPhysAddress", ctypes.c_ushort), # ushort
        ("dwPdOffsIn", ctypes.c_uint), # uint
        ("dwPdSizeIn", ctypes.c_uint), # uint
        ("dwPdOffsOut", ctypes.c_uint), # uint
        ("dwPdSizeOut", ctypes.c_uint), # uint
        ("dwPdOffsIn2", ctypes.c_uint), # uint
        ("dwPdSizeIn2", ctypes.c_uint), # uint
        ("dwPdOffsOut2", ctypes.c_uint), # uint
        ("dwPdSizeOut2", ctypes.c_uint), # uint
        ("dwPdOffsIn3", ctypes.c_uint), # uint
        ("dwPdSizeIn3", ctypes.c_uint), # uint
        ("dwPdOffsOut3", ctypes.c_uint), # uint
        ("dwPdSizeOut3", ctypes.c_uint), # uint
        ("dwPdOffsIn4", ctypes.c_uint), # uint
        ("dwPdSizeIn4", ctypes.c_uint), # uint
        ("dwPdOffsOut4", ctypes.c_uint), # uint
        ("dwPdSizeOut4", ctypes.c_uint), # uint
        ("wCfgPhyAddress", ctypes.c_ushort), # ushort
        ("abyDeviceName", ctypes.c_char * 80), # string[80]
        ("bIsMailboxSlave", ctypes.c_uint), # bool
        ("dwMbxOutSize", ctypes.c_uint), # uint
        ("dwMbxInSize", ctypes.c_uint), # uint
        ("dwMbxOutSize2", ctypes.c_uint), # uint
        ("dwMbxInSize2", ctypes.c_uint), # uint
        ("dwErrorCode", ctypes.c_uint), # uint
        ("dwSBErrorCode", ctypes.c_uint), # uint
        ("byPortDescriptor", ctypes.c_ubyte), # byte
        ("byESCType", ctypes.c_ubyte), # byte
        ("wSupportedMbxProtocols", ctypes.c_ushort), # ushort
        ("wAlStatusValue", ctypes.c_ushort), # ushort
        ("wAlStatusCode", ctypes.c_ushort), # ushort
        ("bIsOptional", ctypes.c_uint), # bool
        ("bIsPresent", ctypes.c_uint), # bool
        ("wNumProcessVarsInp", ctypes.c_ushort), # ushort
        ("wNumProcessVarsOutp", ctypes.c_ushort), # ushort
        ("dwSlaveId", ctypes.c_uint), # uint
        ("bIsHCGroupPresent", ctypes.c_uint), # bool
        ("aPortSlaveIds", ctypes.c_uint * 4), # uint[4]
        ("dwSystemTimeDifference", ctypes.c_uint), # uint
    ]

class SDN_EC_T_PROCESS_VAR_INFO(ctypes.Structure):
    _fields_ = [
        ("szName", ctypes.c_char * 72), # string[72]
        ("wDataType", ctypes.c_ushort), # ushort
        ("wFixedAddr", ctypes.c_ushort), # ushort
        ("nBitSize", ctypes.c_int), # int
        ("nBitOffs", ctypes.c_int), # int
        ("bIsInputData", ctypes.c_uint), # bool
    ]

class SDN_EC_T_PROCESS_VAR_INFO_EX(ctypes.Structure):
    _fields_ = [
        ("szName", ctypes.c_char * 128), # string[128]
        ("wDataType", ctypes.c_ushort), # ushort
        ("wFixedAddr", ctypes.c_ushort), # ushort
        ("nBitSize", ctypes.c_int), # int
        ("nBitOffs", ctypes.c_int), # int
        ("bIsInputData", ctypes.c_uint), # bool
        ("wIndex", ctypes.c_ushort), # ushort
        ("wSubIndex", ctypes.c_ushort), # ushort
        ("wPdoIndex", ctypes.c_ushort), # ushort
        ("wWkcStateDiagOffs", ctypes.c_ushort), # ushort
        ("wMasterSyncUnit", ctypes.c_ushort), # ushort
        ("wRes1", ctypes.c_ushort), # ushort
        ("dwRes1", ctypes.c_uint), # uint
    ]

class SDN_EC_T_COE_EMERGENCY(ctypes.Structure):
    _fields_ = [
        ("wErrorCode", ctypes.c_ushort), # ushort
        ("byErrorRegister", ctypes.c_ubyte), # byte
        ("abyData", ctypes.c_uint8 * 5), # byte[5]
        ("wStationAddress", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_MBX_DATA_COE(ctypes.Structure):
    _fields_ = [
        ("wStationAddress", ctypes.c_ushort), # ushort
        ("wIndex", ctypes.c_ushort), # ushort
        ("bySubIndex", ctypes.c_ubyte), # byte
        ("bCompleteAccess", ctypes.c_uint), # bool
    ]

class SDN_EC_T_MBX_DATA_FOE(ctypes.Structure):
    _fields_ = [
        ("dwTransferredBytes", ctypes.c_uint), # uint
        ("dwRequestedBytes", ctypes.c_uint), # uint
        ("dwBusyDone", ctypes.c_uint), # uint
        ("dwBusyEntire", ctypes.c_uint), # uint
        ("szBusyComment", ctypes.c_char * 32), # string[32]
        ("dwFileSize", ctypes.c_uint), # uint
        ("wStationAddress", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_SOE_NOTIFICATION(ctypes.Structure):
    _fields_ = [
        ("wHeader", ctypes.c_ushort), # ushort
        ("wIdn", ctypes.c_ushort), # ushort
        ("abyData", ctypes.c_uint8 * 5), # byte[5]
        ("wStationAddress", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_SOE_EMERGENCY(ctypes.Structure):
    _fields_ = [
        ("wHeader", ctypes.c_ushort), # ushort
        ("abyData", ctypes.c_uint8 * 5), # byte[5]
        ("wStationAddress", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_AOE_NETID(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_AOE_CMD_RESPONSE(ctypes.Structure):
    _fields_ = [
        ("dwErrorCode", ctypes.c_uint), # uint
        ("dwCmdResult", ctypes.c_uint), # uint
        ("dwRsvd", ctypes.c_uint), # uint
    ]

class SDN_EC_T_ADS_ADAPTER_START_PARMS(ctypes.Structure):
    _fields_ = [
        ("dwSignature", ctypes.c_uint), # uint
        ("dwSize", ctypes.c_uint), # uint
        ("cpuAffinityMask", ctypes.c_uint64), # uint64
        ("dwThreadPriority", ctypes.c_uint), # uint
        ("targetNetID", SDN_EC_T_AOE_NETID), # SDN_EC_T_AOE_NETID
        ("targetPort", ctypes.c_ushort), # ushort
    ]

class SDN_EC_T_SELFTESTSCAN_PARMS(ctypes.Structure):
    _fields_ = [
        ("dwTimeout", ctypes.c_uint), # uint
        ("dwFrameCount", ctypes.c_uint), # uint
        ("dwFrameSizeMin", ctypes.c_uint), # uint
        ("dwFrameSizeMax", ctypes.c_uint), # uint
        ("dwFrameSizeStep", ctypes.c_uint), # uint
        ("bDetectBadConnections", ctypes.c_uint), # bool
    ]

class SDN_ETHERNET_ADDRESS(ctypes.Structure):
    _fields_ = [
        ("b", ctypes.c_uint8 * 6), # byte[6]
    ]

class SDN_EC_T_LOG_PARMS(ctypes.Structure):
    _fields_ = [
        ("dwLogLevel", ctypes.c_uint), # uint
    ]

class SDN_EC_T_DAQ_REC_STATISTIC(ctypes.Structure):
    _fields_ = [
        ("bStarted", ctypes.c_uint), # bool
        ("qwCycles", ctypes.c_uint64), # uint64
        ("dwTriggers", ctypes.c_uint), # uint
    ]

class SDN_EC_T_DAQ_MEMORY_INFO(ctypes.Structure):
    _fields_ = [
        ("dwCycleCount", ctypes.c_uint), # uint
        ("dwVariablesCount", ctypes.c_uint), # uint
        ("dwDataEntrySize", ctypes.c_uint), # uint
    ]

class SDN_EC_T_DAQ_MEMORY_VARIABLE(ctypes.Structure):
    _fields_ = [
        ("szName", ctypes.c_char * 128), # string[128]
        ("wDataType", ctypes.c_ushort), # ushort
        ("nBitOffs", ctypes.c_int), # int
        ("nBitSize", ctypes.c_int), # int
    ]

class SDN_EC_T_DAQ_READER_INFO(ctypes.Structure):
    _fields_ = [
        ("dwGroupCount", ctypes.c_uint), # uint
        ("dwVariablesCount", ctypes.c_uint), # uint
        ("dwRecordDataCount", ctypes.c_uint), # uint
        ("dwRecordDataSize", ctypes.c_uint), # uint
    ]

class SDN_EC_T_DAQ_READER_GROUP(ctypes.Structure):
    _fields_ = [
        ("szName", ctypes.c_char * 128), # string[128]
    ]

class SDN_EC_T_DAQ_READER_VARIABLE(ctypes.Structure):
    _fields_ = [
        ("szName", ctypes.c_char * 128), # string[128]
        ("wDataType", ctypes.c_ushort), # ushort
        ("nBitOffs", ctypes.c_int), # int
        ("nBitSize", ctypes.c_int), # int
        ("nGroupIndex", ctypes.c_int), # int
    ]

class SDN_EC_T_WORKER_THREAD_PARMS(ctypes.Structure):
    _fields_ = [
        ("dwPrio", ctypes.c_uint), # uint
        ("cpuAffinityMask", ctypes.c_uint64), # uint64
    ]

class SDN_EC_T_PACKETCAPTURE_INFO(ctypes.Structure):
    _fields_ = [
        ("eStatus", ctypes.c_uint), # DN_EC_T_PACKETCAPTURE_STATUS
        ("szFileName", ctypes.c_char * (255 + 1)), # string[(255 + 1)]
        ("qwFrameNumberTotal", ctypes.c_uint64), # uint64
        ("qwFrameNumberCur", ctypes.c_uint64), # uint64
        ("qwBytesProcessed", ctypes.c_uint64), # uint64
        ("qwFileSize", ctypes.c_uint64), # uint64
        ("qwTimeStamp", ctypes.c_uint64), # uint64
        ("dwCyclesProcessed", ctypes.c_uint), # uint
    ]

class SDN_EC_T_PACKETCAPTURE_PARMS(ctypes.Structure):
    _fields_ = [
        ("szFileName", ctypes.c_char * (255 + 1)), # string[(255 + 1)]
        ("bReadMultipleFiles", ctypes.c_uint), # bool
        ("dwMaxFrameCnt", ctypes.c_uint), # uint
        ("dwMaxFileSize", ctypes.c_uint), # uint
        ("dwRingBufferFileCnt", ctypes.c_uint), # uint
    ]

class SDN_EC_T_MONITOR_STATUS(ctypes.Structure):
    _fields_ = [
        ("bNextFramesReceived", ctypes.c_uint), # bool
        ("dwCyclesProcessed", ctypes.c_uint), # uint
        ("wEthTapPositionAutoIncAddr", ctypes.c_ushort), # ushort
        ("bNextCyclicEntryReceived", ctypes.c_uint), # bool
    ]

#// @CODEGENERATOR_IMPL_END@

class NotificationDataType(Enum):
    Default = 0
    Error = 1
    Notify = 2

class SDN_ATEMRAS_T_CLNTPARMS(ctypes.Structure):
    _fields_ = [
        ("cpuAffinityMask", ctypes.c_uint64),    #/**< [in]   CPU affinity mask */
        ("dwAdmPrio", ctypes.c_uint),            #/**< [in]   Priority of Administrative task */
        ("dwAdmStackSize", ctypes.c_uint),       #/**< [in]   Stack size of Administrative task */
        ("pvNotifCtxt", ctypes.c_void_p),         #/**< [in]   Notification context returned while calling pfNotification */
        ("pfNotification", ctypes.c_void_p),      #/**< [in]   Function pointer called to notify error and status
                                                    #//*         information generated by Remote API Layer */
        #  /* logging */
        ("dwLogLevel", ctypes.c_uint),           #/**< [in] log level. See EC_LOG_LEVEL_... */
        ("pfLogMsgCallBack", ctypes.c_void_p),    #/**< [in] optional call back function to log msg from the RAS Client. set to EC_NULL if not used. */
    ]

class SDN_EC_T_MBX_GATEWAY_CLNT_PARMS(ctypes.Structure):
    _fields_ = [
        ("dwSignature", ctypes.c_uint),   #/**< [in]   Set to EC_MBX_GATEWAY_CLNT_SIGNATURE */
        ("dwSize", ctypes.c_uint),            #/**< [in]   Set to sizeof(EC_T_MBX_GATEWAY_CLNT_PARAMS) */
        ("dwLogLevel", ctypes.c_uint),           #/**< [in]   log level. See EC_LOG_LEVEL_... */
        ("pfLogMsgCallBack", ctypes.c_void_p),    #/**< [in]   optional call back function to log msg from the RAS Client. set to EC_NULL if not used. */
    ]

class SDN_EC_T_MBX_GATEWAY_CLNT_CONDESC(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_MBX_GATEWAY_SRV_PARMS(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_LINK_PARMS_DEFAULT(ctypes.Structure):
    _fields_ = [
        ("dummy", ctypes.c_uint8),
    ]

class SDN_EC_T_LINK_PARMS_WINPCAP(ctypes.Structure):
    _fields_ = [
        ("abyIpAddress", ctypes.c_uint8 * 4),
        ("szAdapterId", ctypes.c_char * 39), #/* {XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX} */
    ]

class SDN_EC_T_LINK_PARMS_SOCKRAW(ctypes.Structure):
    _fields_ = [
        ("szAdapterName", ctypes.c_char * 64),
    ]

class SDN_EC_T_LINK_PARMS_I8254X(ctypes.Structure):
    _fields_ = [
        ("wRxBufferCnt", ctypes.c_ushort),          #/* RX buffer count, 0: default to 96 */
        ("wRxBufferSize", ctypes.c_ushort),         #/* RX buffer size (single Ethernet frame).
                                                    #   \sa EC_T_LINK_I8254X_BUFFERSIZE.
                                                    #    0: buffer optimized for standard Ethernet frame. */
        ("wTxBufferCnt", ctypes.c_ushort),          #/* TX buffer count, 0: default to 96 */
        ("wTxBufferSize", ctypes.c_ushort),         #/* TX buffer size (single Ethernet frame).
                                                    #   \sa EC_T_LINK_I8254X_BUFFERSIZE.
                                                    #    0: buffer optimized for standard Ethernet frame. */
        ("bDisableLocks", ctypes.c_uint),           #/* Locks in LL Disabled */
    ]

class SDN_EC_T_LINK_PARMS_UDP(ctypes.Structure):
    _fields_ = [
        ("szAdapterName", ctypes.c_char * 64), #// MAX_LEN_UDP_ADAPTER_NAME 64
        ("abyIpAddress", ctypes.c_uint8 * 4),
        ("wPort", ctypes.c_ushort),
    ]

class SDN_EC_T_LINK_PARMS_NDIS(ctypes.Structure):
    _fields_ = [
        ("szAdapterName", ctypes.c_char * 64),      #/**< ServiceName of network adapter, see HKLM\SOFTWARE\Microsoft\Windows NT\CurrentVersion\NetworkCards in registry (zero terminated) */
        ("abyIpAddress", ctypes.c_uint8 * 4),       #/**< IP address of network adapter */
        ("bDisablePromiscuousMode", ctypes.c_uint), #/**< Disable adapter promiscuous mode */
        ("bDisableForceBroadcast", ctypes.c_uint),  #/**< Don't change target MAC address to FF:FF:FF:FF:FF:FF */
    ]

class SDN_EC_T_SIMULATOR_DEVICE_CONNECTION_DESC(ctypes.Structure):
    _fields_ = [
        ("dwType", ctypes.c_uint), # uint
        ("dwInstanceID", ctypes.c_uint), # uint
        ("wCfgFixedAddress", ctypes.c_ushort), # ushort
        ("byPort", ctypes.c_ubyte), # byte
    ]

class SDN_EC_T_LINK_PARMS_SIMULATOR(ctypes.Structure):
    _fields_ = [
        #/* topology parameters */
        ("szEniFilename", ctypes.c_char * 256), #/**< optional: create slaves from ENI/EXI (zero terminated string) */  #// EC_SIMULATOR_ENI_FILE_NAME_SIZE
        ("oDeviceConnection", SDN_EC_T_SIMULATOR_DEVICE_CONNECTION_DESC), #/**< See EC_SIMULATOR_DEVICE_CONNECTION_TYPE_... */
        ("bConnectHcGroups", ctypes.c_uint),                                      #/**< Connect hot connect groups in topology (floating group heads to free ports) */

        #/* EC-Simulator core parameters */
        ("dwSimulatorAddress", ctypes.c_uint),                            #/**< Reserved */
        ("dwBusCycleTimeUsec", ctypes.c_uint),                            #/**< Cycle time of simulator job task */
        ("bDisableProcessDataImage", ctypes.c_uint),                     #/**< Don't allocate Process Data Image at simulator (legacy support, CiA402 simulation) */
        ("qwOemKey", ctypes.c_uint64),                                      #/**< 64 bit OEM key (optional) */

        #/* adapter parameters */
        ("abyMac", ctypes.c_uint8 * 6),                                     #/**< MAC station address */
        ("dwRxBufferCnt", ctypes.c_uint),                                   #/**< Frame buffer count for IST */

        #/* application specific */
        ("bJobsExecutedByApp", ctypes.c_uint),                           #/**< EC_FALSE: esExecJob explicitely called by application, EC_TRUE: implicitely by emllSimulator */

        #/* license parameters */
        ("szLicenseKey", ctypes.c_char * 256),                                 #/**< License key (zero terminated string) */ #// EC_SIMULATOR_KEY_SIZE
        #("aoLinkParms", SDN_EC_T_LINK_PARMS * 4),   #/**< link parms of network adapters passed to EC-Simulator Core, e.g. for validation of MAC address of license key */ #// EC_SIMULATOR_MAX_LINK_PARMS

        #/* AtesRasSrv parameters */
        ("bStartRasServer", ctypes.c_uint),                                   #/**< RAS server port */
        ("wRasServerPort", ctypes.c_ushort),                                   #/**< RAS server threads CPU affinity mask */
        ("oRasCpuAffinityMask", ctypes.c_uint64),                                   #/**< RAS server threads priority */
        ("dwRasPriority", ctypes.c_uint),                                   #/**< RAS server threads priority */
        ("dwRasStackSize", ctypes.c_uint),                                   #/**< RAS server threads stack size */

        #/* Performance Measurements */
        ("PerfMeasInternalParms", SDN_EC_T_PERF_MEAS_INTERNAL_PARMS), #/**< [in] Internal performance measurement parameters */
    ]

class SDN_EC_T_LINK_PARMS_PROXY(ctypes.Structure):
    _fields_ = [
        ("dwSocketType", ctypes.c_uint),       #/**< Socket type. Must be set to 2 (emrassocktype_udp) */
        ("abySrcIpAddress", ctypes.c_uint8 * 4), #/**< Source adapter IP address (listen) */
        ("wSrcPort", ctypes.c_ushort),           #/**< Source port number (listen) */
        ("abyDstIpAddress", ctypes.c_uint8 * 4), #/**< Destination adapter IP address (connect) */
        ("wDstPort", ctypes.c_ushort),           #/**< Destination port number (connect) */

        ("abyMac", ctypes.c_uint8 * 6),          #/**< MAC address */
        ("dwRxBufferCnt", ctypes.c_uint),      #/**< Frame buffer count for interrupt service thread (IST) */
    ]

class SDN_EC_T_LINK_PARMS(ctypes.Structure):
    _fields_ = [
        ("eLinkType", ctypes.c_uint),
        ("eLinkMode", ctypes.c_uint),
        ("dwInstance", ctypes.c_uint),
        ("cpuIstCpuAffinityMask", ctypes.c_uint64),
        ("dwIstPriority", ctypes.c_uint),
        ("oDefault", SDN_EC_T_LINK_PARMS_DEFAULT),
        ("oWinPcap", SDN_EC_T_LINK_PARMS_WINPCAP),
        ("oSockRaw", SDN_EC_T_LINK_PARMS_SOCKRAW),
        ("oI8254x", SDN_EC_T_LINK_PARMS_I8254X),
        ("oUdp", SDN_EC_T_LINK_PARMS_UDP),
        ("oNdis", SDN_EC_T_LINK_PARMS_NDIS),
        ("oSimulator", SDN_EC_T_LINK_PARMS_SIMULATOR),
        ("oProxy", SDN_EC_T_LINK_PARMS_PROXY),
    ]

class SDN_EC_T_INIT_MASTER_PARMS(ctypes.Structure):
    _fields_ = [
        ("dwSignature", ctypes.c_uint),                        #/*< [in] set to ATECAT_SIGNATURE */
        ("dwSize", ctypes.c_uint),                             #/*< [in] set to sizeof(EC_T_INIT_MASTER_PARMS) */

        ("oLinkParms", SDN_EC_T_LINK_PARMS),         #/*< [in] Link layer parameters */
        ("oLinkParmsRed", SDN_EC_T_LINK_PARMS),     #/*< [in] Link layer parameters for red device */

        ("dwBusCycleTimeUsec", ctypes.c_uint),                 #/*< [in] [usec] bus cycle time in microseconds */

        #/* memory */
        ("dwMaxBusSlaves", ctypes.c_uint),                     #/*< [in] maximum pre-allocated bus slave objects */
        ("dwMaxAcycFramesQueued", ctypes.c_uint),              #/*< [in] maximum queued Ethernet frames */
        ("dwAdditionalEoEEndpoints", ctypes.c_uint),           #/*< [in] additional EoE endpoints */

        #/* bus load */
        ("dwMaxAcycBytesPerCycle", ctypes.c_uint),             #/*< [in] maximum bytes sent during eUsrJob_SendAcycFrames per cycle */

        #/* CPU load */
        ("dwMaxAcycFramesPerCycle", ctypes.c_uint),             #/*< [in] maximum frames amount sent during eUsrJob_SendAcycFrames per cycle */
        ("dwMaxAcycCmdsPerCycle", ctypes.c_uint),               #/*< [in] maximum cmds amount sent during eUsrJob_SendAcycFrames per cycle */
        ("dwMaxSlavesProcessedPerCycle", ctypes.c_uint),        #/*< [in] maximum slave-related state machine calls per cycle */

        #/* retry and timeouts */
        ("dwEcatCmdMaxRetries", ctypes.c_uint),                 #/*< [in] maximum retries to send pending ethercat command frames */
        ("dwEcatCmdTimeout", ctypes.c_uint),                    #/*< [in] timeout to send pending ethercat command frames */
        ("dwEoETimeout", ctypes.c_uint),                        #/*< [in] timeout sending EoE frames */
        ("dwFoEBusyTimeout", ctypes.c_uint),                    #/*< [in] obsolete */

        #/* VLAN */
        ("bVLANEnable", ctypes.c_uint),                         #/*< [in] E=enable (1/0) */
        ("wVLANId", ctypes.c_ushort),                            #/*< [in] I=VLAN Id (12Bit)*/
        ("byVLANPrio", ctypes.c_ubyte),                          #/*< [in] P=Prio (3Bit) */

        #/* logging */
        ("dwLogLevel", ctypes.c_uint),                         #/*< [in] log level. See EC_LOG_LEVEL_... */
        ("pfLogMsgCallBack",  ctypes.c_void_p),                #/*< [in] optional call back function to log msg from the EC-Master. set to EC_NULL if not used. */

        ("MasterRedParms", SDN_EC_T_MASTER_RED_PARMS),           # /**< [in] Master Redundancy parameters */

        #/* Slave to slave mailbox communication */
        ("dwMaxS2SMbxSize", ctypes.c_uint),                     #/*< [in] Size of the queued S2S mailbox in bytes */
        ("dwMaxQueuedS2SMbxTfer", ctypes.c_uint),               #/*< [in] S2S Fifo number of entries */

        ("wMaxSlavesProcessedPerBusScanStep", ctypes.c_uint16), #/**< [in] maximum slave-related calls per cycle during bus scans */
        ("wReserved", ctypes.c_uint16),

        ("bApiLockByApp", ctypes.c_uint),                      #/**< [in] EC_TRUE: Don't lock pending API calls to increase performance */

        #/* Performance Measurements */
        ("PerfMeasInternalParms", SDN_EC_T_PERF_MEAS_INTERNAL_PARMS), #/**< [in] Internal performance measurement parameters */
    ]

class SDN_EC_STRING_HLP(ctypes.Structure):
    _fields_ = [
        ("Data", ctypes.c_char * 0x200), # string[0x200]
    ]

class SDN_ATEMRAS_T_CLNTCONDESC(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_REGISTERPARMS(ctypes.Structure):
    _fields_ = [
        ("pCallerData", ctypes.c_void_p),         #/*< [in] used by all callback functions */
        ("pfnNotify", ctypes.c_void_p),           #/*< [in] notify callback function pointer */
    ]

class SDN_EC_T_CYCFRAME_RX_CBDESC(ctypes.Structure):
    _fields_ = [
        ("pCallbackContext", ctypes.c_void_p),    #/*< [in]  Context pointer. This pointer is used as parameter every time when the callback function is called */
        ("pfnCallback", ctypes.c_void_p),         #/*< [in]  This function will be called after the cyclic frame is received, if there is more than one cyclic frame after the last frame. The application has to assure that these functions will not block. */
    ]

class SDN_ATEMRAS_T_SRVPARMS(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_IOCTLOPARMS(ctypes.Structure):
    _fields_ = [
        ("pbyInBuf", ctypes.c_void_p),                         #/*< [in] input data buffer */
        ("dwInBufSize", ctypes.c_uint),                        #/*< [in] size of input data buffer in byte */
        ("pbyOutBuf", ctypes.c_void_p),                        #/*< [out] output data buffer */
        ("dwOutBufSize", ctypes.c_uint),                       #/*< [in] size of output data buffer in byte */
        ("pdwNumOutData", ctypes.c_uint),                      #/*< [out] number of output data bytes stored in output data buffer */
    ]

class SDN_EC_T_SB_SLAVEINFO_DESC(ctypes.Structure):
    _fields_ = [
        ("dwScanBusStatus", ctypes.c_uint),                    #/* Status during last Bus Scan */
        ("dwVendorId", ctypes.c_uint),                         #/* Vendor Identification */
        ("dwProductCode", ctypes.c_uint),                      #/* Product Code */
        ("dwRevisionNumber", ctypes.c_uint),                   #/* Revision Number */
        ("dwSerialNumber", ctypes.c_uint),                     #/* Serial Number */
    ]

class SDN_EC_T_COE_OBDESC(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_COE_ENTRYDESC(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_ATEMRAS_T_CONNOTIFYDESC(ctypes.Structure):
    _fields_ = [
        ("dwCause", ctypes.c_uint),      #/**< [in]   Connection change type */
        ("dwCookie", ctypes.c_uint),     #/**< [in]   Cookie of connection */
    ]

class SDN_ATEMRAS_T_REGNOTIFYDESC(ctypes.Structure):
    _fields_ = [
        ("dwCookie", ctypes.c_uint),       #/**< [in]   Cookie of connection */
        ("dwResult", ctypes.c_uint),       #/**< [in]   Result of call */
        ("dwInstanceId", ctypes.c_uint),   #/**< [in]   ID master instance */
        ("dwClientId", ctypes.c_uint),     #/**< [in]   ID of un- / registered client */
    ]

class SDN_ATEMRAS_T_MARSHALERRORDESC(ctypes.Structure):
    _fields_ = [
        ("dwCookie", ctypes.c_uint),       #/**< [in]   Cookie of faulting connection */
        ("dwCause", ctypes.c_uint),        #/**< [in]   Cause of fault */
        ("dwLenStatCmd", ctypes.c_uint),   #/**< [in]   Faulty Protocol Header */
        ("dwCommandCode", ctypes.c_uint),  #/**< [in]   Faulting command code */
    ]

class SDN_ATEMRAS_T_NONOTIFYMEMORYDESC(ctypes.Structure):
    _fields_ = [
        ("dwCookie", ctypes.c_uint),       #/**< [in]   Cookie of faulting connection */
        ("dwCode", ctypes.c_uint),         #/**< [in]   Fault causing notification code */
    ]

#///<summary>RAS parameters</summary>
class SDN_DN_EC_T_INITRASPARAMS(ctypes.Structure):
    _fields_ = [
        ("abyIpAddr", ctypes.c_uint8 * 4),                  #/**< [in]   IP Address */
        ("wPort", ctypes.c_ushort),                       #/**< [in]   IP Port */
        ("dwWatchDog", ctypes.c_uint),                 #/**< [in]   Watchdog interval when to send IDL packets */
        ("dwCycleTime", ctypes.c_uint),                 #/**< [in]   Cycle Time for Recv Polling */
        ("dwWDTOLimit", ctypes.c_uint),                 #/**< [in]   Amount of cycles without receiving commands (idles) before
                                                #      *         Entering state wdexpired
                                                #      */
        ("qwOemKey", ctypes.c_uint64),                  # /**< [in]   OEM Key */
    ]

class SDN_EC_T_CYC_CONFIG_DESC(ctypes.Structure):
    _fields_ = [
        ("dwNumCycEntries", ctypes.c_uint),
        ("dwTaskId", ctypes.c_uint),
        ("dwPriority", ctypes.c_uint),
        ("dwCycleTime", ctypes.c_uint),
    ]

class _t_sAddr(ctypes.Structure):
    _fields_ = [
        ("by", ctypes.c_uint8 * 4),
    ]

class SDN_EC_T_IPADDR(ctypes.Structure):
    _fields_ = [
        ("dwAddr", ctypes.c_uint),
        #public _t_sAddr sAddr;
    ]

class SDN_EC_T_PTS_SRV_START_PARMS(ctypes.Structure):
    _fields_ = [
        ("oIpAddr", ctypes.c_uint8 * 4),
        ("dwPtsThreadPriority", ctypes.c_uint),
        ("wPort", ctypes.c_ushort),
    ]

class SDN_EC_T_SIMULATOR_INIT_PARMS(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_MONITOR_INIT_PARMS(ctypes.Structure):
    _fields_ = [
        ("dwNotSupported", ctypes.c_uint), # eval limitation
    ]

class SDN_EC_T_DAQ_INIT_PARMS(ctypes.Structure):
    _fields_ = [
        # /* logging */
        ("dwLogLevel", ctypes.c_uint),               #/**< [in]   log level. See EC_LOG_LEVEL_... */
        ("pfLogMsgCallBack", ctypes.c_void_p),         #/**< [in]   optional call back function to log msg from the RAS Client. set to EC_NULL if not used. */

        #/* general */
        ("dwMasterInstanceId", ctypes.c_uint),       #/*< [in] Master instance ID */
        ("szWriter", ctypes.c_char * 64),       #/*< [in] Writer name (e.g. MDF or CSV) */
        ("szName", ctypes.c_char * 128),       #/*< [in] Writer title */
        ("szFile", ctypes.c_char * 256),       #/*< [in] Writer file name */
        ("dwSampleRate", ctypes.c_uint),       #/*< [in] Sample rate */
        ("dwBusCycleTimeUsec", ctypes.c_uint),       #/*< [in] Bus cycle time in usec */
        ("bRealTimeStamp", ctypes.c_uint),       #/*< [in] EC_TRUE: Real time stamp, EC_FALSE: Virtual time stamp */
        ("bCycleCounter", ctypes.c_uint),       #/*< [in] Cycle counter */
        ("bElapsedTimeMsec", ctypes.c_uint),       #/*< [in] Elapsed time in msec */
        ("bElapsedTimeUsec", ctypes.c_uint),       #/*< [in] Elapsed time in usec */

        #/* limits */
        ("dwLimitsMaxFileSize", ctypes.c_uint),       #/*< [in] Maximal file size in bytes */
        ("dwLimitsMaxDuration", ctypes.c_uint),       #/*< [in] Maximal duration in msec */
        ("dwLimitsMaxFiles", ctypes.c_uint),       #/*< [in] Maximal count of files */

        #/* thread */
        ("dwThreadMaxPendingDataSets", ctypes.c_uint),       #
        ("dwThreadCpuSet", ctypes.c_uint),       #/*< [in] CPU set of thread */
        ("dwThreadPrio", ctypes.c_uint),       #/*< [in] Priority of thread */
        ("dwThreadStackSize", ctypes.c_uint),       #/*< [in] Stack size of thread */

        #/* oversampling */
        ("dwOversamplingMaxRate", ctypes.c_uint),       #/*< [in] Maximal oversampling rate */

        #/* memory */
        ("dwMemoryCycleCount", ctypes.c_uint),       #/*< [in] Memory cycle count */
    ]
    
class SDN_EC_T_DAQ_READER_PARMS(ctypes.Structure):
    _fields_ = [
        # /* logging */
        ("dwLogLevel", ctypes.c_uint),               #/**< [in]   log level. See EC_LOG_LEVEL_... */
        ("pfLogMsgCallBack", ctypes.c_void_p),       #/**< [in]   optional call back function to log msg from the RAS Client. set to EC_NULL if not used. */

        #/* general */
        ("szFile", ctypes.c_char * 256),             #/*< [in] File name */
        ("szFormat", ctypes.c_char * 128),           #/*< [in] Format name (e.g. MDF or CSV) */
    ]

class SDN_EC_T_MBXTFER(ctypes.Structure):
    _fields_ = [
        #//EC_T_DWORD          dwClntId;                   #/*< client ID */
        #//EC_T_MBXTFER_DESC   MbxTferDesc;                #/*< mailbox transfer descriptor */
        ("eMbxTferType", ctypes.c_uint),               #/*< mailbox transfer type */
        ("dwDataLen", ctypes.c_uint),                  #/*< available/reserved length within mailbox data buffer (scope: API call) */
        #//EC_T_BYTE*          pbyMbxTferData;             #/*< pointer to mailbox data buffer */
        ("eTferStatus", ctypes.c_uint),                #/*< current transfer status */
        ("dwErrorCode", ctypes.c_uint),                #/*< transfer error code */
        ("dwTferId", ctypes.c_uint),                   #/*< unique transfer ID */
        #//EC_T_MBX_DATA       MbxData;                    #/*< mailbox data */
    ]

class SDN_EC_T_COE_OBJ1018(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("wSubIndex0", ctypes.c_ushort),
        ("dwVendorID", ctypes.c_uint),
        ("dwProductcode", ctypes.c_uint),
        ("dwRevision", ctypes.c_uint),
        ("dwSerialnumber", ctypes.c_uint),
    ]

class SDN_EC_T_COE_OBJ2001(ctypes.Structure):
    _fields_ = [
        ("dwMasterStateSummary", ctypes.c_uint),
    ]

class SDN_EC_T_PERF_MEAS_INFO(ctypes.Structure):
    _fields_ = [
        ("szName", ctypes.c_char * 80), # string[80]
        ("qwFrequency", ctypes.c_uint64), # uint64
        ("eUserJob", ctypes.c_uint), # DN_EC_T_USER_JOB
        ("dwBinCountHistogram", ctypes.c_uint), # uint
        ("dwFlags", ctypes.c_uint), # uint
    ]

class SDN_EC_T_PERF_MEAS_HISTOGRAM(ctypes.Structure):
    _fields_ = [
        ("dwBinCount", ctypes.c_uint), # uint
        ("aBins", ctypes.c_void_p), # IntPtr
        ("qwMinTicks", ctypes.c_uint64), # uint64
        ("qwMaxTicks", ctypes.c_uint64), # uint64
    ]

class SDN_EC_T_PERF_MEAS_VAL(ctypes.Structure):
    _fields_ = [
        ("qwCurrTicks", ctypes.c_uint64), # uint64
        ("qwMinTicks", ctypes.c_uint64), # uint64
        ("qwMaxTicks", ctypes.c_uint64), # uint64
        ("qwAvgTicks", ctypes.c_uint64), # uint64
    ]

class SDN_EC_T_PERF_MEAS_INFO_PARMS(ctypes.Structure):
    _fields_ = [
        ("szName", ctypes.c_char * 80), # string[80]
        ("dwFlags", ctypes.c_uint), # uint
    ]

class SDN_EC_T_PERF_MEAS_APP_PARMS(ctypes.Structure):
    _fields_ = [
        ("dwNumMeas", ctypes.c_uint), # uint
        ("aPerfMeasInfos", SDN_EC_T_PERF_MEAS_INFO_PARMS * 20), # SDN_EC_T_PERF_MEAS_INFO_PARMS[20]
        ("CounterParms", SDN_EC_T_PERF_MEAS_COUNTER_PARMS), # SDN_EC_T_PERF_MEAS_COUNTER_PARMS
        ("HistogramParms", SDN_EC_T_PERF_MEAS_HISTOGRAM_PARMS), # SDN_EC_T_PERF_MEAS_HISTOGRAM_PARMS
    ]

SDN_EC_T_VARIANT_MaxBufferSize = 0x200

class SDN_EC_T_VARIANT_UNION(ctypes.Union):
    _fields_ = [
        ("abyBuffer", ctypes.c_uint8 * SDN_EC_T_VARIANT_MaxBufferSize), # byte[]
        ("nInteger8", ctypes.c_byte), # sbyte
        ("nInteger16", ctypes.c_short), # short
        ("nInteger32", ctypes.c_int), # int
        ("nInteger64", ctypes.c_int64), # int64
        ("nUnsigned8", ctypes.c_ubyte), # byte
        ("nUnsigned16", ctypes.c_ushort), # ushort
        ("nUnsigned32", ctypes.c_uint), # uint
        ("nUnsigned64", ctypes.c_uint64), # uint64
        ("nReal32", ctypes.c_float), # float
        ("nReal64", ctypes.c_double), # double
    ]

class SDN_EC_T_VARIANT(ctypes.Structure):
    MaxBufferSize = SDN_EC_T_VARIANT_MaxBufferSize
    _fields_ = [
        ("nBufferSize", ctypes.c_uint),
        ("uVariant", SDN_EC_T_VARIANT_UNION),
    ]

    def GetBuffer(self):
        bytes_ = []
        for i in range(self.nBufferSize):
            a = self.uVariant.abyBuffer[i]
            bytes_.append(a)
        return bytes_

    def SetBuffer(self, bytes_):
        for i in range(len(bytes_)):
            self.uVariant.abyBuffer[i] = ctypes.c_uint8(bytes_[i])
        self.nBufferSize = len(bytes_)

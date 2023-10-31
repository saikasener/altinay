/*-----------------------------------------------------------------------------
 * MotionDemo.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Stefan Zintgraf
 * Description              EC-Master demo header
 *---------------------------------------------------------------------------*/

#include "EcOs.h"

/*-SUPPORT-SELECTION----------------------------------------------------------*/
#define INCLUDE_EC_MASTER

/* the RAS server is necessary to support the EC-Engineer or other remote applications */
#if (!defined INCLUDE_RAS_SERVER) && (defined EC_SOCKET_SUPPORTED)
#define INCLUDE_RAS_SERVER
#endif

/*-INCLUDES------------------------------------------------------------------*/
#include "AtEthercat.h"
#include "MotionDemoConfig.h"
#include "ecatDemoCommon.h"

#ifdef VXWORKS
#include "wvLib.h"
#endif

#if (defined INCLUDE_RAS_SERVER)
#include "AtEmRasSrv.h"
#endif

/*-DEFINES-------------------------------------------------------------------*/
/* tag names for DemoConfig.xml file */
#define DEMO_CFG_DEFAULT_FILENAME       (EC_T_CHAR*)"DemoConfig.xml"
#define DEMO_CFG_TAG_ENI_FILENAME       (EC_T_CHAR*)"Config\\Common\\ENIFileName"
#define DEMO_CFG_TAG_LOG_FILEPREFIX     (EC_T_CHAR*)"Config\\Common\\LogFilePrefix"
#define DEMO_CFG_TAG_LINK_LAYER         (EC_T_CHAR*)"Config\\Common\\LinkLayer"
#define DEMO_CFG_TAG_LINK_LAYER2        (EC_T_CHAR*)"Config\\Common\\LinkLayer2"
#define DEMO_CFG_TAG_DURATION           (EC_T_CHAR*)"Config\\Common\\DemoDuration"
#define DEMO_CFG_TAG_CPU_AFFINITY       (EC_T_CHAR*)"Config\\Common\\CpuAffinity"
#define DEMO_CFG_TAG_VERBOSITY_LVL      (EC_T_CHAR*)"Config\\Common\\VerbosityLevel"
#define DEMO_CFG_TAG_PERF_MEASURE       (EC_T_CHAR*)"Config\\Common\\PerfMeasurement"
#define DEMO_CFG_TAG_RAS_ENABLED        (EC_T_CHAR*)"Config\\Common\\RASEnabled"
#define DEMO_CFG_TAG_RAS_PORT           (EC_T_CHAR*)"Config\\Common\\RASPort"
#define DEMO_CFG_TAG_AUXCLK             (EC_T_CHAR*)"Config\\Common\\AuxClk"
#define DEMO_CFG_TAG_BUSCYCLETIME       (EC_T_CHAR*)"Config\\Common\\BusCycleTime"
#define MAX_LINKLAYER 5


#define REMOTE_WD_TO_LIMIT          10000
#define REMOTE_CYCLE_TIME           2

#if (defined INCLUDE_RAS_SERVER)
#define ATEMRAS_MAX_WATCHDOG_TIMEOUT    10000
#define ATEMRAS_CYCLE_TIME              2
#endif

/*-FUNCTION DECLARATIONS-----------------------------------------------------*/
EC_T_DWORD MotionDemo(
     EC_T_CNF_TYPE       eCnfType
    ,EC_T_PBYTE          pbyCnfData
    ,EC_T_DWORD          dwCnfDataLen
    ,EC_T_DWORD          dwBusCycleTimeUsec
    ,EC_T_INT            nVerbose
    ,EC_T_DWORD          dwDuration
    ,EC_T_LINK_PARMS*    poLinkParms
    ,EC_T_VOID*          pvTimingEvent
    ,EC_T_CPUSET         CpuSet
    ,EC_T_BOOL           bEnaPerfJobs
#ifdef INCLUDE_RAS_SERVER 
    ,EC_T_WORD           wServerPort
#endif
    ,EC_T_LINK_PARMS*    poLinkParmsRed
    ,EC_T_BOOL           bNoMasterSync
    ,EC_T_VOID*          pvCfgFileHandle
    ,EC_T_BYTE*          pbyMotionLogBuffer
    ,EC_T_DWORD          dwMotionLogBufSize
    ,EC_T_CHAR*          szLogPrefix
    );

/*--------------------------------------------------------------------------*/
/* Performance measurements of jobs                                         */
/* This is only available on CPUs with TSC support                          */
/*--------------------------------------------------------------------------*/
#define JOB_ProcessAllRxFrames  0
#define JOB_SendAllCycFrames    1
#define JOB_MasterTimer         2
#define JOB_SendAcycFrames      3
#define JOB_Total               4
#define PERF_CycleTime          5
#define PERF_myAppWorkpd        6
#define PERF_DCM_Logfile        7
#define PERF_MC_CalcVal_1       8
#define PERF_MC_CalcVal_2       9
#define PERF_MC_CalcVal_3      10
#define MAX_JOB_NUM            11

#define PERF_MEASURE_JOBS_INIT(msgcb)   ecatPerfMeasInit(&pDemoThreadParam->TscMeasDesc,0,MAX_JOB_NUM,msgcb);ecatPerfMeasEnable(&pDemoThreadParam->TscMeasDesc)
#define PERF_MEASURE_JOBS_DEINIT()      ecatPerfMeasDeinit(&pDemoThreadParam->TscMeasDesc)
#define PERF_MEASURE_JOBS_SHOW()        ecatPerfMeasShow(&pDemoThreadParam->TscMeasDesc,0xFFFFFFFF,S_aszMeasInfo)
#define PERF_MEASURE_JOBS_RESET()       ecatPerfMeasReset(&pDemoThreadParam->TscMeasDesc,0xFFFFFFFF)
#define PERF_JOB_START(nJobIndex)       ecatPerfMeasStart(&pDemoThreadParam->TscMeasDesc,(EC_T_DWORD)(nJobIndex))
#define PERF_JOB_END(nJobIndex)         ecatPerfMeasEnd(&pDemoThreadParam->TscMeasDesc,(EC_T_DWORD)(nJobIndex))

#define PERF_JOB_TOTAL  { if (pDemoThreadParam->TscMeasDesc.bMeasEnabled) { \
                              EC_T_TSC_TIME* pTscTime = &pDemoThreadParam->TscMeasDesc.aTscTime[JOB_Total]; \
                              if (pTscTime->bMeasReset) { pTscTime->dwCurr = 0; pTscTime->dwAvg = 0; pTscTime->dwMin = (EC_T_DWORD)ULONG_MAX; pTscTime->dwMax = 0; pTscTime->bMeasReset = EC_FALSE; } \
                              else { \
                                  pTscTime->dwCurr =  pDemoThreadParam->TscMeasDesc.aTscTime[JOB_ProcessAllRxFrames].dwCurr + pDemoThreadParam->TscMeasDesc.aTscTime[JOB_SendAllCycFrames].dwCurr \
                                                    + pDemoThreadParam->TscMeasDesc.aTscTime[JOB_MasterTimer].dwCurr + pDemoThreadParam->TscMeasDesc.aTscTime[JOB_SendAcycFrames].dwCurr; \
                                  pTscTime->dwAvg =  pDemoThreadParam->TscMeasDesc.aTscTime[JOB_ProcessAllRxFrames].dwAvg + pDemoThreadParam->TscMeasDesc.aTscTime[JOB_SendAllCycFrames].dwAvg \
                                                    + pDemoThreadParam->TscMeasDesc.aTscTime[JOB_MasterTimer].dwAvg + pDemoThreadParam->TscMeasDesc.aTscTime[JOB_SendAcycFrames].dwAvg; \
                                  pTscTime->dwMax = EC_MAX(pTscTime->dwMax, pTscTime->dwCurr); \
                                  pTscTime->dwMin = EC_MIN(pTscTime->dwMin, pTscTime->dwCurr); }}}

/*-END OF SOURCE FILE--------------------------------------------------------*/

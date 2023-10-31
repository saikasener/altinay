/*-----------------------------------------------------------------------------
 * EcDemoApp.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Holger Oelhaf
 * Description              Application specific settings for EC-Master demo
 *---------------------------------------------------------------------------*/

#ifndef INC_ECDEMOAPP_H
#define INC_ECDEMOAPP_H 1

/*-LOGGING-------------------------------------------------------------------*/
#ifndef pEcLogParms
#define pEcLogParms (&(pAppContext->LogParms))
#endif

#define INCLUDE_EC_MASTER

/*-INCLUDES------------------------------------------------------------------*/
#include "AtEthercat.h"
#include "EcDemoPlatform.h"
#include "EcDemoParms.h"
#include "EcLogging.h"
#include "EcNotification.h"
#include "EcSdoServices.h"
#include "EcSelectLinkLayer.h"
#include "EcSlaveInfo.h"

/*-DEFINES-------------------------------------------------------------------*/
#define EC_DEMO_APP_NAME (EC_T_CHAR*)"EcMasterDemo"

/* the RAS server is necessary to support the EC-Engineer or other remote applications */
#if (!defined INCLUDE_RAS_SERVER) && (defined EC_SOCKET_SUPPORTED)
#define INCLUDE_RAS_SERVER
#endif

#if (defined INCLUDE_RAS_SERVER)
#include "AtEmRasSrv.h"
#define ATEMRAS_MAX_WATCHDOG_TIMEOUT    10000
#define ATEMRAS_CYCLE_TIME              2
#endif

/*-FUNCTION DECLARATIONS-----------------------------------------------------*/
EC_T_VOID  ShowSyntaxAppUsage(T_EC_DEMO_APP_CONTEXT* pAppContext);
EC_T_VOID  ShowSyntaxApp(T_EC_DEMO_APP_CONTEXT* pAppContext);
EC_T_VOID  ShowSyntaxLinkLayer(EC_T_VOID);
EC_T_DWORD EcDemoApp(T_EC_DEMO_APP_CONTEXT* pAppContext);

#define PRINT_PERF_MEAS() ((EC_NULL != pEcLogContext)?((CAtEmLogging*)pEcLogContext)->PrintPerfMeas(pAppContext->dwInstanceId, 0, pEcLogContext) : 0)
#define PRINT_HISTOGRAM() ((EC_NULL != pEcLogContext)?((CAtEmLogging*)pEcLogContext)->PrintHistogramAsCsv(pAppContext->dwInstanceId) : 0)

#endif /* INC_ECDEMOAPP_H */

/*-END OF SOURCE FILE--------------------------------------------------------*/

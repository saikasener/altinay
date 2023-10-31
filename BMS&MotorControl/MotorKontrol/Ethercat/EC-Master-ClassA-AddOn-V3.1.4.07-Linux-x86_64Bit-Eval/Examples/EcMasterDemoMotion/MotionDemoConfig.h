/*-----------------------------------------------------------------------------
 * MotionDemoConfig.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Christoph Widmann
 * Description              EtherCAT Master motion demo configuration header
 *---------------------------------------------------------------------------*/

/*-INCLUDES------------------------------------------------------------------*/


/*-DEFINES-------------------------------------------------------------------*/

/*********************/
/* thread priorities */
/*********************/
#if defined WIN32 && !defined UNDER_CE && !defined RTOS_32 && !defined UNDER_RTSS
 /* we need to set all threads to the highest possible priority to avoid errors! */
 #define TIMER_THREAD_PRIO           ((EC_T_DWORD)THREAD_PRIORITY_TIME_CRITICAL) /* EtherCAT master trigger thread priority */
 #define JOBS_THREAD_PRIO            ((EC_T_DWORD)THREAD_PRIORITY_TIME_CRITICAL) /* EtherCAT master job thread priority */
 #define RECV_THREAD_PRIO            ((EC_T_DWORD)THREAD_PRIORITY_TIME_CRITICAL) /* EtherCAT master interrupt service thread priority */
 #define LOG_THREAD_PRIO             ((EC_T_DWORD)THREAD_PRIORITY_LOWEST)  /* EtherCAT message logging thread priority */
 #define MAIN_THREAD_PRIO            ((EC_T_DWORD)THREAD_PRIORITY_NORMAL)
#elif (defined UNDER_RTSS)
 #define TIMER_THREAD_PRIO           (RT_PRIORITY_MAX-0) /* EtherCAT master trigger thread priority */
 #define JOBS_THREAD_PRIO            (RT_PRIORITY_MAX-1) /* EtherCAT master job thread priority */
 #define RECV_THREAD_PRIO            (RT_PRIORITY_MAX-2) /* EtherCAT master interrupt service thread priority */
 #define LOG_THREAD_PRIO             (RT_PRIORITY_MIN)   /* EtherCAT message logging thread priority */
 #define LOG_ROLLOVER                ((EC_T_WORD)0)      /* EtherCAT message logging rollover counter limit */
 #define MAIN_THREAD_PRIO            (RT_PRIORITY_MIN)
 #define JOBS_THREAD_STACKSIZE       0x8000
 #define LOG_THREAD_STACKSIZE        0x8000
#elif (defined RTOS_32)
 #define MAIN_THREAD_PRIO_OFFSET     10
 #define TIMER_THREAD_PRIO           ((EC_T_DWORD)64)    /* EtherCAT master trigger thread priority */
 #define JOBS_THREAD_PRIO            ((EC_T_DWORD)63)    /* EtherCAT master job thread priority */
 #define RECV_THREAD_PRIO            ((EC_T_DWORD)62)    /* EtherCAT master interrupt service thread priority */
 #define LOG_THREAD_PRIO             ((EC_T_DWORD)1 )    /* EtherCAT message logging thread priority */
 #define MAIN_THREAD_PRIO            ((EC_T_DWORD)3+MAIN_THREAD_PRIO_OFFSET)        
 #define JOBS_THREAD_STACKSIZE       0x8000
 #define LOG_THREAD_STACKSIZE        0x8000
#elif (defined EC_VERSION_QNX)
 #define TIMER_THREAD_PRIO           ((EC_T_DWORD)100)   /* EtherCAT master trigger thread priority */
 #define JOBS_THREAD_PRIO            ((EC_T_DWORD)99)    /* EtherCAT master job thread priority */
 #define RECV_THREAD_PRIO            ((EC_T_DWORD)98)    /* EtherCAT master interrupt service thread priority */
 #define LOG_THREAD_PRIO             ((EC_T_DWORD)40)    /* EtherCAT message logging thread priority */
 #define MAIN_THREAD_PRIO            ((EC_T_DWORD)30)
 #define REMOTE_RECV_THREAD_PRIO     ((EC_T_DWORD)50)    /* slightly higher than logger */
#elif (defined LINUX)
 #define TIMER_THREAD_PRIO           ((EC_T_DWORD)99)   /* EtherCAT master timer task (tEcTimingTask) */
 #define JOBS_THREAD_PRIO            ((EC_T_DWORD)98)   /* EtherCAT master job thread priority (tEcJobTask) */
 #define RECV_THREAD_PRIO            ((EC_T_DWORD)97)   /* EtherCAT master packet receive thread priority (tLOsaL_IST) */
 #define LOG_THREAD_PRIO             ((EC_T_DWORD)29)   /* EtherCAT message logging thread priority (tAtEmLog) */
 #define MAIN_THREAD_PRIO            ((EC_T_DWORD)39)   /* Main thread */
#elif (defined __INTEGRITY)
 #define TIMER_THREAD_PRIO           ((EC_T_DWORD)200)   /* EtherCAT master timer task (tEcTimingTask) */
 #define JOBS_THREAD_PRIO            ((EC_T_DWORD)199)   /* EtherCAT master job thread priority */
 #define RECV_THREAD_PRIO            ((EC_T_DWORD)198)   /* EtherCAT master interrupt service thread priority */
 #define LOG_THREAD_PRIO             ((EC_T_DWORD)40)    /* EtherCAT message logging thread priority */
 #define MAIN_THREAD_PRIO            ((EC_T_DWORD)40)
#elif (defined __INTIME__)
 /*Priority 0 to 127 Used by the OS for servicing external interrupts. */
 #define TIMER_THREAD_PRIO           ((EC_T_DWORD)128)   /* EtherCAT master timer task (tEcTimingTask) */
 #define JOBS_THREAD_PRIO            ((EC_T_DWORD)129)   /* EtherCAT master job thread priority */
 #define RECV_THREAD_PRIO            ((EC_T_DWORD)0)     /* Is not used for INtime. The receive thread priority is defined by the Interrupt Level  */
 #define LOG_THREAD_PRIO             ((EC_T_DWORD)200)   /* EtherCAT message logging thread priority */
 #define MAIN_THREAD_PRIO            ((EC_T_DWORD)130)
#elif (defined __TKERNEL)
 #define TIMER_THREAD_PRIO           ((EC_T_DWORD)1)     /* EtherCAT master timer task (tEcTimingTask) */
 #define JOBS_THREAD_PRIO            ((EC_T_DWORD)2)     /* EtherCAT master job thread priority */
 #define RECV_THREAD_PRIO            ((EC_T_DWORD)3)     /* EtherCAT master interrupt service thread priority */
 #define LOG_THREAD_PRIO             ((EC_T_DWORD)4)     /* EtherCAT message logging thread priority */
 #define MAIN_THREAD_PRIO            ((EC_T_DWORD)10)
#elif (defined EC_VERSION_SYSBIOS)
 #define TIMER_THREAD_PRIO           ((EC_T_DWORD)12)   /* EtherCAT master timer task (tEcTimingTask) */
 #define JOBS_THREAD_PRIO            ((EC_T_DWORD)12)   /* EtherCAT master job thread priority (tEcJobTask) */
 #define RECV_THREAD_PRIO            ((EC_T_DWORD)12)   /* EtherCAT master packet receive thread priority (tLOsaL_IST) */
 #define LOG_THREAD_PRIO             ((EC_T_DWORD)1)    /* EtherCAT message logging thread priority (tAtEmLog) */
 #define MAIN_THREAD_PRIO            ((EC_T_DWORD)2)    /* Main thread */
 #if defined SOC_AM572x
  #if defined INCLUDE_EMLLICSS
   #define DEMO_PARAMETERS            "-auxclk 1000 -v 2 -t 20000 -perf " \
                                      "-icss "                    \
                                      "4 "    /* Instance */      \
                                      "1 "    /* mode */          \
                                      "m "    /* master/slave */          \
                                      "am572x-idk"  /* reference board */
  #elif defined INCLUDE_EMLLCPSW
   #define DEMO_PARAMETERS            "-auxclk 1000 -v 2 -t 20000 -perf " \
                                      "-cpsw "                    \
                                      "2 "    /* port */          \
                                      "1 "    /* mode */          \
                                      "1 "    /* priority */      \
                                      "m "    /* master flag */   \
                                      "am572x-idk " /* reference board */
  #endif
#elif (defined SOC_AM437x)
  #if defined INCLUDE_EMLLCPSW
 #define DEMO_PARAMETERS              "-auxclk 1000 -v 2 -t 20000 -perf " \
                                      "-cpsw "                    \
                                      "1 "    /* port */          \
                                      "1 "    /* mode */          \
                                      "1 "    /* priority */      \
                                      "m "    /* master flag */   \
                                      "am437x-idk " /* custom board for AM437x*/
  #endif
#elif (defined SOC_AM335x)
  #if defined INCLUDE_EMLLCPSW
 #define DEMO_PARAMETERS              "-auxclk 1000 -v 2 -t 20000 -perf " \
                                      "-cpsw "                    \
                                      "1 "    /* port */          \
                                      "1 "    /* mode */          \
                                      "1 "    /* priority */      \
                                      "m "    /* master flag */   \
                                      "custom am33XX " /* custom board for AM33xx*/   \
                                      "1 "    /* PHY address */   \
                                      "1 "    /* PHY connection mode: RGMII */ \
                                      "0 "    /* Not use DMA buffers */
  #endif
#else

#endif /* SOC_AM571x */

#elif (defined EC_VERSION_RIN32M3)
 #define TIMER_THREAD_PRIO           ((EC_T_DWORD)3)   /* EtherCAT master timer task (tEcTimingTask) */
 #define JOBS_THREAD_PRIO            ((EC_T_DWORD)3)   /* EtherCAT master job thread priority (tEcJobTask) */
 #define RECV_THREAD_PRIO            ((EC_T_DWORD)3)   /* EtherCAT master packet receive thread priority (tLOsaL_IST) */
 #define LOG_THREAD_PRIO             ((EC_T_DWORD)14)    /* EtherCAT message logging thread priority (tAtEmLog) */
 #define DEMO_PARAMETERS             "-auxclk 1000 -t 0 -v 2 -perf -rin32m3"
#elif (defined EC_VERSION_XILINX_STANDALONE)
 #define TIMER_THREAD_PRIO           ((EC_T_DWORD)3)   /* EtherCAT master timer task (tEcTimingTask) */
 #define JOBS_THREAD_PRIO            ((EC_T_DWORD)3)   /* EtherCAT master job thread priority (tEcJobTask) */
 #define RECV_THREAD_PRIO            ((EC_T_DWORD)3)   /* EtherCAT master packet receive thread priority (tLOsaL_IST) */
 #define LOG_THREAD_PRIO             ((EC_T_DWORD)14)    /* EtherCAT message logging thread priority (tAtEmLog) */
 #define DEMO_PARAMETERS             "-auxclk 1000 -v 2 -t 20000 -perf -rin32m3"
#elif (defined EC_VERSION_ETKERNEL)
 #define TIMER_THREAD_PRIO           ((EC_T_DWORD)1)     /* EtherCAT master timer task (tEcTimingTask) */
 #define JOBS_THREAD_PRIO            ((EC_T_DWORD)2)     /* EtherCAT master job thread priority */
 #define RECV_THREAD_PRIO            ((EC_T_DWORD)3)     /* EtherCAT master interrupt service thread priority */
 #define LOG_THREAD_PRIO             ((EC_T_DWORD)4)     /* EtherCAT message logging thread priority */
 #define MAIN_THREAD_PRIO            ((EC_T_DWORD)10)
 #define DEMO_PARAMETERS             "-b 2000 -fslfec 1 1 sabresd -v 2 -t 20000 -perf"
#elif (defined EC_VERSION_RZT1)
 #define TIMER_THREAD_PRIO           ((EC_T_DWORD)1)     /* EtherCAT master timer task (tEcTimingTask) */
 #define JOBS_THREAD_PRIO            ((EC_T_DWORD)2)     /* EtherCAT master job thread priority */
 #define RECV_THREAD_PRIO            ((EC_T_DWORD)3)     /* EtherCAT master interrupt service thread priority */
 #define LOG_THREAD_PRIO             ((EC_T_DWORD)4)     /* EtherCAT message logging thread priority */
 #define MAIN_THREAD_PRIO            ((EC_T_DWORD)10)
 #define DEMO_PARAMETERS             "-rzt1 1 -v 2 -t 20000 -perf -auxclk 1000"
 #define LOG_THREAD_STACKSIZE		 0x2000
#elif (defined EC_VERSION_RZGNOOS)
 #define TIMER_THREAD_PRIO           ((EC_T_DWORD)1)     /* EtherCAT master timer task (tEcTimingTask) */
 #define JOBS_THREAD_PRIO            ((EC_T_DWORD)2)     /* EtherCAT master job thread priority */
 #define RECV_THREAD_PRIO            ((EC_T_DWORD)3)     /* EtherCAT master interrupt service thread priority */
 #define LOG_THREAD_PRIO             ((EC_T_DWORD)4)     /* EtherCAT message logging thread priority */
 #define MAIN_THREAD_PRIO            ((EC_T_DWORD)10)
 #define DEMO_PARAMETERS             "-sheth 1 1 rzg1e -v 3 -t 20000 -perf -auxclk 1000"
 #define LOG_THREAD_STACKSIZE        0x2000
#elif (defined EC_VERSION_ECOS)
 #define TIMER_THREAD_PRIO           ((EC_T_DWORD)1)     /* EtherCAT master timer task (tEcTimingTask) */
 #define JOBS_THREAD_PRIO            ((EC_T_DWORD)2)     /* EtherCAT master job thread priority */
 #define RECV_THREAD_PRIO            ((EC_T_DWORD)3)     /* EtherCAT master interrupt service thread priority */
 #define LOG_THREAD_PRIO             ((EC_T_DWORD)4)     /* EtherCAT message logging thread priority */
#elif (defined EC_VERSION_JSLWARE)
 #define TIMER_THREAD_PRIO           ((EC_T_DWORD)12)   /* EtherCAT master timer task (tEcTimingTask) */
 #define JOBS_THREAD_PRIO            ((EC_T_DWORD)12)   /* EtherCAT master job thread priority (tEcJobTask) */
 #define RECV_THREAD_PRIO            ((EC_T_DWORD)12)   /* EtherCAT master packet receive thread priority (tLOsaL_IST) */
 #define LOG_THREAD_PRIO             ((EC_T_DWORD)1)    /* EtherCAT message logging thread priority (tAtEmLog) */
 #define MAIN_THREAD_PRIO            ((EC_T_DWORD)2)    /* Main thread */
   #define DEMO_PARAMETERS            "-auxclk 500 -v 2 -t 20000 -perf " \
                                      "-cpsw "                    \
                                      "1 "    /* port */          \
                                      "1 "    /* mode */          \
                                      "1 "    /* priority */      \
                                      "m "    /* master flag */   \
                                      "custom am33XX " /* custom board for AM33xx*/   \
                                      "1 "    /* PHY address */   \
                                      "1 "    /* PHY connection mode: RGMII */ \
                                      "0 "    /* Not use DMA buffers */
#elif (defined EC_VERSION_UCOS)
#define TIMER_THREAD_PRIO           ((EC_T_DWORD)10)   /* EtherCAT master timer task (tEcTimingTask) */
#define JOBS_THREAD_PRIO            ((EC_T_DWORD)8)   /* EtherCAT master job thread priority (tEcJobTask) */
#define RECV_THREAD_PRIO            ((EC_T_DWORD)8)   /* EtherCAT master packet receive thread priority (tLOsaL_IST) */
#define LOG_THREAD_PRIO             ((EC_T_DWORD)8)    /* EtherCAT message logging thread priority (tAtEmLog) */
#define MAIN_THREAD_PRIO            ((EC_T_DWORD)8)    /* Main thread */
#define DEMO_PARAMETERS            "-v 2 -fslfec 1 1 custom imx6 rmii 1 -perf"

#define TIMER_THREAD_STACKSIZE       0x1000
#define JOBS_THREAD_STACKSIZE        0x1000
#define LOG_THREAD_STACKSIZE         0x1000

#else

 #ifdef VXWORKS_NORMAL_PRIO
  #define REALTIME_PRIORITY_OFFSET    60
 #else
  #define REALTIME_PRIORITY_OFFSET    2
 #endif

 #define TIMER_THREAD_PRIO           (0+REALTIME_PRIORITY_OFFSET)    /* EtherCAT master timer task (tEcTimingTask) */
 #define JOBS_THREAD_PRIO            (1+REALTIME_PRIORITY_OFFSET)    /* EtherCAT master cyclic packet send thread priority */
 #define RECV_THREAD_PRIO            (2+REALTIME_PRIORITY_OFFSET)    /* EtherCAT master packet receive thread priority */
 #define MBX_THREAD_PRIO             (3+REALTIME_PRIORITY_OFFSET)    /* mailbox demo thread priority */

 #define LOG_THREAD_PRIO             (200)                           /* EtherCAT message logging thread priority */
 #define MAIN_THREAD_PRIO            (4+REALTIME_PRIORITY_OFFSET)
 #define REMOTE_RECV_THREAD_PRIO     0

#endif
#ifndef TIMER_THREAD_STACKSIZE
#define TIMER_THREAD_STACKSIZE       0x1000
#endif
#ifndef JOBS_THREAD_STACKSIZE
#define JOBS_THREAD_STACKSIZE        0x8000
#endif
#ifndef LOG_THREAD_STACKSIZE
#define LOG_THREAD_STACKSIZE         0x8000
#endif

/******************/
/* timer settings */
/******************/
#define CYCLE_TIME          1           /* 1 msec */
#define BASE_PERIOD         1000        /* 1000 usec */

/***********************************************/
/* static EtherCAT master configuration values */
/***********************************************/
#if !(defined EC_DEMO_TINY)
#define MASTER_CFG_ECAT_MAX_BUS_SLAVES       256    /* max number of pre-allocated bus slave objects */
#define MASTER_CFG_MAX_ACYC_FRAMES_QUEUED     32    /* max number of acyc frames queued, 127 = the absolute maximum number */
#define MASTER_CFG_MAX_ACYC_BYTES_PER_CYC   4096    /* max number of bytes sent during eUsrJob_SendAcycFrames within one cycle */
#else
#define MASTER_CFG_ECAT_MAX_BUS_SLAVES         8    /* max number of pre-allocated bus slave objects */
#define MASTER_CFG_MAX_ACYC_FRAMES_QUEUED     32    /* max number of acyc frames queued, 127 = the absolute maximum number */
#define MASTER_CFG_MAX_ACYC_BYTES_PER_CYC    512    /* max number of bytes sent during eUsrJob_SendAcycFrames within one cycle */
#endif /* EC_DEMO_TINY */
#define MASTER_CFG_MAX_ACYC_CMD_RETRIES        3

/* DCM controller set value: Distance between frame send time and sync impulse */
#define DCM_CONTROLLER_SETVAL_NANOSEC    ((BASE_PERIOD/2)*1000)            /* 50 % */

/* maximum controller error in nanoseconds */
#if defined WIN32 && !defined UNDER_CE
#define DCM_MAX_CTL_ERROR_NANOSEC       90000
#else
#define DCM_MAX_CTL_ERROR_NANOSEC       30000
#endif

/* maximum time to get in-sync */
#define DCM_SETVAL_TIMEOUT_MSEC         12000


/* tag names for DemoConfig.xml file */
#define MOTIONDEMO_CFG_TAG_NO_BUS_SHIFT      (EC_T_CHAR*)"Config\\MotionDemo\\NoDCMBusShift"        /* Disable DCM controller */
#define MOTIONDEMO_CFG_TAG_CTL_SETVAL        (EC_T_CHAR*)"Config\\MotionDemo\\DCMCtlSetVal"         /* DCM controller set value in nanosec */
#define MOTIONDEMO_CFG_TAG_CMD_MODE          (EC_T_CHAR*)"Config\\MotionDemo\\CmdMode"              /* Command Mode */

#define MOTIONDEMO_CFG_TAG_DRIVE_ADDR       (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\Address"          /* EtherCAT Address */
#define MOTIONDEMO_CFG_TAG_DRIVE_MODE       (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\OperationMode"    /* Mode of operation */
#define MOTIONDEMO_CFG_TAG_DRIVE_NO         (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\SercosDriveNo"    /* Adressed drive number within SERCOS servo controller */
#define MOTIONDEMO_CFG_TAG_DRIVE_IDX_OPMODE (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\CoeIdxOpMode"     /* Index of CoE object 'Operation Mode'. Default 0x6060 */
#define MOTIONDEMO_CFG_TAG_DRIVE_PROFILE    (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\DriveProfile"     /* "DS402" or "SERCOS" */
#define MOTIONDEMO_CFG_TAG_DRIVE_INCPERMM   (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\IncPerMM"         /* Increments per mm */
#define MOTIONDEMO_CFG_TAG_DRIVE_INCFACTOR  (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\IncFactor"        /* Increments are scaled with this value */
#define MOTIONDEMO_CFG_TAG_DRIVE_ACC        (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\Acc"              /* Move acc */
#define MOTIONDEMO_CFG_TAG_DRIVE_DEC        (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\Dec"              /* Move dec */
#define MOTIONDEMO_CFG_TAG_DRIVE_VEL        (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\Vel"              /* Move vel */
#define MOTIONDEMO_CFG_TAG_DRIVE_JERK       (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\Jerk"             /* Move jerk */
#define MOTIONDEMO_CFG_TAG_DRIVE_DIST       (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\Distance"         /* Move distance */
#define MOTIONDEMO_CFG_TAG_DRIVE_POS_WIND   (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\PositionWindow"   /* Position Window */
#define MOTIONDEMO_CFG_TAG_DRIVE_VEL_GAIN   (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\VelocityGain"     /* Velocity gain */
#define MOTIONDEMO_CFG_TAG_DRIVE_TOR_GAIN   (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\TorqueGain"       /* Torque gain */

#define MOTIONDEMO_CFG_TAG_DRIVE_IDX_STATUS    (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\IdxStatus"     /* Object Index Statusword */
#define MOTIONDEMO_CFG_TAG_DRIVE_IDX_CONTROL   (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\IdxControl"    /* Object Index Controlword */
#define MOTIONDEMO_CFG_TAG_DRIVE_IDX_POSACT    (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\IdxPosact"     /* Object Index Position actual */
#define MOTIONDEMO_CFG_TAG_DRIVE_IDX_TARPOS    (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\IdxTargetpos"  /* Object Index Target Position */
#define MOTIONDEMO_CFG_TAG_DRIVE_IDX_TARVEL    (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\IdxTargetvel"  /* Object Index Target Velocity */
#define MOTIONDEMO_CFG_TAG_DRIVE_IDX_ACTTRQ    (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\IdxActualtrq"  /* Object Index Actual Torque */
#define MOTIONDEMO_CFG_TAG_DRIVE_IDX_TARTRQ    (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\IdxTargettrq"  /* Object Index Target Torque */

#define MOTIONDEMO_CFG_TAG_DRIVE_IDX_VELOFF    (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\IdxVelOffset"  /* Object Index Velocity Offset */
#define MOTIONDEMO_CFG_TAG_DRIVE_IDX_TOROFF    (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\IdxTorOffset"  /* Object Index Torque Offset */
#define MOTIONDEMO_CFG_TAG_DRIVE_IDX_MODE      (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\IdxMode"       /* Object Index Mode of Operation */
#define MOTIONDEMO_CFG_TAG_DRIVE_IDX_MODE_DISPLAY      (EC_T_CHAR*)"Config\\MotionDemo\\Drive%d\\IdxModeDisplay"       /* Object Index Mode of Operation Dissplay */



/* Default values */
/* -------------- */
#define DEFAULT_INCPERMM            100000          /* [inc] increments per mm */

/* Move parameters */
#define DEFAULT_DISTANCE            500             /* [mm]     distance                */
#define DEFAULT_MAX_VELOCITY        1000            /* [mm/s]   maximum velocity        */
#define DEFAULT_ACC                 2000            /* [mm/s^2] maximum acceleration    */
#define DEFAULT_DEC                 3000            /* [mm/s^2] maximum deceleration increments/cycle^2 */
#define DEFAULT_JERK                6000            /* [mm/s^3] jerk */
#define DEFAULT_POS_WINDOW          10
#define DEFAULT_VEL_GAIN            1
#define DEFAULT_TORQUE_GAIN         1

/*-END OF SOURCE FILE--------------------------------------------------------*/

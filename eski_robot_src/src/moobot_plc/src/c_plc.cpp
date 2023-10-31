#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <ethercat.h>
#include <bitset>
#include <iostream>
#include <bits/stdc++.h>
#include "ros/ros.h"
#include <moobot_msgs/moobot_status.h>
#include <moobot_msgs/moobot_scanner.h>
#include <moobot_msgs/moobot_sensor_status.h>
#include <moobot_msgs/ultrasonic_status.h>
#include "actionlib_msgs/GoalID.h"
#include "geometry_msgs/Twist.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#define EC_TIMEOUTMON 500

#define ULTRASONIC_PROTECTION_ZONE 1
#define ULTRASONIC_WARNING_1 2
#define ULTRASONIC_WARNING_2 3
#define ULTRASONIC_NO_ZONE 4
enum
{
   AUTO,
   MANUAL,
   SERVICE
};
enum
{
   ZERO,
   IMU_ERROR,
   SCAN_DATA_ERROR,
   DRIVER_ERROR,
   BMS_ERROR,
   CONVEYOR_ERROR,
   PGV_ERROR
};
char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
bool old_stop = false;
bool current_stop;
int old_mode;
int error_status;
bool reset_required = false;
bool conveyor_loaded = false;
bool ssp_error = false;
bool ossd_error = false;
bool old_ssp_error = true;
bool old_ossd_error = true;
bool old_agv_break = true;
bool agv_break = false;
bool reset_printed = false;
bool start_printed = false;
bool ultrasonic_status1_2 = false;
bool ultrasonic_status3_4 = false;
int ultrasonic_status1_2_counter = 0;
int ultrasonic_status3_4_counter = 0;

std::string XTIO1_input_message;
std::string XTIO2_out_message;
std::string XTIO3_out_message;
std::string XTDI3_input_message;
std::string XTIO1_out_message;
std::string LOGIC_result;
std_msgs::Bool conveyor_loaded_button_msg;
int out_msg_0[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // for diagnostic
int out_msg_1[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // for scanner area change
moobot_msgs::moobot_status moobot_status_msg;
moobot_msgs::moobot_scanner scanner_status_msg;
actionlib_msgs::GoalID goal_id;
geometry_msgs::Twist twist_msg;

uint8 output_message = 3;
ros::Publisher twist_pub;
ros::Publisher cancel_pub;
ros::Publisher moobot_status_pub_s_plc;
ros::Publisher scanner_status_pub;
ros::Subscriber moobot_sensor_status_sub_s_plc;
ros::Subscriber emg_from_c_plc_sub;
ros::Publisher conveyor_loaded_button_pub;
ros::Subscriber scanner_area_sub;
ros::Publisher scanner_area_pub;
ros::Subscriber ultrasonic_sensor_sub3_4;
ros::Subscriber ultrasonic_sensor_sub1_2;
std::string convertHexToBinary(uint8 *hexadecimal)
{
   int dec = hexadecimal;
   std::string ret = std::bitset<8>(dec).to_string();
   return ret;
}

void change_mode(int mode)
{
   moobot_status_msg.agv_mode = mode;
   moobot_status_pub_s_plc.publish(moobot_status_msg);
  // std::cout << moobot_status_msg.agv_stopped << std::endl;
   /*if (mode == AUTO)
      system("gnome-terminal -x sh -c 'rosrun moobot_plc plc_loop.py'");
   else if (mode == MANUAL)
      system("gnome-terminal -x sh -c 'rosnode kill controller_plc'");*/
}
bool emg_situation = true;
bool old_emg_situation = false;
void stop_agv(int current_mode)
{

   moobot_status_msg.agv_stopped = true;
   moobot_status_msg.emergency = emg_situation;
   if (emg_situation == true)
      ROS_INFO("Emergency jjj situation");
   ROS_INFO("AGV stopped.");
   bool cancel = true;
   if (emg_situation)
   {
      moobot_status_msg.agv_mode = MANUAL;
      try
      {
         cancel_pub.publish(goal_id);
      }
      catch (...)
      {
         cancel = false;
         ROS_INFO("No goal to cancel");
      }
      if (cancel)
         moobot_status_msg.job = "Job cancelled";
   }
   else
   {
      moobot_status_msg.agv_mode = current_mode;
   }
   twist_msg.angular.z = 0.0;
   twist_msg.linear.x = 0.0;
   twist_pub.publish(twist_msg);
   moobot_status_pub_s_plc.publish(moobot_status_msg);
}

void reset_system()
{
   switch (error_status)
   {
   case IMU_ERROR:
      ROS_INFO("Reset IMU");
      //system(" gnome-terminal -x sh -c '\" rosnode kill ros_imu_bno055_node; sh\"'");
      break;
   case SCAN_DATA_ERROR:
      ROS_INFO("Reset Scan Data node");
      break;
   case PGV_ERROR:
      ROS_INFO("Reset PGV");
      break;
   case CONVEYOR_ERROR:
      ROS_INFO("Reset Conveyor");
      break;
   case BMS_ERROR:
      ROS_INFO("Reset BMS");
      break;
   case DRIVER_ERROR:
      ROS_INFO("Reset Driver");
      break;
   default:
      break;
   }
}
//////////////////////////////////////////
///////////  INPUT MODULLERI  ///////////
/////////////////////////////////////////
///
/// XTDI_3_IN->1 : E-Stop 1. kontak
/// XTDI_3_IN->2 : Mode switch auto contact
/// XTDI_3_IN->3 : Mode switch manual contact
/// XTDI_3_IN->4 : Start button
/// XTDI_3_IN->5 : Front Scanner Warning 2
/// XTDI_3_IN->6 : Front Scanner Warning 1
/// XTDI_3_IN->7 : Front Scanner OSSD 1
/// XTDI_3_IN->8 : Back Scanner OSSD 1
/////////////////////////////////////////
/// XTIO_1_IN->1 : E-Stop 2. kontak
/// XTIO_1_IN->2 : Reset button
/// XTIO_1_IN->3 : Single channel ??
/// XTIO_1_IN->4 : Single channel ??
/// XTIO_1_IN->5 : Back Scanner Warning 2
/// XTIO_1_IN->6 : Back Scanner Warning 1
/// XTIO_1_IN->7 : Back Scanner OSSD 2
/// XTIO_1_IN->8 : Front Scanner OSSD 2
/////////////////////////////////////////

/////////////////////////////////////////
//////////  OUTPUT MODULLERI  ///////////
/// XTIO_1_OUT->1 : FS_Field1 -> A1
/// XTIO_1_OUT->2 : FS_Field2 -> A2
/// XTIO_1_OUT->3 : BS_Field1 -> A1
/// XTIO_1_OUT->4 : BS_Field2 -> A2
/// XTIO_1_OUT->5 :
/// XTIO_1_OUT->6 :
/// XTIO_1_OUT->7 :
/// XTIO_1_OUT->8 :
/////////////////////////////////////////
/// XTIO_2_OUT->1 : Brake
/// XTIO_2_OUT->2 : Res_Req
/// XTIO_2_OUT->3 : Yellow Lamb
/// XTIO_2_OUT->4 :
/// XTIO_2_OUT->5 :
/// XTIO_2_OUT->6 :
/// XTIO_2_OUT->7 :
/// XTIO_2_OUT->8 :
/////////////////////////////////////////

/////////////////////////////////////////
//////////  LOGIC RESULT  ///////////////
/// LOGIC->0 : Emergency signal
/// LOGIC->1 : Reset for PC
/// LOGIC->2 : Manual mode
/// LOGIC->3 : Auto mode
/// LOGIC->4 : PLC Reset Done
/// LOGIC->5 : Pallet Loaded Button
/// LOGIC->6 : SSP error
/// LOGIC->7 : Protection zone emg
/////////////////////////////////////////

/////////////////////////////////////////
//////////  OUTPUT TO PLC  //////////////
/////////////////////////////////////////
/// OUT_MSG_0->0 : IMU error
/// OUT_MSG_0->1 : SCAN data error
/// OUT_MSG_0->2 : C-PLC e-stop butonu
/// OUT_MSG_0->3 : BMS error
/// OUT_MSG_0->4 : Conveyor error
/// OUT_MSG_0->5 : PGV error
/// OUT_MSG_0->6 : Reset Done
/// OUT_MSG_0->7 : System Works
/////////////////////////////////////////
/////////////////////////////////////////
/// OUT_MSG_1->0 : Scanner Area 1 (For conveyor area)
/// OUT_MSG_1->1 : Scanner Area 2 (For robot area)
/// OUT_MSG_1->2 : Scanner Area 3 (For high speed area)
/// OUT_MSG_1->3 : Scanner Area 4 (For lift area)
/// OUT_MSG_1->4 : PC to PLC Break Signal
/// OUT_MSG_1->5 :
/// OUT_MSG_1->6 :
/// OUT_MSG_1->7 :
/////////////////////////////////////////
int current_mode = MANUAL;
int current_scanner_area_type = 0;
int scanner_area_type = 0;
bool reset_done = false;
bool old_reset_done = true;

void change_area(const std_msgs::Int16 &area_msg)
{ // değişiklik olmadan da bu datayı sürekli yollamakta sıkıntı olmaz
   scanner_area_type = area_msg.data - 1;
   for (int i = 0; i < 8; i++)
   {
      if (i == scanner_area_type)
      {
         out_msg_1[i] = 1;
      }
      else
      {
         if (i != 4)
            out_msg_1[i] = 0;
      }
   }
}

void send_break(bool break_signal)
{
   if (break_signal)
      out_msg_1[4] = 1;
   else
      out_msg_1[4] = 0;
}
void check_area_info(std::string XTIO1_out_message)
{
   //        FS A1                             FS A2                            BS A1                               BS A2
   std_msgs::Int16 msg;
   if (XTIO1_out_message.at(7) == '0' && XTIO1_out_message.at(6) == '1' && XTIO1_out_message.at(5) == '0' && XTIO1_out_message.at(4) == '1')
   {
      current_scanner_area_type = msg.data = 4; //TODO: şu anlık lift 4
   }
   else if (XTIO1_out_message.at(7) == '1' && XTIO1_out_message.at(6) == '0' && XTIO1_out_message.at(5) == '1' && XTIO1_out_message.at(4) == '0')
   {
      current_scanner_area_type = msg.data = 0;
   }
   scanner_area_pub.publish(msg);
}
void checkMessage(std::string XTIO1_input_message, std::string XTDI3_input_message, std::string XTIO3_out_message, std::string LOGIC_result, std::string XTIO1_out_message)
{
   /*for (int i = 0; i < 8; i++)
   {
      std::cout << LOGIC_result.at(i) ;
   }
   std::cout << std::endl;*/
   if (LOGIC_result.at(6) == '1')
   {
      ROS_INFO("Reset required");
      reset_required = true;
      reset_system();
   }
   else
   {
      reset_required = false;
      out_msg_0[6] = 0;
   }
   if (LOGIC_result.at(7) == '1')
   { // Emergency signal comes from plc
      current_stop = true;
      emg_situation = true;
   }
   else
   {

      current_stop = false;
      emg_situation = false;
   }

   if (XTDI3_input_message.at(0) == '0' || XTIO1_input_message.at(1) == '0' || XTIO1_input_message.at(0) == '0' || XTDI3_input_message.at(1) == '0' || LOGIC_result.at(7) == '1')
   { // scanner ossd data
      current_stop = true;
   }
   else
   {
      current_stop = false;
   }
   if (XTIO1_input_message.at(0) == '0' || XTDI3_input_message.at(1) == '0')
   { // FS ossd active
      scanner_status_msg.fs_ossd = 1;

      //ROS_INFO("FS ossd active.");
   }
   else
      scanner_status_msg.fs_ossd = 0;

   if (XTDI3_input_message.at(0) == '0' || XTIO1_input_message.at(1) == '0')
   { //BS ossd active
      scanner_status_msg.bs_ossd = 1;
      //ROS_INFO("BS ossd active.");
   }
   else
      scanner_status_msg.bs_ossd = 0;

   if (XTIO1_input_message.at(2) == '0')
   { //bs warning 1
      //ROS_INFO("BS w1 active.");
      scanner_status_msg.bs_w1 = 1;
   }
   else
      scanner_status_msg.bs_w1 = 0;

   if (XTIO1_input_message.at(3) == '0')
   { //bs warning 2
      //ROS_INFO("BS w2 active.");
      scanner_status_msg.bs_w2 = 1;
   }
   else
      scanner_status_msg.bs_w2 = 0;

   if (XTDI3_input_message.at(2) == '0')
   { // fs warning 1
      //ROS_INFO("FS w1 active.");
      scanner_status_msg.fs_w1 = 1;
   }
   else
      scanner_status_msg.fs_w1 = 0;

   if (XTDI3_input_message.at(3) == '0')
   { // fs warning 2
      //ROS_INFO("FS w2 active.");
      scanner_status_msg.fs_w2 = 1;
   }
   else
      scanner_status_msg.fs_w2 = 0;

   scanner_status_pub.publish(scanner_status_msg);

   check_area_info(XTIO1_out_message);
   if (!emg_situation)
   {
      if (LOGIC_result.at(4) == '1')
      { //If active then gives 1, mode auto
         current_mode = AUTO;
         if (old_mode != current_mode)
         {
            ROS_INFO("Auto mode");
            change_mode(AUTO);
         }
      }
      if (LOGIC_result.at(5) == '1')
      { //If active then gives 1, mode manual
         current_mode = MANUAL;
         if (old_mode != current_mode)
         {
            change_mode(MANUAL);
            ROS_INFO("Manual mode");
         }
      }
   }
   
   if (LOGIC_result.at(3) == '1') //If active then gives 1, reset done
       reset_done = true;
   else reset_done = false;

   if (XTDI3_input_message.at(4) == '1')
   { //If active then gives 1, start pressed
      //TODO
      if (start_printed == false)
      {
         ROS_INFO("Start button pressed");
         moobot_status_msg.agv_mode = MANUAL;
         moobot_status_msg.agv_stopped = false;
         moobot_status_pub_s_plc.publish(moobot_status_msg);
         start_printed = true;
      }
   }
   else
      start_printed = false;

   if (XTIO1_input_message.at(6) == '1')
   { //If active then gives 1, reset pressed
      //TODO: reset procedure
      if (reset_printed == false)
      {
         ROS_INFO("Reset button pressed");
         reset_printed = true;
      }
   }
   else
      reset_printed = false;

   //burada conveyor_loaded butonunu oku ve sadece 1 ken publish et
   if (LOGIC_result.at(2) == '1')
   {
      conveyor_loaded_button_msg.data = true;
      conveyor_loaded_button_pub.publish(conveyor_loaded_button_msg);
   }
   if (LOGIC_result.at(0) == '1') // ossd den dolayı emg olursa 1 veriyor
      ossd_error = true;
   else
      ossd_error = false;

   if (old_ossd_error != ossd_error)
   {
      if (ossd_error)
         ROS_INFO("Protection zone emergency");
   }
   if (LOGIC_result.at(1) == '1') // ssp koparsa 1 veriyor
      ssp_error = true;
   else
      ssp_error = false;

   if (old_ssp_error != ssp_error)
   {
      if (ssp_error)
         ROS_INFO("SSP error");
   }
   agv_break = (ultrasonic_status1_2 || ultrasonic_status3_4) && current_scanner_area_type != 4;
   //agv_break = ultrasonic_status1_2 || ultrasonic_status3_4;
   
   if (old_emg_situation != emg_situation){
      if(emg_situation) stop_agv(current_mode);
   }
   //TODO: stopken emg gelirse nolcak bakkkkkkkkkk
   if (current_stop != old_stop)
   {
      if (current_stop)
         stop_agv(current_mode);
      else
      {
         current_stop = false;
         moobot_status_msg.agv_mode = current_mode;
         moobot_status_msg.agv_stopped = false;
         moobot_status_msg.emergency = emg_situation;
         moobot_status_pub_s_plc.publish(moobot_status_msg);
      }
   }
   if (old_agv_break != agv_break)
   {
      send_break(agv_break);
      if (agv_break)
         stop_agv(current_mode);
      else
      {
         agv_break = false;
         moobot_status_msg.agv_mode = current_mode;
         moobot_status_msg.agv_stopped = false;
         moobot_status_msg.emergency = emg_situation;
         moobot_status_pub_s_plc.publish(moobot_status_msg);
      }
   }
   if (reset_done != old_reset_done){
       if(reset_done){
           ROS_INFO("Reset done");
           change_mode(MANUAL);
       }

   }
   
   old_emg_situation = emg_situation;
   old_stop = current_stop;
   old_mode = current_mode;
   old_ossd_error = ossd_error;
   old_ssp_error = ssp_error;
   old_agv_break = agv_break;
   old_reset_done = reset_done;
}

OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
   int slave;
   (void)ptr; /* Not used */

   while (1)
   {
      if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
      {
         if (needlf)
         {
            needlf = FALSE;
            printf("\n");
         }
         /* one ore more slaves are not responding */
         ec_group[currentgroup].docheckstate = FALSE;
         ec_readstate();
         for (slave = 1; slave <= ec_slavecount; slave++)
         {
            if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
            {
               ec_group[currentgroup].docheckstate = TRUE;
               if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
               {
                  printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                  ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                  ec_writestate(slave);
               }
               else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
               {
                  printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                  ec_slave[slave].state = EC_STATE_OPERATIONAL;
                  ec_writestate(slave);
               }
               else if (ec_slave[slave].state > EC_STATE_NONE)
               {
                  if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d reconfigured\n", slave);
                  }
               }
               else if (!ec_slave[slave].islost)
               {
                  /* re-check state */
                  ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                  if (ec_slave[slave].state == EC_STATE_NONE)
                  {
                     ec_slave[slave].islost = TRUE;
                     printf("ERROR : slave %d lost\n", slave);
                  }
               }
            }
            if (ec_slave[slave].islost)
            {
               if (ec_slave[slave].state == EC_STATE_NONE)
               {
                  if (ec_recover_slave(slave, EC_TIMEOUTMON))
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d recovered\n", slave);
                  }
               }
               else
               {
                  ec_slave[slave].islost = FALSE;
                  printf("MESSAGE : slave %d found\n", slave);
               }
            }
         }
         if (!ec_group[currentgroup].docheckstate)
            printf("OK : all slaves resumed OPERATIONAL.\n");
      }
      osal_usleep(10000);
   }
}

bool stop_program = false;
void intHandler(int signum)
{
   stop_program = true;
}

void check_status(const moobot_msgs::moobot_sensor_status &msg)
{ // checks status and sends to PLC
   int type = msg.state_type;

   if (type != 0)
   {
      error_status = type;
      ROS_INFO("Moobot fault");
      //Send fault type to plc.
      out_msg_0[type - 1] = 1;
   }
   else
   {
      if (reset_required == true)
      {
         reset_done = true;
         out_msg_0[6] = 1;
      }
   }
}
void check_emg(const std_msgs::Bool &msg)
{ // if emergency comes from controller plc, safety plc gives emergency error
   if (msg.data)
      out_msg_0[2] = 1;
   else
      out_msg_0[2] = 0;
}
void check_ultrasonic_status1_2(const moobot_msgs::ultrasonic_status &msg)
{
    /*if(msg.S_Protection == true) ultrasonic_status1_2_counter ++;
    else ultrasonic_status1_2_counter = 0;
    if(ultrasonic_status1_2_counter == 10) ultrasonic_status1_2 = true;
    else ultrasonic_status1_2 = false;*/
    
    ultrasonic_status1_2 = msg.S_Protection;
}
void check_ultrasonic_status3_4(const moobot_msgs::ultrasonic_status &msg)
{
   /* if(msg.S_Protection == true) ultrasonic_status3_4_counter ++;
    else ultrasonic_status3_4_counter = 0;
    if(ultrasonic_status3_4_counter == 10) ultrasonic_status3_4 = true;
    else ultrasonic_status3_4 = false;*/
    ultrasonic_status3_4 = msg.S_Protection;
}
int convert_msg(int msg[])
{
   int base = 1;
   int result = 0;
   for (int i = 0; i < 8; i++)
   {
      if (msg[i] != 0)
      {
         result += base;
      }
      base *= 2;
   }
   return result;
}

int main(int argc, char **argv)
{
   printf("Safety PLC communication is started\n\n");
   ros::init(argc, argv, "plc_listener");
   ros::NodeHandle nh;
   moobot_status_pub_s_plc = nh.advertise<moobot_msgs::moobot_status>("/agv_status", 100);
   //ultrasonic_sensor_sub1_2 = nh.subscribe("/ultrasonic_status1_2", 100, check_ultrasonic_status1_2);
   //ultrasonic_sensor_sub3_4 = nh.subscribe("/ultrasonic_status3_4", 100, check_ultrasonic_status3_4);
   scanner_status_pub = nh.advertise<moobot_msgs::moobot_scanner>("/scanner_status", 100);
   cancel_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 100);
   twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
   moobot_sensor_status_sub_s_plc = nh.subscribe("/agv_sensor_status", 100, check_status);
   emg_from_c_plc_sub = nh.subscribe("/emg_from_controller_plc", 100, check_emg);
   conveyor_loaded_button_pub = nh.advertise<std_msgs::Bool>("/conveyor_loaded", 10);
   scanner_area_sub = nh.subscribe("/change_scanner_area", 10, change_area);
   scanner_area_pub = nh.advertise<std_msgs::Int16>("/scanner_area", 10);
   ros::Rate loop_rate(100);
   char *ifname = "enp1s0";

   signal(SIGINT, intHandler); // to get Ctrl-C (it cannot get normally bc of while(1)

   /* create thread to handle slave error handling in OP */
   osal_thread_create(&thread1, 128000, &ecatcheck, (void *)&ctime); //calls pthread_create inside

   /* start cyclic part */

   int i, j, oloop, iloop, chk;
   needlf = FALSE;
   inOP = FALSE;

   printf("Starting connection with PLC\n");

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n", ifname);
      /* find and auto-config slaves */

      if (ec_config_init(FALSE) > 0)
      {
         printf("%d slaves found and configured.\n", ec_slavecount);

         ec_config_map(&IOmap);

         ec_configdc();

         printf("Slaves mapped, state to SAFE_OP.\n");
         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

         oloop = ec_slave[0].Obytes;
         if ((oloop == 0) && (ec_slave[0].Obits > 0))
            oloop = 1;
         if (oloop > 60)
            oloop = 60;
         iloop = ec_slave[0].Ibytes;
         if ((iloop == 0) && (ec_slave[0].Ibits > 0))
            iloop = 1;
         if (iloop > 60)
            iloop = 60;
         printf("\n iloop: %d", iloop);

         //printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

         printf("Request operational state for all slaves\n");
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* send one valid process data to make outputs in slaves happy*/
         ec_send_processdata();
         ec_receive_processdata(EC_TIMEOUTRET);
         /* request OP state for all slaves */
         ec_writestate(0);
         chk = 200;
         /* wait for all slaves to reach OP state */
         do
         {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
         } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
         if (ec_slave[0].state == EC_STATE_OPERATIONAL)
         {
            printf("Operational state reached for all slaves.\n");
            inOP = TRUE;
            /* cyclic loop */
            while (!stop_program)
            {
               ec_send_processdata();
               wkc = ec_receive_processdata(EC_TIMEOUTRET);

               if (wkc >= expectedWKC)
               {
                  /*printf("Processdata cycle %4d, WKC %d , O:", i, wkc);
                  printf(" \n O:");
                        for(j = 0 ; j < oloop; j++)
                        {
                            printf(" %2.2x", *(ec_slave[0].outputs + j));
                        }

                        printf(" I:");
                        for(j = 0 ; j < iloop; j++)
                        {
                            printf(" %2.2x", *(ec_slave[0].inputs + j));
                        }
                        printf(" T:%"PRId64"\r",ec_DCtime); */
                  needlf = TRUE;
                  if (ros::ok())
                  { //ros kopsa bile loop devam etsin diye
                     XTDI3_input_message = convertHexToBinary(*(ec_slave[0].inputs + 1));
                     XTIO1_input_message = convertHexToBinary(*(ec_slave[0].inputs + 2));
                     XTIO3_out_message = convertHexToBinary(*(ec_slave[0].inputs + 6)); //TODO : xtio2 olacak
                     LOGIC_result = convertHexToBinary(*(ec_slave[0].inputs + 9));
                     XTIO1_out_message = convertHexToBinary(*(ec_slave[0].inputs + 3));
                     checkMessage(XTIO1_input_message, XTDI3_input_message, XTIO3_out_message, LOGIC_result, XTIO1_out_message);
                     out_msg_0[7] = 1;
                     *(ec_slave[0].outputs) = convert_msg(out_msg_0);
                     *(ec_slave[0].outputs + 1) = convert_msg(out_msg_1);
                     ros::spinOnce();

                  } //TODO: else give error
               }
               osal_usleep(5000);
            }
            inOP = FALSE;
         }
         else
         {
            printf("Not all slaves reached operational state.\n");
            ec_readstate();
            for (i = 1; i <= ec_slavecount; i++)
            {
               if (ec_slave[i].state != EC_STATE_OPERATIONAL)
               {
                  printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                         i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
               }
            }
         }
         printf("\nRequest init state for all slaves\n");
         ec_slave[0].state = EC_STATE_INIT;
         /* request INIT state for all slaves */
         ec_writestate(0);
      }
      else
      {
         stop_agv(MANUAL);
         printf("No slaves found!\n");
      }
   }
   else
   {
      stop_agv(MANUAL);
      printf("No socket connection on %s\nExecute as root\n", ifname);
   }

   printf("End PLC communication, close socket\n");
   /* stop SOEM, close socket */
   ec_close();

   printf("End program\n");
   return (0);
}

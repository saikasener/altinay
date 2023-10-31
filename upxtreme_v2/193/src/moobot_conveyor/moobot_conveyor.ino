#include <ros.h>
#include "moobot_msgs/conveyor_status.h"
#include "moobot_msgs/moobot_status.h"
#include "std_msgs/Bool.h"
int addr_M1 = 0;

//sensÃ¶r pinleri
int sensor1_pin2 = 5; //s_start_pin2 gÃ¶rdÃ¼ÄÃ¼nde 0 gÃ¶rmediÄinde 1
int sensor2_pin2 = 7; //s_stop_pin2 gÃ¶rdÃ¼ÄÃ¼nde 0 gÃ¶rmediÄinde 1
int powersupply = 3; //power supply remote off pini



//motor sÃ¼rÃ¼cÃ¼ pinleri
unsigned static int in1 = 8; //PH
unsigned static int in2 = 9; //EN
unsigned static int nSleep = 10;
unsigned static int vref = 11;
bool conveyor_loaded = false;

//PC den gelen veri
int load_Command;
int unload_Command;

//PC ye gidecek veri
int load_done;
int unload_done;


int state = 0;

int s_start_pin2 = digitalRead(sensor1_pin2);
int s_stop_pin2 = digitalRead(sensor2_pin2);
bool S1_ON =  !(s_start_pin2);
bool S2_ON =  !(s_stop_pin2);
bool load_conveyor, unload_conveyor, load_conveyor_done, unload_conveyor_done;
bool stop_conveyor = false;
void runConveyor(const moobot_msgs::conveyor_status& msg) {
  if (msg.load_conveyor == true) {
    //if (!conveyor_loaded) {
      load_conveyor = true;
      unload_conveyor = false;
      load_conveyor_done = false;
    //}


  }
  else if (msg.unload_conveyor == true) {
    //if (conveyor_loaded) {
      load_conveyor = false;
      unload_conveyor = true;
      unload_conveyor_done = false;
    //}
  }
}

void stopConveyor(const moobot_msgs::moobot_status& msg) {
  if (msg.emergency ==  true || msg.agv_stopped == true) {
    stop_conveyor = true;
  } else {
    stop_conveyor = false;
  }
}

std_msgs::Bool conveyor_transfer_done_msg;
ros::NodeHandle nh;
ros::Subscriber <moobot_msgs::conveyor_status> conveyor_status_sub("/conveyor_status", &runConveyor);
ros::Subscriber <moobot_msgs::moobot_status> moobot_status_sub_conveyor("/agv_status", &stopConveyor);
ros::Publisher conveyor_done_pub("/conveyor_transfer_done", & conveyor_transfer_done_msg);

void setup()
{
  pinMode(sensor1_pin2, INPUT_PULLUP);
  pinMode(sensor2_pin2, INPUT_PULLUP);
  pinMode(powersupply, OUTPUT);
  pinMode(nSleep, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(vref, OUTPUT);
  analogWrite(vref, 255);
  digitalWrite(in1, HIGH);  //yÃ¶n pini 1 ise dÃ¼z
  nh.initNode();
  nh.advertise(conveyor_done_pub);
  nh.subscribe(conveyor_status_sub);
  nh.subscribe(moobot_status_sub_conveyor);

}
bool make_motor_slower = false;
unsigned long time_t = 0;
void loop()
{
  s_start_pin2 = digitalRead(sensor1_pin2);
  s_stop_pin2 = digitalRead(sensor2_pin2);
  S1_ON =  !(s_start_pin2); // sensor verileri terslendi
  S2_ON =  !(s_stop_pin2);

  if (S1_ON == 1 && S2_ON == 1) conveyor_loaded = true;
  else conveyor_loaded = false;

  if (stop_conveyor) {
    stop_motor();
  } else {
    if (unload_conveyor && S2_ON == 0 && S1_ON == 0) {
      unload_conveyor_done = true;
      //delay(1000);
    }
    

    if (load_conveyor && S2_ON == 1) {
      load_conveyor_done = true;
    }
    if (load_conveyor && S1_ON == 1) {
      make_motor_slower = true;
      time_t = millis();
    }
    if (load_conveyor && make_motor_slower) {
       if((millis() - time_t) > 1500){
         analogWrite(in2, 20);
       }
    }

    if (load_conveyor && !load_conveyor_done) {
      digitalWrite(in1, HIGH); 
      analogWrite(in2, 89); 
      run_motor();
    } else if (load_conveyor && load_conveyor_done) {
      stop_motor();
      nh.loginfo("stop motor load");
      load_conveyor = false;
    } else if (unload_conveyor && !unload_conveyor_done) { // load done
      //digitalWrite(in1, LOW); 
      analogWrite(in2, 120); 
      run_motor();
      
    } else if (unload_conveyor && unload_conveyor_done) { // unload done
      stop_motor();
      nh.loginfo("stop motor unload");
      unload_conveyor = false;
      unload_conveyor_done = false;
      
    }
  }
//  if (load_conveyor) nh.loginfo("load true");
//  if (unload_conveyor) nh.loginfo("unload true");
//  if (load_conveyor_done) nh.loginfo("load done true");
//  if (unload_conveyor_done) nh.loginfo("unload done true");
//  
//  if (S1_ON) nh.loginfo("S1 : 1");
//  else nh.loginfo("S1 : 0");
//  if (S2_ON) nh.loginfo("S2 : 1");
//  else nh.loginfo("S2 : 0");
  nh.spinOnce();

}



void stop_motor()
{
  digitalWrite(nSleep, LOW);
  conveyor_transfer_done_msg.data = true;
  conveyor_done_pub.publish(&conveyor_transfer_done_msg);
}

void run_motor()
{
  
  digitalWrite(nSleep, HIGH);
  conveyor_transfer_done_msg.data = false;
  //conveyor_done_pub.publish(&conveyor_transfer_done_msg);
}
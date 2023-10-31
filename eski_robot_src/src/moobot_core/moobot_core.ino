#include <Encoder.h>
#include <ros.h>
#include <ros/time.h>
#include "geometry_msgs/Twist.h"
#include "moobot_msgs/moobot_status.h"
#include "std_msgs/Bool.h"

#define DIAGNOSTIC_STATUS_LENGTH 2
#define BAUD 57600

#define WHEEL_NUM                        2

#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

//////////////////////////////////////////////////
// AGV Mekanik Parametreler
//////////////////////////////////////////////////
#define wheel_radius 0.1000 // tekerlek yarıçapı // [m]
#define wheel_perimeter (2.0000 * M_PI * wheel_radius) // tekerlek çevresi [m]
#define wheel_length 0.58 // iki tekerlek arası mesafe // [m]
#define encoder_ppr 28672.0
//////////////////////////////////////////////////


//////////////////////////////////////////////////
// Motor Pinleri
//////////////////////////////////////////////////
// Motor - left
unsigned static int m1_in1 = 9; //(PH)
unsigned static int m1_in2 = 10; //(EN)
unsigned static int m1_nSleep = 11;
unsigned static int m1_vref = 12;
unsigned static int m1_SNSout = A1;
unsigned static int m1_nFault = 20; //Belirlenecek

// Motor - right
unsigned static int m2_in1 = 4; //(PH)
unsigned static int m2_in2 = 5; //(EN)
unsigned static int m2_nSleep = 6;
unsigned static int m2_vref = 7;
unsigned static int m2_SNSout = A0;
unsigned static int m2_nFault = 13;//Belirlenecek
//////////////////////////////////////////////////


//////////////////////////////////////////////////
// Enkoder Parametreleri
//////////////////////////////////////////////////
Encoder Tire_l(2, 3);
Encoder Tire_r(19, 18);

bool enc_msgs_ok = false;
double velTimer_old = 0.00;
double velTimer_new = 0.00;
double accTimer_old = 0.00;
double accTimer_new = 0.00;
long tire_old_l = 0,
     tire_old_r = 0;
double tireAngles[2] = {0, 0};
double oldtireAngles[2] = {0, 0};
//////////////////////////////////////////////////


//////////////////////////////////////////////////
// Hız Kontrol Parametreleri
//////////////////////////////////////////////////

double left_wheel_distance = wheel_length / 2;
double right_wheel_distance = wheel_length / 2;

unsigned long timer_motor_speed = 0;

float w_speed_act[2];
float w_speed_des[2];
float linearSpeed_act, angularSpeed_act;

//PID PID PID
double speedl_input, speedl_output, speedl_setpoint, speedl_kp, speedl_ki, speedl_kd; //speedleft
double speedr_input, speedr_output, speedr_setpoint, speedr_kp, speedr_ki, speedr_kd; //speedright

float ades_l = 0.0,
      ades_r = 0.0, //!!! karar verilecek
      saturated_ades_l = 0.0,
      saturated_ades_r = 0.0;
float linearSpeed = 0.0,
      angularSpeed = 0.0; //Linear in m/s, angular in rad/s


//////////////////////////////////////////////////
// AGV PARAMETRELERİ
//////////////////////////////////////////////////

int agv_mode = 1;
bool agv_stopped = false;

//////////////////////////////////////////////////
// ROS
//////////////////////////////////////////////////

ros::NodeHandle nh;
geometry_msgs::Twist cmd_vel_act_msg;


void speeedUpdate(const geometry_msgs::Twist& vel_msg) {

  linearSpeed = vel_msg.linear.x;
  angularSpeed = vel_msg.angular.z;
}

void statusUpdate(const moobot_msgs::moobot_status &status_msg) {
  agv_stopped = status_msg.agv_stopped;
}

void enableLineFollow(const std_msgs::Bool &msg){
  if(msg.data == true){
    left_wheel_distance = 0.185;
    right_wheel_distance = 0.395;
  }else{
    left_wheel_distance = right_wheel_distance = wheel_length / 2;
  }
}

ros::Subscriber <geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &speeedUpdate);
ros::Subscriber <moobot_msgs::moobot_status> moobot_sub("/agv_status", &statusUpdate);
ros::Publisher cmd_vel_act_pub("/cmd_vel_act", &cmd_vel_act_msg);
ros::Subscriber <std_msgs::Bool> line_follow_mode_sub("/follow_line_mode", &enableLineFollow);



void setup() {

  // Set Baudrate between Arduino - Up2 connection
  nh.getHardware()->setBaud(BAUD);

  //////////////////////////////////////////////////
  // PinMode Tanımlamaları
  //////////////////////////////////////////////////
  pinMode(m1_nSleep, OUTPUT);
  pinMode(m1_in2, OUTPUT);
  pinMode(m1_in1, OUTPUT);
  pinMode(m1_vref, OUTPUT);
  pinMode(m1_nFault, INPUT);
  pinMode(m2_nSleep, OUTPUT);
  pinMode(m2_in2, OUTPUT);
  pinMode(m2_in1, OUTPUT);
  pinMode(m2_vref, OUTPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(m2_nFault, INPUT);

  //////////////////////////////////////////////////
  // Motor Pin Değerlerinin Atanması
  //////////////////////////////////////////////////
  // Motor 1
  digitalWrite(m1_in1, HIGH);  //yön pinleri 0 verirsem ters 1 verirsem düz
  digitalWrite(m1_nSleep, HIGH);
  analogWrite(m1_vref, 255);
  // Motor 2
  analogWrite(m2_vref, 255);
  digitalWrite(m2_in1, LOW);//yön pinleri 0 verirsem ters 1 verirsem düz
  digitalWrite(m2_nSleep, HIGH);

  speedl_kp = speedr_kp = 1.5;
  speedl_ki = speedr_ki = 0.04;
  speedl_kd = speedr_kd = 0;


  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(moobot_sub);
  nh.advertise(cmd_vel_act_pub);
  nh.subscribe(line_follow_mode_sub);

}


void loop() {

  if (nh.connected())
  {
    giveSpeedtoAGV();
    update_cmd_vel_act();
  }
  else
  {
    digitalWrite(m1_nSleep, LOW);
    digitalWrite(m2_nSleep, LOW);
    linearSpeed = 0.0;
    angularSpeed = 0.0;
  }

  nh.spinOnce();
}

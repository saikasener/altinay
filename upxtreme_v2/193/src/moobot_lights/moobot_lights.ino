/*
  8x8 LED Matrix MAX7219
  Based on the following library:
  GitHub | riyas-org/max7219 https://github.com/riyas-org/max7219
*/

#include <ros.h>
#include <ros/time.h>
#include <MaxMatrix.h>
#include "moobot_msgs/moobot_status.h"
#include "moobot_msgs/bms_status.h"
#include <actionlib_msgs/GoalStatusArray.h>
#define BAUD 57600

////////////////////////////////////////////////////////////
// Relay Input Pins - Turns on with pulling it to GND
////////////////////////////////////////////////////////////
int red_light = 9;
int green_light = 10;
int yellow_light = 11;
int buzzer_on = 12;
////////////////////////////////////////////////////////////

bool buzzer_status = false;

// Front LEDs
int CLK_Pin = 6; // CLK pin of MAX7219 module
int CS_Pin = 7;  // CS pin of MAX7219 module
int DIN_Pin = 5;
int front_right = 1;
int front_left = 2;
int rear_left = 3;
int rear_right = 4;

MaxMatrix front_right_leds(DIN_Pin, CS_Pin, CLK_Pin, front_right);
MaxMatrix front_left_leds(DIN_Pin, CS_Pin, CLK_Pin, front_left);
MaxMatrix rear_right_leds(DIN_Pin, CS_Pin, CLK_Pin, rear_right);
MaxMatrix rear_left_leds(DIN_Pin, CS_Pin, CLK_Pin, rear_left);

unsigned char const DUZ[] = {
    8,
    9,
    B00111100,
    B01100110,
    B11000011,
    B10011001,
    B10011001,
    B11000011,
    B01100110,
    B00111100,
};
unsigned char const SAG[] = {
    8,
    8,
    B00111100,
    B01100110,
    B11000011,
    B10001101,
    B10001101,
    B11000011,
    B01100110,
    B00111100,
};
unsigned char const SOL[] = {8, 8,
                             B00111100,
                             B01100110,
                             B11000011,
                             B10110001,
                             B10110001,
                             B11000011,
                             B01100110,
                             B00111100};
unsigned char const UYARI[] = {
    8,
    8,
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00000000,
    B00011000,
};
unsigned char const SARJBOS[] = {
    8,
    8,
    B00011000,
    B01111110,
    B01000010,
    B01000010,
    B01000010,
    B01000010,
    B01111110,
    B01111110,
};
unsigned char const SARJOLUYOR[] = {
    8,
    8,
    B00011000,
    B01111110,
    B01000010,
    B01111110,
    B01000010,
    B01000010,
    B01111110,
    B01111110,
};

unsigned char const FULL[] = {
    8,
    8,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
};

unsigned char const FULLRIGHT[] = {
    8,
    8,
    B00000000,
    B00000000,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
};

unsigned char const FULLLEFT[] = {
    8,
    8,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B00000000,
    B00000000,
};

unsigned char const TURNOFF[] = {
    8,
    8,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
};
//////////////////////////////////////////////////
// ROS
//////////////////////////////////////////////////"

int agv_mode = 0;
void updateLed(const moobot_msgs::moobot_status &agv_status_msg)
{

  if (agv_status_msg.agv_stopped == true)
  {

    digitalWrite(red_light, LOW);
    digitalWrite(green_light, HIGH);
    digitalWrite(yellow_light, HIGH);
    digitalWrite(buzzer_on, HIGH);
    buzzer_status = false;
  }
  else
  {

    agv_mode = agv_status_msg.agv_mode;

    if (agv_status_msg.agv_mode == 0)
    {
      digitalWrite(green_light, LOW);
      digitalWrite(yellow_light, HIGH);
      digitalWrite(red_light, HIGH);
      digitalWrite(buzzer_on, HIGH);
      buzzer_status = true;
    }
    else if (agv_status_msg.agv_mode == 1)
    {
      digitalWrite(yellow_light, LOW);
      digitalWrite(red_light, HIGH);
      digitalWrite(green_light, HIGH);
      digitalWrite(buzzer_on, HIGH);
      buzzer_status = false;
    }
  }
}

int old_status = 0;

void updateBuzzer(const actionlib_msgs::GoalStatusArray &msg_goal)
{
  //if (msg_goal.status_list != NULL) {
  if (old_status != msg_goal.status_list[0].status && msg_goal.status_list[0].status == 3)
  {

    digitalWrite(buzzer_on, LOW);
    digitalWrite(red_light, HIGH);
    digitalWrite(yellow_light, HIGH);
    digitalWrite(green_light, HIGH);
    delay(1000);

    //MODUNA GERI DONER..
    if (agv_mode == 1)
    {
      digitalWrite(yellow_light, LOW);
      digitalWrite(red_light, HIGH);
      digitalWrite(green_light, HIGH);
      digitalWrite(buzzer_on, HIGH);
    }
    else if (agv_mode == 0)
    {
      digitalWrite(green_light, LOW);
      digitalWrite(yellow_light, HIGH);
      digitalWrite(red_light, HIGH);
      digitalWrite(buzzer_on, HIGH);
    }

    //}
  }
  old_status = msg_goal.status_list[0].status;
}

void bmsUpdate(const moobot_msgs::bms_status &bms_msg)
{
  if (bms_msg.battery_volt_total < 23.0)
  buzzer_status == true;
}

ros::NodeHandle nh;
ros::Subscriber<moobot_msgs::moobot_status> agv_status_sub_led("/agv_status", &updateLed);
ros::Subscriber<actionlib_msgs::GoalStatusArray> goal_reached_sub_led("/move_base/status", &updateBuzzer);
ros::Subscriber<moobot_msgs::bms_status> bms_status_sub("/bms_status", &bmsUpdate);

void setup()
{

  // Set Baudrate between Arduino - Up2 connection
  nh.getHardware()->setBaud(BAUD);

  // Stack light
  pinMode(red_light, OUTPUT);
  pinMode(green_light, OUTPUT);
  pinMode(yellow_light, OUTPUT);
  pinMode(buzzer_on, OUTPUT);

  digitalWrite(red_light, LOW);
  digitalWrite(green_light, HIGH);
  digitalWrite(yellow_light, HIGH);
  digitalWrite(buzzer_on, HIGH);

  // Front LEDs Initialization
  front_right_leds.init();          // MAX7219 initialization
  front_right_leds.setIntensity(5); // initial led matrix intensity, 0-15
  front_left_leds.init();           // MAX7219 initialization
  front_left_leds.setIntensity(5);  // initial led matrix intensity, 0-15

  // Rear LEDs Initialization
  rear_right_leds.init();          // MAX7219 initialization
  rear_right_leds.setIntensity(1); // initial led matrix intensity, 0-15
  rear_left_leds.init();           // MAX7219 initialization
  rear_left_leds.setIntensity(1);  // initial led matrix intensity, 0-15

  front_right_leds.writeSprite(0, 0, DUZ);
  front_left_leds.writeSprite(0, 0, DUZ);
  rear_right_leds.writeSprite(0, 0, FULL);
  rear_left_leds.writeSprite(0, 0, FULL);

  nh.initNode();
  nh.subscribe(agv_status_sub_led);
  nh.subscribe(goal_reached_sub_led);
}
void loop()
{
  if (buzzer_status == true)
    digitalWrite(buzzer_on, LOW);
  else
    digitalWrite(buzzer_on, HIGH);

  nh.spinOnce();
}

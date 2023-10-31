#include <ros.h>
#include <ros/time.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "sensor_msgs/Imu.h"
#include "moobot_msgs/SensorState.h"

#define BAUD                                   57600
#define IMU_PUBLISH_FREQUENCY                  200  // 200 hz

//////////////////////////////////////////////////
// IMU (for mega)
// SDA -> 20
// SCL -> 21
//////////////////////////////////////////////////

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
//moobot_msgs::SensorState sensor_state_msg;

ros::Publisher imu_pub("/imu", &imu_msg);
//ros::Publisher sensor_state_pub("/sensor_state", &sensor_state_msg);
static uint32_t tTime[2];


void setup() {
  nh.getHardware()->setBaud(BAUD);

  nh.initNode();
  nh.advertise(imu_pub);
  bno.begin(); //start imu
  updateIMU();



}

void loop() {
  uint8_t system_status, self_test_results, system_error;

  system_status = self_test_results = system_error = 0;

  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  //uint32_t t = millis();

  if (nh.connected()) {
    //if ((t - tTime[0]) >= (1000 / IMU_PUBLISH_FREQUENCY)){
    updateIMU();

    // }

  } else {
    nh.logerror("IMU is not connected.");
  }
  nh.spinOnce(); //TODO buraya rate koy.........




}

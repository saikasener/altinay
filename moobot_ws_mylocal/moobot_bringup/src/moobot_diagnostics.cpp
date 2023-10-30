/*
  Reads sensor data and gets time.
  If a sensor did not send any data for a time publishes error message.
  This diagnostic data is used for sending safety plc.
*/
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <moobot_msgs/SensorState.h>
#include <moobot_msgs/moobot_sensor_status.h>
#include <moobot_msgs/moobot_scanner.h>

#include <string>

ros::Publisher moobot_diagnostics_pub;
ros::Publisher moobot_diagnostic_status_pub;

diagnostic_msgs::DiagnosticStatus imu_state;
// diagnostic_msgs::DiagnosticStatus motor_state;
diagnostic_msgs::DiagnosticStatus BS_state;
diagnostic_msgs::DiagnosticStatus FS_state;
diagnostic_msgs::DiagnosticStatus battery_state;
time_t imu_last_time;
time_t fs_last_time;
time_t bs_last_time;

#define IMU_ERROR_STATE 1
#define SCAN_ERROR_STATE 2

void split(std::string data, std::string separator, std::string *temp)
{
  int cnt = 0;
  std::string copy = data;

  while (true)
  {
    std::size_t index = copy.find(separator);

    if (index != std::string::npos)
    {
      temp[cnt] = copy.substr(0, index);

      copy = copy.substr(index + 1, copy.length());
    }
    else
    {
      temp[cnt] = copy.substr(0, copy.length());
      break;
    }

    ++cnt;
  }
}

void setDiagnosisMsg(diagnostic_msgs::DiagnosticStatus *diag, uint8_t level, std::string name, std::string message, std::string hardware_id)
{
  diag->level = level;
  diag->name = name;
  diag->message = message;
  diag->hardware_id = hardware_id;
}

void setIMUDiagnosis(uint8_t level, std::string message)
{
  imu_last_time = time(0);
  setDiagnosisMsg(&imu_state, level, "IMU Sensor", message, "BNO055");
}

void setBatteryDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&battery_state, level, "Power System", message, "Battery");
}

void setFSDiagnosis(uint8_t level, std::string message)
{
  fs_last_time = time(0);
  setDiagnosisMsg(&FS_state, level, "Front Scanner", message, "Front SICK_Scanner");
}

void setBSDiagnosis(uint8_t level, std::string message)
{
  bs_last_time = time(0);
  setDiagnosisMsg(&BS_state, level, "Back Scanner", message, "Back SICK_Scanner");
}

void imuMsgCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  setIMUDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}

void FSMsgCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  setFSDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}
void BSMsgCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  setBSDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}

void sensorStateMsgCallback(const moobot_msgs::SensorState::ConstPtr &msg)
{
  if (msg->battery > 11.0)
    setBatteryDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
  else
    setBatteryDiagnosis(diagnostic_msgs::DiagnosticStatus::WARN, "Charge!!! Charge!!!");
}

void msgPub()
{
  diagnostic_msgs::DiagnosticArray moobot_diagnostics;
  moobot_msgs::moobot_sensor_status moobot_sensor_status_msg;

  moobot_diagnostics.header.stamp = ros::Time::now();
  moobot_diagnostics.status.clear();
  time_t current_time = time(0);
  if (current_time - imu_last_time > 6)
  {
    setIMUDiagnosis(diagnostic_msgs::DiagnosticStatus::ERROR, "No imu message");
    moobot_sensor_status_msg.state_type = IMU_ERROR_STATE;
    moobot_sensor_status_msg.state = "IMU message error";
    moobot_diagnostic_status_pub.publish(moobot_sensor_status_msg);
  }
  moobot_diagnostics.status.push_back(imu_state);
  if (current_time - fs_last_time > 3 || current_time - bs_last_time > 3)
  {
    setFSDiagnosis(diagnostic_msgs::DiagnosticStatus::ERROR, "No front scanner message");
    setBSDiagnosis(diagnostic_msgs::DiagnosticStatus::ERROR, "No back scanner message");
    moobot_sensor_status_msg.state_type = SCAN_ERROR_STATE;
    moobot_sensor_status_msg.state = "Scanner message error";
    moobot_diagnostic_status_pub.publish(moobot_sensor_status_msg);
  }
  moobot_diagnostics.status.push_back(FS_state);
  moobot_diagnostics.status.push_back(BS_state);
  // moobot_diagnostics.status.push_back(battery_state);
  moobot_diagnostics_pub.publish(moobot_diagnostics);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moobot_diagnostic");
  ros::NodeHandle nh;

  moobot_diagnostics_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);
  moobot_diagnostic_status_pub = nh.advertise<moobot_msgs::moobot_sensor_status>("/agv_sensor_status", 10);

  ros::Subscriber imu = nh.subscribe("/imu_data", 10, imuMsgCallback);
  ros::Subscriber fs_scan_sub = nh.subscribe("/fs_scan", 10, FSMsgCallback);
  ros::Subscriber bs_scan_sub = nh.subscribe("/bs_scan", 10, BSMsgCallback);
  ros::Subscriber moobot_sensor = nh.subscribe("/sensor_state", 10, sensorStateMsgCallback);

  // ros::Rate loop_rate(10);

  while (ros::ok())
  {
    msgPub();
    ros::spinOnce();
    // loop_rate.sleep();
  }

  return 0;
}

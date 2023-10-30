#ifndef MOOBOT_HMI_H
#define MOOBOT_HMI_H

#include <QMainWindow>
#include "switchbutton.h"
#include "switch.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QTimer>
#include "statuspanel.h"
#include "agv_nav.h"
//#include "showspeed.h"
#include "joypad.h"
#include <QLabel>
#include <QTimer>
#include "moobot_msgs/moobot_status.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "moobot_msgs/bms_status.h"
#include "std_msgs/String.h"
enum{
    AUTO,
    MANUAL,
    SERVICE
};
namespace Ui {
class Moobot_hmi;
}

class Moobot_hmi : public QMainWindow
{
    Q_OBJECT

public:
    explicit Moobot_hmi(QWidget *parent = 0);
    ~Moobot_hmi();
    void initWidgets();
    Agv_nav* agv_nav;
    ros::Publisher moobot_status_pub;
    ros::Subscriber cmd_vel_act_sub;
    ros::Publisher twist_pub_d;
    ros::Publisher cancel_pub;
    ros::Subscriber moobot_status_dashboard_sub;
    ros::Subscriber cmd_vel_auto_sub; //subscribes only in auto mode.
    ros::Subscriber bms_sub_d;
    ros::Subscriber agv_job_status_sub;
    QTimer *timer;
    bool old_mode;
    bool old_agv_stopped;
    void updateActualSpeeds(const geometry_msgs::Twist &msg);
    void updateStatus(const moobot_msgs::moobot_status &msg);
    void updateDesiredSpeedsAuto(const geometry_msgs::Twist &msg);
    void updateBmsStatus(const moobot_msgs::bms_status &msg);
    void updateAgvJobStatus(const std_msgs::String &msg);
    ros::NodeHandle nh;
    ros::Rate loop_rate;




public slots:
    void on_startStopButton_pressed();
    void changeMode(bool mode);
signals:
    void stopSignal(bool isStopped);



private:
    Ui::Moobot_hmi *ui;
    QWidget* main_widget;
    QHBoxLayout * main_layout;
    QWidget* upper_widget;
    QHBoxLayout * upper_layout;
    QLabel* auto_label, *manual_label;
    QLabel* altinay_logo;
    Switch *mode_switch;
    ShowSpeed * speed_widget;
    QPushButton * startStopButton;
    QWidget* right_widget;
    QVBoxLayout* right_layout;
    QHBoxLayout* battery_layout, *battery_label_layout;
    QWidget* battery_widget, *battery_label_widget;
    QLabel* bms_voltage, *bms_current, *bms_power, *bms_temperature;
    QLabel* bms_voltage_label, *bms_current_label, *bms_power_label, *bms_temperature_label;
    QWidget* message_widget;
    QLabel* message_label;
    Joypad* joypad_widget;
    QVBoxLayout* dashboard_layout;
    bool isStopped = false;
    QString continue_path;
    QString stop_path;


private slots:
    void callSpin();

};

#endif // MOOBOT_HMI_H

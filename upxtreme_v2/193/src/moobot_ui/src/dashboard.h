#ifndef DASHBOARD_H
#define DASHBOARD_H

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
class Dashboard;
}

class Dashboard : public QMainWindow
{
    Q_OBJECT

public:
    explicit Dashboard(QWidget *parent = 0);
    ~Dashboard();
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
    void on_save_map_button_pressed();
    void on_run_navigation_button_pressed();
    void on_stop_navigation_button_pressed();
    void on_start_mapping_button_pressed();
    void on_stop_mapping_button_pressed();
    void on_start_loop_button_pressed();
    void on_stop_loop_button_pressed();
    void changeMode(bool mode);
signals:
    void stopSignal(bool isStopped);



private:
    Ui::Dashboard *ui;
    QWidget* main_widget;
    QVBoxLayout * main_layout;
    QWidget* upper_widget;
    QHBoxLayout * upper_layout;
    QWidget* lower_widget;
    QHBoxLayout * lower_layout;
    QLabel* auto_label, *manual_label;
    QLabel* altinay_logo;
    Switch *mode_switch;
    ShowSpeed * speed_widget;
    QPushButton * startStopButton;
    QWidget * map_menu_widget;
    QHBoxLayout * map_menu_layout;
    QPushButton * start_mapping_button;
    QPushButton * stop_mapping_button;
    QPushButton * start_loop_button;
    QPushButton * stop_loop_button;
    QPushButton * save_map_button;
    QPushButton * run_navigation_button;
    QPushButton * stop_navigation_button;
    QWidget* map_main_widget;
    QVBoxLayout* map_main_layout;
    QWidget* right_widget;
    QVBoxLayout* right_layout;
    QHBoxLayout* battery_layout, *battery_label_layout;
    QWidget* battery_widget, *battery_label_widget;
    QLabel* bms_voltage, *bms_current, *bms_power, *bms_temperature;
    QLabel* bms_voltage_label, *bms_current_label, *bms_power_label, *bms_temperature_label;
    QWidget* message_widget;
    QLabel* message_label;
    Joypad* joypad_widget;
    StatusPanel* status_widget;
    QVBoxLayout* dashboard_layout;
    bool isStopped = false;
    QString continue_path;
    QString stop_path;


private slots:
    void callSpin();

};

#endif // DASHBOARD_H

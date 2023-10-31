#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <QStandardPaths>

#include <QWidget>
#include <QStackedWidget>
#include <QPainter>
#include <QtMath>
#include <QTimer>
//#include <QParallelAnimationGroup>
//#include <QPropertyAnimation>
#include <QMouseEvent>
#include "geometry_msgs/Twist.h"
#include "../../devel/include/moobot_ui/agv_status.h"
#include "ros/ros.h"

//#include <QDebug>
#include <QObject>
#include <QLabel>
//#include <QGraphicsItem>
#include "showspeed.h"
#include "../../devel/include/diag_msg/diag_msg.h"

#define USER "rnd/moobot/moobot_ws"
#define angularSpeedMax 0.35
#define linearSpeedMax 0.2
#define angularSpeedMin -0.35
#define linearSpeedMin -0.2


namespace Ui {
class Joystick;
}
class ShowSpeed;

class Joystick : public QStackedWidget
{
    Q_OBJECT

public:

    explicit Joystick(QStackedWidget *parent = 0);
    ~Joystick();
    bool isStopPressed;
    double angularSpeed;
    double linearSpeed;
    int y1_joypad;
    int x1_joypad;
    int y2_joypad;
    int x2_joypad;
    int x_center_joypad;
    int y_center_joypad;
    int currentPage = 0;
    int diag_time = 0;
    double maxAngularSpeed = angularSpeedMax;
    double maxLinearSpeed = linearSpeedMax;
    double minAngularSpeed = angularSpeedMin;
    double minLinearSpeed = linearSpeedMin;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

    QTimer *timer;



private slots:

    void on_stop_button_pressed();

    void on_auto_button_pressed();

    void on_manual_button_pressed();

    void on_page_button_left_clicked();

    void on_page_button_right_clicked();

    void on_exitButton_clicked();


    void on_set_speed_button_clicked();

    void callSpin();

    void on_set_speed_button_max_clicked();

    void on_set_speed_button_min_clicked();

private:
    Ui::Joystick *ui;
    ros::Publisher twist_pub;
    ros::Publisher agv_status_pub;

};

#endif // JOYSTICK_H


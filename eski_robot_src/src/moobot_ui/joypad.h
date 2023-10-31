#ifndef JOYPAD_H
#define JOYPAD_H

#include <QWidget>
#include <QStandardPaths>

#include <QWidget>

#include <QPainter>
#include <QtMath>
#include <QTimer>
//#include <QParallelAnimationGroup>
//#include <QPropertyAnimation>
#include <QMouseEvent>
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

//#include <QDebug>
#include <QObject>
#include <QLabel>


#include "../../devel/include/diag_msg/diag_msg.h"

#define USER "rnd/moobot/moobot_ws"
#define angularSpeedMax 0.35
#define linearSpeedMax 0.2
#define angularSpeedMin -0.35
#define linearSpeedMin -0.2

namespace Ui {
class Joypad;
}

class Joypad : public QWidget
{
    Q_OBJECT

public:
    explicit Joypad(QWidget *parent = 0);
    ~Joypad();

    bool isStopPressed;
    double angularSpeed;
    double linearSpeed;
    int y1_joypad;
    int x1_joypad;
    int y2_joypad;
    int x2_joypad;
    int x_center_joypad;
    int y_center_joypad;
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
    void on_set_speed_button_clicked();

    void callSpin();

private:
    Ui::Joypad *ui;
    ros::Publisher twist_pub;
    ros::Publisher agv_status_pub;
};

#endif // JOYPAD_H

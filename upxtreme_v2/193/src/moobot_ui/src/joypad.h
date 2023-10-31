#ifndef JOYPAD_H
#define JOYPAD_H

#include <QWidget>
#include <QStandardPaths>
#include <QWidget>
#include <QtMath>
#include <QTimer>
#include <QObject>
#include <QLabel>
#include <QMouseEvent>
#include <QPixmap>

#include "showspeed.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include  <moobot_msgs/moobot_status.h>


#define USER "rnd/moobot_ws"
#define angularSpeedMax 3.0
#define linearSpeedMax 1.35
#define angularSpeedMin -3.0
#define linearSpeedMin -1.35

namespace Ui {
class Joypad;
}

class Joypad : public QWidget
{
    Q_OBJECT

public:
    explicit Joypad(QWidget *parent = 0);
    ~Joypad();
    QLabel * thumb;
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
    ros::Rate loop_rate;
signals:
   void speedSignal();
   void maxSpeedSignal();
   void minSpeedSignal();

public slots:
    void on_set_speed_button_clicked();
    void on_set_speed_button_max_clicked();
    void on_set_speed_button_min_clicked();
    void on_stopButtonPressed(bool isStopped);
    void callSpin();

private slots:
    void on_reduce_ratio_valueChanged(int arg1);

private:
    Ui::Joypad *ui;
    ros::Publisher twist_pub;
    ros::Publisher agv_status_pub;
};

#endif // JOYPAD_H

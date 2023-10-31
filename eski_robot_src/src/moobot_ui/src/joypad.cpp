#include "joypad.h"
#include "ui_joypad.h"


geometry_msgs::Twist twist_msg;
moobot_msgs::moobot_status agv_status_msg_joystick;


Joypad::Joypad(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Joypad),
    loop_rate(10)

{
    ui->setupUi(this);
    isStopPressed = false;
    x1_joypad=ui->JoyPad->x();
    y1_joypad=ui->JoyPad->y();
    x2_joypad=x1_joypad + ui->JoyPad->width();
    y2_joypad=y1_joypad + ui->JoyPad->height();
    x_center_joypad = (x1_joypad + x2_joypad) / 2;
    y_center_joypad = (y1_joypad + y2_joypad) / 2;


    QString joy_path("/home/"); joy_path = joy_path+ USER+ "/src/moobot_ui/src/pictures/joystick_background.png";
    QPixmap pixmap_joy(joy_path);
    ui->JoyPad->setPixmap(pixmap_joy.scaled(230,230,Qt::KeepAspectRatio));

    thumb = new QLabel(this);
    thumb->setFixedSize(60,60);

    QString thumb_path("/home/"); thumb_path = thumb_path+ USER+ "/src/moobot_ui/src/pictures/joystick_thumb.png";
    QPixmap pixmap_thumb(thumb_path);
    thumb->setPixmap(pixmap_thumb.scaled(60,60,Qt::KeepAspectRatio));
    thumb->setGeometry(QRect((x_center_joypad - 30),(y_center_joypad - 30),60,60));


    ShowSpeed::linearSpeed_des=0;
    ShowSpeed::angularSpeed_des=0;
    ShowSpeed::linearSpeed_max = linearSpeedMax;
    ShowSpeed::linearSpeed_min = linearSpeedMin;
    ShowSpeed::angularSpeed_max = angularSpeedMax;
    ShowSpeed::angularSpeed_min = angularSpeedMin;
    emit speedSignal();
    emit maxSpeedSignal();
    emit minSpeedSignal();


    static ros::NodeHandle nh;

    if (!ros::ok())
    {
        ROS_INFO("No master started!");
        this->close();
    }
    twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    agv_status_pub = nh.advertise<moobot_msgs::moobot_status>("/agv_status", 1000);

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(callSpin()));
    timer->start(100);

}

Joypad::~Joypad()
{
    delete timer;
    delete ui;
}
void Joypad::mousePressEvent(QMouseEvent *event){

    if(!isStopPressed){
        int x = event->x();
        int y = event->y();
        int distance = sqrt(pow((x-x_center_joypad),2)+pow((y-y_center_joypad),2));
        int r= ui->JoyPad->width()/2;


        if(x > x1_joypad && x < x2_joypad && y > y1_joypad && y < y2_joypad && distance < r){

            QWidget::mouseMoveEvent(event);
            thumb->setGeometry(QRect((x - 30),(y-30),60,60));

        }
    }


}

void Joypad::mouseMoveEvent(QMouseEvent *event){

    if(!isStopPressed){

        int x = event->x();
        int y = event->y();
        int distance = sqrt(pow((x-x_center_joypad),2)+pow((y-y_center_joypad),2));
        int r= ui->JoyPad->width()/2;


        if(x > x1_joypad && x < x2_joypad && y > y1_joypad && y < y2_joypad && distance < r){

            QWidget::mouseMoveEvent(event);
            thumb->setGeometry((x-30),(y-30),60,60);


            if( y <= y_center_joypad){
                linearSpeed=(double)(y_center_joypad-y) * maxLinearSpeed/(ui->JoyPad->height()/2);
                if( x < x_center_joypad)  angularSpeed= (double)  (-1) *(x_center_joypad-x) * minAngularSpeed / (ui->JoyPad->width()/2);
                else angularSpeed=(double) (-1) *(x- x_center_joypad ) * maxAngularSpeed / (ui->JoyPad->width()/2);
            }else{
                linearSpeed=(double)(y-y_center_joypad) * minLinearSpeed/(ui->JoyPad->height()/2);
                if( x < x_center_joypad)  angularSpeed= (double)  (-1) * (x-x_center_joypad) * minAngularSpeed / (ui->JoyPad->width()/2);
                else angularSpeed=(double) (-1) *(x_center_joypad -x) * maxAngularSpeed / (ui->JoyPad->width()/2);
            }

            twist_msg.angular.z=angularSpeed;
            twist_msg.linear.x=linearSpeed;
            while (twist_pub.getNumSubscribers() == 0)
                  loop_rate.sleep();
            twist_pub.publish(twist_msg);


            ShowSpeed::linearSpeed_des=linearSpeed;
            ShowSpeed::angularSpeed_des=angularSpeed;
            emit speedSignal();

            ui->linear_val->setText(QString("<span style=\" font-size:16pt; font-weight:600;\"><center>%1</center></span>").arg(QString::number(linearSpeed,'f',ShowSpeed::precision)));
            ui->angular_val->setText(QString("<span style=\" font-size:16pt; font-weight:600;\"><center>%1</center></span>").arg(QString::number(angularSpeed,'f',ShowSpeed::precision)));
        }
    }


}
void Joypad::mouseReleaseEvent(QMouseEvent *event){

    int x = event->x();
    int y = event->y();
    if(x > (x1_joypad - 100) && x < (x2_joypad + 100) && y > (y1_joypad - 100) && y < (y2_joypad + 100) ){ //fazla sürükleme ihtimaline karşılık

        QWidget::mouseReleaseEvent(event);
        thumb->setGeometry(QRect((x_center_joypad - 30),(y_center_joypad - 30),60,60));

        linearSpeed = 0.0;
        angularSpeed = 0.0;

        twist_msg.angular.z=angularSpeed;
        twist_msg.linear.x=linearSpeed;
        twist_pub.publish(twist_msg);

        ShowSpeed::linearSpeed_des=linearSpeed;
        ShowSpeed::angularSpeed_des=angularSpeed;
        emit speedSignal();

        ui->linear_val->setText(QString("<span style=\" font-size:16pt; font-weight:600;\"><center>%1</center></span>").arg(QString::number(linearSpeed,'f',ShowSpeed::precision)));
        ui->angular_val->setText(QString("<span style=\" font-size:16pt; font-weight:600;\"><center>%1</center></span>").arg(QString::number(angularSpeed,'f',ShowSpeed::precision)));
    }

}

void Joypad::on_set_speed_button_clicked()
{
    if(!isStopPressed){
        angularSpeed=ui->angular_speed_entered->value();
        linearSpeed = ui->linear_speed_entered->value();

        twist_msg.angular.z=angularSpeed;
        twist_msg.linear.x=linearSpeed;
        twist_pub.publish(twist_msg);
        ShowSpeed::linearSpeed_des=linearSpeed;
        ShowSpeed::angularSpeed_des=angularSpeed;

        emit speedSignal();
        ui->linear_val->setText(QString("<span style=\" font-size:18pt; font-weight:600;\"><center>%1</center></span>").arg(QString::number(linearSpeed,'f',ShowSpeed::precision)));
        ui->angular_val->setText(QString("<span style=\" font-size:18pt; font-weight:600;\"><center>%1</center></span>").arg(QString::number(angularSpeed,'f',ShowSpeed::precision)));
    }


}

void Joypad::on_set_speed_button_max_clicked()
{
    double ang_val = ui->max_angular_speed_entered->value();
    if(ang_val != 0.00){
        maxAngularSpeed = ang_val; //rad olarak girdiğini var sayıyorum
        ShowSpeed::angularSpeed_max = ang_val;
    }

    double lin_val = ui->max_linear_speed_entered->value();
    if(lin_val != 0.00){
        maxLinearSpeed = lin_val;
        ShowSpeed::linearSpeed_max = lin_val;

    }
    emit maxSpeedSignal();



}

void Joypad::on_set_speed_button_min_clicked()
{
    double ang_val = ui->min_angular_speed_entered->value();
    if(ang_val != 0.00){
        minAngularSpeed = ang_val; //rad olarak girdiğini var sayıyorum
        ShowSpeed::angularSpeed_min = ang_val;
    }
    double lin_val = ui->min_linear_speed_entered->value();
    if(lin_val != 0.00){
        minLinearSpeed = lin_val;
        ShowSpeed::linearSpeed_min = lin_val;      
    }
    emit minSpeedSignal();

}

void Joypad::on_stopButtonPressed(bool isStopped){
    this->isStopPressed = isStopped;
    if(isStopped){
        twist_msg.angular.z = 0.0;
        twist_msg.linear.x = 0.0;
        while (twist_pub.getNumSubscribers() == 0)
              loop_rate.sleep();
        twist_pub.publish(twist_msg);

    }
}

void Joypad::callSpin(){

    //ui->showSpeed->ShowSpeedValues();
    ros::spinOnce();
}

void Joypad::on_reduce_ratio_valueChanged(int reduce_ratio)
{
    maxLinearSpeed = ShowSpeed::linearSpeed_max * reduce_ratio / 100;
    maxAngularSpeed = ShowSpeed::angularSpeed_max * reduce_ratio / 100;
    minLinearSpeed = ShowSpeed::linearSpeed_min * reduce_ratio / 100;
    minAngularSpeed = ShowSpeed::angularSpeed_min * reduce_ratio / 100;
    emit maxSpeedSignal();
}

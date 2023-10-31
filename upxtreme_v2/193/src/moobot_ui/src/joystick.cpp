#include "joystick.h"
#include "ui_joystick.h"


geometry_msgs::Twist twist_msg;
moobot_ui::agv_status agv_status_msg_joystick;

Joystick::Joystick(QStackedWidget *parent) :
    QStackedWidget(parent),
    ui(new Ui::Joystick)
{
    ui->setupUi(this);
    isStopPressed = false;
    x1_joypad=ui->JoyPad->x();
    y1_joypad=ui->JoyPad->y();
    x2_joypad=x1_joypad + ui->JoyPad->width();
    y2_joypad=y1_joypad + ui->JoyPad->height();
    x_center_joypad = (x1_joypad + x2_joypad) / 2;
    y_center_joypad = (y1_joypad + y2_joypad) / 2;
    QString altinay_path("/home/"); altinay_path = altinay_path+ USER+ "/src/moobot_ui/src/pictures/altinay.jpeg";

    QPixmap pixmap(altinay_path);
    ui->logo_label->setPixmap(pixmap.scaled(255,60,Qt::KeepAspectRatio));

    QString stop_path("/home/"); stop_path = stop_path+ USER+ "/src/moobot_ui/src/pictures/stop.png";
    ui->stop_button->setIcon(QIcon(stop_path));
    ui->stop_button->setIconSize(QSize(120, 120));

    QString joy_path("/home/"); joy_path = joy_path+ USER+ "/src/moobot_ui/src/pictures/joystick_background.png";
    QPixmap pixmap_joy(joy_path);
    ui->JoyPad->setPixmap(pixmap_joy.scaled(230,230,Qt::KeepAspectRatio));

    QString thumb_path("/home/"); thumb_path = thumb_path+ USER+ "/src/moobot_ui/src/pictures/joystick_thumb.png";
    QPixmap pixmap_thumb(thumb_path);
    ui->thumb->setPixmap(pixmap_thumb.scaled(60,60,Qt::KeepAspectRatio));
    ui->thumb->setGeometry(QRect((x_center_joypad - 30),(y_center_joypad - 30),60,60));

    QString exit_path("/home/"); exit_path = exit_path+ USER+ "/src/moobot_ui/src/pictures/exit.jpg";
    ui->exitButton->setIcon(QIcon(exit_path));
    ui->exitButton->setIconSize(QSize(40, 40));

    QString right_path("/home/"); right_path = right_path+ USER+ "/src/moobot_ui/src/pictures/right_arrow.jpeg";
    QString left_path("/home/"); left_path = left_path+ USER+ "/src/moobot_ui/src/pictures/left_arrow.jpeg";
    ui->page_button_right->setIcon(QIcon(right_path));
    ui->page_button_right->setIconSize(QSize(30, 30));
    ui->page_button_left->setIcon(QIcon(left_path));
    ui->page_button_left->setIconSize(QSize(30, 30));


    ui->stackedWidget->setCurrentIndex(0);

    ShowSpeed::linearSpeed_des=0;
    ShowSpeed::angularSpeed_des=0;
    ShowSpeed::linearSpeed_max = linearSpeedMax;
    ShowSpeed::linearSpeed_min = linearSpeedMin;
    ShowSpeed::angularSpeed_max = angularSpeedMax;
    ShowSpeed::angularSpeed_min = angularSpeedMin;
    ui->showSpeed->ShowSpeedValues();
    ui->showSpeed->updateMaxSpeed();
    ui->showSpeed->updateMinSpeed();



    static ros::NodeHandle nh;

    if (!ros::ok())
    {
        ROS_INFO("No master started!");
        this->close();
    }
    twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    agv_status_pub = nh.advertise<moobot_ui::agv_status>("/agv_status", 1000);



    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(callSpin()));
    timer->start(1000);

}

Joystick::~Joystick()
{
    delete timer;
    delete ui;
}

void Joystick::on_stop_button_pressed()
{

    if(!isStopPressed){ //stopped
        QString continue_path("/home/"); continue_path = continue_path+ USER+ "/src/moobot_ui/src/pictures/continue.png";
        ui->stop_button->setIcon(QIcon(continue_path));
        ui->stop_button->setIconSize(QSize(120, 120));
        ui->mode_label->setText("SERVICE");
        twist_msg.angular.z=0;
        twist_msg.linear.x=0;
        agv_status_msg_joystick.agv_stopped=true;
        agv_status_msg_joystick.agv_mode=2; //service modu
        twist_pub.publish(twist_msg);
        agv_status_pub.publish(agv_status_msg_joystick);
        angularSpeed = linearSpeed = 0;
        ui->angular_val->setText(QString("<span style=\" font-size:18pt; font-weight:600;\">%1</span>").arg(QString::number(angularSpeed,'f',ShowSpeed::precision)));
        ui->linear_val->setText(QString("<span style=\" font-size:18pt; font-weight:600;\">%1</span>").arg(QString::number(linearSpeed,'f',ShowSpeed::precision)));
        ShowSpeed::linearSpeed_des=0;
        ShowSpeed::angularSpeed_des=0;
        ui->showSpeed->ShowSpeedValues();
        isStopPressed = true;
    }else{
        QString stop_path("/home/"); stop_path = stop_path+ USER+ "/src/moobot_ui/src/pictures/stop.png";
        ui->stop_button->setIcon(QIcon(stop_path));

        ui->stop_button->setIconSize(QSize(120, 120));
        agv_status_msg_joystick.agv_stopped=false;
        agv_status_pub.publish(agv_status_msg_joystick);
        isStopPressed = false;

    }

}


void Joystick::on_auto_button_pressed()
{


    ui->mode_label->setText("AUTO");
    agv_status_msg_joystick.agv_mode=0; //auto mode 0
    agv_status_pub.publish(agv_status_msg_joystick);

}

void Joystick::on_manual_button_pressed()
{
    ui->mode_label->setText("MANUAL");
    agv_status_msg_joystick.agv_mode=1; //manual mode 1
    agv_status_pub.publish(agv_status_msg_joystick);

}

void Joystick::mousePressEvent(QMouseEvent *event){
    if(!isStopPressed){
        int x = event->x()-ui->stackedWidget->x();
        int y = event->y()-ui->stackedWidget->y();
        int distance = sqrt(pow((x-x_center_joypad),2)+pow((y-y_center_joypad),2));
        int r= ui->JoyPad->width()/2;


        if(x > x1_joypad && x < x2_joypad && y > y1_joypad && y < y2_joypad && distance < r){

            QWidget::mouseMoveEvent(event);
            ui->thumb->setGeometry(QRect((x - 30),(y-30),60,60));

        }
    }


}

void Joystick::mouseMoveEvent(QMouseEvent *event){
    if(!isStopPressed && ui->stackedWidget->currentIndex() == 0 ){

        int x = event->x()-ui->stackedWidget->x(); // Stacked widget'ın 0 noktası ile daashboard'unki farklı
        int y = event->y()-ui->stackedWidget->y();
        int distance = sqrt(pow((x-x_center_joypad),2)+pow((y-y_center_joypad),2));
        int r= ui->JoyPad->width()/2;


        if(x > x1_joypad && x < x2_joypad && y > y1_joypad && y < y2_joypad && distance < r){

            QWidget::mouseMoveEvent(event);
            ui->thumb->setGeometry((x-30),(y-30),60,60);


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
            twist_pub.publish(twist_msg);

            ShowSpeed::linearSpeed_des=linearSpeed;
            ShowSpeed::angularSpeed_des=angularSpeed;
            ui->showSpeed->ShowSpeedValues();

            ui->linear_val->setText(QString("<span style=\" font-size:18pt; font-weight:600;\">%1</span>").arg(QString::number(linearSpeed,'f',ShowSpeed::precision)));
            ui->angular_val->setText(QString("<span style=\" font-size:18pt; font-weight:600;\">%1</span>").arg(QString::number(angularSpeed,'f',ShowSpeed::precision)));
        }
    }


}
void Joystick::mouseReleaseEvent(QMouseEvent *event){
    int x = event->x()-ui->stackedWidget->x();
    int y = event->y()-ui->stackedWidget->y();
    if(x > (x1_joypad-30) && x < (x2_joypad+30) && y > (y1_joypad - 30) && y < (y2_joypad+30) && ui->stackedWidget->currentIndex() == 0 ){ //fazla sürükleme ihtimaline karşılık

        QWidget::mouseReleaseEvent(event);
        ui->thumb->setGeometry(QRect((x_center_joypad - 30),(y_center_joypad - 30),60,60));

        linearSpeed=0;
        angularSpeed=0;

        twist_msg.angular.z=angularSpeed;
        twist_msg.linear.x=linearSpeed;        twist_pub.publish(twist_msg);

        ShowSpeed::linearSpeed_des=linearSpeed;
        ShowSpeed::angularSpeed_des=angularSpeed;
        ui->showSpeed->ShowSpeedValues();

        ui->linear_val->setText(QString("<span style=\" font-size:18pt; font-weight:600;\">%1</span>").arg(QString::number(linearSpeed,'f',ShowSpeed::precision)));
        ui->angular_val->setText(QString("<span style=\" font-size:18pt; font-weight:600;\">%1</span>").arg(QString::number(angularSpeed,'f',ShowSpeed::precision)));
    }

}



void Joystick::on_page_button_left_clicked()
{
    int newPage=(currentPage - 1) % 4;
    ui->stackedWidget->setCurrentIndex(newPage);
    currentPage--;
}

void Joystick::on_page_button_right_clicked()
{
    int newPage=(currentPage + 1) % 4;
    ui->stackedWidget->setCurrentIndex(newPage);
    currentPage++;
}

void Joystick::on_exitButton_clicked()
{
    linearSpeed=0;
    angularSpeed=0;

    twist_msg.angular.z=angularSpeed;
    twist_msg.linear.x=linearSpeed;
    twist_pub.publish(twist_msg);

    Joystick::close();
}

void Joystick::on_set_speed_button_clicked()
{
    if(!isStopPressed){
        angularSpeed=ui->angular_speed_entered->value();
        linearSpeed = ui->linear_speed_entered->value();

        twist_msg.angular.z=angularSpeed;
        twist_msg.linear.x=linearSpeed;
        twist_pub.publish(twist_msg);

        ShowSpeed::linearSpeed_des=linearSpeed;
        ShowSpeed::angularSpeed_des=angularSpeed;
        ui->showSpeed->ShowSpeedValues();

        ui->linear_val->setText(QString("<span style=\" font-size:18pt; font-weight:600;\">%1</span>").arg(QString::number(linearSpeed,'f',ShowSpeed::precision)));
        ui->angular_val->setText(QString("<span style=\" font-size:18pt; font-weight:600;\">%1</span>").arg(QString::number(angularSpeed,'f',ShowSpeed::precision)));
    }


}

void Joystick::callSpin(){
    diag_time++;
    ui->showSpeed->ShowSpeedValues();
    ros::spinOnce();
}

void Joystick::on_set_speed_button_max_clicked()
{
    double ang_val = ui->max_angular_speed_entered->value();
    if(ang_val != 0.00){
        maxAngularSpeed = ang_val/deg; //deg olarak girdiğini var sayıyorum
        ui->showSpeed->angularSpeed_max = maxAngularSpeed;

    }



    double lin_val = ui->max_linear_speed_entered->value();
    if(lin_val != 0.00){
        maxLinearSpeed = lin_val;
        ui->showSpeed->linearSpeed_max = maxLinearSpeed;
    }
    ui->showSpeed->updateMaxSpeed();


}

void Joystick::on_set_speed_button_min_clicked()
{
    double ang_val = ui->min_angular_speed_entered->value();
    if(ang_val != 0.00){
        minAngularSpeed = ang_val/deg; //deg olarak girdiğini var sayıyorum
        ui->showSpeed->angularSpeed_min = minAngularSpeed;
    }
    double lin_val = ui->min_linear_speed_entered->value();
    if(lin_val != 0.00){
        minLinearSpeed = lin_val;
         ui->showSpeed->linearSpeed_min = minLinearSpeed;
    }
    ui->showSpeed->updateMinSpeed();
}

#include "moobot_hmi.h"
#include "ui_moobot_hmi.h"



moobot_msgs::moobot_status moobot_status_msg;

sensor_msgs::JointState joint_states_msg;

geometry_msgs::Twist twist_msg_d;

actionlib_msgs::GoalID goal_id;

Moobot_hmi::Moobot_hmi(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Moobot_hmi),
    loop_rate(10)
{


    if (!ros::ok())
    {
        ROS_INFO("No master started!");
        this->close();
    }
    ui->setupUi(this);
    initWidgets();


   moobot_status_pub = nh.advertise<moobot_msgs::moobot_status>("/agv_status", 1000);
   cmd_vel_act_sub = nh.subscribe("/cmd_vel_act",1000,&Moobot_hmi::updateActualSpeeds,this);
   twist_pub_d= nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
   cancel_pub = nh.advertise <actionlib_msgs::GoalID>("/move_base/cancel",100);
   moobot_status_dashboard_sub = nh.subscribe("/agv_status",1000,&Moobot_hmi::updateStatus,this);
   bms_sub_d = nh.subscribe("/bms_status",1000,&Moobot_hmi::updateBmsStatus,this);
   agv_job_status_sub = nh.subscribe("/agv_job_status", 1000, &Moobot_hmi::updateAgvJobStatus,this);
   //cmd_vel_auto_sub = nh.subscribe("/cmd_vel",1000,&Dashboard::updateDesiredSpeedsAuto,this);

   QObject::connect(joypad_widget, &Joypad::speedSignal, speed_widget,  &ShowSpeed::on_speedChange);
   QObject::connect(joypad_widget, &Joypad::maxSpeedSignal, speed_widget, &ShowSpeed::on_maxSpeedChange);
   QObject::connect(joypad_widget, &Joypad::minSpeedSignal, speed_widget, &ShowSpeed::on_minSpeedChange);
   QObject::connect(this, &Moobot_hmi::stopSignal, joypad_widget, &Joypad::on_stopButtonPressed);

   timer = new QTimer(this);
   connect(timer, SIGNAL(timeout()), this, SLOT(callSpin()));
   timer->start(100);

}

Moobot_hmi::~Moobot_hmi()
{
  delete right_layout;
  delete joypad_widget;
  delete speed_widget;
  delete right_widget;
  delete dashboard_layout;
  delete ui;
  delete timer;
  delete agv_nav;
  delete message_label;
  delete message_widget;
  delete main_layout;
  delete main_widget;
  delete auto_label;
  delete manual_label;
  delete mode_switch;
  delete altinay_logo;
  delete startStopButton;
  delete upper_layout;
  delete upper_widget;

}

void Moobot_hmi::initWidgets(){

    upper_widget = new QWidget();
    upper_layout = new QHBoxLayout(upper_widget);

    auto_label = new QLabel(upper_widget);
    manual_label = new QLabel(upper_widget);
    auto_label->setText(QString("<span style=\" font-size:16pt; font-weight:600;\">%1</span>").arg("AUTO"));
    manual_label->setText(QString("<span style=\" font-size:16pt; font-weight:600;\">%1</span>").arg("MANUAL"));
    altinay_logo = new QLabel(upper_widget);
    mode_switch = new Switch(upper_widget);
    altinay_logo->setFixedSize(344,90);

    QPixmap pixmap(QString("/home/rnd/moobot_ws/src/moobot_ui/src/pictures/altinay_logo.jpeg"));
    altinay_logo->setPixmap(pixmap.scaled(300,91,Qt::KeepAspectRatio));
    auto_label->setFixedSize(80,30);
    manual_label->setFixedSize(120,30);
    mode_switch->setFixedSize(100,30);

    startStopButton = new QPushButton(upper_widget);
    startStopButton->setFixedSize(80,80);
    startStopButton->setStyleSheet("border: 0px;");

    stop_path = "/home/rnd/moobot_ws/src/moobot_ui/media/stop.png";
    continue_path = "/home/rnd/moobot_ws/src/moobot_ui/media/start.png";
    startStopButton->setIcon(QIcon(stop_path));
    startStopButton->setIconSize(QSize(80,80));

    message_widget = new QWidget();
    //message_widget->setStyleSheet("background-color:black;");
    message_label = new QLabel(message_widget);
    message_label->setFixedSize(900,70);
    message_label->setText(QString("<span style=\" font-size:16pt; font-weight:600;\">%1</span>").arg("LOGOS"));
    message_label->setAlignment(Qt::AlignCenter);



    upper_layout->addWidget(altinay_logo);
    //upper_layout->addStretch();

    upper_layout->addWidget(message_widget);
    //upper_layout->addStretch();
    upper_layout->addWidget(auto_label);
    upper_layout->addWidget(mode_switch);
    upper_layout->addWidget(manual_label);
    upper_layout->addWidget(startStopButton);

    upper_widget->setFixedHeight(95);
    message_widget->setFixedHeight(70);

    right_widget = new QWidget();
    right_layout = new QVBoxLayout(right_widget);
    joypad_widget = new Joypad();
    joypad_widget->setFixedWidth(360);

    speed_widget = new ShowSpeed();
    speed_widget->setFixedSize(321,172);

    battery_widget = new QWidget();
    battery_label_widget = new QWidget();
    battery_widget->setFixedSize(321,35);
    battery_label_widget->setFixedSize(321,35);
    battery_layout = new QHBoxLayout(battery_widget);
    battery_label_layout = new QHBoxLayout(battery_label_widget);

    bms_voltage = new QLabel();
    bms_voltage->setText(QString("<span style=\" font-size:12pt; font-weight:600;\">%1</span>").arg("0.0"));
    bms_voltage->setAlignment(Qt::AlignCenter);

    bms_current = new QLabel();
    bms_current->setText(QString("<span style=\" font-size:12pt; font-weight:600;\">%1</span>").arg("0.0"));
    bms_current->setAlignment(Qt::AlignCenter);

    bms_power = new QLabel();
    bms_power->setText(QString("<span style=\" font-size:12pt; font-weight:600;\">%1</span>").arg("0.0"));
    bms_power->setAlignment(Qt::AlignCenter);

    bms_temperature = new QLabel();
    bms_temperature->setText(QString("<span style=\" font-size:12pt; font-weight:600;\">%1</span>").arg("0.0"));
    bms_temperature->setAlignment(Qt::AlignCenter);

    bms_voltage_label = new QLabel();
    bms_voltage_label->setText(QString("<span style=\" font-size:11pt; font-weight:600;\">%1</span>").arg("Voltage"));
    bms_voltage_label->setAlignment(Qt::AlignCenter);

    bms_current_label = new QLabel();
    bms_current_label->setText(QString("<span style=\" font-size:11pt; font-weight:600;\">%1</span>").arg("Current"));
    bms_current_label->setAlignment(Qt::AlignCenter);

    bms_power_label = new QLabel();
    bms_power_label->setText(QString("<span style=\" font-size:11pt; font-weight:600;\">%1</span>").arg("SoC"));
    bms_power_label->setAlignment(Qt::AlignCenter);

    bms_temperature_label = new QLabel();
    bms_temperature_label->setText(QString("<span style=\" font-size:11pt; font-weight:600;\">%1</span>").arg("Temp"));
    bms_temperature_label->setAlignment(Qt::AlignCenter);

    battery_layout->addWidget(bms_voltage);
    battery_layout->addWidget(bms_current);
    battery_layout->addWidget(bms_power);
    battery_layout->addWidget(bms_temperature);

    battery_label_layout->addWidget(bms_voltage_label);
    battery_label_layout->addWidget(bms_current_label);
    battery_label_layout->addWidget(bms_power_label);
    battery_label_layout->addWidget(bms_temperature_label);

    right_layout->addWidget(joypad_widget);
    right_layout->addWidget(speed_widget);

    right_layout->addWidget(battery_label_widget);
    right_layout->addWidget(battery_widget);



    main_widget = new QWidget();
    main_layout = new QHBoxLayout(main_widget);

    agv_nav = new Agv_nav();
    agv_nav->frame_->loadDisplayConfig(QString::fromStdString("/home/rnd/moobot_ws/src/moobot_navigation/moobot_navigation_hmi.rviz"));
    QObject::connect(this, &Moobot_hmi::stopSignal, agv_nav, &Agv_nav::stopSlot);
    agv_nav->scrollArea->setFixedWidth(0);
    main_layout->addWidget(agv_nav);
    main_layout->addWidget(right_widget);

    dashboard_layout = new QVBoxLayout();
    dashboard_layout->addWidget(upper_widget);
    dashboard_layout->addWidget(main_widget);

    ui->centralWidget->setLayout(dashboard_layout);
    startStopButton->setObjectName("startStopButton");

    // Connect button signal to appropriate slot
    QObject::connect(startStopButton, SIGNAL (pressed()), this, SLOT (on_startStopButton_pressed()));
    QObject::connect(mode_switch, &Switch::modeSignal, this, &Moobot_hmi::changeMode);
    speed_widget->ShowSpeedValues();
    speed_widget->updateMaxSpeed();
    speed_widget->updateMinSpeed();

    //system(" gnome-terminal -e 'sh -c \" rosrun moobot_bringup moobot_diagnostics ; sh\"'");

}
void Moobot_hmi::on_startStopButton_pressed(){ //TODO: loop_rate
  emit stopSignal(!isStopped);
  if(!isStopped){ //Stop the agv
      moobot_status_msg.agv_stopped = true;
      isStopped = true;

      //TODO: buradaki publishler bazen olmuyor bunu çözzzzz!!!!
      startStopButton->setIcon(QIcon(continue_path));
      /*while (moobot_status_pub.getNumSubscribers() == 0)
            loop_rate.sleep();*/
      moobot_status_pub.publish(moobot_status_msg);
      twist_msg_d.angular.z = 0.0;
      twist_msg_d.linear.x = 0.0;
      /*while (twist_pub_d.getNumSubscribers() == 0)
            loop_rate.sleep();*/
      twist_pub_d.publish(twist_msg_d);

      //If goalid is empty cancels all goal
      /*while (cancel_pub.getNumSubscribers() == 0)
            loop_rate.sleep();*/
      cancel_pub.publish(goal_id);
      ROS_INFO("Goal canceled!");
      ShowSpeed::linearSpeed_des = 0.0;
      ShowSpeed::angularSpeed_des = 0.0;
      speed_widget->ShowSpeedValues();

  }else{ //continue
      moobot_status_msg.agv_stopped = false;
      isStopped = false;
      startStopButton->setIcon(QIcon(stop_path));
      moobot_status_pub.publish(moobot_status_msg);
  }

}



void Moobot_hmi::changeMode(bool mode){

    moobot_status_msg.agv_mode = mode;
    moobot_status_pub.publish(moobot_status_msg);

}

void Moobot_hmi::updateActualSpeeds(const geometry_msgs::Twist &msg){
    ShowSpeed::linearSpeed_act = msg.linear.x;
    ShowSpeed::angularSpeed_act = msg.angular.z;
    speed_widget->ShowSpeedValues();
}
void Moobot_hmi::updateStatus(const moobot_msgs::moobot_status &msg){


    moobot_status_msg = msg;

    bool current_mode = msg.agv_mode;
    if(old_mode != current_mode){  //mode değiştiyse
        mode_switch->changeMode(current_mode);
    }
    old_mode = current_mode;

    /*bool current_agv_stopped = msg.agv_stopped;
    if(old_agv_stopped != current_agv_stopped){
        if(current_agv_stopped){
            startStopButton->setIcon(QIcon(continue_path));
            isStopped = true;
        }else{
            startStopButton->setIcon(QIcon(stop_path));
            isStopped = false;
        }
    }
    old_agv_stopped = current_agv_stopped;*/
    

}

void Moobot_hmi::updateDesiredSpeedsAuto(const geometry_msgs::Twist &msg){
    if(moobot_status_msg.agv_mode == AUTO){
        ShowSpeed::linearSpeed_des = msg.linear.x;
        ShowSpeed::angularSpeed_des = msg.angular.z;
        speed_widget->ShowSpeedValues();
    }
}

void Moobot_hmi::updateBmsStatus(const moobot_msgs::bms_status &msg){
//   double temp = (msg.battery_temp_2 + msg.battery_temp_1) / 2;
  double temp = msg.temperature_bms;
  bms_voltage->setText(QString("<span style=\" font-size:12pt; font-weight:600;\">%1</span>").arg(QString::number(msg.battery_voltage, 'f', 2)));
  bms_current->setText(QString("<span style=\" font-size:12pt; font-weight:600;\">%1</span>").arg(QString::number(msg.current_load, 'f', 2)));
  bms_power->setText(QString("<span style=\" font-size:12pt; font-weight:600;\">%1</span>").arg(QString::number(msg.soc, 'f', 2)));
  bms_temperature->setText(QString("<span style=\" font-size:12pt; font-weight:600;\">%1</span>").arg(QString::number(temp, 'f', 2)));
}

void Moobot_hmi::updateAgvJobStatus(const std_msgs::String &msg){
  QString message = "STATUS : " + QString::fromStdString(msg.data);
 // std::cout << msg.data << std::endl;
 // QString message = "bok";
  message_label->setText(QString("<span style=\" font-size:16pt; font-weight:600;\">%1</span>").arg(message));
}
void Moobot_hmi::callSpin(){
    speed_widget->ShowSpeedValues();
    ros::spinOnce();
}




#include "dashboard.h"
#include "ui_dashboard.h"



moobot_msgs::moobot_status moobot_status_msg;

sensor_msgs::JointState joint_states_msg;

geometry_msgs::Twist twist_msg_d;

actionlib_msgs::GoalID goal_id;

Dashboard::Dashboard(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Dashboard),
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
   cmd_vel_act_sub = nh.subscribe("/cmd_vel_act",1000,&Dashboard::updateActualSpeeds,this);
   twist_pub_d= nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
   cancel_pub = nh.advertise <actionlib_msgs::GoalID>("/move_base/cancel",100);
   moobot_status_dashboard_sub = nh.subscribe("/agv_status",1000,&Dashboard::updateStatus,this);
   bms_sub_d = nh.subscribe("/bms_status",1000,&Dashboard::updateBmsStatus,this);
   //cmd_vel_auto_sub = nh.subscribe("/cmd_vel",1000,&Dashboard::updateDesiredSpeedsAuto,this);
    agv_job_status_sub = nh.subscribe("/agv_job_status", 1000, &Dashboard::updateAgvJobStatus,this);
   QObject::connect(joypad_widget, &Joypad::speedSignal, speed_widget,  &ShowSpeed::on_speedChange);
   QObject::connect(joypad_widget, &Joypad::maxSpeedSignal, speed_widget, &ShowSpeed::on_maxSpeedChange);
   QObject::connect(joypad_widget, &Joypad::minSpeedSignal, speed_widget, &ShowSpeed::on_minSpeedChange);
   QObject::connect(this, &Dashboard::stopSignal, joypad_widget, &Joypad::on_stopButtonPressed);

   timer = new QTimer(this);
   connect(timer, SIGNAL(timeout()), this, SLOT(callSpin()));
   timer->start(100);

}

Dashboard::~Dashboard()
{
    delete dashboard_layout;
    delete ui;
    delete timer;
    delete status_widget;
    delete start_mapping_button;
    delete stop_mapping_button;
    delete start_loop_button;
    delete stop_loop_button;
    delete save_map_button;
    delete agv_nav;
    delete map_menu_layout;
    delete map_menu_widget;
    delete map_main_layout;
    delete map_main_widget;
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
    delete speed_widget;
    delete joypad_widget;
    delete right_layout;
    delete right_widget;
    delete lower_layout;
    delete lower_widget;


}

void Dashboard::initWidgets(){

    main_widget = new QWidget();
    main_layout = new QVBoxLayout(main_widget);

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

    status_widget = new StatusPanel();
    status_widget->setFixedSize(500,59);

    startStopButton = new QPushButton(upper_widget);
    startStopButton->setFixedSize(80,80);
    startStopButton->setStyleSheet("border: 0px;");

    stop_path = "/home/rnd/moobot_ws/src/moobot_ui/media/stop.png";
    continue_path = "/home/rnd/moobot_ws/src/moobot_ui/media/start.png";
    startStopButton->setIcon(QIcon(stop_path));
   // startStopButton->setStyleSheet("border-image: url(:/home/rnd/moobot/moobot_ws/src/moobot_ui/media/stop.png);");
    startStopButton->setIconSize(QSize(80,80));

    upper_layout->addWidget(altinay_logo);
    upper_layout->addStretch();

    upper_layout->addWidget(status_widget);
    upper_layout->addStretch();
    upper_layout->addWidget(auto_label);
    upper_layout->addWidget(mode_switch);
    upper_layout->addWidget(manual_label);

    upper_layout->addWidget(startStopButton);

    map_menu_widget = new QWidget();
    map_menu_widget->setFixedHeight(50);

    map_menu_layout = new QHBoxLayout(map_menu_widget);

    start_mapping_button = new QPushButton("Start Mapping",map_menu_widget);
    stop_mapping_button = new QPushButton("Stop Mapping",map_menu_widget);
    start_loop_button = new QPushButton("Start Loop",map_menu_widget);
    stop_loop_button = new QPushButton("Stop Loop",map_menu_widget);
    save_map_button = new QPushButton("Save Map",map_menu_widget);
    run_navigation_button = new QPushButton("Start Navigation", map_menu_widget);
    stop_navigation_button = new QPushButton("Stop Navigation", map_menu_widget);
    start_mapping_button->setFixedSize(120,30);
    stop_mapping_button->setFixedSize(120,30);
    save_map_button->setFixedSize(120,30);
    start_loop_button->setFixedSize(120,30);
    stop_loop_button->setFixedSize(120,30);

    map_menu_layout->addWidget(start_mapping_button);
    map_menu_layout->addStretch();
    map_menu_layout->addWidget(stop_mapping_button);
    map_menu_layout->addStretch();
    map_menu_layout->addWidget(save_map_button);
    map_menu_layout->addStretch();
    map_menu_layout->addWidget(start_loop_button);
    map_menu_layout->addStretch();
    map_menu_layout->addWidget(stop_loop_button);
    map_menu_layout->addStretch();
    map_menu_layout->addWidget(run_navigation_button);
    map_menu_layout->addStretch();
    map_menu_layout->addWidget(stop_navigation_button);


    map_main_widget = new QWidget();
    map_main_layout = new QVBoxLayout(map_main_widget);

    agv_nav = new Agv_nav();
    QObject::connect(this, &Dashboard::stopSignal, agv_nav, &Agv_nav::stopSlot);
    map_main_layout->addWidget(agv_nav);
    map_main_layout->addWidget(map_menu_widget);

    message_widget = new QWidget();
    message_label = new QLabel(message_widget);
    message_label->setFixedSize(1400,70);
    message_label->setText(QString("<span style=\" font-size:16pt; font-weight:600;\">%1</span>").arg("LOGOS"));
    message_label->setAlignment(Qt::AlignCenter);


    upper_widget->setFixedHeight(95);
    message_widget->setFixedHeight(70);

    main_layout->addWidget(map_main_widget);
    main_layout->addWidget(message_widget);

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
    bms_temperature_label->setText(QString("<span style=\" font-size:11pt; font-weight:600;\">%1</span>").arg("Temperature"));
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

    lower_widget = new QWidget();
    lower_layout = new QHBoxLayout(lower_widget);

    lower_layout->addWidget(main_widget);
    lower_layout->addWidget(right_widget);

    dashboard_layout = new QVBoxLayout();
    dashboard_layout->addWidget(upper_widget);
    dashboard_layout->addWidget(lower_widget);

    ui->centralWidget->setLayout(dashboard_layout);
    startStopButton->setObjectName("startStopButton");

    // Connect button signal to appropriate slot
    QObject::connect(startStopButton, SIGNAL (pressed()), this, SLOT (on_startStopButton_pressed()));
    QObject::connect(mode_switch, &Switch::modeSignal, this, &Dashboard::changeMode);
    QObject::connect(save_map_button, SIGNAL (pressed()), this, SLOT (on_save_map_button_pressed()));
    QObject::connect(run_navigation_button, SIGNAL (pressed()), this, SLOT (on_run_navigation_button_pressed()));
    QObject::connect(stop_navigation_button, SIGNAL (pressed()), this, SLOT (on_stop_navigation_button_pressed()));
    QObject::connect(start_mapping_button, SIGNAL (pressed()), this, SLOT (on_start_mapping_button_pressed()));
    QObject::connect(stop_mapping_button, SIGNAL (pressed()), this, SLOT (on_stop_mapping_button_pressed()));
    QObject::connect(start_loop_button, SIGNAL (pressed()), this, SLOT (on_start_loop_button_pressed()));
    QObject::connect(stop_loop_button, SIGNAL (pressed()), this, SLOT (on_stop_loop_button_pressed()));

    speed_widget->ShowSpeedValues();
    speed_widget->updateMaxSpeed();
    speed_widget->updateMinSpeed();

    //system(" gnome-terminal -e 'sh -c \" rosrun moobot_bringup moobot_diagnostics ; sh\"'");

}
void Dashboard::on_startStopButton_pressed(){ //TODO: loop_rate
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

// stops mapping and saves map
void Dashboard::on_save_map_button_pressed(){
    // agv_nav->frame_->saveDisplayConfig(QString::fromStdString("/home/rnd/moobot_ws/src/moobot_navigation/moobot_navigation.rviz"));
    QProcess process;
    process.startDetached("/bin/bash", QStringList()<< "/home/rnd/moobot_ws/src/moobot_ui/src/scripts/saveMap.sh");
}

void Dashboard::on_run_navigation_button_pressed(){
    QProcess process;
    process.startDetached("/bin/bash", QStringList()<< "/home/rnd/moobot_ws/src/moobot_ui/src/scripts/runNavigation.sh");
//   std::string command = "/home/rnd/moobot_ws/src/moobot_ui/src/scripts/runMapping.sh";
//   char buffer[128];
//   std::string result = "";

//   // Create pipe and write command output
//   FILE* pipe = popen(command.c_str(), "r");

//   // read pipe
//   while (!feof(pipe)) {

//   // use buffer to read and add to result
//   if (fgets(buffer, 128, pipe) != NULL)
//      result += buffer;
//   }
//   // close pipe
//   pclose(pipe);
//   ROS_INFO("%s\n", result.c_str());
//     //system("gnome-terminal -x sh -c ''");
}
void Dashboard::on_stop_navigation_button_pressed(){
    QProcess process;
    process.startDetached("/bin/bash", QStringList()<< "/home/rnd/moobot_ws/src/moobot_ui/src/scripts/stopNavigation.sh");
}
void Dashboard::on_start_mapping_button_pressed(){
    // system("gnome-terminal -x sh -c 'source /home/rnd/moobot_ws/src/moobot_ui/src/scripts/runMapping.sh' ");
    QProcess process;
    process.startDetached("/bin/bash", QStringList()<< "/home/rnd/moobot_ws/src/moobot_ui/src/scripts/runMapping.sh");
}
void Dashboard::on_stop_mapping_button_pressed(){
    // system("gnome-terminal -x sh -c 'source /home/rnd/moobot_ws/src/moobot_ui/src/scripts/runMapping.sh' ");
    QProcess process;
    process.startDetached("/bin/bash", QStringList()<< "/home/rnd/moobot_ws/src/moobot_ui/src/scripts/stopMapping.sh");
}
void Dashboard::on_start_loop_button_pressed(){
    // system("gnome-terminal -x sh -c 'source /home/rnd/moobot_ws/src/moobot_ui/src/scripts/runMapping.sh' ");
    QProcess process;
    process.startDetached("/bin/bash", QStringList()<< "/home/rnd/moobot_ws/src/moobot_ui/src/scripts/startLoop.sh");
}
void Dashboard::on_stop_loop_button_pressed(){
    // system("gnome-terminal -x sh -c 'source /home/rnd/moobot_ws/src/moobot_ui/src/scripts/runMapping.sh' ");
    QProcess process;
    process.startDetached("/bin/bash", QStringList()<< "/home/rnd/moobot_ws/src/moobot_ui/src/scripts/stopLoop.sh");
}

void Dashboard::changeMode(bool mode){

    moobot_status_msg.agv_mode = mode;
    moobot_status_pub.publish(moobot_status_msg);

}

void Dashboard::updateActualSpeeds(const geometry_msgs::Twist &msg){
    ShowSpeed::linearSpeed_act = msg.linear.x;
    ShowSpeed::angularSpeed_act = msg.angular.z;
    speed_widget->ShowSpeedValues();
}
void Dashboard::updateStatus(const moobot_msgs::moobot_status &msg){


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

void Dashboard::updateDesiredSpeedsAuto(const geometry_msgs::Twist &msg){
    if(moobot_status_msg.agv_mode == AUTO){
        ShowSpeed::linearSpeed_des = msg.linear.x;
        ShowSpeed::angularSpeed_des = msg.angular.z;
        speed_widget->ShowSpeedValues();
    }
}

void Dashboard::updateBmsStatus(const moobot_msgs::bms_status &msg){
//   double temp = (msg.battery_temp_2 + msg.battery_temp_1) / 2;
  double temp = msg.temperature_bms;
  bms_voltage->setText(QString("<span style=\" font-size:12pt; font-weight:600;\">%1</span>").arg(QString::number(msg.battery_voltage, 'f', 2))+ QString("<span style=\" font-size:12pt; font-weight:600;\">%1</span>").arg(" V"));
  bms_current->setText(QString("<span style=\" font-size:12pt; font-weight:600;\">%1</span>").arg(QString::number(msg.current_load, 'f', 2))+ QString("<span style=\" font-size:12pt; font-weight:600;\">%1</span>").arg(" A"));
  bms_power->setText(QString("<span style=\" font-size:12pt; font-weight:600;\">%1</span>").arg(QString::number(msg.soc, 'f', 2))+ QString("<span style=\" font-size:12pt; font-weight:600;\">%1</span>").arg(" %"));
  bms_temperature->setText(QString("<span style=\" font-size:12pt; font-weight:600;\">%1</span>").arg(QString::number(temp, 'f', 2))+ QString("<span style=\" font-size:12pt; font-weight:600;\">%1</span>").arg(" °C"));
}

void Dashboard::updateAgvJobStatus(const std_msgs::String &msg){
  QString message = "STATUS : " + QString::fromStdString(msg.data);
 // std::cout << msg.data << std::endl;
 // QString message = "bok";
  message_label->setText(QString("<span style=\" font-size:16pt; font-weight:600;\">%1</span>").arg(message));
}
void Dashboard::callSpin(){
    speed_widget->ShowSpeedValues();
    ros::spinOnce();
}




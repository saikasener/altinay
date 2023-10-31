#include "statuspanel.h"
#include "ui_statuspanel.h"

#define USER "rnd/moobot_ws"
StatusPanel::StatusPanel(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::StatusPanel)
{
    ui->setupUi(this);


    int argc = 0; char **argv = NULL;
    ros::init(argc, argv, "diagnostic_panel");
    static ros::NodeHandle nh;
    if (!ros::ok())
    {
        ROS_INFO("No master started!");
        this->close();
    }
    diagnostic_sub =nh.subscribe("/diagnostics", 1000,&StatusPanel::showDiagnostics,this);
    

    QString ok_path("/home/");
    QString error_path("/home/");
    QString warning_path("/home/");
    ok_path = ok_path + USER + "/src/moobot_ui/src/icons/ok.png";
    error_path = error_path + USER + "/src/moobot_ui/src/icons/error.png";
    warning_path = warning_path + USER + "/src/moobot_ui/src/icons/warning.png";
    ok_pixmap = QPixmap(ok_path);
    error_pixmap = QPixmap(error_path);
    warning_pixmap = QPixmap(warning_path);
    ui->imu_status->setPixmap(warning_pixmap.scaled(35,35,Qt::KeepAspectRatio));
    ui->map_status->setPixmap(ok_pixmap.scaled(35,35,Qt::KeepAspectRatio));
    ui->scanner_status->setPixmap(warning_pixmap.scaled(35,35,Qt::KeepAspectRatio));
    ui->led_status->setPixmap(warning_pixmap.scaled(35,35,Qt::KeepAspectRatio));

}

StatusPanel::~StatusPanel()
{
    delete ui;
}

void StatusPanel::showDiagnostics(const diagnostic_msgs::DiagnosticArray diagnostic_msg){
    for(int i = 0; i < diagnostic_msg.status.size(); i++){
        if(diagnostic_msg.status.at(i).hardware_id == "BNO055"){
            if(diagnostic_msg.status.at(i).level == diagnostic_msgs::DiagnosticStatus::OK ){
                ui->imu_status->setPixmap(ok_pixmap.scaled(35,35,Qt::KeepAspectRatio));
            }else{
                ui->imu_status->setPixmap(error_pixmap.scaled(35,35,Qt::KeepAspectRatio));
            }

        }else if(diagnostic_msg.status.at(i).hardware_id == "SICK_Scanner"){
            if(diagnostic_msg.status.at(i).level == diagnostic_msgs::DiagnosticStatus::OK ){
                ui->scanner_status->setPixmap(ok_pixmap.scaled(35,35,Qt::KeepAspectRatio));
            }else{
                ui->scanner_status->setPixmap(error_pixmap.scaled(35,35,Qt::KeepAspectRatio));
            }
        }
    }

}

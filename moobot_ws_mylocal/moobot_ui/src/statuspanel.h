#ifndef STATUSPANEL_H
#define STATUSPANEL_H

#include <QWidget>
#include <QLabel>
#include <QFormLayout>
#include "ros/ros.h"
#include <diagnostic_msgs/DiagnosticArray.h>

namespace Ui {
class StatusPanel;
}

class StatusPanel : public QWidget
{
    Q_OBJECT

public:
    explicit StatusPanel(QWidget *parent = 0);
    ~StatusPanel();
    void showDiagnostics(const diagnostic_msgs::DiagnosticArray);



private:
    Ui::StatusPanel *ui;
    QPixmap warning_pixmap;
    QPixmap ok_pixmap;
    QPixmap error_pixmap;
    ros::Subscriber diagnostic_sub;
};

#endif // STATUSPANEL_H

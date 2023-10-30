#include "joystick.h"
#include <QApplication>

#include <QHBoxLayout>

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    ros::init(argc, argv, "joystick");
    Joystick w;


    w.show();

    return a.exec();
   }

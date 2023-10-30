#include "moobot_hmi.h"
#include <QApplication>

#include <QHBoxLayout>
#include <inttypes.h>
#include <signal.h>
#include <unistd.h>

void signalhandler(int sig)
{
    if (sig == SIGINT)
    {
        printf("will quit by SIGINT\n");

        ros::shutdown();
        qApp->quit();
    }
}

int main(int argc, char *argv[])
{

    signal(SIGINT, signalhandler);
    ros::init(argc, argv, "moobot_hmi", ros::init_options::NoSigintHandler);

    if (ros::master::check())
    {
        QApplication a(argc, argv);
        Moobot_hmi w;
        w.show();
        return a.exec();
    }
    else
    {
        ROS_INFO("No ros master initialized");
        return 1;
    }
}

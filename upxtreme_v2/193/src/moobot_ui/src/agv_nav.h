#ifndef AGV_NAV_H
#define AGV_NAV_H

#include <QWidget>
#include <QVBoxLayout>
#include <QSplitter>
#include <QScrollArea>
#include "ros/ros.h"
#include "rviz/tool.h"
#include "rviz/tool_manager.h"
#include "rviz/tool_properties_panel.h"
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/displays_panel.h"
#include "rviz/visualization_frame.h"
#include "station.h"
#include "way_point.h"

#include "../../devel/include/moobot_ui/station.h"
#include "../../devel/include/moobot_ui/waypoint.h"
#include "actionlib_msgs/GoalID.h"
#include "actionlib/server/server_goal_handle.h"


namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}
namespace Ui {
class Agv_nav;
}

class Agv_nav : public QWidget
{
    Q_OBJECT

public:
    explicit Agv_nav(QWidget *parent = 0);
    ~Agv_nav();
    void initFrame();
    void initLayouts();
    void updateStations(const moobot_ui::station);
    void updateWayPoints(const moobot_ui::waypoint);
    void updateGoal(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
    QHBoxLayout* main_layout;
    QVBoxLayout* station_layout;
    QSplitter* main_splitter;
    QWidget* station_widget;
    QFrame* inner;
    moobot_ui::StationTool *st;
    std::vector<Station*> stations;
    std::vector<WayPoint*> way_points;
    std::vector<WayPoint*> way_points_on_road;
    ros::Subscriber station_sub;
    ros::Subscriber goal_reached_sub;
    ros::Subscriber way_point_sub;
    bool goWithPoint = false;
    int way_point_num = 0;
    int current_way_point_num = 0;
    int index = 0;
    WayPoint* current_station;
    int current_station_num = 0;
    bool isGoalSet = false;
    bool isStopped = false;
    rviz::VisualizationFrame* frame_;
    QScrollArea* scrollArea;


signals:
    void goWithPointErrorSignal(QString error, int station_num);
    void goWithPointIsDone(bool done, int station_num);
public slots:
    void removeStation(int arg);
    void relocateStation(int arg);
    void goWithPointSlot(std::string arg, float x, float y, float z, int station_num);
    //void addStationToGoal(float x, float y, float z);
    void stopSlot(bool isStopped);


private:


    Ui::Agv_nav *ui;


    int stationToolNum;
    int old_status;
};

#endif // AGV_NAV_H

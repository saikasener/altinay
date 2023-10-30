#include "agv_nav.h"
#include <math.h>
class Station;

Agv_nav::Agv_nav(QWidget *parent) :
    QWidget(parent)

{
    ros::NodeHandle nh;
    if (!ros::ok())
    {
        ROS_INFO("No master started!");
        this->close();
    }


    station_sub =nh.subscribe("/station", 1000,&Agv_nav::updateStations,this);
    goal_reached_sub = nh.subscribe("/move_base/status",1000,&Agv_nav::updateGoal,this);
    way_point_sub = nh.subscribe("/waypoint", 1000 , &Agv_nav::updateWayPoints,this);


    initFrame();
    initLayouts();

}

Agv_nav::~Agv_nav()
{


    delete station_layout;
    delete station_widget;
    delete scrollArea; //şurada scroll area daki stationları silebiliriz
    delete frame_;
    delete main_layout;
    delete main_splitter;




}

void Agv_nav::updateStations(const moobot_ui::station station_msgs){

    bool isRelocate = false;
    for(int i=0 ; i < stations.size() ; i++){  //eklenmek istenen station daha önce eklendi mi diye bakıyoruz
        if(stations.at(i)->station_num == station_msgs.station_number){
            stations.at(i)->update_station(station_msgs.x,station_msgs.y,station_msgs.z); //eğer varsa update edilecek
            //TODO: frame_->getManager()->getToolManager()->getTool(stationToolNum)->getPropertyContainer()->childAt()
            isRelocate = true;
            break;
        }
    }

    if(isRelocate == false){   //Add new station
        Station * new_station = new Station();
        new_station->setFixedSize(185,270);  //WARNING: Change this when you changed station.ui
        stations.push_back(new_station);
        new_station->make_station(station_msgs.station_number,station_msgs.x,station_msgs.y,station_msgs.z);
        station_layout->addWidget(new_station);

        // Now remove signal runs removeStation() and relocate signal runs relocateStation()
        QObject::connect(new_station, &Station::removeSignal, this, &Agv_nav::removeStation);
        QObject::connect(new_station, &Station::relocateSignal, this, &Agv_nav::relocateStation);
        QObject::connect(dynamic_cast <moobot_ui::StationTool*> (frame_->getManager()->getToolManager()->getTool(stationToolNum)), &moobot_ui::StationTool::isRelocateDone, new_station, &Station::enableRemove);
        QObject::connect(new_station,&Station::pointSignal,this,&Agv_nav::goWithPointSlot);
        QObject::connect(this,&Agv_nav::goWithPointErrorSignal,new_station,&Station::goWithPointErrorSlot);
        QObject::connect(this,&Agv_nav::goWithPointIsDone,new_station,&Station::goWithPointIsDoneSlot);

    }


}
void Agv_nav::updateWayPoints(const moobot_ui::waypoint way_point_msg){

    WayPoint* new_way_point = new WayPoint(way_point_msg.x, way_point_msg.y, way_point_msg.z);
    new_way_point->name = 'a' + way_point_num;
    way_points.push_back(new_way_point);
    way_point_num ++;

}


void Agv_nav::removeStation(int arg){

    dynamic_cast <moobot_ui::StationTool*> (frame_->getManager()->getToolManager()->getTool(stationToolNum))->removeStation(arg);
    QLayoutItem * item ;
    for(int i = 0 ; i < station_layout->count(); i++){
        if(dynamic_cast <Station*> (station_layout->itemAt(i)->widget())->station_num == (arg)){
            if ((item = station_layout->takeAt(i)) != 0) {
                delete item->widget();
                delete item;
                break;
           }
       }
    }
    // Erase from vector when the station is removed.
    for(int i=0 ; i <stations.size() ; i++){
        if(stations.at(i)->station_num == arg){
            stations.erase(stations.begin()+i);
            break;
        }
    }


}
void Agv_nav::relocateStation(int arg){

    dynamic_cast <moobot_ui::StationTool*> (frame_->getManager()->getToolManager()->getTool(stationToolNum))->relocateStation(arg);
}

void Agv_nav::goWithPointSlot(std::string arg, float x, float y, float z, int station_num){
    current_station_num = station_num;
    goWithPoint = true;
    int len = arg.length();
    if(way_point_num > 0){
        if(len > 2){
            for (int i = 2; i < len ; i=i+2){

                if(arg.at(i) >= 'a' && arg.at(i) <= (way_point_num + 'a') ){
                    way_points_on_road.push_back(way_points.at(arg.at(i) - 'a'));
                    current_way_point_num++;
                }else{
                    goWithPoint = false;
                    emit goWithPointErrorSignal(QString("No way 1!"),current_station_num);
                    break;
                }


            }
        }else if(len >= 1){
            if(!(arg.at(0) >= 'a' && arg.at(0) <= (way_point_num + 'a'))){
                goWithPoint = false;

                emit goWithPointErrorSignal(QString("No way!"),current_station_num);

            }
        }else{
            goWithPoint = false;
            emit goWithPointErrorSignal(QString("Empty"),current_station_num);
        }
    }else{
        goWithPoint = false;
        emit goWithPointErrorSignal(QString("Add way point!"),current_station_num);
    }

    if(goWithPoint){
        emit goWithPointIsDone(false,current_station_num);
        current_station = new WayPoint(x, y, z);
        way_points_on_road.push_back(current_station);
        current_way_point_num++;
        moobot_ui::WayPointTool * way_point_tool = new moobot_ui::WayPointTool();
        float theta = 0.0;  //TODO : kendi odometry bilgini al ve ona göre ilerle
        float x0 = way_points.at(arg.at(0) - 'a')->x;
        float y0 = way_points.at(arg.at(0) - 'a')->y;
        float x1 = way_points_on_road.at(0)->x;
        float y1 = way_points_on_road.at(0)->y;

        theta = atan((y1 - y0) / (x1 - x0));  //For rn gets the next orientation
        if(x1 < x0){
           theta = M_PI + theta;

        }

        way_point_tool->onPoseSet(x0, y0,theta);  //ilk noktayı veriyorum
        delete way_point_tool;

    }




}

void Agv_nav::initFrame(){
    frame_ = new rviz::VisualizationFrame();
    frame_->initialize(QString::fromStdString("/home/rnd/moobot_ws/src/moobot_navigation/moobot_navigation.rviz"));

    stationToolNum = frame_->getManager()->getToolManager()->numTools() - 2;
}
void Agv_nav::initLayouts(){
    main_layout = new QHBoxLayout;
    station_layout = new QVBoxLayout();
    main_splitter = new QSplitter;
    station_widget = new QWidget();
    main_splitter->addWidget(frame_);

    station_widget->setLayout(station_layout);


    scrollArea = new QScrollArea;
    //scrollArea->setFixedWidth(200);
    scrollArea->setHorizontalScrollBarPolicy( Qt::ScrollBarAlwaysOff );
    scrollArea->setVerticalScrollBarPolicy( Qt::ScrollBarAlwaysOn );
    scrollArea->setWidgetResizable( true  );

    scrollArea->takeWidget();
    scrollArea->setWidget(station_widget);
    scrollArea->ensureWidgetVisible(station_widget);


    main_splitter->addWidget(scrollArea);
    main_layout->addWidget(main_splitter);
    setLayout( main_layout );

}

void Agv_nav::updateGoal(const actionlib_msgs::GoalStatusArray::ConstPtr &msg){

    if (!msg->status_list.empty()) {
        int len  = msg->status_list.size();
        int goal_num = 0;
        if(len > 1){
            for(int i = 0 ; i < len ; i++){
                if(msg->status_list[i].status == 1){
                    goal_num = i;
                    break;
                }
            }

        }
        if( goWithPoint && way_points_on_road.size() != 0 && msg->status_list[goal_num].status == 3 ){
            if(msg->status_list[goal_num].status != old_status ){
                if(index == (current_way_point_num)){
                    goWithPoint = false;
                    index = 0;
                    current_way_point_num = 0;
                    way_points_on_road.clear();
                    ROS_INFO("Went to goal");
                    emit goWithPointIsDone(true,current_station_num);
                    current_station_num = 0 ;
                    //send a signal to clear box of way point num
                }
                if(index < current_way_point_num){  // Eğer hala gidilecek nokta varsa
                    moobot_ui::WayPointTool * way_point_tool = new moobot_ui::WayPointTool();
                    float x0 = way_points_on_road.at(index)->x;
                    float y0 = way_points_on_road.at(index)->y;
                    float theta = stations.at(current_station_num-1)->orientation;

                    if ((current_way_point_num - index) > 1){
                        float x1 = way_points_on_road.at(index+1)->x;
                        float y1 = way_points_on_road.at(index+1)->y;
                        theta = atan((y1 - y0) / (x1 - x0));
                        if(x1 < x0){
                           theta = M_PI + theta;

                        }
                    }

                    way_point_tool->onPoseSet(x0, y0,theta);

                    ROS_INFO("Go goal (way point) %.3f, %.3f ",way_points_on_road.at(index)->x,way_points_on_road.at(index)->y);
                    index ++;
                    delete way_point_tool;

                }




            }




         }
        old_status = msg->status_list[goal_num].status;

    }
}
void Agv_nav::stopSlot(bool isStopped){
    if(isStopped){
        this->isStopped = isStopped;
        emit goWithPointIsDone(true,0);
    }
}


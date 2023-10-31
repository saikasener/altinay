#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"


#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include "../../devel/include/moobot_ui/station.h"
#include "station_tool.h"
#include "station.h"


moobot_ui::station station_msgs;
namespace moobot_ui{

StationTool::StationTool()  : moving_station_node_( NULL )
  , current_station_property_( NULL )
{
  shortcut_key_ = 'a';
  try
  {
    station_pub_ = nh_.advertise<moobot_ui::station>("/station",1000);
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("StationTool", e.what());
  }

  isRelocate = false;

}
StationTool::~StationTool(){
  for( unsigned i = 0; i < station_nodes_.size(); i++ )
  {
    scene_manager_->destroySceneNode( station_nodes_[ i ].station_node);
  }
}

void StationTool::onInitialize()
{
    std::string s = "package://moobot_ui/media/";
    std::string ss = ".obj";
    station_resource_ = "package://moobot_ui/media/altinay_station.obj";

  if( rviz::loadMeshFromResource( station_resource_ ).isNull() )
  {
    ROS_ERROR( "Station Tool: failed to load model resource '%s'.", station_resource_.c_str() );
    return;
  }

  moving_station_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( station_resource_ );
  moving_station_node_->attachObject( entity );
  moving_station_node_->setVisible( false );

}


void StationTool::activate()
{
  if( moving_station_node_ )
  {
    moving_station_node_->setVisible( true );
    if(!isRelocate){
        int station_num = 0;
        if(removedStations.size() == 0){
            station_num = station_nodes_.size()+1;
        }else{
            station_num = removedStations.at(0);

        }
        current_station_property_ = new rviz::VectorProperty( "Station " + QString::number( station_num ));
        current_station_property_->setReadOnly( false );
        getPropertyContainer()->addChild( current_station_property_ );
    }

  }
}

void StationTool::deactivate()
{
  if( moving_station_node_ )
  {
    moving_station_node_->setVisible( false );
    if(!isRelocate){
        delete current_station_property_;
        current_station_property_ = NULL;
    }
  }
}


int StationTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
   if( !moving_station_node_ ) //&& !isRelocate
  {
    return Render;
  }

  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
  if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                         ground_plane,
                                         event.x, event.y, intersection ))
  {
    moving_station_node_->setVisible( true );
    moving_station_node_->setPosition( intersection );
    if(!isRelocate){
        current_station_property_->setVector( intersection );
    }else{
        dynamic_cast <rviz::VectorProperty*> (getPropertyContainer()->childAt(relocate_index-1))->setVector(intersection); //TODO: burada arama yapmak lazım
    }
    if( event.leftDown() )
    {
      makeStation( intersection );
      if(!isRelocate){
          current_station_property_ = NULL; // Drop the reference so that deactivate() won't remove it.
      }

      return Render | Finished;
    }
  }
  else
  {
    moving_station_node_->setVisible( false ); // If the mouse is not pointing at the ground plane, don't show the station.
  }
  return Render;
}

void StationTool::makeStation( const Ogre::Vector3& position )
{
    if(!isRelocate){
        std::string s = "package://moobot_ui/media/";
        std::string ss = ".obj";
        int num = 0;
        if(removedStations.size() == 0){
            station_resource_ = s + std::to_string(station_nodes_.size()+1) + ss;

            num = station_nodes_.size()+1;
            ROS_INFO("Station nodes size %d", num);
        }else{
            station_resource_ = s + std::to_string(removedStations.at(0)) + ss;
            num = removedStations.at(0);
            removedStations.erase(removedStations.begin());

        }

        if( rviz::loadMeshFromResource( station_resource_ ).isNull() )
        {
          ROS_ERROR( "Station Tool: failed to load model resource '%s'.", station_resource_.c_str() );
          return;
        }
        Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
        Ogre::Entity* entity = scene_manager_->createEntity( station_resource_);


        node->attachObject( entity );
        node->setVisible( true );
        node->setPosition( position );
        stations station = {node,num};
        station_nodes_.push_back( station );

        station_msgs.station_number = num;
    }else{

        station_nodes_.at(relocate_index-1).station_node->setPosition(position);
        isRelocate= false;
        station_msgs.station_number = relocate_index;
        emit isRelocateDone(true);


    }
    station_msgs.x = position.x;
    station_msgs.y = position.y;
    station_msgs.z = position.z;
    station_pub_.publish(station_msgs);


}

void StationTool::save( rviz::Config config ) const
{
  config.mapSetValue( "Class", getClassId() );

  rviz::Config stations_config = config.mapMakeChild( "Stations" );

  rviz::Property* container = getPropertyContainer();
  int num_children = container->numChildren();
  for( int i = 0; i < num_children; i++ )
  {
    rviz::Property* position_prop = container->childAt( i );
    
    rviz::Config station_config = stations_config.listAppendNew();
    // Into the station's config we store its name:
    station_config.mapSetValue( "Name", position_prop->getName() );
    // ... and its position.
    position_prop->save( station_config );
  }
}

void StationTool::load( const rviz::Config& config )
{
  
  rviz::Config stations_config = config.mapGetChild( "Stations" );
  int num_stations = stations_config.listLength();
  for( int i = 0; i < num_stations; i++ )
  {
    rviz::Config station_config = stations_config.listChildAt( i );
   
    QString name = "Station " + QString::number( i + 1 );
    
    station_config.mapGetString( "Name", &name );
    rviz::VectorProperty* prop = new rviz::VectorProperty( name );

    prop->load( station_config );
   
    prop->setReadOnly( true );
    getPropertyContainer()->addChild( prop );
    makeStation( prop->getVector() );
  }
}

void StationTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame ="map"; //= context_->getFixedFrame().toStdString();
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Stamped<tf::Pose> p =
      tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
  geometry_msgs::PoseStamped goal;
  tf::poseStampedTFToMsg(p, goal);
  ROS_INFO("Setting station on: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = "
           "Angle: %.3f\n",
           fixed_frame.c_str(), goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
           goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z,
           goal.pose.orientation.w, theta);
  pub_.publish(goal);
}


 void StationTool::removeStation(int station_num){
     removedStations.push_back(station_num);
     std::sort(removedStations.begin() , removedStations.end());
     for(int i = 0; i < station_nodes_.size() ; i ++){
        if(station_nodes_[i].station_num == station_num ){
            station_nodes_[i].station_node->setVisible( false );
            station_nodes_.erase(station_nodes_.begin()+i);
            delete getPropertyContainer()->childAt(i);
            break;
        }
     }
     

 }


 void StationTool::relocateStation(int station_num){

     isRelocate = true;
     relocate_index = station_num;



 }

}//END NAMESPACE moobot_ui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moobot_ui::StationTool,rviz::Tool )



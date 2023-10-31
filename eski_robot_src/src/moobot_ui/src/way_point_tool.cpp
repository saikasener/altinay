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
#include "../../devel/include/moobot_ui/waypoint.h"
#include "way_point_tool.h"

moobot_ui::waypoint way_point_msg;


namespace moobot_ui{

WayPointTool::WayPointTool()  : moving_way_point_node_( NULL )
  , current_way_point_property_( NULL ) //, ac("move_base", true)
{
  shortcut_key_ = 'w';
  try
  {
    way_point_pub = nh_.advertise<moobot_ui::waypoint>("/waypoint",1000);
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("WayPointTool", e.what());
  }


}
WayPointTool::~WayPointTool(){
  for( unsigned i = 0; i < way_point_nodes_.size(); i++ )
  {
    scene_manager_->destroySceneNode( way_point_nodes_[ i ]);
  }
}

void WayPointTool::onInitialize()
{
  way_point_resource_ = "package://moobot_ui/media/wp1.obj";

  if( rviz::loadMeshFromResource( way_point_resource_ ).isNull() )
  {
    ROS_ERROR( "WayPointTool: failed to load model resource '%s'.", way_point_resource_.c_str() );
    return;
  }

  moving_way_point_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( way_point_resource_ );
  moving_way_point_node_->attachObject( entity );
  moving_way_point_node_->setVisible( false );

}


void WayPointTool::activate()
{
  if( moving_way_point_node_ )
  {
    moving_way_point_node_->setVisible( true );

    current_way_point_property_ = new rviz::VectorProperty( "WayPoint " + QString::number( way_point_nodes_.size() +1 ));
    current_way_point_property_->setReadOnly( false );
    getPropertyContainer()->addChild( current_way_point_property_ );


  }
}

void WayPointTool::deactivate()
{
  if( moving_way_point_node_ )
  {
    moving_way_point_node_->setVisible( false );

    delete current_way_point_property_;
    current_way_point_property_ = NULL;

  }
}


int WayPointTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
   if( !moving_way_point_node_ )
  {
    return Render;
  }

  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
  if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                         ground_plane,
                                         event.x, event.y, intersection ))
  {
    moving_way_point_node_->setVisible( true );
    moving_way_point_node_->setPosition( intersection );

    current_way_point_property_->setVector( intersection );

    if( event.leftDown() )
    {
      makeStation( intersection );

      current_way_point_property_ = NULL; // Drop the reference so that deactivate() won't remove it.


      return Render | Finished;
    }
  }
  else
  {
    moving_way_point_node_->setVisible( false ); // If the mouse is not pointing at the ground plane, don't show the station.
  }
  return Render;
}

void WayPointTool::makeStation( const Ogre::Vector3& position )
{
    std::string s = "package://moobot_ui/media/wp";
    std::string ss = ".obj";


    way_point_resource_ = s + std::to_string(way_point_nodes_.size()+1) + ss;

    if( rviz::loadMeshFromResource( way_point_resource_ ).isNull() )
    {
      ROS_ERROR( "Way Point Tool: failed to load model resource '%s'.", way_point_resource_.c_str() );
      return;
    }

    Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity = scene_manager_->createEntity( way_point_resource_ );
    node->attachObject( entity );
    node->setVisible( true );
    node->setPosition( position );
    way_point_nodes_.push_back( node );

    way_point_msg.x = position.x;
    way_point_msg.y = position.y;
    way_point_msg.z = position.z;
    way_point_pub.publish(way_point_msg);


}

void WayPointTool::save( rviz::Config config ) const
{
  config.mapSetValue( "Class", getClassId() );

  rviz::Config stations_config = config.mapMakeChild( "WayPoints" );

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

void WayPointTool::load( const rviz::Config& config )
{
  
  rviz::Config stations_config = config.mapGetChild( "WayPoints" );
  int num_stations = stations_config.listLength();
  for( int i = 0; i < num_stations; i++ )
  {
    rviz::Config station_config = stations_config.listChildAt( i );
   
    QString name = "WayPoint " + QString::number( i + 1 );
    
    station_config.mapGetString( "Name", &name );
    rviz::VectorProperty* prop = new rviz::VectorProperty( name );

    prop->load( station_config );
   
    prop->setReadOnly( true );
    getPropertyContainer()->addChild( prop );
    makeStation( prop->getVector() );
  }
}

void WayPointTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame ="map"; //= context_->getFixedFrame().toStdString();
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Stamped<tf::Pose> p =
      tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
  geometry_msgs::PoseStamped goal;
  tf::poseStampedTFToMsg(p, goal);
  ROS_INFO("Setting way point goal on: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = "
           "Angle: %.3f\n",
           fixed_frame.c_str(), goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
           goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z,
           goal.pose.orientation.w, theta);
  pub_.publish(goal);

}




}//END NAMESPACE moobot_ui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moobot_ui::WayPointTool,rviz::Tool )



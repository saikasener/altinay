#ifndef WAY_POINT_TOOL_H
#define WAY_POINT_TOOL_H

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>
#include <OgreVector3.h>
#include "rviz/default_plugin/tools/pose_tool.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include <actionlib/client/simple_action_client.h>
//#include <move_base_msgs/MoveBaseAction.h>
//#include <move_base_msgs/MoveBaseActionFeedback.h>
#endif

namespace Ogre
{
class SceneNode;
class Vector3;
}

namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
class DisplayContext;
class StringProperty;
}


//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
namespace moobot_ui
{

class WayPointTool : public rviz::Tool
{
  Q_OBJECT
public:
  WayPointTool();
  ~WayPointTool();
  virtual void onInitialize() override;
  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  void onPoseSet(double x, double y, double theta);
  //void doneCb(const actionlib::SimpleClientGoalState& state);

  std::vector<Ogre::SceneNode*> way_point_nodes_;

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
 // MoveBaseClient ac;


  rviz::StringProperty* topic_property_;
  void makeStation( const Ogre::Vector3& position );


  Ogre::SceneNode* moving_way_point_node_;
  std::string way_point_resource_;
  rviz::VectorProperty* current_way_point_property_;
  ros::Publisher way_point_pub;


  //void updateGoal(const move_base_msgs::MoveBaseActionResult);


};

} // namespace moobot_ui

#endif

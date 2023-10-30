#ifndef STATION_TOOL_H
#define STATION_TOOL_H

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <bits/stdc++.h>
#include <algorithm>
#include <ros/ros.h>
#include <OgreVector3.h>
#include "rviz/default_plugin/tools/pose_tool.h"
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

namespace moobot_ui
{

class StationTool : public rviz::Tool
{
  Q_OBJECT
public:
  StationTool();
  ~StationTool();
  virtual void onInitialize() override;
  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  void onPoseSet(double x, double y, double theta);
  void removeStation(int station_num);
  void relocateStation(int station_num);
  struct stations{
      Ogre::SceneNode* station_node;
      int station_num ;
      bool operator < (const stations &iData) const
        {
          return station_num < iData.station_num;
        }
  };

  std::vector<stations> station_nodes_;
  std::vector<int> removedStations;


private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;


  rviz::StringProperty* topic_property_;
  void makeStation( const Ogre::Vector3& position );


  Ogre::SceneNode* moving_station_node_;
  Ogre::SceneNode* relocating_station_node_;
  std::string station_resource_;
  rviz::VectorProperty* current_station_property_;
  ros::Publisher station_pub_;
  bool isRelocate = false;
  int relocate_index;

signals:
    void isRelocateDone(const bool isRelocateDone);


};

} // namespace moobot_ui

#endif

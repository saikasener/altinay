#ifndef WAYPOINT_H
#define WAYPOINT_H
#include <iostream>


namespace Ui {
class WayPoint;
}

class WayPoint
{
public:
    WayPoint(float x, float y, float z);
    
    char name;

    float x,y,z;
    
};

#endif // WAYPOINT_H

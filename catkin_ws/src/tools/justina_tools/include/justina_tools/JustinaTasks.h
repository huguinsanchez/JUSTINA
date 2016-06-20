#pragma once
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTools.h"

class JustinaTasks
{
private:
    static bool is_node_set;

public:
    //
    //The startSomething functions, only publish the goal pose or path and return inmediately after starting movement
    //The others, block until a goal-reached signal is received
    //
    
    static bool setNodeHandle(ros::NodeHandle* nh);
    static bool alignWithTable();
    static bool moveToGraspingPosition(float objectX, float objectY, float objectZ, bool withLeftArm);
};

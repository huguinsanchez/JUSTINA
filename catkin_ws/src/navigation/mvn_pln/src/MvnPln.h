#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <cmath>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "navig_msgs/PathFromMap.h"
#include "sensor_msgs/LaserScan.h"
#include "navig_msgs/PlanPath.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaHardware.h"

#define SM_INIT 0
#define SM_WAITING_FOR_NEW_TASK 1
#define SM_CALCULATE_PATH 2
#define SM_START_MOVE_PATH 3
#define SM_WAIT_FOR_MOVE_FINISHED 4
#define SM_COLLISION_DETECTED 5
#define SM_CORRECT_FINAL_ANGLE 6
#define SM_WAIT_FOR_ANGLE_CORRECTED 7

class MvnPln
{
public:
    MvnPln();
    ~MvnPln();

private:
    ros::NodeHandle* nh;
    //Publishers and subscribers for the commands executed by this node
    ros::ServiceServer srvPlanPath;
    ros::Subscriber subGetCloseLoc;
    ros::Subscriber subGetCloseXYA;
    ros::Subscriber subClickedPoint; //Used to catch clicks on rviz and modify location positions
    ros::Publisher pubGoalReached;
    ros::Publisher pubLocationMarkers;
    ros::Publisher pubLastPath;
    ros::Subscriber subLaserScan;
    ros::Subscriber subCollisionRisk;
    //Ros stuff for path planning
    ros::ServiceClient cltGetMap;
    ros::ServiceClient cltPathFromMapAStar; //Path calculation using only the occupancy grid

    bool newTask;
    bool correctFinalAngle;
    float goalX;
    float goalY;
    float goalAngle;
    std::map<std::string, std::vector<float> > locations;
    nav_msgs::Path lastCalcPath;
    bool collisionDetected;
    bool stopReceived;
    sensor_msgs::LaserScan lastLaserScan;

public:
    void initROSConnection(ros::NodeHandle* nh);
    bool loadKnownLocations(std::string path);
    void spin();

private:
    visualization_msgs::Marker getLocationMarkers();
    bool planPath(float startX, float startY, float goalX, float goalY, nav_msgs::Path& path);
    bool callbackPlanPath(navig_msgs::PlanPath::Request& req, navig_msgs::PlanPath::Response& resp);
    void callbackClickedPoint(const geometry_msgs::PointStamped::ConstPtr& msg);
    void callbackGetCloseLoc(const std_msgs::String::ConstPtr& msg);
    void callbackGetCloseXYA(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg);
    void callbackCollisionRisk(const std_msgs::Bool::ConstPtr& msg);
};

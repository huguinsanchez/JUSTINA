#ifndef TOOLS_JUSTINA_TOOLS_SRC_JUSTINAREPRESENTATION_H_
#define TOOLS_JUSTINA_TOOLS_SRC_JUSTINAREPRESENTATION_H_

#include "ros/ros.h"

#include <map>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"

#include "planning_msgs/PlanningCmdClips.h"
#include "planning_msgs/planning_cmd.h"

#include <boost/algorithm/string/replace.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

class JustinaRepresentation {
private:
	ros::NodeHandle * nh;
	
  static ros::Publisher * command_runCLIPS;
  static ros::Publisher * command_resetCLIPS;
  static ros::Publisher * command_factCLIPS;
  static ros::Publisher * command_ruleCLIPS;
  static ros::Publisher * command_agendaCLIPS;
  static ros::Publisher * command_sendCLIPS;
  static ros::Publisher * command_loadCLIPS;
  

public:

  ~JustinaRepresentation();

	static void setNodeHandle(ros::NodeHandle * nh);
	static void runCLIPS(bool enable);
	static void resetCLIPS(bool enable);
    static void factCLIPS(bool enable);
    static void ruleCLIPS(bool enable);
	static void agendaCLIPS(bool enable);
	static void sendCLIPS(std::string command);
    static void loadCLIPS(std::string file);
    static void getLocations(std::string path ,std::map<std::string, std::vector<std::string> >& locations);
    static void getObjects(std::string path ,std::map<std::string, std::vector<std::string> >& objects);
    static void addLocations(std::map<std::string, std::vector<std::string> >& locations, std::string name, std::vector<std::string> values);
    static void addObjects(std::map<std::string, std::vector<std::string> >& objects, std::string name, std::vector<std::string> values);
  
};

#endif /* TOOLS_JUSTINA_TOOLS_SRC_JUSTINAREPRESENTATION_H_ */
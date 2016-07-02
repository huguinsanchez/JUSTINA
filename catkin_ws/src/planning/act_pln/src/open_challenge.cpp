
#include "ros/ros.h"

#include "planning_msgs/PlanningCmdClips.h"
#include "planning_msgs/planning_cmd.h"

#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"
#include "justina_tools/JustinaManip.h"

#include <vector>

using namespace boost::algorithm;

class GPSRTasks{
public:
	GPSRTasks(ros::NodeHandle * n = 0){
		JustinaNavigation::setNodeHandle(n);
		JustinaHardware::setNodeHandle(n);
		JustinaHRI::setNodeHandle(n);
		JustinaVision::setNodeHandle(n);
		JustinaTasks::setNodeHandle(n);
		JustinaManip::setNodeHandle(n);
		if(n != 0){
			cltSpgSay = n->serviceClient<bbros_bridge::Default_ROS_BB_Bridge>("/spg_say");
		}
	}

	void initRosConnection(ros::NodeHandle * n, std::string locationsFilePath){
		JustinaNavigation::setNodeHandle(n);
		JustinaHardware::setNodeHandle(n);
		JustinaHRI::setNodeHandle(n);
		JustinaVision::setNodeHandle(n);
		JustinaTasks::setNodeHandle(n);
		JustinaManip::setNodeHandle(n);
		cltSpgSay = n->serviceClient<bbros_bridge::Default_ROS_BB_Bridge>("/spg_say");
		loadKnownLocations(locationsFilePath);
		listener = new tf::TransformListener();
		std::cout << "Size of map location:" << locations.size() << std::endl;
		//speechTasks.initRosConnection(n);
	}

	bool loadKnownLocations(std::string path)
	{
	    std::cout << "MvnPln.->Loading known locations from " << path << std::endl;
	    std::vector<std::string> lines;
	    std::ifstream file(path.c_str());
	    std::string tempStr;
	    while(std::getline(file, tempStr))
	        lines.push_back(tempStr);

	    //Extraction of lines without comments
	    for(size_t i=0; i< lines.size(); i++)
	    {
	        size_t idx = lines[i].find("//");
	        if(idx!= std::string::npos)
	            lines[i] = lines[i].substr(0, idx);
	    }

	    this->locations.clear();
	    float locX, locY, locAngle;
	    bool parseSuccess;
	    for(size_t i=0; i<lines.size(); i++)
	    {
	        //std::cout << "MvnPln.->Parsing line: " << lines[i] << std::endl;
	        std::vector<std::string> parts;
	        std::vector<float> loc;
	        boost::split(parts, lines[i], boost::is_any_of(" ,\t"), boost::token_compress_on);
	        if(parts.size() < 3)
	            continue;
	        //std::cout << "MvnPln.->Parsing splitted line: " << lines[i] << std::endl;
	        parseSuccess = true;
	        std::stringstream ssX(parts[1]);
	        if(!(ssX >> locX)) parseSuccess = false;
	        std::stringstream ssY(parts[2]);
	        if(!(ssY >> locY)) parseSuccess = false;
	        loc.push_back(locX);
	        loc.push_back(locY);
	        if(parts.size() >= 4)
	        {
	            std::stringstream ssAngle(parts[3]);
	            if(!(ssAngle >> locAngle)) parseSuccess = false;
	            loc.push_back(locAngle);
	        }

	        if(parseSuccess)
	        {
	            this->locations[parts[0]] = loc;
	        }
	    }
	    std::cout << "GPSRTasks.->Total number of known locations: " << this-locations.size() << std::endl;
	    for(std::map<std::string, std::vector<float> >::iterator it=this->locations.begin(); it != this->locations.end(); it++)
	    {
	        std::cout << "GPSRTasks.->Location " << it->first << " " << it->second[0] << " " << it->second[1];
	        if(it->second.size() > 2)
	            std::cout << " " << it->second[2];
	        std::cout << std::endl;
	    }
	    if(this->locations.size() < 1)
	        std::cout << "GPSRTasks.->WARNING: Cannot load known locations from file: " << path << ". There are no known locations." << std::endl;
	    
	    return true;
	}

	tf::Vector3 transformPoint(tf::StampedTransform transform, tf::Vector3 point){
		return transform * point;
	}

	tf::StampedTransform getTransform(std::string frame1, std::string frame2){
		tf::StampedTransform transform;
		try{
			listener->lookupTransform(frame1, frame2,  
                             	ros::Time(0), transform);
		}
		catch(tf::TransformException ex){
			std::cerr << "error:" << ex.what() << std::endl;
			ros::Duration(1.0).sleep();
		}
		return transform;
	}

	bool syncNavigate(std::string location, float timeOut){
		std::stringstream ss;
		std::cout << "Navigation to " << location << std::endl;
		ss << "I will navigate to the " << location;
		asyncSpeech(ss.str());
		bool reachedLocation = JustinaNavigation::getClose(location, timeOut);
		ss.str("");
		if(reachedLocation){
			ss << "I have reached the " << location;
			syncSpeech(ss.str(), 30000, 2000);
		}
		else{
			ss.str("");
			ss << "I cannot reached the " << location;
			syncSpeech(ss.str(), 30000, 2000);
		}
		return reachedLocation;
	}

	bool syncNavigate(float x, float y, float timeOut){
		return JustinaNavigation::getClose(x, y, timeOut);
	}

	void asyncNavigate(float x, float y){
		JustinaNavigation::startGetClose(x, y);
	}

	bool syncMove(float distance, float angle, float timeOut){
		return JustinaNavigation::moveDistAngle(distance, angle, timeOut);
	}

	void waitHeadGoalPose(float goalHeadPan, float goalHeadTile, float timeOut){
		float currHeadPan, currHeadTile;
		float errorPan, errorTile;
		boost::posix_time::ptime curr;
		boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
		do{
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			JustinaHardware::getHeadCurrentPose(currHeadPan, currHeadTile);
			errorPan = pow(currHeadPan - goalHeadPan, 2);
			errorTile = pow(currHeadTile - goalHeadTile, 2);
			curr = boost::posix_time::second_clock::local_time();
		}while(ros::ok() && errorPan > 0.003 && errorTile > 0.003 && (curr - prev).total_milliseconds() < timeOut);
	}

	void syncMoveHead(float goalHeadPan, float goalHeadTile, float timeOut){
		JustinaHardware::setHeadGoalPose(goalHeadPan, goalHeadTile);
		waitHeadGoalPose(goalHeadPan, goalHeadTile, timeOut);
	}

	void asyncMoveHead(float goalHeadPan, float goalHeadTile){
		JustinaHardware::setHeadGoalPose(goalHeadPan, goalHeadTile);
	}

	void eneableDisableQR(bool option){
		if(option)
			JustinaVision::startQRReader();
		else
			JustinaVision::stopQRReader();
	}

	void enableLegFinder(bool enable){
		JustinaHRI::enableLegFinder(enable);
	}

	void startFollowHuman(){
		JustinaHRI::startFollowHuman();
	}

	void stopFollowHuman(){
		JustinaHRI::stopFollowHuman();	
	}

	bool frontalLegsFound(){
		return JustinaHRI::frontalLegsFound();
	}

	std::vector<vision_msgs::VisionFaceObject> waitRecognizeFace(float timeOut, std::string id, bool &recognized){
		boost::posix_time::ptime curr;
		boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
		boost::posix_time::time_duration diff;
		std::vector<vision_msgs::VisionFaceObject> lastRecognizedFaces;
		if(id.compare("") == 0)
			JustinaVision::facRecognize();
		else
			JustinaVision::facRecognize(id);
		do{
			boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
			ros::spinOnce();
			JustinaVision::getLastRecognizedFaces(lastRecognizedFaces);
			curr = boost::posix_time::second_clock::local_time();
		}while(ros::ok() && (curr - prev).total_milliseconds() < timeOut && lastRecognizedFaces.size() == 0);

		if(lastRecognizedFaces.size() > 0)
			recognized = true;
		else
			recognized = false;
		std::cout << "recognized:" << recognized << std::endl;
		return lastRecognizedFaces;
	}

	Eigen::Vector3d filterRecognizeFace(std::vector<vision_msgs::VisionFaceObject> facesObject, float distanceMax, 
		bool &found){
		int indexMin;
		float distanceMin = 99999999.0;
		Eigen::Vector3d faceCentroid = Eigen::Vector3d::Zero();
		found = false;
		for(int i = 0; i < facesObject.size(); i++){
			vision_msgs::VisionFaceObject vro = facesObject[i];
			Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
			centroid(0,0) = vro.face_centroid.x;
			centroid(1,0) = vro.face_centroid.y;
			centroid(2,0) = vro.face_centroid.z;
			float dist = centroid.norm();
			if(dist < distanceMax && dist < distanceMin){
				indexMin = i;
				distanceMin = dist;
				found = true;
			}
		}
		if(found){
			std::cout << "I found the centroid nearest to robot" << std::endl;
			faceCentroid(0, 0) = facesObject[indexMin].face_centroid.x;
			faceCentroid(1, 0) = facesObject[indexMin].face_centroid.y;
			faceCentroid(2, 0) = facesObject[indexMin].face_centroid.z;
		}
		std::cout << "Face centroid:" << faceCentroid(0, 0) << "," << faceCentroid(1, 0) << "," 
			<< faceCentroid(2, 0) << std::endl;
		return faceCentroid;
	}

	Eigen::Vector3d turnAndRecognizeFace(std::string id,float initAngPan, float incAngPan, float maxAngPan, 
		float incAngleTurn, float maxAngleTurn, bool &recog){

		float currAngPan = initAngPan;
		float currAngleTurn = 0.0;
		float turn = 0.0;
		bool continueReco = true;
		Eigen::Vector3d centroidFace = Eigen::Vector3d::Zero();
		
		do{
			std::cout << "Move base" << std::endl;
			std::cout << "currAngleTurn:"  << currAngleTurn << std::endl;
			asyncMoveHead(currAngPan, 0.0);
			syncMove(0.0, turn, 5000);
			waitHeadGoalPose(currAngPan, 0.0, 5000);
			do{
				std::cout << "Sync move head start" << std::endl;
				std::cout << "Head goal:"  << currAngPan << std::endl;
				syncMoveHead(currAngPan, 0.0, 5000);
				std::cout << "Sync move head end" << std::endl;
				currAngPan += incAngPan;
				boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
				std::vector<vision_msgs::VisionFaceObject> facesObject = waitRecognizeFace(2000, id, recog);
				if(continueReco)
					centroidFace = filterRecognizeFace(facesObject, 3.0, recog);
				if(recog)
					continueReco = false;
			}while(ros::ok() && currAngPan <= maxAngPan && continueReco);
			std::cout << "End turnAndRecognizeFace" << std::endl;
			currAngleTurn += incAngleTurn;
			currAngPan = initAngPan;
			turn = incAngleTurn;
		}while(ros::ok() && currAngleTurn < maxAngleTurn && continueReco);
		return centroidFace;
	}

	bool syncSpeech(std::string textSpeech, float timeOut, float timeSleep){
		bbros_bridge::Default_ROS_BB_Bridge srv;
    	srv.request.parameters = textSpeech;
    	srv.request.timeout = timeOut;
    	if(cltSpgSay.call(srv)){
    		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    		return true;
    	}
		return false;
	}

	void asyncSpeech(std::string textSpeech){
		bbros_bridge::Default_ROS_BB_Bridge srv;
    	srv.request.parameters = textSpeech;
    	srv.request.timeout = 0;
    	cltSpgSay.call(srv);
	}

	bool syncDetectObjects(std::vector<vision_msgs::VisionObject>& recoObjList){
		return JustinaVision::detectObjects(recoObjList);
	}

	void getCurrPose(float &x, float &y, float &theta){
		JustinaNavigation::getRobotPose(x, y, theta);
	}

	bool torsoGoTo(float goalSpine, float goalWaist, float goalShoulders, int timeOut_ms){
		return JustinaManip::torsoGoTo(goalSpine, goalWaist, goalShoulders, timeOut_ms);
	}

	bool findPerson(std::string person = ""){

		std::vector<int> facesDistances;
		std::stringstream ss;

		JustinaVision::startFaceRecognitionOld();
		syncMoveHead(0, 0, 5000);

		std::cout << "Find a person " << person << std::endl;

		ss << "I am going to find a person " << person;
		syncSpeech(ss.str(), 30000, 2000);

		bool recog;
		Eigen::Vector3d centroidFace = turnAndRecognizeFace(person, -M_PI_4, M_PI_4, M_PI_4, M_PI_2, 2 * M_PI, recog);
		std::cout << "CentroidFace:" << centroidFace(0,0) << "," << centroidFace(1,0) << "," << centroidFace(2,0) << ")" << std::endl;
		personLocation.clear();
		JustinaVision::stopFaceRecognition();

		ss.str("");
		if(!recog){
			std::cout << "I have not found a person " << person << std::endl;
			ss << "I have not found a person " << person;
			syncSpeech(ss.str(), 30000, 2000);
			return false;
		}

		std::cout << "I have found a person " << person << std::endl;
		ss << "I have found a person " << person;
		syncSpeech(ss.str(), 30000, 2000);

		tf::StampedTransform transform = getTransform("/map", "/base_link");
		std::cout << "Transform:" << transform.getOrigin().x() << "," << transform.getOrigin().y() << ","
				<< transform.getOrigin().z() << std::endl;

		/*tf::Vector3 kinectFaceCentroid(-centroidFace(1, 0), centroidFace(0, 0), 
					centroidFace(2, 0));*/
		tf::Vector3 kinectFaceCentroid(centroidFace(0, 0), centroidFace(1, 0), centroidFace(2, 0));
		tf::Vector3 worldFaceCentroid = transformPoint(transform, kinectFaceCentroid);
		std::cout << "Person 3D:" << worldFaceCentroid.x() << "," << worldFaceCentroid.y() << ","
				<< worldFaceCentroid.z() << std::endl;

        syncSpeech("I'am getting close to you", 30000, 2000);
        personLocation.push_back(worldFaceCentroid);

        asyncNavigate(worldFaceCentroid.x(), worldFaceCentroid.y());
        float currx, curry, currtheta;
        bool finishReachedPerdon = false;
		do{
			float distanceToGoal;
			getCurrPose(currx, curry, currtheta);
			distanceToGoal = sqrt(pow(currx - worldFaceCentroid.x(), 2) + pow(curry - worldFaceCentroid.y(), 2));
			if((obstacleInFront() && distanceToGoal < 0.6) || distanceToGoal < 0.6)
				finishReachedPerdon = true;
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			ros::spinOnce();
		}while(ros::ok() && !finishReachedPerdon);
		syncNavigate(currx, curry, 10000);

		/*syncNavigate(worldFaceCentroid.x(), worldFaceCentroid.y(), 10000);
		float currx, curry, currtheta;
		getCurrPose(currx, curry, currtheta);
		syncNavigate(currx, curry, 10000);*/

		syncMoveHead(0, 0, 5000);

		return true;
	}

	bool findMan(std::string goalLocation){
		bool found = findPerson();
		if(!found)
			return false;
		std::stringstream ss;
		ss << "I have a follow you to the " << goalLocation;
		std::cout << "Follow to the " << goalLocation << std::endl;
		asyncSpeech(ss.str());
		std_msgs::Bool msg;
		msg.data = true;
		enableLegFinder(true);

		while(ros::ok() && !frontalLegsFound()){
			std::cout << "Not found a legs try to found." << std::endl;
			boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
			ros::spinOnce();
		}
		startFollowHuman();
		
		float currx, curry, currtheta;
		float errorx, errory;
		float dis;
		std::vector<float> location = locations.find(goalLocation)->second;
		do{
			getCurrPose(currx, curry, currtheta);
			errorx = currx - location[0];
			errory = curry - location[1];
			dis = sqrt(pow(errorx,2) + pow(errory,2));
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			ros::spinOnce();
		}while(ros::ok() && dis > 0.6);

		std::cout << "I have reach a location to follow a person in the " << goalLocation << std::endl;
		ss.str("");
		ss << "I have finish follow a person ";
		asyncSpeech(ss.str());

		msg.data = false;
		stopFollowHuman();
		enableLegFinder(false);
		return true;

	}

	bool alignWithTable(){
		bool isAlign = JustinaTasks::alignWithTable(0.35);
		std::cout << "Align With table " << std::endl;
		if(!isAlign){
			std::cout << "Can not align with table." << std::endl;
			return false;
		}
		return true;
	}

	bool alignWithTable(float distToTable){
		std::stringstream ss;
		if(!JustinaManip::hdGoTo(0, -0.9, 5000))
        	JustinaManip::hdGoTo(0, -0.9, 5000);
        float x1, y1, z1, x2, y2, z2;
		bool foundLine = JustinaVision::findLine(x1, y1, z1, x2, y2, z2);
		if(!foundLine){
			ss << "I have not align with table ";
			syncSpeech(ss.str(), 30000, 2000);
			return false;
		}
		if(fabs(z1 - z2) > 0.3){
	        std::cout << "JustinaTasks.->Found line is not confident. " << std::endl;
	        ss << "I have not align with table ";
			syncSpeech(ss.str(), 30000, 2000);
	        return false;
	    }

	    float robotX = 0, robotY =0, robotTheta = 0;
	    float A = y1 - y2;
	    float B = x2 - x1;
	    float C = -(A*x1 + B*y1);
	    float distance = fabs(A*robotX + B*robotY + C)/sqrt(A*A + B*B) - distToTable;

		float deltax , deltay;
		deltax = x1 - x2;
		deltay = y1 - y2;
		float currx, curry, currtheta;
		getCurrPose(currx, curry, currtheta);
		float secondPx = currx + cos(currtheta);
		float secondPy = curry + sin(currtheta);
		Eigen::Vector3d v1 = Eigen::Vector3d::Zero();
		v1(0, 0) = currx - secondPx;
		v1(1, 0) = curry - secondPy;
		Eigen::Vector3d v2 = Eigen::Vector3d::Zero();
		v2(0, 0) = x1 - x2;
		v2(1, 0) = y1 - y2;
		float angle = acos(v1.dot(v2) / (v1.norm() * v2.norm()));

		ss.str();
		ss << "I'am already aligned with table ";
		syncSpeech(ss.str(), 30000, 2000);
		JustinaNavigation::moveDistAngle(distance, angle, 10000);

		return true;
	}

	bool findObject(std::string idObject, geometry_msgs::Pose & pose){
		std::vector<vision_msgs::VisionObject> recognizedObjects;
		std::stringstream ss;
		std::string toSpeech = idObject;

		boost::replace_all(idObject, "_", "-");
		boost::replace_all(toSpeech, "_", " ");

		std::cout << "Find a object " << idObject << std::endl;

		syncMoveHead(0, -0.7854, 5000);
		bool found = syncDetectObjects(recognizedObjects);
		int indexFound = 0;
		if(found){
			found = false;
			for(int i = 0; i < recognizedObjects.size(); i++){
				vision_msgs::VisionObject vObject = recognizedObjects[i];
				if(vObject.id.compare(idObject) == 0){
					found = true;
					indexFound = i;
					break;
				}
			}
		}

		ss.str("");
		if(!found || recognizedObjects.size() == 0){
			ss << "I have not found the object " << toSpeech;
			syncSpeech(ss.str(), 30000, 2000);
			return false;
		}
		
		ss << "I have found the object " << toSpeech;
		syncSpeech(ss.str(), 30000, 2000);

		pose = recognizedObjects[indexFound].pose;
		std::cout << "Position:" << pose.position.x << "," << pose.position.y << "," << pose.position.z << std::endl;
		std::cout << "Orientation:" << pose.orientation.x << "," << pose.orientation.y << 
			"," << pose.orientation.z << "," << pose.orientation.w << std::endl;

		return true;
	}

	bool moveActuator(float x, float y, float z, std::string id){
		std::cout << "Move actuator " << id << std::endl;
		std::vector<vision_msgs::VisionObject> visionObjects;
		std::stringstream ss;

		vision_msgs::VisionObject object;
		object.id = id;
		object.pose.position.x = x;
		object.pose.position.y = y;
		object.pose.position.z = z;
		visionObjects.push_back(object);

		ss << "I'am going to take an object " << id;
		syncSpeech(ss.str(), 30000, 2000);

		bool grasp = JustinaTasks::graspNearestObject(visionObjects, false);
		// TODO Validate to the grasp
		/*if(!grasp){
			ss.str("");
			ss << "I cat not take an object " << id;
			syncSpeech(ss.str(), 30000, 2000);
			return false;
		}*/

		//JustinaManip::startRaCloseGripper(0.4);
		//boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
		JustinaNavigation::moveDistAngle(-0.3, 0.0, 10000);

		ss.str("");
		ss << "I have taken an object " << id;
		syncSpeech(ss.str(), 30000, 2000);

		//JustinaManip::laGoTo("home", 10000);
		return true;

	}

	bool moveActuator(float x, float y, float z, bool withLeftArm,std::string id){
		std::cout << "Move actuator " << id << std::endl;
		std::vector<vision_msgs::VisionObject> visionObjects;
		std::stringstream ss;

		if(withLeftArm)
	        std::cout << "left arm" << std::endl;
	    else
	        std::cout << "right arm" << std::endl;

	    ss << "I'am going to take an object " << id;
		syncSpeech(ss.str(), 30000, 2000);

	    JustinaTasks::graspObject(x, y, z, false);

		//JustinaManip::laGoTo("home", 10000);
		return true;

	}

	bool drop(){
		syncSpeech("I'am going to bring it to you", 30000, 2000);
		syncSpeech("please put your hand", 30000, 2000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
		JustinaManip::raGoTo("take", 10000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
		syncSpeech("I'am going handover the object", 30000, 2000);
		JustinaManip::startRaOpenGripper(0.6);
		JustinaManip::raGoTo("home", 10000);
	}

	bool obstacleInFront(){
		return JustinaNavigation::obstacleInFront();
	}

	std::vector<tf::Vector3> getPersonLocation(){
		return personLocation;
	}

private:
	ros::ServiceClient cltSpgSay;
	tf::TransformListener * listener;
	std::map<std::string, std::vector<float> > locations;
	std::vector<tf::Vector3> personLocation;
};

enum SMState{
	SM_INIT,
	SM_SAY_WAIT_FOR_DOOR,
	SM_WAIT_FOR_DOOR,
	SM_NAVIGATE_TO_THE_LOCATION,
	SM_SEND_INIT_CLIPS,
	SM_RUN_SM_CLIPS
};

std::vector<std::string> objectsids;
ros::Publisher command_response_pub;
SMState state;
std::string testPrompt;
bool hasBeenInit;
GPSRTasks tasks;

bool runSMCLIPS = true;
bool startSignalSM = false;
planning_msgs::PlanningCmdClips initMsg;


// This is for the attemps for a actions
std::string lastCmdName = "";
int numberAttemps = 0;

ros::ServiceClient srvCltGetTasks;
ros::ServiceClient srvCltInterpreter;
ros::ServiceClient srvCltWaitConfirmation;
ros::ServiceClient srvCltWaitForCommand;
ros::ServiceClient srvCltAnswer;
ros::ServiceClient srvCltWhatSee;

void validateAttempsResponse(planning_msgs::PlanningCmdClips msg){
	lastCmdName = msg.name;
	if(msg.successful == 0 && (msg.name.compare("move_actuator") == 0 || msg.name.compare("find_object") == 0)){
		if(msg.name.compare(lastCmdName) != 0)
			numberAttemps = 0;
		else if(numberAttemps == 3){
			msg.successful = 1;
			numberAttemps = 0;
		}
		else
			numberAttemps++;
	}
	command_response_pub.publish(msg);
}

void callbackCmdSpeech(const planning_msgs::PlanningCmdClips::ConstPtr& msg)
{
	std::cout << testPrompt << "--------- Command Speech ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	bool success = true;
	startSignalSM = true;
	
	//if(!runSMCLIPS)
	//	success = false;

	success = success & ros::service::waitForService("/planning_open_challenge/wait_command", 50000);
	if(success){
		planning_msgs::planning_cmd srv;
		srv.request.name = "test_wait";
		srv.request.params = "Ready";
		if(srvCltWaitForCommand.call(srv)){
			std::cout << "Response of wait for command:" << std::endl;
			std::cout << "Success:" << (long int)srv.response.success << std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
		}
		else{
			std::cout << testPrompt << "Failed to call service of wait_command" << std::endl;
			responseMsg.successful = 0;
		}
		responseMsg.params = srv.response.args;
		responseMsg.successful = srv.response.success;
	}
	else{
		if(!runSMCLIPS){
			initMsg = responseMsg;
			return;
		}
		std::cout << testPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	if(runSMCLIPS){
		validateAttempsResponse(responseMsg);
		//command_response_pub.publish(responseMsg);
	}
}

void callbackCmdInterpret(const planning_msgs::PlanningCmdClips::ConstPtr& msg)
{
	std::cout << testPrompt << "--------- Command interpreter ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	bool success = ros::service::waitForService("/planning_open_challenge/interpreter", 5000);
	if(success){
		planning_msgs::planning_cmd srv;
		srv.request.name = "test_interprete";
		srv.request.params = "Ready to interpretation";
		if(srvCltInterpreter.call(srv)){
			std::cout << "Response of interpreter:" << std::endl;
			std::cout << "Success:" << (long int)srv.response.success << std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
			responseMsg.params = srv.response.args;
		responseMsg.successful = srv.response.success;
		}
		else{
			std::cout << testPrompt << "Failed to call service of interpreter" << std::endl;
			responseMsg.successful = 0;
		}
	}
	else{
		std::cout << testPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);

}

void callbackCmdConfirmation(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command confirmation ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	
	bool success = ros::service::waitForService("spg_say", 5000);
	success = success & ros::service::waitForService("/planning_open_challenge/confirmation", 5000);
	if(success){
		std::string to_spech = responseMsg.params;
		boost::replace_all(to_spech, "_", " ");
		std::stringstream ss;
		ss << "Do you want me " << to_spech;
		std::cout << "------------- to_spech: ------------------ " << ss.str() << std::endl;
		tasks.syncSpeech(ss.str(), 30000, 2000);

		planning_msgs::planning_cmd srv;
		srv.request.name = "test_confirmation";
		srv.request.params = responseMsg.params;
		if(srvCltWaitConfirmation.call(srv)){
			std::cout << "Response of confirmation:" << std::endl;
			std::cout << "Success:" << (long int)srv.response.success << std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
			if(srv.response.success)
				tasks.syncSpeech("Ok i start to make the command", 30000, 2000);
			else
				tasks.syncSpeech("Repeate the command please", 30000, 2000);

			responseMsg.params = srv.response.args;
			responseMsg.successful = srv.response.success;
		}
		else{
			std::cout << testPrompt << "Failed to call service of confirmation" << std::endl;
			responseMsg.successful = 0;
			tasks.syncSpeech("Repeate the command please", 30000, 2000);
		}
	}
	else{
		std::cout << testPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

void callbackCmdGetTasks(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command get tasks ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	bool success = ros::service::waitForService("/planning_open_challenge/get_task" ,5000);
	if(success){
		planning_msgs::planning_cmd srv;
		srv.request.name = "cmd_task";
		srv.request.params = "Test of get_task module";
		if(srvCltGetTasks.call(srv)){
			std::cout << "Response of get tasks:" << std::endl;
			std::cout << "Success:" << (long int)srv.response.success << std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
			responseMsg.params = srv.response.args;
			responseMsg.successful = srv.response.success;
		}
		else{
			std::cout << testPrompt << "Failed to call get tasks" << std::endl;
			responseMsg.successful = 0;
		}
	}
	else{
		std::cout << testPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

void callbackCmdExplainThePlan(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Explain the plan ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	bool success = ros::service::waitForService("/planning_open_challenge/get_task", 5000);
	if(success){
		bool finish = false;
		do{
			planning_msgs::planning_cmd srv;
			srv.request.name = "cmd_task";
			srv.request.params = "Test of get_task module";
			if(srvCltGetTasks.call(srv)){
				std::cout << "Response of get tasks:" << std::endl;
				std::cout << "Success:" << (long int)srv.response.success << std::endl;
				std::cout << "Args:" << srv.response.args << std::endl;
				if(!srv.response.success)
					finish = true;
				else{

					std::stringstream ss;
					std::vector<std::string> tokens;
					std::string str = srv.response.args;
					split(tokens, str, is_any_of(" "));

					/*if(tokens[0].compare("find_person_in_room") == 0){
						ss << "I have to find a person "  <<
						tasks.syncSpeech();
					}*/
					/*else if(tokens[0].compare("wait_for_user_instruction") == 0){
						tasks.syncSpeech();
					}*/
					if(tokens[0].compare("update_object_location") == 0){
						ss << "I have to locate the object " << tokens[1] << " on the "  << tokens[1];
						tasks.syncSpeech(ss.str(), 30000, 2000);
					}
					else if(tokens[0].compare("get_object") == 0){
						ss << "I have to align with table";
						tasks.syncSpeech(ss.str(), 30000, 2000);
						ss << "I have to find the object " << tokens[1];
						ss.str("");
						tasks.syncSpeech(ss.str() , 30000, 2000);
						ss << "I have to grasp the object " << tokens[1];
						ss.str("");
						tasks.syncSpeech(ss.str() , 30000, 2000);
					}
					/*else if(tokens[0].compare("put_object_in_location") == 0){
						tasks.syncSpeech();
					}
					else if(tokens[0].compare("save_position") == 0){
						tasks.syncSpeech();
					}
					else if(tokens[0].compare("deliver_in_position") == 0){
						tasks.syncSpeech();
					}*/
					else if(tokens[0].compare("handover_object") == 0){
						//ss << "I have to find the person " << tokens[2];
						ss << "I have to find the person ";
						tasks.syncSpeech(ss.str() ,30000, 2000);
						ss.str("");
						ss << "I have to drop the object " << tokens[1];
						tasks.syncSpeech(ss.str() ,30000, 2000);
					}
				}
			}
			else{
				std::cout << testPrompt << "Failed to call get tasks" << std::endl;
				responseMsg.successful = 0;
			}
			responseMsg.successful = 1;
		}while(ros::ok() && !finish);
	}
	else{
		std::cout << testPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	validateAttempsResponse(responseMsg);
}


void callbackCmdWorld(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command World ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;
	
	int robert = 0;
	int other = 0;
	int robertCI = 0;
	int robertCD = 0;
	int otherCI = 0;
	int otherCD = 0;
	int arthur = 0;
	int arthurCI  = 0;
	int arthurCD  = 0;

	

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	planning_msgs::PlanningCmdClips responseModify;
	responseModify.name = "cmd_modify";
	responseModify.params = "modify";
	responseModify.id = msg->id;

	planning_msgs::PlanningCmdClips responseObject;
	responseObject.name = "cmd_wobj";
	responseObject.params = "wobj";
	responseObject.id = msg->id;

	std::stringstream ss;

	//responseMsg.successful = 1;
	//command_response_pub.publish(responseMsg);

	
	
	//bool success = ros::service::waitForService("spg_say", 5000);
	bool success = ros::service::waitForService("/planning_open_challenge/what_see", 5000);
	if(success){

		planning_msgs::planning_cmd srv;
		srv.request.name = "test_what_see";
		srv.request.params = responseMsg.params;
		tasks.waitHeadGoalPose(0.0, -0.7, 3000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
		tasks.waitHeadGoalPose(-0.6, 0.0, 3000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
		tasks.waitHeadGoalPose(0.6, 0.0, 3000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
		JustinaNavigation::moveLateral(0.3, 4000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(6000));
		JustinaNavigation::moveLateral(-0.3, 4000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(6000));

		if(srvCltWhatSee.call(srv)){
			JustinaVision::startFaceRecognition();
			bool recognized = false;
			float timeOut = 25000.0;
			std::vector<vision_msgs::VisionFaceObject> lastRecognizedFaces;

			boost::posix_time::ptime curr;
			boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
			boost::posix_time::time_duration diff;
			
			std::cout << "Response of what do you see:" << std::endl;
			std::cout << "Success:" << (long int)srv.response.success << std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;


			do{
				boost::this_thread::sleep(boost::posix_time::milliseconds(100));
				JustinaVision::facRecognize();
				JustinaVision::getLastRecognizedFaces(lastRecognizedFaces);
			
				for(int i=0; i<lastRecognizedFaces.size(); i++){
					if(lastRecognizedFaces[i].id == "Peter"){
						robert++;
						if(i == 0){
							robertCI++;
						}
						else{
							robertCD++;
						}
					}
					else if(lastRecognizedFaces[i].id == "John"){
						arthur++;
						if(i == 0){
							arthurCI++;
						}
						else{
							arthurCD++;
						}
					}
					else if(lastRecognizedFaces[i].id == "unknown"){
						other++;
						if(i == 0){
							otherCI++;
						}
						else{
							otherCD++;
						}
					}
				}

				curr = boost::posix_time::second_clock::local_time();
				ros::spinOnce();
			}while(ros::ok() && (curr - prev).total_milliseconds()< timeOut && srv.response.args == "what_see_yes");

			if(arthurCI != arthurCD && arthurCI > arthurCD){
				std::cout << "John esta a la Izquerda" << std::endl;
				ss << "john izquierda";
				tasks.syncSpeech("John is in the left", 30000, 2000);
			}
			else if(arthurCI != arthurCD && arthurCI < arthurCD){
				std::cout << "John esta a la Derecha" << std::endl;
				ss << "john derecha";
				tasks.syncSpeech("john is in the right", 30000, 2000);
			}
			else{
				if(arthur>0){
					std::cout << "John esta SOLO" << std::endl;
					ss << "john solo";
					tasks.syncSpeech("john is the only person I can see", 30000, 2000);
				}
				else
					ss << "john nil";
			}

			if(robertCI != robertCD && robertCI > robertCD)
			{
				std::cout << "Peter esta a la Izquerda" << std::endl;
				ss << " peter izquierda";
				tasks.syncSpeech("Peter is in the left", 30000, 2000);
			}
			else if(robertCI != robertCD && robertCI < robertCD)
			{
				std::cout << "Peter esta a la Derecha" << std::endl;
				ss << " peter derecha";
				tasks.syncSpeech("Peter is in the right", 30000, 2000);
			}
			else
			{
				if(robert>0)
				{
					std::cout << "Peter esta SOLO" << std::endl;
					ss << " peter solo";
					tasks.syncSpeech("Peter is the only person I can see", 30000, 2000);
				}
				else
				{
					ss << " peter nil";
				}
			}
			
			
			
			std::string s = ss.str();
			responseModify.params = s;
			responseModify.successful = 1;
			if(srv.response.args == "what_see_yes")
				command_response_pub.publish(responseModify);
			
				

			//std::cout << "Vector: " << lastRecognizedFaces[0].id << std::endl;
			std::cout << "peter times: " << robert << std::endl;
			std::cout << "john times: " << arthur << std::endl;
			
			//std::cout << "unknown times: " << other << std::endl;
			
			std::cout << "peterIzquierda times: " << robertCI << std::endl;
			//std::cout << "unknowmIzquierda times: " << otherCI << std::endl;
			std::cout << "ArthurIzquierda times: " << arthurCI << std::endl;

			std::cout << "peterDerecha times: " << robertCD << std::endl;
			//std::cout << "unknownDerecha times: " << otherCD << std::endl;
			std::cout << "johnDerecha times: " << arthurCD << std::endl;
			
			/*if(lastRecognizedFaces.size()>0)
				recognized = true;
			else
				recognized = false;*/
			
			
			//command_response_pub.publish(responseMsg);
			JustinaVision::stopFaceRecognition();
			responseMsg.params = srv.response.args;
			responseMsg.successful = srv.response.success;

			if(srv.response.args == "what_see_yes"){
				
				if(objectsids.size()>0)
					objectsids.erase(objectsids.begin());

			int chocosyrup = 0;
			int coconutmilk = 0;
			int coke = 0;
			int shampoo = 0;
			int cranberry = 0;

			std::vector<vision_msgs::VisionObject> recognizedObjects;
			std::cout << "Find a object " << std::endl;
			bool found=0;
			for(int j = 0; j<10; j++){
				std::cout << "Test object" << std::endl;
				found = tasks.syncDetectObjects(recognizedObjects);
				int indexFound = 0;
				if(found){
					found = false;
					for(int i = 0; i < recognizedObjects.size(); i++){
						vision_msgs::VisionObject vObject = recognizedObjects[i];
						std::cout << "object:  " << vObject.id << std::endl;
						if(vObject.id == "choco-syrup")
								chocosyrup++;
						if(vObject.id == "coconut-milk")
								coconutmilk++;
						if(vObject.id == "coke")
								coke++;
						if(vObject.id == "shampoo")
								shampoo++;
						if(vObject.id == "cranberry_juice")
								cranberry++;
						
					}
						
				}
				
			}

			responseObject.successful = 1;
			if(chocosyrup>0){						
				responseObject.params ="choco_syrup table";
				tasks.syncSpeech("There are a grape juice on the table", 30000, 2000);
				command_response_pub.publish(responseObject);
				objectsids.push_back("choco_syrup");
			}
			else
			{
				responseObject.params ="choco_syrup nil";
				tasks.syncSpeech("No", 30000, 2000);
				command_response_pub.publish(responseObject);
			}

			if(coconutmilk>0){						
				responseObject.params ="coconut_milk table";
				tasks.syncSpeech("There are a coconut milk on the table", 30000, 2000);
				command_response_pub.publish(responseObject);
				objectsids.push_back("coconut milk");
			}
			else
			{
				responseObject.params ="coconut_milk nil";
				tasks.syncSpeech("No", 30000, 2000);
				command_response_pub.publish(responseObject);
			}

			if(coke>0){						
				responseObject.params ="coke table";
				tasks.syncSpeech("There are a coke on the table", 30000, 2000);
				command_response_pub.publish(responseObject);
				objectsids.push_back("coke");
			}
			else
			{
				responseObject.params ="coke nil";
				tasks.syncSpeech("No", 30000, 2000);
				command_response_pub.publish(responseObject);
			}
			if(shampoo>0){						
				responseObject.params ="shampoo table";
				tasks.syncSpeech("There are a shampoo on the table", 30000, 2000);
				command_response_pub.publish(responseObject);
				objectsids.push_back("shampoo");
			}

			else
			{
				responseObject.params ="shampoo nil";
				command_response_pub.publish(responseObject);
			}
			if(cranberry>0){						
				responseObject.params ="cranberry_juice";
				tasks.syncSpeech("There are a cranberry juice on the table", 30000, 2000);
				command_response_pub.publish(responseObject);
				objectsids.push_back("cranberry juice");
			}
			else
			{
				responseObject.params ="cranberry_juice nil";
				command_response_pub.publish(responseObject);
			}
				

			}///termina recog objects
		}
		else{
			std::cout << testPrompt << "Failed to call service what do you see" << std::endl;
			responseMsg.successful = 0;
			tasks.syncSpeech("Repeate the question please", 30000, 2000);
		}
	}
	else{
		std::cout << testPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	command_response_pub.publish(responseMsg);
}

void callbackCmdDescribe(const planning_msgs::PlanningCmdClips::ConstPtr& msg)
{
	std::cout << testPrompt << "--------- Command Describe ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseDescribe;
	responseDescribe.name = "cmd_world";
	responseDescribe.params =  "what_see_yes";
	responseDescribe.id = msg->id;
	responseDescribe.successful = 1;
	
	
	std::vector<std::string> tokens;
	std::string str = msg->params;
	split(tokens, str, is_any_of(" "));

	if(tokens[3] == "nil" && tokens[1] == "nil")
	{
		tasks.syncSpeech("There are no persons in the room", 30000, 2000);
		std::cout << "There are no persons in the room" << std::endl;
	}
	else{
		if(tokens[3] != "nil" && tokens[3] != "solo"){
			if(tokens[1] == "derecha"){
				tasks.syncSpeech("peter is in the left of john", 30000, 2000);
				std::cout << "peter is in the left of john" << std::endl;
			}
			if(tokens[1] == "izquerda"){
				tasks.syncSpeech("peter is in the right of john", 30000, 2000);
				std::cout << "peter is in the right of john" << std::endl;
			}
		
		}
		if(tokens[3] == "nil" && tokens[1]!= "nil"){
			tasks.syncSpeech("peter is the only person in the room", 30000, 2000);
			std::cout << "peter is in the only person in the room" << std::endl;
		}

		if(tokens[1] != "nil" && tokens[1] != "solo"){
			if(tokens[3] == "derecha"){
				tasks.syncSpeech("john is in the left of peter", 30000, 2000);
				std::cout << "john is in the left of peter" << std::endl;
			}
			if(tokens[3] == "izquerda"){
				tasks.syncSpeech("john is in the right of peter", 30000, 2000);
				std::cout << "john is in the right of peter" << std::endl;
			}
		}
		if(tokens[1] == "nil" && tokens[3]!="nil"){
			tasks.syncSpeech("john is the only person in the room", 30000, 2000);
			std::cout << "john is the only person in the room" << std::endl;
		}
	}
	std::stringstream ss;
	
	for(int k = 0; k<objectsids.size(); k++ ){
		ss.str("");
		ss << "There are a " << objectsids[k] << " in the table";
		std::cout << ss.str() << std::endl;
		tasks.syncSpeech(ss.str(), 30000, 2000);
		
	}
	if(objectsids.size() == 0)
	{
		std::cout << "There are not objects in the room"<< std::endl;
		tasks.syncSpeech("there are not objects in the room", 30000, 2000);
	}

	

	command_response_pub.publish(responseDescribe);
}

void callbackCmdTakeOrder(const planning_msgs::PlanningCmdClips::ConstPtr& msg)
{
	std::cout << testPrompt << "--------- Take order ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	bool success = true;
	startSignalSM = true;
	
	//if(!runSMCLIPS)
	//	success = false;
	std::cout << "aqui" << std::endl;
	success = success & ros::service::waitForService("/planning_open_challenge/wait_command", 50000);
	std::cout << "hola" << std::endl;
	if(success){
		planning_msgs::planning_cmd srv;
		srv.request.name = "test_wait";
		srv.request.params = "Ready";
		std::cout << "hola" << std::endl;
		if(srvCltWaitForCommand.call(srv)){
			std::cout << "Response of wait for command:" << std::endl;
			std::cout << "Success:" << (long int)srv.response.success << std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
		}
		else{
			std::cout << testPrompt << "Failed to call service of wait_command" << std::endl;
			responseMsg.successful = 0;
		}
		responseMsg.params = srv.response.args;
		responseMsg.successful = srv.response.success;
	}
	else{
		if(!runSMCLIPS){
			initMsg = responseMsg;
			return;
		}
		std::cout << testPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	if(runSMCLIPS){
		validateAttempsResponse(responseMsg);
		//command_response_pub.publish(responseMsg);
	}
	
}

int main(int argc, char **argv){

	ros::init(argc, argv, "open_challenge_test");
	ros::NodeHandle n;
	
	srvCltWhatSee = n.serviceClient<planning_msgs::planning_cmd>("/planning_open_challenge/what_see");
	ros::Subscriber subCmdWorld = n.subscribe("/planning_open_challenge/cmd_world", 1 , callbackCmdWorld);
	ros::Subscriber subCmdDescribe = n.subscribe("/planning_open_challenge/cmd_describe", 1 , callbackCmdDescribe);
	ros::Subscriber subCmdTakeOrder = n.subscribe("/planning_open_challenge/cmd_order", 1 , callbackCmdTakeOrder);
	ros::Subscriber subCmdSpeech = n.subscribe("/planning_open_challenge/cmd_speech", 1 , callbackCmdSpeech);
	ros::Subscriber subCmdInterpret = n.subscribe("/planning_open_challenge/cmd_int", 1 , callbackCmdInterpret);
	ros::Subscriber subCmdConfirmation = n.subscribe("/planning_open_challenge/cmd_conf", 1, callbackCmdConfirmation);
	ros::Subscriber subCmdGetTasks = n.subscribe("/planning_open_challenge/cmd_task", 1, callbackCmdGetTasks);
	ros::Subscriber subCmdExplain = n.subscribe("/planning_open_challenge/cmd_explain", 1, callbackCmdExplainThePlan);

	srvCltGetTasks = n.serviceClient<planning_msgs::planning_cmd>("/planning_open_challenge/get_task");
	srvCltInterpreter = n.serviceClient<planning_msgs::planning_cmd>("/planning_open_challenge/interpreter");
	srvCltWaitConfirmation = n.serviceClient<planning_msgs::planning_cmd>("/planning_open_challenge/confirmation");
	srvCltWaitForCommand = n.serviceClient<planning_msgs::planning_cmd>("/planning_open_challenge/wait_command");
	srvCltAnswer = n.serviceClient<planning_msgs::planning_cmd>("/planning_open_challenge/answer");

	
	command_response_pub = n.advertise<planning_msgs::PlanningCmdClips>("/planning_open_challenge/command_response", 1);

	std::string locationsFilePath = "";
    for(int i=0; i < argc; i++){
        std::string strParam(argv[i]);
        if(strParam.compare("-f") == 0)
            locationsFilePath = argv[++i];
    }

	tasks.initRosConnection(&n, locationsFilePath);

	ros::Rate rate(10);
	state = SM_INIT;

	while(ros::ok()){
 
		switch(state){
			case SM_INIT:
				if(startSignalSM){
					tasks.syncSpeech("Hellow my name is Justina, I'm ready for the open chanlenge", 30000, 2000);
					state = SM_NAVIGATE_TO_THE_LOCATION;
				}
				break;
			case SM_NAVIGATE_TO_THE_LOCATION:
				tasks.syncSpeech("I'am going to the table.", 30000, 2000);
				tasks.syncMove(0.5, 0.0, 3000);
				tasks.alignWithTable();
				state = SM_SEND_INIT_CLIPS;
				break;
			case SM_SEND_INIT_CLIPS:
				tasks.eneableDisableQR(true);
				initMsg.successful = false;
				runSMCLIPS = true;
				command_response_pub.publish(initMsg);
				state = SM_RUN_SM_CLIPS;
				break;
			case SM_RUN_SM_CLIPS:
				break;
		}

		rate.sleep();
		ros::spinOnce();
	}

	return 0;

}

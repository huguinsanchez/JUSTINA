#include "justina_tools/JustinaTasks.h"

bool JustinaTasks::is_node_set = false;

bool JustinaTasks::setNodeHandle(ros::NodeHandle* nh) {
	if (JustinaTasks::is_node_set)
		return true;
	if (nh == 0)
		return false;

	std::cout << "JustinaTasks.->Setting ros node..." << std::endl;
	JustinaHardware::setNodeHandle(nh);
	JustinaHRI::setNodeHandle(nh);
	JustinaManip::setNodeHandle(nh);
	JustinaNavigation::setNodeHandle(nh);
	JustinaVision::setNodeHandle(nh);
	JustinaTools::setNodeHandle(nh);
	JustinaKnowledge::setNodeHandle(nh);

	JustinaTasks::is_node_set = true;
	return true;
}

bool JustinaTasks::alignWithTable() {
	return JustinaTasks::alignWithTable(0.4);
}

bool JustinaTasks::alignWithTable(float distToTable) {
	std::cout << "JustinaTasks.->Aligning with table. Moving head to 0 -0.9"
			<< std::endl;
	if (!JustinaManip::hdGoTo(0, -0.9, 5000))
		JustinaManip::hdGoTo(0, -0.9, 5000);
	std::cout << "JustinaTasks.->Requesting line to line_finder" << std::endl;
	float x1, y1, z1, x2, y2, z2;
	if (!JustinaVision::findLine(x1, y1, z1, x2, y2, z2)) {
		std::cout << "JustinaTasks.->Cannot find line. " << std::endl;
		return false;
	}
	if (fabs(z1 - z2) > 0.3) {
		std::cout << "JustinaTasks.->Found line is not confident. "
				<< std::endl;
		return false;
	}
	float robotX = 0, robotY = 0, robotTheta = 0;
	//std::cout << "JustinaTasks.->Getting robot position.." << std::endl;
	//JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
	//Since line is give wrt robot, we can consider that robot is at zero
	float A = y1 - y2;
	float B = x2 - x1;
	float C = -(A * x1 + B * y1);
	//The robot center should be 0.4 m away of the table
	float distance = fabs(A * robotX + B * robotY + C) / sqrt(A * A + B * B)
			- distToTable;
	float angle = atan2(y2 - y1, x2 - x1) - M_PI / 2;
	if (angle < 0)
		angle += M_PI;
	std::cout << "JustinaTasks.->Moving base: dist=" << distance << "  angle="
			<< angle << std::endl;
	JustinaNavigation::moveDistAngle(distance, angle, 10000);
	return true;
}

bool JustinaTasks::graspNearestObject(bool withLeftArm) {
	std::cout
			<< "JustinaTasks.->Trying to detect objects for manipulating with ";
	if (withLeftArm)
		std::cout << "left arm." << std::endl;
	else
		std::cout << "right arm." << std::endl;
	if (!JustinaManip::hdGoTo(0, -0.9, 5000))
		JustinaManip::hdGoTo(0, -0.9, 5000);
	ros::Rate loop(10);
	int delays = 10;
	//w
	std::cout << "JustinaTasks.->Trying to detect objects..." << std::endl;
	std::vector<vision_msgs::VisionObject> recoObjList;
	if (!JustinaVision::detectObjects(recoObjList)) {
		std::cout << "JustinaTasks.->Cannot dectect objects :'(" << std::endl;
		return false;
	}
	return JustinaTasks::graspNearestObject(recoObjList, withLeftArm);
}

bool JustinaTasks::graspNearestObject(
		std::vector<vision_msgs::VisionObject>& recoObjList, bool withLeftArm) {
	std::cout
			<< "JustinaTasks.->Moving to a good-pose for grasping objects with ";
	if (withLeftArm)
		std::cout << "left arm" << std::endl;
	else
		std::cout << "right arm" << std::endl;

	float idealX = 0.4;
	float idealY = withLeftArm ? 0.235 : -0.235; //It is the distance from the center of the robot, to the center of the arm
	float idealZ = 0.618; //It is the ideal height for taking an object when torso is at zero height.
	float torsoSpine, torsoWaist, torsoShoulders;
	JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist,
			torsoShoulders);
	idealZ += torsoSpine;

	float minDist = 1000000;
	int nearestObj = -1;
	for (size_t i = 0; i < recoObjList.size(); i++) {
		float objX = recoObjList[i].pose.position.x;
		float objY = recoObjList[i].pose.position.y;
		float objZ = recoObjList[i].pose.position.z;
		float temp = sqrt(
				(objX - idealX) * (objX - idealX)
						+ (objY - idealY) * (objY - idealY)
						+ (objZ - idealZ) * (objZ - idealZ));
		if (temp < minDist) {
			minDist = temp;
			nearestObj = i;
		}
	}

	std::string id = recoObjList[nearestObj].id;
	float objToGraspX = recoObjList[nearestObj].pose.position.x;
	float objToGraspY = recoObjList[nearestObj].pose.position.y;
	float objToGraspZ = recoObjList[nearestObj].pose.position.z;
	std::cout << "JustinaTasks.->ObjToGrasp: " << id << "  " << objToGraspX
			<< "  " << objToGraspY << "  " << objToGraspZ << std::endl;
	float movFrontal = -(idealX - objToGraspX);
	float movLateral = -(idealY - objToGraspY);
	float movVertical = -(idealZ - objToGraspZ);
	float goalTorso = torsoSpine + movVertical;
	if (goalTorso < 0)
		goalTorso = 0;
	if (goalTorso > 0.45)
		goalTorso = 0.45;

	std::cout << "JustinaTasks.->Adjusting with frontal=" << movFrontal
			<< " lateral=" << movLateral << " and vertical=" << movVertical
			<< std::endl;
	float lastRobotX, lastRobotY, lastRobotTheta;
	JustinaNavigation::getRobotPose(lastRobotX, lastRobotY, lastRobotTheta);
	JustinaManip::startTorsoGoTo(goalTorso, 0, 0);
	JustinaNavigation::moveLateral(movLateral, 3000);
	JustinaNavigation::moveDist(movFrontal, 3000);
	int waitTime = (int) (30000 * movFrontal + 2000);
	JustinaManip::waitForTorsoGoalReached(waitTime);
	float robotX, robotY, robotTheta;
	JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
	//Adjust the object position according to the new robot pose
	//I don't request again the object position due to the possibility of not recognizing it again
	objToGraspX -= (robotX - lastRobotX);
	objToGraspY -= (robotY - lastRobotY);
	//The position it is adjusted and converted to coords wrt to the corresponding arm
	std::string destFrame = withLeftArm ? "left_arm_link1" : "right_arm_link1";
	if (!JustinaTools::transformPoint("base_link", objToGraspX, objToGraspY,
			objToGraspZ, destFrame, objToGraspX, objToGraspY, objToGraspZ)) {
		std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
		return false;
	}
	std::cout << "JustinaTasks.->Moving ";

	if (withLeftArm)
		std::cout << "left arm";
	else
		std::cout << "right arm";

	std::cout << " to " << objToGraspX << "  " << objToGraspY << "  "
			<< objToGraspZ << std::endl;

	if (withLeftArm) {
		JustinaManip::startLaOpenGripper(0.6);
		JustinaManip::laGoTo("navigation", 5000);
		JustinaManip::laGoToCartesian(objToGraspX - 0.03, objToGraspY - 0.04,
				objToGraspZ, 0, 0, 1.5708, 0, 5000);
		JustinaManip::startLaCloseGripper(0.4);
		ros::Rate loop(10);
		int attempts = 20;
		while (ros::ok() && --attempts > 0)
			loop.sleep();
		JustinaManip::startTorsoGoTo(goalTorso + 0.03, 0, 0);
		JustinaManip::waitForTorsoGoalReached(3000);
		JustinaNavigation::moveDist(-0.15, 3000);
		JustinaManip::laGoTo("navigation", 5000);
	} else {
		JustinaManip::startRaOpenGripper(0.6);
		JustinaManip::raGoTo("navigation", 5000);
		JustinaManip::raGoToCartesian(objToGraspX - 0.03, objToGraspY - 0.04,
				objToGraspZ, 0, 0, 1.5708, 0, 5000);
		JustinaManip::startRaCloseGripper(0.4);
		ros::Rate loop(10);
		int attempts = 20;
		while (ros::ok() && --attempts > 0)
			loop.sleep();
		JustinaManip::startTorsoGoTo(goalTorso + 0.03, 0, 0);
		JustinaManip::waitForTorsoGoalReached(3000);
		JustinaNavigation::moveDist(-0.15, 3000);
		JustinaManip::raGoTo("navigation", 5000);
	}
}

bool JustinaTasks::graspObject(float x, float y, float z, bool withLeftArm,
		std::string idObject) {
	std::cout
			<< "JustinaTasks.->Moving to a good-pose for grasping objects with ";
	if (withLeftArm)
		std::cout << "left arm" << std::endl;
	else
		std::cout << "right arm" << std::endl;

	bool objectInHand = false;
	float idealX = 0.4;
	float idealY = withLeftArm ? 0.234 : -0.235; //It is the distance from the center of the robot, to the center of the arm
	float idealZ = 0.618; //It is the ideal height for taking an object when torso is at zero height.

	float torsoSpine, torsoWaist, torsoShoulders;
	JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist,
			torsoShoulders);
	idealZ += torsoSpine;

	float objToGraspX = x;
	float objToGraspY = y;
	float objToGraspZ = z;
	std::cout << "JustinaTasks.->ObjToGrasp: " << "  " << objToGraspX << "  "
			<< objToGraspY << "  " << objToGraspZ << std::endl;
	float movFrontal = -(idealX - objToGraspX);
	float movLateral = -(idealY - objToGraspY);
	float movVertical = -(idealZ - objToGraspZ);
	float goalTorso = torsoSpine + movVertical;
	if (goalTorso < 0)
		goalTorso = 0;
	if (goalTorso > 0.45)
		goalTorso = 0.45;

	std::cout << "JustinaTasks.->Adjusting with frontal=" << movFrontal
			<< " lateral=" << movLateral << " and vertical=" << movVertical
			<< std::endl;
	float lastRobotX, lastRobotY, lastRobotTheta;
	//JustinaNavigation::getRobotPose(lastRobotX, lastRobotY, lastRobotTheta);
	JustinaNavigation::getRobotPose(lastRobotX, lastRobotY, lastRobotTheta);
	//JustinaManip::startTorsoGoTo(goalTorso, 0, 0);
	JustinaNavigation::moveLateral(movLateral, 6000);
	JustinaNavigation::moveDist(movFrontal, 6000);
	int waitTime = (int) (30000 * movFrontal + 2000);
	//JustinaManip::waitForTorsoGoalReached(waitTime);

	bool found = false;
	std::vector<vision_msgs::VisionObject> recognizedObjects;
	int indexFound = 0;
	if (idObject.compare("") != 0) {
		JustinaManip::startHdGoTo(0, -0.9);
		JustinaManip::waitForHdGoalReached(5000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		found = JustinaVision::detectObjects(recognizedObjects);
		if (found) {
			found = false;
			for (int i = 0; i < recognizedObjects.size(); i++) {
				vision_msgs::VisionObject vObject = recognizedObjects[i];
				if (vObject.id.compare(idObject) == 0) {
					found = true;
					indexFound = i;
					break;
				}
			}
		}
	}

	if (found) {
		std::cout << "The object was found again, update the new coordinates."
				<< std::endl;
		objToGraspX = recognizedObjects[indexFound].pose.position.x;
		objToGraspY = recognizedObjects[indexFound].pose.position.y;
	} else if (!found && idObject.compare("") == 0) {
		std::cout
				<< "The object was not found again, update new coordinates with the motion of robot."
				<< std::endl;
		float robotX, robotY, robotTheta;
		//JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
		JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
		//Adjust the object position according to the new robot pose
		//I don't request again the object position due to the possibility of not recognizing it again
		float dxa = (robotX - lastRobotX);
		float dya = (robotY - lastRobotY);
		float dxr = dxa * cos(robotTheta) + dya * sin(robotTheta);
		float dyr = -dxa * sin(robotTheta) + dya * cos(robotTheta);

		objToGraspX -= dxr;
		objToGraspY -= dyr;
		std::cout << "lastRobotX:" << lastRobotX << ",lastRobotY:" << lastRobotY
				<< ",lastRobotTheta:" << lastRobotTheta << std::endl;
		std::cout << "robotX:" << robotX << ",robotY:" << robotY
				<< ",robotTheta:" << robotTheta << std::endl;
		std::cout << "objToGraspX:" << objToGraspX << ",objToGraspY:"
				<< objToGraspY << ",objToGraspZ:" << objToGraspZ << std::endl;
	} else if (!found && idObject.compare("") != 0) {
		JustinaNavigation::moveDist(-0.2, 3000);
		return false;
	}

	//The position it is adjusted and converted to coords wrt to the corresponding arm
	std::string destFrame = withLeftArm ? "left_arm_link1" : "right_arm_link1";
	if (!JustinaTools::transformPoint("base_link", objToGraspX, objToGraspY,
			objToGraspZ, destFrame, objToGraspX, objToGraspY, objToGraspZ)) {
		std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
		return false;
	}
	std::cout << "JustinaTasks.->Moving ";
	if (withLeftArm)
		std::cout << "left arm";
	else
		std::cout << "right arm";
	std::cout << " to " << objToGraspX << "  " << objToGraspY << "  "
			<< objToGraspZ << std::endl;

	if (withLeftArm) {
		JustinaManip::startLaOpenGripper(0.6);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		JustinaManip::laGoTo("navigation", 7000);
		JustinaManip::laGoToCartesian(objToGraspX - 0.05, objToGraspY + 0.04,
				objToGraspZ, 0, 0, 1.5708, 0, 5000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		JustinaManip::startLaCloseGripper(0.5);
		boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
		ros::spinOnce();
		//JustinaManip::startTorsoGoTo(goalTorso + 0.03, 0, 0);
		//JustinaManip::waitForTorsoGoalReached(6000);
		if (JustinaManip::objOnLeftHand()) {
			JustinaManip::laGoToCartesian(objToGraspX - 0.13,
					objToGraspY + 0.04, objToGraspZ, 0, 0, 1.5708, 0, 5000);
			JustinaNavigation::moveDist(-0.35, 3000);
			JustinaManip::laGoTo("navigation", 5000);
			std::cout
					<< "The object was grasp with the left arm in the first test"
					<< std::endl;
			return true;
		}
		JustinaNavigation::moveDist(-0.2, 3000);
		JustinaManip::laGoTo("navigation", 5000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));
		if (JustinaManip::objOnLeftHand()) {
			std::cout
					<< "The object was grasp with the left arm in the second test"
					<< std::endl;
			return true;
		}
		std::cout << "The object was not grasp with the left arm" << std::endl;
		return false;
	} else {
		JustinaManip::startRaOpenGripper(0.6);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		JustinaManip::raGoTo("navigation", 10000);
		JustinaManip::raGoToCartesian(objToGraspX, objToGraspY + 0.04,
				objToGraspZ, 0, 0, 1.5708, 0, 5000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		JustinaManip::startRaCloseGripper(0.5);
		boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
		ros::spinOnce();
		//JustinaManip::startTorsoGoTo(goalTorso + 0.03, 0, 0);
		//JustinaManip::waitForTorsoGoalReached(6000);
		if (JustinaManip::objOnRightHand()) {
			JustinaManip::raGoToCartesian(objToGraspX - 0.1, objToGraspY + 0.04,
					objToGraspZ, 0, 0, 1.5708, 0, 5000);
			JustinaNavigation::moveDist(-0.35, 3000);
			JustinaManip::raGoTo("navigation", 5000);
			std::cout
					<< "The object was grasp with the right arm in the first test"
					<< std::endl;
			return true;
		}
		JustinaNavigation::moveDist(-0.2, 3000);
		JustinaManip::raGoTo("navigation", 5000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));
		if (JustinaManip::objOnRightHand()) {
			std::cout
					<< "The object was grasp with the right arm in the second test"
					<< std::endl;
			return true;
		}
		std::cout << "The object was not grasp with the right arm" << std::endl;
		return false;
	}
	return false;
}

bool JustinaTasks::placeObject(bool withLeftArm) {
	float x, y, z;
	JustinaVision::findVacantPlane(x, y, z);
	std::cout << "JustinaTasks::placeObject..." << std::endl;

	return true;
}

void JustinaTasks::sayAndAsyncNavigateToLoc(std::string location, bool say) {
	std::stringstream ss;
	std::cout << "Navigation to " << location << std::endl;
	ss << "I will navigate to the " << location;
	if (say)
		JustinaHRI::say(ss.str());
	JustinaNavigation::startGetClose(location);
}

bool JustinaTasks::sayAndSyncNavigateToLoc(std::string location, int timeout,
		bool say) {
	std::stringstream ss;
	std::cout << "Navigation to " << location << std::endl;
	ss << "I will navigate to the " << location;
	if (say)
		JustinaHRI::say(ss.str());
	bool reachedLocation = JustinaNavigation::getClose(location, timeout);
	ss.str("");
	if (reachedLocation) {
		ss << "I have reached the " << location;
		if (say)
			JustinaHRI::waitAfterSay(ss.str(), 4000);
	} else {
		ss.str("");
		ss << "I cannot reached the " << location;
		if (say)
			JustinaHRI::waitAfterSay(ss.str(), 4000);
	}
	return reachedLocation;
}

std::vector<vision_msgs::VisionFaceObject> JustinaTasks::waitRecognizedFace(
		float timeout, std::string id, bool &recognized) {
	boost::posix_time::ptime curr;
	boost::posix_time::ptime prev =
			boost::posix_time::second_clock::local_time();
	boost::posix_time::time_duration diff;
	std::vector<vision_msgs::VisionFaceObject> lastRecognizedFaces;
	if (id.compare("") == 0)
		JustinaVision::facRecognize();
	else
		JustinaVision::facRecognize(id);
	do {
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		ros::spinOnce();
		JustinaVision::getLastRecognizedFaces(lastRecognizedFaces);
		curr = boost::posix_time::second_clock::local_time();
	} while (ros::ok() && (curr - prev).total_milliseconds() < timeout
			&& lastRecognizedFaces.size() == 0);

	if (lastRecognizedFaces.size() > 0)
		recognized = true;
	else
		recognized = false;
	std::cout << "recognized:" << recognized << std::endl;
	return lastRecognizedFaces;
}

Eigen::Vector3d JustinaTasks::getNearestRecognizedFace(
		std::vector<vision_msgs::VisionFaceObject> facesObject,
		float distanceMax, bool &found) {
	int indexMin;
	float distanceMin = 99999999.0;
	Eigen::Vector3d faceCentroid = Eigen::Vector3d::Zero();
	found = false;
	for (int i = 0; i < facesObject.size(); i++) {
		vision_msgs::VisionFaceObject vro = facesObject[i];
		Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
		centroid(0, 0) = vro.face_centroid.x;
		centroid(1, 0) = vro.face_centroid.y;
		centroid(2, 0) = vro.face_centroid.z;
		float dist = centroid.norm();
		if (dist < distanceMax && dist < distanceMin) {
			indexMin = i;
			distanceMin = dist;
			found = true;
		}
	}
	if (found) {
		std::cout << "I found the centroid nearest to robot" << std::endl;
		faceCentroid(0, 0) = facesObject[indexMin].face_centroid.x;
		faceCentroid(1, 0) = facesObject[indexMin].face_centroid.y;
		faceCentroid(2, 0) = facesObject[indexMin].face_centroid.z;
	}
	std::cout << "Face centroid:" << faceCentroid(0, 0) << ","
			<< faceCentroid(1, 0) << "," << faceCentroid(2, 0);
	std::cout << std::endl;
	return faceCentroid;
}

Eigen::Vector3d JustinaTasks::turnAndRecognizeFace(std::string id,
		float initAngPan, float incAngPan, float maxAngPan, float incAngleTurn,
		float maxAngleTurn, bool &recog) {

	float currAngPan = initAngPan;
	float currAngleTurn = 0.0;
	float turn = 0.0;
	bool continueReco = true;
	Eigen::Vector3d centroidFace = Eigen::Vector3d::Zero();

	do {
		std::cout << "Move base" << std::endl;
		std::cout << "currAngleTurn:" << currAngleTurn << std::endl;
		JustinaManip::startHdGoTo(currAngPan, 0.0);
		JustinaNavigation::moveDistAngle(0, turn, 10000);
		JustinaManip::waitForHdGoalReached(5000);
		do {
			std::cout << "Sync move head start" << std::endl;
			std::cout << "Head goal:" << currAngPan << std::endl;
			JustinaManip::startHdGoTo(currAngPan, 0.0);
			JustinaManip::waitForHdGoalReached(5000);
			std::cout << "Sync move head end" << std::endl;
			currAngPan += incAngPan;
			boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
			std::vector<vision_msgs::VisionFaceObject> facesObject =
					waitRecognizedFace(2000, id, recog);
			if (continueReco)
				centroidFace = getNearestRecognizedFace(facesObject, 3.0,
						recog);
			if (recog)
				continueReco = false;
		} while (ros::ok() && currAngPan <= maxAngPan && continueReco);
		std::cout << "End turnAndRecognizeFace" << std::endl;
		currAngleTurn += incAngleTurn;
		currAngPan = initAngPan;
		turn = incAngleTurn;
	} while (ros::ok() && currAngleTurn < maxAngleTurn && continueReco);
	return centroidFace;
}

bool JustinaTasks::findPerson(std::string person) {

	std::vector<int> facesDistances;
	std::stringstream ss;

	JustinaVision::startFaceRecognitionOld();

	JustinaManip::startHdGoTo(0, 0.0);
	JustinaManip::waitForHdGoalReached(5000);

	std::cout << "Find a person " << person << std::endl;

	ss << person << ", I am going to find you";
	JustinaHRI::waitAfterSay(ss.str(), 2000);

	bool recog;
	Eigen::Vector3d centroidFace = turnAndRecognizeFace(person, -M_PI_4,
	M_PI_4, M_PI_4, M_PI_2, 2 * M_PI, recog);
	std::cout << "Centroid Face in coordinates of robot:" << centroidFace(0, 0)
			<< "," << centroidFace(1, 0) << "," << centroidFace(2, 0) << ")";
	std::cout << std::endl;
	//personLocation.clear();
	JustinaVision::stopFaceRecognition();

	ss.str("");
	if (!recog) {
		std::cout << "I have not found a person " << person << std::endl;
		ss << "I did not find the person " << person;
		JustinaHRI::waitAfterSay(ss.str(), 2000);
		return false;
	}

	std::cout << "I have found a person " << person << std::endl;
	ss << person << ", I found you";
	JustinaHRI::waitAfterSay(ss.str(), 2000);

	float cx, cy, cz;
	cx = centroidFace(0, 0);
	cy = centroidFace(1, 0);
	cz = centroidFace(2, 0);
	JustinaTools::transformPoint("/base_link", cx, cy, cz, "/map", cx, cy, cz);
	tf::Vector3 worldFaceCentroid(cx, cy, cz);

	JustinaHRI::waitAfterSay("I am getting close to you", 2000);
	//personLocation.push_back(worldFaceCentroid);

	float currx, curry, currtheta;
	bool finishReachedPerson = false;

	float distanceToGoal;
	JustinaNavigation::getRobotPose(currx, curry, currtheta);
	distanceToGoal = sqrt(
			pow(worldFaceCentroid.x() - currx, 2)
					+ pow(worldFaceCentroid.y() - curry, 2));
	if (distanceToGoal > 0.8) {
		JustinaNavigation::startGetClose(worldFaceCentroid.x(),
				worldFaceCentroid.y());
		do {
			JustinaNavigation::getRobotPose(currx, curry, currtheta);
			distanceToGoal = sqrt(
					pow(worldFaceCentroid.x() - currx, 2)
							+ pow(worldFaceCentroid.y() - curry, 2));
			if ((JustinaNavigation::obstacleInFront() && distanceToGoal < 0.8)
					|| distanceToGoal < 0.8)
				finishReachedPerson = true;
			else {
				boost::this_thread::sleep(boost::posix_time::milliseconds(100));
				ros::spinOnce();
			}
		} while (ros::ok() && !finishReachedPerson);
		std::cout << "JustinaTasks.->I have the reached position." << std::endl;
		JustinaHardware::stopRobot();
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));
		ros::spinOnce();
	} else
		std::cout << "JustinaTasks.->Robot dont need to move." << std::endl;

	float thetaToGoal = atan2(worldFaceCentroid.y() - curry,
			worldFaceCentroid.x() - currx);
	if (thetaToGoal < 0.0f)
		thetaToGoal = 2 * M_PI + thetaToGoal;
	float theta = thetaToGoal - currtheta;
	std::cout << "JustinaTasks.->Turn in direction of robot:" << theta
			<< std::endl;
	JustinaNavigation::moveDistAngle(0, theta, 2000);

	JustinaManip::startHdGoTo(0, 0.0);
	JustinaManip::waitForHdGoalReached(5000);

	return true;
}

bool JustinaTasks::findAndFollowPersonToLoc(std::string goalLocation) {
	bool found = findPerson();
	if (!found)
		return false;
	std::stringstream ss;
	ss << "I am going to follow you to the " << goalLocation;
	std::cout << "Follow to the " << goalLocation << std::endl;
	JustinaHRI::say(ss.str());

	JustinaHRI::enableLegFinder(true);

	while (ros::ok() && !JustinaHRI::frontalLegsFound()) {
		std::cout << "Not found a legs try to found." << std::endl;
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		ros::spinOnce();
	}

	JustinaHRI::startFollowHuman();

	float currx, curry, currtheta;
	float errorx, errory;
	float dis;

	std::map<std::string, std::vector<float> > locations;
	JustinaKnowledge::getKnownLocations(locations);
	std::vector<float> location = locations.find(goalLocation)->second;
	do {
		JustinaNavigation::getRobotPose(currx, curry, currtheta);
		errorx = currx - location[0];
		errory = curry - location[1];
		dis = sqrt(pow(errorx, 2) + pow(errory, 2));
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		ros::spinOnce();
	} while (ros::ok() && dis > 1.6);

	std::cout << "I have reach a location to follow a person in the "
			<< goalLocation << std::endl;
	ss.str("");
	ss << "I have finish follow a person ";
	JustinaHRI::say(ss.str());

	JustinaHRI::stopFollowHuman();

	JustinaHRI::enableLegFinder(false);

	return true;
}

bool JustinaTasks::findObject(std::string idObject, geometry_msgs::Pose & pose,
		bool & withLeftOrRightArm) {
	std::vector<vision_msgs::VisionObject> recognizedObjects;
	std::stringstream ss;
	std::string toSpeech = idObject;

	boost::replace_all(idObject, "_", "-");
	boost::replace_all(toSpeech, "_", " ");

	std::cout << "Find a object " << idObject << std::endl;

	JustinaManip::startHdGoTo(0, -0.785);
	JustinaManip::waitForHdGoalReached(5000);

	bool found = JustinaVision::detectObjects(recognizedObjects);
	int indexFound = 0;
	if (found) {
		found = false;
		for (int i = 0; i < recognizedObjects.size(); i++) {
			vision_msgs::VisionObject vObject = recognizedObjects[i];
			if (vObject.id.compare(idObject) == 0) {
				found = true;
				indexFound = i;
				break;
			}
		}
	}

	ss.str("");
	if (!found || recognizedObjects.size() == 0) {
		ss << "I can not find the " << toSpeech;
		JustinaHRI::waitAfterSay(ss.str(), 2000);
		return false;
	}

	ss << "I found the " << toSpeech;
	JustinaHRI::waitAfterSay(ss.str(), 2000);

	pose = recognizedObjects[indexFound].pose;
	std::cout << "Position:" << pose.position.x << "," << pose.position.y << ","
			<< pose.position.z << std::endl;
	std::cout << "Orientation:" << pose.orientation.x << ","
			<< pose.orientation.y << "," << pose.orientation.z << ","
			<< pose.orientation.w << std::endl;

	if (pose.position.y <= 0)
		withLeftOrRightArm = false;
	else
		withLeftOrRightArm = true;

	std::cout << "JustinaTask.->withLeftOrRightArm:" << withLeftOrRightArm
			<< std::endl;

	return true;
}

bool JustinaTasks::moveActuatorToGrasp(float x, float y, float z,
		bool withLeftArm, std::string id) {
	std::cout << "Move actuator " << id << std::endl;
	std::stringstream ss;

	if (withLeftArm)
		std::cout << "left arm" << std::endl;
	else
		std::cout << "right arm" << std::endl;

	ss << "I am going to take the " << id;
	JustinaHRI::waitAfterSay(ss.str(), 2000);

	float xf = x, yf = y, zf = z;

	int maxAttemps = 4;
	bool isGrasp = false, isFind = true;
	for (int attemps = 0; attemps < maxAttemps && !isGrasp; attemps++) {
		if (attemps > 0) {
			int attempsToFind = 0, indexFound;
			geometry_msgs::Pose pose;
			std::vector<vision_msgs::VisionObject> recognizedObjects;

			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			JustinaTasks::alignWithTable(0.35);
			JustinaManip::startHdGoTo(0, -0.785);
			JustinaManip::waitForHdGoalReached(5000);

			isFind = false;
			for (int findAttemps = 0; findAttemps < maxAttemps && !isFind;
					findAttemps++) {
				boost::this_thread::sleep(boost::posix_time::milliseconds(500));
				isFind = JustinaVision::detectObjects(recognizedObjects);
				if (isFind) {
					isFind = false;
					for (int i = 0; i < recognizedObjects.size(); i++) {
						vision_msgs::VisionObject vObject = recognizedObjects[i];
						if (vObject.id.compare(id) == 0) {
							isFind = true;
							indexFound = i;
							break;
						}
					}
				}
				if (isFind)
					pose = recognizedObjects[indexFound].pose;
			}
			if (isFind) {
				xf = pose.position.x;
				yf = pose.position.y;
				zf = pose.position.z;
			}
		}
		if (isFind)
			isGrasp = JustinaTasks::graspObject(xf, yf, zf, withLeftArm,
					attemps < maxAttemps - 1 ? id : "");
	}

	//JustinaManip::laGoTo("home", 10000);
	return isGrasp;

}

bool JustinaTasks::dropObject(std::string id) {
	JustinaManip::hdGoTo(0, 0.0, 5000);
	if (id.compare("") == 0)
		JustinaHRI::waitAfterSay("I am going to give it to you", 2000);
	else {
		std::stringstream ss;
		ss << "I am going to give you the " << id;
		JustinaHRI::waitAfterSay(ss.str(), 2000);
	}
	JustinaManip::hdGoTo(0, -0.9, 5000);
	JustinaHRI::waitAfterSay("please put your hand", 2000);
	JustinaManip::raGoTo("take", 10000);
	boost::this_thread::sleep(boost::posix_time::milliseconds(200));
	JustinaVision::startHandDetectBB(0.50, -0.15, 0.95);
	ros::Rate rate(10);
	while (ros::ok() && !JustinaVision::getDetectionHandBB()) {
		rate.sleep();
		ros::spinOnce();
	}
	JustinaVision::stopHandDetectBB();
	boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
	JustinaHRI::waitAfterSay("I am going hand over the object", 2000);
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	JustinaManip::startRaOpenGripper(0.6);
	boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
	JustinaManip::startRaOpenGripper(0.0);
	JustinaNavigation::moveDist(-0.25, 2000);
	JustinaManip::raGoTo("navigation", 10000);
	JustinaManip::raGoTo("home", 10000);
	return true;
}

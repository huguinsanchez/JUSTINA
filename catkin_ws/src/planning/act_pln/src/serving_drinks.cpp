#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "knowledge_msgs/PlanningCmdClips.h"
#include "knowledge_msgs/planning_cmd.h"
#include "knowledge_msgs/StrQueryKDB.h"

#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaRepresentation.h"

#include <vector>
#include <ctime>
#include <map>
#define TIMEOUT_MEMORIZING 3000

using namespace boost::algorithm;
std::vector<std::string> idsPerson;

enum SMState {
	SM_INIT,
	SM_SAY_WAIT_FOR_DOOR,
    SM_SCRIPT,
    SM_WAIT_CLIPS,
	SM_WAIT_FOR_DOOR,
	SM_NAVIGATE_TO_THE_LOCATION,
	SM_SEND_INIT_CLIPS,
	SM_RUN_SM_CLIPS,
    SM_RESET_CLIPS 
};

ros::Publisher command_response_pub;
ros::Publisher sendAndRunClips_pub;
ros::Publisher train_face_pub;
ros::Publisher pubStartTime; 
ros::Publisher pubResetTime;
std::string testPrompt;
SMState state = SM_INIT;
bool runSMCLIPS = false;
bool startSignalSM = false;
knowledge_msgs::PlanningCmdClips initMsg;
knowledge_msgs::PlanningCmdClips initSpeech;

// This is for the attemps for a actions
std::string lastCmdName = "";
std::string currentName = "";
std::string objectName = "";
std::string categoryName = "";
int numberAttemps = 0;
int cantidad = 0;
int women;
int men;
int sitting;
int standing;
int lying;
ros::Time beginPlan;
bool fplan = false;
double maxTime = 180;
std::string cat_grammar= "eegpsr_montreal.xml";
bool alternative_drink = true;
std::string no_drink;
std::string prev_drink = "no_prev";

ros::ServiceClient srvCltGetTasks;
ros::ServiceClient srvCltInterpreter;
ros::ServiceClient srvCltWaitConfirmation;
ros::ServiceClient srvCltWaitForCommand;
ros::ServiceClient srvCltAnswer;
ros::ServiceClient srvCltAskName;
ros::ServiceClient srvCltAskIncomplete;
ros::ServiceClient srvCltQueryKDB;

void validateAttempsResponse(knowledge_msgs::PlanningCmdClips msg) {
	//lastCmdName = msg.name;
	if (msg.successful == 0
			&& (msg.name.compare("move_actuator") == 0
					|| msg.name.compare("find_object") == 0
					|| msg.name.compare("status_object") == 0
					|| msg.name.compare("many_obj") == 0
					|| msg.name.compare("answer") == 0
					|| msg.name.compare("drop") == 0)) {
		if (msg.name.compare(lastCmdName) != 0)
			numberAttemps = 0;
		else if (numberAttemps == 0) {
			msg.successful = 1;
			numberAttemps = 0;
		} else
			numberAttemps++;
	}
	else if (msg.successful == 1){
		numberAttemps = 0;
	}
	
	lastCmdName = msg.name;
	command_response_pub.publish(msg);
}

bool validateContinuePlan(double currentTime, bool fplan)
{
	bool result = true;

	if (currentTime >= maxTime && fplan){
		std::stringstream ss;
		knowledge_msgs::StrQueryKDB srv;
		ss.str("");
		ss << "(assert (cmd_finish_plan 1))";
		srv.request.query = ss.str();
		if (srvCltQueryKDB.call(srv)) {
			std::cout << "Response of KBD Query:" << std::endl;
			std::cout << "TEST QUERY Args:" << srv.response.result << std::endl;
			result = false;
			beginPlan = ros::Time::now();
		} else {
			std::cout << testPrompt << "Failed to call service of KBD query"<< std::endl;
			result =  true;
		}
	}
	
	return result;

}

void callbackCmdSpeech(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Speech ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	bool success = true;
	startSignalSM = true;

	if (!runSMCLIPS)
		success = false;

	success = success
			& ros::service::waitForService("/planning_clips/wait_command",
					50000);
    success = false;
	if (success) {
		knowledge_msgs::planning_cmd srv;
		srv.request.name = "test_wait";
		srv.request.params = "Ready";
		if (srvCltWaitForCommand.call(srv)) {
			std::cout << "Response of wait for command:" << std::endl;
			std::cout << "Success:" << (long int) srv.response.success
					<< std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
		} else {
			std::cout << testPrompt << "Failed to call service of wait_command"
					<< std::endl;
			responseMsg.successful = 0;
		}
		responseMsg.params = srv.response.args;
		responseMsg.successful = srv.response.success;
	} else {
		if (!runSMCLIPS) {
			initMsg = responseMsg;
			return;
		}
		std::cout << testPrompt << "Needed services are not available :'("
				<< std::endl;
		responseMsg.successful = 0;
	}
	if (runSMCLIPS) {
		//validateAttempsResponse(responseMsg);
		//command_response_pub.publish(responseMsg);
	}
}


void callbackCmdSpeechGenerator(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command Speech Generator-----" << std::endl;
	std::cout << "name: " << msg->name << std::endl;
	std::cout << "params: " << msg->params << std::endl;
	
	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::stringstream ss;
        std::vector<std::string> tokens;
        std::string str = responseMsg.params;
        split(tokens, str, is_any_of("_"));
	
	
	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;

	ss << tokens[0];
	for(int i=1 ; i<tokens.size(); i++)
		ss << " "<< tokens[i];
	
	JustinaHRI::waitAfterSay(ss.str(), 10000);
	
	responseMsg.successful = 1;

	command_response_pub.publish(responseMsg);
	

}

void callbackCmdNavigation(
		const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Navigation ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;

	bool success = true;
	bool nfp = true;	
	if (tokens[1] != "arena" && tokens[1] != "exitdoor")
		nfp = validateContinuePlan(d.toSec(), fplan);

	if (tokens[1] == "person") {
		success = true;
	} else {
		if (nfp)
			success = JustinaTasks::sayAndSyncNavigateToLoc(tokens[1], 120000);
	}
	if (success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;
	if (nfp)
		validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}


void callbackCmdFindObject(
		const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command find a object ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));
	std::stringstream ss;

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;
	bool nfp = validateContinuePlan(d.toSec(), fplan);
	int timeout = (fplan == true) ? (maxTime - (int)d.toSec() )*1000 : maxTime * 1000;
	std::cout << "TIMEOUT: " << timeout <<std::endl;
	
	bool success = ros::service::waitForService("spg_say", 5000);
	if (success && nfp) {
		std::cout << testPrompt << "find: " << tokens[0] << std::endl;

		ss.str("");
		if (tokens[0] == "person") {
			//success = JustinaTasks::findPerson("", -1, JustinaTasks::NONE, false, tokens[1]);
            success = JustinaTasks::findYolo(idsPerson);
			ss << responseMsg.params << " " << 1 << " " << 1 << " " << 1;
		} else if (tokens[0] == "man") {
			JustinaHRI::loadGrammarSpeechRecognized("follow_confirmation.xml");
			if(tokens[1] == "no_location")
				success = JustinaTasks::followAPersonAndRecogStop("stop follow me");
			else
				success = JustinaTasks::findAndFollowPersonToLoc(tokens[1]);
			ss << responseMsg.params;
		} else if (tokens[0] == "man_guide") {
			JustinaNavigation::moveDistAngle(0, 3.1416 ,2000);
			boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
			success = JustinaTasks::guideAPerson(tokens[1], timeout);
			ss << responseMsg.params;
		} else if (tokens[0] == "specific") {
			success = JustinaTasks::findPerson(tokens[1], -1, JustinaTasks::NONE, false, tokens[2]);//success = JustinaTasks::findPerson(tokens[1])
			ss << "find_spc_person " << tokens[0] << " " << tokens[1] << " " << tokens[2];//ss << responseMsg.params;
        }else if (tokens[0] == "specific_eegpsr"){
            success = JustinaTasks::findPerson(tokens[1], -1, JustinaTasks::NONE, true, tokens[2]);//success = JustinaTasks::findPerson(tokens[1])
			ss << "find_spc_person " << tokens[0] << " " << tokens[1] << " " << tokens[2];//ss << responseMsg.params;
		} else if (tokens[0] == "only_find"){
			bool withLeftOrRightArm;
			ss.str("");
			ss << "I am looking for " << tokens[1] << " on the " << tokens[2];
			geometry_msgs::Pose pose;
			JustinaTasks::alignWithTable(0.42);
			boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
			JustinaHRI::waitAfterSay(ss.str(), 2500);
			success = JustinaTasks::findObject(tokens[1], pose, withLeftOrRightArm);
			ss.str("");
			ss << tokens[1] << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z ;
		} else {
			geometry_msgs::Pose pose;
			bool withLeftOrRightArm;
			bool finishMotion = false;
			float pos = 0.0, advance = 0.3, maxAdvance = 0.3;
			do{
				success = JustinaTasks::findObject(tokens[0], pose, withLeftOrRightArm);
				/*pos += advance;
				if ( pos == maxAdvance && !success){
					JustinaNavigation::moveLateral(advance, 2000);
					boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
					advance = -2 * advance;
				}
				if (pos == -1 * maxAdvance && !success){
					JustinaNavigation::moveLateral(advance, 2000);
					boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
				}
				if (pos == -3 *maxAdvance && !success){
					JustinaNavigation::moveLateral(0.3, 2000);
					boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
					finishMotion = true;}*/
				finishMotion = true;
			}while(!finishMotion && !success);

			if(withLeftOrRightArm)
				ss << responseMsg.params << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << " left";
			else
				ss << responseMsg.params << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z << " right";
		}
		responseMsg.params = ss.str();
	}
	JustinaHRI::loadGrammarSpeechRecognized(cat_grammar);
	if (success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;
	if(tokens[0] == "only_find"){
		if(nfp) command_response_pub.publish(responseMsg);}
	else
		{if(nfp) validateAttempsResponse(responseMsg);}
	
}


void callbackMoveActuator(
		const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Move actuator ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));
	bool armFlag = true;
	std::stringstream ss;

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;
	
	bool success = ros::service::waitForService("spg_say", 5000);
	//success = success & tasks.moveActuator(atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()), tokens[0]);
	if(tokens[4] == "false")
			armFlag = false;

	//ss << "I try to grasp the " << tokens[0];
	//JustinaHRI::waitAfterSay(ss.str(), 10000);
	
	success = success & JustinaTasks::moveActuatorToGrasp(atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()), armFlag, tokens[0], true);
	if (success)
		responseMsg.successful = 1;
	else{
		ss.str("");
		ss << "I did not grasp the " << tokens[0];
		JustinaHRI::waitAfterSay(ss.str(), 100000);
		responseMsg.successful = 0;
	}

	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

void callbackDrop(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command Drop ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));
	std::stringstream ss;
	bool armFlag = true;
	bool succes;

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;
	bool nfp = validateContinuePlan(d.toSec(), fplan);
	
	if(tokens[2] == "false")
			armFlag = false;

	if(tokens[0] == "person" && nfp)
		succes = JustinaTasks::dropObject(tokens[1], armFlag, 30000);
	else if(tokens[0] == "object" && nfp ){
		ss.str("");
		ss << "I am going to deliver the " << tokens[1];
		JustinaHRI::waitAfterSay(ss.str(), 2000);
		succes = JustinaTasks::placeObject(armFlag);
		(armFlag) ? JustinaManip::laGoTo("home", 6000) : JustinaManip::raGoTo("home", 6000);
	}
	
	if (succes)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;

	if(nfp) validateAttempsResponse(responseMsg);
}

void callbackUnknown(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command unknown ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	responseMsg.successful = 1;
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}


void callbackAskPerson(
		const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << testPrompt << "--------- Command ask for person ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	ros::Time finishPlan = ros::Time::now();
	ros::Duration d = finishPlan - beginPlan;
	std::cout << "TEST PARA MEDIR EL TIEMPO: " << d.toSec() << std::endl;
	
	bool success = ros::service::waitForService("spg_say", 5000);
	success = success
			& ros::service::waitForService("/planning_clips/confirmation",
					5000);
	JustinaManip::startHdGoTo(0, 0.0);
	if (success) {
		std::string to_spech = responseMsg.params;
		boost::replace_all(to_spech, "_", " ");
		std::stringstream ss;
		ss << "Hello, Tell me robot yes, or robot no in order to response my question, Well, Is your name, " << to_spech;
		//JustinaHRI::waitAfterSay(ss.str(), 1500);
		//ss << "Well, " << to_spech << " is your name";
		std::cout << "------------- to_spech: ------------------ " << ss.str() << std::endl;

		JustinaHRI::waitAfterSay(ss.str(), 10000);

		knowledge_msgs::planning_cmd srv;
		srv.request.name = "test_confirmation";
		srv.request.params = responseMsg.params;
		if (srvCltWaitConfirmation.call(srv)) {
			std::cout << "Response of confirmation:" << std::endl;
			std::cout << "Success:" << (long int) srv.response.success
					<< std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
			if (srv.response.success){
				ss.str("");
				ss << "Hello " << to_spech;
				JustinaHRI::waitAfterSay(ss.str(),1500);
				
			}
			else{
				ss.str("");
				ss << to_spech << ", I try to find you again ";
				JustinaHRI::waitAfterSay(ss.str(), 1500);
				boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
				JustinaNavigation::moveDistAngle(0, 1.57, 10000);
				boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
			}

			responseMsg.params = responseMsg.params;//srv.response.args;
			responseMsg.successful = srv.response.success;
		} else {
			std::cout << testPrompt << "Failed to call service of confirmation"
					<< std::endl;
			responseMsg.successful = 0;
			JustinaHRI::waitAfterSay("Repeate the command please", 2000);
		}
	} else {
		std::cout << testPrompt << "Needed services are not available :'("
				<< std::endl;
		responseMsg.successful = 0;
	}
	validateAttempsResponse(responseMsg);
	//command_response_pub.publish(responseMsg);
}

void callbackCmdTaskConfirmation( const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
    std::cout << testPrompt << "--------- Command confirm task -------" << std::endl;
    std::cout << "name: " << msg->name << std::endl;
    std::cout << "params: " << msg->params << std::endl;

    knowledge_msgs::PlanningCmdClips responseMsg;
    responseMsg.name = msg->name;
    responseMsg.params = msg->params;
    responseMsg.id = msg->id;
    
    bool success = ros::service::waitForService("spg_say", 5000);

    if (success) {

        std::string to_spech = responseMsg.params;
        boost::replace_all(to_spech, "_", " ");
        std::stringstream ss;
        std::string lastReco;

        ss << to_spech;
        std::cout << "------------- to_spech: ------------------ " << ss.str()
            << std::endl;
        JustinaHRI::enableGrammarSpeechRecognized("confirmation", 2.0);
        JustinaHRI::enableSpeechRecognized(false);
        JustinaHRI::waitAfterSay(ss.str(),5000);
        JustinaHRI::enableSpeechRecognized(true);

        JustinaHRI::waitForSpeechRecognized(lastReco,10000);
        responseMsg.params = "conf";
        if(lastReco == "robot yes" || lastReco == "justina yes"){
            responseMsg.successful = true;
        }
        else{
            responseMsg.successful = false;
            JustinaHRI::waitAfterSay("I am at your service",5000);
			JustinaNavigation::moveDistAngle(0, 1.57, 10000);
			boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
        }

    } else {
        std::cout << testPrompt << "Needed services are not available :'("
            << std::endl;
        responseMsg.successful = 0;
    }
    command_response_pub.publish(responseMsg);
}

/*void callbackRemindPerson(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command remind person ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	
    std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));
	std::stringstream ss;

    JustinaVision::startFaceRecognition(true);

    std_msgs::String person_name;
    person_name.data = tokens[0];

    for(int i=0; i < 15; i++){
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        //JustinaVision::facRecognize();
        train_face_pub.publish(person_name);
    }


    JustinaVision::startFaceRecognition(true);
	responseMsg.successful = 1;
	//validateAttempsResponse(responseMsg);
	command_response_pub.publish(responseMsg);
}*/

void callbackCmdObjectOnLocation(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command Find Objects on Location ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	
    std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));
	std::stringstream ss;
    
	std::vector<vision_msgs::VisionObject> recoObjForTake;
    	sensor_msgs::Image image;
	
	if(!JustinaTasks::alignWithTable(0.35)){
	    JustinaNavigation::moveDist(0.10, 3000);
	    if(!JustinaTasks::alignWithTable(0.35)){
		std::cout << "I can´t alignWithTable... :'(" << std::endl;
		JustinaNavigation::moveDist(-0.15, 3000);
	    }
	}
        recoObjForTake.clear();
	if(!JustinaVision::detectAllObjectsVot(recoObjForTake, image, 5)){
		std::cout << "I  can't detect anything" << std::endl;
	}else{
		for(int i = 0; i < recoObjForTake.size(); i++){
			ss.str("");
			ss << "(assert (update-object-on-location " << recoObjForTake[i].id << "1))";
			JustinaRepresentation::sendAndRunCLIPS(ss.str());
		}
	}

	responseMsg.successful = 1;
	//validateAttempsResponse(responseMsg);
	command_response_pub.publish(responseMsg);
}

void callbackCmdOfferDrink(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command offer drink to person ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	
    std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));
	std::stringstream ss;
    std::string lastReco;
    std::string lastInt;

    bool drink_conf = false;
    bool name_conf = false;
    std::string drink;
    std::string name;
	std::string query;
    int count  = 0;
    
    bool success = ros::service::waitForService("spg_say", 5000);
    //success = success & ros::service::waitForService("/planning_clips/confirmation", 5000);

    while(!drink_conf && count < 3){
        JustinaHRI::enableGrammarSpeechRecognized("order_drink", 2.0);
        JustinaHRI::enableSpeechRecognized(false);
        JustinaHRI::waitAfterSay("tell me what drink do you want",5000);
        JustinaHRI::enableSpeechRecognized(true);
        if(JustinaHRI::waitForSpeechRecognized(lastReco,10000)){
            if(JustinaRepresentation::stringInterpretation(lastReco, drink))
                std::cout << "last int: " << drink << std::endl;
                ss.str("");
                ss << "Do you want " << drink;
                count++;

		if (!alternative_drink && (no_drink == drink || prev_drink == drink)){
			count--;
			continue;
		}
                
                JustinaHRI::enableGrammarSpeechRecognized("confirmation", 2.0);
                JustinaHRI::enableSpeechRecognized(false);
                JustinaHRI::waitAfterSay(ss.str(),5000);
                JustinaHRI::enableSpeechRecognized(true);

                JustinaHRI::waitForSpeechRecognized(lastReco,10000);
                if(lastReco == "robot yes" || lastReco == "justina yes"){
                    drink_conf = true;
			if (alternative_drink){
				
				JustinaRepresentation::strQueryKDB(ss.str(), query, 1000);
				if (query != "on_the_bar"){
					ss.str("");
					ss << "I am sorry, the " << drink << " is out of stock , please choose another one";
					no_drink = drink;
					drink_conf = false;
					JustinaHRI::waitAfterSay(ss.str(), 2000);
					alternative_drink = false;
					count--;
				}
			}
		}
                //count++;
        }
    }
        count = 0;
        drink_conf = false;
        ss.str("");
	prev_drink = drink;
        ss << "Well a " << drink << " for you";
        JustinaHRI::waitAfterSay(ss.str(),5000);
    
    while(!drink_conf && count < 3){
        JustinaHRI::enableGrammarSpeechRecognized("people_names", 2.0);
        JustinaHRI::enableSpeechRecognized(false);
        JustinaHRI::waitAfterSay("tell me what is your name please",5000);
        JustinaHRI::enableSpeechRecognized(true);
        if(JustinaHRI::waitForSpeechRecognized(lastReco,10000)){
            if(JustinaRepresentation::stringInterpretation(lastReco, name))
                std::cout << "last int: " << name << std::endl;
                ss.str("");
                ss << "your name is " << name;
                JustinaHRI::enableGrammarSpeechRecognized("confirmation", 2.0);
                JustinaHRI::enableSpeechRecognized(false);
                JustinaHRI::waitAfterSay(ss.str(),5000);
                JustinaHRI::enableSpeechRecognized(true);
                
                JustinaHRI::waitForSpeechRecognized(lastReco,10000);
                if(lastReco == "robot yes" || lastReco == "justina yes")
                    drink_conf = true;
                count++;
        }
    }
        
        ss.str("");
        ss << "Ok " << name << " I will bring you a " << drink << " from the bar";
        JustinaHRI::waitAfterSay(ss.str(),5000);

    ss.str("");
    ss << "(assert (give-drink-to-person " << drink << " " << name << "))";
    JustinaRepresentation::sendAndRunCLIPS(ss.str());
    JustinaHRI::enableSpeechRecognized(false);

	responseMsg.successful = 1;
	//validateAttempsResponse(responseMsg);
	command_response_pub.publish(responseMsg);
}

void callbackCmdTrainPerson(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command  Train person ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	
    std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));
	std::stringstream ss;
    bool finish_train = false;
    
    boost::posix_time::ptime prev;
	boost::posix_time::ptime curr;
    
    JustinaHRI::waitAfterSay("please not move, and look at me", 6000);
    JustinaVision::faceTrain(tokens[0], 4);
    curr = boost::posix_time::second_clock::local_time();
    prev = curr;
    
    while(!finish_train){
        if((curr - prev).total_milliseconds() < TIMEOUT_MEMORIZING){
            if(JustinaVision::getLastTrainingResult() == 0)
                finish_train = true;
            curr = boost::posix_time::second_clock::local_time();
        }
    }
	
    JustinaNavigation::moveDistAngle(0, 1.57, 10000);
	boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
	
    responseMsg.successful = 1;
	//validateAttempsResponse(responseMsg);
	command_response_pub.publish(responseMsg);
}


void callbackCmdGetOrderObject(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command get objects from the guest orders ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	
    	std::vector<std::string> tokens;
    	std::vector<std::string> tokens1;
	std::string str = responseMsg.params;
	split(tokens1, str, is_any_of(" "));
	boost::replace_all(tokens1[0], "_", " ");
	split(tokens, tokens1[0], is_any_of(" "));
	std::stringstream ss;
    int attemps = 0;
	bool success = false;
    bool la = false;
    bool ra = false;

    while(!success && attemps<4){
        success = JustinaTasks::sayAndSyncNavigateToLoc(tokens1[tokens1.size() - 1], 120000);
        attemps++;
    }

   for (int i = 1; i < tokens.size()-1; i++){
        ss.str("");
        ss << "Barman please put the " << tokens[i] << " in my gripper";
        JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
        if(!ra){

            JustinaManip::raGoTo("navigation", 3000);
            JustinaTasks::detectObjectInGripper(tokens[i], false, 7000);
            ra = true;
            ss.str("");
            ss << "(assert (set_object_arm " << tokens[i] << " false))";
            JustinaRepresentation::sendAndRunCLIPS(ss.str());
        }
        if(!la && tokens.size() > 3){
            JustinaManip::laGoTo("navigation", 3000);
            JustinaTasks::detectObjectInGripper(tokens[i], false, 7000);
            la = true;
            ss << "(assert (set_object_arm " << tokens[i] << " true))";
            JustinaRepresentation::sendAndRunCLIPS(ss.str());
        }

   } 
	
    responseMsg.successful = 1;
	//validateAttempsResponse(responseMsg);
	command_response_pub.publish(responseMsg);
}

void callbackCmdDeliverOrder(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command deliver object ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	knowledge_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	
    std::vector<std::string> tokens;
    std::vector<std::string> tokens1;
	std::string str = responseMsg.params;
	split(tokens1, str, is_any_of(" "));
	boost::replace_all(tokens1[0], "_", " ");
	split(tokens, tokens1[0], is_any_of(" "));
	std::stringstream ss;
    
    std::string name;
    std::string arm;

    int attemps = 0;
	bool success = false;
    bool armFlag = false;

    while(!success && attemps<4){
        success = JustinaTasks::sayAndSyncNavigateToLoc(tokens1[tokens1.size()-1], 120000);
        attemps++;
    }
    success = false;
   for (int i = 1; i < tokens.size() -1 ; i++){
       ss.str("");
       ss << "(assert (get_person " << tokens[i] << "))";
       JustinaRepresentation::strQueryKDB(ss.str(), name, 1000);
       if(name != "None"){
            ss.str("");
            ss << name << " please look at me, I try to find you";
            JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
            while(!success && attemps <4){
    			success = JustinaTasks::findPerson(name, -1, JustinaTasks::NONE, true, tokens[tokens.size()-1]);
                attemps++;
            }
            if(success){
                ss.str("");
                ss << "(assert (get_arm " << tokens[i] << "))" ;
                JustinaRepresentation::strQueryKDB(ss.str(), arm, 1000);
                armFlag = (arm == "true") ? true : false;
		        success = JustinaTasks::dropObject(tokens[i], armFlag, 30000);
            }
            else{
                ss.str("");
                ss << "I am sorry, I cant find you " << name;
                JustinaHRI::waitAfterSay(ss.str(), 5000, 0);
            }
       }
   } 
	
    responseMsg.successful = 1;
	//validateAttempsResponse(responseMsg);
	command_response_pub.publish(responseMsg);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "gpsr_test");
	ros::NodeHandle n;
	ros::Rate rate(10);

	srvCltGetTasks = n.serviceClient<knowledge_msgs::planning_cmd>("/planning_clips/get_task");
	srvCltInterpreter = n.serviceClient<knowledge_msgs::planning_cmd>("/planning_clips/spr_interpreter");
	srvCltWaitConfirmation = n.serviceClient<knowledge_msgs::planning_cmd>("/planning_clips/confirmation");
	srvCltWaitForCommand = n.serviceClient<knowledge_msgs::planning_cmd>("/planning_clips/wait_command");
	srvCltAnswer = n.serviceClient<knowledge_msgs::planning_cmd>("/planning_clips/answer");
	srvCltAskName = n.serviceClient<knowledge_msgs::planning_cmd>("/planning_clips/ask_name");
	srvCltAskIncomplete = n.serviceClient<knowledge_msgs::planning_cmd>("/planning_clips/ask_incomplete");
	srvCltQueryKDB = n.serviceClient<knowledge_msgs::StrQueryKDB>("/planning_clips/str_query_KDB");

	ros::Subscriber subCmdSpeech = n.subscribe("/planning_clips/cmd_speech", 1, callbackCmdSpeech);

	ros::Subscriber subCmdNavigation = n.subscribe("/planning_clips/cmd_goto", 1, callbackCmdNavigation);
	ros::Subscriber subCmdFindObject = n.subscribe("/planning_clips/cmd_find_object", 1, callbackCmdFindObject);
	ros::Subscriber subCmdMoveActuator = n.subscribe("/planning_clips/cmd_move_actuator", 1, callbackMoveActuator);
	ros::Subscriber subCmdUnknown = n.subscribe("/planning_clips/cmd_unknown", 1, callbackUnknown);
	ros::Subscriber subAskPerson = n.subscribe("/planning_clips/cmd_ask_person", 1, callbackAskPerson);
	ros::Subscriber subSpeechGenerator = n.subscribe("/planning_clips/cmd_speech_generator", 1, callbackCmdSpeechGenerator);
    ros::Subscriber subCmdTaskConfirmation = n.subscribe("/planning_clips/cmd_task_conf", 1, callbackCmdTaskConfirmation);
    ros::Subscriber subCmdOfferDrink = n.subscribe("/planning_clips/cmd_offer_drink", 1, callbackCmdOfferDrink);
    ros::Subscriber subCmdTrainPerson = n.subscribe("/planning_clips/cmd_train_person", 1, callbackCmdTrainPerson);
    ros::Subscriber subCmdGetOrderObject = n.subscribe("/planning_clips/cmd_get_order_object", 1, callbackCmdGetOrderObject);
    ros::Subscriber subCmdDeliverOrder = n.subscribe("/planning_clips/cmd_deliver_order", 1, callbackCmdDeliverOrder);
    ros::Subscriber subCmdObjectOnLocation = n.subscribe("/planning_clips/cmd_object_on_location", 1, callbackCmdObjectOnLocation);

	command_response_pub = n.advertise<knowledge_msgs::PlanningCmdClips>("/planning_clips/command_response", 1);
    sendAndRunClips_pub = n.advertise<std_msgs::String>("/planning_clips/command_sendAndRunCLIPS", 1);
    train_face_pub = n.advertise<std_msgs::String>("/vision/face_recognizer/run_face_trainer", 1);

    pubStartTime = n.advertise<std_msgs::Int32>("/planning/start_time", 1); 
    pubResetTime = n.advertise<std_msgs::Empty>("/planning/restart_time", 1);

	JustinaHRI::setNodeHandle(&n);
	JustinaHardware::setNodeHandle(&n);
	JustinaKnowledge::setNodeHandle(&n);
	JustinaManip::setNodeHandle(&n);
	JustinaNavigation::setNodeHandle(&n);
	JustinaTasks::setNodeHandle(&n);
	JustinaTools::setNodeHandle(&n);
	JustinaVision::setNodeHandle(&n);
	JustinaRepresentation::setNodeHandle(&n);
    JustinaHRI::usePocketSphinx = true;
	
	JustinaRepresentation::initKDB("", false, 20000);
    JustinaRepresentation::initKDB("/serving_drinks/serving_drinks.dat", false, 20000);

	/*if (argc > 3){
		std::cout << "FPLAN FLAG: " << argv[3] << std::endl;
		fplan = atoi(argv[3]);
		maxTime = atof(argv[4]);
		cat_grammar = argv[5];
		std::cout << "FPLAN FLAG: " << fplan << std::endl;
		std::cout << "MAX TIME: " << maxTime << std::endl;
		std::cout << "Grammar: " << cat_grammar << std::endl;}*/
    fplan = false;
    maxTime = 60.0; 
    idsPerson.push_back("person");

	while (ros::ok()) {

		switch (state) {
		case SM_INIT:
			if (startSignalSM) {
				JustinaHRI::waitAfterSay("I am ready for the serving drinks test", 4000);
                JustinaHRI::loadGrammarSpeechRecognized("people_names", "/grammars/pre_guadalajara/people_names.jsgf");
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                JustinaHRI::loadGrammarSpeechRecognized("order_drink", "/grammars/pre_guadalajara/order_drink.jsgf");
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                JustinaHRI::loadGrammarSpeechRecognized("confirmation", "/grammars/pre_sydney/restaurant/confirmation.jsgf");
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                JustinaHRI::enableSpeechRecognized(false);
				state = SM_SCRIPT;
			}
			break;
		case SM_SAY_WAIT_FOR_DOOR:
			JustinaHRI::waitAfterSay("I am waiting for the door to be open",
					4000);
			state = SM_WAIT_FOR_DOOR;
			break;
        case SM_SCRIPT:
            initSpeech.successful = false;
            runSMCLIPS = true;
            
            initSpeech.name = "cmd_set_task";
            initSpeech.id = 10;

            initSpeech.params = "plan-1021 1";
            initSpeech.successful = true;
            command_response_pub.publish(initSpeech);
            boost::this_thread::sleep(boost::posix_time::milliseconds(400));
            ros::spinOnce();

            initSpeech.params = "robot serving_drinks bar living_room 1";
            initSpeech.successful = true;
            command_response_pub.publish(initSpeech);
            boost::this_thread::sleep(boost::posix_time::milliseconds(400));
            ros::spinOnce();

            initSpeech.params = "Final";
            initSpeech.successful = false;
            command_response_pub.publish(initSpeech);
            boost::this_thread::sleep(boost::posix_time::milliseconds(400));
            ros::spinOnce();
            
            state = SM_WAIT_CLIPS; 
            break;
        case SM_WAIT_CLIPS:
            break;
		}

		rate.sleep();
		ros::spinOnce();
	}

	JustinaVision::stopQRReader();

	return 0;

}

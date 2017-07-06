#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaKnowledge.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"
#include "std_msgs/Bool.h"
//#include "string"

#define SM_INIT 0
#define SM_INSTRUCTIONS 5
#define SM_WAIT_FOR_OPERATOR 10
#define SM_MEMORIZING_OPERATOR 20
#define SM_WAIT_FOR_LEGS_FOUND 25
#define SM_FOLLOWING_PHASE 30
#define SM_BRING_GROCERIES 40
#define SM_BRING_GROCERIES_CONF 41
#define SM_BRING_GROCERIES_TAKE 42
#define SM_BAG_DELIVERY 50
#define SM_BAG_DELIVERY_PLACE 60
#define SM_LOOKING_HELP 70
#define SM_GUIDING_ASK 75
#define SM_GUIDING_HELP 80
#define SM_GUIDING_MEMORIZING_OPERATOR 90
#define SM_GUIDING_MEMORIZING_OPERATOR_ELF 91
#define SM_GUIDING_MEMORIZING_OPERATOR_SAY 92
#define SM_GUIDING_PHASE 100
#define SM_GUIDING_STOP 101
#define SM_GUIDING_CAR 102
#define SM_FINAL_STATE 110

#define MAX_ATTEMPTS_RECOG 3
#define MAX_ATTEMPTS_CONF 3

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING HELP ME CARRY TEST..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaHardware::setNodeHandle(&n);
    JustinaHRI::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaTools::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    JustinaTasks::setNodeHandle(&n);
    ros::Rate loop(10);

    boost::posix_time::ptime prev;
    boost::posix_time::ptime curr;

    //int c_point=0,i=1;
    int nextState = 0;
    bool fail = false;
    bool success = false;
    float x, y ,z;
    std::stringstream ss;
    std::vector<std::string> tokens;
    int attemptsRecogLoc = 0;
    int attemptsConfLoc = 0;

    std::string lastRecoSpeech;
    std::string location;
    std::vector<std::string> validCommandsStop;
    std::vector<std::string> validCommandsTake;
    validCommandsStop.push_back("here is the car");
    validCommandsStop.push_back("stop follow me");
    //places
    validCommandsTake.push_back("take this bag to the sofa");
    validCommandsTake.push_back("take this bag to the kitchen");
    validCommandsTake.push_back("take this bag to the bed");
    validCommandsTake.push_back("take this bag to the bedroom table");
    validCommandsTake.push_back("take this bag to the dinner table");
    validCommandsTake.push_back("take this bag to the shelf");
    validCommandsTake.push_back("take this bag to the bookcase");
    validCommandsTake.push_back("take this bag to the cabinet");
    validCommandsTake.push_back("take this bag to the t.v.	|");
    validCommandsTake.push_back("take this bag to the fridge");
    validCommandsTake.push_back("take this bag to the stove");
    validCommandsTake.push_back("get this bag to the sofa");
    validCommandsTake.push_back("get this bag to the kitchen");
    validCommandsTake.push_back("get this bag to the bed");
    validCommandsTake.push_back("get this bag to the bedroom table");
    validCommandsTake.push_back("get this bag to the dinner table");
    validCommandsTake.push_back("get this bag to the shelf");
    validCommandsTake.push_back("get this bag to the bookcase");
    validCommandsTake.push_back("get this bag to the cabinet");
    validCommandsTake.push_back("get this bag to the t.v.	|");
    validCommandsTake.push_back("get this bag to the fridge");
    validCommandsTake.push_back("get this bag to the stove");
    //validCommands.push_back("return home");
    //validCommands.push_back("help me");
    //validCommands.push_back("robot no");

    bool hokuyoRear = false;
    bool userConfirmation = false;
    bool alig_to_place=true;
    int cont_z=0;

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {

            case SM_INIT:

                std::cout << "State machine: SM_INIT" << std::endl;	
                JustinaHRI::waitAfterSay("I am ready for the help me carry test", 2000);
                nextState = SM_INSTRUCTIONS;

                break;

            case SM_INSTRUCTIONS:
                std::cout << "State machine: SM_INSTRUCTIONS" << std::endl;
                JustinaHRI::waitAfterSay("Tell me, here is the car, when we reached the car location", 10000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                JustinaHRI::waitAfterSay("please tell me robot yes for confirm the command", 10000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                JustinaHRI::waitAfterSay("please tell me robot no for repeat the command", 10000);                
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                JustinaHRI::waitAfterSay("Please, tell me, follow me, for start following you", 3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                cont_z=0;
                nextState=SM_WAIT_FOR_OPERATOR;
                break;    

            case SM_WAIT_FOR_OPERATOR:

                std::cout << "State machine: SM_WAIT_FOR_OPERATOR" << std::endl;
                
                if(JustinaHRI::waitForSpecificSentence("follow me" , 15000))
                    nextState = SM_MEMORIZING_OPERATOR;
                else{
                    nextState = SM_WAIT_FOR_OPERATOR;
                    cont_z++;    		
                }
                if(cont_z>3){
                    JustinaHRI::say("Please repeat the command");
                    cont_z=0;
                }
                break;

            case SM_MEMORIZING_OPERATOR:

                std::cout << "State machine: SM_MEMORIZING_OPERATOR" << std::endl;
                JustinaHRI::waitAfterSay("Human, please put in front of me", 2500);
                JustinaHRI::enableLegFinder(true);
                nextState=SM_WAIT_FOR_LEGS_FOUND;	    
                break;

            case SM_WAIT_FOR_LEGS_FOUND:

                std::cout << "State machine: SM_WAIT_FOR_LEGS_FOUND" << std::endl;
                if(JustinaHRI::frontalLegsFound()){
                    std::cout << "NavigTest.->Frontal legs found!" << std::endl;
                    JustinaHRI::waitAfterSay("I found you, i will start to follow you human, please walk. ", 10000);
                    JustinaHRI::startFollowHuman();
                    nextState = SM_FOLLOWING_PHASE;
                }


                break;

            case SM_FOLLOWING_PHASE:

                std::cout << "State machine: SM_FOLLOWING_PHASE" << std::endl;

                if(JustinaHRI::waitForSpecificSentence(validCommandsStop, lastRecoSpeech, 7000)){
                    if(lastRecoSpeech.find("here is the car") != std::string::npos || lastRecoSpeech.find("stop follow me") != std::string::npos){
                        JustinaHRI::waitAfterSay("is the car location", 4500);
                        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                    	JustinaHRI::waitForUserConfirmation(userConfirmation, 15000);
                		if(userConfirmation){
                    		JustinaHRI::stopFollowHuman();
	                        JustinaHRI::enableLegFinder(false);
	                        JustinaKnowledge::addUpdateKnownLoc("car_location");	
	                        JustinaHRI::waitAfterSay("I stopped", 1500);
	                        nextState = SM_BRING_GROCERIES;
                            cont_z=0;
	                        break;
                			}

                		else 
                    		JustinaHRI::waitAfterSay("Ok, please walk. ", 10000);
                		
                    }
                }
                if(!JustinaHRI::frontalLegsFound()){
                    std::cout << "State machine: SM_FOLLOWING_PHASE -> Lost human!" << std::endl;
                    JustinaHRI::waitAfterSay("I lost you, please put in front of me again", 1500);
                }        

                break;

            case SM_BRING_GROCERIES:
                std::cout << "State machine: SM_BRING_GROCERIES" << std::endl; 
                if(cont_z==0){
                    JustinaHRI::waitAfterSay("I am ready to help you, Please tell me, take this bag to some location", 4500);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                }
                if(JustinaHRI::waitForSpecificSentence(validCommandsTake, lastRecoSpeech, 7000)){
                    attemptsRecogLoc++;
                    if(lastRecoSpeech.find("this bag to the sofa") != std::string::npos){
                        location = "sofa";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    else if(lastRecoSpeech.find("this bag to the bed") != std::string::npos){
                        location = "bed";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    else if(lastRecoSpeech.find("this bag to the bedroom") != std::string::npos){
                        location = "bedroom_table";
                        alig_to_place=true;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    else if(lastRecoSpeech.find("this bag to the bedroom table") != std::string::npos){
                        location = "bedroom_table";
                        alig_to_place=true;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    else if(lastRecoSpeech.find("this bag to the dinning room") != std::string::npos){
                        location = "dinner_table";
                        alig_to_place=true;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    else if(lastRecoSpeech.find("this bag to the dinner table") != std::string::npos){
                        location = "dinner_table";
                        alig_to_place=true;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    else if(lastRecoSpeech.find("this bag to the shelf") != std::string::npos){
                        location = "shelf";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    else if(lastRecoSpeech.find("this bag to the bookcase") != std::string::npos){
                        location = "bookcase";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    else if(lastRecoSpeech.find("this bag to the cabinet") != std::string::npos){
                        location = "cabinet";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    else if(lastRecoSpeech.find("this bag to the t.v.") != std::string::npos){
                        location = "tv";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    else if(lastRecoSpeech.find("this bag to the fridge") != std::string::npos){
                        location = "fridge";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    else if(lastRecoSpeech.find("this bag to the stove") != std::string::npos){
                        location = "stove";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    else if(lastRecoSpeech.find("this bag to the hall") != std::string::npos){
                        location = "bookcase";
                        alig_to_place=false;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    else if(lastRecoSpeech.find("this bag to the kitchen") != std::string::npos){
                        location = "kitchen_table";
                        alig_to_place=true;
                        nextState=SM_BRING_GROCERIES_CONF;
                    }
                    else if(attemptsRecogLoc >= MAX_ATTEMPTS_RECOG){
                        location = "kitchen_table";
                        alig_to_place=true;
                        nextState = SM_BRING_GROCERIES_TAKE;
                    } 
                    if(location.compare("") != 0 && nextState == SM_BRING_GROCERIES_CONF){
                        ss.str("");
                        ss << "Do you want me take this bag to the "; 
                        tokens.clear();
                        boost::algorithm::split(tokens, location, boost::algorithm::is_any_of("_"));
                        for(int i = 0; i < tokens.size(); i++)
                            ss << tokens[i] << " ";
                        JustinaHRI::waitAfterSay(ss.str(), 5000);
                    }
                    
                }
                break;
            
            case SM_BRING_GROCERIES_CONF:
                std::cout << "State machine: SM_BRING_GROCERIES_CONF" << std::endl;
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                JustinaHRI::waitForUserConfirmation(userConfirmation, 15000);
                attemptsConfLoc++;
                if(userConfirmation)
                    nextState = SM_BRING_GROCERIES_TAKE;
                else if(attemptsConfLoc < MAX_ATTEMPTS_CONF){ 
                    nextState = SM_BRING_GROCERIES;
                    cont_z=0;
                }
                else
                    nextState = SM_BRING_GROCERIES_TAKE;
                break;

            case SM_BRING_GROCERIES_TAKE:    
                std::cout << "State machine: SM_BRING_GROCERIES_TAKE" << std::endl;
                JustinaHRI::waitAfterSay("Please, wait", 4500);
                JustinaManip::laGoTo("take", 4000);
                JustinaManip::startLaOpenGripper(0.6);
                JustinaManip::hdGoTo(0, -0.9, 3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                JustinaManip::getLeftHandPosition(x, y, z);
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                std::cout << "helMeCarry.->Point(" << x << "," << y << "," << z << ")" << std::endl;
                JustinaVision::startHandDetectBB(x, y, z);
                prev = boost::posix_time::second_clock::local_time();
                curr = prev;
                JustinaHRI::waitAfterSay("Please put the bag in my hand", 3000);
                while(ros::ok() && !JustinaVision::getDetectionHandBB() && (curr - prev).total_milliseconds() < 30000){
                    loop.sleep();
                    ros::spinOnce();
                    curr = boost::posix_time::second_clock::local_time();
                }
                JustinaVision::stopHandDetectBB();
                JustinaHRI::waitAfterSay("Thank you", 1500);
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                JustinaManip::startLaCloseGripper(0.4);
                JustinaManip::laGoTo("navigation", 10000);

                
                ss.str("");
                ss << "Ok human, I will go to the "; 
                tokens.clear();
                boost::algorithm::split(tokens, location, boost::algorithm::is_any_of("_"));
                for(int i = 0; i < tokens.size(); i++)
                    ss << tokens[i] << " ";
                ss << "and i will be back to the car";
                JustinaHRI::waitAfterSay(ss.str(), 5000);
                nextState=SM_BAG_DELIVERY;     
                break;

            case SM_BAG_DELIVERY:
                std::cout << "State machine: SM_BAG_DELIVERY" << std::endl;
                std::cout << "Location -> " << location << std::endl;
                if(!JustinaNavigation::getClose(location, 200000))
                	if(!JustinaNavigation::getClose(location, 200000))
                		JustinaNavigation::getClose(location, 200000);
                JustinaHRI::waitAfterSay("I arrived", 2000);
                nextState=SM_BAG_DELIVERY_PLACE;

                break;

            case SM_BAG_DELIVERY_PLACE:
                std::cout << "State machine: SM_BAG_DELIVERY_PLACE" << std::endl;
                JustinaHRI::waitAfterSay("I will delivery the bags", 3000);
                if(alig_to_place==true){
                    if(!JustinaTasks::alignWithTable(0.35)){
                        JustinaNavigation::moveDist(0.15, 3000);
                        JustinaTasks::alignWithTable(0.35);                
                    }
                }    

                if(!JustinaTasks::placeObject(true, 0.35, true))
                    if(!JustinaTasks::placeObject(true, 0.35, true))
                        JustinaTasks::placeObject(true, 0.35, true);

                nextState=SM_LOOKING_HELP;

                break;

            case SM_LOOKING_HELP:
                std::cout << "State machine: SM_LOOKING_HELP" << std::endl;
                JustinaHRI::waitAfterSay("I will look for help", 3000);
                if(JustinaTasks::findPerson())
                    nextState=SM_GUIDING_ASK;
                else
                    JustinaHRI::waitAfterSay("I did not find anyone", 3000);   
                break;

            case SM_GUIDING_ASK:
                std::cout << "State machine: SM_GUIDING_ASK" << std::endl;
                JustinaHRI::waitAfterSay("Human, can you help me bring some bags please", 5000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
                JustinaHRI::waitForUserConfirmation(userConfirmation, 15000);
                if(userConfirmation)
                    nextState = SM_GUIDING_MEMORIZING_OPERATOR_SAY;
                else {
                    nextState = SM_LOOKING_HELP;
                    JustinaNavigation::moveDistAngle(0.0, 1.5708, 10000);
                }	    

                break;        

            case SM_GUIDING_MEMORIZING_OPERATOR_SAY:
                std::cout << "State machine: SM_GUIDING_MEMORIZING_OPERATOR_SAY" << std::endl;
                JustinaHRI::waitAfterSay("I will guide you to the car location", 4000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                JustinaNavigation::moveDistAngle(0.0, 3.14159, 10000);
                JustinaHRI::waitAfterSay("Please, stand behind me", 3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                location="car_location";
                cont_z=0;
                nextState=SM_GUIDING_MEMORIZING_OPERATOR_ELF;
                
                break;

            case SM_GUIDING_MEMORIZING_OPERATOR_ELF:
                std::cout << "State machine: SM_GUIDING_MEMORIZING_OPERATOR_ELF" << std::endl;
                JustinaHRI::enableLegFinderRear(true); ////igcdkjgdhghksd
                nextState = SM_GUIDING_MEMORIZING_OPERATOR;

                break;

            case SM_GUIDING_MEMORIZING_OPERATOR:
                std::cout << "State machine: SM_GUIDING_MEMORIZING_OPERATOR" << std::endl;
                hokuyoRear = JustinaHRI::rearLegsFound();
                if(hokuyoRear){
                    JustinaHRI::waitAfterSay("Ok, let us go", 2500);
                    nextState=SM_GUIDING_PHASE;
                    JustinaNavigation::startGetClose(location);
                    cont_z=0;
                }
                else{
                    if(cont_z>3){
                    JustinaHRI::waitAfterSay("Human, stand behind me", 3000);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                    cont_z=0;
                    }
                    z++;
                }
                
                break;    

            case SM_GUIDING_PHASE:
                std::cout << "State machine: SM_GUIDING_PHASE" << std::endl;
                std::cout << "Location -> " << location << std::endl;
                hokuyoRear = JustinaHRI::rearLegsFound();
                if(!hokuyoRear)
                    nextState=SM_GUIDING_STOP;
                else{
                    z=0;
                }

                if(JustinaNavigation::isGlobalGoalReached())
                    nextState=SM_GUIDING_CAR;

                break;

            case SM_GUIDING_STOP:
                std::cout << "State machine: SM_GUIDING_STOP" << std::endl;
                if(cont_z>3){
                    
                    JustinaHardware::stopRobot();
                    JustinaHardware::stopRobot();
                    JustinaHardware::stopRobot();
                    ros::spinOnce();
                    JustinaHRI::waitAfterSay("I lost you", 1500);
                    JustinaHRI::enableLegFinderRear(false);
                    JustinaHRI::waitAfterSay("Human, stand behind me", 3000);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
                    cont_z=0;
                    nextState=SM_GUIDING_MEMORIZING_OPERATOR_ELF;
                }
                
                else{
                    nextState=SM_GUIDING_PHASE;
                }

                break;

            case SM_GUIDING_CAR:
                std::cout << "State machine: SM_GUIDING_CAR" << std::endl;
                JustinaHRI::waitAfterSay("Here is the car, please help us", 2500);
                JustinaHRI::enableLegFinderRear(false);
                nextState=SM_FINAL_STATE;    
                break;

            case SM_FINAL_STATE:
                std::cout << "State machine: SM_FINAL_STATE" << std::endl;
                success = true;
                break;

        }
        
        ros::spinOnce();
        loop.sleep();
    }



    return 1;
}


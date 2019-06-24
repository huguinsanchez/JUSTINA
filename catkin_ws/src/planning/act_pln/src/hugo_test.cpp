#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaKnowledge.h"
#include <iostream>
#include <fstream>


#include <sstream>

int attempt=0;

std::string NumberToString ( int Number ){
     std::ostringstream ss;
     ss << Number;
     return ss.str();
  }

std::string sentence(std::string s){

    return  NumberToString(attempt)+","+s+"\n";
}


int main(int argc, char** argv){

    std::cout << "test" << std::endl;
    ros::init(argc, argv, "hugo_test");
    ros::NodeHandle n;
    JustinaVision::setNodeHandle(&n);
    JustinaKnowledge::setNodeHandle(&n);
    ros::Rate loop(.3);
    std::string s="";
    std::string object="";
    std::ofstream myfile;
    std::vector<vision_msgs::VisionObject> recoObjList;
    float robot_x,robot_y,robot_a;
    std::string room="";
    

   /* myfile.open ("/home/pumas/Desktop/data_hugo/cutting_board.csv",std::ios::app|std::ios::out);

    if(!myfile.is_open()){
        s="q";
        std::cout<<"Error openning the file";
    }
    myfile.flush();*/
    while(ros::ok() && s!="q"){

        //attempt++;
        if(JustinaVision::detectObjectsGCM(recoObjList,"kitchen",false)){
            object=recoObjList[0].id;
        }
        else{
            object="no detected";
        }
        std::cout<<"obj_reco.id: "<<object<<std::endl;
        //JustinaKnowledge::getRobotPose(robot_x,robot_y,robot_a);
        //room=JustinaKnowledge::getRoomOfPoint(robot_x, robot_y);
        /*if(JustinaKnowledge::isPointInKnownArea(robot_x, robot_y, "office"))
            std::cout<<"true"<<std::endl;
        else
            std::cout<<"false"<<std::endl;*/
        //JustinaKnowledge::getRobotPoseRoom(room);
        /*if(room!="")
            std::cout<<"location: "<<room<<std::endl;
        else
            std::cout<<"Error "<<std::endl;*/

        //JustinaVision::detectObjectsGCM(recoObjList,"bedroom",false);
        /*std::cout<<sentence(object)<<std::endl;
        myfile << sentence(object);
        myfile.flush();*/
        ros::spinOnce();
        loop.sleep();
        /*std::cout << "Please enter q to quit: ";
        std::cin >> s;*/
    }
    //myfile.close();
    return 0;
}
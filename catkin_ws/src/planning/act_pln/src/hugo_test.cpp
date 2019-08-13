#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaKnowledge.h"
#include <iostream>
#include <fstream>
#include <utility>

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
    double prob;
    std::map<std::string, std::vector<float> > locations;
    std::vector<std::string> rooms;
    std::vector<double> probs;

   /* myfile.open ("/home/pumas/Desktop/data_hugo/cutting_board.csv",std::ios::app|std::ios::out);

    if(!myfile.is_open()){
        s="q";
        std::cout<<"Error openning the file";
    }
    myfile.flush();*/
    while(ros::ok() && s!="q"){

        JustinaKnowledge::getRobotPoseRoom(room);
        if(room!=""){
            std::cout<<"location: "<<room<<std::endl;
            //JustinaKnowledge::getProbOfBeingRoom("kitchen",prob);
            //std::cout<<room<<": "<<prob<<std::endl;
        }
        else
            std::cout<<"Error "<<std::endl;
        /*JustinaKnowledge::getProbOfBeingRoom("kitchen",prob);
        std::cout<<"kitchen: "<<prob<<std::endl;
        JustinaKnowledge::getProbOfBeingRoom("office",prob);
        std::cout<<"office: "<<prob<<std::endl;
        JustinaKnowledge::getProbOfBeingRoom("arena",prob);
        std::cout<<"arena: "<<prob<<std::endl;*/
        JustinaKnowledge::getAllRooms(rooms);
        std::cout <<"calculatin, probs " <<"\n";
        JustinaKnowledge::getProbOfBeingRoom(rooms, probs);
        for(size_t i=0; i<rooms.size();i++){
            std::cout<<rooms[i]<<" : "<<probs[i]<<std::endl;

        }
        /*JustinaKnowledge::getKnownLocations(locations);

        for(auto it = locations.cbegin(); it != locations.cend(); ++it){
            std::cout << it->first << " " << it->second<<"\n";
        }*/

        if(JustinaVision::detectObjectsGCM(recoObjList,rooms,probs,room,false)){
            object=recoObjList[0].id;
        }
        else{
            object="no detected";
        }
        std::cout<<"obj_reco.id: "<<object<<std::endl;

        ros::spinOnce();
        loop.sleep();
    }
    //myfile.close();
    return 0;
}
#include <iostream>
#include "justina_tools/JustinaTasks.h"

#define SM_INIT 3

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST JUST FOR TEST BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaTasks::setNodeHandle(&n);
    ros::Rate loop(10);

    int nextState = SM_INIT;
    bool fail = false;
    bool success = false;

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {
        case SM_INIT:
            JustinaTasks::graspNearestObjectLeftArm();
            break;
        default:
            std::cout << "NavigTest.->Somebody very stupid programmed this shit. " << std::endl;
            fail = true;
            break;
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}

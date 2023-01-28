//
// Created by aung on 2023/1/5.
//
#include "rm_fsm/StateMachine.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sm");
    ros::NodeHandle nh("~");
    StateMachine sm(nh);

    ros::Rate loop_rate(500);


    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        sm.processState();
        if (sm.hp>=0) sm.hp--;
        else sm.hp=1000;
    }
    return 0;
}

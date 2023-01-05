//
// Created by aung on 2023/1/5.
//
#include "chassis_controller/EasyStateMachine.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "es_fsm");
    ros::NodeHandle nh("~");
    EasyStateMachine sm(nh);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        sm.processState();
    }
    return 0;
}

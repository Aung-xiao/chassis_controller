//
// Created by aung on 2022/11/11.
//

#include "ros_manual/manual_base.h"

int main(int argc, char** argv)
{
    std::string robot;
    ros_manual::ManualBase* manual_control;
    ros::init(argc, argv, "ros_manual");
    ros::NodeHandle nh("~");
    robot = "standard";
    if (robot == "standard")
        manual_control = new ros_manual::ManualBase(nh);
    else
    {
        ROS_ERROR("no robot type ");
        return 0;
    }
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        manual_control->run();
        loop_rate.sleep();
    }
    return 0;
}

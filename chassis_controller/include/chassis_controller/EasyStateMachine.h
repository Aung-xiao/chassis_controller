//
// Created by aung on 2023/1/4.
//

#ifndef SRC_EASYSTATEMACHINE_H
#define SRC_EASYSTATEMACHINE_H

#include "geometry_msgs/Twist.h"
#include <ros/ros.h>
#include "EasyStateMachine_fsm.h"

class EasyStateMachine
{
public:
    EasyStateMachine(ros::NodeHandle& nh);
    void move_forward();
    void move_back();
    void processState();
    bool car_state();
    float hp;
    void car_echo();

private:

    ros::Publisher vel_pub_;
    geometry_msgs::Twist chassis_vel_msgs_;
    EasyStateMachine_fsm context_;
};
#endif //SRC_EASYSTATEMACHINE_H

//
// Created by aung on 2023/1/4.
//

#ifndef SRC_EASYSTATEMACHINE_H
#define SRC_EASYSTATEMACHINE_H

#include "geometry_msgs/Twist.h"
#include <ros/ros.h>
#include "EasyStateMachine_sm.h"

class EasyStateMachine
{
    EasyStateMachine_sm _fsm;
public:
    void hp_reduce();
    void hp_increase();
    void move_forward();
    void move_back();
private:
    ros::Publisher vel_pub_;
    geometry_msgs::Twist chassis_vel_msgs_;
    float hp=1000.0;
};
#endif //SRC_EASYSTATEMACHINE_H

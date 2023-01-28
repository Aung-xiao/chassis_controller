//
// Created by aung on 2023/1/4.
//

#ifndef SRC_STATEMACHINE_H
#define SRC_STATEMACHINE_H

#include "geometry_msgs/Twist.h"
#include <ros/ros.h>
#include "StateMachine_sm.h"

class StateMachine
{
public:
    StateMachine(ros::NodeHandle& nh);
    void move_forward();
    void move_back();
    void processState();
    bool car_state();
    float hp;
    void car_echo();

private:

    ros::Publisher vel_pub_;
    geometry_msgs::Twist chassis_vel_msgs_;
    StateMachine_sm context_;
};
#endif //SRC_STATEMACHINE_H

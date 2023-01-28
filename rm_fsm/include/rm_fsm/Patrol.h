//
// Created by aung on 2023/1/28.
//

#ifndef PATROL_H
#define PATROL_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "Patrol_sm.h"
class Patrol {
public:
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    Patrol(ros::NodeHandle &nh, MoveBaseClient &ac);
    void move2Point1();
    void move2Point2();
    void move2Point3();
    void move2Point4();
    void processState();
    int getState();

private:
    MoveBaseClient ac;
    Patrol_sm context_;
    move_base_msgs::MoveBaseGoal goal;
    int next_point;
};


#endif //PATROL_H

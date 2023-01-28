//
// Created by aung on 2023/1/4.
//

#include <rm_fsm/Patrol.h>
Patrol::Patrol(ros::NodeHandle &nh, MoveBaseClient &ac) : context_(*this),ac("move_base",true){
    next_point=1;
    context_.enterStartState();
    goal.target_pose.header.frame_id="map";
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
}
int Patrol::getState(){
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Reach goal point");
        if(next_point==5) next_point=1;
        else next_point++;
        return next_point;
    }
    else{
        ROS_INFO("The base failed to move forward  for some reason");
    }
}
void Patrol::processState() {
    context_.processState();
}
void Patrol::move2Point1() {
    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal1");
    ac.sendGoal(goal);
}
void Patrol::move2Point2() {
    goal.target_pose.pose.position.y = 1.0;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal2");
    ac.sendGoal(goal);
}
void Patrol::move2Point3() {
    goal.target_pose.pose.position.x = 2.0;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal3");
    ac.sendGoal(goal);
}
void Patrol::move2Point4() {
    goal.target_pose.pose.position.y = 2.0;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal4");
    ac.sendGoal(goal);
}
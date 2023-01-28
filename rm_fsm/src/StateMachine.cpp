//
// Created by aung on 2023/1/4.
//

#include <rm_fsm/StateMachine.h>
StateMachine::StateMachine(ros::NodeHandle& nh) : context_(*this){
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    hp=1000;
    context_.enterStartState();
}

void StateMachine::processState(){
    context_.processState();
}
void StateMachine::move_forward() {
    ROS_INFO("i am in Safe state");
    chassis_vel_msgs_.linear.x=0.5;
    vel_pub_.publish(chassis_vel_msgs_);
}
void StateMachine::move_back() {
    ROS_INFO("i am in Danger state");
    chassis_vel_msgs_.linear.x=-0.5;
    vel_pub_.publish(chassis_vel_msgs_);
}
bool StateMachine::car_state() {
    return hp>=500;
}
void StateMachine::car_echo() {
    ROS_INFO("i am in Default state");
}
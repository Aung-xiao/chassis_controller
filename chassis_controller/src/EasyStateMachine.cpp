//
// Created by aung on 2023/1/4.
//

#include <chassis_controller/EasyStateMachine.h>
EasyStateMachine::EasyStateMachine(ros::NodeHandle& nh) : context_(*this){
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    context_.enterStartState();
    hp=1000;
}

void EasyStateMachine::processState(){
    context_.processState();
}
void EasyStateMachine::hp_increase() {
    hp+=1;
    ROS_INFO("current hp:%f",hp);
}
void EasyStateMachine::hp_reduce() {
    hp-=1;
    ROS_INFO("current hp:%f",hp);
}
void EasyStateMachine::move_forward() {
    chassis_vel_msgs_.linear.x=0.5;
    vel_pub_.publish(chassis_vel_msgs_);
}
void EasyStateMachine::move_back() {
    chassis_vel_msgs_.linear.x=-0.5;
    vel_pub_.publish(chassis_vel_msgs_);
}
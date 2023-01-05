//
// Created by aung on 2023/1/4.
//

#include <chassis_controller/EasyStateMachine.h>
EasyStateMachine::EasyStateMachine(ros::NodeHandle& nh) : context_(*this){
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    hp=1000;
    context_.enterStartState();
}

void EasyStateMachine::processState(){
    context_.processState();
}
void EasyStateMachine::move_forward() {
    ROS_INFO("move forward");
    chassis_vel_msgs_.linear.x=0.5;
    vel_pub_.publish(chassis_vel_msgs_);
}
void EasyStateMachine::move_back() {
    ROS_INFO("move back");
    chassis_vel_msgs_.linear.x=-0.5;
    vel_pub_.publish(chassis_vel_msgs_);
}
bool EasyStateMachine::car_state() {
    return hp>=500;
}
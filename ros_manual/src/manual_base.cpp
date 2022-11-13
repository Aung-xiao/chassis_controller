//
// Created by aung on 2022/11/11.
//

#include "ros_manual/manual_base.h"
namespace ros_manual
{

ManualBase::ManualBase(ros::NodeHandle& nh){
    dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &ManualBase::dbusDataCallback, this);
    vel_pub_  =nh.advertise<geometry_msgs::Twist>("/chassis/cmd_vel",1000);
    left_switch_down_event_.setRising(boost::bind(&ManualBase::leftSwitchDownRise, this));
}

void ManualBase::run(){
    ros::Time time = ros::Time::now();
    left_switch_down_event_.update(dbus_data_.s_l == rm_msgs::DbusData::DOWN);
    velUpdate(chassis_vel_msgs_);

}

void ManualBase::velUpdate(geometry_msgs::Twist& cmd_vel) {
    cmd_vel.linear.x=dbus_data_.ch_r_x;
    cmd_vel.linear.y=dbus_data_.ch_r_y;
    cmd_vel.angular.z=dbus_data_.ch_l_x;
    vel_pub_.publish(cmd_vel);
}

void ManualBase::leftSwitchDownRise(){
    ROS_INFO("left swithc down rise");
}

}
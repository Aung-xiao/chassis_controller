//
// Created by aung on 2022/11/11.
//
#pragma once

#include <queue>
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include "ros_manual/input_event.h"
#include "geometry_msgs/Twist.h"
#include <rm_msgs/DbusData.h>

namespace ros_manual
{
class ManualBase
{
public:
    explicit ManualBase(ros::NodeHandle& nh);
    virtual void run();
protected:
    virtual void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
    {
        dbus_data_ = *data;
    }
    virtual void leftSwitchDownRise();
    virtual void velUpdate(geometry_msgs::Twist& cmd_vel) ;

    InputEvent left_switch_down_event_;
    ros::Subscriber dbus_sub_;
    ros::Publisher vel_pub_;
    rm_msgs::DbusData dbus_data_;
    geometry_msgs::Twist chassis_vel_msgs_;
};

}
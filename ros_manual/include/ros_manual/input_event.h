//
// Created by aung on 2022/11/11.
//

#pragma once

#include <ros/ros.h>
#include <utility>

namespace ros_manual
{
class InputEvent
{
public:
    InputEvent() : last_state_(false)
    {
    }
    void setRising(boost::function<void()> handler)
    {
        rising_handler_ = std::move(handler);
    }
    void setFalling(boost::function<void()> handler)
    {
        falling_handler_ = std::move(handler);
    }
    void setActiveHigh(boost::function<void(ros::Duration)> handler)
    {
        active_high_handler_ = std::move(handler);
    }
    void setActiveLow(boost::function<void(ros::Duration)> handler)
    {
        active_low_handler_ = std::move(handler);
    }
    void setEdge(boost::function<void()> rising_handler, boost::function<void()> falling_handler)
    {
        rising_handler_ = std::move(rising_handler);
        falling_handler_ = std::move(falling_handler);
    }
    void setActive(boost::function<void(ros::Duration)> high_handler, boost::function<void(ros::Duration)> low_handler)
    {
        active_high_handler_ = std::move(high_handler);
        active_low_handler_ = std::move(low_handler);
    }
    void update(bool state)
    {
        if (state != last_state_)
        {
            if (state && rising_handler_)
                rising_handler_();
            else if (!state && falling_handler_)
                falling_handler_();
            last_state_ = state;
            last_change_ = ros::Time::now();
        }
        if (state && active_high_handler_)
            active_high_handler_(ros::Time::now() - last_change_);
        if (!state && active_low_handler_)
            active_low_handler_(ros::Time::now() - last_change_);
    }

private:
    bool last_state_;
    ros::Time last_change_;
    boost::function<void(ros::Duration)> active_high_handler_, active_low_handler_;
    boost::function<void()> rising_handler_, falling_handler_;
};

}  // namespace ros_manual

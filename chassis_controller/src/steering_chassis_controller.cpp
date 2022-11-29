//
// Created by aung on 2022/11/13.
//

#include <chassis_controller/steering_chassis_controller.h>
#include <controller_interface/controller.h>
#include "pluginlib/class_list_macros.h"
#include "ros/ros.h"

namespace steering_chassis_controller {
    /*
     effort_joint_interface:是机器人硬件部分的句柄，用于操控机器人
     root_nh：是ros控制端的节点的句柄
     controller_nh：是用来访问controller的参数或者函数之类的句柄
    */
    bool SteeringChassisController::init(hardware_interface::EffortJointInterface* hw,
                                 ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh){
        //get_handle
        front_right_wheel_joint_ = hw->getHandle("right_front_wheel_joint");
        front_left_wheel_joint_ = hw->getHandle("left_front_wheel_joint");
        back_left_wheel_joint_ = hw->getHandle("left_back_wheel_joint");
        back_right_wheel_joint_ = hw->getHandle("right_back_wheel_joint");

        front_right_pivot_joint_ = hw->getHandle("right_front_pivot_joint");
        front_left_pivot_joint_ = hw->getHandle("left_front_pivot_joint");
        back_left_pivot_joint_ = hw->getHandle("left_back_pivot_joint");
        back_right_pivot_joint_ = hw->getHandle("right_back_pivot_joint");

        //vel_sub
        cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("/chassis/cmd_vel", 1, &SteeringChassisController::cmdVelCallback, this);

        //get_pid
        flPid.init(ros::NodeHandle(controller_nh, "fl_pid_follow"));
        frPid.init(ros::NodeHandle(controller_nh, "fr_pid_follow"));
        blPid.init(ros::NodeHandle(controller_nh, "bl_pid_follow"));
        brPid.init(ros::NodeHandle(controller_nh, "br_pid_follow"));

        flpPid.init(ros::NodeHandle(controller_nh, "flp_pid_follow"));
        frpPid.init(ros::NodeHandle(controller_nh, "frp_pid_follow"));
        blpPid.init(ros::NodeHandle(controller_nh, "blp_pid_follow"));
        brpPid.init(ros::NodeHandle(controller_nh, "brp_pid_follow"));


        return true;
    }

    void SteeringChassisController::update(const ros::Time &time, const ros::Duration &period) {
        kinematicComputer(fl_vel,fr_vel,bl_vel,br_vel,flp_vel,frp_vel,blp_vel,brp_vel);
        pidComputer(time, period);
    }

    void SteeringChassisController::pidComputer(const ros::Time &time, const ros::Duration &period){
        double fl_pid_error =(fl_vel-front_left_wheel_joint_.getVelocity());
        double fr_pid_error =(fr_vel-front_right_wheel_joint_.getVelocity());
        double bl_pid_error =(bl_vel-back_left_wheel_joint_.getVelocity());
        double br_pid_error =(br_vel-back_right_wheel_joint_.getVelocity());
        double flp_pid_error =(flp_vel-front_left_pivot_joint_.getVelocity());
        double frp_pid_error =(frp_vel-front_right_pivot_joint_.getVelocity());
        double blp_pid_error =(blp_vel-back_left_pivot_joint_.getVelocity());
        double brp_pid_error =(brp_vel-back_right_pivot_joint_.getVelocity());

        flPid.computeCommand(fl_pid_error, period);
        frPid.computeCommand(fr_pid_error, period);
        blPid.computeCommand(bl_pid_error, period);
        brPid.computeCommand(br_pid_error, period);
        flpPid.computeCommand(flp_pid_error, period);
        frpPid.computeCommand(frp_pid_error, period);
        blpPid.computeCommand(blp_pid_error, period);
        brpPid.computeCommand(brp_pid_error, period);

        front_left_wheel_joint_.setCommand(flPid.getCurrentCmd());
        front_right_wheel_joint_.setCommand(frPid.getCurrentCmd());
        back_left_wheel_joint_.setCommand(blPid.getCurrentCmd());
        back_right_wheel_joint_.setCommand(brPid.getCurrentCmd());
        front_left_pivot_joint_.setCommand(flpPid.getCurrentCmd());
        front_right_pivot_joint_.setCommand(frpPid.getCurrentCmd());
        back_left_pivot_joint_.setCommand(blpPid.getCurrentCmd());
        back_right_pivot_joint_.setCommand(brpPid.getCurrentCmd());
    }

    void SteeringChassisController::kinematicComputer(double &fl_vel,double &fr_vel,double &bl_vel,double &br_vel,
                                                      double &flp_vel,double &frp_vel,double &blp_vel,double &brp_vel){
        //这里进行运动学解算
        fl_vel=cmd_vel_.linear.x*10;
        fr_vel=cmd_vel_.linear.x*10;
        bl_vel=cmd_vel_.linear.x*10;
        br_vel=cmd_vel_.linear.x*10;
        flp_vel=cmd_vel_.linear.y*10;
        frp_vel=cmd_vel_.linear.y*10;
        blp_vel=cmd_vel_.linear.y*10;
        brp_vel=cmd_vel_.linear.y*10;

    }

    void SteeringChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg_p){
        cmd_vel_.linear= msg_p->linear;
        cmd_vel_.angular=msg_p->angular;
    }


    PLUGINLIB_EXPORT_CLASS(steering_chassis_controller::SteeringChassisController, controller_interface::ControllerBase)

}

#include <chassis_controller/chassis_controller.h>
#include <controller_interface/controller.h>
#include "pluginlib/class_list_macros.h"
#include "ros/ros.h"

namespace chassis_controller {
    /*
     effort_joint_interface:是机器人硬件部分的句柄，用于操控机器人
     root_nh：是ros控制端的节点的句柄
     controller_nh：是用来访问controller的参数或者函数之类的句柄
    */
    bool ChassisController::init(hardware_interface::EffortJointInterface* hw,
             ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh){
        //get_handle
        front_right_wheel_joint_ = hw->getHandle("front_right_wheel_joint");
        front_left_wheel_joint_ = hw->getHandle("front_left_wheel_joint");
        back_left_wheel_joint_ = hw->getHandle("back_left_wheel_joint");
        back_right_wheel_joint_ = hw->getHandle("back_right_wheel_joint");

        //vel_sub
        cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &ChassisController::cmdVelCallback, this);

        //get_pid
        if (!flPid.init(ros::NodeHandle(controller_nh, "fl_pid_follow")))
        {
            ROS_ERROR("dont have flPid ");
            return false;
        }
        if (!frPid.init(ros::NodeHandle(controller_nh, "fr_pid_follow")))
        {
            ROS_ERROR("dont have frPid ");
            return false;
        }
        if (!blPid.init(ros::NodeHandle(controller_nh, "bl_pid_follow")))
        {
            ROS_ERROR("dont have blPid ");
            return false;
        }
        if (!brPid.init(ros::NodeHandle(controller_nh, "br_pid_follow")))
        {
            ROS_ERROR("dont have brPid ");
            return false;
        }
        return true;
    }

    void ChassisController::update(const ros::Time &time, const ros::Duration &period) {
        kinematicComputer(fl_vel_,fr_vel_,bl_vel_,br_vel_);
        pidComputer(time, period);
    }

    void ChassisController::pidComputer(const ros::Time &time, const ros::Duration &period){
        double fl_pid_error =(fl_vel_-front_left_wheel_joint_.getVelocity());
        double fr_pid_error =(fr_vel_-front_right_wheel_joint_.getVelocity());
        double bl_pid_error =(bl_vel_-back_left_wheel_joint_.getVelocity());
        double br_pid_error =(br_vel_-back_right_wheel_joint_.getVelocity());

        flPid.computeCommand(fl_pid_error, period);
        frPid.computeCommand(fr_pid_error, period);
        blPid.computeCommand(bl_pid_error, period);
        brPid.computeCommand(br_pid_error, period);

        front_left_wheel_joint_.setCommand(flPid.getCurrentCmd());
        front_right_wheel_joint_.setCommand(frPid.getCurrentCmd());
        back_left_wheel_joint_.setCommand(blPid.getCurrentCmd());
        back_right_wheel_joint_.setCommand(brPid.getCurrentCmd());
    }

    void ChassisController::kinematicComputer(double &fl_vel,double &fr_vel,double &bl_vel,double &br_vel){
        //这里进行运动学解算;这里有问题，实际速度慢了一倍
        fr_vel=(cmd_vel_.linear.x+cmd_vel_.linear.y+cmd_vel_.angular.z*(wheel_base+wheel_track)/2)/wheel_radius;
        fl_vel=(cmd_vel_.linear.x-cmd_vel_.linear.y-cmd_vel_.angular.z*(wheel_base+wheel_track)/2)/wheel_radius;
        bl_vel=(cmd_vel_.linear.x+cmd_vel_.linear.y-cmd_vel_.angular.z*(wheel_base+wheel_track)/2)/wheel_radius;
        br_vel=(cmd_vel_.linear.x-cmd_vel_.linear.y+cmd_vel_.angular.z*(wheel_base+wheel_track)/2)/wheel_radius;

    }

    void ChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg_p){
      cmd_vel_.linear= msg_p->linear;
      cmd_vel_.angular=msg_p->angular;
    }


PLUGINLIB_EXPORT_CLASS(chassis_controller::ChassisController, controller_interface::ControllerBase)

}

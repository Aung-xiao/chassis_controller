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
        front_left_wheel_joint_ = hw->getHandle("front_left_wheel_joint");
        front_right_wheel_joint_ = hw->getHandle("front_right_wheel_joint");
        back_left_wheel_joint_ = hw->getHandle("back_left_wheel_joint");
        back_right_wheel_joint_ = hw->getHandle("back_right_wheel_joint");

        //sub
        cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &ChassisController::cmdVelCallback, this);

        //get_pid
        if (controller_nh.hasParam("fl_pid_follow"))
            if (!flPid.init(ros::NodeHandle(controller_nh, "fl_pid_follow")))
            {
                ROS_ERROR("dont have flPid ");
                return false;
            }
        if (controller_nh.hasParam("fr_pid_follow"))
            if (!frPid.init(ros::NodeHandle(controller_nh, "fr_pid_follow")))
            {
                ROS_ERROR("dont have frPid ");
                return false;
            }
        if (controller_nh.hasParam("bl_pid_follow"))
            if (!blPid.init(ros::NodeHandle(controller_nh, "bl_pid_follow")))
            {
                ROS_ERROR("dont have blPid ");
                return false;
            }
        if (controller_nh.hasParam("br_pid_follow"))
            if (!brPid.init(ros::NodeHandle(controller_nh, "br_pid_follow")))
            {
                ROS_ERROR("dont have brPid ");
                return false;
            }
        return true;
    }

    void ChassisController::update(const ros::Time &time, const ros::Duration &period) {
        pidComputer(time, period);
        front_left_wheel_joint_.setCommand(flPid.getCurrentCmd());
        front_right_wheel_joint_.setCommand(frPid.getCurrentCmd());
        back_left_wheel_joint_.setCommand(blPid.getCurrentCmd());
        back_right_wheel_joint_.setCommand(brPid.getCurrentCmd());
    }

    void ChassisController::pidComputer(const ros::Time &time, const ros::Duration &period){
//        myPid.printValues();
        double fl_pid_error =-(front_left_wheel_joint_.getVelocity()-cmd_vel_.linear.x);
        flPid.computeCommand(fl_pid_error, period);
        double fr_pid_error =-(front_right_wheel_joint_.getVelocity()-cmd_vel_.linear.x);
        flPid.computeCommand(fr_pid_error, period);
        double bl_pid_error =-(back_left_wheel_joint_.getVelocity()-cmd_vel_.linear.x);
        flPid.computeCommand(bl_pid_error, period);
        double br_pid_error =-(back_right_wheel_joint_.getVelocity()-cmd_vel_.linear.x);
        flPid.computeCommand(br_pid_error, period);
    }

    void ChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg_p){
      cmd_vel_.linear= msg_p->linear;
    }


PLUGINLIB_EXPORT_CLASS(chassis_controller::ChassisController, controller_interface::ControllerBase)

}

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

        //sub
        cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &ChassisController::cmdVelCallback, this);

        //get_pid
        if (controller_nh.hasParam("fl_pid_follow"))
            if (!flPid.init(ros::NodeHandle(controller_nh, "fl_pid_follow")))
            {
                ROS_ERROR("dont have pid ");
                return false;
            }

        return true;
    }

    void ChassisController::update(const ros::Time &time, const ros::Duration &period) {
        pidComputer(time, period);
        front_left_wheel_joint_.setCommand(flPid.getCurrentCmd());
    }

    void ChassisController::pidComputer(const ros::Time &time, const ros::Duration &period){
//        myPid.printValues();
        double pid_error =-(front_left_wheel_joint_.getVelocity()-cmd_vel_.linear.x);
        flPid.computeCommand(pid_error, period);
    }

    void ChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg_p){
      cmd_vel_.linear= msg_p->linear;
    }


PLUGINLIB_EXPORT_CLASS(chassis_controller::ChassisController, controller_interface::ControllerBase)

}

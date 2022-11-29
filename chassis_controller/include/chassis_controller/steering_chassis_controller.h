//
// Created by aung on 2022/11/13.
//

#ifndef SRC_STEERING_CHASSIS_CONTROLLER_H
#define SRC_STEERING_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <effort_controllers/joint_velocity_controller.h>

//joint_state
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

// cmd_vel
#include "geometry_msgs/Twist.h"

//pid
#include "control_toolbox/pid.h"


namespace steering_chassis_controller {
    class SteeringChassisController :public controller_interface::Controller<hardware_interface::EffortJointInterface>{
    public:
        SteeringChassisController() = default;
        ~SteeringChassisController() override = default;

        bool init(hardware_interface::EffortJointInterface* robot_hw,
                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void pidComputer(const ros::Time &time, const ros::Duration &period) ;

        void kinematicComputer(double &fl_vel,double &fr_vel,double &bl_vel,double &br_vel,
                               double &flp_vel,double &frp_vel,double &blp_vel,double &brp_vel);

    private:
        int state_{};
        ros::Time current_time_, last_time_;
        hardware_interface::JointHandle front_left_wheel_joint_,front_right_wheel_joint_,back_left_wheel_joint_,back_right_wheel_joint_,
                                        front_left_pivot_joint_,front_right_pivot_joint_,back_left_pivot_joint_,back_right_pivot_joint_;

        //cmd_vel
        ros::Subscriber cmd_vel_sub_;
        geometry_msgs::Twist cmd_vel_;
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

        //pid
        control_toolbox::Pid flPid,frPid,blPid,brPid,
                             flpPid,frpPid,blpPid,brpPid;

        //kinematic
        double fl_vel,fr_vel,bl_vel,br_vel,
               flp_vel,frp_vel,blp_vel,brp_vel;
    };

}

#endif //SRC_STEERING_CHASSIS_CONTROLLER_H

//
// Created by aung on 2022/11/19.
//

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include "cmath"

double joint_vel[4];

void JointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state){
    joint_vel[0]=joint_state->velocity[0];
    joint_vel[1]=joint_state->velocity[1];
    joint_vel[2]=joint_state->velocity[2];
    joint_vel[3]=joint_state->velocity[3];
    ROS_INFO("xxxxxx joint1:%f,joint2:%f,joint3:%f,joint4:%f",joint_vel[0],joint_vel[1],joint_vel[2],joint_vel[3]);
    for(int i=0;i<=3;i++){
        if (abs(joint_vel[i])<=1.0) joint_vel[i]=0;
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber joint_state_sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, &JointStateCallback );
    tf::TransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;

    double wheel_radius=0.07625;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(1);
    while(n.ok()){

        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();
        ROS_INFO("joint1:%f,joint2:%f,joint3:%f,joint4:%f",joint_vel[0],joint_vel[1],joint_vel[2],joint_vel[3]);
        vx = ((joint_vel[1]-joint_vel[2])/2)*wheel_radius;
        vy = ((joint_vel[3]+joint_vel[2])/2)*wheel_radius;
        vth = ((joint_vel[1]-joint_vel[3])/((0.475+0.5)))*wheel_radius;
        ROS_INFO("vx:%f,vy:%f,vth:%f",vx,vy,vth);

        double delta_x = vx;
        double delta_y = vy;
        double delta_th = vth;

        x += delta_x;
        y += delta_y;
        th += delta_th;
        ROS_INFO("x:%f,y:%f,th:%f",x,y,th);
        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        ROS_INFO("odomx：%f,odomy:%f,odomth:%f",odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();
    }
}


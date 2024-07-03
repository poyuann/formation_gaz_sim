#ifndef GIMBAL_H
#define GIMBAL_H

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <Eigen/Dense>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <gazebo_msgs/ModelStates>
#include <mavros_msgs/State>
class Gimbal::
{
private::
    ros::Subscriber gimbal_sub;
    ros::Subscriber gain_sub;
    ros::Subscriber target_sub;
    ros::Subscriber self_sub;
    ros::Subscriber gt_sub;
    
    ros::Publisher  pitch_vel_cmd;
    ros::Publisher  yaw_vel_cmd;

    geometry_msgs::Pose target_sb;
    geometry_msgs::Pose current_pose;
    
public::
    Gimbal();
    
}
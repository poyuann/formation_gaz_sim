#include <ros/ros.h>
#include "ros/param.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/Mavlink.h>

#include "Gimbal.h"

int main(int argc, cahr **argv)
{
    ros::init( argc, argv, "gimbal_ctrl");
    ros::NodeHandle nh;

    ros::Rate rate(100);

    Gimbal gimbal;

    for(int i = 100; ros::ok() && i > 0; --i)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("gimbal ready to ctrl");

    while(ros::ok())
    {
        gimbal.pitch_vel_cmd.publish();
        gimbal.yaw_vel_cmd.publish();   
    }
}
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/MountControl.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include "getch.h"
#include <cmath>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>
#include <queue>
#include <state_estimation/Mav.h>
#include <vector>
void bound_yaw(double* yaw){
        if(*yaw>M_PI)
            *yaw = *yaw - 2*M_PI;
        else if(*yaw<-M_PI)
            *yaw = *yaw + 2*M_PI;
}

int main(int argc, char **argv)
{
    //  ROS_initialize  //
    ros::init(argc, argv, "keyboard_ctrl");
    ros::NodeHandle nh;

    // object
    MAV mav(nh, "target", 0);

    // publisher
    ros::Publisher desired_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher vel_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    // ros::Publisher mount_pub = nh.advertise<mavros_msgs::MountControl>("mavros/mount_control/command", 30);
    ros::Publisher gimbalVel_pitch_pub = nh.advertise<std_msgs::Float32>("/typhoon_h4800/gimbal/pitch/cmd_joint_velocity", 5);
    ros::Publisher gimbalVel_yaw_pub = nh.advertise<std_msgs::Float32>("/typhoon_h4800/gimbal/yaw/cmd_joint_velocity", 5);
    // service
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");


    ros::Rate rate(100);
    
    // cmd_msgs

    ROS_INFO("Wait for pose and desired input init");
    while (ros::ok() && (!mav.pose_init)) 
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for pose init %d",mav.pose_init);
    }
    ROS_INFO("pose initialized");
    
    ROS_INFO("Wait for FCU connection");
    while (ros::ok() && !mav.getState().connected) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for FCU");
    }
    ROS_INFO("FCU connected");

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    mavros_msgs::CommandTOL land_request;
    
    offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        ROS_INFO("takeoff enabled");
        // for(int i=0; i<100 ;i++)
        //     std::cout << "wait for init\n";
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        ROS_INFO("Offboard enabled");
    if( arming_client.call(arm_cmd) && arm_cmd.response.success)
        ROS_INFO("Vehicle armed");

    geometry_msgs::PoseStamped desired_pose;
    desired_pose.pose.position.x = 0;
    desired_pose.pose.position.y = 0;
    desired_pose.pose.position.z = 0;

    geometry_msgs::TwistStamped desired_vel;
    desired_vel.twist.linear.x = 0;
    desired_vel.twist.linear.y = 0;
    desired_vel.twist.linear.z = 0;    
    
    std_msgs::Float32 desired_gimbal_pitch;
    std_msgs::Float32 desired_gimbal_yaw;
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        desired_pose_pub.publish(desired_pose);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Vehicle is ready to start");
    double des_x = 0, des_y = 0, des_z = 0;
    double move_step = 0.3;
    double desired_yaw = 0;
    bool trajectory = false;
    double trajectory_time = 0;
    double trajectory_step = 0;
    double current_x = 0;
    double current_y = 0;
    double current_z = 0;
    double yaw, pitch, roll;
        std::vector<std::vector<double>> waypoint{  {0 , 0},
                                                {-30 , 0},
                                                {-30, 30},
                                                {0 , 30 }};
    // Eigen::Vector3d 
    bool step = false;
    while (ros::ok()) 
    {
        if (!step )
        {
            // if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            //     ROS_INFO("takeoff enabled");
        }
        else{
            if (mav.getState().mode != "OFFBOARD" &&
                    (ros::Time::now() - last_request > ros::Duration(2.0))) {
                if( set_mode_client.call(offb_set_mode) &&
                        offb_set_mode.response.mode_sent) {
                    ROS_INFO("Offboard enabled" );
                }
                last_request = ros::Time::now();
            } else {

                if (!mav.getState().armed &&
                        (ros::Time::now() - last_request > ros::Duration(2.0))) {
                    if( arming_client.call(arm_cmd) &&
                            arm_cmd.response.success) {
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
        }

        // for(int i=0; i<mavNum; i++)
        tf::Quaternion Q(
            mav.getPose().pose.orientation.x,
            mav.getPose().pose.orientation.y,
            mav.getPose().pose.orientation.z,
            mav.getPose().pose.orientation.w
        );
        // yaw = tf::getYaw(Q);
        tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
        
        current_x = mav.getPose().pose.position.x;
        current_y = mav.getPose().pose.position.y;
        current_z = mav.getPose().pose.position.z;
        //keyboard control
        int c = getch();
        //ROS_INFO("C: %d",c);
        //update desired pose
        if (c != 0) 
        {
            switch (c) {
                case 65:    // key up
                    // desired_pose.pose.position.z += move_step;
                    des_z += move_step;
                    break;
                case 66:    // key down
                    // desired_pose.pose.position.z += -move_step;
                    des_z += -move_step;
                    
                    break;
                case 67:    // key CW(->)
                    desired_yaw -= 0.03;
                    bound_yaw(&desired_yaw); 
                    desired_pose.pose.orientation.z = desired_yaw;
                    desired_vel.twist.angular.z = (desired_yaw - yaw); 

                    break;
                case 68:    // key CCW(<-)
                    desired_yaw += 0.03;
                    bound_yaw(&desired_yaw); 
                    desired_vel.twist.angular.z = (desired_yaw - yaw); 

                    break;
                case 119:    // key foward(w)
                    desired_pose.pose.position.y += move_step;
                    des_y += move_step;
                    break;
                case 115:    // key back(s)
                    desired_pose.pose.position.y -= move_step;
                    des_y -= move_step;
                    break;
                case 97:    // key left(a)
                    desired_pose.pose.position.x -= move_step;
                    des_x += move_step;
                    break;
                case 100:    // key right(d)
                    desired_pose.pose.position.x += move_step;
                    des_x -= move_step;
                    break;
                case 108:    // key land(l)
                    desired_pose.pose.position.z = 0.5;
                    break;
                case 101:    // key trajectory_CCW(e)
                    trajectory = true;
                    trajectory_time = 0;
                    // current_x = mav.getPose().pose.position.x;
                    // current_y = mav.getPose().pose.position.y;
                    break;
                case 112:    // key stop trajectory(p)
                    trajectory = false;
                    break;  
                case 111:
                    desired_pose.pose.position.x = 0;
                    desired_pose.pose.position.y = 0;
                    desired_pose.pose.position.z = 40;
                    desired_pose.pose.orientation.z = 1.7;
                    des_x = 0;
                    des_y = 0;
                    des_z = 40;
                    step = true;
                    break;
                case 107:   // key kill(k)
                    return 0;
            }
            ROS_INFO("setpoint: %.2f, %.2f, %.2f", desired_pose.pose.position.x, desired_pose.pose.position.y, desired_pose.pose.position.z);
        }
        
        if(trajectory)
        {
            // trajectory_time += 0.4;
            // des_x = current_x + 30*cos(trajectory_time);
            // des_y = current_y + 30*sin(trajectory_time);
            des_x = waypoint[trajectory_step][0];
            des_y = waypoint[trajectory_step][1];
            // desired_vel.twist.linear.x = 0.5*(des_x - current_x);
            // desired_vel.twist.linear.y = 0.5*(des_y - current_y);     
            // desired_vel.twist.linear.z = 0.5*(des_z - current_z); 
            if ((pow(current_x-waypoint[trajectory_step][0],2)+pow(current_y-waypoint[trajectory_step][1],2))< 1)
                trajectory_step++;
                // desired_yaw += M_PI/2;
                bound_yaw(&desired_yaw); 

                if(trajectory_step > 3)
                    trajectory_step = 0;
        }
        desired_vel.twist.linear.x = 0.1*(des_x - current_x);
        desired_vel.twist.linear.y = 0.1*(des_y - current_y);     
        desired_vel.twist.linear.z = 0.5*(des_z - current_z); 

        desired_vel.twist.angular.z = (desired_yaw - yaw); 

        // desired_mount.mode = 2;;
        desired_gimbal_pitch.data = -pitch;
        desired_gimbal_yaw.data = -yaw;

        // desired_pose_pub.publish(desired_pose);
        vel_cmd_pub.publish(desired_vel);
        // gimbalVel_pitch_pub.publish(desired_gimbal_pitch);
        // gimbalVel_yaw_pub.publish(desired_gimbal_yaw);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
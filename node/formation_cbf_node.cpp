#include <cmath>

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

#include "state_estimation/Mav.h"
#include "Formation.h"
#include "Velocity_cbf.h"
// #include <state_estimation/Plot.h>

class CMD
{
    private:
        bool arm_cmd;
        int mode_cmd;
        int ID;
        int curr_mode;
        string offb_curr_mode;

        ros::NodeHandle nh;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::ServiceClient takeoff_client;
    public:
        bool vision_tracking = false;

        CMD(ros::NodeHandle &nh_, int id)
        {
            nh = nh_;
            ID = id;
            offb_curr_mode = "";

            arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
            set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
            takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
        }
        ~CMD(){}
        ros::Subscriber arm_cmd_sub = nh.subscribe<std_msgs::Bool>("/formation/all_uav_arm", 5, &CMD::arm_cmd_cb, this);
        ros::Subscriber mode_cmd_sub = nh.subscribe<std_msgs::Int32>("/formation/all_uav_mode", 5, &CMD::mode_cmd_cb, this);
        void arm_cmd_cb(const std_msgs::Bool::ConstPtr& msg)
        {
            arm_cmd = msg->data;
            setArm(arm_cmd);
        }
        void mode_cmd_cb(const std_msgs::Int32::ConstPtr& msg)
        {
            mode_cmd = msg->data;
            setMode(mode_cmd);
        }
        void setArm(bool arm_CMD)
        {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = arm_CMD;
            if( arming_client.call(arm_cmd) && arm_cmd.response.success) 
                ROS_INFO("UAV_%i armed switch successfully", ID);
            else
                ROS_INFO("UAV_%i failed to arm", ID);
        }
        void setMode(int mode_CMD)
        {
            mavros_msgs::SetMode offb_set_mode;
            switch (mode_CMD)
            {
            case 0: 
                offb_set_mode.request.custom_mode = "STABILIZED";
                break;
            case 1: 
                offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";
                break;
            case 2: 
                offb_set_mode.request.custom_mode = "AUTO.LAND";
                break;
            case 3:
                offb_set_mode.request.custom_mode = "OFFBOARD";
                break;
            case 5:
                if(!vision_tracking)
                {
                    ROS_INFO("Tracking by vision");
                    vision_tracking = true;
                }
                else
                {
                    ROS_INFO("Stop vision tracking");
                    vision_tracking = false;
                }
                break;
            
            default:
                break;
            }
            if(offb_set_mode.request.custom_mode != offb_curr_mode)
            {
                if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("UAV_%i mode switched to %s", ID, offb_set_mode.request.custom_mode.c_str());
                    offb_curr_mode = offb_set_mode.request.custom_mode;
                    if(offb_set_mode.request.custom_mode == "AUTO.TAKEOFF")
                        sleep(5);
                }
                else
                    ROS_INFO("UAV_%i failed to switch offb_mode", ID);
            }
        }
        int mode_CMD(){return mode_cmd;}
};

class Target_EST_FeedBack
{
private:
    ros::Subscriber target_pose_sub;
    ros::Subscriber target_twist_sub;
    ros::Subscriber isTargetEst_sub;
    geometry_msgs::Pose target_est_pose;
    geometry_msgs::Twist target_est_twist;
    
public:
    bool estimating;
    Target_EST_FeedBack(ros::NodeHandle& nh_)
    {   
        target_pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/typhoon_h480/THEIF/pose", 10, &Target_EST_FeedBack::est_pose_cb, this);
        target_twist_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/typhoon_h480/THEIF/twist", 10, &Target_EST_FeedBack::est_twist_cb, this);
        isTargetEst_sub = nh_.subscribe<std_msgs::Bool>("THEIF/isTargetEst", 10, &Target_EST_FeedBack::isTargetEst_cb, this);
    }
    void est_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){target_est_pose = msg->pose;}
    void est_twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){target_est_twist = msg->twist;}
    void isTargetEst_cb(const std_msgs::Bool::ConstPtr& msg)
    {
        estimating = msg->data;
    }
    geometry_msgs::Pose getPose(){return target_est_pose;}
    geometry_msgs::Twist getTwist(){return target_est_twist;}
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation");
    ros::NodeHandle nh;

    std::string vehicle;
    ros::param::get("vehicle", vehicle);
    ros::Publisher vel_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
  
    MAV mavs[]={MAV(nh, "target", 0 ,0),
                MAV(nh, vehicle, 1 ,0) ,
                MAV(nh, vehicle, 2 ,0),
                MAV(nh, vehicle, 3 ,0)};
    int mavNum = sizeof(mavs)/sizeof(mavs[0]);
    int ID;
    std::vector<MAV_eigen> Mavs_eigen(mavNum);
    ros::param::get("mav_id", ID);
    Formation::ID = Velocity_cbf::ID = ID;
    
    CMD cmd(nh, ID);
    Target_EST_FeedBack target(nh);
    Formation formation(mavNum);
    Velocity_cbf CBF(mavNum);
    double d = 3.0;
    double gamma = 1.2;
    double safe_Distance = 3;

    std::vector<std::vector<bool>> laplacianMap{{1, 0, 0, 0},
                                                {1, 1, 1, 1},
                                                {1, 1, 1, 1},
                                                {1, 1, 1, 1}};
    std::vector<std::vector<double>> formationMap{{0, 0, 0},
                                                {0, d, 2},
                                                {double(sqrt(3))*d/2, -d/2, 2},
                                                {double(-sqrt(3))*d/2, -d/2, 2}};
    // std::vector<std::vector<double>> formationMap{{0, 0, 10},
    //                                             { 1.3580,    -1.2072,  13.5635},
    //                                             { -0.7669, 1.7305 , 13.5238},
    //                                             { -2.3468, -3.0174, 8.8218}};  //best but target error sometimes lidar

    // std::vector<std::vector<double>> formationMap{{0, 0, 0},
    //                                             { 3.30614689,  1.30160112 , 3.51784412},
    //                                             { 0.56290616,  1.79564649 , 4.63236336},
    //                                             { 1.69215895, -0.9835283 ,  4.60100752}};
    // std::vector<std::vector<double>> formationMap{{0, 0, 0},
    //                                             {4.44976, 0.306942, 2.25952},
    //                                             {1.04937, -0.0140828, 4.88862},
    //                                             {-4.17454, 0.211045, 2.74384}
    //                                             };                             // worst camera
    // std::vector<std::vector<double>> formationMap{{0, 0, 0},
    //                                             {-0.937938, -2.75732, 4.06417},
    //                                             {-3.10855, -0.697186, 3.85368},
    //                                             {2.11253, -4.50434, 0.498131}
    //                                             };                             //test worst
    // std::vector<std::vector<double>> formationMap{{0, 0, 0},
    //                                             { 1.77516454e+00 , 4.67426902e+00 , 9.76934257e-06},
    //                                             {-4.93561781e+00 ,-7.99798015e-01 , 8.98211992e-06},
    //                                             { 3.16045033e+00 ,-3.87447464e+00 , 2.38316044e-05}};                             //nei worst
    // std::vector<std::vector<double>> formationMap{{0, 0, 0},
    //                                         {-3.89788, -2.3502, 2.06956}, 
    //                                         {3.21168, -2.32042, 3.04971}, 
    //                                         {-0.435081, 4.87361, 1.0289}
    //                                         };                             // test optimal
    // std::vector<std::vector<double>> formationMap{{0, 0, 0},
    //                                             {1.11057e-08, 3.8913, 3.13971},
    //                                             {-4.89413, -0.321373, 0.9717},
    //                                             {3.39891, -2.07242, 3.02531}
    //                                             };                             // lastest optimal
    // std::vector<std::vector<double>> formationMap{{0, 0, 0},
    //                                         { 2.5         ,0.         , 4.33012702},
    //                                         { 1.76776695  ,3.06186218  ,3.53553391},
    //                                         {-0.33493649  ,1.25        ,4.82962913}};  
    // std::vector<std::vector<double>> formationMap{{0, 0, 0},
    //                                         {  4.33654757 , 1.97100355 , 1.51970398},
    //                                         {-2.02058568 , 1.00561994 , 4.4616098 },
    //                                         { 1.45313501, -4.48373122 , 1.66869798}};  
    formation.setLaplacianMap(laplacianMap);
    formation.setFormationMap(formationMap);

    CBF.setCBFparams(gamma, safe_Distance);


    ros::Rate rate(30);
    while (ros::ok() && !mavs[ID].getState().connected)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("UAV_%i waiting for FCU", ID);
    }
    // while (ros::ok() && !mavs[ID].pose_init)
    // {
    //     ros::spinOnce();
    //     rate.sleep();
    //     ROS_INFO("UAV_%i waiting for position estimation", ID);
    // }
    
    Eigen::Vector3d formation_vel;
    Eigen::VectorXd final_vel;
    double yaw_vel;
    geometry_msgs::TwistStamped vel_msg;
    std::vector<std::vector<double>> tempMap;
    double dt = 0.001;
	double last_t = ros::Time::now().toSec();

    double t_time=0;
    while (ros::ok()) 
    {
        bool armed = mavs[ID].getState().armed;
        // std::cout << "[UAV_" << ID << "]:\n";
        // std::cout << "Armed: " << armed << "\n";
        // std::cout << "Mode: " << mavs[ID].getState().mode << "\n";
        // t_time += 0.01;
        // for(int i=1; i < formationMap.size(); i++)
        // {
        //     tempMap[i][0] = formationMap[i][0] *cos(t_time) + formationMap[i][1]*sin(t_time);
        //     tempMap[i][1] = -formationMap[i][0] * sin(t_time) + formationMap[i][1] *cos(t_time);            
        // }
        // formation.setFormationMap(tempMap);
        for (int i=0;i<4;i++)
            cout <<"i = "<<i<< mavs[i].getPose() << "\n";
        std:: cout <<"mode :  "<<cmd.mode_CMD() <<"\n";

            // cout << "i = " <<i<< tempMap[1][i]<<"\n\n";
        // if(cmd.mode_CMD() == 5)
        // {
        //     if(true)
        //     // if(target.estimating)
        //     {
        //         t_time += 0.01;
        //         for(int i=1; i < formationMap.size(); i++)
        //         {
        //             tempMap[i][0] = formationMap[i][0] *cos(t_time) + formationMap[i][1]*sin(t_time);
        //             tempMap[i][1] = -formationMap[i][0] * sin(t_time) + formationMap[i][1] *cos(t_time);            
        //         }
        //         formation.setFormationMap(tempMap);
        //     }
        //     else
        //     {
        //         ROS_INFO("Target lost, unable to track");
        //         cmd.vision_tracking = false;
        //         geometry_msgs::Twist stop;
        //         stop.linear.x = 0;
        //         stop.linear.y = 0;
        //         stop.linear.z = 0;
        //         mavs[0].setTwist(stop);
        //     }
                
        // }
        for(int i=0; i<mavNum; i++)
            Mavs_eigen[i] = mavMsg2Eigen(mavs[i]);
        formation.setCurr_Pose_Vel(Mavs_eigen);
        CBF.setMavsPosition(Mavs_eigen);
        formation_vel = formation.computeDesiredLVelocity(dt);
        yaw_vel = formation.computeDesiredYawVelocity(cmd.mode_CMD());
        Eigen::Vector3d ang_vel = Eigen::Vector3d::Zero();
        ang_vel(2) = yaw_vel;

        final_vel = formation_vel;
        // if(cmd.mode_CMD() == 5)
        // {
            
        //     if((ros::Time::now() - mavs[Formation::ID].getPose().header.stamp)<ros::Duration(0.1))
        //     {
        //         if(CBF.CBF_init(6))
        //         {
        //             if(CBF.computeCBF(formation_vel, ang_vel))
        //             {
        //                 final_vel = CBF.getOptimizedVel();
        //             }
        //             else
        //                 final_vel = formation_vel;
        //         }
        //     }                
        // }
        vel_msg.header.stamp = ros::Time::now();
        vel_msg.twist.linear.x = final_vel(0);
        vel_msg.twist.linear.y = final_vel(1);
        vel_msg.twist.linear.z = final_vel(2);
        vel_msg.twist.angular.z = yaw_vel;
        if(final_vel.size() > 3)
        {
            vel_msg.twist.angular.x = final_vel(3);
            vel_msg.twist.angular.y = final_vel(4);
            vel_msg.twist.angular.z = final_vel(5);
        }
        std::cout << vel_msg <<"\n";
        vel_cmd_pub.publish(vel_msg);

        dt = ros::Time::now().toSec() - last_t;
    	last_t = ros::Time::now().toSec();
      
        rate.sleep();
        ros::spinOnce();
    }

}
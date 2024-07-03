#include "Gimbal.h"

Gimbal::Gimbal(ros::NodeHandle &nh_, string vehicle, int ID)
{
    nh = nh_;
    pose_init = vel_init = imu_init = false;
    id = ID;
    roll = pitch = yaw = 0;
    topic_count = 0;
    
    gt_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_state", 10, &Gimbal::gt_cb, this);
    imu_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, &Gimbal::imu_cb, this);
 
    mav_state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &Gimbal::mav_state_cb, this);

    /*////////////////////////////////////////////////////////////////////////////////////////*/

    pitch_vel_cmd = nh.advertise<std_msgs::Float32>("gimbal/pitch/cmd_joint_velocity ", 10, 1);
    yaw_vel_cmd = nh.advertise<std_msgs::Float32>("gimbal/yaw/cmd_joint_velocity ", 10, 1);
}
/*//////////////////////////////////////////////////////////////////////////////////////////////////*/
void Gimbal::gt_cb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{

	////////////////////////// get groundTruth model states and arrange their ID////////////////////
	std::vector<string> name = msg->name;
    string prefix =  string("typhoon_h480") + to_string(id);
    for (int i = 0; i<name.size(); i++)
    {
		if(std::isdigit(name[id+2].back())) ////// First one is ground, skip it
		{
            if(prefix == name[i])
            {
                setPose(msg->pose[i]);
                setTwist(msg->twist[i]);
                // std::cout << name[i]<<"\n"<<id <<"\n";

            }
            if(id==0 && string("iris0")== name[i])
            {
                setPose(msg->pose[i]);
                setTwist(msg->twist[i]);
            }
        }
    }

}
void Gimbal::mav_state_cb(const mavros_msgs::State::ConstPtr& msg) 
{
    state = *msg;
}

void Gimbal::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(!pose_init)
        pose_init = true;
    pose_current = *msg;
}

void Gimbal::vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    if(!vel_init)
        vel_init = true;
    vel_current = *msg;
}

void Gimbal::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    if(!imu_init)
        imu_init = true;
    imu_current = *msg;
    pose_current.pose.orientation = imu_current.orientation;
    vel_current.twist.angular = imu_current.angular_velocity;
    acc_current = imu_current.linear_acceleration;
}

sensor_msgs::Imu Gimbal::getImu(){return imu_current;}
geometry_msgs::PoseStamped Gimbal::getPose(){return pose_current;}
geometry_msgs::TwistStamped Gimbal::getVel(){return vel_current;}
geometry_msgs::Vector3 Gimbal::getAcc(){return acc_current;}
mavros_msgs::State Gimbal::getState(){return state;}

void Gimbal::setPose(geometry_msgs::Pose Pose)
{
    pose_current.header.stamp = ros::Time::now();
    pose_current.pose = Pose;
}
void Gimbal::setTwist(geometry_msgs::Twist Twist)
{
    vel_current.header.stamp = ros::Time::now();
    vel_current.twist = Twist;
}

void Gimbal::setOrientation(geometry_msgs::Quaternion q)
{
    pose_current.pose.orientation = q;
}
void Gimbal::anglarVel_ctrl()
{
    
}
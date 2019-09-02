#ifndef DUAL_ROBOCLAW_SKIDSTEER_DRIVETRAIN_H_
#define DUAL_ROBOCLAW_SKIDSTEER_DRIVETRAIN_H_

#include <pigpiod_if2.h>
#include <RoboCommander/drivetrains/dual_roboclaw.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <swanson_msgs/DualClawInfo.h>

using namespace std;

class DualClawSkidsteerDrivetrainInterface{
private:
     int pi;

     ros::NodeHandle m_nh;
	ros::NodeHandle p_nh;
     ros::Rate* _loop_rate;
     int _count;
     ros::Subscriber cmd_sub;
     ros::Publisher data_pub;

     void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg, const int topic_index);

public:
     DualClaw* claws;
     // Contructor/DeConstructor
     DualClawSkidsteerDrivetrainInterface(ros::NodeHandle nh, ros::NodeHandle _nh);
     ~DualClawSkidsteerDrivetrainInterface();

     void update(bool verbose = false);
     int run(bool verbose = false);
};


#endif // DUAL_ROBOCLAW_SKIDSTEER_DRIVETRAIN_H_

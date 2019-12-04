#ifndef DUAL_ROBOCLAW_SKIDSTEER_DRIVETRAIN_H_
#define DUAL_ROBOCLAW_SKIDSTEER_DRIVETRAIN_H_

#include <pigpiod_if2.h>
#include <RoboCommander/drivetrains/dual_roboclaw.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <swanson_msgs/DualClawInfo.h>

#include <tf/transform_broadcaster.h>

using namespace std;

class DualClawSkidsteerDrivetrainInterface{
private:
     int pi;
     int _count;

     /** ROS Objects */
     ros::NodeHandle m_nh;
	ros::NodeHandle p_nh;
     ros::Rate* _loop_rate;
     ros::Subscriber cmd_sub;
     ros::Publisher data_pub;
     ros::Publisher pose_pub;
     ros::Publisher odom_pub;
     tf::TransformBroadcaster _br;

     /** ROS Params */
     std::string _ns;
     std::string _tf_prefix;
     std::string _tf_odom;
     std::string _tf_base;

     /** Transforms */
     tf::Transform _tfBaseToOdom;

     /** ROS Subscriber Callbacks */
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

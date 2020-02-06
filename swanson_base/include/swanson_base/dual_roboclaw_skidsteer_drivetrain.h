#ifndef DUAL_ROBOCLAW_SKIDSTEER_DRIVETRAIN_H_
#define DUAL_ROBOCLAW_SKIDSTEER_DRIVETRAIN_H_

#include <mutex>
#include <pigpiod_if2.h>
#include <RoboCommander/drivetrains/dual_roboclaw.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Quaternion.h>
#include <swanson_msgs/DualClawInfo.h>

#include <tf/transform_broadcaster.h>

using namespace std;

class DualClawSkidsteerDrivetrainInterface{
private:
     int pi;
     int _count;
     int _cmd_count;
     std::mutex _lock;
     float _target_vel;
     float _target_rot;
     std::vector<int32_t> _cmds;

     /** ROS Objects */
     ros::NodeHandle m_nh;
	ros::NodeHandle p_nh;
     ros::Rate* _loop_rate;
     ros::Subscriber cmd_sub;
     ros::Publisher data_pub;
     ros::Publisher pose_pub;
     ros::Publisher odom_pub;
     tf::TransformBroadcaster _br;
     ros::ServiceServer _reset_enc;

     /** ROS Params */
     std::string _ns;
     bool _publishTf;
     bool _verbose;
     std::string _tf_prefix;
     std::string _tf_odom;
     std::string _tf_base;

     /** Pre-initialize ROS Topic Messages */
     nav_msgs::Odometry _odomMsg;
     geometry_msgs::PoseStamped _poseMsg;
     geometry_msgs::TransformStamped _odom_tf;


     /** ROS Subscriber Callbacks */
     void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg, const int topic_index);
     bool reset_odometry(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
public:
     DualClaw* claws;
     // Contructor/DeConstructor
     DualClawSkidsteerDrivetrainInterface(ros::NodeHandle nh, ros::NodeHandle _nh);
     ~DualClawSkidsteerDrivetrainInterface();

     void update();
     int run(bool verbose = false);
};

#endif // DUAL_ROBOCLAW_SKIDSTEER_DRIVETRAIN_H_

#ifndef DIFFDRIVE_ROBOCLAW_SKIDSTEER_DRIVETRAIN_H_
#define DIFFDRIVE_ROBOCLAW_SKIDSTEER_DRIVETRAIN_H_

#include <mutex>
#include <pigpiod_if2.h>
#include <robocommander/drivetrains/diffdrive_roboclaw.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>

#include <swanson_base/DrivetrainConfig.h>
#include <swanson_msgs/DiffDriveClawInfo.h>

using namespace std;

class DiffDriveClawSkidsteerDrivetrainInterface{
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
    dynamic_reconfigure::Server<swanson_base::DrivetrainConfig> _cfg_server;
    dynamic_reconfigure::Server<swanson_base::DrivetrainConfig>::CallbackType _cfg_f;

    /** ROS Params */
    std::string _ns;
    bool _verbose;
    bool _publishTf;
    std::string _tf_prefix;
    std::string _tf_odom;
    std::string _tf_base;

    /** Dynamic Variable Changing */
    float _max_speed;
    float _base_width;
    float _wheel_diameter;
    float _max_turn_radius;
    int _qpps_per_meter;
    bool _was_cmd_dir_flipped;
    bool _was_odom_dir_flipped;

    /** Pre-initialize ROS Topic Messages */
    nav_msgs::Odometry _odomMsg;
    geometry_msgs::PoseStamped _poseMsg;
    geometry_msgs::TransformStamped _odom_tf;

    /** ROS Subscriber Callbacks */
    bool reset_odometry(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void cfgCallback(swanson_base::DrivetrainConfig &config, uint32_t level);
    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg, const int topic_index);
public:
    DiffDriveClaw* claw;
    // Contructor/DeConstructor
    DiffDriveClawSkidsteerDrivetrainInterface(ros::NodeHandle nh, ros::NodeHandle _nh);
    ~DiffDriveClawSkidsteerDrivetrainInterface();

    void update();
    int run(bool verbose = false);
};

#endif // DIFFDRIVE_ROBOCLAW_SKIDSTEER_DRIVETRAIN_H_

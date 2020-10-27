#ifndef CAMERA_GIMBAL_CONTROLLER_ROS_H_
#define CAMERA_GIMBAL_CONTROLLER_ROS_H_

#include <pigpiod_if2.h>
#include <robocommander/base/peripherals.h>
#include <robocommander/devices/camera_gimbal.h>
#include <robocommander/devices/tca9548a.h>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <swanson_controls/GimbalConfig.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

using namespace std;

class CameraGimbalControllerRos{
private:
     /** Constants */
     int _pi;
     int _count;
     float _maxAngle;
     float _targetAngle;
     float _curAngles[3] = {0.0, 0.0, 0.0};
     int _nullPulse;
     float _control_dt;
     float _kp, _ki, _kd;
     bool _flag_start = false;

     /** Device Communication Parameters */
     COMMUNICATION_CONFIGURATION _actCommCfg;

     /** ROS Elements */
     ros::NodeHandle m_nh;
	ros::NodeHandle p_nh;
     ros::Rate* _loop_rate;

     dynamic_reconfigure::Server<swanson_controls::GimbalConfig> _cfg_server;
     dynamic_reconfigure::Server<swanson_controls::GimbalConfig>::CallbackType _f;

     ros::Subscriber cmd_sub;
     ros::Subscriber imu_sub;
     ros::Publisher data_pub;

     void cmdCallback(const std_msgs::Float32::ConstPtr& msg, const int topic_index);
     void imuCallback(const geometry_msgs::Vector3::ConstPtr& msg, const int topic_index);
     void cfgCallback(swanson_controls::GimbalConfig &config, uint32_t level);

public:
     CameraGimbal* cg;

     // Contructor/DeConstructor
     CameraGimbalControllerRos(ros::NodeHandle nh, ros::NodeHandle _nh);
     ~CameraGimbalControllerRos();

     void update(bool verbose = false);
     int run(bool verbose = false);

     void stop();
};


#endif // CAMERA_GIMBAL_CONTROLLER_ROS_H_

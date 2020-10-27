#ifndef ANDROID_ROS_INTERFACE_LISTENER_H_
#define ANDROID_ROS_INTERFACE_LISTENER_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#include <robocommander/interfaces/android_app_interface.h>

using namespace std;

class AndroidRosInterface{
private:
     ros::NodeHandle m_nh;
	ros::NodeHandle p_nh;
     ros::Rate* _loop_rate;

     ros::Publisher cmd_pub;
     ros::Publisher angF_pub;
     ros::Publisher angB_pub;
     ros::Publisher angR_pub;
     ros::Publisher angL_pub;

     AndroidAppInterface* mRelay;

     float _max_speed;
     float _max_omega;

     int _count;
     int _rc_msg_count;
     int _header_count;
     int _gimbal_msg_count;
     int timeoutCnt;
     int maxTimeout;
public:
     // Contructor/DeConstructor
     AndroidRosInterface(ros::NodeHandle nh, ros::NodeHandle _nh);
     ~AndroidRosInterface();

     void update();
     int run();
};


#endif // ANDROID_ROS_INTERFACE_LISTENER_H_

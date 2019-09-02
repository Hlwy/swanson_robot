#ifndef RTIMU_ROS_H_
#define RTIMU_ROS_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>

#include <RoboCommander/base/definitions.h>
#include <RoboCommander/sensors/generic_rtimu.h>

using namespace std;

class RtImuRos{
private:
     ros::NodeHandle m_nh;
	ros::NodeHandle p_nh;
     ros::Rate* _loop_rate;

     ros::Publisher imu_pub;
     ros::Publisher pose_pub;

     float dt;
     int _count;
public:
     // Contructor/DeConstructor
     RtImuRos(ros::NodeHandle nh, ros::NodeHandle _nh);
     ~RtImuRos();

     GenericRTIMU* imu;

     void update(bool verbose = false);
     int run(bool verbose = false);
};


#endif // RTIMU_ROS_H_

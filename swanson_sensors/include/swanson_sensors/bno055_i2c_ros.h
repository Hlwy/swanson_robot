#ifndef BNO055_I2C_ROS_H_
#define BNO055_I2C_ROS_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

#include <RoboCommander/base/definitions.h>
#include <RoboCommander/sensors/bno055_i2c.h>
#include <RoboCommander/devices/tca9548a.h>

using namespace std;

class BNO055_I2C_Ros{
private:
     int _pi;
     ros::NodeHandle m_nh;
	ros::NodeHandle p_nh;
     ros::Rate* _loop_rate;

     ros::Publisher imu_pub;
     ros::Publisher mag_pub;
     ros::Publisher pose_pub;
     ros::Publisher angle_pub;

     float dt;
     int _count;
public:
     // Contructor/DeConstructor
     BNO055_I2C_Ros(ros::NodeHandle nh, ros::NodeHandle _nh, int* pih = nullptr);
     ~BNO055_I2C_Ros();

     BNO055_I2C* imu;

     void update(bool verbose = false);
     int run(bool verbose = false);
};


#endif // BNO055_I2C_ROS_H_

#ifndef RTIMU_ROS_H_
#define RTIMU_ROS_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/PoseStamped.h>

#include <robocommander/base/definitions.h>
#include <robocommander/sensors/generic_rtimu.h>

using namespace std;

class RtImuRos{
private:
     float _dt;
     int _count;

     /** ROS Objects */
     ros::NodeHandle m_nh;
	ros::NodeHandle p_nh;
     ros::Rate* _loop_rate;
     ros::Publisher imu_pub;
     ros::Publisher mag_pub;
     ros::Publisher pose_pub;

     /** ROS Params */
     bool _publishTf;
     bool _verbose;
     std::string _tf_prefix;
     std::string _tf_imu;

     /** ROS Msgs */
     sensor_msgs::Imu imuMsg;
     sensor_msgs::MagneticField magMsg;
     geometry_msgs::PoseStamped poseMsg;
public:
     GenericRTIMU* imu;

     // Contructor/DeConstructor
     RtImuRos(ros::NodeHandle nh, ros::NodeHandle _nh);
     ~RtImuRos();

     void init_rosmsgs();
     int update(float timeout = 5.0);
     int run(bool verbose = false);
};

#endif // RTIMU_ROS_H_

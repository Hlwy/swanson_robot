#ifndef SWANSON_ALGORITHMS_ANGLE_TEST_H_
#define SWANSON_ALGORITHMS_ANGLE_TEST_H_

#include <thread>
#include <atomic>
#include <mutex>
#include <math.h>
#include <robocommander/utilities/utils.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

class AngleTestRos{
private:
     // Multi-threading objects
     std::mutex _lock;
     // ROS Objects
     ros::NodeHandle m_nh;
     ros::NodeHandle p_nh;
     ros::Rate* _loop_rate;

     ros::Subscriber _imu_sub;
     ros::Subscriber _pose_sub;
     ros::Subscriber _pose_stamped_sub;

     // ROS namespacing
     std::string _ns;
     std::string _parent_tf;
     std::string _camera_tf;

     void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
public:

     // Class Construction / Deconstruction
     AngleTestRos(ros::NodeHandle nh, ros::NodeHandle _nh);
     ~AngleTestRos();

     // Runtime Functions
     int run();
};

#endif // SWANSON_ALGORITHMS_ANGLE_TEST_H_

#ifndef VBOATS_ROS_H_
#define VBOATS_ROS_H_

#include <thread>
#include <atomic>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
// #include <boost/thread/mutex.hpp>

#include <RoboCommander/sensors/camera_d415.h>
#include <RoboCommander/algorithms/vboats/vboats.h>

using namespace std;

class VboatsRos{
private:
     ros::NodeHandle m_nh;
	ros::NodeHandle p_nh;
     ros::Rate* _loop_rate;

     ros::Publisher imu_pub;
     ros::Publisher pose_pub;

     cv::Mat _rgb;
     cv::Mat _depth;
     cv::Mat _disparity;

     float dt;
     int _count;
     // boost::mutex _lock;
     std::mutex _lock;
     std::thread _cam_thread;
     std::atomic_bool _stop_threads;
     std::atomic_bool _thread_started;
public:
     // Contructor/DeConstructor
     VboatsRos(ros::NodeHandle nh, ros::NodeHandle _nh);
     ~VboatsRos();

     CameraD415* cam;
     void cameraThreadFunction();
     void start();
     void stop();

     void update(bool verbose = false);
     int run(bool verbose = false);
};


#endif // VBOATS_ROS_H_

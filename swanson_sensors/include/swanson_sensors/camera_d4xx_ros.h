#ifndef SWANSON_SENSORS_CAMERA_D4XX_ROS_H_
#define SWANSON_SENSORS_CAMERA_D4XX_ROS_H_

#include <RoboCommander/sensors/camera_d4xx.h>

#include <thread>
#include <mutex>
#include <atomic>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>

using namespace std;

class CameraD4XXRos{
private:
     /** ROS Objects */
     ros::NodeHandle m_nh;
	ros::NodeHandle p_nh;
     tf::TransformBroadcaster _br;
     image_transport::ImageTransport _it;
     int _update_rate;
     ros::Rate* _loop_rate;

     ros::Publisher _rgb_info_pub;
     ros::Publisher _depth_info_pub;
     ros::Publisher _rgb_pub;
     ros::Publisher _depth_pub;
     ros::Publisher _disparity_pub;
     // image_transport::Publisher _rgb_pub;
     // image_transport::Publisher _depth_pub;
     // image_transport::Publisher _disparity_pub;

     /** ROS topics and tf frames */
     std::string _ns;
     std::string _camera_name;
     std::string _tf_prefix;
     std::string _parent_tf;
	std::string _cam_base_tf;
	std::string _rgb_base_tf;
	std::string _rgb_optical_tf;
	std::string _depth_base_tf;
	std::string _depth_optical_tf;
	std::string _aligned_base_tf;

     /** Internally Stored Images */
     cv::Mat _rgb;
     cv::Mat _depth;
     cv::Mat _disparity;
     /** Camera Intrinsic/Extrinsic Properties */
     float _dscale;
     float _baseline;
     float _fxd, _fyd, _pxd, _pyd;
	float _fxc, _fyc, _pxc, _pyc;
     cv::Mat _Krgb;
     cv::Mat _Prgb;
     cv::Mat _Kdepth;
     cv::Mat _Pdepth;
     float _focal[2];
     float _principle[2];

     /** Published Image Msg Header containers */
     std_msgs::Header _rgbImgHeader;
     std_msgs::Header _depthImgHeader;
     std_msgs::Header _disparityImgHeader;

     /** Camera Info msg containers */
     sensor_msgs::CameraInfo _rgb_info_msg;
     sensor_msgs::CameraInfo _depth_info_msg;
     /** TF Frame containers */
     tf::Transform _tfOpticalBaseToCamBase;
     tf::Transform _tfOpticalToOpticalBase;

     /** Counters and Timers */
     float dt;
     int _count;
     int _img_count;
     /** Flags */
     bool _verbose_timings;
     bool _publish_images;
     bool _use_float_depth;
     bool _use_8bit_depth;
     bool _get_aligned;
     bool _post_process;
     bool _publish_tf;
     bool _calc_disparity;
     /** Multi-threading objects */
     // boost::mutex _lock;
     std::mutex _lock;
     std::atomic_bool _stop_threads;
     std::atomic_bool _thread_started;
public:
     // Contructor/DeConstructor
     CameraD4XXRos(ros::NodeHandle nh, ros::NodeHandle _nh);
     ~CameraD4XXRos();

     CameraD4XX* cam;

     void start();
     void stop();

     void initTfs();
     void publish_tfs();
     void publish_images(cv::Mat _rgb, cv::Mat _depth, cv::Mat _disparity);
     void update(bool verbose = false);
     int run(bool verbose = false);
};


#endif // SWANSON_SENSORS_CAMERA_D4XX_ROS_H_

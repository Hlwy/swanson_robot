#ifndef VBOATS_ROS_H_
#define VBOATS_ROS_H_

#include <thread>
#include <atomic>
#include <mutex>
// #include <boost/thread/mutex.hpp>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>

#include <RoboCommander/sensors/camera_d415.h>
#include <RoboCommander/algorithms/vboats/vboats.h>

using namespace std;

class VboatsRos{
private:
     /** ROS Objects */
     ros::NodeHandle m_nh;
	ros::NodeHandle p_nh;
     tf::TransformBroadcaster _br;
     image_transport::ImageTransport _it;
     int _update_rate;
     ros::Rate* _loop_rate;

     image_transport::Publisher _rgb_pub;
     image_transport::Publisher _depth_pub;
     ros::Publisher _rgb_info_pub;
     ros::Publisher _depth_info_pub;
     image_transport::Publisher _disparity_pub;
     image_transport::Publisher _obstacles_img_pub;
     ros::Publisher _detected_obstacle_info_pub;

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
     cv::Mat _umap;
     cv::Mat _vmap;
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
     float _disparity2depth;

     /** Published Image Msg Header containers */
     std_msgs::Header _rgbImgHeader;
     std_msgs::Header _depthImgHeader;
     std_msgs::Header _disparityImgHeader;
     std_msgs::Header _obsImgHeader;

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
     bool _verbose_obstacles;
     bool _verbose_timings;
     bool _publish_images;
     bool _use_float_depth;
     bool _get_aligned;
     bool _publish_tf;
     bool _publish_obs_display;
     /** Multi-threading objects */
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
     VBOATS* vb;
     void cameraThreadFunction();
     void start();
     void stop();

     void initTfs();
     void publish_tfs();
     void publish_images(cv::Mat _rgb, cv::Mat _depth, cv::Mat _disparity);
     void publish_obstacle_image(cv::Mat image);
     void update(const cv::Mat& image, const cv::Mat& umap, const cv::Mat& vmap, float conversion_gain, bool verbose = false, bool debug_timing = false);
     // void update(const cv::Mat& image, float conversion_gain, bool is_disparity = true, bool verbose = false, bool debug_timing = false);
     int run(bool verbose = false);
};


#endif // VBOATS_ROS_H_

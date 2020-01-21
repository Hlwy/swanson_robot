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

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


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

     ros::Subscriber _depth_sub;
     ros::Subscriber _cam_info_sub;
     ros::Subscriber _disparity_sub;
     ros::Publisher _umap_pub;
     ros::Publisher _vmap_pub;
     ros::Publisher _new_img_pub;
     ros::Publisher _obstacles_img_pub;
     ros::Publisher _detected_obstacle_info_pub;
     ros::Publisher _cloud_pub;
     ros::Publisher _filtered_cloud_pub;

     /** ROS topics and tf frames */
     std::string _ns;
     std::string _parent_tf;
	std::string _camera_tf;

     /** Internally Stored Images */
     cv::Mat _umap;
     cv::Mat _vmap;
     cv::Mat _depth;
     cv::Mat _disparity;
     /** Camera Intrinsic/Extrinsic Properties */
     float _dscale;
     float _baseline;
     float _fx, _fy, _px, _py;
     float _focal[2];
     float _principle[2];
     float _depth2disparityFactor;

     /** Published Image Msg Header containers */
     std_msgs::Header _umapImgHeader;
     std_msgs::Header _vmapImgHeader;
     std_msgs::Header _filteredImgHeader;
     std_msgs::Header _obsImgHeader;

     /** Counters and Timers */
     float dt;
     int _count;
     int _img_count;
     int _info_count;
     /** Flags */
     bool _verbose_obstacles;
     bool _verbose_timings;
     bool _publish_images;
     bool _visualize_images;
     bool _recvd_cam_info;
     bool _flag_depth_based;
     bool _flag_pub_cloud;
     bool _flag_pub_filtered_cloud;
     float _max_obstacle_height;
     /** Multi-threading objects */
     std::mutex _lock;

     /** ROS Subscriber Callbacks */
     void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg, const int value);
     void depthCallback(const sensor_msgs::Image::ConstPtr& msg, const int value);
     void disparityCallback(const sensor_msgs::Image::ConstPtr& msg, const int value);
     void depth_to_disparity(const cv::Mat& depth, cv::Mat* disparity, float gain);
public:
     // Contructor/DeConstructor
     VboatsRos(ros::NodeHandle nh, ros::NodeHandle _nh);
     ~VboatsRos();

     VBOATS* vb;

     void publish_images(const cv::Mat& umap, const cv::Mat& vmap, const cv::Mat& filtered);
     void publish_obstacle_image(cv::Mat image);
     void publish_obstacle_data(vector<Obstacle> obstacles);

     int remove_ground(const cv::Mat& disparity, const cv::Mat& vmap, const cv::Mat& depth, float* line_params);
     void generate_pointcloud(const cv::Mat& depth);

     int process(const cv::Mat& disparity, const cv::Mat& umap, const cv::Mat& vmap, vector<Obstacle>* obstacles, const cv::Mat& depth);
     int update(bool verbose = false, bool debug_timing = true);

     // void update(const cv::Mat& image, const cv::Mat& umap, const cv::Mat& vmap, float conversion_gain, bool verbose = false, bool debug_timing = false);
     int run(bool verbose = false);
};


#endif // VBOATS_ROS_H_

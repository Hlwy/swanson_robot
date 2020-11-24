#ifndef SWANSON_ALGORITHMS_VBOATS_ROS_H_
#define SWANSON_ALGORITHMS_VBOATS_ROS_H_

#include <thread>
#include <atomic>
#include <mutex>
#include <math.h>
#include <robocommander/algorithms/vboats/vboats.h>
#include <robocommander/algorithms/vboats/obstacle.h>
#include <robocommander/algorithms/vboats/vboats_utils.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>

#include <swanson_algorithms/VboatsConfig.h>
#include <swanson_msgs/VboatsObstacle.h>
#include <swanson_msgs/VboatsObstacles.h>

class VboatsRos{
private:
     /** Multi-threading objects */
     std::mutex _lock;
     /** ROS Objects */
     ros::NodeHandle m_nh;
     ros::NodeHandle p_nh;
     ros::Rate* _loop_rate;
     tf::TransformBroadcaster _br;
     ros::Subscriber _depth_sub;
     ros::Subscriber _cam_info_sub;

     ros::Publisher _raw_umap_pub;
     ros::Publisher _raw_vmap_pub;
     ros::Publisher _umap_pub;
     ros::Publisher _vmap_pub;

     ros::Publisher _gnd_filtered_img_pub;
     ros::Publisher _obstacles_img_pub;
     ros::Publisher _detected_obstacle_info_pub;
     ros::Publisher _generated_disparity_pub;

     ros::Publisher _raw_cloud_pub;
     ros::Publisher _unfiltered_cloud_pub;
     ros::Publisher _filtered_cloud_pub;

     ros::Subscriber _imu_sub;
     ros::Subscriber _pose_sub;
     ros::Subscriber _pose_stamped_sub;
     ros::Subscriber _disparity_sub;

     image_transport::ImageTransport _it;
     dynamic_reconfigure::Server<swanson_algorithms::VboatsConfig> _cfg_server;
     dynamic_reconfigure::Server<swanson_algorithms::VboatsConfig>::CallbackType _cfg_f;

     /** ROS namespacing */
     std::string _ns;
     std::string _parent_tf;
     std::string _camera_tf;

     /** Counters / Timers / Loop Exiting */
     int _count               = 0;
     cv::Mat _depth;
     cv::Mat _generated_disparity;
     cv::Mat _umap_raw;
     cv::Mat _vmap_raw;
     /** Depth Pre-Processing Parameters */
     double _cam_min_depth_x            = 0.0;
     double _cam_max_depth_x            = 0.0;
     double _cam_min_depth_y            = 0.0;
     double _cam_max_depth_y            = 0.0;
     /** Pointcloud Filtering Parameters */
     bool _do_cloud_limit_filtering     = true;
     float _max_cloud_height            = 1.0;
     float _min_cloud_height            = 1.0;
     float _max_cloud_range             = 15.0;
     float _min_cloud_range             = 0.1;
     bool _do_cloud_downsampling        = true;
     float _voxel_res_x                 = 0.02;
     float _voxel_res_y                 = 0.02;
     float _voxel_res_z                 = 0.02;
     bool _do_cloud_outlier_removal     = true;
     int _cloud_outlier_min_neighbors   = 50;
     float _cloud_outlier_search_radius = 1.0;

     /** Debug Flags */
     bool _verbose_update                    = false;
     bool _debug_angle_inputs                = false;
     bool _verbose_obstacles                 = false;
     bool _debug_timings                     = false;

     bool _flag_pub_raw_cloud                = false;
     bool _flag_pub_unfiltered_cloud         = false;
     bool _flag_pub_filtered_cloud           = false;
     bool _publish_obs_data                  = false;
     bool _publish_obstacle_segmented_image  = false;

     bool _publish_generated_disparity       = false;
     bool _publish_umap_raw                  = false;
     bool _publish_vmap_raw                  = false;
     bool _publish_umap_processed            = false;
     bool _publish_vmap_processed            = false;
     bool _overlay_gnd_lines                 = false;
     bool _overlay_filtered_contours         = false;
     bool _overlay_object_search_windows     = false;

     bool _publish_mid_level_debug_images    = false;
     bool _visualize_gnd_line_keep_mask      = false;
     bool _visualize_obj_candidate_keep_mask = false;
     bool _visualize_umap_keep_mask          = false;
     bool _visualize_vmap_keep_mask          = false;

     bool _publish_low_level_debug_images                   = false;
     bool _visualize_umap_sobel_raw                         = false;
     bool _visualize_umap_sobel_preprocessed                = false;
     bool _visualize_umap_sobel_dilated                     = false;
     bool _visualize_umap_sobel_blurred                     = false;
     bool _visualize_vmap_sobelized_preprocessed            = false;
     bool _visualize_vmap_sobelized_postprocessed_threshed  = false;
     bool _visualize_vmap_sobelized_postprocessed_blurred   = false;

     /** ROS Subscriber Callbacks */
     void cfgCallback(swanson_algorithms::VboatsConfig &config, uint32_t level);
     void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg, const int value);
     void depthCallback(const sensor_msgs::Image::ConstPtr& msg, const int value);

     void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
     void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
     void _publish_image(ros::Publisher publisher, const cv::Mat& image);
     void _publish_pointcloud(ros::Publisher publisher, cloudxyz_t::Ptr inputCloud);
public:
     Vboats* vb;

     /** Class Construction / Deconstruction */
     VboatsRos(ros::NodeHandle nh, ros::NodeHandle _nh);
     ~VboatsRos();

     /** ROS Interface Helper Functions */
     void publish_pointclouds(cv::Mat raw_depth, cv::Mat filtered_depth);
     void publish_auxillery_images();
     void publish_debugging_images();

     /** Data Generation Functions */
     cloudxyz_t::Ptr filter_pointcloud(cloudxyz_t::Ptr inputCloud, bool debug_timing = false);

     /** Runtime Functions */
     int update();
     int run();
};

#endif // SWANSON_ALGORITHMS_VBOATS_ROS_H_

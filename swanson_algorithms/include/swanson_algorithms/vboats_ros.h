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

#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/MarkerArray.h>

#include <swanson_algorithms/VboatsConfig.h>
#include <swanson_msgs/VboatsObstacle.h>
#include <swanson_msgs/VboatsObstacles.h>

class VboatsRos{
private:
     // Multi-threading objects
     std::mutex _lock;
     // ROS Objects
     ros::NodeHandle m_nh;
     ros::NodeHandle p_nh;
     double _update_rate;

     ros::Subscriber _depth_sub;
     ros::Subscriber _cam_info_sub;
     ros::Subscriber _imu_sub;
     ros::Subscriber _pose_sub;
     ros::Subscriber _pose_stamped_sub;

     sensor_msgs::CameraInfo::ConstPtr _filtered_depth_info;
     // image_transport::Publisher _filtered_depth_pub;
     image_transport::CameraPublisher _filtered_depth_pub;
     ros::Publisher _raw_cloud_pub;
     ros::Publisher _unfiltered_cloud_pub;
     ros::Publisher _filtered_cloud_pub;
     ros::Publisher _detected_obstacle_info_pub;
     ros::Publisher _obstacle_markers_pub;
     ros::Publisher _corrected_depth_pub;
     ros::Publisher _generated_disparity_pub;
     ros::Publisher _raw_umap_pub;
     ros::Publisher _raw_vmap_pub;
     ros::Publisher _proc_umap_pub;
     ros::Publisher _proc_vmap_pub;

     ros::Publisher _gnd_line_mask_pub;
     ros::Publisher _gnd_line_vmask_pub;
     ros::Publisher _obj_candidate_mask_pub;
     ros::Publisher _umap_keep_mask_pub;
     ros::Publisher _vmap_keep_mask_pub;
     ros::Publisher _umap_debug_pub;
     ros::Publisher _vmap_debug_pub;

     bool _is_node_paused                                   = false;
     ros::ServiceServer _pause_service;
     ros::ServiceServer _resume_service;
     ros::ServiceServer _correction_angle_calibration_service;

     image_transport::ImageTransport _it;
     dynamic_reconfigure::Server<swanson_algorithms::VboatsConfig> _cfg_server;
     dynamic_reconfigure::Server<swanson_algorithms::VboatsConfig>::CallbackType _cfg_f;

     // ROS namespacing
     std::string _ns;
     std::string _parent_tf;
     std::string _camera_tf;

     // Counters / Timers / Loop Exiting
     cv::Mat _depth;
     int _count                                             = 0;
     int _last_markers_count                                = 0;
     bool _do_angle_offsets_calibration                     = false;
     bool _angle_offsets_calibration_performed              = false;
     int _angle_offsets_count                               = 0;
     int _angle_offset_samples                              = 10;
     double _time_offset                                    = 0.0;
     double _avg_roll                                       = 0.0;
     double _avg_pitch                                      = 0.0;
     double _avg_yaw                                        = 0.0;
     double _user_angle_offset                              = 0.0;
     float _prev_gnd_line_slope                             = 0.0;
     int _prev_gnd_line_intercept                           = 0;

     // Pointcloud Filtering Parameters
     bool _do_cloud_limit_filtering                         = true;
     float _max_cloud_height                                = 1.0;
     float _min_cloud_height                                = 1.0;
     float _max_cloud_range                                 = 15.0;
     float _min_cloud_range                                 = 0.1;
     bool _do_cloud_downsampling                            = true;
     float _voxel_res_x                                     = 0.02;
     float _voxel_res_y                                     = 0.02;
     float _voxel_res_z                                     = 0.02;
     bool _do_cloud_outlier_removal                         = true;
     int _cloud_outlier_min_neighbors                       = 50;
     float _cloud_outlier_search_radius                     = 1.0;

     // Debug Flags
     bool _verbose_update                                   = false;
     bool _verbose_obstacles                                = false;
     bool _debug_timings                                    = false;
     bool _debug_angle_inputs                               = false;
     bool _debug_ground_line_params                         = false;
     bool _debug_disparity_generation                       = false;

     bool _flag_pub_raw_cloud                               = false;
     bool _flag_pub_unfiltered_cloud                        = false;
     bool _flag_pub_filtered_cloud                          = false;
     bool _publish_obs_data                                 = false;
     bool _publish_obstacle_markers                         = false;

     bool _publish_corrected_depth                          = false;
     bool _publish_generated_disparity                      = false;
     bool _publish_umap_raw                                 = false;
     bool _publish_vmap_raw                                 = false;
     bool _visualize_umap_raw                               = false;
     bool _visualize_vmap_raw                               = false;
     bool _publish_umap_processed                           = false;
     bool _publish_vmap_processed                           = false;
     bool _overlay_gnd_lines                                = false;
     bool _overlay_filtered_contours                        = false;
     bool _overlay_object_search_windows                    = false;
     bool _overlay_obstacle_bounding_boxes                  = false;

     bool _publish_mid_level_debug_images                   = false;
     bool _visualize_gnd_line_keep_mask                     = false;
     bool _visualize_obj_candidate_keep_mask                = false;
     bool _visualize_umap_keep_mask                         = false;
     bool _visualize_vmap_keep_mask                         = false;

     bool _publish_low_level_debug_images                   = false;
     bool _show_uvmap_debug_titles                          = false;
     bool _visualize_umap_final                             = false;
     bool _visualize_vmap_final                             = false;
     bool _visualize_umap_sobel_raw                         = false;
     bool _visualize_umap_sobel_preprocessed                = false;
     bool _visualize_umap_sobel_dilated                     = false;
     bool _visualize_umap_sobel_blurred                     = false;
     bool _visualize_vmap_sobelized_preprocessed            = false;
     bool _visualize_vmap_sobelized_postprocessed_threshed  = false;
     bool _visualize_vmap_sobelized_postprocessed_blurred   = false;

     // ROS Subscriber Callbacks
     void cfgCallback(swanson_algorithms::VboatsConfig &config, uint32_t level);
     void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg, const int value);
     void depthCallback(const sensor_msgs::Image::ConstPtr& msg, const int value);

     void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
     void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

     template<typename ROS_OBJ> void _publish_image(ROS_OBJ publisher, const cv::Mat& image, bool colorize = false);
     template<typename ROS_OBJ> void _publish_image(ROS_OBJ publisher, const cv::Mat& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info, bool colorize = false);
     // void _publish_image(image_transport::Publisher publisher, const cv::Mat& image, bool colorize = false);
     void _publish_extracted_obstacle_data(ros::Publisher publisher, std::vector<Obstacle> obstacles);
     void _publish_pointcloud(ros::Publisher publisher, cloudxyz_t::Ptr inputCloud);

public:
     Vboats* vb;

     // Class Construction / Deconstruction
     VboatsRos(ros::NodeHandle nh, ros::NodeHandle _nh);
     ~VboatsRos();

     // Data Generation Functions
     cloudxyz_t::Ptr filter_pointcloud(cloudxyz_t::Ptr inputCloud, bool debug_timing = false);

     // Runtime Functions
     bool calibrate_orientation_offsets_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
     bool pause_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
     bool resume_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
     int update();
     int run();

     // ROS Interface Helper Functions
     void publish_pointclouds(cv::Mat raw_depth, cv::Mat filtered_depth);
     void publish_auxillery_images(const cv::Mat& disparity_gen, const cv::Mat& umap_proc, const cv::Mat& vmap_proc);
     void publish_debugging_images();
     void visualize_obstacle_markers(const std::vector<Obstacle>& obstacles);
};

#endif // SWANSON_ALGORITHMS_VBOATS_ROS_H_

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

typedef pcl::PointCloud<pcl::PointXYZ> cloudxyz_t;

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
     /** Common ROS Msg Header containers */
     std_msgs::Header _obsImgHeader;
     std_msgs::Header _umapImgHeader;
     std_msgs::Header _vmapImgHeader;
     std_msgs::Header _filteredImgHeader;

     /** Camera Intrinsic/Extrinsic Properties */
     float _dscale;
     float _baseline;
     float _fx, _fy, _px, _py;
     float _focal[2];
     float _principle[2];
     float _depth2disparityFactor;
     /** Internally Stored Images */
     cv::Mat _umap;
     cv::Mat _vmap;
     cv::Mat _depth;
     cv::Mat _disparity;
     std::vector<Obstacle> _segmented_obstacle_data;

     /** Counters / Timers / Loop Exiting */
     float dt                 = 0.0;
     int _count               = 0;
     int _img_count           = 0;
     int _info_count          = 0;
     bool _recvd_image        = false;
     bool _recvd_cam_info     = false;

     /** Depth Pre-Processing Parameters */
     bool _do_angle_correction          = false;
     double _correction_angle_offset    = 0.0;
     double _correction_roll            = 0.0;
     double _correction_pitch           = 0.0;
     double _correction_yaw             = 0.0;
     double _disparity_hard_min         = 0.1;
     double _disparity_hard_max         = 15.0;
     double _cam_min_depth              = 0.1;
     double _cam_max_depth              = 15.0;
     double _cam_min_depth_x            = 0.0;
     double _cam_max_depth_x            = 0.0;
     double _cam_min_depth_y            = 0.0;
     double _cam_max_depth_y            = 0.0;
     /** U-Map Pre-Processing Flags */
     vector<float> _uThreshs;
     float _simple_umap_thresh_ratio    = 0.07;
     bool _use_custom_umap_filtering    = true;
     int _umap_contour_filter_method    = 1;
     float _umap_contour_min_thresh     = 40.0;
     float _umap_pre_blur_thresh        = 0.0;
     float _umap_post_blur_thresh       = 0.0;
     float _umap_sobel_thresh           = 0.0;
     bool _do_umap_primary_blur         = false;
     bool _do_umap_secondary_dilate     = false;
     bool _do_umap_secondary_blur       = false;
     int _umap_mask_primary_blur_size   = 1;
     int _umap_mask_secondary_blur_size = 1;
     int _kernel_x_multiplier           = 1;
     int _kernel_y_multiplier           = 1;
     int _umap_mask_secondary_dilate_size = 1;
     /** V-Map Pre-Processing Flags */
     vector<float> _vThreshs;
     bool _use_custom_vmap_filtering    = true;
     float _vmap_sobel_thresh           = 30;
     float _vmap_thresh                 = 0.0;
     bool _do_sobel_pre_thresholding    = true;
     float _sobel_pre_thresh            = 0.0;
     float _vmap_sobel_sec_thresh       = 0.0;
     bool _do_vmap_sobel_dilation       = false;
     bool _do_vmap_sobel_sec_dilation   = false;
     int _sobel_dilate_kernel_size      = 3;
     int _sobel_sec_dilate_kernel_size  = 1;
     bool _do_sobel_vmask_subtraction   = true;
     bool _do_vmap_sobel_blurring       = false;
     bool _do_vmap_sobel_sec_blurring   = false;
     bool _do_vmap_sobel_sec_thresh     = false;
     int _sobel_blur_kernel_size        = 3;
     int _sobel_sec_blur_kernel_size    = 3;
     /** Ground-Segmentation Parameters */
     bool _use_gnd_line_based_removal   = true;
     double _gnd_line_max_ang           = 89.0;
     double _gnd_line_min_ang           = 26.0;
     int _gnd_line_upper_offset         = 5;
     int _gnd_line_lower_offset         = 25;
     bool _do_object_segmented_filtering = true;
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

     /** Algorithm Flags */
     bool _do_post_depth_morphing            = true;
     int _depth_morph_kernel_size            = 3;
     bool _do_individual_obstacle_detection  = true;

     /** ROS Flags */
     bool _publish_aux_images                = false;
     bool _flag_pub_raw_cloud                = false;
     bool _flag_pub_unfiltered_cloud         = false;
     bool _flag_pub_filtered_cloud           = false;
     bool _publish_obs_data                  = false;
     bool _publish_obstacle_segmented_image  = false;

     /** Debug Flags */
     bool _debug                             = false;
     bool _verbose                           = false;
     bool _verbose_update                    = false;
     bool _debug_angle_inputs                = false;
     bool _do_cv_wait_key                    = false;
     bool _do_vmap_viz                       = false;
     bool _do_umap_viz                       = false;
     bool _do_misc_viz                       = false;
     bool _verbose_obstacles                 = false;
     bool _verbose_gnd_line_removal          = false;
     bool _debug_timings                     = false;
     bool _debug_gnd_line_removal            = false;
     bool _debug_disparity_generation        = false;
     bool _debug_published_cloud_sizes       = false;
     bool _visualize_filtered_depth          = false;
     bool _visualize_inputs                  = false;
     bool _visualize_generated_disparity     = false;
     bool _visualize_umap_contours           = false;
     bool _visualize_angle_corrected_depth   = false;
     bool _visualize_umap_raw                = false;
     bool _visualize_umap_filtered           = false;
     bool _visualize_vmap_raw                = false;
     bool _visualize_vmap_blurred            = false;
     bool _visualize_vmap_dilated            = false;
     bool _visualize_vmap_thresh             = false;
     bool _visualize_vmap_sec_thresh         = false;
     bool _visualize_vmap_sec_dilated        = false;
     bool _visualize_vmap_sec_blur           = false;
     bool _visualize_vmap_mask               = false;
     bool _visualize_sobel_processed         = false;
     bool _visualize_vmap_raw_w_lines        = false;
     bool _visualize_vmap_sobel_raw          = false;
     bool _visualize_vmap_sobel_filtered     = false;
     bool _visualize_obj_detection_vmap      = false;
     bool _visualize_process_input_vmap      = false;
     bool _visualize_gnd_mask                = false;
     bool _visualize_gnd_filter_img          = false;
     bool _visualize_obj_mask                = false;
     bool _visualize_obj_filter_img          = false;
     bool _visualize_obstacle_search_windows = false;
     bool _visualize_segmented_obstacle_regions = false;

     /** ROS Subscriber Callbacks */
     void cfgCallback(swanson_algorithms::VboatsConfig &config, uint32_t level);
     void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg, const int value);
     void depthCallback(const sensor_msgs::Image::ConstPtr& msg, const int value);
     void disparityCallback(const sensor_msgs::Image::ConstPtr& msg, const int value);

     void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
     void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

public:
     Vboats* vb;

     /** Class Construction / Deconstruction */
     VboatsRos(ros::NodeHandle nh, ros::NodeHandle _nh);
     ~VboatsRos();

     /** ROS Interface Helper Functions */
     void publish_images(cv::Mat* umap = nullptr, cv::Mat* vmap = nullptr,
          cv::Mat* filtered_depth = nullptr, cv::Mat* filtered_umap = nullptr, cv::Mat* filtered_vmap = nullptr
     );
     void publish_obstacle_image(cv::Mat image);
     void publish_pointclouds(cv::Mat raw_depth, cv::Mat filtered_depth);

     /** Data Generation Functions */
     void depth_to_disparity(const cv::Mat& depth, cv::Mat* disparity, float gain);
     cloudxyz_t::Ptr generate_cloud_from_depth(const cv::Mat& depth);
     cloudxyz_t::Ptr filter_pointcloud(cloudxyz_t::Ptr inputCloud);

     /** Image Processing Functions */
     int remove_objects(const cv::Mat& vmap, const cv::Mat& disparity,
          const cv::Mat& depth, const vector<vector<cv::Point>>& contours,
          std::vector<float> line_params, cv::Mat* filtered_img = nullptr,
          cv::Mat* generated_mask = nullptr, bool debug_timing = false
     );
     int remove_ground(const cv::Mat& disparity, const cv::Mat& vmap,
          const cv::Mat& depth, std::vector<float> line_params,
          cv::Mat* filtered_img = nullptr, cv::Mat* generated_mask = nullptr
     );

     int process(const cv::Mat& depth, const cv::Mat& disparity,
          const cv::Mat& umap, const cv::Mat& vmap, vector<Obstacle>* obstacles = nullptr,
          cv::Mat* filtered = nullptr, cv::Mat* processed_umap = nullptr, cv::Mat* processed_vmap = nullptr
     );

     /** Runtime Functions */
     int update();
     int run();
};

#endif // SWANSON_ALGORITHMS_VBOATS_ROS_H_

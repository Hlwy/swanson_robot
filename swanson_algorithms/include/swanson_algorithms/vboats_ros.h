#ifndef VBOATS_ROS_H_
#define VBOATS_ROS_H_

#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>
#include <bits/stdc++.h> // For memset

#include <opencv2/opencv.hpp>
#include <opencv2/core/version.hpp>
#include <RoboCommander/algorithms/vboats/vboats.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>

#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <swanson_algorithms/VboatsConfig.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

typedef pcl::PointCloud<pcl::PointXYZ> cloudxyz_t;

using namespace std;

class VboatsRos{
private:
     // std::string classLbl = txt_bold_magenta() + "VboatsHandler" + txt_reset_color();
     std::string classLbl = "VboatsHandler";
     /** ROS Objects */
     ros::NodeHandle m_nh;
     ros::NodeHandle p_nh;
     ros::Rate* _loop_rate;
     ros::Subscriber _imu_sub;
     ros::Subscriber _pose_sub;
     ros::Subscriber _pose_stamped_sub;
     ros::Subscriber _depth_sub;
     ros::Subscriber _cam_info_sub;
     ros::Subscriber _disparity_sub;
     ros::Publisher _umap_pub;
     ros::Publisher _vmap_pub;
     ros::Publisher _new_img_pub;
     ros::Publisher _obstacles_img_pub;
     ros::Publisher _detected_obstacle_info_pub;
     ros::Publisher _raw_cloud_pub;
     ros::Publisher _filtered_cloud_pub;
     ros::Publisher _unfiltered_cloud_pub;
     tf::TransformBroadcaster _br;

     image_transport::ImageTransport _it;
     dynamic_reconfigure::Server<swanson_algorithms::VboatsConfig> _cfg_server;
     dynamic_reconfigure::Server<swanson_algorithms::VboatsConfig>::CallbackType _cfg_f;

     /** ROS topics and tf frames */
     std::string _ns;
     std::string _parent_tf;
     std::string _camera_tf;

     /** Internally Stored Images */
     cv::Mat _umap;
     cv::Mat _vmap;
     cv::Mat _depth;
     cv::Mat _disparity;
     std::vector<Obstacle> _segmented_obstacle_data;
     /** Camera Intrinsic/Extrinsic Properties */
     float _dscale;
     float _baseline;
     float _fx, _fy, _px, _py;
     float _focal[2];
     float _principle[2];
     float _depth2disparityFactor = 1.0;

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
     bool _debug_image_info;
     /** Configurable Parameters */
     bool _do_cloud_downsampling;
     bool _do_cloud_outlier_removal;
     bool _do_cloud_limit_filtering;
     bool _do_angle_correction;
     double _correction_roll = 0.0;
     double _correction_pitch = 0.0;
     double _correction_yaw = 0.0;
     double _cam_min_depth;
     double _cam_max_depth;
     double _cam_min_depth_x;
     double _cam_max_depth_x;
     double _cam_min_depth_y;
     double _cam_max_depth_y;
     int _contourFiltMeth;
     float _contourFiltMinThresh;
     float _max_obstacle_height;
     float _min_obstacle_height;
     float _max_obstacle_range;
     float _min_obstacle_range;
     float _voxel_res_x;
     float _voxel_res_y;
     float _voxel_res_z;
     int _gnd_line_upper_offset;
     int _gnd_line_lower_offset;
     double _gnd_line_max_ang;
     double _gnd_line_min_ang;
     int _cloud_outlier_min_neighbors;
     float _cloud_outlier_search_radius;
     vector<float> _uThreshs;
     vector<float> _vThreshs;
     bool _use_custom_umap_filtering = true;
     bool _primary_blur;
     int _umask_primary_blur_size;
     float _pre_blur_thresh;
     float _post_blur_thresh;
     float _umap_sobel_thresh;
     bool _secondary_dilate;
     bool _secondary_blur;
     int _umask_secondary_blur_size;
     int _umask_secondary_dilate_size;
     int _kernel_x_multiplier;
     int _kernel_y_multiplier;

     bool _use_custom_vmap_filtering = true;
     bool _do_sobel_vmask_subtraction;
     float _custom_uThresh_perc;
     float _vmap_sobel_thresh;
     float _vmap_thresh = 0.0;
     bool _do_sobel_pre_thresholding = true;
     float _sobel_pre_thresh = 0.0;
     float _vmap_sobel_sec_thresh = 0;
     bool _do_vmap_sobel_dilation;
     bool _do_vmap_sobel_sec_dilation = false;
     int _sobel_dilate_kernel_size;
     int _sobel_sec_dilate_kernel_size = 1;
     bool _do_vmap_sobel_blurring            = false;
     bool _do_vmap_sobel_sec_blurring            = false;
     bool _do_vmap_sobel_sec_thresh            = false;
     int _vmap_sobel_passes                  = 3;
     int _sobel_blur_kernel_size             = 3;
     int _sobel_sec_blur_kernel_size             = 3;
     bool _use_scharr                         = false;
     bool _use_filtered_depth_object_detection             = false;
     bool _do_post_depth_morphing             = false;
     int _depth_morph_kernel_size             = 3;
     /** Flags Original*/
     bool _recvd_cam_info;
     bool _recvd_image;
     bool _publish_aux_images;
     bool _flag_pub_raw_cloud;
     bool _flag_pub_unfiltered_cloud;
     bool _flag_pub_filtered_cloud;

     bool _publish_obs_data = true;
     bool _publish_obstacle_segmented_image  = true;
     bool _do_individual_obstacle_detection  = true;
     bool _do_object_segmented_filtering     = true;
     bool _do_depth_based_processing         = true;
     bool _use_gnd_line_based_removal        = true;
     bool _flip_gnd_line_mask        = true;

     /** Flags */
     bool _debug;
     bool _verbose;
     bool _verbose_update;
     bool _do_cv_wait_key = false;
     bool _do_vmap_viz = false;
     bool _do_umap_viz = false;
     bool _do_misc_viz = false;
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
     bool _visualize_gnd_mask = false;
     bool _visualize_gnd_filter_img = false;
     bool _visualize_obj_mask = false;
     bool _visualize_obj_filter_img = false;
     bool _visualize_obstacle_search_windows = false;
     bool _visualize_segmented_obstacle_regions = false;

     /** Multi-threading objects */
     std::mutex _lock;

     /** ROS Subscriber Callbacks */
     void cfgCallback(swanson_algorithms::VboatsConfig &config, uint32_t level);
     void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg, const int value);
     void depthCallback(const sensor_msgs::Image::ConstPtr& msg, const int value);
     void disparityCallback(const sensor_msgs::Image::ConstPtr& msg, const int value);
     void zedDisparityCallback(const stereo_msgs::DisparityImage::ConstPtr& msg);

     void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
     void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
     void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

public:
     // Contructor/DeConstructor
     VboatsRos(ros::NodeHandle nh, ros::NodeHandle _nh);
     ~VboatsRos();

     VBOATS* vb;

     // void publish_images(const cv::Mat& umap, const cv::Mat& vmap, const cv::Mat& filtered);
     void publish_images(cv::Mat* umap = nullptr, cv::Mat* vmap = nullptr, cv::Mat* filtered = nullptr);
     void publish_obstacle_image(cv::Mat image);

     void depth_to_disparity(const cv::Mat& depth, cv::Mat* disparity, float gain);
     cloudxyz_t::Ptr generate_cloud_from_depth(const cv::Mat& depth);
     cloudxyz_t::Ptr filter_pointcloud(cloudxyz_t::Ptr inputCloud);

     int remove_ground(const cv::Mat& disparity, const cv::Mat& vmap, const cv::Mat& depth,
          float* line_params, cv::Mat* filtered_img, cv::Mat* generated_mask = nullptr
     );
     int remove_objects(const cv::Mat& vmap, const cv::Mat& disparity,
          const cv::Mat& depth, const vector<vector<cv::Point>>& contours,
          float* line_params, cv::Mat* filtered_img,
          cv::Mat* generated_mask = nullptr, bool debug_timing = false
     );

     // int process(const cv::Mat& disparity, const cv::Mat& umap, const cv::Mat& vmap, vector<Obstacle>* obstacles, const cv::Mat& depth);
     int process(const cv::Mat& depth, const cv::Mat& disparity,
          const cv::Mat& umap, const cv::Mat& vmap, vector<Obstacle>* obstacles = nullptr,
          cv::Mat* filtered = nullptr, cv::Mat* processed_umap = nullptr, cv::Mat* processed_vmap = nullptr
     );
     int update(bool verbose = false, bool debug_timing = true);

     int run(bool verbose = false);
};


#endif // VBOATS_ROS_H_

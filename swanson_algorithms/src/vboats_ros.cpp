#include <math.h>
#include <iostream>

#include "swanson_algorithms/vboats_ros.h"

#include <RoboCommander/utilities/utils.h>
#include <RoboCommander/utilities/image_utils.h>
#include <RoboCommander/algorithms/vboats/uvmap_utils.h>

#include <swanson_msgs/VboatsObstacle.h>
#include <swanson_msgs/VboatsObstacles.h>

#define M_DEG2RAD (2*M_PI)/360
#define M_RAD2DEG 360/(2*M_PI)

using namespace std;

/** SECTION:
     CONSTRUCTOR & DECONSTRUCTOR
*/
VboatsRos::VboatsRos(ros::NodeHandle nh, ros::NodeHandle _nh) : m_nh(nh), p_nh(_nh), _it(nh), _focal(), _principle(){
     // Initialize internal constants
     _count = 0;
     _img_count = 0;
     _info_count = 0;
     _debug = false;
     _verbose = false;
     _use_gnd_line_based_removal = true;
     _recvd_image = false;
     _recvd_cam_info = false;
     _do_depth_based_processing = true;

     /** Flag Configuration */
     std::string namespaced = m_nh.getNamespace();
     bool flag_verbose_obstacles = false;
     bool flag_verbose_timings = false;
     bool flag_use_tf_prefix = false;
     bool flag_publish_imgs = true;
     bool publish_aux_images = false;
     bool visualize_imgs = false;
     bool disparity_based = false;
     bool gnd_method = true;
     bool flag_detect_obstacles = false;
     bool flag_filter_ground = true;
     bool flag_filter_cloud = false;
     bool publish_raw_cloud = false;
     bool publish_filtered_cloud = false;
     bool publish_unfiltered_cloud = false;
     int update_rate = 30;

     p_nh.getParam("namespace",namespaced);
     p_nh.getParam("verbose_obstacles",flag_verbose_obstacles);
     p_nh.getParam("verbose_timings",flag_verbose_timings);
     p_nh.getParam("publish_obstacle_image",flag_publish_imgs);
     p_nh.getParam("publish_aux_images",publish_aux_images);
     p_nh.getParam("show_images",visualize_imgs);
     p_nh.getParam("update_rate",update_rate);
     p_nh.getParam("use_disparity",disparity_based);
     p_nh.getParam("gnd_filter_meth",gnd_method);

     p_nh.getParam("detect_obstacles",flag_detect_obstacles);
     p_nh.getParam("publish_raw_cloud",publish_raw_cloud);
     p_nh.getParam("publish_filtered_cloud",publish_filtered_cloud);
     p_nh.getParam("publish_unfiltered_cloud",publish_unfiltered_cloud);

     this->_ns = namespaced;
     this->_verbose_obstacles = flag_verbose_obstacles;
     this->_debug_timings = flag_verbose_timings;
     this->_publish_obstacle_segmented_image = flag_publish_imgs;
     this->_publish_aux_images = publish_aux_images;
     this->_use_gnd_line_based_removal = gnd_method;
     this->_do_depth_based_processing = !disparity_based;
     // this->_filter_ground = flag_filter_ground;
     // this->_filter_cloud = flag_filter_cloud;
     this->_flag_pub_raw_cloud = publish_raw_cloud;
     this->_flag_pub_filtered_cloud = publish_filtered_cloud;
     this->_flag_pub_unfiltered_cloud = publish_unfiltered_cloud;
     this->_do_individual_obstacle_detection = flag_detect_obstacles;

     float max_obs_height = 1.0;
     float min_obs_height = 1.0;
     float max_obs_range = 10.0;
     float min_obs_range = 0.1;
     float voxel_resx = 0.05;
     float voxel_resy = 0.05;
     float voxel_resz = 0.05;
     int gnd_buffer_upper_offset = 22;
     int gnd_buffer_lower_offset = 25;
     int sor_mink = 50;
     float sor_thresh = 1.0;

     p_nh.getParam("max_obstacle_height",max_obs_height);
     p_nh.getParam("min_obstacle_height",min_obs_height);
     p_nh.getParam("max_obstacle_range",max_obs_range);
     p_nh.getParam("min_obstacle_range",min_obs_range);
     p_nh.getParam("voxel_resolution_x",voxel_resx);
     p_nh.getParam("voxel_resolution_y",voxel_resy);
     p_nh.getParam("voxel_resolution_z",voxel_resz);
     p_nh.getParam("gnd_upper_buffer_offset",gnd_buffer_upper_offset);
     p_nh.getParam("gnd_lower_buffer_offset",gnd_buffer_lower_offset);
     p_nh.getParam("cloud_outlier_min_neighbors",sor_mink);
     p_nh.getParam("cloud_outlier_search_radius",sor_thresh);

     this->_max_obstacle_height = max_obs_height;
     this->_min_obstacle_height = min_obs_height;
     this->_max_obstacle_range = max_obs_range;
     this->_min_obstacle_range = min_obs_range;
     this->_voxel_res_x = voxel_resx;
     this->_voxel_res_y = voxel_resy;
     this->_voxel_res_z = voxel_resz;
     this->_gnd_line_upper_offset = gnd_buffer_upper_offset;
     this->_gnd_line_lower_offset = gnd_buffer_lower_offset;
     this->_gnd_line_max_ang = 89.0;
     this->_gnd_line_min_ang = 26.0;
     this->_cloud_outlier_min_neighbors = sor_mink;
     this->_cloud_outlier_search_radius = sor_thresh;
     this->_contourFiltMeth = 1;
     this->_contourFiltMinThresh = 50.0;
     this->_uThreshs = {0.3,0.295,0.275,0.3};
     this->_vThreshs = {0.3, 0.3,0.25,0.4};
     this->_use_custom_umap_filtering = false;
     this->_custom_uThresh_perc = 0.25;
     this->_vmap_sobel_thresh = 30;
     this->_do_vmap_sobel_dilation = false;
     this->_sobel_dilate_kernel_size = 3;
     this->_do_vmap_sobel_blurring = false;
     this->_vmap_sobel_passes = 3;
     this->_sobel_blur_kernel_size = 3;

     /** ROS Object Configuration */
     std::string depth_image_topic = "camera/depth/image_rect_raw";
     std::string camera_info_topic = "camera/depth/camera_info";
     std::string disparity_image_topic = "camera/disparity/image_raw";
     std::string umap_topic = "vboats/umap/image_raw";
     std::string vmap_topic = "vboats/vmap/image_raw";
     std::string filtered_image_topic = "vboats/depth/image_filtered";
     std::string obstacles_image_topic = "vboats/obstacles/image_raw";
     std::string detected_obstacles_info_topic = "vboats/obstacles/data";
     std::string raw_cloud_topic = "vboats/cloud/raw";
     std::string filtered_cloud_topic = "vboats/cloud/filtered";
     std::string unfiltered_cloud_topic = "vboats/cloud/pre_processed";

     std::string imu_topic = "";
     std::string pose_topic = "";
     std::string pose_stamped_topic = "";

     p_nh.getParam("depth_image_topic",depth_image_topic);
     p_nh.getParam("camera_info_topic",camera_info_topic);
     p_nh.getParam("disparity_image_topic",disparity_image_topic);
     p_nh.getParam("umap_topic",umap_topic);
     p_nh.getParam("vmap_topic",vmap_topic);
     p_nh.getParam("filtered_image_topic",filtered_image_topic);
     p_nh.getParam("obstacles_image_topic",obstacles_image_topic);
     p_nh.getParam("obstacles_info_topic",detected_obstacles_info_topic);
     p_nh.getParam("raw_cloud_topic",raw_cloud_topic);
     p_nh.getParam("unfiltered_cloud_topic",unfiltered_cloud_topic);
     p_nh.getParam("filtered_cloud_topic",filtered_cloud_topic);

     p_nh.getParam("imu_topic",imu_topic);
     p_nh.getParam("pose_topic",pose_topic);
     p_nh.getParam("pose_stamped_topic",pose_stamped_topic);

     /** Initialize ROS-Objects */
     this->_cam_info_sub = m_nh.subscribe<sensor_msgs::CameraInfo>(camera_info_topic, 1, boost::bind(&VboatsRos::infoCallback,this,_1,1));
     this->_depth_sub = m_nh.subscribe<sensor_msgs::Image>(depth_image_topic, 30, boost::bind(&VboatsRos::depthCallback,this,_1,5));
     // this->_disparity_sub = m_nh.subscribe<sensor_msgs::Image>(disparity_image_topic, 30, boost::bind(&VboatsRos::disparityCallback,this,_1,5));
     this->_disparity_sub = m_nh.subscribe<stereo_msgs::DisparityImage>(disparity_image_topic, 30, &VboatsRos::zedDisparityCallback,this);
     this->_new_img_pub = m_nh.advertise<sensor_msgs::Image>(filtered_image_topic, 1);
     this->_obstacles_img_pub = m_nh.advertise<sensor_msgs::Image>(obstacles_image_topic, 1);
     this->_detected_obstacle_info_pub = m_nh.advertise<swanson_msgs::VboatsObstacles>(detected_obstacles_info_topic, 100);

     if(strcmp(imu_topic.c_str(), "") != 0){
          this->_imu_sub = m_nh.subscribe<sensor_msgs::Imu>(imu_topic, 30, &VboatsRos::imuCallback,this);
     }
     if(strcmp(pose_topic.c_str(), "") != 0){
          this->_pose_sub = m_nh.subscribe<geometry_msgs::Pose>(pose_topic, 30, &VboatsRos::poseCallback,this);
     }
     if(strcmp(pose_stamped_topic.c_str(), "") != 0){
          this->_pose_stamped_sub = m_nh.subscribe<geometry_msgs::PoseStamped>(pose_stamped_topic, 30, &VboatsRos::poseStampedCallback,this);
     }

     this->_umap_pub = m_nh.advertise<sensor_msgs::Image>(umap_topic, 1);
     this->_vmap_pub = m_nh.advertise<sensor_msgs::Image>(vmap_topic, 1);
     this->_raw_cloud_pub = m_nh.advertise<cloudxyz_t>(raw_cloud_topic, 1);
     this->_unfiltered_cloud_pub = m_nh.advertise<cloudxyz_t>(unfiltered_cloud_topic, 1);
     this->_filtered_cloud_pub = m_nh.advertise<cloudxyz_t>(filtered_cloud_topic, 1);

     /** ROS tf frames Configuration */
     std::string tf_prefix = "/";
     std::string parent_tf = "base_link";
     std::string cam_base_tf = "camera_link";
     p_nh.getParam("parent_frame",parent_tf);
     p_nh.getParam("camera_base_frame",cam_base_tf);

     std::string tmp_parent_tf = parent_tf;
     std::string tmp_cam_base_tf = cam_base_tf;
     if(flag_use_tf_prefix){
          tmp_parent_tf = tf_prefix + tmp_parent_tf;
          tmp_cam_base_tf = tf_prefix + tmp_cam_base_tf;
     }
     this->_parent_tf = tmp_parent_tf;
     this->_camera_tf = tmp_cam_base_tf;

     /** Pre-initialize Common variables */
     this->_umapImgHeader = std_msgs::Header();
     this->_vmapImgHeader = std_msgs::Header();
     this->_filteredImgHeader = std_msgs::Header();
     this->_obsImgHeader = std_msgs::Header();
     this->_depth2disparityFactor = 1.0;

     /** Initialize VBOATS */
     this->vb = new VBOATS();

     printf("[INFO] VboatsRos::VboatsRos() ---- Successfully Initialized!\r\n");
     this->_cfg_f = boost::bind(&VboatsRos::cfgCallback, this, _1, _2);
     this->_cfg_server.setCallback(this->_cfg_f);

     this->_loop_rate = new ros::Rate(update_rate);
}
VboatsRos::~VboatsRos(){
     delete this->_loop_rate;
     delete this->vb;
}

/** SECTION:
     ROS CALLBACKS
*/
void VboatsRos::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
     std::lock_guard<std::mutex> lock(_lock);
     tf::Quaternion quat;
     double roll, pitch, yaw;
     geometry_msgs::Quaternion orientMsg = msg->orientation;
     tf::quaternionMsgToTF(orientMsg, quat);
     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
     printf("[INFO] VboatsRos::imuCallback() --- roll=%f pitch=%f yaw=%f.\r\n", roll*M_RAD2DEG, pitch*M_RAD2DEG, yaw*M_RAD2DEG);
     this->_correction_roll = -roll * M_RAD2DEG;
     this->_correction_pitch = -pitch * M_RAD2DEG;
     this->_correction_yaw = -yaw * M_RAD2DEG;
}
void VboatsRos::poseCallback(const geometry_msgs::Pose::ConstPtr& msg){
     std::lock_guard<std::mutex> lock(_lock);
     tf::Quaternion quat;
     double roll, pitch, yaw;
     geometry_msgs::Quaternion orientMsg = msg->orientation;
     tf::quaternionMsgToTF(orientMsg, quat);
     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
     // printf("[INFO] VboatsRos::poseCallback() --- roll=%f pitch=%f yaw=%f.\r\n", roll*M_RAD2DEG, pitch*M_RAD2DEG, yaw*M_RAD2DEG);
     this->_correction_roll = -roll * M_RAD2DEG;
     this->_correction_pitch = -pitch * M_RAD2DEG;
     this->_correction_yaw = -yaw * M_RAD2DEG;
}
void VboatsRos::poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
     std::lock_guard<std::mutex> lock(_lock);
     tf::Quaternion quat;
     double roll, pitch, yaw;
     geometry_msgs::Quaternion orientMsg = msg->pose.orientation;
     tf::quaternionMsgToTF(orientMsg, quat);
     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
     // printf("[INFO] VboatsRos::poseStampedCallback() --- roll=%f pitch=%f yaw=%f.\r\n", roll*M_RAD2DEG, pitch*M_RAD2DEG, yaw*M_RAD2DEG);
     this->_correction_roll = -roll * M_RAD2DEG;
     this->_correction_pitch = -pitch * M_RAD2DEG;
     this->_correction_yaw = -yaw * M_RAD2DEG;
}
void VboatsRos::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg, const int value){
     if(this->_info_count < value){
          this->_fx = msg->K[0];
          this->_fy = msg->K[4];
          this->_px = msg->K[2];
          this->_py = msg->K[5];
          this->_dscale = msg->D[0];
          if(this->_dscale == 0) this->_dscale = 1.0;
          float Tx = msg->P[3];
          this->_baseline = -Tx / this->_fx;
          if(this->_baseline == 0) this->_baseline = 0.12;
          printf("[INFO] VboatsRos::infoCallback() ---- Dscale = %.4f, Baseline = %.6f,  Focals = [%.2f, %.2f],  Principles = [%.2f, %.2f]\r\n", this->_dscale, this->_baseline, this->_fx, this->_fy, this->_px, this->_py);
          this->_focal[0] = this->_fx;
          this->_focal[1] = this->_fy;
          this->_principle[0] = this->_px;
          this->_principle[1] = this->_py;
          this->_depth2disparityFactor = (this->_fx * this->_baseline) / this->_dscale;
          this->_recvd_cam_info = true;
     }
     this->_info_count++;
}
void VboatsRos::depthCallback(const sensor_msgs::Image::ConstPtr& msg, const int value){
     std::lock_guard<std::mutex> lock(_lock);
     cv::Mat image = cv_bridge::toCvCopy(msg)->image;
     // this->_depth = depth.clone();
     this->_depth = image;
     this->_img_count++;
     if(this->_img_count >= value){ this->_recvd_image = true; }
}
void VboatsRos::disparityCallback(const sensor_msgs::Image::ConstPtr& msg, const int value){
     std::lock_guard<std::mutex> lock(_lock);

     cv::Mat disparity;
     cv::Mat image = cv_bridge::toCvCopy(msg)->image;
     if(image.type() != CV_8UC1){
          double minVal, maxVal;
          cv::minMaxLoc(image, &minVal, &maxVal);
          if(maxVal == 0) maxVal = 1.0;
          image.convertTo(disparity, CV_8UC1, (255.0/maxVal) );
     } else disparity = image;

     // this->_disparity = disparity.clone();
     this->_disparity = disparity;
     this->_img_count++;
     if(this->_img_count >= value){ this->_recvd_image = true; }
}
void VboatsRos::zedDisparityCallback(const stereo_msgs::DisparityImage::ConstPtr& msg){
     std::lock_guard<std::mutex> lock(_lock);
     sensor_msgs::Image msgImg = msg->image;
     cv::Mat disparity;
     cv::Mat image = cv_bridge::toCvCopy(msgImg)->image;
     if(image.type() != CV_8UC1){
          double minVal, maxVal;
          cv::minMaxLoc(image, &minVal, &maxVal);
          if(maxVal == 0) maxVal = 1.0;
          image.convertTo(disparity, CV_8UC1, (255.0/maxVal) );
     } else disparity = image;

     // this->_disparity = disparity.clone();
     this->_disparity = disparity;
     // this->_img_count++;
}

void VboatsRos::cfgCallback(swanson_algorithms::VboatsConfig &config, uint32_t level){
     std::lock_guard<std::mutex> lock(_lock);
     /** Update Variables only if they are different from current settings */
     // Core
     if(config.cam_min_depth != this->_cam_min_depth) this->_cam_min_depth = config.cam_min_depth;
     if(config.cam_max_depth != this->_cam_max_depth) this->_cam_max_depth = config.cam_max_depth;
     if(config.cam_min_depth_x != this->_cam_min_depth_x) this->_cam_min_depth_x = config.cam_min_depth_x;
     if(config.cam_max_depth_x != this->_cam_max_depth_x) this->_cam_max_depth_x = config.cam_max_depth_x;
     if(config.cam_min_depth_y != this->_cam_min_depth_y) this->_cam_min_depth_y = config.cam_min_depth_y;
     if(config.cam_max_depth_y != this->_cam_max_depth_y) this->_cam_max_depth_y = config.cam_max_depth_y;
     if(config.use_depth_for_process != this->_do_depth_based_processing){
          this->_do_depth_based_processing = config.use_depth_for_process;
     }
     if(config.do_angle_correction != this->_do_angle_correction){
          this->_do_angle_correction = config.do_angle_correction;
     }
     if(config.detect_obstacles != this->_do_individual_obstacle_detection){
          this->_do_individual_obstacle_detection = config.detect_obstacles;
     }
     if(config.use_depth_obj_detection != this->_use_filtered_depth_object_detection){
          this->_use_filtered_depth_object_detection = config.use_depth_obj_detection;
     }
     if(config.do_gnd_line_based_filtering != this->_use_gnd_line_based_removal){
          this->_use_gnd_line_based_removal = config.do_gnd_line_based_filtering;
     }
     if(config.do_object_segmented_filtering != this->_do_object_segmented_filtering){
          this->_do_object_segmented_filtering = config.do_object_segmented_filtering;
     }
     if(config.flip_gnd_line_mask != this->_flip_gnd_line_mask) this->_flip_gnd_line_mask = config.flip_gnd_line_mask;
     if(config.do_post_depth_morphing != this->_do_post_depth_morphing) this->_do_post_depth_morphing = config.do_post_depth_morphing;
     if(config.depth_morph_kernel_size != this->_depth_morph_kernel_size) this->_depth_morph_kernel_size = config.depth_morph_kernel_size;

     // Umap Filtering
     if(config.use_custom_umap_filtering != this->_use_custom_umap_filtering) this->_use_custom_umap_filtering = config.use_custom_umap_filtering;
     if(config.contourFiltMeth != this->_contourFiltMeth) this->_contourFiltMeth = config.contourFiltMeth;
     if(config.contourFiltMinThresh != this->_contourFiltMinThresh) this->_contourFiltMinThresh = config.contourFiltMinThresh;
     if(config.umap_custom_thresh_perc != this->_custom_uThresh_perc) this->_custom_uThresh_perc = config.umap_custom_thresh_perc;

     if(config.pre_blur_thresh != this->_pre_blur_thresh) this->_pre_blur_thresh = config.pre_blur_thresh;
     if(config.primary_blur != this->_primary_blur) this->_primary_blur = config.primary_blur;
     if(config.umask_primary_blur_size != this->_umask_primary_blur_size){
          if(config.umask_primary_blur_size == 0) this->_umask_primary_blur_size = 1;
          else this->_umask_primary_blur_size = config.umask_primary_blur_size;
     }
     if(config.umap_sobel_thresh != this->_umap_sobel_thresh) this->_umap_sobel_thresh = config.umap_sobel_thresh;
     if(config.secondary_dilate != this->_secondary_dilate) this->_secondary_dilate = config.secondary_dilate;
     if(config.umask_secondary_dilate_sz != this->_umask_secondary_dilate_size){
          if(config.umask_secondary_dilate_sz == 0) this->_umask_secondary_dilate_size = 1;
          else this->_umask_secondary_dilate_size = config.umask_secondary_dilate_sz;
     }
     if(config.secondary_blur != this->_secondary_blur) this->_secondary_blur = config.secondary_blur;
     if(config.umask_secondary_blur_size != this->_umask_secondary_blur_size){
          if(config.umask_secondary_blur_size == 0) this->_umask_secondary_blur_size = 1;
          else this->_umask_secondary_blur_size = config.umask_secondary_blur_size;
     }
     if(config.post_blur_thresh != this->_post_blur_thresh) this->_post_blur_thresh = config.post_blur_thresh;
     if(config.kernel_x_multiplier != this->_kernel_x_multiplier) this->_kernel_x_multiplier = config.kernel_x_multiplier;
     if(config.kernel_y_multiplier != this->_kernel_y_multiplier) this->_kernel_y_multiplier = config.kernel_y_multiplier;

     // Vmap Filtering
     if(config.use_custom_vmap_filtering != this->_use_custom_vmap_filtering) this->_use_custom_vmap_filtering = config.use_custom_vmap_filtering;
     if(config.do_sobel_pre_thresholding != this->_do_sobel_pre_thresholding) this->_do_sobel_pre_thresholding = config.do_sobel_pre_thresholding;
     if(config.sobel_pre_thresh != this->_sobel_pre_thresh) this->_sobel_pre_thresh = config.sobel_pre_thresh;
     if(config.vmask_blur_sobel != this->_do_vmap_sobel_blurring) this->_do_vmap_sobel_blurring = config.vmask_blur_sobel;
     if(config.vmask_blur_size != this->_sobel_blur_kernel_size){
          if(config.vmask_blur_size == 0) this->_sobel_blur_kernel_size = 1;
          else this->_sobel_blur_kernel_size = config.vmask_blur_size;
     }
     if(config.dilate_sobel != this->_do_vmap_sobel_dilation) this->_do_vmap_sobel_dilation = config.dilate_sobel;
     if(config.vmask_sobel_dilate_sz != this->_sobel_dilate_kernel_size){
          if(config.vmask_sobel_dilate_sz == 0) this->_sobel_dilate_kernel_size = 1;
          else this->_sobel_dilate_kernel_size = config.vmask_sobel_dilate_sz;
     }
     if(config.use_scharr != this->_use_scharr) this->_use_scharr = config.use_scharr;
     if(config.sobel_kernel_size != this->_vmap_sobel_passes) this->_vmap_sobel_passes = config.sobel_kernel_size;
     if(config.vmap_sobel_thresh != this->_vmap_sobel_thresh) this->_vmap_sobel_thresh = config.vmap_sobel_thresh;
     // Ground-Line Extraction
     if(config.gnd_upper_offset != this->_gnd_line_upper_offset) this->_gnd_line_upper_offset = config.gnd_upper_offset;
     if(config.gnd_lower_offset != this->_gnd_line_lower_offset) this->_gnd_line_lower_offset = config.gnd_lower_offset;
     if(config.max_gnd_deg != this->_gnd_line_max_ang) this->_gnd_line_max_ang = config.max_gnd_deg;
     if(config.min_gnd_deg != this->_gnd_line_min_ang) this->_gnd_line_min_ang = config.min_gnd_deg;
     // Vmap Secondary Filtering
     if(config.vmask_subtract_sobel != this->_do_sobel_vmask_subtraction) this->_do_sobel_vmask_subtraction = config.vmask_subtract_sobel;
     if(config.do_secondary_sobel_thresholding != this->_do_vmap_sobel_sec_thresh) this->_do_vmap_sobel_sec_thresh = config.do_secondary_sobel_thresholding;
     if(config.vmap_sobel_sec_thresh != this->_vmap_sobel_sec_thresh) this->_vmap_sobel_sec_thresh = config.vmap_sobel_sec_thresh;
     if(config.vmask_sec_blur_sobel != this->_do_vmap_sobel_sec_blurring) this->_do_vmap_sobel_sec_blurring = config.vmask_sec_blur_sobel;
     if(config.vmask_secondary_blur_size != this->_sobel_sec_blur_kernel_size){
          if(config.vmask_secondary_blur_size == 0) this->_sobel_sec_blur_kernel_size = 1;
          else this->_sobel_sec_blur_kernel_size = config.vmask_secondary_blur_size;
     }
     if(config.dilate_secondary_sobel != this->_do_vmap_sobel_sec_dilation) this->_do_vmap_sobel_sec_dilation = config.dilate_secondary_sobel;
     if(config.vmask_sobel_dilate_sec_sz != this->_sobel_sec_dilate_kernel_size){
          if(config.vmask_sobel_dilate_sec_sz == 0) this->_sobel_sec_dilate_kernel_size = 1;
          else this->_sobel_sec_dilate_kernel_size = config.vmask_sobel_dilate_sec_sz;
     }
     if(config.vmap_thresh != this->_vmap_thresh) this->_vmap_thresh = config.vmap_thresh;

     // Cloud Filtering
     if(config.cloud_filter_limits != this->_do_cloud_limit_filtering) this->_do_cloud_limit_filtering = config.cloud_filter_limits;
     if(config.max_obstacle_height != this->_max_obstacle_height) this->_max_obstacle_height = config.max_obstacle_height;
     if(config.min_obstacle_height != this->_min_obstacle_height) this->_min_obstacle_height = config.min_obstacle_height;
     if(config.max_obstacle_range != this->_max_obstacle_range) this->_max_obstacle_range = config.max_obstacle_range;
     if(config.min_obstacle_range != this->_min_obstacle_range) this->_min_obstacle_range = config.min_obstacle_range;
     if(config.cloud_filter_voxelize != this->_do_cloud_downsampling) this->_do_cloud_downsampling = config.cloud_filter_voxelize;
     if(config.voxel_res_x != this->_voxel_res_x) this->_voxel_res_x = config.voxel_res_x;
     if(config.voxel_res_y != this->_voxel_res_y) this->_voxel_res_y = config.voxel_res_y;
     if(config.voxel_res_z != this->_voxel_res_z) this->_voxel_res_z = config.voxel_res_z;
     if(config.do_cloud_outlier_removal != this->_do_cloud_outlier_removal) this->_do_cloud_outlier_removal = config.do_cloud_outlier_removal;
     if(config.cloud_outlier_min_neighbors != this->_cloud_outlier_min_neighbors) this->_cloud_outlier_min_neighbors = config.cloud_outlier_min_neighbors;
     if(config.cloud_outlier_search_radius != this->_cloud_outlier_search_radius) this->_cloud_outlier_search_radius = config.cloud_outlier_search_radius;

     // Data Publishing
     if(config.publish_obstacle_data != this->_publish_obs_data){ this->_publish_obs_data = config.publish_obstacle_data; }
     if(config.publish_obstacle_image != this->_publish_obstacle_segmented_image){ this->_publish_obstacle_segmented_image = config.publish_obstacle_image; }
     if(config.publish_aux_imgs != this->_publish_aux_images){ this->_publish_aux_images = config.publish_aux_imgs; }
     if(config.publish_raw_cloud != this->_flag_pub_raw_cloud){ this->_flag_pub_raw_cloud = config.publish_raw_cloud; }
     if(config.publish_filter_cloud != this->_flag_pub_filtered_cloud){ this->_flag_pub_filtered_cloud = config.publish_filter_cloud; }
     if(config.publish_unfilter_cloud != this->_flag_pub_unfiltered_cloud){ this->_flag_pub_unfiltered_cloud = config.publish_unfilter_cloud; }

     // Debugging Printouts
     if(config.debug != this->_debug){ this->_debug = config.debug; }
     if(config.verbose != this->_verbose){ this->_verbose = config.verbose; }
     if(config.verbose_update != this->_verbose_update) this->_verbose_update = config.verbose_update;
     if(config.verbose_gnd_line_removal != this->_verbose_gnd_line_removal){ this->_verbose_gnd_line_removal = config.verbose_gnd_line_removal; }
     if(config.verbose_obstacles != this->_verbose_obstacles) this->_verbose_obstacles = config.verbose_obstacles;
     if(config.debug_timings != this->_debug_timings){ this->_debug_timings = config.debug_timings; }
     if(config.debug_gnd_line_removal != this->_debug_gnd_line_removal){ this->_debug_gnd_line_removal = config.debug_gnd_line_removal; }
     if(config.debug_disparity_generation != this->_debug_disparity_generation){ this->_debug_disparity_generation = config.debug_disparity_generation; }
     if(config.debug_published_cloud_sizes != this->_debug_published_cloud_sizes){ this->_debug_published_cloud_sizes = config.debug_published_cloud_sizes; }
     if(config.debug_image_info != this->_debug_image_info){ this->_debug_image_info = config.debug_image_info; }

     // Input Visualization
     if(config.visualize_inputs != this->_visualize_inputs) this->_visualize_inputs = config.visualize_inputs;
     if(config.visualize_angle_corrected_depth != this->_visualize_angle_corrected_depth) this->_visualize_angle_corrected_depth = config.visualize_angle_corrected_depth;
     if(config.visualize_generated_disparity != this->_visualize_generated_disparity) this->_visualize_generated_disparity = config.visualize_generated_disparity;
     // Umap Visualization
     if(config.visualize_umap_raw != this->_visualize_umap_raw) this->_visualize_umap_raw = config.visualize_umap_raw;
     if(config.visualize_umap_filtered != this->_visualize_umap_filtered) this->_visualize_umap_filtered = config.visualize_umap_filtered;
     if(config.visualize_umap_contours != this->_visualize_umap_contours) this->_visualize_umap_contours = config.visualize_umap_contours;
     // Vmap Visualization
     if(config.visualize_vmap_raw != this->_visualize_vmap_raw) this->_visualize_vmap_raw = config.visualize_vmap_raw;
     if(config.visualize_vmap_raw_w_lines != this->_visualize_vmap_raw_w_lines) this->_visualize_vmap_raw_w_lines = config.visualize_vmap_raw_w_lines;
     if(config.visualize_vmap_blurred != this->_visualize_vmap_blurred) this->_visualize_vmap_blurred = config.visualize_vmap_blurred;
     if(config.visualize_vmap_dilated != this->_visualize_vmap_dilated) this->_visualize_vmap_dilated = config.visualize_vmap_dilated;
     if(config.visualize_vmap_sobel_raw != this->_visualize_vmap_sobel_raw) this->_visualize_vmap_sobel_raw = config.visualize_vmap_sobel_raw;
     if(config.visualize_vmap_thresh != this->_visualize_vmap_thresh) this->_visualize_vmap_thresh = config.visualize_vmap_thresh;
     if(config.visualize_sobel_sec_thresh != this->_visualize_vmap_sec_thresh) this->_visualize_vmap_sec_thresh = config.visualize_sobel_sec_thresh;
     if(config.visualize_sobel_sec_blur != this->_visualize_vmap_sec_blur) this->_visualize_vmap_sec_blur = config.visualize_sobel_sec_blur;
     if(config.visualize_sobel_sec_dilate != this->_visualize_vmap_sec_dilated) this->_visualize_vmap_sec_dilated = config.visualize_sobel_sec_dilate;
     if(config.visualize_vmap_mask != this->_visualize_vmap_mask) this->_visualize_vmap_mask = config.visualize_vmap_mask;
     if(config.visualize_process_input_vmap != this->_visualize_process_input_vmap) this->_visualize_process_input_vmap = config.visualize_process_input_vmap;
     if(config.visualize_obj_detection_vmap != this->_visualize_obj_detection_vmap) this->_visualize_obj_detection_vmap = config.visualize_obj_detection_vmap;
     // Segmenetation Visualization
     if(config.visualize_gnd_line_mask != this->_visualize_gnd_mask) this->_visualize_gnd_mask = config.visualize_gnd_line_mask;
     if(config.visualize_object_mask != this->_visualize_obj_mask) this->_visualize_obj_mask = config.visualize_object_mask;
     if(config.visualize_gnd_filter_image != this->_visualize_gnd_filter_img) this->_visualize_gnd_filter_img = config.visualize_gnd_filter_image;
     if(config.visualize_obj_filter_image != this->_visualize_obj_filter_img) this->_visualize_obj_filter_img = config.visualize_obj_filter_image;
     if(config.visualize_obstacle_search_windows != this->_visualize_obstacle_search_windows) this->_visualize_obstacle_search_windows = config.visualize_obstacle_search_windows;
     if(config.visualize_segmented_obstacle_regions != this->_visualize_segmented_obstacle_regions) this->_visualize_segmented_obstacle_regions = config.visualize_segmented_obstacle_regions;
     // Outputs Visualization
     if(config.visualize_filtered_depth != this->_visualize_filtered_depth) this->_visualize_filtered_depth = config.visualize_filtered_depth;

     if(this->_verbose){
          ROS_INFO("Reconfigure Request:");
          ROS_INFO(" -- Using Ground Line-based object segmentation = %s", this->_use_gnd_line_based_removal?"True":"False");
          ROS_INFO(" -- Contour Filter Method = %d", this->_contourFiltMeth);
          ROS_INFO(" -- Contour Min Thresh = %.3f", this->_contourFiltMinThresh);
          ROS_INFO(" -- Cloud Height Limits = %.4f, %.4f", this->_max_obstacle_height, this->_min_obstacle_height);
          ROS_INFO(" -- Cloud Range Limits = %.4f, %.4f", this->_min_obstacle_height, this->_max_obstacle_range);
          ROS_INFO(" -- Cloud Voxel Resolution (X,Y,Z) = %.4f, %.4f, %.4f", this->_voxel_res_x, this->_voxel_res_y, this->_voxel_res_z);
          ROS_INFO(" -- Ground Line Offsets (upper, lower) = %d, %d", this->_gnd_line_upper_offset, this->_gnd_line_lower_offset);
          ROS_INFO(" -- Cloud Filtering-> Keeping points having at least %d neighbors w/in %.4f meters", this->_cloud_outlier_min_neighbors, this->_cloud_outlier_search_radius);
     }

     this->_do_umap_viz = (this->_visualize_umap_contours || this->_visualize_umap_raw || this->_visualize_umap_filtered);
     this->_do_misc_viz = (this->_visualize_generated_disparity
          || this->_visualize_segmented_obstacle_regions
          || this->_visualize_gnd_mask
          || this->_visualize_obj_mask
          || this->_visualize_gnd_filter_img
          || this->_visualize_obj_filter_img
          || this->_visualize_inputs
          || this->_visualize_angle_corrected_depth
     );
     this->_do_vmap_viz = (this->_visualize_vmap_raw || this->_visualize_vmap_raw_w_lines ||
          this->_visualize_vmap_sobel_raw  || this->_visualize_process_input_vmap ||
          this->_visualize_vmap_blurred || this->_visualize_vmap_dilated ||
          this->_visualize_vmap_thresh || this->_visualize_vmap_sec_thresh ||
          this->_visualize_vmap_sec_blur || this->_visualize_vmap_sec_dilated ||
          this->_visualize_vmap_mask || this->_visualize_obj_detection_vmap ||
          this->_visualize_obstacle_search_windows
     );
     this->_do_cv_wait_key = (this->_do_vmap_viz || this->_do_umap_viz || this->_do_misc_viz);
}

/** SECTION:
     ROS Publisher Helper Functions
*/
void VboatsRos::publish_images(cv::Mat* umap, cv::Mat* vmap, cv::Mat* filtered){
     ros::Time time = ros::Time::now();
     /** Umap Image */
     cv::Mat tmpOut;
     if(umap != nullptr){
          tmpOut = (*umap).clone();
          if(!tmpOut.empty()){
               this->_umapImgHeader.stamp = time;
               this->_umapImgHeader.seq = this->_count;
               this->_umapImgHeader.frame_id = this->_camera_tf;

               // sensor_msgs::ImagePtr umapImgMsg = cv_bridge::CvImage(this->_umapImgHeader, "8UC1", umap).toImageMsg();
               sensor_msgs::ImagePtr umapImgMsg;
               if(tmpOut.type() == CV_8UC1) umapImgMsg = cv_bridge::CvImage(this->_umapImgHeader, "8UC1", tmpOut).toImageMsg();
               else if(tmpOut.type() == CV_16UC1) umapImgMsg = cv_bridge::CvImage(this->_umapImgHeader, "16UC1", tmpOut).toImageMsg();
               else if(tmpOut.type() == CV_32F) umapImgMsg = cv_bridge::CvImage(this->_umapImgHeader, "32FC1", tmpOut).toImageMsg();
               else if(tmpOut.type() == CV_8UC3) umapImgMsg = cv_bridge::CvImage(this->_umapImgHeader, "bgr8", tmpOut).toImageMsg();
               this->_umap_pub.publish(umapImgMsg);
          }
     }
     /** Vmap Image */
     if(vmap != nullptr){
          tmpOut = (*vmap).clone();
          if(!tmpOut.empty()){
               this->_vmapImgHeader.stamp = time;
               this->_vmapImgHeader.seq = this->_count;
               this->_vmapImgHeader.frame_id = this->_camera_tf;

               // sensor_msgs::ImagePtr vmapImgMsg = cv_bridge::CvImage(this->_vmapImgHeader, "8UC1", vmap).toImageMsg();
               sensor_msgs::ImagePtr vmapImgMsg;
               if(tmpOut.type() == CV_8UC1) vmapImgMsg = cv_bridge::CvImage(this->_vmapImgHeader, "8UC1", tmpOut).toImageMsg();
               else if(tmpOut.type() == CV_16UC1) vmapImgMsg = cv_bridge::CvImage(this->_vmapImgHeader, "16UC1", tmpOut).toImageMsg();
               else if(tmpOut.type() == CV_32F) vmapImgMsg = cv_bridge::CvImage(this->_vmapImgHeader, "32FC1", tmpOut).toImageMsg();
               else if(tmpOut.type() == CV_8UC3) vmapImgMsg = cv_bridge::CvImage(this->_vmapImgHeader, "bgr8", tmpOut).toImageMsg();
               this->_vmap_pub.publish(vmapImgMsg);
          }
     }

     /** Filtered Disparity */
     if(filtered != nullptr){
          tmpOut = (*filtered).clone();
          if(!tmpOut.empty()){
               this->_filteredImgHeader.stamp = time;
               this->_filteredImgHeader.seq = this->_count;
               this->_filteredImgHeader.frame_id = this->_camera_tf;

               sensor_msgs::ImagePtr filteredImgMsg;
               if(tmpOut.type() == CV_8UC1) filteredImgMsg = cv_bridge::CvImage(this->_filteredImgHeader, "8UC1", tmpOut).toImageMsg();
               else if(tmpOut.type() == CV_16UC1) filteredImgMsg = cv_bridge::CvImage(this->_filteredImgHeader, "16UC1", tmpOut).toImageMsg();
               else if(tmpOut.type() == CV_32F) filteredImgMsg = cv_bridge::CvImage(this->_filteredImgHeader, "32FC1", tmpOut).toImageMsg();
               else if(tmpOut.type() == CV_8UC3) filteredImgMsg = cv_bridge::CvImage(this->_filteredImgHeader, "bgr8", tmpOut).toImageMsg();
               this->_new_img_pub.publish(filteredImgMsg);
          }
     }
}
void VboatsRos::publish_obstacle_image(cv::Mat image){
     ros::Time time = ros::Time::now();
     /** Obstacles Image */
     if(!image.empty()){
          this->_obsImgHeader.stamp = time;
          this->_obsImgHeader.seq = this->_count;
          this->_obsImgHeader.frame_id = this->_camera_tf;

          sensor_msgs::ImagePtr obsImgMsg;
          if(image.type() == CV_8UC1) obsImgMsg = cv_bridge::CvImage(this->_obsImgHeader, "8UC1", image).toImageMsg();
          else if(image.type() == CV_8UC3) obsImgMsg = cv_bridge::CvImage(this->_obsImgHeader, "bgr8", image).toImageMsg();
          else if(image.type() == CV_16UC1) obsImgMsg = cv_bridge::CvImage(this->_obsImgHeader, "16UC1", image).toImageMsg();
          else if(image.type() == CV_16UC3) obsImgMsg = cv_bridge::CvImage(this->_obsImgHeader, "16UC3", image).toImageMsg();
          else if(image.type() == CV_32FC1) obsImgMsg = cv_bridge::CvImage(this->_obsImgHeader, "32FC1", image).toImageMsg();
          else{
               cv::Mat output;
               cv::Mat cvtd = image.clone();
               cv::cvtColor(cvtd, cvtd, cv::COLOR_BGR2GRAY);
               double minVal, maxVal;
               cv::minMaxLoc(cvtd, &minVal, &maxVal);
               if(cvtd.channels() > 1) cvtd.convertTo(output, CV_8UC3, (255.0/maxVal) );
               else cvtd.convertTo(output, CV_8UC1, (255.0/maxVal) );
               obsImgMsg = cv_bridge::CvImage(this->_obsImgHeader, "8UC1", output).toImageMsg();
          }
          this->_obstacles_img_pub.publish(obsImgMsg);
     } else{
          printf("[WARNING] %s::publish_obstacle_image() --- Image is empty, not publishing.\r\n", this->classLbl.c_str());
          cvinfo(image, "publish_obstacle_image() --- Input Image: ");
     }
}

/** SECTION:
     Misc Functions
*/
void VboatsRos::depth_to_disparity(const cv::Mat& depth, cv::Mat* disparity, float gain){
     if(depth.empty()) return;
     cv::Mat _disparity, _disparity8;
     if(this->_debug_disparity_generation){
          printf("[DEBUG] %s::depth_to_disparity() --- Using conversion gain = %f.\r\n", this->classLbl.c_str(), gain);
          cvinfo(depth, "VboatsRos::depth_to_disparity() --- Input Depth Image: ");
     }
     if(depth.type() != CV_32F) depth.convertTo(_disparity, CV_32F);
     else _disparity = depth.clone();
     if(this->_debug_disparity_generation) cvinfo(_disparity, "VboatsRos::depth_to_disparity() --- Input image before Disparity conversion: ");

     // ForEachDepthConverter<float> converter(gain);
     // ForEachDepthConverter<float> converter(gain, this->_min_obstacle_range, this->_max_obstacle_range);
     ForEachDepthConverter<float> converter(gain, this->_cam_min_depth, this->_cam_max_depth);
     _disparity.forEach<float>(converter);
     if(_disparity.empty()) return;

     if(this->_debug_disparity_generation) cvinfo(_disparity, "VboatsRos::depth_to_disparity() --- Converted Disparity Image: ");
     if(_disparity.type() != CV_8UC1){
          double minVal, maxVal;
          cv::minMaxLoc(_disparity, &minVal, &maxVal);
          _disparity.convertTo(_disparity8, CV_8UC1, (255.0/maxVal) );
     } else _disparity8 = _disparity;
     if(this->_debug_disparity_generation) cvinfo(_disparity8, "VboatsRos::depth_to_disparity() --- Stored Disparity Image: ");
     if(disparity) *disparity = _disparity8.clone();
}
cloudxyz_t::Ptr VboatsRos::generate_cloud_from_depth(const cv::Mat& depth){
     cloudxyz_t::Ptr tmpCloud(new cloudxyz_t);
     // Return early with empty cloud object if input depth image is empty
     // or the necessary camera information was never recevied
     if( (depth.empty()) || (!this->_recvd_cam_info) ) return tmpCloud;
     // Convert input image to proper data type needed for cloud generation
     cv::Mat tmpDepth = depth.clone();
     if(depth.type() != CV_32F) tmpDepth.convertTo(tmpDepth, CV_32F);

     // Loop through depth image and generate cloud XYZ points using depth information
     float* ptrP;
     pcl::PointXYZ pt;
     double t = (double)cv::getTickCount();
     for(int y = 0; y < tmpDepth.rows; y+=4){
          ptrP = tmpDepth.ptr<float>(y);
          for(int x = 0; x < tmpDepth.cols; x+=4){
               float depthVal = (float) ptrP[x] * this->_dscale;
               if(depthVal > 0){
                    pt.x = ((float)x - this->_px) * depthVal / this->_fx;
                    pt.y = ((float)y - this->_py) * depthVal / this->_fy;
                    pt.z = depthVal;
                    tmpCloud->points.push_back(pt);
               }
          }
     }
     tmpCloud->header.seq = this->_count;
     tmpCloud->header.frame_id = this->_camera_tf;
     tmpCloud->height = 1;
     tmpCloud->width = tmpCloud->points.size();
     double dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
     // printf("[INFO] %s::generate_pointcloud(): Cloud generation Time: Pointer = %.4lf ms (%.2lf Hz)\r\n", this->classLbl.c_str(), dt*1000.0, (1.0/dt));
     return tmpCloud;
}
cloudxyz_t::Ptr VboatsRos::filter_pointcloud(cloudxyz_t::Ptr inputCloud){
     cloudxyz_t::Ptr filtered_cloud(new cloudxyz_t);
     if(inputCloud->points.size() == 0) return filtered_cloud;

     double t = (double)cv::getTickCount();
     if(this->_do_cloud_limit_filtering){
          pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ> ());
          range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -this->_max_obstacle_height)));
          range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, this->_min_obstacle_height)));
          range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, this->_min_obstacle_range)));
          range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, this->_max_obstacle_range)));

          pcl::ConditionalRemoval<pcl::PointXYZ> range_filt;
          range_filt.setInputCloud(inputCloud);
          range_filt.setCondition(range_cond);
          range_filt.filter(*filtered_cloud);
     }

     if(this->_do_cloud_outlier_removal){
          pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
          if(filtered_cloud->size() > 0) outrem.setInputCloud(filtered_cloud);
          else outrem.setInputCloud(inputCloud);
          outrem.setRadiusSearch(this->_cloud_outlier_search_radius);
          outrem.setMinNeighborsInRadius(this->_cloud_outlier_min_neighbors);
          outrem.filter(*filtered_cloud);
     }

     if(this->_do_cloud_downsampling){
          pcl::VoxelGrid<pcl::PointXYZ> voxg;
          if(filtered_cloud->size() > 0) voxg.setInputCloud(filtered_cloud);
          else voxg.setInputCloud(inputCloud);
          voxg.setLeafSize(this->_voxel_res_x, this->_voxel_res_y, this->_voxel_res_z);
          voxg.filter(*filtered_cloud);
     }

     // // Another potentially useful filter for filtering a noisy cloud
     // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
     // sor.setInputCloud(filtered_cloud);
     // sor.setMeanK(this->_cloud_outlier_min_neighbors);
     // sor.setStddevMulThresh(this->_cloud_outlier_search_radius);
     // sor.filter(*filtered_cloud);

     double dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
     // printf("[INFO] %s::generate_pointcloud(): Cloud generation Time: Pointer = %.4lf ms (%.2lf Hz)\r\n", this->classLbl.c_str(), dt*1000.0, (1.0/dt));
     return filtered_cloud;
}


/** SECTION:
     Processing Functions
*/
int VboatsRos::remove_ground(const cv::Mat& disparity, const cv::Mat& vmap,
     const cv::Mat& depth, float* line_params, cv::Mat* filtered_img, cv::Mat* generated_mask)
{
     cv::Mat depthImg, refImg, img;
     if(depth.empty()){
          printf("[WARNING] %s::remove_ground() --- Depth Image is empty, skipping ground removal.\r\n", this->classLbl.c_str());
          return -1;
     }
     if(vmap.empty()){
          printf("[WARNING] %s::remove_ground() --- Vmap input Image is empty, skipping ground removal.\r\n", this->classLbl.c_str());
          return -2;
     }
     if(disparity.empty()){
          printf("[WARNING] %s::remove_ground() --- Disparity input Image is empty, skipping ground removal.\r\n", this->classLbl.c_str());
          return -3;
     }
     refImg = vmap.clone();
     img = disparity.clone();
     depthImg = depth.clone();
     cv::Mat mask = cv::Mat::zeros(disparity.size(), CV_8UC1);

     /** Calculate ROI limits based on estimated ground line coefficients */
     float slope = line_params[0];
     int y0 = (int) line_params[1];
     int y1 = y0 + this->_gnd_line_lower_offset;
     int y1f = (int)(vmap.cols * slope + (y1));
     int y2 = y0 - this->_gnd_line_upper_offset;
     int y2f = (int)(vmap.cols * slope + (y2));
     if(y0 < 0) y0 = 0;
     if(this->_verbose_gnd_line_removal){
          printf("[INFO] VboatsRos::remove_ground() --- slope = %.2f, intercept = %d\r\n", slope, y0);
          cvinfo(refImg, "VboatsRos::remove_ground() --- Vmap Reference: ");
          cvinfo(img, "VboatsRos::remove_ground() --- Disparity Input: ");
          cvinfo(depthImg, "VboatsRos::remove_ground() --- Depth Input: ");
     }

     uchar* pix;
     cv::Mat refRow;
     cv::Mat refMask;
     cv::Mat nonzero;
     double t, dt;
     int minx, maxx, tmpx;
     t = (double)cv::getTickCount();
     /** For each row of the disparity image,
      keep only pixels whose value (i.e. disparity) is within the range of disparity
      values found from the nonzero pixels in the same row of the corresponding
      pre-processed vmap image, while also removing disparity values that fall under
      the estimated ground line (i.e. below the floor).
     */
     for(int v = y0; v < img.rows; ++v){
          // Get the non-zero pixels in the current row of the reference vmap image
          refRow = refImg.row(v);
          refMask = refRow > 0;
          cv::findNonZero(refMask, nonzero);
          maxx = 0; minx = 1000;
          // Use the estimated ground line coefficients to get the disparity limit for this row
          int xlim = (int)((float)(v - y2) / slope);
          if(this->_debug_gnd_line_removal) printf("[INFO] VboatsRos::remove_ground() --- %d Nonzero pixels found for row %d (w/ xlimit = %d): \r\n\r\n", (int) nonzero.total(), v, xlim);
          // Loop through all of the nonzero pixels and determine the min/max disparities
          for(int i = 0; i < nonzero.total(); i++ ){
               tmpx = nonzero.at<cv::Point>(i).x;
               if(tmpx < xlim) continue;
               if(this->_debug_gnd_line_removal) printf("%d, ", tmpx);
               if(tmpx > maxx) maxx = tmpx;
               if(tmpx < minx) minx = tmpx;
          }
          if(this->_debug_gnd_line_removal) printf("\r\n[INFO] VboatsRos::remove_ground() --- Limits found from pixels in row %d = %d, %d\r\n",
          v, (int) nonzero.total(), minx, maxx);

          // Get all the pixels in the current row of the disparity image
          // and use the disparity limits found above to construct a mask used
          // for filtering the pixels in the depth image
          pix = img.ptr<uchar>(v);
          for(int u = 0; u < img.cols; ++u){
               int dvalue = pix[u];
               if( (dvalue >= minx) && (dvalue <= maxx) ) mask.at<uchar>(v, u) = 255;
          }
     }

     cv::Mat maskInv, gndFilteredImg;
     if(!this->_flip_gnd_line_mask){
          depthImg.copyTo(gndFilteredImg, mask);
          if(generated_mask) *generated_mask = mask.clone();
     } else{
          cv::Mat maskInv;
          cv::bitwise_not(mask,maskInv);
          depthImg.copyTo(gndFilteredImg, maskInv);
          if(generated_mask) *generated_mask = maskInv.clone();
     }
     if(filtered_img) *filtered_img = gndFilteredImg.clone();
     return 0;
}
int VboatsRos::remove_objects(const cv::Mat& vmap, const cv::Mat& disparity,
     const cv::Mat& depth, const vector<vector<cv::Point>>& contours,
     float* line_params, cv::Mat* filtered_img, cv::Mat* generated_mask, bool debug_timing)
{
     int err = 0;
     cv::Mat depthImg, disparityImg, refImg;
     /** If depth image is empty, exit since we have no image to filter objects from */
     if(depth.empty()){
          printf("[WARNING] %s::remove_objects() --- Depth Image is empty, skipping object removal.\r\n", this->classLbl.c_str());
          return -1;
     }
     if(vmap.empty()){
          printf("[WARNING] %s::remove_objects() --- Vmap input Image is empty, skipping object removal.\r\n", this->classLbl.c_str());
          return -2;
     }
     if(disparity.empty()){
          printf("[WARNING] %s::remove_objects() --- Disparity input Image is empty, skipping object removal.\r\n", this->classLbl.c_str());
          return -3;
     }
     refImg = vmap.clone();
     depthImg = depth.clone();
     disparityImg = disparity.clone();
     if(this->_debug) printf("[INFO] VboatsRos::remove_objects() --- Initializing key variables.\r\n");

     /** Image-dependent variable initialization */
     int h = vmap.rows, w = vmap.cols;
     cv::Mat depthMask = cv::Mat::zeros(disparity.size(), CV_8UC1);
     cv::Mat vmask = cv::Mat::zeros(h,w, CV_8UC1);

     cv::Mat roiDisplay;
     if(this->_visualize_segmented_obstacle_regions){
          if(refImg.channels() < 3) cv::cvtColor(refImg, roiDisplay, cv::COLOR_GRAY2BGR);
          else roiDisplay = refImg.clone();
     }

     /** Initialize default coefficients in case of ground line estimation failure */
     int y0 = 0, yf = h;
     int d0 = 0, df = w;
     int b; float slope;
     if(line_params){
          slope = line_params[0];
          b = (int) line_params[1];
     } else{
          slope = 0.0;
          b = yf;
     }
     if(this->_verbose) printf("[INFO] VboatsRos::remove_objects() --- Using linear coefficients: slope = %.2f, intercept = %d\r\n", slope, b);

     /** Initialize storage variables */
     if(this->_debug) printf("[INFO] VboatsRos::remove_objects() --- Beginning contour loop process.\r\n");
     int dmin, dmid, dmax;
     vector<int> xLims, dLims;
     for(int i = 0; i < contours.size(); i++){
          vector<cv::Point> contour = contours[i];
          this->vb->extract_contour_bounds(contour,&xLims, &dLims);
          dmin = dLims.at(0);
          dmax = dLims.at(1);
          dmid = (int)( (float)(dmin + dmax) / 2.0 );
          yf = (int)( (float)dmid * slope) + b - this->_gnd_line_upper_offset;
          /** NOTE: Previously used logic
          if(yf >= h) yf = h;
          else if(yf < 0) yf = 0;
          */
          if(yf >= h) yf = h;
          else if(yf < 0){
               if(slope != 0) yf = 0;
               else yf = h;
          }
          if(this->_debug) printf("[INFO] VboatsRos::remove_objects() ------ Contour[%d]: dmin, dmid, dmax = (%d, %d, %d) |  ROI Rect = (%d, %d) -> (%d, %d)\r\n", i, dmin, dmid, dmax, dmin,y0, dmax,yf);

          /** Extract ROI containing current contour data from vmap for processing */
          cv::Rect roiRect = cv::Rect( cv::Point(dmin,y0), cv::Point(dmax,yf) );
          cv::Mat roi = refImg(roiRect);
          roi.copyTo(vmask(roiRect));

          if(this->_visualize_segmented_obstacle_regions){ cv::rectangle(roiDisplay, roiRect, cv::Scalar(0, 255, 255), 1); }
     }
     if(this->_debug) printf("[INFO] VboatsRos::remove_objects() --- Creating Depth image mask.\r\n");
     double testT1 = (double)cv::getTickCount();

     ForEachObsMaskGenerator masker(vmask, h, 256);
     cv::Mat dMask = disparityImg.clone();
     dMask.forEach<uchar>(masker);
     if(debug_timing){
          double testDt1 = ((double)cv::getTickCount() - testT1)/cv::getTickFrequency();
          printf("[INFO] VboatsRos::remove_objects(): Segmentation mask generation Time: ForEach = %.4lf ms (%.2lf Hz)\r\n", testDt1*1000.0, (1.0/testDt1));
     }

     if(generated_mask) *generated_mask = dMask.clone();
     if(!depthMask.empty()){
          cv::Mat obsFilteredImg;
          depthImg.copyTo(obsFilteredImg, dMask);
          if(filtered_img) *filtered_img = obsFilteredImg.clone();

          if(this->_visualize_segmented_obstacle_regions){
               if(!roiDisplay.empty()) cv::imshow("Obstacle Segmented Regions", roiDisplay);
               if(!dMask.empty()){
                    cv::Mat display;
                    cv::applyColorMap(dMask, display, cv::COLORMAP_JET);
                    cv::imshow("Constructed Obstacle Segmentation Mask", display);
               }
          }
     } else{
          printf("[WARNING] VboatsRos::remove_objects() --- Object mask is empty, skipping object removal.\r\n");
          err = -2;
     }
     masker.remove();
     return err;
}

// int VboatsRos::process(const cv::Mat& disparity, const cv::Mat& umap, const cv::Mat& vmap, vector<Obstacle>* obstacles, const cv::Mat& depth){
int VboatsRos::process(const cv::Mat& depth, const cv::Mat& disparity,
     const cv::Mat& umap, const cv::Mat& vmap, vector<Obstacle>* obstacles,
     cv::Mat* filtered, cv::Mat* processed_umap, cv::Mat* processed_vmap)
{
     int nObs = 0;
     vector<Obstacle> _obstacles;
     cv::Mat tmpDepth, tmpDisparity, uTmp, vCopy;
     if(depth.empty()){
          printf("[WARNING] %s::process() --- Depth input is empty, skipping ground removal.\r\n", this->classLbl.c_str());
          return -1;
     }
     if(disparity.empty()){
          printf("[WARNING] %s::process() --- Disparity input is empty, skipping ground removal.\r\n", this->classLbl.c_str());
          return -2;
     }
     if(umap.empty()){
          printf("[WARNING] %s::process() --- Umap input is empty, skipping ground removal.\r\n", this->classLbl.c_str());
          return -3;
     }
     if(vmap.empty()){
          printf("[WARNING] %s::process() --- Vmap input is empty, skipping ground removal.\r\n", this->classLbl.c_str());
          return -4;
     }
     uTmp = umap.clone();
     vCopy = vmap.clone();
     tmpDepth = depth.clone();
     tmpDisparity = disparity.clone();
     if(this->_debug) printf("[INFO] VboatsRos::process() --- Pre-filtering Umap.\r\n");

     float Tx = (float)(this->_fx * this->_baseline);
     int maxDisparity = (int) ceil(Tx / (float) this->_cam_min_depth);
     int minDisparity = (int) ceil(Tx / (float) this->_cam_max_depth);

     /** Pre-filter Umap */
     cv::Mat uProcessed;
     vector<vector<cv::Point>> deadzoneUmap;
     vector<cv::Point> deadzonePtsUmap = {
          cv::Point(0,0),
          cv::Point(umap.cols,0),
          cv::Point(umap.cols,minDisparity),
          cv::Point(0,minDisparity),
          cv::Point(0,0)
     };
     deadzoneUmap.push_back(deadzonePtsUmap);
     cv::fillPoly(uTmp, deadzoneUmap, cv::Scalar(0));
     cv::boxFilter(uTmp,uTmp,-1, cv::Size(2,2));
     if(this->_use_custom_umap_filtering){
          cv::Mat uSobIn, uSobel, uSobelThresh, uSobelBlur, uSobelMask, custUmap;

          uTmp.convertTo(uSobIn, CV_64F);
          threshold(uSobIn, uSobel, this->_pre_blur_thresh, 255, cv::THRESH_TOZERO);
          if(this->_primary_blur){
               cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                    cv::Size(this->_kernel_x_multiplier*this->_umask_primary_blur_size,
                         this->_kernel_y_multiplier*this->_umask_primary_blur_size)
               );
               cv::dilate(uSobel, uSobel, element, cv::Point(-1,-1), 1);
          }

          // if(this->_primary_blur) cv::blur(uSobel, uSobel,
          //      cv::Size(this->_kernel_x_multiplier*this->_umask_primary_blur_size,
          //           this->_kernel_y_multiplier*this->_umask_primary_blur_size)
          //      );
          cv::Sobel(uSobel, uSobel, CV_64F, 0, 1, 3);
          double minUVal, maxUVal;
          cv::minMaxLoc(uSobel, &minUVal, &maxUVal);
          uSobel = uSobel * (255.0/maxUVal);
          cv::convertScaleAbs(uSobel, uSobel, 1, 0);
          threshold(uSobel, uSobelThresh, this->_umap_sobel_thresh, 255, cv::THRESH_TOZERO);
          // imshowCmap(uSobel, "Umap Sobel");
          // imshowCmap(uSobelThresh, "Umap Sobel Thresholded");

          if(this->_secondary_dilate){
               cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                    cv::Size(this->_kernel_x_multiplier*this->_umask_secondary_dilate_size,
                         this->_kernel_y_multiplier*this->_umask_secondary_dilate_size)
               );
               cv::dilate(uSobelThresh, uSobelThresh, element, cv::Point(-1,-1), 1);
          }

          if(this->_secondary_blur) cv::blur(uSobelThresh, uSobelBlur,
               cv::Size(this->_kernel_x_multiplier*this->_umask_secondary_blur_size,
                    this->_kernel_y_multiplier*this->_umask_secondary_blur_size)
               );
          else uSobelBlur = uSobelThresh;
          // threshold(uSobelBlur, uSobelMask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
          threshold(uSobelBlur, uSobelMask, 0, 255, cv::THRESH_BINARY);

          uTmp.copyTo(custUmap, uSobelMask);
          threshold(custUmap, uProcessed, this->_post_blur_thresh, 255, cv::THRESH_TOZERO);

          // imshowCmap(uSobelMask, "Umap Sobel Mask");
          // imshowCmap(uProcessed, "Custom Umap");
          // if(!this->_do_cv_wait_key) cv::waitKey(1);
     } else{
          vector<float> threshsU(this->_uThreshs);
          this->vb->filter_disparity_umap(uTmp, &uProcessed, &threshsU);
     }
     // else{
     //      double uMin, uMax;
     //      cv::minMaxLoc(uTmp, &uMin, &uMax);
     //      int wholeThresh = int((float) uMax * this->_custom_uThresh_perc);
     //      cv::threshold(uTmp, uProcessed, wholeThresh, 255, cv::THRESH_TOZERO);
     // }

     if(this->_debug) printf("[INFO] VboatsRos::process() --- Finding contours in filtered Umap.\r\n");
     /** Find contours in Umap needed later for obstacle filtering */
     vector<vector<cv::Point>> contours;
     this->vb->find_contours(uProcessed, &contours, this->_contourFiltMeth, this->_contourFiltMinThresh, nullptr, -1, false, this->_visualize_umap_contours);

     if(this->_debug) printf("[INFO] VboatsRos::process() --- Pre-filtering Vmap.\r\n");
     /** Pre-filter Vmap: Approach 1 */
     cv::Mat vTmp, vProcessed;
     vTmp = vCopy.clone();
     vector<vector<cv::Point>> deadzoneVmap;
     vector<cv::Point> deadzonePtsVmap = {
          cv::Point(0,0),
          cv::Point(0,vmap.rows),
          cv::Point(minDisparity,vmap.rows),
          cv::Point(minDisparity,0),
          cv::Point(0,0)
     };
     vector<cv::Point> deadzonePts2Vmap = {
          cv::Point(vmap.cols, 0),
          cv::Point(vmap.cols, vmap.rows),
          cv::Point(maxDisparity,vmap.rows),
          cv::Point(maxDisparity,0),
          cv::Point(vmap.cols,0)
     };
     deadzoneVmap.push_back(deadzonePtsVmap);
     deadzoneVmap.push_back(deadzonePts2Vmap);
     cv::fillPoly(vTmp, deadzoneVmap, cv::Scalar(0));

     cv::Mat sobelRawInput, sobelInput, sobelPreThresh, rawSobel, blurSobel, dilatedSobel, sobelThresh, sobelPreprocessed, sobelMask;
     if(this->_do_sobel_pre_thresholding){
          threshold(vTmp, sobelPreThresh, this->_sobel_pre_thresh, 255, cv::THRESH_TOZERO);
          sobelRawInput = sobelPreThresh.clone();
     } else sobelRawInput = vTmp.clone();

     if(this->_use_custom_vmap_filtering){
          if(this->_debug) printf("[INFO] VboatsRos::process() --- Creating Sobelized Vmap.\r\n");
          sobelRawInput.convertTo(rawSobel, CV_64F);
          sobelInput = rawSobel.clone();
          if(this->_do_vmap_sobel_blurring){
               cv::blur(rawSobel, blurSobel, cv::Size(this->_sobel_blur_kernel_size,this->_sobel_blur_kernel_size));
               sobelInput = blurSobel.clone();
          }
          if(this->_do_vmap_sobel_dilation){
               cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( this->_sobel_dilate_kernel_size, this->_sobel_dilate_kernel_size));
               cv::dilate(sobelInput, dilatedSobel, element, cv::Point(-1,-1), 1);
               sobelInput = dilatedSobel.clone();
          }
          if(!this->_use_scharr) cv::Sobel(sobelInput, sobelInput, CV_64F, 0, 1, this->_vmap_sobel_passes);
          else cv::Scharr(sobelInput, sobelInput, CV_64F, 0, 1, this->_vmap_sobel_passes);
          rawSobel = sobelInput.clone();

          double minVal, maxVal;
          cv::minMaxLoc(sobelInput, &minVal, &maxVal);
          sobelInput = sobelInput * (255.0/maxVal);
          cv::convertScaleAbs(sobelInput, sobelInput, 1, 0);
          threshold(sobelInput, sobelThresh, this->_vmap_sobel_thresh, 255, cv::THRESH_TOZERO);
     } else{
          vector<float> threshsV(this->_vThreshs);
          this->vb->filter_disparity_vmap(sobelRawInput, &sobelThresh, &threshsV);
     }
     sobelPreprocessed = sobelThresh.clone();
     if(this->_debug) printf("[INFO] %s::process() --- Looking for Ground line.\r\n", this->classLbl.c_str());

     /** Extract ground line parameters (if ground is present) */
     float* line_params;
     float gndM; int gndB;
     bool gndPresent = this->vb->find_ground_line(sobelPreprocessed, &gndM,&gndB, this->_gnd_line_min_ang, this->_gnd_line_max_ang);
     if(gndPresent){
          float tmpParams[] = {gndM, (float) gndB};
          line_params = &tmpParams[0];
     } else line_params = nullptr;
     if(this->_debug) printf("[INFO] %s::process() --- Creating Segmentation Mask.\r\n", this->classLbl.c_str());

     cv::Mat sobelSecThresh, sobelSecDilate, sobelSecBlur, segMask;
     if(this->_do_sobel_vmask_subtraction){
          if(this->_do_vmap_sobel_sec_thresh){
               threshold(sobelThresh, sobelSecThresh, this->_vmap_sobel_sec_thresh, 255, cv::THRESH_TOZERO);
          } else sobelSecThresh = sobelThresh.clone();

          if(this->_do_vmap_sobel_sec_blurring){
               cv::blur(sobelSecThresh, sobelSecBlur, cv::Size(this->_sobel_sec_blur_kernel_size,this->_sobel_sec_blur_kernel_size));
          } else sobelSecBlur = sobelSecThresh.clone();

          if(this->_do_vmap_sobel_sec_dilation){
               cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( this->_sobel_sec_dilate_kernel_size, this->_sobel_sec_dilate_kernel_size));
               cv::dilate(sobelSecBlur, sobelSecDilate, element, cv::Point(-1,-1), 1);
               segMask = sobelSecDilate.clone();
          } else segMask = sobelSecBlur.clone();

          threshold(segMask, segMask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
          cv::bitwise_not(segMask,segMask);
          vTmp.copyTo(vProcessed, segMask);
     } else vProcessed = vTmp.clone();

     threshold(vProcessed, vProcessed, this->_vmap_thresh, 255, cv::THRESH_TOZERO);

     if(this->_debug) printf("[INFO] %s::process() --- Performing Obstacle Segmentation.\r\n", this->classLbl.c_str());
     // cvinfo(depth, "VboatsHandler::process() --- Depth before ground segmentation: ");
     // cvinfo(vProcessed, "VboatsHandler::process() --- Vmap input before ground segmentation: ");
     int err;
     cv::Mat noGndImg, noObsImg, gndMask, objMask, filtered_image, filtered_depth;
     if(this->_do_object_segmented_filtering){
          err = this->remove_objects(vProcessed, tmpDisparity, tmpDepth, contours, line_params, &noObsImg, &objMask);
          if(err >= 0){
               if(!noObsImg.empty()) filtered_image = noObsImg.clone();
               else printf("[WARNING] %s::process() --- Object filtered depth image is empty, skipping pointcloud generation.\r\n", this->classLbl.c_str());
          } else printf("[WARNING] %s::process() --- Unable to filter objects from depth image, skipping pointcloud generation.\r\n", this->classLbl.c_str());
     } else filtered_image = tmpDepth.clone();

     if((gndPresent) && this->_use_gnd_line_based_removal){                         /** Ground segmentation */
          if(filtered_image.empty()) filtered_image = tmpDepth.clone();
          err = this->remove_ground(tmpDisparity,vProcessed, filtered_image, line_params, &noGndImg, &gndMask);
          if(err >= 0){
               if(!noGndImg.empty()) filtered_depth = noGndImg.clone();
               else printf("[WARNING] %s::process() --- Ground filtered depth image is empty, skipping pointcloud generation.\r\n", this->classLbl.c_str());
          } else printf("[WARNING] %s::process() --- Unable to filter ground from depth image, skipping pointcloud generation.\r\n", this->classLbl.c_str());
     }

     cv::Mat morphedDepth;
     if(this->_do_post_depth_morphing){
          cv::Mat morphElement = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( this->_depth_morph_kernel_size, this->_depth_morph_kernel_size));
          cv::morphologyEx(filtered_image, morphedDepth, cv::MORPH_OPEN, morphElement);
          filtered_depth = morphedDepth.clone();
     } else filtered_depth = filtered_image.clone();

     // Obstacle data extractionaa
     cv::Mat obsSearchVmap;
     std::vector< std::vector<cv::Rect> > objectsWindows;
     if(this->_do_individual_obstacle_detection){
          if(this->_use_filtered_depth_object_detection){
               cv::Mat genFilteredDisparity, newUmap, newVmap;
               double disparityGain = (double) this->_depth2disparityFactor;
               this->depth_to_disparity(filtered_depth, &genFilteredDisparity, disparityGain);
               if(!genFilteredDisparity.empty()){
                    genUVMapThreaded(genFilteredDisparity,&newUmap,&newVmap, 2.0);
                    if(!newVmap.empty()) obsSearchVmap = newVmap;
                    else obsSearchVmap = vProcessed;
               } else obsSearchVmap = vProcessed;
          } else obsSearchVmap = vProcessed;

          if(this->_visualize_obstacle_search_windows) nObs = this->vb->find_obstacles_disparity(obsSearchVmap, contours, &_obstacles, line_params, &objectsWindows);
          else nObs = this->vb->find_obstacles_disparity(obsSearchVmap, contours, &_obstacles, line_params);
     }

     // Return Output images if requested before visualization
     if(obstacles) *obstacles = _obstacles;
     if(filtered) *filtered = filtered_depth.clone();
     if(processed_umap) *processed_umap = uProcessed.clone();
     if(processed_vmap) *processed_vmap = vProcessed.clone();

     if(this->_do_cv_wait_key){
          if(this->_visualize_gnd_mask) imshowCmap(gndMask, "Ground-Line Mask");
          if(this->_visualize_obj_mask) imshowCmap(objMask, "Object Mask");
          if(this->_visualize_gnd_filter_img) imshowCmap(noGndImg, "Ground-Line Filtered Image");
          if(this->_visualize_obj_filter_img) imshowCmap(noObsImg, "Object Filtered Image");
          if(this->_visualize_generated_disparity) imshowCmap(tmpDisparity, "Generated Disparity");

          if(this->_do_vmap_viz){
               std::vector<cv::Mat> vmapsDisps;
               if(this->_visualize_vmap_raw || this->_visualize_vmap_raw_w_lines){
                    cv::Mat vmapCmap = imCvtCmap(vTmp);
                    if(this->_visualize_vmap_raw && (!vmapCmap.empty()) ){
                         cv::putText(vmapCmap, "Vmap Raw", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                         vmapsDisps.push_back(vmapCmap);
                    }
                    if(this->_visualize_vmap_raw_w_lines && (!vmapCmap.empty()) && gndPresent ){
                         cv::Mat lineDisplay = vmapCmap.clone();
                         int yk = int(vmapCmap.cols * gndM) + gndB;
                         int yu = int(vmapCmap.cols * gndM) + (gndB-this->_gnd_line_upper_offset);
                         int yl = int(vmapCmap.cols * gndM) + (gndB+this->_gnd_line_lower_offset);
                         cv::line(lineDisplay, cv::Point(0, gndB), cv::Point(vmapCmap.cols, yk), cv::Scalar(0,255,0), 2, cv::LINE_AA);
                         cv::line(lineDisplay, cv::Point(0, (gndB-this->_gnd_line_upper_offset)), cv::Point(vmapCmap.cols, yu), cv::Scalar(255,0,0), 2, cv::LINE_AA);
                         cv::line(lineDisplay, cv::Point(0, (gndB+this->_gnd_line_lower_offset)), cv::Point(vmapCmap.cols, yl), cv::Scalar(0,0,255), 2, cv::LINE_AA);
                         vmapsDisps.push_back(lineDisplay);
                    }
               }
               cv::Mat tmpVmapDisp;
               if(this->_visualize_vmap_blurred && (!blurSobel.empty())){
                    tmpVmapDisp = imCvtCmap(blurSobel);
                    if(!tmpVmapDisp.empty()){
                         cv::putText(tmpVmapDisp, "Vmap Blurred", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                         vmapsDisps.push_back(tmpVmapDisp);
                    }
               }
               if(this->_visualize_vmap_dilated && (!dilatedSobel.empty())){
                    tmpVmapDisp = imCvtCmap(dilatedSobel);
                    if(!tmpVmapDisp.empty()){
                         cv::putText(tmpVmapDisp, "Vmap Dilated", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                         vmapsDisps.push_back(tmpVmapDisp);
                    }
               }
               if(this->_visualize_vmap_sobel_raw && (!rawSobel.empty()) ){
                    tmpVmapDisp = imCvtCmap(rawSobel);
                    if(!tmpVmapDisp.empty()){
                         cv::putText(tmpVmapDisp, "Unprocessed Sobel", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                         vmapsDisps.push_back(tmpVmapDisp);
                    }
               }
               if(this->_visualize_vmap_thresh && (!sobelThresh.empty()) ){
                    tmpVmapDisp = imCvtCmap(sobelThresh);
                    if(!tmpVmapDisp.empty()){
                         cv::putText(tmpVmapDisp, "Vmap Thresholded", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                         vmapsDisps.push_back(tmpVmapDisp);
                    }
               }
               if(this->_visualize_vmap_sec_thresh && (!sobelSecThresh.empty()) ){
                    tmpVmapDisp = imCvtCmap(sobelSecThresh);
                    if(!tmpVmapDisp.empty()){
                         cv::putText(tmpVmapDisp, "Sobel Secondary Thresholding", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                         vmapsDisps.push_back(tmpVmapDisp);
                    }
               }
               if(this->_visualize_vmap_sec_dilated && (!sobelSecDilate.empty()) ){
                    tmpVmapDisp = imCvtCmap(sobelSecDilate);
                    if(!tmpVmapDisp.empty()){
                         cv::putText(tmpVmapDisp, "Sobel Secondary Dilate", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                         vmapsDisps.push_back(tmpVmapDisp);
                    }
               }
               if(this->_visualize_vmap_sec_blur && (!sobelSecBlur.empty()) ){
                    tmpVmapDisp = imCvtCmap(sobelSecBlur);
                    if(!tmpVmapDisp.empty()){
                         cv::putText(tmpVmapDisp, "Sobel Secondary Blurring", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                         vmapsDisps.push_back(tmpVmapDisp);
                    }
               }
               if(this->_visualize_vmap_mask && (!segMask.empty()) ){
                    tmpVmapDisp = imCvtCmap(segMask);
                    if(!tmpVmapDisp.empty()){
                         cv::putText(tmpVmapDisp, "Vmap Mask", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                         vmapsDisps.push_back(tmpVmapDisp);
                    }
               }
               if(this->_visualize_process_input_vmap || this->_visualize_obstacle_search_windows){
                    cv::Mat vmapProcDisp = imCvtCmap(vProcessed);
                    if(this->_visualize_process_input_vmap && (!vmapProcDisp.empty()) ){
                         cv::putText(vmapProcDisp, "Processing Input", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                         vmapsDisps.push_back(vmapProcDisp);
                    }
                    if(this->_visualize_obstacle_search_windows && (!vmapProcDisp.empty()) && (objectsWindows.size() != 0) ){
                         cv::Mat windowDisplay = vmapProcDisp.clone();
                         for(auto objWindows : objectsWindows){
                              for(cv::Rect window : objWindows){
                                   cv::rectangle(windowDisplay, window, cv::Scalar(0, 255, 255), 1);
                              }
                         }
                         vmapsDisps.push_back(windowDisplay);
                    }
               }
               if(this->_visualize_obj_detection_vmap){
                    tmpVmapDisp = imCvtCmap(obsSearchVmap);
                    if(!tmpVmapDisp.empty()){
                         cv::putText(tmpVmapDisp, "Object Detection Vmap", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, cv::LINE_AA);
                         vmapsDisps.push_back(tmpVmapDisp);
                    }
               }
               cv::Mat vmapsMerged;
               if(vmapsDisps.size() > 0){
                    cv::hconcat(vmapsDisps, vmapsMerged);
                    cv::namedWindow("Vmaps", cv::WINDOW_NORMAL);
                    cv::imshow("Vmaps", vmapsMerged);
               }
          }
          if(this->_visualize_umap_raw && (!uTmp.empty()) ){
               cv::Mat umapRawDisplay = imCvtCmap(uTmp);
               cv::namedWindow("Raw Umap", cv::WINDOW_NORMAL);
               cv::imshow("Raw Umap", umapRawDisplay);
          }
          if(this->_visualize_umap_filtered) imshowCmap(uProcessed, "Filtered Umap");
     }
     return (int) _obstacles.size();
}

/** SECTION:
     Execution Functions
*/
int VboatsRos::update(bool verbose, bool debug_timing){
     int nObs = 0;
     bool angCorrectionPerformed = false;
     double disparityGain;
     double correctionAngle;
     vector<Obstacle> obs;
     cv::Mat curDepth, curDisparity;
     cv::Mat genDisparity, depthInput, umap, vmap;
     /** Don't do any proessing if we haven't received valid camera info */
     if(!this->_recvd_cam_info) return -1;
     if(!this->_recvd_image) return -2;

     this->_lock.lock();
     curDepth = this->_depth;
     curDisparity = this->_disparity;
     disparityGain = (double) this->_depth2disparityFactor;
     correctionAngle = (double) this->_correction_roll;
     this->_lock.unlock();

     // cvinfo(curDepth, "VboatsRos::update() --- Depth Image before preprocessing: ");
     // ForEachPrepareDepthConverter<float> preconverter((float) this->_cam_min_depth, (float) this->_cam_max_depth);
     ForEachSaturateDepthLimits<float> preconverter((float) this->_cam_min_depth, (float) this->_cam_max_depth,
          (float) this->_fx, (float) this->_fy, (float) this->_px, (float) this->_py,
          (float) this->_cam_min_depth_x, (float) this->_cam_max_depth_x,
          (float) this->_cam_min_depth_y, (float) this->_cam_max_depth_y
     );
     curDepth.forEach<float>(preconverter);
     // cvinfo(curDepth, "VboatsRos::update() --- Depth Image after preprocessing: ");

     if(this->_do_depth_based_processing){
          if(this->_do_angle_correction){
               cv::Mat warpedDepth = rotate_image(curDepth, correctionAngle);
               // cvinfo(warpedDepth, "VboatsRos::update() --- Depth Image after warping: ");
               if(!warpedDepth.empty() ){
                    depthInput = warpedDepth.clone();
                    angCorrectionPerformed = true;
                    if(this->_visualize_angle_corrected_depth && (!warpedDepth.empty())){
                         cv::Mat warpDepthViz = imCvtCmap(warpedDepth);
                         if(!warpDepthViz.empty()){
                              cv::namedWindow("Angle Corrected Depth", cv::WINDOW_NORMAL);
                              cv::imshow("Angle Corrected Depth", warpDepthViz);
                         }
                    }
               } else depthInput = curDepth.clone(); angCorrectionPerformed = false;
          } else depthInput = curDepth;

          this->depth_to_disparity(depthInput,&genDisparity, disparityGain);
     } else genDisparity = curDisparity;

     /** Don't do any proessing if disparity is empty */
     if(genDisparity.empty()) return -1;
     if(this->_debug_disparity_generation) cvinfo(genDisparity, "VboatsRos::update() --- Using Received Disparity Image: ");

     double t = (double)cv::getTickCount();
     genUVMapThreaded(genDisparity,&umap,&vmap, 2.0);

     cv::Mat procDepth, procUmap, procVmap;
     // nObs = this->process(genDisparity, umap, vmap, &obs, curDepth);
     nObs = this->process(depthInput, genDisparity, umap, vmap, &obs,
          &procDepth, &procUmap, &procVmap);
     if(this->_verbose_update) printf("[INFO] %s::update() --- Found %d obstacles.\r\n", this->classLbl.c_str(), nObs);

     cv::Mat filtered_depth;
     if(angCorrectionPerformed){
          filtered_depth = rotate_image(procDepth, -correctionAngle);
     } else filtered_depth = procDepth.clone();

     ros::Time procEndTime = ros::Time::now();
     if(this->_publish_aux_images){
          cv::Mat vmapProcDisp, umapProcDisplay;
          cv::Mat filterDepthDisp = imCvtCmap(filtered_depth);
          umapProcDisplay = imCvtCmap(procUmap);
          vmapProcDisp = imCvtCmap(procVmap);
          this->publish_images(&umapProcDisplay, &vmapProcDisp, &filterDepthDisp);
     }

     if(this->_do_individual_obstacle_detection && (nObs > 0)){
          int n = 0;
          cv::Mat display, output;
          if(this->_publish_obstacle_segmented_image){
               if(!filtered_depth.empty()){
                    display = filtered_depth.clone();
                    double minVal, maxVal;
                    cv::minMaxLoc(display, &minVal, &maxVal);
                    display.convertTo(output, CV_8UC1, (255.0/maxVal) );
                    cv::cvtColor(output, output, cv::COLOR_GRAY2BGR);
               }
          }

          swanson_msgs::VboatsObstacles obsMsg;
          for(Obstacle ob : obs){
               if(this->_verbose_obstacles) printf("Obstacle [%d]: ", n+1);
               ob.update(false,this->_baseline, this->_dscale, this->_focal, this->_principle, 1.0, 1.0, this->_verbose_obstacles);

               if(this->_publish_obs_data){
                    swanson_msgs::VboatsObstacle tmpOb;
                    tmpOb.header.seq = n;
                    tmpOb.header.stamp = obsMsg.header.stamp;
                    tmpOb.header.frame_id = obsMsg.header.frame_id;
                    tmpOb.distance = ob.distance;
                    tmpOb.angle = ob.angle;
                    tmpOb.position.x = ob.location.x;
                    tmpOb.position.y = ob.location.y;
                    tmpOb.position.z = ob.location.z;
                    obsMsg.obstacles.push_back(tmpOb);
               }
               if(this->_publish_obstacle_segmented_image && (!output.empty()) ){
                    cv::rectangle(output, ob.minXY, ob.maxXY, cv::Scalar(255, 0, 255), 1);
               }

               n++;
          }
          if(this->_publish_obs_data){
               obsMsg.header.stamp = ros::Time::now();
               obsMsg.header.frame_id = this->_camera_tf;
               this->_detected_obstacle_info_pub.publish(obsMsg);
          }
          // cvinfo(output, "Individual obstacles: ");
          if(this->_publish_obstacle_segmented_image) this->publish_obstacle_image(output);
     }

     /** Publish raw pointcloud extrapolated for uncut depth map */
     if(this->_flag_pub_unfiltered_cloud || this->_flag_pub_filtered_cloud){
          cloudxyz_t::Ptr obsFilteredCloud = this->generate_cloud_from_depth(filtered_depth);
          if(obsFilteredCloud->points.size() != 0){
               this->_unfiltered_cloud_pub.publish(obsFilteredCloud);
               // if(unfiltered_cloud) pcl::copyPointCloud(*unfiltCloud, *unfiltered_cloud);
               if(this->_flag_pub_filtered_cloud){
                    cloudxyz_t::Ptr filtCloud = this->filter_pointcloud(obsFilteredCloud);
                    if(filtCloud->points.size() != 0) this->_filtered_cloud_pub.publish(filtCloud);
               }
          }
     }
     if(this->_flag_pub_unfiltered_cloud){
          cloudxyz_t::Ptr rawCloud = this->generate_cloud_from_depth(curDepth);
          if(rawCloud->points.size() != 0) this->_raw_cloud_pub.publish(rawCloud);
     }
     this->_count++;

     if(this->_do_cv_wait_key) cv::waitKey(1);
     return 0;
}
int VboatsRos::run(bool verbose){
     double t = (double)cv::getTickCount();
     while(ros::ok()){
          this->update();
          ros::spinOnce();
          this->_loop_rate->sleep();
     }

     return 0;
}

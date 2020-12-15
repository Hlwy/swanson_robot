#include <iostream>
#include "swanson_algorithms/vboats_ros.h"

#include <robocommander/utilities/utils.h>
#include <robocommander/utilities/image_utils.h>
#include <robocommander/algorithms/vboats/vboats.h>

#include <swanson_msgs/VboatsObstacle.h>
#include <swanson_msgs/VboatsObstacles.h>

using namespace std;

/** SECTION:
     CONSTRUCTOR & DECONSTRUCTOR
*/
VboatsRos::VboatsRos(ros::NodeHandle nh, ros::NodeHandle _nh) : m_nh(nh), p_nh(_nh), _it(nh){
     // Flag Configuration
     int update_rate;
     bool flag_use_tf_prefix = false;
     std::string namespaced = m_nh.getNamespace();
     p_nh.getParam("namespace",namespaced);
     p_nh.param<int>("update_rate", update_rate, 30);
     this->_ns = namespaced;
     p_nh.getParam("use_cuda",      this->_try_cuda);

     // ROS Object Configuration
     {
          // ROS Subscribers
          std::string depth_image_topic           = "camera/depth/image_rect_raw";
          std::string camera_info_topic           = "camera/depth/camera_info";
          std::string imu_topic                   = "";
          std::string pose_topic                  = "";
          std::string pose_stamped_topic          = "";
          p_nh.getParam("depth_image_topic",      depth_image_topic);
          p_nh.getParam("camera_info_topic",      camera_info_topic);
          p_nh.getParam("imu_topic",              imu_topic);
          p_nh.getParam("pose_topic",             pose_topic);
          p_nh.getParam("pose_stamped_topic",     pose_stamped_topic);
          // Initialize ROS-Objects
          this->_depth_sub = m_nh.subscribe<sensor_msgs::Image>(depth_image_topic, 30, boost::bind(&VboatsRos::depthCallback,this,_1,5));
          this->_cam_info_sub = m_nh.subscribe<sensor_msgs::CameraInfo>(camera_info_topic, 1, boost::bind(&VboatsRos::infoCallback,this,_1,1));
          if(strcmp(pose_stamped_topic.c_str(), "") != 0){
               this->_pose_stamped_sub = m_nh.subscribe<geometry_msgs::PoseStamped>(pose_stamped_topic, 30, &VboatsRos::poseStampedCallback,this);
          } else if(strcmp(imu_topic.c_str(), "") != 0){
               this->_imu_sub = m_nh.subscribe<sensor_msgs::Imu>(imu_topic, 30, &VboatsRos::imuCallback,this);
          }

          // ROS Publishers
          std::string filtered_image_topic             = "vboats/depth/image_filtered";
          std::string raw_cloud_topic                  = "vboats/cloud/raw";
          std::string unfiltered_cloud_topic           = "vboats/cloud/pre_processed";
          std::string filtered_cloud_topic             = "vboats/cloud/filtered";
          std::string detected_obstacles_info_topic    = "vboats/obstacles/data";
          std::string corrected_depth_topic            = "vboats/depth/image_angle_corrected";
          std::string generated_disparity_topic        = "vboats/disparity/image_generated";
          std::string raw_umap_topic                   = "vboats/umap/image_raw";
          std::string raw_vmap_topic                   = "vboats/vmap/image_raw";
          std::string filt_umap_topic                  = "vboats/umap/image_filtered";
          std::string filt_vmap_topic                  = "vboats/vmap/image_filtered";
          // ROS Param Configuration
          p_nh.getParam("filtered_image_topic",        filtered_image_topic);
          p_nh.getParam("raw_cloud_topic",             raw_cloud_topic);
          p_nh.getParam("unfiltered_cloud_topic",      unfiltered_cloud_topic);
          p_nh.getParam("filtered_cloud_topic",        filtered_cloud_topic);
          p_nh.getParam("obstacles_info_topic",        detected_obstacles_info_topic);
          p_nh.getParam("angle_corrected_depth_topic", corrected_depth_topic);
          p_nh.getParam("generated_disparity_topic",   generated_disparity_topic);
          p_nh.getParam("raw_umap_topic",              raw_umap_topic);
          p_nh.getParam("raw_vmap_topic",              raw_vmap_topic);
          p_nh.getParam("filtered_umap_topic",         filt_umap_topic);
          p_nh.getParam("filtered_vmap_topic",         filt_vmap_topic);

          std::string gnd_line_mask_topic, gnd_line_vmask_topic, obj_candidates_mask_topic;
          std::string umap_debugging_topic, vmap_debugging_topic;
          std::string umap_mask_topic, vmap_mask_topic;
          std::string obstacle_markers_topic;
          p_nh.param<std::string>("gnd_line_mask_topic",       gnd_line_mask_topic,       "vboats/vmap/gnd_line_mask");
          p_nh.param<std::string>("gnd_line_vmask_topic",      gnd_line_vmask_topic,      "vboats/vmap/gnd_line_vmask");
          p_nh.param<std::string>("obj_candidates_mask_topic", obj_candidates_mask_topic, "vboats/vmap/obj_candidates_mask");
          p_nh.param<std::string>("vmap_mask_topic",           vmap_mask_topic,           "vboats/vmap/keep_mask");
          p_nh.param<std::string>("vmap_debug_topic",          vmap_debugging_topic,      "vboats/vmap/debug/low_lvl");
          p_nh.param<std::string>("umap_mask_topic",           umap_mask_topic,           "vboats/umap/keep_mask");
          p_nh.param<std::string>("umap_debug_topic",          umap_debugging_topic,      "vboats/umap/debug/low_lvl");
          p_nh.param<std::string>("obstacle_markers_topic",    obstacle_markers_topic,    "vboats/obstacles");

          // Initialize ROS-Objects
          // this->_filtered_depth_pub          = _it.advertise(filtered_image_topic, 1);
          this->_filtered_depth_pub          = _it.advertiseCamera(filtered_image_topic, 1);
          this->_raw_cloud_pub               = m_nh.advertise<cloudxyz_t>(raw_cloud_topic, 1);
          this->_unfiltered_cloud_pub        = m_nh.advertise<cloudxyz_t>(unfiltered_cloud_topic, 1);
          this->_filtered_cloud_pub          = m_nh.advertise<cloudxyz_t>(filtered_cloud_topic, 1);
          this->_detected_obstacle_info_pub  = m_nh.advertise<swanson_msgs::VboatsObstacles>(detected_obstacles_info_topic, 100);
          this->_corrected_depth_pub         = m_nh.advertise<sensor_msgs::Image>(corrected_depth_topic, 1);
          this->_generated_disparity_pub     = m_nh.advertise<sensor_msgs::Image>(generated_disparity_topic, 1);
          this->_raw_umap_pub                = m_nh.advertise<sensor_msgs::Image>(raw_umap_topic, 1);
          this->_raw_vmap_pub                = m_nh.advertise<sensor_msgs::Image>(raw_vmap_topic, 1);
          this->_proc_umap_pub               = m_nh.advertise<sensor_msgs::Image>(filt_umap_topic, 1);
          this->_proc_vmap_pub               = m_nh.advertise<sensor_msgs::Image>(filt_vmap_topic, 1);
          this->_gnd_line_mask_pub           = m_nh.advertise<sensor_msgs::Image>(gnd_line_mask_topic, 1);
          this->_gnd_line_vmask_pub          = m_nh.advertise<sensor_msgs::Image>(gnd_line_vmask_topic, 1);
          this->_obj_candidate_mask_pub      = m_nh.advertise<sensor_msgs::Image>(obj_candidates_mask_topic, 1);
          this->_umap_keep_mask_pub          = m_nh.advertise<sensor_msgs::Image>(umap_mask_topic, 1);
          this->_vmap_keep_mask_pub          = m_nh.advertise<sensor_msgs::Image>(vmap_mask_topic, 1);
          this->_umap_debug_pub              = m_nh.advertise<sensor_msgs::Image>(umap_debugging_topic, 1);
          this->_vmap_debug_pub              = m_nh.advertise<sensor_msgs::Image>(vmap_debugging_topic, 1);
          this->_obstacle_markers_pub        = m_nh.advertise<visualization_msgs::MarkerArray>(obstacle_markers_topic, 10);
     }

     // ROS tf frames Configuration
     {
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
     }

     // Initialize VBOATS
     this->vb = new Vboats();

     this->_pause_service = m_nh.advertiseService("vboats/pause", &VboatsRos::pause_callback, this);
     this->_resume_service = m_nh.advertiseService("vboats/resume", &VboatsRos::resume_callback, this);
     this->_correction_angle_calibration_service = m_nh.advertiseService("vboats/calibrate_correction_angle", &VboatsRos::calibrate_orientation_offsets_callback, this);
     this->_cfg_f = boost::bind(&VboatsRos::cfgCallback, this, _1, _2);
     this->_cfg_server.setCallback(this->_cfg_f);
     ROS_INFO("[INFO] VboatsRos::VboatsRos() ---- Successfully Initialized!");
}
VboatsRos::~VboatsRos(){ delete this->vb; }

/** -------------------------------------------------------------------------------------------------
*                                     ROS Subscriber Callbacks
* ------------------------------------------------------------------------------------------------- */
void VboatsRos::cfgCallback(swanson_algorithms::VboatsConfig &config, uint32_t level){
     std::lock_guard<std::mutex> lock(_lock);

     // Debugging Printouts
     {
          if(config.verbose_update != this->_verbose_update){
               this->_verbose_update = config.verbose_update;
          }
          if(config.verbose_obstacles != this->_verbose_obstacles){
               this->_verbose_obstacles = config.verbose_obstacles;
          }
          if(config.debug_timings != this->_debug_timings){
               this->_debug_timings = config.debug_timings;
               this->vb->enable_image_processing_timings_debug(this->_debug_timings);
          }
          if(config.debug_disparity_generation != this->_debug_disparity_generation){
               this->_debug_disparity_generation = config.debug_disparity_generation;
          }
          if(config.debug_ground_line_params != this->_debug_ground_line_params){
               this->_debug_ground_line_params = config.debug_ground_line_params;
          }
     }

     // Umap Filtering
     {
          this->vb->set_umap_processing_method(config.umap_filtering_method);
          this->vb->set_contour_filtering_method(config.contour_filter_method);

          if(config.contour_min_thresh != this->vb->umapParams.contour_filtering_thresh_min){
               this->vb->umapParams.set_contour_filtering_threshold_min(config.contour_min_thresh);
          }
          if(config.umap_pre_sobelizing_thresh != this->vb->umapParams.sobel_thresh_pre_sobel){
               this->vb->umapParams.set_threshold_before_sobelizing(config.umap_pre_sobelizing_thresh);
          }
          if(config.umap_sobelized_thresh_prefiltering != this->vb->umapParams.sobel_thresh_sobel_preprocess){
               this->vb->umapParams.set_threshold_sobelize_preprocessing(config.umap_sobelized_thresh_prefiltering);
          }
          if(config.umap_sobelized_thresh_postfiltering != this->vb->umapParams.sobel_thresh_sobel_postprocess){
               this->vb->umapParams.set_threshold_sobelize_postprocessing(config.umap_sobelized_thresh_postfiltering);
          }
          if(config.umap_sobelized_dilate_size != this->vb->umapParams.sobel_dilate_size){
               if(config.umap_sobelized_dilate_size == 0) this->vb->umapParams.set_sobelize_dilate_size(1);
               else this->vb->umapParams.set_sobelize_dilate_size(config.umap_sobelized_dilate_size);
          }
          if(config.umap_sobelized_blur_size != this->vb->umapParams.sobel_blur_size){
               if(config.umap_sobelized_blur_size == 0) this->vb->umapParams.set_sobelize_blur_size(1);
               else this->vb->umapParams.set_sobelize_blur_size(config.umap_sobelized_blur_size);
          }
          if(config.ukernel_x_multiplier != this->vb->umapParams.sobel_kernel_multipliers[0]){
               if(config.ukernel_x_multiplier == 0) this->vb->umapParams.set_sobelize_kernel_x_multiplier(1);
               else this->vb->umapParams.set_sobelize_kernel_x_multiplier(config.ukernel_x_multiplier);
          }
          if(config.ukernel_y_multiplier != this->vb->umapParams.sobel_kernel_multipliers[1]){
               if(config.ukernel_y_multiplier == 0) this->vb->umapParams.set_sobelize_kernel_y_multiplier(1);
               else this->vb->umapParams.set_sobelize_kernel_y_multiplier(config.ukernel_y_multiplier);
          }
     }

     // Vmap Filtering
     {
          this->vb->set_vmap_processing_method(config.vmap_filtering_method);
          if(config.vmap_sobel_preprocessing_thresh != this->vb->vmapParams.sobel_preprocessing_thresh_sobel){
               this->vb->vmapParams.set_threshold_before_sobelizing(config.vmap_sobel_preprocessing_thresh);
          }
          if(config.vmap_sobel_postprocessing_thresh_prefiltering != this->vb->vmapParams.sobel_postprocessing_thresh_prefiltering){
               this->vb->vmapParams.set_threshold_sobelize_postprocessing_prefiltering(config.vmap_sobel_postprocessing_thresh_prefiltering);
          }
          if(config.vmap_sobel_postprocessing_thresh_postfiltering != this->vb->vmapParams.sobel_postprocessing_thresh_postfiltering){
               this->vb->vmapParams.set_threshold_sobelize_postprocessing_postfiltering(config.vmap_sobel_postprocessing_thresh_postfiltering);
          }
          if(config.vmap_sobel_preprocessing_blur_size != this->vb->vmapParams.sobel_preprocessing_blur_size){
               this->vb->vmapParams.set_sobelize_preprocessing_blur_size(config.vmap_sobel_preprocessing_blur_size);
          }
          if(config.vmap_sobel_postprocessing_blur_size != this->vb->vmapParams.sobel_postprocessing_blur_size){
               this->vb->vmapParams.set_sobelize_postprocessing_blur_size(config.vmap_sobel_postprocessing_blur_size);
          }
          if(config.vkernel_x_multiplier != this->vb->vmapParams.sobel_kernel_multipliers[0]){
               if(config.vkernel_x_multiplier == 0) this->vb->vmapParams.set_sobelize_kernel_x_multiplier(1);
               else this->vb->vmapParams.set_sobelize_kernel_x_multiplier(config.vkernel_x_multiplier);
          }
          if(config.vkernel_y_multiplier != this->vb->vmapParams.sobel_kernel_multipliers[1]){
               if(config.vkernel_y_multiplier == 0) this->vb->vmapParams.set_sobelize_kernel_y_multiplier(1);
               else this->vb->vmapParams.set_sobelize_kernel_y_multiplier(config.vkernel_y_multiplier);
          }
          if(config.gnd_line_search_hough_thresh != this->vb->vmapParams.gnd_line_search_hough_thresh){
               if(config.gnd_line_search_hough_thresh == 0) this->vb->vmapParams.set_ground_line_search_hough_threshold(1);
               else this->vb->vmapParams.set_ground_line_search_hough_threshold(config.gnd_line_search_hough_thresh);
          }
          if(config.gnd_line_search_deadzone != this->vb->vmapParams.gnd_line_search_deadzone){
               this->vb->vmapParams.set_ground_line_search_deadzone(config.gnd_line_search_deadzone);
          }
          if(config.gnd_line_search_min_deg != this->vb->vmapParams.gnd_line_search_min_deg){
               this->vb->vmapParams.set_ground_line_search_min_angle(config.gnd_line_search_min_deg);
          }
          if(config.gnd_line_search_max_deg != this->vb->vmapParams.gnd_line_search_max_deg){
               this->vb->vmapParams.set_ground_line_search_max_angle(config.gnd_line_search_max_deg);
          }
          if(config.gnd_line_intercept_offset != this->vb->vmapParams.depth_filtering_gnd_line_intercept_offset){
               this->vb->vmapParams.set_depth_filtering_ground_line_intercept_offset(config.gnd_line_intercept_offset);
          }
     }

     // Misc Depth Image Filtering
     {
          this->vb->enable_angle_correction(config.do_angle_correction);
          this->vb->set_image_angle_correction_type(config.angle_correction_method);
          this->vb->enable_correction_angle_sign_flip(config.flip_correction_angle_sign);
          if(config.correction_angle_offset_deg != this->_user_angle_offset){
               this->_user_angle_offset = config.correction_angle_offset_deg;
               this->vb->set_camera_angle_offset(config.correction_angle_offset_deg);
          }
          if(config.time_offset != this->_time_offset){
               this->_time_offset = config.time_offset;
          }
          if(config.debug_angle_inputs != this->_debug_angle_inputs) this->_debug_angle_inputs = config.debug_angle_inputs;

          this->vb->enable_filtered_depth_denoising(config.do_post_depth_denoising);
          this->vb->set_depth_denoising_kernel_size(config.depth_denoising_kernel_size);
          this->vb->enable_obstacle_data_extraction(config.do_obstacle_data_extraction);

          this->vb->set_absolute_minimum_depth(config.depth_absolute_min);
          this->vb->set_absolute_maximum_depth(config.depth_absolute_max);

          this->vb->enable_noisy_gnd_line_filtering(config.do_noisy_gnd_line_filtering);
          this->vb->set_gnd_line_slope_error_threshold(config.noisy_gnd_line_slope_thresh);
          this->vb->set_gnd_line_intercept_error_threshold(config.noisy_gnd_line_intercept_thresh);

          this->vb->set_object_dimension_limits_x(
               (float) config.object_geometric_limit_x_min,
               (float) config.object_geometric_limit_x_max
          );
          this->vb->set_object_dimension_limits_y(
               (float) config.object_geometric_limit_y_min,
               (float) config.object_geometric_limit_y_max
          );
          this->vb->flip_object_dimension_x_limits(config.flip_geometric_limits_x);
          this->vb->flip_object_dimension_y_limits(config.flip_geometric_limits_y);
          this->vb->toggle_disparity_generation_debug_verbosity(config.debug_disparity_generation);
     }

     // Cloud Filtering
     {
          if(config.do_cloud_limit_filtering != this->_do_cloud_limit_filtering){
               this->_do_cloud_limit_filtering = config.do_cloud_limit_filtering;
          }
          if(config.max_cloud_height != this->_max_cloud_height){
               this->_max_cloud_height = config.max_cloud_height;
          }
          if(config.min_cloud_height != this->_min_cloud_height){
               this->_min_cloud_height = config.min_cloud_height;
          }
          if(config.max_cloud_range != this->_max_cloud_range){
               this->_max_cloud_range = config.max_cloud_range;
          }
          if(config.min_cloud_range != this->_min_cloud_range){
               this->_min_cloud_range = config.min_cloud_range;
          }
          if(config.do_cloud_downsampling != this->_do_cloud_downsampling){
               this->_do_cloud_downsampling = config.do_cloud_downsampling;
          }
          if(config.voxel_res_x != this->_voxel_res_x){
               this->_voxel_res_x = config.voxel_res_x;
          }
          if(config.voxel_res_y != this->_voxel_res_y){
               this->_voxel_res_y = config.voxel_res_y;
          }
          if(config.voxel_res_z != this->_voxel_res_z){
               this->_voxel_res_z = config.voxel_res_z;
          }
          if(config.do_cloud_outlier_removal != this->_do_cloud_outlier_removal){
               this->_do_cloud_outlier_removal = config.do_cloud_outlier_removal;
          }
          if(config.cloud_outlier_min_neighbors != this->_cloud_outlier_min_neighbors){
               this->_cloud_outlier_min_neighbors = config.cloud_outlier_min_neighbors;
          }
          if(config.cloud_outlier_search_radius != this->_cloud_outlier_search_radius){
               this->_cloud_outlier_search_radius = config.cloud_outlier_search_radius;
          }
     }

     // Data Publishing
     {
          if(config.update_rate != this->_update_rate){
               this->_update_rate = config.update_rate;
          }
          if(config.publish_obstacle_data != this->_publish_obs_data){
               this->_publish_obs_data = config.publish_obstacle_data;
          }
          if(config.publish_obstacle_markers != this->_publish_obstacle_markers){
               this->_publish_obstacle_markers = config.publish_obstacle_markers;
          }
          if(config.publish_raw_cloud != this->_flag_pub_raw_cloud){
               this->_flag_pub_raw_cloud = config.publish_raw_cloud;
          }
          if(config.publish_unfilter_cloud != this->_flag_pub_unfiltered_cloud){
               this->_flag_pub_unfiltered_cloud = config.publish_unfilter_cloud;
          }
          if(config.publish_filter_cloud != this->_flag_pub_filtered_cloud){
               this->_flag_pub_filtered_cloud = config.publish_filter_cloud;
          }

          if(config.publish_corrected_depth != this->_publish_corrected_depth){
               this->_publish_corrected_depth = config.publish_corrected_depth;
               this->vb->processingDebugger.enable_angle_corrected_depth_visualization(this->_publish_corrected_depth);
          }
          if(config.publish_generated_disparity != this->_publish_generated_disparity){
               this->_publish_generated_disparity = config.publish_generated_disparity;
          }
          if(config.publish_mid_level_debug_images != this->_publish_mid_level_debug_images){
               this->_publish_mid_level_debug_images = config.publish_mid_level_debug_images;
          }
          if(config.publish_low_level_debug_images != this->_publish_low_level_debug_images){
               this->_publish_low_level_debug_images = config.publish_low_level_debug_images;
          }

          if(config.publish_umap_raw != this->_publish_umap_raw){
               this->_publish_umap_raw = config.publish_umap_raw;
               this->vb->processingDebugger.enable_umap_raw_visualization(this->_publish_umap_raw);
          }
          if(config.visualize_umap_raw != this->_visualize_umap_raw){
               this->_visualize_umap_raw = config.visualize_umap_raw;
               this->vb->processingDebugger.enable_umap_raw_visualization(this->_visualize_umap_raw);
          }

          if(config.publish_vmap_raw != this->_publish_vmap_raw){
               this->_publish_vmap_raw = config.publish_vmap_raw;
               this->vb->processingDebugger.enable_vmap_raw_visualization(this->_publish_vmap_raw);
          }
          if(config.visualize_vmap_raw != this->_visualize_vmap_raw){
               this->_visualize_vmap_raw = config.visualize_vmap_raw;
               this->vb->processingDebugger.enable_vmap_raw_visualization(this->_visualize_vmap_raw);
          }

          if(config.publish_umap_processed != this->_publish_umap_processed){
               this->_publish_umap_processed = config.publish_umap_processed;
               this->vb->processingDebugger.enable_umap_processed_visualization(this->_publish_umap_processed);
          }
          if(config.visualize_umap_processed != this->_visualize_umap_final){
               this->_visualize_umap_final = config.visualize_umap_processed;
               this->vb->processingDebugger.enable_umap_processed_visualization(this->_visualize_umap_final);
          }

          if(config.publish_vmap_processed != this->_publish_vmap_processed){
               this->_publish_vmap_processed = config.publish_vmap_processed;
               this->vb->processingDebugger.enable_vmap_processed_visualization(this->_publish_vmap_processed);
          }
          if(config.visualize_vmap_processed != this->_visualize_vmap_final){
               this->_visualize_vmap_final = config.visualize_vmap_processed;
               this->vb->processingDebugger.enable_vmap_processed_visualization(this->_visualize_vmap_final);
          }

          if(config.overlay_gnd_lines != this->_overlay_gnd_lines){ this->_overlay_gnd_lines = config.overlay_gnd_lines; }
          if(config.overlay_filtered_contours != this->_overlay_filtered_contours){ this->_overlay_filtered_contours = config.overlay_filtered_contours; }
          if(config.overlay_object_search_windows != this->_overlay_object_search_windows){
               this->_overlay_object_search_windows = config.overlay_object_search_windows;
          }
          if(config.overlay_obstacle_boxes != this->_overlay_obstacle_bounding_boxes){
               this->_overlay_obstacle_bounding_boxes = config.overlay_obstacle_boxes;
          }
     }

     // Mid to Low-level Debugging Image Visualization
     {
          if(config.visualize_debug_tile_names != this->_show_uvmap_debug_titles){ this->_show_uvmap_debug_titles = config.visualize_debug_tile_names; }
          // Override flags for mid-level debugging images to false if we aren't publishing these types
          // Otherwise grab the configured flags for enabling/disabling the viewing of individual images
          if(!this->_publish_mid_level_debug_images){
               this->_visualize_gnd_line_keep_mask = false;
               this->_visualize_obj_candidate_keep_mask = false;
               this->_visualize_umap_keep_mask = false;
               this->_visualize_vmap_keep_mask = false;
          } else{
               if(config.visualize_gnd_line_keep_mask != this->_visualize_gnd_line_keep_mask){
                    this->_visualize_gnd_line_keep_mask = config.visualize_gnd_line_keep_mask;
               }
               if(config.visualize_obj_candidate_keep_mask != this->_visualize_obj_candidate_keep_mask){
                    this->_visualize_obj_candidate_keep_mask = config.visualize_obj_candidate_keep_mask;
               }
               if(config.visualize_umap_keep_mask != this->_visualize_umap_keep_mask){
                    this->_visualize_umap_keep_mask = config.visualize_umap_keep_mask;
               }
               if(config.visualize_vmap_keep_mask != this->_visualize_vmap_keep_mask){
                    this->_visualize_vmap_keep_mask = config.visualize_vmap_keep_mask;
               }
          }
          // Signal the processing debugger class which objects to populate
          this->vb->processingDebugger.enable_gnd_line_keep_mask_visualization(this->_visualize_gnd_line_keep_mask);
          this->vb->processingDebugger.enable_obj_candidate_keep_mask_visualization(this->_visualize_obj_candidate_keep_mask);
          this->vb->processingDebugger.enable_umap_keep_mask_visualization(this->_visualize_umap_keep_mask);
          this->vb->processingDebugger.enable_vmap_post_keep_mask_visualization(this->_visualize_vmap_keep_mask);

          // Override flags for low-level debugging images to false if we aren't publishing these types
          // Otherwise grab the configured flags for enabling/disabling the viewing of individual images
          if(!this->_publish_low_level_debug_images){
               this->_visualize_umap_sobel_raw = false;
               this->_visualize_umap_sobel_preprocessed = false;
               this->_visualize_umap_sobel_dilated = false;
               this->_visualize_umap_sobel_blurred = false;
               this->_visualize_vmap_sobelized_preprocessed = false;
               this->_visualize_vmap_sobelized_postprocessed_threshed = false;
               this->_visualize_vmap_sobelized_postprocessed_blurred = false;
          } else{
               if(config.visualize_umap_sobel_raw != this->_visualize_umap_sobel_raw){
                    this->_visualize_umap_sobel_raw = config.visualize_umap_sobel_raw;
               }
               if(config.visualize_umap_sobel_preprocessed != this->_visualize_umap_sobel_preprocessed){
                    this->_visualize_umap_sobel_preprocessed = config.visualize_umap_sobel_preprocessed;
               }
               if(config.visualize_umap_sobel_dilated != this->_visualize_umap_sobel_dilated){
                    this->_visualize_umap_sobel_dilated = config.visualize_umap_sobel_dilated;
               }
               if(config.visualize_umap_sobel_blurred != this->_visualize_umap_sobel_blurred){
                    this->_visualize_umap_sobel_blurred = config.visualize_umap_sobel_blurred;
               }
               if(config.visualize_vmap_sobelized_preprocessed != this->_visualize_vmap_sobelized_preprocessed){
                    this->_visualize_vmap_sobelized_preprocessed = config.visualize_vmap_sobelized_preprocessed;
               }
               if(config.visualize_vmap_sobelized_postprocessed_threshed != this->_visualize_vmap_sobelized_postprocessed_threshed){
                    this->_visualize_vmap_sobelized_postprocessed_threshed = config.visualize_vmap_sobelized_postprocessed_threshed;
               }
               if(config.visualize_vmap_sobelized_postprocessed_blurred != this->_visualize_vmap_sobelized_postprocessed_blurred){
                    this->_visualize_vmap_sobelized_postprocessed_blurred = config.visualize_vmap_sobelized_postprocessed_blurred;
               }
          }
          // Signal the processing debugger class which objects to populate
          this->vb->processingDebugger.enable_umap_sobel_raw_visualization(this->_visualize_umap_sobel_raw);
          this->vb->processingDebugger.enable_umap_sobel_preprocessed_visualization(this->_visualize_umap_sobel_preprocessed);
          this->vb->processingDebugger.enable_umap_sobel_dilated_visualization(this->_visualize_umap_sobel_dilated);
          this->vb->processingDebugger.enable_umap_sobel_blurred_visualization(this->_visualize_umap_sobel_blurred);
          this->vb->processingDebugger.enable_vmap_sobelized_preprocessed_visualization(this->_visualize_vmap_sobelized_preprocessed);
          this->vb->processingDebugger.enable_vmap_post_sobel_threshed_visualization(this->_visualize_vmap_sobelized_postprocessed_threshed);
          this->vb->processingDebugger.enable_vmap_post_sobel_blurred_visualization(this->_visualize_vmap_sobelized_postprocessed_blurred);
     }
}
void VboatsRos::depthCallback(const sensor_msgs::Image::ConstPtr& msg, const int value){
     std::lock_guard<std::mutex> lock(_lock);
     cv::Mat image = cv_bridge::toCvCopy(msg)->image;
     this->_depth = image;
}
void VboatsRos::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg, const int value){
     std::lock_guard<std::mutex> lock(_lock);
     this->_filtered_depth_info = msg;
     float Tx = msg->P[3];
     float baseline = -Tx / msg->K[0];
     float depth_scale = msg->D[0];
     if(depth_scale == 0.0) depth_scale = 1.0;
     this->vb->set_camera_info(msg->K[0], msg->K[4], msg->K[2], msg->K[5], depth_scale, baseline, true);
}
void VboatsRos::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
     std::lock_guard<std::mutex> lock(_lock);
     this->vb->set_camera_orientation(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w, this->_debug_angle_inputs);
     if(this->_do_angle_offsets_calibration){
          if(this->_angle_offsets_count < this->_angle_offset_samples){
               std::vector<double> curAngles = this->vb->get_camera_angles();
               this->_avg_roll    += curAngles[0];
               this->_avg_pitch   += curAngles[1];
               this->_avg_yaw     += curAngles[2];
               this->_angle_offsets_count++;
          } else{
               double avg_offset;
               ImageAngleCorrectionType corType = this->vb->get_angle_correction_type();
               if(corType == ROLL_CORRECTION) avg_offset = this->_avg_roll;
               else if(corType == PITCH_CORRECTION) avg_offset = this->_avg_pitch;
               else if(corType == YAW_CORRECTION) avg_offset = this->_avg_yaw;

               avg_offset = avg_offset / (double) this->_angle_offsets_count;
               avg_offset = -avg_offset*M_RAD2DEG;
               ROS_INFO("Camera Correction Angle Calibration Complete. Setting Correction Angle to %.2lf deg.", avg_offset);
               this->vb->set_camera_angle_offset(avg_offset);

               this->_avg_roll    = 0.0;
               this->_avg_pitch   = 0.0;
               this->_avg_yaw     = 0.0;
               this->_angle_offsets_count = 0;
               this->_do_angle_offsets_calibration = false;
               this->_angle_offsets_calibration_performed = true;
          }
     }
}
void VboatsRos::poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
     std::lock_guard<std::mutex> lock(_lock);
     this->vb->set_camera_orientation(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w, this->_debug_angle_inputs);
}
bool VboatsRos::pause_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
     this->_is_node_paused = true;
     ROS_INFO("[INFO] VboatsRos::VboatsRos() ---- Paused.");
}
bool VboatsRos::resume_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
     this->_is_node_paused = false;
     ROS_INFO("[INFO] VboatsRos::VboatsRos() ---- Resuming.");
}
bool VboatsRos::calibrate_orientation_offsets_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
     this->_do_angle_offsets_calibration = true;
     this->_avg_roll    = 0.0;
     this->_avg_pitch   = 0.0;
     this->_avg_yaw     = 0.0;
     this->_angle_offsets_count = 0;
     return true;
}
/** -------------------------------------------------------------------------------------------------
*                                     ROS Interface Helper Functions
* ------------------------------------------------------------------------------------------------- */
template<typename ROS_OBJ> void VboatsRos::_publish_image(ROS_OBJ publisher, const cv::Mat& image, bool colorize){
     if(!image.empty()){
          cv::Mat data;
          if(colorize) data = imCvtCmap(image);
          else data = image;

          std_msgs::Header tmpHeader;
          tmpHeader.stamp = ros::Time::now();
          tmpHeader.seq = this->_count;
          tmpHeader.frame_id = this->_camera_tf;

          sensor_msgs::ImagePtr tmpImgMsg;
          if(data.type() == CV_8UC1) tmpImgMsg = cv_bridge::CvImage(tmpHeader, "8UC1", data).toImageMsg();
          else if(data.type() == CV_8UC3) tmpImgMsg = cv_bridge::CvImage(tmpHeader, "bgr8", data).toImageMsg();
          else if(data.type() == CV_16UC1) tmpImgMsg = cv_bridge::CvImage(tmpHeader, "16UC1", data).toImageMsg();
          else if(data.type() == CV_16UC3) tmpImgMsg = cv_bridge::CvImage(tmpHeader, "16UC3", data).toImageMsg();
          else if(data.type() == CV_32FC1) tmpImgMsg = cv_bridge::CvImage(tmpHeader, "32FC1", data).toImageMsg();
          else{
               ROS_DEBUG("VboatsRos::_publish_image() --- Image type is one not currently handled, converting to CV_8U type.");
               cvinfo(data, "_publish_image() --- Input Image: ");
               cv::Mat output;
               cv::Mat tmpCopy = data.clone();
               cv::cvtColor(tmpCopy, tmpCopy, cv::COLOR_BGR2GRAY);
               double minVal, maxVal;
               cv::minMaxLoc(tmpCopy, &minVal, &maxVal);
               if(tmpCopy.channels() > 1){
                    tmpCopy.convertTo(output, CV_8UC3, (255.0/maxVal) );
                    tmpImgMsg = cv_bridge::CvImage(tmpHeader, "bgr8", output).toImageMsg();
               } else{
                    tmpCopy.convertTo(output, CV_8UC1, (255.0/maxVal) );
                    tmpImgMsg = cv_bridge::CvImage(tmpHeader, "8UC1", output).toImageMsg();
               }
          }
          publisher.publish(tmpImgMsg);
     } else{
          ROS_DEBUG("VboatsRos::_publish_image() --- Image is empty, not publishing.");
          cvinfo(image, "_publish_image() --- Input Image: ");
     }
}
template<typename ROS_OBJ> void VboatsRos::_publish_image(ROS_OBJ publisher,
     const cv::Mat& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info, bool colorize)
{
     if(!image.empty()){
          cv::Mat data;
          if(colorize) data = imCvtCmap(image);
          else data = image;

          std_msgs::Header tmpHeader;
          tmpHeader.stamp = ros::Time::now();
          tmpHeader.seq = this->_count;
          tmpHeader.frame_id = this->_camera_tf;

          sensor_msgs::ImagePtr tmpImgMsg;
          if(data.type() == CV_8UC1) tmpImgMsg = cv_bridge::CvImage(tmpHeader, "8UC1", data).toImageMsg();
          else if(data.type() == CV_8UC3) tmpImgMsg = cv_bridge::CvImage(tmpHeader, "bgr8", data).toImageMsg();
          else if(data.type() == CV_16UC1) tmpImgMsg = cv_bridge::CvImage(tmpHeader, "16UC1", data).toImageMsg();
          else if(data.type() == CV_16UC3) tmpImgMsg = cv_bridge::CvImage(tmpHeader, "16UC3", data).toImageMsg();
          else if(data.type() == CV_32FC1) tmpImgMsg = cv_bridge::CvImage(tmpHeader, "32FC1", data).toImageMsg();
          else{
               ROS_DEBUG("VboatsRos::_publish_image() --- Image type is one not currently handled, converting to CV_8U type.");
               cvinfo(data, "_publish_image() --- Input Image: ");
               cv::Mat output;
               cv::Mat tmpCopy = data.clone();
               cv::cvtColor(tmpCopy, tmpCopy, cv::COLOR_BGR2GRAY);
               double minVal, maxVal;
               cv::minMaxLoc(tmpCopy, &minVal, &maxVal);
               if(tmpCopy.channels() > 1){
                    tmpCopy.convertTo(output, CV_8UC3, (255.0/maxVal) );
                    tmpImgMsg = cv_bridge::CvImage(tmpHeader, "bgr8", output).toImageMsg();
               } else{
                    tmpCopy.convertTo(output, CV_8UC1, (255.0/maxVal) );
                    tmpImgMsg = cv_bridge::CvImage(tmpHeader, "8UC1", output).toImageMsg();
               }
          }
          // boost::shared_ptr< ::sensor_msgs::CameraInfo const> tmpCamInfo(&this->_filtered_depth_info);
          publisher.publish(tmpImgMsg, this->_filtered_depth_info);
     } else{
          ROS_DEBUG("VboatsRos::_publish_image() --- Image is empty, not publishing.");
          cvinfo(image, "_publish_image() --- Input Image: ");
     }
}
void VboatsRos::_publish_extracted_obstacle_data(ros::Publisher publisher, std::vector<Obstacle> obstacles){
     if( (!obstacles.empty()) && (obstacles.size() > 0) ){
          int n = 0;
          swanson_msgs::VboatsObstacles obsMsg;
          obsMsg.header.seq = this->_count;
          obsMsg.header.stamp = ros::Time::now();
          obsMsg.header.frame_id = this->_camera_tf;
          for(Obstacle ob : obstacles){
               swanson_msgs::VboatsObstacle tmpOb;
               tmpOb.header.seq = n;
               tmpOb.header.stamp = obsMsg.header.stamp;
               tmpOb.header.frame_id = obsMsg.header.frame_id;
               tmpOb.distance = ob._distance;
               tmpOb.angle = ob._angle;
               tmpOb.position.x = ob._location.x;
               tmpOb.position.y = ob._location.y;
               tmpOb.position.z = ob._location.z;
               obsMsg.obstacles.push_back(tmpOb);
               n++;
          }
          publisher.publish(obsMsg);
     }
}
void VboatsRos::_publish_pointcloud(ros::Publisher publisher, cloudxyz_t::Ptr inputCloud){
     if(inputCloud->points.size() != 0){
          ros::Duration smallDt(this->_time_offset);
          sensor_msgs::PointCloud2 tmpCloudMsg;
          pcl::toROSMsg(*inputCloud, tmpCloudMsg);
          tmpCloudMsg.header.stamp = ros::Time::now()+smallDt;
          tmpCloudMsg.header.seq = this->_count;
          tmpCloudMsg.header.frame_id = this->_camera_tf;
          publisher.publish(tmpCloudMsg);
     } else{ ROS_DEBUG("VboatsRos::_publish_pointcloud() --- Input pointcloud has no points, not publishing."); }
}
void VboatsRos::publish_pointclouds(cv::Mat raw_depth, cv::Mat filtered_depth){
     cv::Mat testMat, inMat;
     double minVal, maxVal;
     cv::minMaxLoc(filtered_depth, &minVal, &maxVal);
     filtered_depth.convertTo(inMat, CV_8UC1, (255.0/maxVal) );
     threshold(inMat, inMat, 1, 255, cv::THRESH_BINARY);
     raw_depth.copyTo(testMat, inMat);
     // imshowCmap(testMat, "Filtered Depth Image 4 PCL conversion");
     // cv::waitKey(1);
     if(!testMat.empty()){
          // Publish raw pointcloud extrapolated for uncut depth map
          if(this->_flag_pub_unfiltered_cloud || this->_flag_pub_filtered_cloud){
               cloudxyz_t::Ptr obsFilteredCloud = this->vb->generate_pointcloud_from_depth(testMat);
               if(obsFilteredCloud->points.size() != 0){
                    if(this->_flag_pub_unfiltered_cloud) this->_publish_pointcloud(this->_unfiltered_cloud_pub, obsFilteredCloud);
                    // if(unfiltered_cloud) pcl::copyPointCloud(*unfiltCloud, *unfiltered_cloud);
                    if(this->_flag_pub_filtered_cloud){
                         cloudxyz_t::Ptr filtCloud = this->filter_pointcloud(obsFilteredCloud);
                         if(filtCloud->points.size() <= 0) this->_publish_pointcloud(this->_filtered_cloud_pub, obsFilteredCloud);
                         else this->_publish_pointcloud(this->_filtered_cloud_pub, filtCloud);
                    }
               }
          }
     }
     if(this->_flag_pub_raw_cloud){
          if(!raw_depth.empty()){
               cloudxyz_t::Ptr rawCloud = this->vb->generate_pointcloud_from_depth(raw_depth);
               this->_publish_pointcloud(this->_raw_cloud_pub, rawCloud);
          }
     }
}

void VboatsRos::publish_auxillery_images(const cv::Mat& disparity_gen, const cv::Mat& umap_proc, const cv::Mat& vmap_proc){
     if(this->_publish_corrected_depth){ this->_publish_image<ros::Publisher>(this->_corrected_depth_pub, this->vb->processingDebugger.angle_corrected_depth_img, true); }
     if(this->_publish_generated_disparity){ this->_publish_image<ros::Publisher>(this->_generated_disparity_pub, disparity_gen, true); }
     if(this->_publish_umap_raw){ this->_publish_image<ros::Publisher>(this->_raw_umap_pub, this->vb->processingDebugger.umap_raw, true); }
     if(this->_publish_vmap_raw){ this->_publish_image<ros::Publisher>(this->_raw_vmap_pub, this->vb->processingDebugger.vmap_raw, true); }
     if(this->_publish_umap_processed){ this->_publish_image<ros::Publisher>(this->_proc_umap_pub, umap_proc, true); }
     if(this->_publish_vmap_processed){ this->_publish_image<ros::Publisher>(this->_proc_vmap_pub, vmap_proc, true); }

     if(this->_publish_mid_level_debug_images){
          this->_publish_image<ros::Publisher>(this->_gnd_line_mask_pub, this->vb->processingDebugger.gnd_line_filtering_keep_mask);
          this->_publish_image<ros::Publisher>(this->_gnd_line_vmask_pub, this->vb->processingDebugger.gnd_line_filtering_keep_mask_vmap);
          this->_publish_image<ros::Publisher>(this->_obj_candidate_mask_pub, this->vb->processingDebugger.obj_candidate_filtering_keep_mask);
          this->_publish_image<ros::Publisher>(this->_umap_keep_mask_pub, this->vb->processingDebugger.umap_keep_mask);
          this->_publish_image<ros::Publisher>(this->_vmap_keep_mask_pub, this->vb->processingDebugger.vmap_postproc_keep_mask);
     }
}
void VboatsRos::publish_debugging_images(){
     cv::Mat umapDebugImg = this->vb->processingDebugger.construct_low_level_umap_image(this->_overlay_filtered_contours, this->_show_uvmap_debug_titles);
     cv::Mat vmapDebugImg = this->vb->processingDebugger.construct_low_level_vmap_image(this->_overlay_gnd_lines, this->_overlay_object_search_windows, this->_show_uvmap_debug_titles);
     this->_publish_image(this->_umap_debug_pub, umapDebugImg);
     this->_publish_image(this->_vmap_debug_pub, vmapDebugImg);
}
void VboatsRos::visualize_obstacle_markers(const std::vector<Obstacle>& obstacles){
     if( (obstacles.empty()) || (obstacles.size() <= 0) ) return;
     std_msgs::ColorRGBA blue;   blue.r = 0; blue.g = 0; blue.b = 1.0; blue.a = 1.0;
     std_msgs::ColorRGBA red;    red.r = 1.0; red.g = 0; red.b = 0; red.a = 1.0;
     std_msgs::ColorRGBA green;  green.r = 0; green.g = 1.0; green.b = 0; green.a = 1.0;
     std_msgs::ColorRGBA purple; purple.r = 1.0; purple.g = 0; purple.b = 1.0; purple.a = 1.0;

     ROS_DEBUG("visualising %lu obstacles", obstacles.size());
     visualization_msgs::MarkerArray markers_msg;
     std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
     visualization_msgs::Marker m;
     m.header.stamp = ros::Time::now();
     m.header.frame_id = this->_camera_tf;
     m.ns = "obstacles";

     double scale = 0.1;
     m.scale.x = scale; m.scale.y = scale; m.scale.z = scale;
     m.color.r = 0; m.color.g = 0; m.color.b = 255; m.color.a = 255;
     m.lifetime = ros::Duration(0); // lives forever
     m.frame_locked = true;
     m.action = visualization_msgs::Marker::ADD;

     size_t id = 0;
     int obsIdx = 0;
     geometry_msgs::Quaternion q; q.w = 1.0; q.x = 0.0; q.y = 0.0; q.z = 0.0;
     for(Obstacle obstacle : obstacles){
          m.type = visualization_msgs::Marker::SPHERE;
          m.id = int(id);
          m.points = {};
          m.color = red;
          m.scale.x = scale; m.scale.y = scale; m.scale.z = scale;

          cv::Point3f loc = obstacle.get_location();
          ROS_DEBUG(" --- obstacle %d = %s", obsIdx, obstacle.toString().c_str());
          m.pose.position.x = loc.x;
          m.pose.position.y = loc.y;
          m.pose.position.z = loc.z;
          m.pose.orientation = q;

          markers.push_back(m);
          ++id;
          obsIdx++;
     }
     // delete previous markers, which are now unused
     int current_markers_count = (int) markers.size();
     m.action = visualization_msgs::Marker::DELETE;
     for(; id < this->_last_markers_count; ++id){ m.id = int(id); markers.push_back(m); }
     this->_last_markers_count = current_markers_count;
     this->_obstacle_markers_pub.publish(markers_msg);
}

/** -------------------------------------------------------------------------------------------------
*                                       Data Generation Functions
* ------------------------------------------------------------------------------------------------- */
cloudxyz_t::Ptr VboatsRos::filter_pointcloud(cloudxyz_t::Ptr inputCloud, bool debug_timing){
     cloudxyz_t::Ptr filtered_cloud(new cloudxyz_t);
     if(inputCloud->points.size() == 0) return filtered_cloud;
     try{
          double t;
          if(debug_timing) t = (double)cv::getTickCount();

          if(this->_do_cloud_limit_filtering){
               pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ> ());
               range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -this->_max_cloud_height)));
               range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, this->_min_cloud_height)));
               range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, this->_min_cloud_range)));
               range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, this->_max_cloud_range)));

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

          if(debug_timing){
               t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
               ROS_INFO("generate_pointcloud_from_depth() ---- took %.4lf ms (%.2lf Hz) to filter a pointcloud.", t*1000.0, (1.0/t));
          }
     } catch(std::exception e){
          ROS_ERROR("filter_pointcloud() --- %s", e.what());
     }

     return filtered_cloud;
}

/** -------------------------------------------------------------------------------------------------
*                                       Runtime Functions
* ------------------------------------------------------------------------------------------------- */
int VboatsRos::update(){
     cv::Mat curDepth;
     this->_lock.lock();
     curDepth = this->_depth;
     this->_lock.unlock();

     vector<Obstacle> obs;
     vector<float> gnd_line_coefficients;
     cv::Mat inUmap, inVmap;
     cv::Mat procUmap, procVmap;
     cv::Mat filtered_depth, genDisparity;
     int nObs;
     if(this->_try_cuda){
          nObs = this->vb->process_w_cuda(curDepth, &filtered_depth, &obs, &gnd_line_coefficients, &genDisparity, &procUmap, &procVmap, nullptr, nullptr, this->_verbose_obstacles);
          // ROS_INFO("CUDA OUTPUT = %d", nObs);
     } else{
          nObs = this->vb->process(curDepth, &filtered_depth, &obs, &gnd_line_coefficients, &genDisparity, &procUmap, &procVmap, nullptr, nullptr, this->_verbose_obstacles);
     }

     // Misc Printouts
     if(!gnd_line_coefficients.empty()){
          float cur_gnd_line_slope      = (float) gnd_line_coefficients[0];
          int cur_gnd_line_intercept    = (int) gnd_line_coefficients[1];
          float delta_slope             = cur_gnd_line_slope - this->_prev_gnd_line_slope;
          int delta_intercept           = cur_gnd_line_intercept - this->_prev_gnd_line_intercept;
          this->_prev_gnd_line_slope     = cur_gnd_line_slope;
          this->_prev_gnd_line_intercept = cur_gnd_line_intercept;
          if(this->_debug_ground_line_params){
               ROS_INFO("Estimated Ground Line Coefficients (slope, intercept) = %.4f, %d &&&& Delta = %.4f, %d",
                    cur_gnd_line_slope, cur_gnd_line_intercept, delta_slope, delta_intercept
               );
          }
     }

     // Data Publishing
     if(this->_publish_obs_data){ this->_publish_extracted_obstacle_data(this->_detected_obstacle_info_pub, obs); }
     if(this->_overlay_obstacle_bounding_boxes){
          cv::Mat obs_display = this->vb->processingDebugger.overlay_obstacle_bounding_boxes(filtered_depth, obs);
          this->_publish_image<image_transport::CameraPublisher>(this->_filtered_depth_pub, obs_display, this->_filtered_depth_info);
     } else{this->_publish_image<image_transport::CameraPublisher>(this->_filtered_depth_pub, filtered_depth, this->_filtered_depth_info); }
     this->publish_auxillery_images(genDisparity, procUmap, procVmap);
     if(this->_publish_low_level_debug_images){ this->publish_debugging_images(); }
     this->publish_pointclouds(curDepth, filtered_depth);
     if(this->_publish_obstacle_markers) this->visualize_obstacle_markers(obs);

     this->_count++;
     return nObs;
}

int VboatsRos::run(){
     double t;
     if(this->_verbose_update) t = (double)cv::getTickCount();
     while(ros::ok()){
          ros::Rate rate( (int) this->_update_rate );
          if(!this->_is_node_paused){
               int nObjects = this->update();
               if(this->_verbose_update){
                    double now = (double)cv::getTickCount();
                    double dt = (now - t)/cv::getTickFrequency();
                    ROS_INFO("[INFO] VboatsRos::update() --- Found %d obstacles in %.4lf ms (%.2lf Hz).", nObjects, dt*1000.0, (1.0/dt));
                    t = now;
               }
          }
          ros::spinOnce();
          rate.sleep();
     }
     return 0;
}

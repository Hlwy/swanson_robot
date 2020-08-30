#include <iostream>
#include "swanson_algorithms/vboats_ros.h"

#include <RoboCommander/utilities/utils.h>
#include <RoboCommander/utilities/image_utils.h>
#include <RoboCommander/algorithms/vboats/uvmap_utils.h>
#include <swanson_msgs/VboatsObstacle.h>
#include <swanson_msgs/VboatsObstacles.h>

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
     _use_gnd_meth = true;
     _recvd_image = false;
     _recvd_cam_info = false;
     _flag_depth_based = true;

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
     bool publish_cloud = false;
     bool publish_fitered_cloud = false;
     int update_rate = 30;

     p_nh.getParam("namespace",namespaced);
     p_nh.getParam("verbose_obstacles",flag_verbose_obstacles);
     p_nh.getParam("verbose_timings",flag_verbose_timings);
     p_nh.getParam("publish_images",flag_publish_imgs);
     p_nh.getParam("publish_aux_images",publish_aux_images);
     p_nh.getParam("show_images",visualize_imgs);
     p_nh.getParam("update_rate",update_rate);
     p_nh.getParam("use_disparity",disparity_based);
     p_nh.getParam("gnd_filter_meth",gnd_method);

     p_nh.getParam("detect_obstacles",flag_detect_obstacles);
     p_nh.getParam("filter_ground",flag_filter_ground);
     p_nh.getParam("filtered_cloud",flag_filter_cloud);
     p_nh.getParam("publish_cloud",publish_cloud);
     p_nh.getParam("publish_filtered_cloud",publish_fitered_cloud);

     this->_ns = namespaced;
     this->_verbose_obstacles = flag_verbose_obstacles;
     this->_verbose_timings = flag_verbose_timings;
     this->_publish_images = flag_publish_imgs;
     this->_publish_aux_images = publish_aux_images;
     this->_visualize_images = visualize_imgs;
     this->_use_gnd_meth = gnd_method;
     this->_flag_depth_based = !disparity_based;
     this->_detect_obstacles = flag_detect_obstacles;
     this->_filter_ground = flag_filter_ground;
     this->_filter_cloud = flag_filter_cloud;
     this->_flag_pub_cloud = publish_cloud;
     this->_flag_pub_filtered_cloud = publish_fitered_cloud;

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
     p_nh.getParam("sor_min_neighbors",sor_mink);
     p_nh.getParam("sor_dist_thresh",sor_thresh);

     this->_max_obstacle_height = max_obs_height;
     this->_min_obstacle_height = min_obs_height;
     this->_max_obstacle_range = max_obs_range;
     this->_min_obstacle_range = min_obs_range;
     this->_voxel_res_x = voxel_resx;
     this->_voxel_res_y = voxel_resy;
     this->_voxel_res_z = voxel_resz;
     this->_gnd_upper_offset = gnd_buffer_upper_offset;
     this->_gnd_lower_offset = gnd_buffer_lower_offset;
     this->_sor_min_neighbors = sor_mink;
     this->_sor_dist_thresh = sor_thresh;
     this->_contourFiltMeth = 1;
     this->_contourFiltMinThresh = 50.0;
     this->_uThreshs = {0.3,0.295,0.275,0.3};
     this->_vThreshs = {0.3, 0.3,0.25,0.4};
     this->_use_custom_umap_filtering = false;
     this->_custom_uThresh_perc = 0.25;
     this->_vmap_sobel_thresh = 30;
     this->_dilate_sobel_to_segmask = false;
     this->_sobel_dilate_ksize = 3;

     /** ROS Object Configuration */
     std::string depth_image_topic = this->_ns + "/camera/depth/image_rect_raw";
     std::string camera_info_topic = this->_ns + "/camera/depth/camera_info";
     std::string disparity_image_topic = this->_ns + "/camera/disparity/image_raw";
     std::string umap_topic = "/vboats/umap/image_raw";
     std::string vmap_topic = "/vboats/vmap/image_raw";
     std::string filtered_image_topic = "/vboats/depth/image_filtered";
     std::string obstacles_image_topic = "/vboats/obstacles/image_raw";
     std::string detected_obstacles_info_topic = "/vboats/obstacles/data";
     std::string raw_cloud_topic = "/vboats/cloud/raw";
     std::string filtered_cloud_topic = "/vboats/cloud/filtered";
     p_nh.getParam("depth_image_topic",depth_image_topic);
     p_nh.getParam("camera_info_topic",camera_info_topic);
     p_nh.getParam("disparity_image_topic",disparity_image_topic);
     p_nh.getParam("umap_topic",umap_topic);
     p_nh.getParam("vmap_topic",vmap_topic);
     p_nh.getParam("filtered_image_topic",filtered_image_topic);
     p_nh.getParam("obstacles_image_topic",obstacles_image_topic);
     p_nh.getParam("obstacles_info_topic",detected_obstacles_info_topic);
     p_nh.getParam("raw_cloud_topic",raw_cloud_topic);
     p_nh.getParam("filtered_cloud_topic",filtered_cloud_topic);

     /** Initialize ROS-Objects */
     this->_cam_info_sub = m_nh.subscribe<sensor_msgs::CameraInfo>(camera_info_topic, 1, boost::bind(&VboatsRos::infoCallback,this,_1,1));
     if(this->_flag_depth_based) this->_depth_sub = m_nh.subscribe<sensor_msgs::Image>(depth_image_topic, 30, boost::bind(&VboatsRos::depthCallback,this,_1,5));
     else this->_disparity_sub = m_nh.subscribe<sensor_msgs::Image>(disparity_image_topic, 30, boost::bind(&VboatsRos::disparityCallback,this,_1,5));
     this->_new_img_pub = m_nh.advertise<sensor_msgs::Image>(filtered_image_topic, 1);
     this->_obstacles_img_pub = m_nh.advertise<sensor_msgs::Image>(obstacles_image_topic, 1);
     this->_detected_obstacle_info_pub = m_nh.advertise<swanson_msgs::VboatsObstacles>(detected_obstacles_info_topic, 100);

     this->_umap_pub = m_nh.advertise<sensor_msgs::Image>(umap_topic, 1);
     this->_vmap_pub = m_nh.advertise<sensor_msgs::Image>(vmap_topic, 1);
     this->_cloud_pub = m_nh.advertise<cloudxyz_t>(raw_cloud_topic, 1);
     this->_filtered_cloud_pub = m_nh.advertise<cloudxyz_t>(filtered_cloud_topic, 1);
     // if(this->_flag_pub_cloud) this->_cloud_pub = m_nh.advertise<cloudxyz_t>(raw_cloud_topic, 1);
     // if(this->_flag_pub_filtered_cloud) this->_filtered_cloud_pub = m_nh.advertise<cloudxyz_t>(filtered_cloud_topic, 1);
     // if(this->_publish_aux_images){
     //      this->_umap_pub = m_nh.advertise<sensor_msgs::Image>(umap_topic, 1);
     //      this->_vmap_pub = m_nh.advertise<sensor_msgs::Image>(vmap_topic, 1);
     // }

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
void VboatsRos::cfgCallback(swanson_algorithms::VboatsConfig &config, uint32_t level){
     std::lock_guard<std::mutex> lock(_lock);
     /** Update Variables only if they are different from current settings */
     if(config.gnd_seg_method != this->_use_gnd_meth){
          this->_use_gnd_meth = config.gnd_seg_method;
     }
     if(config.filter_gnd != this->_filter_ground){
          this->_filter_ground = config.filter_gnd;
     }
     if(config.visualize_debug_imgs != this->_visualize_images){
          this->_visualize_images = config.visualize_debug_imgs;
     }
     if(config.contourFiltMeth != this->_contourFiltMeth){
          this->_contourFiltMeth = config.contourFiltMeth;
     }
     if(config.contourFiltMinThresh != this->_contourFiltMinThresh){
          this->_contourFiltMinThresh = config.contourFiltMinThresh;
     }
     if(config.max_obstacle_height != this->_max_obstacle_height){
          this->_max_obstacle_height = config.max_obstacle_height;
     }
     if(config.min_obstacle_height != this->_min_obstacle_height){
          this->_min_obstacle_height = config.min_obstacle_height;
     }
     if(config.max_obstacle_range != this->_max_obstacle_range){
          this->_max_obstacle_range = config.max_obstacle_range;
     }
     if(config.min_obstacle_range != this->_min_obstacle_range){
          this->_min_obstacle_range = config.min_obstacle_range;
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
     if(config.gnd_upper_offset != this->_gnd_upper_offset){
          this->_gnd_upper_offset = config.gnd_upper_offset;
     }
     if(config.gnd_lower_offset != this->_gnd_lower_offset){
          this->_gnd_lower_offset = config.gnd_lower_offset;
     }
     if(config.sor_min_neighbors != this->_sor_min_neighbors){
          this->_sor_min_neighbors = config.sor_min_neighbors;
     }
     if(config.sor_dist_thresh != this->_sor_dist_thresh){
          this->_sor_dist_thresh = config.sor_dist_thresh;
     }

     if(config.verbose != this->_verbose){ this->_verbose = config.verbose; }
     if(config.debug != this->_debug){ this->_debug = config.debug; }
     if(config.publish_aux_imgs != this->_publish_aux_images){ this->_publish_aux_images = config.publish_aux_imgs; }
     if(config.publish_raw_cloud != this->_flag_pub_cloud){ this->_flag_pub_cloud = config.publish_raw_cloud; }
     if(config.publish_filter_cloud != this->_flag_pub_filtered_cloud){ this->_flag_pub_filtered_cloud = config.publish_filter_cloud; }

     if(config.vmask_subtract_sobel != this->_sobel_vmask_subtract){
          this->_sobel_vmask_subtract = config.vmask_subtract_sobel;
     }
     if(config.use_custom_umap_filtering != this->_use_custom_umap_filtering){
          this->_use_custom_umap_filtering = config.use_custom_umap_filtering;
     }
     if(config.umap_custom_thresh_perc != this->_custom_uThresh_perc){
          this->_custom_uThresh_perc = config.umap_custom_thresh_perc;
     }
     if(config.vmap_sobel_thresh != this->_vmap_sobel_thresh){
          this->_vmap_sobel_thresh = config.vmap_sobel_thresh;
     }
     if(config.vmask_dilate_sobel != this->_dilate_sobel_to_segmask){
          this->_dilate_sobel_to_segmask = config.vmask_dilate_sobel;
     }
     if(config.vmask_sobel_dilate_sz != this->_sobel_dilate_ksize){
          this->_sobel_dilate_ksize = config.vmask_sobel_dilate_sz;
     }

     vector<float> uthreshs = extractFloatStringList(config.uThreshsStr, ",");
     vector<float> vthreshs = extractFloatStringList(config.vThreshsStr, ",");
     this->_uThreshs.assign(uthreshs.begin(), uthreshs.end());
     this->_vThreshs.assign(vthreshs.begin(), vthreshs.end());

     if(this->_verbose){
          ROS_INFO("Reconfigure Request:");
          ROS_INFO(" -- Using Ground Line-based object segmentation = %s", this->_use_gnd_meth?"True":"False");
          ROS_INFO(" -- Contour Filter Method = %d", this->_contourFiltMeth);
          ROS_INFO(" -- Contour Min Thresh = %.3f", this->_contourFiltMinThresh);
          ROS_INFO(" -- Cloud Height Limits = %.4f, %.4f", this->_max_obstacle_height, this->_min_obstacle_height);
          ROS_INFO(" -- Cloud Range Limits = %.4f, %.4f", this->_min_obstacle_height, this->_max_obstacle_range);
          ROS_INFO(" -- Cloud Voxel Resolution (X,Y,Z) = %.4f, %.4f, %.4f", this->_voxel_res_x, this->_voxel_res_y, this->_voxel_res_z);
          ROS_INFO(" -- Ground Line Offsets (upper, lower) = %d, %d", this->_gnd_upper_offset, this->_gnd_lower_offset);
          ROS_INFO(" -- Cloud Filtering-> Keeping points having at least %d neighbors w/in %.4f meters", this->_sor_min_neighbors, this->_sor_dist_thresh);
          ROS_INFO(" -- Umap Thresholds = [%s]", vector_str(this->_uThreshs, ", ").c_str() );
          ROS_INFO(" -- Vmap Thresholds = [%s]", vector_str(this->_vThreshs, ", ").c_str());
     }
}
void VboatsRos::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg, const int value){
     if(this->_info_count < value){
          this->_fx = msg->K[0];
          this->_fy = msg->K[4];
          this->_px = msg->K[2];
          this->_py = msg->K[5];
          this->_dscale = msg->D[0];
          float Tx = msg->P[3];
          this->_baseline = -Tx / this->_fx;
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

/** SECTION:
     ROS Publisher Helper Functions
*/
void VboatsRos::publish_images(const cv::Mat& umap, const cv::Mat& vmap, const cv::Mat& filtered){
     ros::Time time = ros::Time::now();
     /** Umap Image */
     if(!umap.empty()){
          this->_umapImgHeader.stamp = time;
          this->_umapImgHeader.seq = this->_count;
          this->_umapImgHeader.frame_id = this->_camera_tf;

          // sensor_msgs::ImagePtr umapImgMsg = cv_bridge::CvImage(this->_umapImgHeader, "8UC1", umap).toImageMsg();
          sensor_msgs::ImagePtr umapImgMsg;
          if(umap.type() == CV_8UC1) umapImgMsg = cv_bridge::CvImage(this->_umapImgHeader, "8UC1", umap).toImageMsg();
          else if(umap.type() == CV_16UC1) umapImgMsg = cv_bridge::CvImage(this->_umapImgHeader, "16UC1", umap).toImageMsg();
          else if(umap.type() == CV_32F) umapImgMsg = cv_bridge::CvImage(this->_umapImgHeader, "32FC1", umap).toImageMsg();
          else if(umap.type() == CV_8UC3) umapImgMsg = cv_bridge::CvImage(this->_umapImgHeader, "bgr8", umap).toImageMsg();
          this->_umap_pub.publish(umapImgMsg);
     }

     /** Vmap Image */
     if(!vmap.empty()){
          this->_vmapImgHeader.stamp = time;
          this->_vmapImgHeader.seq = this->_count;
          this->_vmapImgHeader.frame_id = this->_camera_tf;

          // sensor_msgs::ImagePtr vmapImgMsg = cv_bridge::CvImage(this->_vmapImgHeader, "8UC1", vmap).toImageMsg();
          sensor_msgs::ImagePtr vmapImgMsg;
          if(vmap.type() == CV_8UC1) vmapImgMsg = cv_bridge::CvImage(this->_vmapImgHeader, "8UC1", vmap).toImageMsg();
          else if(vmap.type() == CV_16UC1) vmapImgMsg = cv_bridge::CvImage(this->_vmapImgHeader, "16UC1", vmap).toImageMsg();
          else if(vmap.type() == CV_32F) vmapImgMsg = cv_bridge::CvImage(this->_vmapImgHeader, "32FC1", vmap).toImageMsg();
          else if(vmap.type() == CV_8UC3) vmapImgMsg = cv_bridge::CvImage(this->_vmapImgHeader, "bgr8", vmap).toImageMsg();
          this->_vmap_pub.publish(vmapImgMsg);
     }

     /** Filtered Disparity */
     if(!filtered.empty()){
          this->_filteredImgHeader.stamp = time;
          this->_filteredImgHeader.seq = this->_count;
          this->_filteredImgHeader.frame_id = this->_camera_tf;

          sensor_msgs::ImagePtr filteredImgMsg;
          if(filtered.type() == CV_8UC1) filteredImgMsg = cv_bridge::CvImage(this->_filteredImgHeader, "8UC1", filtered).toImageMsg();
          else if(filtered.type() == CV_16UC1) filteredImgMsg = cv_bridge::CvImage(this->_filteredImgHeader, "16UC1", filtered).toImageMsg();
          else if(filtered.type() == CV_32F) filteredImgMsg = cv_bridge::CvImage(this->_filteredImgHeader, "32FC1", filtered).toImageMsg();
          else if(filtered.type() == CV_8UC3) filteredImgMsg = cv_bridge::CvImage(this->_filteredImgHeader, "bgr8", filtered).toImageMsg();
          this->_new_img_pub.publish(filteredImgMsg);
     }
}
void VboatsRos::publish_obstacle_image(cv::Mat image){
     ros::Time time = ros::Time::now();
     /** Obstacles Image */
     if(!image.empty()){
          this->_obsImgHeader.stamp = time;
          this->_obsImgHeader.seq = this->_count;
          this->_obsImgHeader.frame_id = this->_camera_tf;

          sensor_msgs::ImagePtr obsImgMsg = cv_bridge::CvImage(this->_obsImgHeader, "bgr8", image).toImageMsg();
          this->_obstacles_img_pub.publish(obsImgMsg);
     }
}
void VboatsRos::publish_obstacle_data(vector<Obstacle>& obstacles, const cv::Mat& dImage){
     cv::Mat display = dImage.clone();
     if(this->_publish_images) cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);
     int n = 0;
     swanson_msgs::VboatsObstacles obsMsg;
     obsMsg.header.stamp = ros::Time::now();
     obsMsg.header.frame_id = this->_camera_tf;
     for(Obstacle ob : obstacles){
          if(this->_verbose_obstacles) printf("Obstacle [%d]: ", n+1);
          ob.update(false,this->_baseline, this->_dscale, this->_focal, this->_principle, 1.0, 1.0, this->_verbose_obstacles);

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

          if(this->_publish_images) cv::rectangle(display, ob.minXY, ob.maxXY, cv::Scalar(255, 0, 255), 1);
          n++;
     }
     this->_detected_obstacle_info_pub.publish(obsMsg);
     if(this->_publish_images) this->publish_obstacle_image(display);
}

/** SECTION:
     Misc Functions
*/
void VboatsRos::depth_to_disparity(const cv::Mat& depth, cv::Mat* disparity, float gain){
     cv::Mat _disparity, _disparity8;
     if(depth.type() != CV_32F) depth.convertTo(_disparity, CV_32F);
     else _disparity = depth.clone();

     ForEachDepthConverter<float> converter(gain);
     _disparity.forEach<float>(converter);

     if(_disparity.type() != CV_8UC1){
          double minVal, maxVal;
          cv::minMaxLoc(_disparity, &minVal, &maxVal);
          _disparity.convertTo(_disparity8, CV_8UC1, (255.0/maxVal) );
     } else _disparity8 = _disparity;
     if(disparity) *disparity = _disparity8.clone();
}
void VboatsRos::generate_pointcloud(cv::Mat& depth){
     // cloudxyz_t::Ptr pointcloud_msg (new cloudxyz_t);
     // pointcloud_msg->header.seq = this->_count;
     // pointcloud_msg->header.frame_id = this->_camera_tf;
     // pcl::PointXYZ pt;
     // double testT0 = (double)cv::getTickCount();
     // for(int y = 0; y < depth.rows; y+=4){
     //      for(int x = 0; x < depth.cols; x+=4){
     //           float depthVal = (float) depth.at<short int>(cv::Point(x,y)) * this->_dscale;
     //           if(depthVal > 0){
     //                pt.x = ((float)x - this->_px) * depthVal / this->_fx;
     //                pt.y = ((float)y - this->_py) * depthVal / this->_fy;
     //                pt.z = depthVal;
     //                pointcloud_msg->points.push_back(pt);
     //           }
     //      }
     // }
     // pointcloud_msg->height = 1;
     // pointcloud_msg->width = pointcloud_msg->points.size();
     // double testDt0 = ((double)cv::getTickCount() - testT0)/cv::getTickFrequency();
     // printf("[INFO] VboatsRos::generate_pointcloud(): Cloud generation Time: Naive = %.4lf ms (%.2lf Hz)\r\n", testDt0*1000.0, (1.0/testDt0));

     cloudxyz_t::Ptr test_cloud(new cloudxyz_t);
     test_cloud->header.seq = this->_count;
     test_cloud->header.frame_id = this->_camera_tf;
     short int* ptrP;
     pcl::PointXYZ pt;
     double testT1 = (double)cv::getTickCount();
     for(int y = 0; y < depth.rows; y+=4){
          ptrP = depth.ptr<short int>(y);
          for(int x = 0; x < depth.cols; x+=4){
               float depthVal = (float) ptrP[x] * this->_dscale;
               if(depthVal > 0){
                    pt.x = ((float)x - this->_px) * depthVal / this->_fx;
                    pt.y = ((float)y - this->_py) * depthVal / this->_fy;
                    pt.z = depthVal;
                    test_cloud->points.push_back(pt);
               }
          }
     }
     test_cloud->height = 1;
     test_cloud->width = test_cloud->points.size();
     double testDt1 = ((double)cv::getTickCount() - testT1)/cv::getTickFrequency();
     // printf("[INFO] VboatsRos::generate_pointcloud(): Cloud generation Time: Pointer = %.4lf ms (%.2lf Hz)\r\n", testDt1*1000.0, (1.0/testDt1));


     if(this->_flag_pub_cloud) this->_cloud_pub.publish(test_cloud);

     if(this->_filter_cloud){
          cloudxyz_t::Ptr cloud_filtered(new cloudxyz_t);

          pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ> ());
          range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -this->_max_obstacle_height)));
          range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, this->_min_obstacle_height)));
          range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, this->_min_obstacle_range)));
          range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr (new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, this->_max_obstacle_range)));

          pcl::ConditionalRemoval<pcl::PointXYZ> range_filt;
          range_filt.setInputCloud(test_cloud);
          range_filt.setCondition(range_cond);
          range_filt.filter(*cloud_filtered);

          pcl::VoxelGrid<pcl::PointXYZ> voxg;
          voxg.setInputCloud(cloud_filtered);
          voxg.setLeafSize(this->_voxel_res_x, this->_voxel_res_y, this->_voxel_res_z);
          voxg.filter(*cloud_filtered);

          // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
          // sor.setInputCloud(cloud_filtered);
          // sor.setMeanK(this->_sor_min_neighbors);
          // sor.setStddevMulThresh(this->_sor_dist_thresh);
          // sor.filter(*cloud_filtered);

          pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
          outrem.setInputCloud(cloud_filtered);
          outrem.setRadiusSearch(this->_sor_dist_thresh);
          outrem.setMinNeighborsInRadius(this->_sor_min_neighbors);
          outrem.filter(*cloud_filtered);

          if(this->_flag_pub_filtered_cloud) this->_filtered_cloud_pub.publish(cloud_filtered);
     }
}

/** SECTION:
     Processing Functions
*/
int VboatsRos::remove_ground(const cv::Mat& disparity, const cv::Mat& vmap, const cv::Mat& depth, float* line_params, cv::Mat* filtered_img){
     cv::Mat refImg = vmap.clone();
     cv::Mat img = disparity.clone();
     cv::Mat depthImg;
     if(!depth.empty()){ depthImg = depth.clone(); }
     else{
          printf("[WARNING] VboatsRos::remove_ground() --- Depth Image is empty, skipping ground removal.\r\n");
          return -1;
     }
     cv::Mat mask = cv::Mat::zeros(disparity.size(), CV_8UC1);

     /** Calculate ROI limits based on estimated ground line coefficients */
     float slope = line_params[0];
     int y0 = (int) line_params[1];
     int y1 = y0 + this->_gnd_lower_offset;
     int y1f = (int)(vmap.cols * slope + (y1));
     int y2 = y0 - this->_gnd_upper_offset;
     int y2f = (int)(vmap.cols * slope + (y2));
     if(y0 < 0) y0 = 0;
     // printf("[INFO] VboatsRos::remove_ground() --- slope = %.2f, intercept = %d\r\n", slope, b);

     uchar* pix;
     cv::Mat refRow;
     cv::Mat refMask;
     cv::Mat nonzero;
     double t, dt;
     int minx, maxx, tmpx;
     t = (double)cv::getTickCount();
     for(int v = y0; v < img.rows; ++v){
          refRow = refImg.row(v);
          refMask = refRow > 0;
          cv::findNonZero(refMask, nonzero);
          maxx = 0; minx = 1000;
          for(int i = 0; i < nonzero.total(); i++ ){
               tmpx = nonzero.at<cv::Point>(i).x;
               int xlim = (int)((float)(v - y2) / slope);
               if(tmpx > xlim) continue;
               if(tmpx > maxx) maxx = tmpx;
               if(tmpx < minx) minx = tmpx;
          }

          pix = img.ptr<uchar>(v);
          for(int u = 0; u < img.cols; ++u){
               int dvalue = pix[u];
               if( (dvalue >= minx) && (dvalue <= maxx) ) mask.at<uchar>(v, u) = 255;
          }
     }

     cv::Mat maskInv;
     cv::bitwise_not(mask,maskInv);
     cv::Mat gndFilteredImg;
     depthImg.copyTo(gndFilteredImg, maskInv);

     if(filtered_img) *filtered_img = gndFilteredImg.clone();
     return 0;
}
int VboatsRos::remove_objects(const cv::Mat& vmap, const cv::Mat& disparity, const cv::Mat& depth, const vector<vector<cv::Point>>& contours, float* line_params, cv::Mat* filtered_img){
     cv::Mat depthImg;
     /** If depth image is empty, exit since we have no image to filter objects from */
     if(!depth.empty()){ depthImg = depth.clone(); }
     else{
          printf("[WARNING] VboatsRos::remove_objects() --- Depth Image is empty, skipping object removal.\r\n");
          return -1;
     }
     if(this->_debug) printf("[INFO] VboatsRos::remove_objects() --- Initializing key variables.\r\n");
     int err = 0;
     cv::Mat disparityImg = disparity.clone();
     cv::Mat depthMask = cv::Mat::zeros(disparity.size(), CV_8UC1);

     /** Image-dependent variable initialization */
     int h = vmap.rows, w = vmap.cols;
     cv::Mat refImg = vmap.clone();
     cv::Mat vmask = cv::Mat::zeros(h,w, CV_8UC1);

     cv::Mat roiDisplay;
     bool debug_viz = true;
     bool debug_timing = false;
     if(this->_visualize_images){
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
          yf = (int)( (float)dmid * slope) + b - this->_gnd_upper_offset;
          if(yf >= h) yf = h;
          else if(yf < 0) yf = 0;
          if(this->_debug) printf("[INFO] VboatsRos::remove_objects() ------ Contour[%d]: dmin, dmid, dmax = (%d, %d, %d) |  ROI Rect = (%d, %d) -> (%d, %d)\r\n", i, dmin, dmid, dmax, dmin,y0, dmax,yf);

          /** Extract ROI containing current contour data from vmap for processing */
          cv::Rect roiRect = cv::Rect( cv::Point(dmin,y0), cv::Point(dmax,yf) );
          cv::Mat roi = refImg(roiRect);
          roi.copyTo(vmask(roiRect));

          if(this->_visualize_images){ cv::rectangle(roiDisplay, roiRect, cv::Scalar(0, 255, 255), 1); }
     }

     /** Naive Mask Generation: (DEPRECATED)
     double testT0 = (double)cv::getTickCount();
     cv::Mat nonzero;
     for(int r = 0; r < disparityImg.rows; r++){
          cv::Mat refRow = vmask.row(r);
          cv::findNonZero(refRow, nonzero);
          for(int c = 0; c < disparityImg.cols; c++){
               for(int idx = 0; idx < nonzero.total(); idx++){
                    if(disparityImg.at<uchar>(r, c) == nonzero.at<cv::Point>(idx).x){
                         depthMask.at<uchar>(r,c) = 255;
                         break;
                    }
               }
          }
     }
     double testDt0 = ((double)cv::getTickCount() - testT0)/cv::getTickFrequency();
     printf("[INFO] VboatsRos::remove_objects(): Segmentation mask generation Time: Naive = %.4lf ms (%.2lf Hz)\r\n", testDt0*1000.0, (1.0/testDt0));
     */

     if(this->_debug) printf("[INFO] VboatsRos::remove_objects() --- Creating Depth image mask.\r\n");
     double testT1 = (double)cv::getTickCount();
     ForEachObsMaskGenerator masker(vmask, h, 256);
     cv::Mat dMask = disparityImg.clone();
     dMask.forEach<uchar>(masker);
     if(debug_timing){
          double testDt1 = ((double)cv::getTickCount() - testT1)/cv::getTickFrequency();
          printf("[INFO] VboatsRos::remove_objects(): Segmentation mask generation Time: ForEach = %.4lf ms (%.2lf Hz)\r\n", testDt1*1000.0, (1.0/testDt1));
     }

     if(!depthMask.empty()){
          cv::Mat obsFilteredImg;
          depthImg.copyTo(obsFilteredImg, dMask);
          if(filtered_img) *filtered_img = obsFilteredImg.clone();

          if(this->_visualize_images){
               cv::Mat display;
               if(!vmask.empty()){
                    cv::applyColorMap(vmask, display, cv::COLORMAP_JET);
                    cv::imshow("Vmask", display);
               }
               if(!obsFilteredImg.empty()){
                    if(obsFilteredImg.type() != CV_8UC1){
                         double minVal, maxVal;
                         cv::minMaxLoc(obsFilteredImg, &minVal, &maxVal);
                         obsFilteredImg.convertTo(display, CV_8UC1, (255.0/maxVal) );
                    } else display = obsFilteredImg.clone();
                    cv::applyColorMap(display, display, cv::COLORMAP_JET);
                    cv::imshow("obsFilteredImg", display);
               }
               cv::imshow("ROI's ", roiDisplay);
               cv::waitKey(0);
          }
     } else{
          printf("[WARNING] VboatsRos::remove_objects() --- Object mask is empty, skipping object removal.\r\n");
          err = -2;
     }
     masker.remove();
     return err;
}

int VboatsRos::process(const cv::Mat& disparity, const cv::Mat& umap, const cv::Mat& vmap, vector<Obstacle>* obstacles, const cv::Mat& depth){
     int nObs = 0;
     vector<Obstacle> _obstacles;

     if(this->_debug) printf("[INFO] VboatsRos::process() --- Pre-filtering Umap.\r\n");
     /** Pre-filter Umap */
     cv::Mat uProcessed;
     cv::Mat uTmp = umap.clone();
     vector<vector<cv::Point>> deadzoneUmap;
     vector<cv::Point> deadzonePtsUmap = { cv::Point(0,0), cv::Point(umap.cols,0), cv::Point(umap.cols,3), cv::Point(0,3), cv::Point(0,0) };
     deadzoneUmap.push_back(deadzonePtsUmap);
     cv::fillPoly(uTmp, deadzoneUmap, cv::Scalar(0));
     cv::boxFilter(uTmp,uTmp,-1, cv::Size(2,2));
     if(!this->_use_custom_umap_filtering){
          vector<float> threshsU(this->_uThreshs);
          this->vb->filter_disparity_umap(uTmp, &uProcessed, &threshsU);
     } else{
          double uMin, uMax;
          cv::minMaxLoc(uTmp, &uMin, &uMax);
          int wholeThresh = int((float) uMax * this->_custom_uThresh_perc);
          cv::threshold(uTmp, uProcessed, wholeThresh, 255, cv::THRESH_TOZERO);
     }

     if(this->_debug) printf("[INFO] VboatsRos::process() --- Finding contours in filtered Umap.\r\n");
     /** Find contours in Umap needed later for obstacle filtering */
     vector<vector<cv::Point>> contours;
     this->vb->find_contours(uProcessed, &contours, this->_contourFiltMeth, this->_contourFiltMinThresh, nullptr, -1, false, this->_visualize_images);

     if(this->_debug) printf("[INFO] VboatsRos::process() --- Pre-filtering Vmap.\r\n");
     /** Pre-filter Vmap: Approach 1 */
     // cv::Mat vProcessed;
     cv::Mat vTmp = vmap.clone();
     vector<vector<cv::Point>> deadzoneVmap;
     vector<cv::Point> deadzonePtsVmap = { cv::Point(0,0), cv::Point(0,vmap.rows), cv::Point(3,vmap.rows), cv::Point(3,0), cv::Point(0,0) };
     deadzoneVmap.push_back(deadzonePtsVmap);
     cv::fillPoly(vTmp, deadzoneVmap, cv::Scalar(0));
     // vector<float> threshsV(this->_vThreshs);
     // this->vb->filter_disparity_vmap(vTmp, &vProcessed, &threshsV);

     if(this->_debug) printf("[INFO] VboatsRos::process() --- Creating Sobelized Vmap.\r\n");
     /** Pre-filter Vmap: Approach 2 - better highlight useful lines*/
     cv::Mat tmpSobel, sobelV;
     double minVal, maxVal;
     cv::Sobel(vTmp, tmpSobel, CV_64F, 0, 1, 3);
     cv::minMaxLoc(tmpSobel, &minVal, &maxVal);
     tmpSobel = tmpSobel * (255.0/maxVal);
     cv::convertScaleAbs(tmpSobel, tmpSobel, 1, 0);
     threshold(tmpSobel, sobelV, this->_vmap_sobel_thresh, 255, cv::THRESH_TOZERO);

     if(this->_debug) printf("[INFO] VboatsRos::process() --- Looking for Ground line.\r\n");
     /** Extract ground line parameters (if ground is present) */
     float* line_params;
     float gndM; int gndB;
     bool gndPresent = this->vb->find_ground_line(sobelV, &gndM,&gndB);
     if(gndPresent){
          float tmpParams[] = {gndM, (float) gndB};
          line_params = &tmpParams[0];
     } else line_params = nullptr;

     if(this->_debug) printf("[INFO] VboatsRos::process() --- Creating Segmentation Mask.\r\n");
     cv::Mat noGndImg, noObsImg, segInputImg;
     if(this->_sobel_vmask_subtract){
          cv::Mat segMask;
          threshold(sobelV, segMask, 0, 255, cv::THRESH_BINARY);
          if(this->_dilate_sobel_to_segmask){
               cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( this->_sobel_dilate_ksize, this->_sobel_dilate_ksize));
               cv::dilate(segMask, segMask, element, cv::Point(-1,-1), 1);
          }
          cv::bitwise_not(segMask,segMask);
          vTmp.copyTo(segInputImg, segMask);
     } else segInputImg = vTmp.clone();

     int err;
     if(this->_debug) printf("[INFO] VboatsRos::process() --- Performing Obstacle Segmentation.\r\n");
     if(this->_use_gnd_meth){                         /** Ground segmentation */
          if((gndPresent) && (this->_filter_ground)){
               err = this->remove_ground(disparity,segInputImg, depth, line_params, &noGndImg);
               if(err >= 0){
                    if(!noGndImg.empty()) this->generate_pointcloud(noGndImg);
                    else printf("[WARNING] VboatsRos::process() --- Ground filtered depth image is empty, skipping pointcloud generation.\r\n");
               } else printf("[WARNING] VboatsRos::process() --- Unable to filter ground from depth image, skipping pointcloud generation.\r\n");
          }
     } else{                                        /** Obstacle Segmentation */
          err = this->remove_objects(segInputImg, disparity, depth, contours, line_params, &noObsImg);
          if(err >= 0){
               if(!noObsImg.empty()) this->generate_pointcloud(noObsImg);
               else printf("[WARNING] VboatsRos::process() --- Object filtered depth image is empty, skipping pointcloud generation.\r\n");
          } else printf("[WARNING] VboatsRos::process() --- Unable to filter objects from depth image, skipping pointcloud generation.\r\n");
     }

     /** Obstacle data extraction */
     if(this->_detect_obstacles){
          /** Extract obstacles */
          nObs = this->vb->find_obstacles_disparity(vmap, contours, &_obstacles, line_params);
          this->publish_obstacle_data(_obstacles, disparity);
     }

     if(this->_publish_aux_images){
          cv::Mat uDisplay, vDisplay, gndDisplay;
          if(!uProcessed.empty()){
               cv::convertScaleAbs(uProcessed,uDisplay);
               cv::applyColorMap(uDisplay, uDisplay, cv::COLORMAP_JET);
          }
          if(!vTmp.empty()){
               cv::convertScaleAbs(vTmp,vDisplay);
               if(gndPresent){
                    cv::cvtColor(vDisplay, vDisplay, cv::COLOR_GRAY2BGR);
                    int yk = int(vTmp.cols * gndM) + gndB;
                    int yu = int(vTmp.cols * gndM) + (gndB-this->_gnd_upper_offset);
                    int yl = int(vTmp.cols * gndM) + (gndB+this->_gnd_lower_offset);
                    cv::line(vDisplay, cv::Point(0, gndB), cv::Point(vTmp.cols, yk), cv::Scalar(0,255,0), 2, cv::LINE_AA);
                    cv::line(vDisplay, cv::Point(0, (gndB-this->_gnd_upper_offset)), cv::Point(vTmp.cols, yu), cv::Scalar(255,0,0), 2, cv::LINE_AA);
                    cv::line(vDisplay, cv::Point(0, (gndB+this->_gnd_lower_offset)), cv::Point(vTmp.cols, yl), cv::Scalar(0,0,255), 2, cv::LINE_AA);
               } else cv::applyColorMap(vDisplay, vDisplay, cv::COLORMAP_JET);
          }
          if(this->_use_gnd_meth){
               if(!noGndImg.empty()){
                    if(noGndImg.type() != CV_8UC1){
                         cv::minMaxLoc(noGndImg, &minVal, &maxVal);
                         noGndImg.convertTo(gndDisplay, CV_8UC1, (255.0/maxVal) );
                    } else gndDisplay = noGndImg.clone();
                    cv::applyColorMap(gndDisplay, gndDisplay, cv::COLORMAP_JET);
               }
          } else{
               if(!noObsImg.empty()){
                    if(noObsImg.type() != CV_8UC1){
                         cv::minMaxLoc(noObsImg, &minVal, &maxVal);
                         noObsImg.convertTo(gndDisplay, CV_8UC1, (255.0/maxVal) );
                    } else gndDisplay = noObsImg.clone();
                    cv::applyColorMap(gndDisplay, gndDisplay, cv::COLORMAP_JET);
               }
          }
          this->publish_images(uDisplay, vDisplay, gndDisplay);
     }

     if(this->_visualize_images){
          imshowCmap(uTmp, "uTmp");
          imshowCmap(uProcessed, "uProcessed");
          imshowCmap(vTmp, "vTmp");
          imshowCmap(sobelV, "sobelV");
          imshowCmap(segInputImg, "segInputImg");
          cv::waitKey(1);
     }

     if(obstacles) *obstacles = _obstacles;
     return nObs;
}

/** SECTION:
     Execution Functions
*/
int VboatsRos::update(bool verbose, bool debug_timing){
     int nObs = 0;
     vector<Obstacle> obs;
     cv::Mat curDepth, curDisparity, umap, vmap;
     /** Don't do any proessing if we haven't received valid camera info */
     if(!this->_recvd_cam_info) return -1;
     if(!this->_recvd_image) return -2;

     this->_lock.lock();
     if(this->_flag_depth_based){
          curDepth = this->_depth;
          this->depth_to_disparity(curDepth,&curDisparity, this->_depth2disparityFactor);
     } else curDisparity = this->_disparity;
     this->_lock.unlock();
     /** Don't do any proessing if disparity is empty */
     if(curDisparity.empty()) return -1;

     double t = (double)cv::getTickCount();
     genUVMapThreaded(curDisparity,&umap,&vmap, 2.0);
     nObs = this->process(curDisparity, umap, vmap, &obs, curDepth);

     this->_count++;
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

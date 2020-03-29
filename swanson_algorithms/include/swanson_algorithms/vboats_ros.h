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
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <swanson_algorithms/VboatsConfig.h>

typedef pcl::PointCloud<pcl::PointXYZ> cloudxyz_t;

using namespace std;

/** Templated struct for fast conversion of depth image into disparity */
template<typename dtype> struct ForEachDepthConverter{
     dtype m_gain;
     ForEachDepthConverter(dtype gain){
          m_gain = gain;
     }
     void operator()(dtype& pixel, const int * idx) const{
          if(pixel != 0.0){ pixel = m_gain / pixel; }
     }
};

/** Templated structure for fast generation of object segmentation mask */
struct ForEachObsMaskGenerator{
     int ** m_table;
     const size_t m_rows;
     const size_t m_cols;
     ForEachObsMaskGenerator(const cv::Mat& input_mask, int num_rows, int num_cols) : m_rows(num_rows), m_cols(num_cols){
          cv::Mat nonzero;
          m_table = new int*[num_rows]();
          for(int v = 0; v < num_rows; v++){
               if(m_table[v]) delete[] m_table[v];
               m_table[v] = new int[num_cols]();
               memset(m_table[v], 0, num_cols*sizeof(int));
               cv::Mat refRow = input_mask.row(v);
               cv::findNonZero(refRow, nonzero);
               for(int u = 0; u < nonzero.total(); ++u){
                    int tmpx = nonzero.at<cv::Point>(u).x;
                    m_table[v][tmpx] = 255;
               }
          }
     }
     void remove(){
          for(auto i = 0; i < m_rows; i++){
               if(m_table[i]) delete[] m_table[i];
               m_table[i] = nullptr;
          }
          if(m_table) delete[] m_table;
          m_table = nullptr;
     }

     void operator()(uchar& pixel, const int * position) const {
          if((position[0] < m_rows) && (pixel < m_cols)){ pixel = (uchar) m_table[position[0]][pixel]; }
     }
};

class VboatsRos{
private:
     /** ROS Objects */
     ros::NodeHandle m_nh;
     ros::NodeHandle p_nh;
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
     bool _debug;
     bool _verbose;
     bool _verbose_obstacles;
     bool _verbose_timings;
     bool _publish_images;
     bool _publish_aux_images;
     bool _visualize_images;
     bool _recvd_cam_info;
     bool _recvd_image;
     bool _flag_depth_based;
     bool _detect_obstacles;
     bool _filter_ground;
     bool _filter_cloud;
     bool _flag_pub_cloud;
     bool _flag_pub_filtered_cloud;
     /** Configurable Parameters */
     bool _use_gnd_meth;
     bool _do_cloud_downsampling;
     bool _do_cloud_outlier_removal;
     bool _do_cloud_limit_filtering;
     int _contourFiltMeth;
     float _contourFiltMinThresh;
     float _max_obstacle_height;
     float _min_obstacle_height;
     float _max_obstacle_range;
     float _min_obstacle_range;
     float _voxel_res_x;
     float _voxel_res_y;
     float _voxel_res_z;
     int _gnd_upper_offset;
     int _gnd_lower_offset;
     double _max_gnd_ang;
     double _min_gnd_ang;
     int _sor_min_neighbors;
     float _sor_dist_thresh;
     vector<float> _uThreshs;
     vector<float> _vThreshs;

     bool _use_custom_umap_filtering;
     bool _sobel_vmask_subtract;
     float _custom_uThresh_perc;
     float _vmap_sobel_thresh;
     bool _dilate_sobel_to_segmask;
     int _sobel_dilate_ksize;
     /** Multi-threading objects */
     std::mutex _lock;

     /** ROS Subscriber Callbacks */
     void cfgCallback(swanson_algorithms::VboatsConfig &config, uint32_t level);
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
     void publish_obstacle_data(vector<Obstacle>& obstacles, const cv::Mat& dImage);

     void generate_pointcloud(cv::Mat& depth);
     int remove_ground(const cv::Mat& disparity, const cv::Mat& vmap, const cv::Mat& depth, float* line_params, cv::Mat* filtered_img);
     int remove_objects(const cv::Mat& vmap, const cv::Mat& disparity, const cv::Mat& depth, const vector<vector<cv::Point>>& contours, float* line_params, cv::Mat* filtered_img);

     int process(const cv::Mat& disparity, const cv::Mat& umap, const cv::Mat& vmap, vector<Obstacle>* obstacles, const cv::Mat& depth);
     int update(bool verbose = false, bool debug_timing = true);

     int run(bool verbose = false);
};


#endif // VBOATS_ROS_H_

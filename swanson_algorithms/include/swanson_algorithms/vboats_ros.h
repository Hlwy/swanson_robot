#ifndef VBOATS_ROS_H_
#define VBOATS_ROS_H_

#include <thread>
#include <atomic>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/core/version.hpp>

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
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <RoboCommander/algorithms/vboats/vboats.h>

typedef pcl::PointCloud<pcl::PointXYZ> cloudxyz_t;

using namespace std;

template<typename dtype>
struct ForEachDepthConverter{
     dtype m_gain;
     ForEachDepthConverter(dtype gain){ m_gain = gain; }

     void operator()(dtype& pixel, const int * idx) const {
          if(pixel != 0.0){ pixel = m_gain / pixel; }
     }
};

template<typename dtype>
struct ForEachObsMaskGenerator{
     std::vector< std::vector<int> > goodVals;
     ForEachObsMaskGenerator(std::vector< std::vector<int> > table){
          for(int i = 0; i < table.size(); i++){
               std::vector<int> tmpvec = table.at(i);
               goodVals.push_back(tmpvec);
          }
     }

     void operator()(dtype& pixel, const int * position) const {
          bool matched = false;
          int currow = position[0];
          std::vector<int> tmpVals = goodVals.at(currow);
          for(int idx = 0; idx < tmpVals.size(); idx++){
               if(matched) continue;
               if((int) pixel == tmpVals[idx]) matched = true;
          }
          if(matched) pixel = (dtype) 255;
          else pixel = 0;
     }
};

template<typename dtype>
struct ForEachPclOperator{
     float m_scale;
     float m_fx;
     float m_fy;
     float m_px;
     float m_py;
     cloudxyz_t::Ptr m_cloud;
     ForEachPclOperator(float dscale, float px, float py, float fx, float fy){
          m_scale = dscale;
          m_px = px;
          m_py = py;
          m_fx = fx;
          m_fy = fy;
          m_cloud.reset(new cloudxyz_t);
     }
     void operator()(dtype& pixel, const int * idx) const {
          float tmpVal = (float)pixel * m_scale;
          if(tmpVal > 0.0){
               pcl::PointXYZ pt;
               pt.x = ((float)idx[0] - m_px) * tmpVal / m_fx;
               pt.y = ((float)idx[1] - m_py) * tmpVal / m_fy;
               pt.z = tmpVal;
               m_cloud->points.push_back(pt);
          }
     }
};

class VboatsRos{
private:
     /** ROS Objects */
     ros::NodeHandle m_nh;
     ros::NodeHandle p_nh;
     tf::TransformBroadcaster _br;
     image_transport::ImageTransport _it;
     ros::Rate* _loop_rate;
     int _update_rate;

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
     bool _publish_aux_images;
     bool _visualize_images;
     bool _use_gnd_meth;
     bool _recvd_cam_info;
     bool _recvd_image;
     bool _flag_depth_based;
     bool _detect_obstacles;
     bool _filter_ground;
     bool _filter_cloud;
     bool _flag_pub_cloud;
     bool _flag_pub_filtered_cloud;
     /** Configurable Parameters */
     float _max_obstacle_height;
     float _min_obstacle_height;
     float _max_obstacle_range;
     float _min_obstacle_range;
     float _voxel_res_x;
     float _voxel_res_y;
     float _voxel_res_z;
     int _gnd_upper_offset;
     int _gnd_lower_offset;
     int _sor_min_neighbors;
     float _sor_dist_thresh;
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
     void publish_obstacle_data(vector<Obstacle>& obstacles, const cv::Mat& dImage);

     void generate_pointcloud(const cv::Mat& depth);
     int remove_ground(const cv::Mat& disparity, const cv::Mat& vmap, const cv::Mat& depth, float* line_params, cv::Mat* filtered_img);
     int remove_objects(const cv::Mat& vmap, const cv::Mat& disparity, const cv::Mat& depth, const vector<vector<cv::Point>>& contours, float* line_params, cv::Mat* filtered_img);

     int process(const cv::Mat& disparity, const cv::Mat& umap, const cv::Mat& vmap, vector<Obstacle>* obstacles, const cv::Mat& depth);
     int update(bool verbose = false, bool debug_timing = true);

     int run(bool verbose = false);
};


#endif // VBOATS_ROS_H_

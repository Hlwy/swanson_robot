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


#include <RoboCommander/utilities/utils.h>
#include <RoboCommander/utilities/image_utils.h>
#include <RoboCommander/algorithms/vboats/uvmap_utils.h>

#define M_DEG2RAD (2*M_PI)/360
#define M_RAD2DEG 360/(2*M_PI)

using namespace std;

/** SECTION:
     CONSTRUCTOR & DECONSTRUCTOR
*/
int main(int argc, char *argv[]){
     const std::string imgDir = "/home/hyoung/devel/vision_playground/zed2_test/";

     // double correctionAngle = -1.520814;
     // std::string depthF = imgDir + "flat/zed_depth_raw_flat.tiff";
     // double correctionAngle = -8.245815;
     // std::string depthF = imgDir + "angled_slight_right/zed_depth_raw_angled_slight_right.tiff";
     double correctionAngle = -31.125103;
     std::string depthF = imgDir + "angled_hard_right/zed_depth_raw_angled_hard_right.tiff";

     printf("Reading in image \'%s\'...\r\n",depthF.c_str());
     cv::Mat image = cv::imread(depthF, cv::IMREAD_UNCHANGED);
     cvinfo(image, "update() --- Depth Image before preprocessing: ");
     ForEachPrepareDepthConverter<float> preconverter;
     image.forEach<float>(preconverter);
     cvinfo(image, "update() --- Depth Image after preprocessing: ");

     cv::Mat inViz = imCvtCmap(image);
     cv::imshow("Image Input", inViz);
     cv::waitKey(0);

     cv::Mat depthInput, genDisparity, umap, vmap;
     depthInput = rotate_image(image, correctionAngle);
     if(!depthInput.empty()){
          cvinfo(depthInput, "Depth Image after warping: ");
          imshowCmap(depthInput, "Warped Depth Image");
          cv::waitKey(0);

          // this->depth_to_disparity(depthInput,&genDisparity, disparityGain);
          // cvinfo(depthInput, "Depth Image after warping: ");
          // imshowCmap(depthInput, "Warped Depth Image");
          // cv::waitKey(0);
          //
          // genUVMapThreaded(genDisparity,&umap,&vmap, 2.0);
          // cvinfo(depthInput, "Depth Image after warping: ");
          // imshowCmap(depthInput, "Warped Depth Image");
          // cv::waitKey(0);
     }

     return 0;
}

// void depth_to_disparity(const cv::Mat& depth, cv::Mat* disparity, float gain){
//      if(depth.empty()) return;
//      cv::Mat _disparity, _disparity8;
//      if(this->_debug_disparity_generation){
//           printf("[DEBUG] %s::depth_to_disparity() --- Using conversion gain = %f.\r\n", this->classLbl.c_str(), gain);
//           cvinfo(depth, "VboatsRos::depth_to_disparity() --- Input Depth Image: ");
//      }
//      if(depth.type() != CV_32F) depth.convertTo(_disparity, CV_32F);
//      else _disparity = depth.clone();
//      if(this->_debug_disparity_generation) cvinfo(_disparity, "VboatsRos::depth_to_disparity() --- Input image before Disparity conversion: ");
//
//      // ForEachDepthConverter<float> converter(gain);
//      ForEachDepthConverter<float> converter(gain, this->_min_obstacle_range, this->_max_obstacle_range);
//      _disparity.forEach<float>(converter);
//      if(_disparity.empty()) return;
//
//      if(this->_debug_disparity_generation) cvinfo(_disparity, "VboatsRos::depth_to_disparity() --- Converted Disparity Image: ");
//      if(_disparity.type() != CV_8UC1){
//           double minVal, maxVal;
//           cv::minMaxLoc(_disparity, &minVal, &maxVal);
//           _disparity.convertTo(_disparity8, CV_8UC1, (255.0/maxVal) );
//      } else _disparity8 = _disparity;
//      if(this->_debug_disparity_generation) cvinfo(_disparity8, "VboatsRos::depth_to_disparity() --- Stored Disparity Image: ");
//      if(disparity) *disparity = _disparity8.clone();
// }

#include <ros/ros.h>
#include "swanson_algorithms/vboats_ros.h"
#include <unistd.h>

using namespace std;

int main (int argc, char** argv){

     ros::init(argc, argv, "vboats_ros_node");
     ros::NodeHandle nh;
	ros::NodeHandle _nh("~");

     // mVboats.start();
     // usleep(60 * 1000000);
     int fps = 90;
	int rgb_resolution[2] = {848, 480};
	int depth_resolution[2] = {848, 480};
	// CameraD415* cam = new CameraD415(60, rgb_resolution, fps, depth_resolution);
	// cam->enable_alignment();
     // cam->enable_filters();
     // cam->start_thread();
     //
     // VboatsRos mVboats(nh, _nh, cam);
     VboatsRos mVboats(nh, _nh);
     mVboats.run(true);
     // while(ros::ok()){
     //      cv::Mat depth, rgb;
     // 	double cvtGain, cvtRatio;
     // 	int err = mVboats.cam->get_processed_queued_images(&rgb, &depth);
     // }
     // delete cam;
     return 0;
}

#include <ros/ros.h>
#include "swanson_sensors/camera_d4xx_ros.h"

using namespace std;

int main (int argc, char** argv){

     ros::init(argc, argv, "camera_d4xx_ros_node");
     ros::NodeHandle nh;
	ros::NodeHandle _nh("~");
     CameraD4XXRos mCam(nh, _nh);

     mCam.run(true);
}

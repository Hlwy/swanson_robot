#include <ros/ros.h>
#include "swanson_controls/camera_gimbal_controller_ros.h"

using namespace std;

int main (int argc, char** argv){

     ros::init(argc, argv, "camera_gimbal_controller_node");
     ros::NodeHandle nh;
	ros::NodeHandle _nh("~");
     CameraGimbalControllerRos gimbal(nh, _nh);

     gimbal.run(true);
}

#include <ros/ros.h>
#include "swanson_algorithms/angle_test.h"

int main (int argc, char** argv){
     ros::init(argc, argv, "angle_test_ros_node");
     ros::NodeHandle nh;
	ros::NodeHandle _nh("~");
     AngleTestRos mVboats(nh, _nh);
     mVboats.run();
     return 0;
}

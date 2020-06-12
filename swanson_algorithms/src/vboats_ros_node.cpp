#include <ros/ros.h>
#include "swanson_algorithms/vboats_ros.h"

int main (int argc, char** argv){
     ros::init(argc, argv, "vboats_ros_node");
     ros::NodeHandle nh;
	ros::NodeHandle _nh("~");
     VboatsRos mVboats(nh, _nh);
     mVboats.run();
     return 0;
}

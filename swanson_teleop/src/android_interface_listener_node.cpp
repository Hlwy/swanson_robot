#include <ros/ros.h>
#include "swanson_teleop/android_ros_interface_listener.h"

using namespace std;

int main (int argc, char** argv){

     ros::init(argc, argv, "android_ros_interface_listener_node");
     ros::NodeHandle nh;
	ros::NodeHandle _nh("~");
     AndroidRosInterface app_iface(nh, _nh);

     app_iface.run();
}

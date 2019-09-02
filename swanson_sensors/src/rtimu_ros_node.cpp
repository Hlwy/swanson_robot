#include <ros/ros.h>
#include "swanson_sensors/rtimu_ros.h"

using namespace std;

int main (int argc, char** argv){

     ros::init(argc, argv, "rtimu_ros_node");
     ros::NodeHandle nh;
	ros::NodeHandle _nh("~");
     RtImuRos mImu(nh, _nh);

     mImu.run(true);
}

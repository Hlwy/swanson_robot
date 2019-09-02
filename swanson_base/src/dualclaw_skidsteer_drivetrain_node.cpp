#include <ros/ros.h>
#include "swanson_base/dual_roboclaw_skidsteer_drivetrain.h"

using namespace std;

int main (int argc, char** argv){

     ros::init(argc, argv, "dualclaw_skidsteer_drivetrain_node");
     ros::NodeHandle nh;
	ros::NodeHandle _nh("~");
     DualClawSkidsteerDrivetrainInterface drivetrain(nh, _nh);

     drivetrain.run(true);
}

#include <ros/ros.h>
#include "swanson_base/diffdrive_roboclaw_skidsteer_drivetrain.h"

using namespace std;

int main (int argc, char** argv){

    ros::init(argc, argv, "diffdrive_skidsteer_drivetrain_node");
    ros::NodeHandle nh;
    ros::NodeHandle _nh("~");
    DiffDriveClawSkidsteerDrivetrainInterface drivetrain(nh, _nh);

    drivetrain.run(true);
}
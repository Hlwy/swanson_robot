#include <pigpiod_if2.h>
#include <ros/ros.h>
#include <RoboCommander/devices/tca9548a.h>

#include "swanson_sensors/bno055_i2c_ros.h"

using namespace std;

int main (int argc, char** argv){
     ros::init(argc, argv, "bno055_i2c_ros_node");
     ros::NodeHandle nh;
     ros::NodeHandle _nh("~");

     /* Connect to Pi. */
     int pi = pigpio_start(NULL, NULL);
	if(pi < 0) return -1;
     printf("[INFO] BNO055_I2C_Node --- pigpiod initialized [%d] declared.\r\n", pi);
     // if(useMux) TCA9548A mux(pi, bus);
     // else

     BNO055_I2C_Ros mImu(nh, _nh, &pi);

     mImu.run(true);
}

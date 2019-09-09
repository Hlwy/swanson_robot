#include <pigpiod_if2.h>
#include <ros/ros.h>
#include <RoboCommander/devices/tca9548a.h>
#include "swanson_sensors/bno055_i2c_ros.h"

using namespace std;

int main (int argc, char** argv){
     ros::init(argc, argv, "bno055_multiplex_node");
     ros::NodeHandle nh;
     ros::NodeHandle _nh("~");

	int mux_addr, mux_channel, mux_bus, update_rate;
     nh.param<int>("muxAddr", mux_addr, 0x41);
     // nh.param<int>("muxChannel", mux_channel, 0);
     nh.param<int>("muxBus", mux_bus, 1);
     nh.param<int>("update_rate", update_rate, 20);

     ros::Rate rate = ros::Rate(update_rate);

     /* Connect to Pi. */
     int pi = pigpio_start(NULL, NULL);
	if(pi < 0) return -1;
     printf("[INFO] BNO055_I2C_Node --- pigpiod initialized [%d] declared.\r\n", pi);

     TCA9548A mux(pi, mux_bus);

     BNO055_I2C_Ros imu1("imu/f", nh, _nh, &pi, 6, &mux);
     BNO055_I2C_Ros imu2("imu/r", nh, _nh, &pi, 1, &mux);
     BNO055_I2C_Ros imu3("imu/l", nh, _nh, &pi, 3, &mux);

     int dummy;
	cout << "Please press [Enter] to start retrieving IMU values...";
	cin >> dummy;
     cout << "Looping..." << endl;

     while(ros::ok()){
		imu1.update();
		imu2.update();
		imu3.update();
          ros::spinOnce();
          rate.sleep();
     }

     return 0;
}

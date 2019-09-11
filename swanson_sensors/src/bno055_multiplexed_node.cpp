#include <pigpiod_if2.h>
#include <ros/ros.h>
#include <RoboCommander/devices/tca9548a.h>
#include "swanson_sensors/bno055_i2c_ros.h"

using namespace std;

int main (int argc, char** argv){
     ros::init(argc, argv, "bno055_multiplex_node");
     ros::NodeHandle nh;
     ros::NodeHandle _nh("~");

     /* Initialize i2c multiplexer */
	int mux_addr, mux_bus;
     nh.param<int>("mux_address", mux_addr, 0x40);
     nh.param<int>("mux_bus", mux_bus, 1);

     int update_rate = 20;
     bool verbose = true;
     std::string imu_topic = "/data";
     std::string angle_topic = "/angles";
     nh.getParam("imu_topic",imu_topic);
     nh.getParam("angle_topic", angle_topic);
     nh.getParam("update_rate",update_rate);
     nh.getParam("verbose", verbose);

     int ch_f, ch_r, ch_l, bus_f, bus_r, bus_l, addr_f, addr_r, addr_l;
     std::string prefix_f, prefix_r, prefix_l;
     /** Front IMU */
     nh.param<std::string>("imu/f/prefix", prefix_f, "imu");
     nh.param<int>("imu/f/bus", bus_f, 1);
     nh.param<int>("imu/f/mux_channel", ch_f, 0);
     nh.param<int>("imu/f/address", addr_f, 0x28);

     /** Right IMU */
     nh.param<std::string>("imu/r/prefix", prefix_r, "imu");
     nh.param<int>("imu/r/bus", bus_r, 1);
     nh.param<int>("imu/r/mux_channel", ch_r, 1);
     nh.param<int>("imu/r/address", addr_r, 0x28);
     /** Left IMU */
     nh.param<std::string>("imu/l/prefix", prefix_l, "imu");
     nh.param<int>("imu/l/bus", bus_l, 1);
     nh.param<int>("imu/l/mux_channel", ch_l, 2);
     nh.param<int>("imu/l/address", addr_l, 0x28);

     printf("=========== Interpretted ROS Parameters =================\r\n");
     printf("ROS Parameters:\r\n");
     printf("  mux_addr:    0x%02X\r\n", mux_addr);
     printf("  mux_bus:     %d\r\n", mux_bus);
     printf("  imu_topic:   %s\r\n", imu_topic.c_str());
     printf("  angle_topic: %s\r\n", angle_topic.c_str());
     printf("  update_rate: %d\r\n", update_rate);

     ros::Rate rate = ros::Rate(update_rate);

     /* Connect to Pi. */
     int pi = pigpio_start(NULL, NULL);
	if(pi < 0) return -1;
     /* Initialize i2c multiplexer */
     TCA9548A mux(pi, mux_bus);
     /* Initialize i2c multiplexed BNO-055's */
     // BNO055_I2C_Ros imu1("imu/l", nh, _nh, &pi, 6, &mux);
     // BNO055_I2C_Ros imu2("imu/r", nh, _nh, &pi, 1, &mux);
     // BNO055_I2C_Ros imu3("imu/f", nh, _nh, &pi, 3, &mux);

     BNO055_I2C_Ros imuF(nh, prefix_f, &pi, &mux, bus_f, addr_f, ch_f, \
          imu_topic,angle_topic, update_rate,verbose);
     BNO055_I2C_Ros imuR(nh, prefix_r, &pi, &mux, bus_r, addr_r, ch_r, \
          imu_topic,angle_topic, update_rate,verbose);
     BNO055_I2C_Ros imuL(nh, prefix_l, &pi, &mux, bus_l, addr_l, ch_l, \
          imu_topic,angle_topic, update_rate,verbose);

     int dummy;
	cout << "Please press [Enter] to start retrieving IMU values...";
	cin >> dummy;
     cout << "Looping..." << endl;
     // while(ros::ok()){
	// 	imu1.update();
	// 	imu2.update();
	// 	imu3.update();
     //      ros::spinOnce();
     //      rate.sleep();
     // }

     return 0;
}

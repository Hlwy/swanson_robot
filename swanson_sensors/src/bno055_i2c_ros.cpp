#include <iostream>
#include <pigpiod_if2.h>
#include "swanson_sensors/bno055_i2c_ros.h"

using namespace std;

/** SECTION:
     CONSTRUCTOR & DECONSTRUCTOR
*/
BNO055_I2C_Ros::BNO055_I2C_Ros(ros::NodeHandle nh, ros::NodeHandle _nh, int* pih) : m_nh(nh), p_nh(_nh){
     // Declare constants
	_count = 0;
	int pi;
	int bus = 1;
	int i2cAddr = 0x28;
	bool verbose = false;
	int update_rate = 20;
	std::string prefix;
	std::string imu_topic = "imu/data";
	std::string angle_topic = "imu/angles";

	bool useMux;
	int mux_addr, mux_channel;
     nh.param<int>("muxAddr", mux_addr, 0x41);
     nh.param<int>("muxChannel", mux_channel, 0);
     nh.param<bool>("useMux", useMux, false);

	p_nh.getParam("i2c_bus",bus);
	p_nh.getParam("i2c_addr",i2cAddr);

	p_nh.getParam("prefix",prefix);
	p_nh.getParam("imu_topic",imu_topic);
	p_nh.getParam("angle_topic",angle_topic);
	p_nh.getParam("update_rate",update_rate);
	p_nh.getParam("verbose",verbose);

	/* Connect to Pi. */
	if(!pih){
		printf("[INFO] BNO055_I2C_Ros::BNO055_I2C_Ros() --- no pigpiod handle declared. Attempting to connect to pigpiod...\r\n");
		pi = pigpio_start(NULL, NULL);
	}
	else{
		pi = *pih;
		printf("[INFO] BNO055_I2C_Ros::BNO055_I2C_Ros() --- pigpiod handle [%d] declared.\r\n", pi);
	}

	if(pi < 0){
		printf("[ERROR] Could not initialize with the pigpiod \r\n");
		exit(0);
	}else{ this->_pi = pi; }

	/** Initialize IMU */
	this->imu = new BNO055_I2C(pi, 1, 0x28);
	int err = imu->startup(verbose);
	if(err < 0){
		printf("[ERROR] Could not initialize Imu. Error Code = %d\r\n", err);
		exit(0);
	}

	// imu_pub = m_nh.advertise<sensor_msgs::Imu>(imu_topic, 1000);
	angle_pub = m_nh.advertise<geometry_msgs::Vector3>(angle_topic, 1000);
	// pose_pub = m_nh.advertise<geometry_msgs::Pose>(pose_topic, 1000);

	_loop_rate = new ros::Rate(update_rate);
	usleep(2 * 1000000);
}

BNO055_I2C_Ros::~BNO055_I2C_Ros(){}


void BNO055_I2C_Ros::update(bool verbose){
	_count++;
	float _angles[3];
	imu->get_euler(&_angles[0]);
	if(verbose) printf("Euler Angles: %f, %f, %f\r\n", _angles[0], _angles[1] , _angles[2]);

     // ros::Time curTime = ros::Time::now();
     // sensor_msgs::Imu imuMsg;
     // imuMsg.header.stamp = curTime;
	// imuMsg.header.seq = _count;
	// imuMsg.orientation.x = imu->quats[0];
	// imuMsg.orientation.y = imu->quats[1];
	// imuMsg.orientation.z = imu->quats[2];
	// imuMsg.orientation.w = imu->quats[3];
	// imuMsg.angular_velocity.x = imu->gyro[0];
	// imuMsg.angular_velocity.y = imu->gyro[1];
	// imuMsg.angular_velocity.z = imu->gyro[2];
	// imuMsg.linear_acceleration.x = imu->accel[0];
	// imuMsg.linear_acceleration.y = imu->accel[1];
	// imuMsg.linear_acceleration.z = imu->accel[2];
     // imu_pub.publish(imuMsg);

     geometry_msgs::Vector3 angMsg;
     angMsg.x = _angles[0];
     angMsg.y = _angles[1];
     angMsg.z = _angles[2];
     angle_pub.publish(angMsg);

     // float roll = fmod((imu->euler[0]*M_RAD2DEG + 360.0),360.0);
     // float pitch = fmod((imu->euler[1]*M_RAD2DEG + 360.0),360.0);
     // float yaw = fmod((imu->euler[2]*M_RAD2DEG + 360.0),360.0);
	//
	// float roll = fmod((_angles[0]*M_RAD2DEG + 360.0),360.0);
     // float pitch = fmod((_angles[1]*M_RAD2DEG + 360.0),360.0);
     // float yaw = fmod((_angles[2]*M_RAD2DEG + 360.0),360.0);

     // if(verbose){
     //      printf("IMU DATA: \r\n");
     //      printf("       Accelerations (m/s^2): %.4f        %.4f      %.4f\r\n", imu->accel[0], imu->accel[1], imu->accel[2]);
     //      printf("       Angular Velocities (rad/sec): %.4f        %.4f      %.4f\r\n", imu->gyro[0], imu->gyro[1], imu->gyro[2]);
     //      printf("       Magnetometer (μT): %.4f        %.4f      %.4f\r\n", imu->mag[0], imu->mag[1],imu-> mag[2]);
     //      printf("       Fused Euler Angles (deg): %.4f        %.4f      %.4f\r\n", roll,pitch,yaw);
     //      printf(" ===================================================== \r\n");
     // }
}

int BNO055_I2C_Ros::run(bool verbose){
     cout << "Looping..." << endl;

     while(ros::ok()){
		this->update(verbose);
          ros::spinOnce();
          _loop_rate->sleep();
     }

     return 0;
}

#include <iostream>
#include <pigpiod_if2.h>
#include "swanson_sensors/bno055_i2c_ros.h"

using namespace std;

/** SECTION:
     CONSTRUCTOR & DECONSTRUCTOR
*/
BNO055_I2C_Ros::BNO055_I2C_Ros(std::string prefix, ros::NodeHandle nh, ros::NodeHandle _nh, int* pih, int mux_channel, TCA9548A* _mux) : m_nh(nh), p_nh(_nh){
     // Declare constants
	_count = 0;
	int pi;
	int bus = 1;
	int i2cAddr = 0x28;
	bool verbose = false;
	int update_rate = 20;
	std::string imu_topic = "/data";
	std::string angle_topic = "/angles";

	p_nh.getParam("i2c_bus",bus);
	p_nh.getParam("i2c_addr",i2cAddr);

	p_nh.getParam("imu_topic",imu_topic);
	p_nh.getParam("angle_topic",angle_topic);
	p_nh.getParam("update_rate",update_rate);
	p_nh.getParam("verbose",verbose);
	imu_topic = prefix + "/" + imu_topic;
	angle_topic = prefix + "/" + angle_topic;

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
	this->imu = new BNO055_I2C(pi, bus, i2cAddr,mux_channel,_mux);
	int err = imu->startup(verbose);
	if(err < 0){
		printf("[ERROR] Could not initialize Imu. Error Code = %d\r\n", err);
		exit(0);
	}

	// imu_pub = m_nh.advertise<sensor_msgs::Imu>(imu_topic, 1000);
	angle_pub = m_nh.advertise<geometry_msgs::Vector3>(angle_topic, 1000);
	// pose_pub = m_nh.advertise<geometry_msgs::Pose>(pose_topic, 1000);

	_reset_service = m_nh.advertiseService("reset_imu", &BNO055_I2C_Ros::reset_imu, this);
     _start_service = m_nh.advertiseService("start_imu", &BNO055_I2C_Ros::start_imu, this);

	_loop_rate = new ros::Rate(update_rate);
	usleep(2 * 1000000);
}

BNO055_I2C_Ros::BNO055_I2C_Ros(ros::NodeHandle nh, std::string prefix, int* pih, \
	TCA9548A* _mux, int bus, int i2cAddr, int mux_channel, std::string imu_topic, \
	std::string angle_topic, int update_rate, bool verbose) : m_nh(nh){
     // Declare constants
	_count = 0;
	int pi;

	imu_topic = prefix + "/" + imu_topic;
	angle_topic = prefix + "/" + angle_topic;

     printf("BNO-055 Parameters:\r\n");
     printf(" ----------------- \r\n");
     printf("  addr:    0x%02X\r\n", i2cAddr);
     printf("  bus:     %d\r\n", bus);
     printf("  channel: %d\r\n", mux_channel);
     printf("  imu_topic:   %s\r\n", imu_topic.c_str());
     printf("  angle_topic: %s\r\n", angle_topic.c_str());
     printf("  update_rate: %d\r\n", update_rate);


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
	this->imu = new BNO055_I2C(pi, bus, i2cAddr,mux_channel,_mux);
	int err = imu->startup(verbose);
	if(err < 0){
		printf("[ERROR] Could not initialize Imu. Error Code = %d\r\n", err);
		exit(0);
	}

	// imu_pub = m_nh.advertise<sensor_msgs::Imu>(imu_topic, 1000);
	angle_pub = m_nh.advertise<geometry_msgs::Vector3>(angle_topic, 1000);
	// pose_pub = m_nh.advertise<geometry_msgs::Pose>(pose_topic, 1000);

	_reset_service = m_nh.advertiseService("reset_imu", &BNO055_I2C_Ros::reset_imu, this);
     _start_service = m_nh.advertiseService("start_imu", &BNO055_I2C_Ros::start_imu, this);

	_loop_rate = new ros::Rate(update_rate);
	usleep(2 * 1000000);
}

BNO055_I2C_Ros::~BNO055_I2C_Ros(){
	delete this->imu;
}


void BNO055_I2C_Ros::update(bool verbose){
	_count++;
	ros::Time curTime = ros::Time::now();

	/** Get Euler Angles */
	float _angles[3];
	imu->get_euler(&_angles[0]);
	if(verbose) printf("Euler Angles: %f, %f, %f\r\n", _angles[0], _angles[1] , _angles[2]);
	geometry_msgs::Vector3 angMsg;
     angMsg.x = _angles[0];
     angMsg.y = _angles[1];
     angMsg.z = _angles[2];
     angle_pub.publish(angMsg);

	/** Get Magnetometer */
	// float _mags[3];
	// imu->get_magnetometer(&_mags[0]);
	// if(verbose) printf("Magnetic Fields: %f, %f, %f\r\n", _mags[0], _mags[1] , _mags[2]);
	// sensor_msgs::MagneticField magMsg;
	// magMsg.header.stamp = curTime;
	// magMsg.header.seq = _count;
	// magMsg.magnetic_field.x = _mags[0];
	// magMsg.magnetic_field.y = _mags[1];
	// magMsg.magnetic_field.z = _mags[2];
	// mag_pub.publish(magMsg);

	/** Get Temperature */
	// float _temp = imu->get_temperature();
	// if(verbose) printf("Temperature: %f\r\n", _temp);
	// sensor_msgs::Temperature tempMsg;
	// tempMsg.header.stamp = curTime;
	// tempMsg.header.seq = _count;
	// tempMsg.temperature = _temp;
	// temp_pub.publish(tempMsg);

	/** Get Accelerometer */
	// float _accels[3];
	// imu->get_accelerometer(&_accel[0]);
	// if(verbose) printf("Accelerations: %f, %f, %f\r\n", _accels[0], _accels[1] , _accels[2]);
	//
	// /** Get Gyroscope */
	// float _gyros[3];
	// imu->get_gyroscope(&_gyros[0]);
	// if(verbose) printf("Angular Velocities: %f, %f, %f\r\n", _gyros[0], _gyros[1] , _gyros[2]);
	//
	// /** Get Quaternions */
	// float _quats[4];
	// imu->get_quaternions(&_quats[0]);
	// if(verbose) printf("Quaternions: %f, %f, %f, %f\r\n", _quats[0], _quats[1] , _quats[2], _quats[3]);
	//
     // sensor_msgs::Imu imuMsg;
     // imuMsg.header.stamp = curTime;
	// imuMsg.header.seq = _count;
	// imuMsg.orientation.x = _quats[1];
	// imuMsg.orientation.y = _quats[2];
	// imuMsg.orientation.z = _quats[3];
	// imuMsg.orientation.w = _quats[0];
	// imuMsg.angular_velocity.x = _gyros[0];
	// imuMsg.angular_velocity.y = _gyros[1];
	// imuMsg.angular_velocity.z = _gyros[2];
	// imuMsg.linear_acceleration.x = _accels[0];
	// imuMsg.linear_acceleration.y = _accels[1];
	// imuMsg.linear_acceleration.z = _accels[2];
     // imu_pub.publish(imuMsg);

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

bool BNO055_I2C_Ros::reset_imu(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	ROS_INFO("[INFO] BNO055_I2C_Ros::reset_imu() --- Resetting IMU...");
	imu->reset();
	return true;
}

bool BNO055_I2C_Ros::start_imu(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	ROS_INFO("[INFO] BNO055_I2C_Ros::start_imu() --- Starting IMU...");
	int err = imu->startup();
	if(err < 0){
		printf("[ERROR] Could not initialize Imu. Error Code = %d\r\n", err);
		return false;
	}
	return true;
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

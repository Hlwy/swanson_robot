#include <iostream>
#include "swanson_sensors/rtimu_ros.h"

using namespace std;

/** SECTION:
     CONSTRUCTOR & DECONSTRUCTOR
*/
RtImuRos::RtImuRos(ros::NodeHandle nh, ros::NodeHandle _nh) : m_nh(nh), p_nh(_nh){

     // Declare constants
	_count = 0;

	std::string root_path = "/home/pi/devel/robo-commander/config/sensors";
	std::string config_file = "mpu9250";

	std::string prefix;
	std::string imu_topic = "imu/data";
	std::string pose_topic = "imu/pose";

	p_nh.getParam("root_path",root_path);
	p_nh.getParam("config_file",config_file);

	p_nh.getParam("prefix",prefix);
	p_nh.getParam("imu_topic",imu_topic);
	p_nh.getParam("pose_topic",pose_topic);

	/** Initialize Dual RoboClaws */
	this->imu = new GenericRTIMU();
	int err = imu->init(root_path, config_file);
	if(err < 0){
		printf("[ERROR] Could not establish RTIMU Imu using specified configuration file at \'%s/%s\'. Error Code = %d\r\n",root_path.c_str(),config_file.c_str(), err);
		exit(0);
	}

	imu_pub = m_nh.advertise<sensor_msgs::Imu>(imu_topic, 1000);
	pose_pub = m_nh.advertise<geometry_msgs::Pose>(pose_topic, 1000);

	usleep(2 * 1000000);
}

RtImuRos::~RtImuRos(){
	delete this->imu;
}


void RtImuRos::update(bool verbose){
	_count++;
     imu->update();

     int dt_us = imu->get_update_period();
     this->dt = (float) dt_us / 1000000.0;

     ros::Time curTime = ros::Time::now();

     sensor_msgs::Imu imuMsg;
     imuMsg.header.stamp = curTime;
	imuMsg.header.seq = _count;
	imuMsg.orientation.x = imu->quats[0];
	imuMsg.orientation.y = imu->quats[1];
	imuMsg.orientation.z = imu->quats[2];
	imuMsg.orientation.w = imu->quats[3];
	imuMsg.angular_velocity.x = imu->gyro[0];
	imuMsg.angular_velocity.y = imu->gyro[1];
	imuMsg.angular_velocity.z = imu->gyro[2];
	imuMsg.linear_acceleration.x = imu->accel[0];
	imuMsg.linear_acceleration.y = imu->accel[1];
	imuMsg.linear_acceleration.z = imu->accel[2];
     imu_pub.publish(imuMsg);

     geometry_msgs::Pose poseMsg;
     poseMsg.orientation = imuMsg.orientation;
     pose_pub.publish(poseMsg);

     float roll = fmod((imu->euler[0]*M_RAD2DEG + 360.0),360.0);
     float pitch = fmod((imu->euler[1]*M_RAD2DEG + 360.0),360.0);
     float yaw = fmod((imu->euler[2]*M_RAD2DEG + 360.0),360.0);

     if(verbose){
          printf("IMU DATA: \r\n");
          printf("       Accelerations (m/s^2): %.4f        %.4f      %.4f\r\n", imu->accel[0], imu->accel[1], imu->accel[2]);
          printf("       Angular Velocities (rad/sec): %.4f        %.4f      %.4f\r\n", imu->gyro[0], imu->gyro[1], imu->gyro[2]);
          printf("       Magnetometer (μT): %.4f        %.4f      %.4f\r\n", imu->mag[0], imu->mag[1],imu-> mag[2]);
          printf("       Fused Euler Angles (deg): %.4f        %.4f      %.4f\r\n", roll,pitch,yaw);
          printf(" ===================================================== \r\n");
     }
}

int RtImuRos::run(bool verbose){
     cout << "Looping..." << endl;

     while(ros::ok()){
		this->update(verbose);
          ros::spinOnce();
          ros::Duration(this->dt).sleep();
     }

     return 0;
}

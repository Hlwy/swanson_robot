#include <iostream>
#include "swanson_sensors/rtimu_ros.h"

using namespace std;

/** SECTION:
	CONSTRUCTOR & DECONSTRUCTOR
*/
RtImuRos::RtImuRos(ros::NodeHandle nh, ros::NodeHandle _nh) : m_nh(nh), p_nh(_nh){
	/** Declare constants */
	_count = 0;

	/** Robot Hardware Interface */
	std::string root_path = "/home/pi/devel/robo-commander/config/sensors";
	std::string config_file = "mpu9250";
	p_nh.getParam("root_path",root_path);
	p_nh.getParam("config_file",config_file);

	/** ROS Topic Config */
	std::string imu_topic = "imu/data/raw";
	std::string mag_topic = "imu/mag";
	std::string pose_topic = "imu/pose/raw";
	p_nh.getParam("imu_topic",imu_topic);
	p_nh.getParam("mag_topic",mag_topic);
	p_nh.getParam("pose_topic",pose_topic);

	/** ROS tf Config */
	std::string prefix;
	std::string imu_tf_frame = "imu_link";
	bool flag_pub_tfs = false;
	p_nh.getParam("tf_prefix",prefix);
	p_nh.getParam("imu_frame_id",imu_tf_frame);
	p_nh.getParam("publish_tf",flag_pub_tfs);
	this->_tf_prefix = prefix;
	this->_tf_imu = imu_tf_frame;
	this->_publishTf = flag_pub_tfs;

	/** Initialize IMU Sensor */
	this->imu = new GenericRTIMU();
	int err = imu->init(root_path, config_file);
	if(err < 0){
		printf("[ERROR] Could not establish RTIMU Imu using specified configuration file at \'%s/%s\'. Error Code = %d\r\n",root_path.c_str(),config_file.c_str(), err);
		exit(0);
	}

	/** Pre-initialize ROS msgs */
	this->init_rosmsgs();

	/** Initialize ROS Objects */
	imu_pub = m_nh.advertise<sensor_msgs::Imu>(imu_topic, 1000);
	mag_pub = m_nh.advertise<sensor_msgs::MagneticField>(mag_topic, 1000);
	pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 1000);
	// usleep(2 * 1000000);
}

RtImuRos::~RtImuRos(){
	delete this->imu;
}

void RtImuRos::init_rosmsgs(){
	this->imuMsg.header.seq = 0;
	this->imuMsg.header.stamp = ros::Time::now();
	this->imuMsg.header.frame_id = this->_tf_imu;
	this->imuMsg.linear_acceleration_covariance[0] = 0.04;
	this->imuMsg.linear_acceleration_covariance[1] = 0;
	this->imuMsg.linear_acceleration_covariance[2] = 0;
	this->imuMsg.linear_acceleration_covariance[3] = 0;
	this->imuMsg.linear_acceleration_covariance[4] = 0.04;
	this->imuMsg.linear_acceleration_covariance[5] = 0;
	this->imuMsg.linear_acceleration_covariance[6] = 0;
	this->imuMsg.linear_acceleration_covariance[7] = 0;
	this->imuMsg.linear_acceleration_covariance[8] = 0.04;

	this->imuMsg.angular_velocity_covariance[0] = 0.02;
	this->imuMsg.angular_velocity_covariance[1] = 0;
	this->imuMsg.angular_velocity_covariance[2] = 0;
	this->imuMsg.angular_velocity_covariance[3] = 0;
	this->imuMsg.angular_velocity_covariance[4] = 0.02;
	this->imuMsg.angular_velocity_covariance[5] = 0;
	this->imuMsg.angular_velocity_covariance[6] = 0;
	this->imuMsg.angular_velocity_covariance[7] = 0;
	this->imuMsg.angular_velocity_covariance[8] = 0.02;

	this->imuMsg.orientation_covariance[0] = 0.0025;
	this->imuMsg.orientation_covariance[1] = 0;
	this->imuMsg.orientation_covariance[2] = 0;
	this->imuMsg.orientation_covariance[3] = 0;
	this->imuMsg.orientation_covariance[4] = 0.0025;
	this->imuMsg.orientation_covariance[5] = 0;
	this->imuMsg.orientation_covariance[6] = 0;
	this->imuMsg.orientation_covariance[7] = 0;
	this->imuMsg.orientation_covariance[8] = 0.0025;

	this->magMsg.header = this->imuMsg.header;
	this->magMsg.magnetic_field_covariance[0] = 0;
	this->magMsg.magnetic_field_covariance[1] = 0;
	this->magMsg.magnetic_field_covariance[2] = 0;
	this->magMsg.magnetic_field_covariance[3] = 0;
	this->magMsg.magnetic_field_covariance[4] = 0;
	this->magMsg.magnetic_field_covariance[5] = 0;
	this->magMsg.magnetic_field_covariance[6] = 0;
	this->magMsg.magnetic_field_covariance[7] = 0;
	this->magMsg.magnetic_field_covariance[8] = 0;

	this->poseMsg.header = this->imuMsg.header;
	this->poseMsg.pose.position.x = 0.0;
	this->poseMsg.pose.position.y = 0.0;
	this->poseMsg.pose.position.z = 0.0;
}

void RtImuRos::update(bool verbose){
	_count++;
	imu->update();

	int dt_us = imu->get_update_period();
	this->dt = (float) dt_us / 1000000.0;
	ros::Time curTime = ros::Time::now();

	this->imuMsg.header.stamp = curTime;
	this->imuMsg.header.seq = _count;
	this->imuMsg.orientation.x = this->imu->quats[0];
	this->imuMsg.orientation.y = this->imu->quats[1];
	this->imuMsg.orientation.z = this->imu->quats[2];
	this->imuMsg.orientation.w = this->imu->quats[3];
	this->imuMsg.angular_velocity.x = this->imu->gyro[0];
	this->imuMsg.angular_velocity.y = this->imu->gyro[1];
	this->imuMsg.angular_velocity.z = this->imu->gyro[2];
	this->imuMsg.linear_acceleration.x = this->imu->accel[0];
	this->imuMsg.linear_acceleration.y = this->imu->accel[1];
	this->imuMsg.linear_acceleration.z = this->imu->accel[2];
	this->imu_pub.publish(this->imuMsg);

	this->magMsg.magnetic_field.x = this->imu->mag[0];
	this->magMsg.magnetic_field.y = this->imu->mag[1];
	this->magMsg.magnetic_field.z = this->imu->mag[2];
	this->mag_pub.publish(this->magMsg);

	this->poseMsg.header = this->imuMsg.header;
	this->poseMsg.pose.orientation = this->imuMsg.orientation;
	this->pose_pub.publish(this->poseMsg);

	float roll = fmod((this->imu->euler[0]*M_RAD2DEG + 360.0),360.0);
	float pitch = fmod((this->imu->euler[1]*M_RAD2DEG + 360.0),360.0);
	float yaw = fmod((this->imu->euler[2]*M_RAD2DEG + 360.0),360.0);

	if(verbose){
		printf("IMU DATA: \r\n");
		printf("       Accelerations (m/s^2): %.4f        %.4f      %.4f\r\n", this->imu->accel[0], this->imu->accel[1], this->imu->accel[2]);
		printf("       Angular Velocities (rad/sec): %.4f        %.4f      %.4f\r\n", this->imu->gyro[0], this->imu->gyro[1], this->imu->gyro[2]);
		printf("       Magnetometer (μT): %.4f        %.4f      %.4f\r\n", this->imu->mag[0], this->imu->mag[1],this->imu-> mag[2]);
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

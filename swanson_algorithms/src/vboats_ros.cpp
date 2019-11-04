#include <iostream>
#include "swanson_algorithms/vboats_ros.h"

using namespace std;

/** SECTION:
     CONSTRUCTOR & DECONSTRUCTOR
*/
VboatsRos::VboatsRos(ros::NodeHandle nh, ros::NodeHandle _nh) : m_nh(nh), p_nh(_nh),
	_cam_thread(), _stop_threads(false), _thread_started(false)
{

     // Declare constants
	_count = 0;

	int depth_fps = 60;
	int update_rate = 90;
	int depth_height = 480;
	int depth_width = 848;
	int color_height = 480;
	int color_width = 848;

	std::string prefix;
	std::string imu_topic = "imu/data";
	std::string pose_topic = "imu/pose";

	p_nh.getParam("fps",depth_fps);
	p_nh.getParam("depth_height",depth_height);
	p_nh.getParam("depth_width",depth_width);
	p_nh.getParam("color_height",color_height);
	p_nh.getParam("color_width",color_width);
	p_nh.getParam("update_rate",update_rate);

	p_nh.getParam("prefix",prefix);
	p_nh.getParam("imu_topic",imu_topic);
	p_nh.getParam("pose_topic",pose_topic);

	/** Initialize Dual RoboClaws */
	int rgb_resolution[2] = {color_width, color_height};
	int depth_resolution[2] = {depth_width, depth_height};
	cam = new CameraD415(60, rgb_resolution, depth_fps, depth_resolution);
	cam->enable_alignment();
	cam->enable_filters();
	cam->start_thread();

	// int err = imu->init(root_path, config_file);
	// if(err < 0){
	// 	printf("[ERROR] Could not establish RTIMU Imu using specified configuration file at \'%s/%s\'. Error Code = %d\r\n",root_path.c_str(),config_file.c_str(), err);
	// 	exit(0);
	// }
	//
	// imu_pub = m_nh.advertise<sensor_msgs::Imu>(imu_topic, 1000);
	// pose_pub = m_nh.advertise<geometry_msgs::Pose>(pose_topic, 1000);
	this->_loop_rate = new ros::Rate(update_rate);
	// usleep(2 * 1000000);
}

VboatsRos::~VboatsRos(){
	this->stop();
	delete this->_loop_rate;
	delete this->cam;
}

void VboatsRos::cameraThreadFunction(){
	this->_thread_started = true;
	printf("[INFO] VboatsRos::cameraThreadFunction() ---- Starting loop...\r\n");
	int count = 0;
	int err = 0;
	while(!this->_stop_threads){
		cv::Mat depth, rgb;
		double cvtGain, cvtRatio;
		_lock.lock();
		err = cam->get_processed_queued_images(&rgb, &depth);
		if(err >= 0){
			this->_rgb = rgb.clone();
			this->_depth = depth.clone();
			this->_disparity = this->cam->convert_to_disparity(depth,&cvtGain, &cvtRatio);
			count++;
			printf("%d images collected.\r\n", count);
		}
		_lock.unlock();
	}
	printf("[INFO] VboatsRos::cameraThreadFunction() ---- Exiting loop...\r\n");
	this->_thread_started = false;
}

void VboatsRos::stop(){
	this->_stop_threads = true;
	if(this->_thread_started){
          if(this->_cam_thread.joinable()){ this->_cam_thread.join(); }
     }
}

void VboatsRos::start(){
	this->_stop_threads = false;
     _cam_thread = std::thread(&VboatsRos::cameraThreadFunction,this);
}

void VboatsRos::update(bool verbose){



     // int dt_us = imu->get_update_period();
     // this->dt = (float) dt_us / 1000000.0;
	//
     // ros::Time curTime = ros::Time::now();
	//
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
	//
     // geometry_msgs::Pose poseMsg;
     // poseMsg.orientation = imuMsg.orientation;
     // pose_pub.publish(poseMsg);
	//
     // float roll = fmod((imu->euler[0]*M_RAD2DEG + 360.0),360.0);
     // float pitch = fmod((imu->euler[1]*M_RAD2DEG + 360.0),360.0);
     // float yaw = fmod((imu->euler[2]*M_RAD2DEG + 360.0),360.0);
	//
     // if(verbose){
     //      printf("IMU DATA: \r\n");
     //      printf("       Accelerations (m/s^2): %.4f        %.4f      %.4f\r\n", imu->accel[0], imu->accel[1], imu->accel[2]);
     //      printf("       Angular Velocities (rad/sec): %.4f        %.4f      %.4f\r\n", imu->gyro[0], imu->gyro[1], imu->gyro[2]);
     //      printf("       Magnetometer (μT): %.4f        %.4f      %.4f\r\n", imu->mag[0], imu->mag[1],imu-> mag[2]);
     //      printf("       Fused Euler Angles (deg): %.4f        %.4f      %.4f\r\n", roll,pitch,yaw);
     //      printf(" ===================================================== \r\n");
     // }
}

int VboatsRos::run(bool verbose){
	this->start();
	usleep(1 * 1000000);
	cout << "Looping..." << endl;
	int count = 0;
	cv::Mat depth, rgb;
     while(ros::ok()){
		_lock.lock();
		depth = this->_depth.clone();
		rgb = this->_rgb.clone();
		_lock.unlock();
		cv::imshow("RGB", rgb);
		cv::imshow("Depth", depth);
	     // printf("%d images collected.\r\n", count);
		// this->update(verbose);
          // ros::spinOnce();
          // this->_loop_rate->sleep();
		cv::waitKey(0);
     }

     return 0;
}

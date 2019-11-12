#include <iostream>
#include "swanson_algorithms/vboats_ros.h"

using namespace std;

/** SECTION:
     CONSTRUCTOR & DECONSTRUCTOR
*/
VboatsRos::VboatsRos(ros::NodeHandle nh, ros::NodeHandle _nh) : m_nh(nh), p_nh(_nh), _it(nh),
	_cam_thread(), _stop_threads(false), _thread_started(false)
{
	std::string ns = m_nh.getNamespace();
	// Initialize internal constants
	_count = 0;

	/** Flag Configuration */
	bool flag_gen_pc = false;
	bool flag_publish_tf = true;
	bool flag_publish_obstacles_img = true;
	bool flag_use_tf_prefix = true;
	bool flag_get_aligned = false;
	bool flag_use_float_depth = true;
	p_nh.getParam("generate_pointcload",flag_gen_pc);
	p_nh.getParam("publish_tf",flag_publish_tf);
	p_nh.getParam("publish_obstacles_image",flag_publish_obstacles_img);
	p_nh.getParam("use_tf_prefix",flag_use_tf_prefix);
	p_nh.getParam("use_aligned",flag_get_aligned);
	p_nh.getParam("use_float_depth",flag_use_float_depth);

	this->_use_float_depth = flag_use_float_depth;
	this->_get_aligned = flag_get_aligned;
	this->_publish_tf = flag_publish_tf;
	this->_publish_obs_display = flag_publish_obstacles_img;

	/** Camera Parameter Configuration */
	int depth_fps = 90;
	int color_fps = 60;
	int depth_height = 480;
	int depth_width = 848;
	int color_height = 480;
	int color_width = 848;
	int update_rate = 90;

	p_nh.getParam("depth_fps",depth_fps);
	p_nh.getParam("depth_height",depth_height);
	p_nh.getParam("depth_width",depth_width);
	p_nh.getParam("color_fps",color_fps);
	p_nh.getParam("color_height",color_height);
	p_nh.getParam("color_width",color_width);
	p_nh.getParam("update_rate",update_rate);

	/** ROS Topics Configuration */
	std::string camera_name = "camera";
	std::string depth_info_topic = "/depth/camera_info";
	std::string depth_image_topic = "/depth/image_rect_raw";
	std::string color_info_topic = "/color/camera_info";
	std::string color_image_topic = "/color/image_raw";
	std::string disparity_image_topic = "/disparity/image_raw";
	std::string obstacles_image_topic = "/vboats/obstacles/image_raw";
	std::string detected_obstacles_info_topic = "/vboats/obstacles/data";
	p_nh.getParam("camera_name",camera_name);
	p_nh.getParam("depth_info_topic",depth_info_topic);
	p_nh.getParam("depth_image_topic",depth_image_topic);
	p_nh.getParam("color_info_topic",color_info_topic);
	p_nh.getParam("color_image_topic",color_image_topic);
	p_nh.getParam("disparity_image_topic",disparity_image_topic);
	p_nh.getParam("obstacles_image_topic",obstacles_image_topic);
	p_nh.getParam("obstacles_info_topic",detected_obstacles_info_topic);

	std::string _depth_info_topic = ns + camera_name + depth_info_topic;
	std::string _depth_image_topic = ns + camera_name + depth_image_topic;
	std::string _color_info_topic = ns + camera_name + color_info_topic;
	std::string _color_image_topic = ns + camera_name + color_image_topic;
	std::string _disparity_image_topic = ns + disparity_image_topic;
	std::string _obstacles_image_topic = ns + obstacles_image_topic;
	std::string _detected_obstacles_info_topic = ns + detected_obstacles_info_topic;

	this->_ns = ns;
	this->_camera_name = camera_name;

	/** ROS tf frames Configuration */
	std::string tf_prefix = "/";
	std::string parent_tf = "base_link";
	std::string cam_base_tf = "camera_link";
	std::string rgb_base_tf = "camera_color_frame";
	std::string rgb_optical_tf = "camera_color_optical_frame";
	std::string depth_base_tf = "camera_depth_frame";
	std::string depth_optical_tf = "camera_depth_optical_frame";
	std::string aligned_base_tf = "camera_aligned_depth_to_color_frame";
	p_nh.getParam("tf_prefix",tf_prefix);
	p_nh.getParam("parent_frame",parent_tf);
	p_nh.getParam("camera_base_frame",cam_base_tf);
	p_nh.getParam("rgb_base_frame",rgb_base_tf);
	p_nh.getParam("rgb_optical_frame",rgb_optical_tf);
	p_nh.getParam("depth_base_frame",depth_base_tf);
	p_nh.getParam("depth_optical_frame",depth_optical_tf);
	p_nh.getParam("aligned_base_frame",aligned_base_tf);

	std::string tmp_parent_tf = parent_tf;
	std::string tmp_cam_base_tf = cam_base_tf;
	std::string tmp_rgb_base_tf = rgb_base_tf;
	std::string tmp_rgb_optical_tf = rgb_optical_tf;
	std::string tmp_depth_base_tf = depth_base_tf;
	std::string tmp_depth_optical_tf = depth_optical_tf;
	std::string tmp_aligned_base_tf = aligned_base_tf;

	if(flag_use_tf_prefix){
		tmp_parent_tf = tf_prefix + _parent_tf;
		tmp_cam_base_tf = tf_prefix + _cam_base_tf;
		tmp_rgb_base_tf = tf_prefix + _rgb_base_tf;
		tmp_rgb_optical_tf = tf_prefix + _rgb_optical_tf;
		tmp_depth_base_tf = tf_prefix + _depth_base_tf;
		tmp_depth_optical_tf = tf_prefix + _depth_optical_tf;
		tmp_aligned_base_tf = tf_prefix + _aligned_base_tf;
	}

	this->_tf_prefix = tf_prefix;
	this->_parent_tf = tmp_parent_tf;
	this->_cam_base_tf = tmp_cam_base_tf;
	this->_rgb_base_tf = tmp_rgb_base_tf;
	this->_rgb_optical_tf = tmp_rgb_optical_tf;
	this->_depth_base_tf = tmp_depth_base_tf;
	this->_depth_optical_tf = tmp_depth_optical_tf;
	this->_aligned_base_tf = tmp_aligned_base_tf;

	/** Initialize D415 Camera */
	int rgb_resolution[2] = {color_width, color_height};
	int depth_resolution[2] = {depth_width, depth_height};
	cam = new CameraD415(color_fps, rgb_resolution, depth_fps, depth_resolution);

	cam->get_intrinsics(RS2_STREAM_COLOR, &_Krgb, &_Prgb, true);
	cam->get_intrinsics(RS2_STREAM_DEPTH, &_Kdepth, &_Pdepth, true);
	this->_dscale = cam->get_depth_scale(false);
	this->_baseline = cam->get_baseline(false);

	float fxc = _Krgb.at<float>(0);
	float fyc = _Krgb.at<float>(4);
	float pxc = _Krgb.at<float>(2);
	float pyc = _Krgb.at<float>(5);
	if(fxc == 0) this->_fxc = 1.0;
	else this->_fxc = fxc;
	if(fyc == 0) this->_fyc = 1.0;
	else this->_fyc = fyc;
	if(pxc == 0) this->_pxc = 1.0;
	else this->_pxc = pxc;
	if(pyc == 0) this->_pyc = 1.0;
	else this->_pyc = pyc;

	this->_rgb_info_msg.header.frame_id = tmp_rgb_optical_tf;
	this->_rgb_info_msg.width = color_width;
	this->_rgb_info_msg.height = color_height;
	this->_rgb_info_msg.distortion_model = "plumb_bob";
	this->_rgb_info_msg.D = {0.0, 0.0, 0.0, 0.0, 0.0};
	this->_rgb_info_msg.K[0] = _Krgb.at<double>(0);
	this->_rgb_info_msg.K[4] = _Krgb.at<double>(4);
	this->_rgb_info_msg.K[2] = _Krgb.at<double>(2);
	this->_rgb_info_msg.K[5] = _Krgb.at<double>(5);
	this->_rgb_info_msg.P[0] = _Prgb.at<double>(0);
	this->_rgb_info_msg.P[5] = _Prgb.at<double>(5);
	this->_rgb_info_msg.P[2] = _Prgb.at<double>(2);
	this->_rgb_info_msg.P[6] = _Prgb.at<double>(6);

	float fxd = _Kdepth.at<float>(0);
	float fyd = _Kdepth.at<float>(4);
	float pxd = _Kdepth.at<float>(2);
	float pyd = _Kdepth.at<float>(5);
	if(fxd == 0) this->_fxd = 1.0;
	else this->_fxd = fxd;
	if(fyd == 0) this->_fyd = 1.0;
	else this->_fyd = fyd;
	if(pxd == 0) this->_pxd = 1.0;
	else this->_pxd = pxd;
	if(pyd == 0) this->_pyd = 1.0;
	else this->_pyd = pyd;

	this->_depth_info_msg.header.frame_id = tmp_depth_optical_tf;
	this->_depth_info_msg.width = depth_width;
	this->_depth_info_msg.height = depth_height;
	this->_depth_info_msg.distortion_model = "plumb_bob";
	this->_depth_info_msg.D = {0.0, 0.0, 0.0, 0.0, 0.0};
	this->_depth_info_msg.K[0] = _Kdepth.at<double>(0);
	this->_depth_info_msg.K[4] = _Kdepth.at<double>(4);
	this->_depth_info_msg.K[2] = _Kdepth.at<double>(2);
	this->_depth_info_msg.K[5] = _Kdepth.at<double>(5);
	this->_depth_info_msg.P[0] = _Pdepth.at<double>(0);
	this->_depth_info_msg.P[5] = _Pdepth.at<double>(5);
	this->_depth_info_msg.P[2] = _Pdepth.at<double>(2);
	this->_depth_info_msg.P[6] = _Pdepth.at<double>(6);

	/** Initialize VBOATS */
	this->vb = new VBOATS();

	/** Initialize ROS-Objects */
	this->_rgb_pub = _it.advertise(_color_image_topic, 1);
	this->_rgb_info_pub = m_nh.advertise<sensor_msgs::CameraInfo>(_color_info_topic, 1);
	this->_depth_pub = _it.advertise(_depth_image_topic, 1);
	this->_depth_info_pub = m_nh.advertise<sensor_msgs::CameraInfo>(_depth_info_topic, 1);
	this->_disparity_pub = _it.advertise(_disparity_image_topic, 1);
	this->_obstacles_img_pub = _it.advertise(_obstacles_image_topic, 1);
	// this->_detected_obstacle_info_pub = m_nh.advertise<geometry_msgs::Pose>(_detected_obstacles_info_topic, 100);

	this->_loop_rate = new ros::Rate(update_rate);

	printf("[INFO] VboatsRos::VboatsRos() ---- Successfully Initialized!\r\n");
	cam->enable_alignment();
	cam->enable_filters();
	cam->start_thread();
}

VboatsRos::~VboatsRos(){
	this->stop();
	delete this->_loop_rate;
	delete this->vb;
	delete this->cam;
}

void VboatsRos::cameraThreadFunction(){
	this->_thread_started = true;
	printf("[INFO] VboatsRos::cameraThreadFunction() ---- Starting loop...\r\n");
	int err = 0;
	int count = 0;
	bool debug_timing = false;
	double t = (double)cv::getTickCount();
	this->_img_count = 0;
	while(!this->_stop_threads){
		cv::Mat rgb, depth, disparity;
		double cvtGain, cvtRatio;
		err = cam->get_processed_queued_images(&rgb, &depth);
		if(err >= 0){
			this->_lock.lock();
			this->_rgb = rgb.clone();
			this->_depth = depth.clone();
			disparity = this->cam->convert_to_disparity(depth,&cvtGain, &cvtRatio);
			this->_disparity = disparity.clone();
			this->_img_count++;
			this->_lock.unlock();
			count++;
			if(debug_timing){
				double dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
				printf("[INFO] VboatsRos::cameraThreadFunction() ---- %d images collected. Current timing %.4lf ms (%.2lf Hz)\r\n", this->_img_count, dt*1000.0, (1.0/dt));
				t = (double)cv::getTickCount();
			}
		}
		// ros::spinOnce();
		// this->_loop_rate->sleep();
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
     this->_cam_thread = std::thread(&VboatsRos::cameraThreadFunction,this);
}

void VboatsRos::publish_image(const cv::Mat& image){
	ros::Time time = ros::Time::now();
	cv_bridge::CvImagePtr cv_ptr;

	cv_ptr->encoding = "bgr8";
	cv_ptr->header.stamp = time;
	cv_ptr->header.seq = this->_img_count;
	cv_ptr->header.frame_id = "/traj_output";

	cv_ptr->image = image;
	// image_pub_.publish(cv_ptr->toImageMsg());

    /**
    rgbImgMsg = self.bridge.cv2_to_imgmsg(self.rgb, "bgr8")
    if(self.use_float_depth): depthImgMsg = self.bridge.cv2_to_imgmsg(_depth*self.dscale, "32FC1")
    else: depthImgMsg = self.bridge.cv2_to_imgmsg(self.depth, "8UC1")

    rgbImgMsg.header.frame_id = self.aligned_base_tf
    depthImgMsg.header.frame_id = self.aligned_base_tf

    self.rgb_pub.publish(rgbImgMsg)
    self.depth_pub.publish(depthImgMsg)

    infoMsgRgb, infoMsgDepth = self.create_camera_info_msg()
    infoMsgRgb.header.stamp = rgbImgMsg.header.stamp
    infoMsgDepth.header.stamp = depthImgMsg.header.stamp

    self.rgb_info_pub.publish(infoMsgRgb)
    self.depth_info_pub.publish(infoMsgDepth)
    */

}

void VboatsRos::update(const cv::Mat& image, bool is_disparity, bool verbose, bool debug_timing){
	// boost::mutex::scoped_lock scoped_lock(_lock);
	double t, t1, dt;
	cv::Mat umap, vmap;
	vector<Obstacle> obs;

	if(debug_timing) t = (double)cv::getTickCount();
	this->vb->get_uv_map(image,&umap,&vmap, false, "raw");
	if(debug_timing){
		dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
		printf("[INFO] VboatsRos::update() ---- UV-Map generation took %.4lf ms (%.2lf Hz)\r\n", dt*1000.0, (1.0/dt));
	}

	// this->_lock.lock();
	// this->_umap = umap.clone();
	// this->_vmap = vmap.clone();
	// this->_lock.unlock();

	if(debug_timing) t1 = (double)cv::getTickCount();
	this->vb->pipeline_disparity(image, umap, vmap, &obs);
	// pipeline_disparity(disparity, umap, vmap, &obs, &element);
	if(debug_timing){
		dt = ((double)cv::getTickCount() - t1)/cv::getTickFrequency();
		printf("[INFO] VboatsRos::update() ---- pipeline_disparity took %.4lf ms (%.2lf Hz)\r\n", dt*1000.0, (1.0/dt));
	}

	// cv::imshow("Umap", umap);
	// cv::imshow("Vmap", vmap);

     // ros::Time curTime = ros::Time::now();

     // sensor_msgs::Imu imuMsg;
     // imuMsg.header.stamp = curTime;
	// imuMsg.header.seq = _count;
	// imuMsg.orientation.x = imu->quats[0];
     // imu_pub.publish(imuMsg);

     // geometry_msgs::Pose poseMsg;
     // poseMsg.orientation = imuMsg.orientation;
     // pose_pub.publish(poseMsg);

     // float yaw = fmod((imu->euler[2]*M_RAD2DEG + 360.0),360.0);

     // if(verbose){
     //      printf("IMU DATA: \r\n");
     //      printf("       Accelerations (m/s^2): %.4f        %.4f      %.4f\r\n", imu->accel[0], imu->accel[1], imu->accel[2]);
     //      printf("       Angular Velocities (rad/sec): %.4f        %.4f      %.4f\r\n", imu->gyro[0], imu->gyro[1], imu->gyro[2]);
     //      printf("       Magnetometer (μT): %.4f        %.4f      %.4f\r\n", imu->mag[0], imu->mag[1],imu-> mag[2]);
     //      printf("       Fused Euler Angles (deg): %.4f        %.4f      %.4f\r\n", roll,pitch,yaw);
     //      printf(" ===================================================== \r\n");
     // }
}

int VboatsRos::run(bool verbose, bool debug_timing){
	bool visualize = false;
	this->start();
	usleep(1 * 1000000);
	cout << "Looping..." << endl;
	int count = 0;
	cv::Mat rgb, depth, disparity;
	double t = (double)cv::getTickCount();
     while(ros::ok()){
		this->_lock.lock();
		// rgb = this->_rgb.clone();
		// depth = this->_depth.clone();
		disparity = this->_disparity.clone();
		this->_lock.unlock();
		this->update(disparity);
		// cvinfo(rgb,"rgb");
		// cvinfo(depth,"depth");

		if(debug_timing){
			double dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
			printf("[INFO] VboatsRos::update() ---- pipeline_disparity took %.4lf ms (%.2lf Hz)\r\n", dt*1000.0, (1.0/dt));
			t = (double)cv::getTickCount();
		}
		if(visualize){
			if(!rgb.empty()) cv::imshow("RGB", rgb);
			if(!depth.empty()) cv::imshow("Depth", depth);
			if(!disparity.empty()) cv::imshow("Disparity", disparity);
		}
          // ros::spinOnce();
          // this->_loop_rate->sleep();
		cv::waitKey(10);
     }

     return 0;
}

#include <iostream>
#include "swanson_sensors/camera_d415_ros.h"
#include <RoboCommander/utilities/cv_utils.h>

using namespace std;

/** SECTION:
     CONSTRUCTOR & DECONSTRUCTOR
*/
CameraD415Ros::CameraD415Ros(ros::NodeHandle nh, ros::NodeHandle _nh) : m_nh(nh), p_nh(_nh), _it(nh),
	_stop_threads(false), _thread_started(false), _focal(), _principle()
{
	std::string ns = m_nh.getNamespace();
	// Initialize internal constants
	_count = 0;

	/** Flag Configuration */
	bool flag_verbose_timings = false;
	bool flag_gen_pc = false;
	bool flag_publish_imgs = true;
	bool flag_publish_tf = true;
	bool flag_use_tf_prefix = true;
	bool flag_get_aligned = false;
	bool flag_use_float_depth = true;
	bool flag_use_8bit_depth = false;
	bool flag_calc_disparity = false;
	p_nh.getParam("verbose_timings",flag_verbose_timings);
	p_nh.getParam("generate_pointcload",flag_gen_pc);
	p_nh.getParam("publish_tf",flag_publish_tf);
	p_nh.getParam("publish_images",flag_publish_imgs);
	p_nh.getParam("use_tf_prefix",flag_use_tf_prefix);
	p_nh.getParam("use_aligned",flag_get_aligned);
	p_nh.getParam("use_float_depth",flag_use_float_depth);
	p_nh.getParam("use_8bit_depth",flag_use_8bit_depth);
	p_nh.getParam("calculate_disparity",flag_calc_disparity);

	this->_verbose_timings = flag_verbose_timings;
	this->_publish_images = flag_publish_imgs;
	this->_use_float_depth = flag_use_float_depth;
	this->_use_8bit_depth = flag_use_8bit_depth;
	this->_get_aligned = flag_get_aligned;
	this->_publish_tf = flag_publish_tf;
	this->_calc_disparity = flag_calc_disparity;
	/** Camera Parameter Configuration */
	int depth_fps = 90;
	int color_fps = 60;
	int depth_height = 480;
	int depth_width = 848;
	int color_height = 480;
	int color_width = 848;
	int update_rate = 30;

	p_nh.getParam("depth_fps",depth_fps);
	p_nh.getParam("depth_height",depth_height);
	p_nh.getParam("depth_width",depth_width);
	p_nh.getParam("color_fps",color_fps);
	p_nh.getParam("color_height",color_height);
	p_nh.getParam("color_width",color_width);
	p_nh.getParam("update_rate",update_rate);
	this->_update_rate = update_rate;

	/** ROS Topics Configuration */
	std::string camera_name = "camera";
	std::string depth_info_topic = "/depth/camera_info";
	std::string depth_image_topic = "/depth/image_rect_raw";
	std::string color_info_topic = "/color/camera_info";
	std::string color_image_topic = "/color/image_raw";
	std::string disparity_image_topic = "/disparity/image_raw";
	p_nh.getParam("camera_name",camera_name);
	p_nh.getParam("depth_info_topic",depth_info_topic);
	p_nh.getParam("depth_image_topic",depth_image_topic);
	p_nh.getParam("color_info_topic",color_info_topic);
	p_nh.getParam("color_image_topic",color_image_topic);
	p_nh.getParam("disparity_image_topic",disparity_image_topic);

	std::string _depth_info_topic = ns + "/" + camera_name + depth_info_topic;
	std::string _depth_image_topic = ns + "/" + camera_name + depth_image_topic;
	std::string _color_info_topic = ns + "/" + camera_name + color_info_topic;
	std::string _color_image_topic = ns + "/" + camera_name + color_image_topic;
	std::string _disparity_image_topic = ns + "/" + camera_name + disparity_image_topic;

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
		tmp_parent_tf = tf_prefix + tmp_parent_tf;
		tmp_cam_base_tf = tf_prefix + tmp_cam_base_tf;
		tmp_rgb_base_tf = tf_prefix + tmp_rgb_base_tf;
		tmp_rgb_optical_tf = tf_prefix + tmp_rgb_optical_tf;
		tmp_depth_base_tf = tf_prefix + tmp_depth_base_tf;
		tmp_depth_optical_tf = tf_prefix + tmp_depth_optical_tf;
		tmp_aligned_base_tf = tf_prefix + tmp_aligned_base_tf;
	}

	this->_tf_prefix = tf_prefix;
	this->_parent_tf = tmp_parent_tf;
	this->_cam_base_tf = tmp_cam_base_tf;
	this->_rgb_base_tf = tmp_rgb_base_tf;
	this->_rgb_optical_tf = tmp_rgb_optical_tf;
	this->_depth_base_tf = tmp_depth_base_tf;
	this->_depth_optical_tf = tmp_depth_optical_tf;
	this->_aligned_base_tf = tmp_aligned_base_tf;

	/** Pre-initialize Common variables */
	this->initTfs();
	this->_rgbImgHeader = std_msgs::Header();
	this->_depthImgHeader = std_msgs::Header();
	this->_disparityImgHeader = std_msgs::Header();

	/** Initialize D415 Camera */
	int rgb_resolution[2] = {color_width, color_height};
	int depth_resolution[2] = {depth_width, depth_height};
	this->cam = new CameraD415(color_fps, rgb_resolution, depth_fps, depth_resolution);

	this->cam->get_intrinsics(RS2_STREAM_COLOR, &_Krgb, &_Prgb, true);
	this->cam->get_intrinsics(RS2_STREAM_DEPTH, &_Kdepth, &_Pdepth, true);
	this->_dscale = this->cam->get_depth_scale(false);
	this->_baseline = this->cam->get_baseline(false);

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

	float fxd = (float)_Kdepth.at<double>(0);
	float fyd = (float)_Kdepth.at<double>(4);
	float pxd = (float)_Kdepth.at<double>(2);
	float pyd = (float)_Kdepth.at<double>(5);
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

	this->_focal[0] = this->_fxd;
	this->_focal[1] = this->_fyd;
	this->_principle[0] = this->_pxd;
	this->_principle[1] = this->_pyd;

	/** Initialize ROS-Objects */
	this->_rgb_info_pub = m_nh.advertise<sensor_msgs::CameraInfo>(_color_info_topic, 1);
	this->_depth_info_pub = m_nh.advertise<sensor_msgs::CameraInfo>(_depth_info_topic, 1);
	this->_rgb_pub = m_nh.advertise<sensor_msgs::Image>(_color_image_topic, 1);
	this->_depth_pub = m_nh.advertise<sensor_msgs::Image>(_depth_image_topic, 1);
	this->_disparity_pub = m_nh.advertise<sensor_msgs::Image>(_disparity_image_topic, 1);
	// this->_rgb_pub = _it.advertise(_color_image_topic, 1);
	// this->_depth_pub = _it.advertise(_depth_image_topic, 1);
	// this->_disparity_pub = _it.advertise(_disparity_image_topic, 1);

	this->_loop_rate = new ros::Rate(update_rate);

	printf("[INFO] CameraD415Ros::CameraD415Ros() ---- Successfully Initialized!\r\n");
	this->cam->enable_alignment();
	this->cam->enable_filters();
	this->cam->start_thread();
}

CameraD415Ros::~CameraD415Ros(){
	this->stop();
	delete this->_loop_rate;
	delete this->cam;
}

void CameraD415Ros::stop(){
	this->_stop_threads = true;
}
void CameraD415Ros::start(){
	this->_stop_threads = false;
}

void CameraD415Ros::initTfs(){
	tf::Vector3 pNull(0.0, 0.0, 0.0);
	tf::Quaternion qNull; qNull.setRPY(0.0, 0.0, 0.0);
	tf::Quaternion qOptical; qOptical.setRPY(-M_PI/2.0, 0.0, -M_PI/2.0);

	this->_tfOpticalBaseToCamBase = tf::Transform(qNull,pNull);
	this->_tfOpticalToOpticalBase = tf::Transform(qOptical,pNull);
}
void CameraD415Ros::publish_tfs(){
	ros::Time curTime = ros::Time::now();

	// printf("[INFO] Sending tf \'%s\' to \'%s\'\r\n",this->_rgb_optical_tf.c_str(),this->_rgb_base_tf.c_str());
	// printf("[INFO] Sending tf \'%s\' to \'%s\'\r\n",this->_rgb_base_tf.c_str(),this->_cam_base_tf.c_str());
	// printf("[INFO] Sending tf \'%s\' to \'%s\'\r\n",this->_depth_optical_tf.c_str(),this->_depth_base_tf.c_str());
	// printf("[INFO] Sending tf \'%s\' to \'%s\'\r\n",this->_depth_base_tf.c_str(),this->_cam_base_tf.c_str());
	// printf("[INFO] Sending tf \'%s\' to \'%s\'\r\n",this->_aligned_base_tf.c_str(),this->_cam_base_tf.c_str());
	this->_br.sendTransform(tf::StampedTransform(this->_tfOpticalToOpticalBase, curTime, this->_rgb_base_tf, this->_rgb_optical_tf));
	this->_br.sendTransform(tf::StampedTransform(this->_tfOpticalBaseToCamBase, curTime, this->_cam_base_tf, this->_rgb_base_tf));
	this->_br.sendTransform(tf::StampedTransform(this->_tfOpticalToOpticalBase, curTime, this->_depth_base_tf, this->_depth_optical_tf));
	this->_br.sendTransform(tf::StampedTransform(this->_tfOpticalBaseToCamBase, curTime, this->_cam_base_tf, this->_depth_base_tf));
	this->_br.sendTransform(tf::StampedTransform(this->_tfOpticalToOpticalBase, curTime, this->_cam_base_tf, this->_aligned_base_tf));
}
void CameraD415Ros::publish_images(cv::Mat _rgb, cv::Mat _depth, cv::Mat _disparity){
	ros::Time time = ros::Time::now();
	/** RGB Image */
	if(!_rgb.empty()){
		this->_rgbImgHeader.stamp = time;
		this->_rgbImgHeader.seq = this->_img_count;
		// this->_rgbImgHeader.frame_id = this->_rgb_optical_tf;
		this->_rgbImgHeader.frame_id = this->_aligned_base_tf;

		sensor_msgs::ImagePtr rgbImgMsg = cv_bridge::CvImage(this->_rgbImgHeader, "bgr8", _rgb).toImageMsg();
		this->_rgb_pub.publish(rgbImgMsg);
		// Camera Info
		this->_rgb_info_msg.header.stamp = time;
		this->_rgb_info_msg.header.seq = this->_img_count;
		this->_rgb_info_pub.publish(this->_rgb_info_msg);
	}

	/** Depth Image */
	if(!_depth.empty()){
		this->_depthImgHeader.stamp = time;
		this->_depthImgHeader.seq = this->_img_count;
		// this->_depthImgHeader.frame_id = this->_depth_optical_tf;
		this->_depthImgHeader.frame_id = this->_aligned_base_tf;

		sensor_msgs::ImagePtr depthImgMsg;
		if(this->_use_float_depth){
			cv::Mat tmp;
			_depth.convertTo(tmp, CV_32F);
			// tmp = tmp * this->_dscale;
			// depthImgMsg = cv_bridge::CvImage(this->_depthImgHeader, "32FC1", tmp).toImageMsg();
			depthImgMsg = cv_bridge::CvImage(this->_depthImgHeader, "32FC1", tmp * this->_dscale).toImageMsg();
		} else if(this->_use_8bit_depth && !this->_use_float_depth){
			// Convert depth image to uint8 if not already
			if(_depth.type() != CV_8UC1){
				cv::Mat depth8;
				_depth.convertTo(depth8, CV_8UC1, (255.0/65535.0));
				depthImgMsg = cv_bridge::CvImage(this->_depthImgHeader, "8UC1", depth8).toImageMsg();
			} else depthImgMsg = cv_bridge::CvImage(this->_depthImgHeader, "8UC1", _depth).toImageMsg();
		} else{
			// Convert depth image to uint16 if not already
			if(_depth.type() != CV_16UC1){
				cv::Mat depth16;
				_depth.convertTo(depth16, CV_16UC1, (65535.0/255.0));
				depthImgMsg = cv_bridge::CvImage(this->_depthImgHeader, "16UC1", depth16).toImageMsg();
			} else depthImgMsg = cv_bridge::CvImage(this->_depthImgHeader, "16UC1", _depth).toImageMsg();
		}
		this->_depth_pub.publish(depthImgMsg);

		// Publish Camera Info
		this->_depth_info_msg.header.stamp = time;
		this->_depth_info_msg.header.seq = this->_img_count;
		this->_depth_info_pub.publish(this->_depth_info_msg);
	}

	/** Disparity Image */
	if(this->_calc_disparity){
		if(!_disparity.empty()){
			this->_disparityImgHeader.stamp = time;
			this->_disparityImgHeader.seq = this->_img_count;
			this->_disparityImgHeader.frame_id = this->_depth_optical_tf;

			sensor_msgs::ImagePtr disparityImgMsg = cv_bridge::CvImage(this->_disparityImgHeader, "8UC1", _disparity).toImageMsg();
			this->_disparity_pub.publish(disparityImgMsg);
		}
	}
}

void CameraD415Ros::update(bool verbose){
	// boost::mutex::scoped_lock scoped_lock(_lock);
	cv::Mat rgb, depth;
	bool visualize = false;
	bool debug_timing = false;
	double cvtGain, cvtRatio;
	double t = (double)cv::getTickCount();
	int err = this->cam->get_processed_queued_images(&rgb, &depth);
	if(err >= 0){
		this->_lock.lock();
		this->_rgb = rgb.clone();
		this->_depth = depth.clone();
		if(this->_calc_disparity){
			cv::Mat disparity = this->cam->convert_to_disparity_test(depth,&cvtGain, &cvtRatio);
			this->_disparity = disparity.clone();
		}
		this->_img_count++;
		this->_lock.unlock();
		if(this->_publish_tf) this->publish_tfs();
		if(this->_publish_images) this->publish_images(rgb, depth, this->_disparity);
		if(debug_timing){
			double dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
			printf("[INFO] CameraD415Ros::update() ---- %d images collected. Current timing %.4lf ms (%.2lf Hz)\r\n", this->_img_count, dt*1000.0, (1.0/dt));
			t = (double)cv::getTickCount();
		}
		if(visualize){
			if(!rgb.empty()) cv::imshow("RGB", rgb);
			if(!depth.empty()) cv::imshow("Depth", depth);
			if(this->_calc_disparity)
				if(!this->_disparity.empty()) cv::imshow("Disparity", this->_disparity);
			cv::waitKey(10);
		}
	}
}

int CameraD415Ros::run(bool verbose){
	this->start();
	usleep(1 * 1000000);
	cout << "Looping..." << endl;
	double t = (double)cv::getTickCount();
     while(ros::ok()){
		this->update();
		if(this->_verbose_timings){
			double dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
			printf("[INFO] CameraD415Ros::run() ---- pipeline_disparity took %.4lf ms (%.2lf Hz)\r\n", dt*1000.0, (1.0/dt));
			t = (double)cv::getTickCount();
		}
          ros::spinOnce();
          this->_loop_rate->sleep();
     }

     return 0;
}

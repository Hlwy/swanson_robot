#include <iostream>
#include "swanson_algorithms/vboats_ros.h"

#include <RoboCommander/algorithms/vboats/uvmap_utils.h>
#include <RoboCommander/utilities/cv_utils.h>
#include <swanson_msgs/VboatsObstacle.h>
#include <swanson_msgs/VboatsObstacles.h>

using namespace std;

template<typename dtype>
struct ForEachDepthConverter{
     dtype m_gain;
     ForEachDepthConverter(dtype gain){ m_gain = gain; }

     void operator()(dtype& pixel, const int * idx) const {
          if(pixel != 0.0){ pixel = m_gain / pixel; }
     }
};

template<typename dtype>
struct ForEachGndMaskGenerator{
     int* m_mins;
     int* m_maxs;
	ForEachGndMaskGenerator(int* mins, int* maxs, int nelems){
		m_mins = new int[nelems];
		m_maxs = new int[nelems];
		m_mins = mins;
		m_maxs = maxs;
     }
     void operator()(dtype& pixel, const int * idx) const {
		int tmpMin = m_mins[idx[1]];
		int tmpMax = m_maxs[idx[1]];
		// if(pixel != 0.0){
		if((tmpMin >= 0) && (tmpMax >= 0)){
			if( (pixel >= tmpMin) && (pixel <= tmpMax) ){
				// m_mask.at<uchar>(idx[1], idx[0]) = 0;
				pixel = 0;
			}
		}
     }
};

template<typename dtype>
struct ForEachPclOperator{
     dtype m_gain;
     ForEachPclOperator(dtype gain){
          m_gain = gain;
     }
     void operator()(dtype& pixel, const int * idx) const {
          if(pixel != 0.0){
               pixel = m_gain / pixel;
          }
     }
};

/** SECTION:
     CONSTRUCTOR & DECONSTRUCTOR
*/
VboatsRos::VboatsRos(ros::NodeHandle nh, ros::NodeHandle _nh) : m_nh(nh), p_nh(_nh), _it(nh), _focal(), _principle(){
	// Initialize internal constants
	_count = 0;
	_img_count = 0;
	_info_count = 0;
	_recvd_cam_info = false;
	_flag_depth_based = true;
	this->_ns = m_nh.getNamespace();

	/** Flag Configuration */
	bool flag_verbose_obstacles = false;
	bool flag_verbose_timings = false;
	bool flag_use_tf_prefix = false;
	bool flag_publish_imgs = true;
	bool publish_aux_images = false;
	bool visualize_imgs = false;
	bool disparity_based = false;
	bool flag_detect_obstacles = false;
	bool flag_filter_ground = true;
	bool flag_filter_cloud = false;
	bool publish_cloud = false;
	bool publish_fitered_cloud = false;
	float max_obs_height = 1.0;
	int update_rate = 30;

	p_nh.getParam("verbose_obstacles",flag_verbose_obstacles);
	p_nh.getParam("verbose_timings",flag_verbose_timings);
	p_nh.getParam("publish_images",flag_publish_imgs);
	p_nh.getParam("publish_aux_images",publish_aux_images);
	p_nh.getParam("show_images",visualize_imgs);
	p_nh.getParam("update_rate",update_rate);
	p_nh.getParam("use_disparity",disparity_based);

	p_nh.getParam("detect_obstacles",flag_detect_obstacles);
	p_nh.getParam("filter_ground",flag_filter_ground);
	p_nh.getParam("filtered_cloud",flag_filter_cloud);
	p_nh.getParam("publish_cloud",publish_cloud);
	p_nh.getParam("publish_filtered_cloud",publish_fitered_cloud);
	p_nh.getParam("max_obstacle_height",max_obs_height);

	this->_verbose_obstacles = flag_verbose_obstacles;
	this->_verbose_timings = flag_verbose_timings;
	this->_publish_images = flag_publish_imgs;
	this->_publish_aux_images = publish_aux_images;
	this->_visualize_images = visualize_imgs;
	this->_update_rate = update_rate;
	this->_flag_depth_based = !disparity_based;
	this->_detect_obstacles = flag_detect_obstacles;
	this->_filter_ground = flag_filter_ground;
	this->_filter_cloud = flag_filter_cloud;
	this->_flag_pub_cloud = publish_cloud;
	this->_flag_pub_filtered_cloud = publish_fitered_cloud;
	this->_max_obstacle_height = max_obs_height;

	/** ROS Object Configuration */
	std::string depth_image_topic = "/camera/depth/image_rect_raw";
	std::string camera_info_topic = "/camera/depth/camera_info";
	std::string disparity_image_topic = "/camera/disparity/image_raw";
	std::string umap_topic = "/vboats/umap/image_raw";
	std::string vmap_topic = "/vboats/vmap/image_raw";
	std::string filtered_image_topic = "/vboats/disparity/image_filtered";
	std::string obstacles_image_topic = "/vboats/obstacles/image_raw";
	std::string detected_obstacles_info_topic = "/vboats/obstacles/data";
	std::string raw_cloud_topic = "/vboats/cloud/raw";
	std::string filtered_cloud_topic = "/vboats/cloud/filtered";
	p_nh.getParam("depth_image_topic",depth_image_topic);
	p_nh.getParam("camera_info_topic",camera_info_topic);
	p_nh.getParam("disparity_image_topic",disparity_image_topic);
	p_nh.getParam("umap_topic",umap_topic);
	p_nh.getParam("vmap_topic",vmap_topic);
	p_nh.getParam("filtered_image_topic",filtered_image_topic);
	p_nh.getParam("obstacles_image_topic",obstacles_image_topic);
	p_nh.getParam("obstacles_info_topic",detected_obstacles_info_topic);
	p_nh.getParam("raw_cloud_topic",raw_cloud_topic);
	p_nh.getParam("filtered_cloud_topic",filtered_cloud_topic);

	/** Initialize ROS-Objects */
	this->_cam_info_sub = m_nh.subscribe<sensor_msgs::CameraInfo>(camera_info_topic, 1, boost::bind(&VboatsRos::infoCallback,this,_1,1));
	if(this->_flag_depth_based) this->_depth_sub = m_nh.subscribe<sensor_msgs::Image>(depth_image_topic, 30, boost::bind(&VboatsRos::depthCallback,this,_1,100));
	else this->_disparity_sub = m_nh.subscribe<sensor_msgs::Image>(disparity_image_topic, 30, boost::bind(&VboatsRos::disparityCallback,this,_1,100));
	if(this->_publish_aux_images){
		this->_umap_pub = m_nh.advertise<sensor_msgs::Image>(umap_topic, 1);
		this->_vmap_pub = m_nh.advertise<sensor_msgs::Image>(vmap_topic, 1);
	}
	this->_new_img_pub = m_nh.advertise<sensor_msgs::Image>(filtered_image_topic, 1);
	this->_obstacles_img_pub = m_nh.advertise<sensor_msgs::Image>(obstacles_image_topic, 1);
	this->_detected_obstacle_info_pub = m_nh.advertise<swanson_msgs::VboatsObstacles>(detected_obstacles_info_topic, 100);
	// this->_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic, 1);
	if(this->_flag_pub_cloud) this->_cloud_pub = m_nh.advertise<PointCloud>(raw_cloud_topic, 1);
	if(this->_flag_pub_filtered_cloud) this->_filtered_cloud_pub = m_nh.advertise<PointCloud>(filtered_cloud_topic, 1);

	/** ROS tf frames Configuration */
	std::string tf_prefix = "/";
	std::string parent_tf = "base_link";
	std::string cam_base_tf = "camera_link";
	p_nh.getParam("parent_frame",parent_tf);
	p_nh.getParam("camera_base_frame",cam_base_tf);

	std::string tmp_parent_tf = parent_tf;
	std::string tmp_cam_base_tf = cam_base_tf;
	if(flag_use_tf_prefix){
		tmp_parent_tf = tf_prefix + tmp_parent_tf;
		tmp_cam_base_tf = tf_prefix + tmp_cam_base_tf;
	}
	this->_parent_tf = tmp_parent_tf;
	this->_camera_tf = tmp_cam_base_tf;

	/** Pre-initialize Common variables */
	this->_umapImgHeader = std_msgs::Header();
	this->_vmapImgHeader = std_msgs::Header();
	this->_filteredImgHeader = std_msgs::Header();
	this->_obsImgHeader = std_msgs::Header();
	this->_depth2disparityFactor = 1.0;

	/** Initialize VBOATS */
	this->vb = new VBOATS();

	printf("[INFO] VboatsRos::VboatsRos() ---- Successfully Initialized!\r\n");
	this->_loop_rate = new ros::Rate(update_rate);
}

VboatsRos::~VboatsRos(){
	delete this->_loop_rate;
	delete this->vb;
}

void VboatsRos::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg, const int value){
	if(this->_info_count < value){
	     this->_fx = msg->K[0];
		this->_fy = msg->K[4];
		this->_px = msg->K[2];
		this->_py = msg->K[5];
		this->_dscale = msg->D[0];
		float Tx = msg->P[3];
		this->_baseline = -Tx / this->_fx;
		printf("[INFO] VboatsRos::infoCallback() ---- Dscale = %.4f, Baseline = %.6f,  Focals = [%.2f, %.2f],  Principles = [%.2f, %.2f]\r\n", this->_dscale, this->_baseline, this->_fx, this->_fy, this->_px, this->_py);
		this->_focal[0] = this->_fx;
		this->_focal[1] = this->_fy;
		this->_principle[0] = this->_px;
		this->_principle[1] = this->_py;
		this->_depth2disparityFactor = (this->_fx * this->_baseline) / this->_dscale;
		this->_recvd_cam_info = true;
	}
	this->_info_count++;
}
void VboatsRos::depthCallback(const sensor_msgs::Image::ConstPtr& msg, const int value){
	std::lock_guard<std::mutex> lock(_lock);

	// cv::Mat depth;
	cv::Mat image = cv_bridge::toCvCopy(msg)->image;
	// this->_depth = depth.clone();
	this->_depth = image;
	this->_img_count++;
}
void VboatsRos::disparityCallback(const sensor_msgs::Image::ConstPtr& msg, const int value){
	std::lock_guard<std::mutex> lock(_lock);

	cv::Mat disparity;
	cv::Mat image = cv_bridge::toCvCopy(msg)->image;
	if(image.type() != CV_8UC1){
		double minVal, maxVal;
	     cv::minMaxLoc(image, &minVal, &maxVal);
		image.convertTo(disparity, CV_8UC1, (255.0/maxVal) );
	} else disparity = image;

	// this->_disparity = disparity.clone();
	this->_disparity = disparity;
	this->_img_count++;
}

void VboatsRos::depth_to_disparity(const cv::Mat& depth, cv::Mat* disparity, float gain){
	cv::Mat _disparity, _disparity8;
     if(depth.type() != CV_32F) depth.convertTo(_disparity, CV_32F);
	else _disparity = depth.clone();

	ForEachDepthConverter<float> converter(gain);
     _disparity.forEach<float>(converter);

	if(_disparity.type() != CV_8UC1){
		double minVal, maxVal;
	     cv::minMaxLoc(_disparity, &minVal, &maxVal);
		_disparity.convertTo(_disparity8, CV_8UC1, (255.0/maxVal) );
	} else _disparity8 = _disparity;

	if(disparity) *disparity = _disparity8.clone();
}

void VboatsRos::publish_images(const cv::Mat& umap, const cv::Mat& vmap, const cv::Mat& filtered){
	ros::Time time = ros::Time::now();
	/** Umap Image */
	if(!umap.empty()){
		this->_umapImgHeader.stamp = time;
		this->_umapImgHeader.seq = this->_count;
		this->_umapImgHeader.frame_id = this->_camera_tf;

		sensor_msgs::ImagePtr umapImgMsg = cv_bridge::CvImage(this->_umapImgHeader, "8UC1", umap).toImageMsg();
		this->_umap_pub.publish(umapImgMsg);
	}

	/** Vmap Image */
	if(!vmap.empty()){
		this->_vmapImgHeader.stamp = time;
		this->_vmapImgHeader.seq = this->_count;
		this->_vmapImgHeader.frame_id = this->_camera_tf;

		sensor_msgs::ImagePtr vmapImgMsg = cv_bridge::CvImage(this->_vmapImgHeader, "8UC1", vmap).toImageMsg();
		this->_vmap_pub.publish(vmapImgMsg);
	}

	/** Filtered Disparity */
	// if(!filtered.empty()){
	// 	this->_filteredImgHeader.stamp = time;
	// 	this->_filteredImgHeader.seq = this->_count;
	// 	this->_filteredImgHeader.frame_id = this->_camera_tf;
	//
	// 	sensor_msgs::ImagePtr filteredImgMsg = cv_bridge::CvImage(this->_filteredImgHeader, "8UC1", filtered).toImageMsg();
	// 	this->_new_img_pub.publish(filteredImgMsg);
	// }
}
void VboatsRos::publish_obstacle_image(cv::Mat image){
	ros::Time time = ros::Time::now();
	/** Obstacles Image */
	if(!image.empty()){
		this->_obsImgHeader.stamp = time;
		this->_obsImgHeader.seq = this->_count;
		this->_obsImgHeader.frame_id = this->_camera_tf;

		sensor_msgs::ImagePtr obsImgMsg = cv_bridge::CvImage(this->_obsImgHeader, "bgr8", image).toImageMsg();
		this->_obstacles_img_pub.publish(obsImgMsg);
	}
}
void VboatsRos::publish_obstacle_data(vector<Obstacle>& obstacles, const cv::Mat& dImage){
	cv::Mat display = dImage.clone();
	if(this->_publish_images) cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);
	int n = 0;
	swanson_msgs::VboatsObstacles obsMsg;
	obsMsg.header.stamp = ros::Time::now();
	obsMsg.header.frame_id = this->_camera_tf;
	for(Obstacle ob : obstacles){
          if(this->_verbose_obstacles) printf("Obstacle [%d]: ", n+1);
          ob.update(false,this->_baseline, this->_dscale, this->_focal, this->_principle, 1.0, 1.0, this->_verbose_obstacles);

		swanson_msgs::VboatsObstacle tmpOb;
		tmpOb.header.seq = n;
		tmpOb.header.stamp = obsMsg.header.stamp;
		tmpOb.header.frame_id = obsMsg.header.frame_id;
		tmpOb.distance = ob.distance;
		tmpOb.angle = ob.angle;
		tmpOb.position.x = ob.location.x;
		tmpOb.position.y = ob.location.y;
		tmpOb.position.z = ob.location.z;
		obsMsg.obstacles.push_back(tmpOb);

          if(this->_publish_images) cv::rectangle(display, ob.minXY, ob.maxXY, cv::Scalar(255, 0, 255), 1);
		n++;
     }
	this->_detected_obstacle_info_pub.publish(obsMsg);
	if(this->_publish_images) this->publish_obstacle_image(display);
}



void VboatsRos::generate_pointcloud(const cv::Mat& depth){
	PointCloud::Ptr pointcloud_msg (new PointCloud);
	// pointcloud_msg->header.stamp = ros::Time::now().toNSec();
	pointcloud_msg->header.seq = this->_count;
	pointcloud_msg->header.frame_id = this->_camera_tf;

	pcl::PointXYZ pt;
	for(int y = 0; y < depth.rows; y+=4){
		for(int x = 0; x < depth.cols; x+=4){
			float depthVal = (float) depth.at<short int>(cv::Point(x,y)) * this->_dscale;
			if(depthVal > 0){
				pt.x = (x - this->_px) * depthVal / this->_fx;
				pt.y = (y - this->_py) * depthVal / this->_fy;
				pt.z = depthVal;
				pointcloud_msg->points.push_back(pt);
			}
		}
	}
	pointcloud_msg->height = 1;
	pointcloud_msg->width = pointcloud_msg->points.size();
	if(this->_flag_pub_cloud) this->_cloud_pub.publish(pointcloud_msg);

	if(this->_filter_cloud){
		PointCloud::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(pointcloud_msg);
		pass.setFilterFieldName("y");
		pass.setFilterLimits(-this->_max_obstacle_height, 1.0);
		// pass.setFilterLimitsNegative(true);
		pass.filter(*cloud_filtered);

		PointCloud::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud_filtered);
		sor.setLeafSize(0.05f, 0.05f, 0.05f);
		sor.filter(*output);
		if(this->_flag_pub_filtered_cloud) this->_filtered_cloud_pub.publish(output);
	}

}

int VboatsRos::remove_ground(const cv::Mat& disparity, const cv::Mat& vmap, const cv::Mat& depth, float* line_params){
	cv::Mat refImg = vmap.clone();
	cv::Mat img = disparity.clone();
	cv::Mat depthImg;
	if(!depth.empty()) depthImg = depth.clone();
	else depthImg = img.clone();
	cv::Mat testImg = depthImg.clone();
	cv::Mat mask = cv::Mat::zeros(disparity.size(), CV_8UC1);
	// cv::Mat mask2 = cv::Mat::zeros(disparity.size(), CV_8UC1);

	/** Calculate ROI limits based on estimated ground line coefficients */
	int dband = 25;
	float slope = line_params[0];
	int b = (int) line_params[1];
	int y0 = b;
	int y1 = y0 + dband;
	int y1f = (int)(vmap.cols * slope + (y1));
	int y2 = y0 - dband - 2;
	int y2f = (int)(vmap.cols * slope + (y2));
	if(y0 < 0) y0 = 0;
	// printf("[INFO] VboatsRos::remove_ground() --- slope = %.2f, intercept = %d\r\n", slope, b);

	// std::vector<std::vector<cv::Point> > fillContAll;
	// std::vector<cv::Point> cnt1;
	// cnt1.push_back(cv::Point(0,0));
	// cnt1.push_back(cv::Point(refImg.cols,0));
	// cnt1.push_back(cv::Point(refImg.cols,y2f));
	// cnt1.push_back(cv::Point(0,y2));
	// cnt1.push_back(cv::Point(0,0));
	// fillContAll.push_back(cnt1);
	// cv::fillPoly( refImg, fillContAll, cv::Scalar(0));

	// cv::Mat testRef = refImg.clone();
	// // int limits[testRef.rows][2];
	// cv::Mat mins = cv::Mat(testRef.rows, 1, CV_8SC1, cv::Scalar(-1));
	// cv::Mat maxs = cv::Mat(testRef.rows, 1, CV_8SC1, cv::Scalar(-1));
	// cv::Mat testMask = testRef > 0;
	// cv::Mat testNonzero;
	// cv::findNonZero(testMask, testNonzero);
	//
	// int testx, currow;
	// int curmin, curmax;
	// printf("[INFO] VboatsRos::remove_ground() --- testNonzero (x, y):\r\n\t");
	// for(int j = 0; j < testNonzero.total(); j++ ){
	// 	currow = testNonzero.at<cv::Point>(j).y;
	// 	curmin = mins.at<int>(currow);
	// 	curmax = maxs.at<int>(currow);
	// 	testx = testNonzero.at<cv::Point>(j).x;
	// 	if(testx > curmax){
	// 		maxs.at<int>(currow) = testx;
	// 		printf("new max for row [%d] = %d -- previous max = %d\r\n", currow, testx,curmax);
	// 	}
	// 	if(testx < curmin){
	// 		mins.at<int>(currow) = testx;
	// 		printf("new min for row [%d] = %d -- previous min = %d\r\n", currow, testx,curmin);
	// 	}
	// }
	//
	// printf("[INFO] VboatsRos::remove_ground() --- testNonzero: \r\n");
	// // cout << "testNonzero " << endl << " "  << testNonzero << endl << endl;
	// cvinfo(testNonzero,"\ttestNonzero");
	// printf(" ----------------- \r\n");

	uchar* pix;
	cv::Mat refRow;
	cv::Mat refMask;
	cv::Mat nonzero;
	double t, dt;
	int minx, maxx, tmpx;
	// int mins[img.rows];
	// int maxs[img.rows];
	// t = (double)cv::getTickCount();
	// for(int row = 0; row < img.rows; ++row){
	// 	if(row < y2){
	// 		mins[row] = -1;
	// 		maxs[row] = -1;
	// 	}else{
	// 		refRow = refImg.row(row);
	// 		refMask = refRow > 0;
	// 		cv::findNonZero(refMask, nonzero);
	// 		int xlim = (int)((float)(row - y2) / slope);
	// 		maxx = 0;
	// 		minx = 1000;
	// 		for(int i = 0; i < nonzero.total(); i++ ){
	// 			tmpx = nonzero.at<cv::Point>(i).x;
	// 			if(tmpx > xlim) continue;
	// 			if(tmpx > maxx) maxx = tmpx;
	// 			if(tmpx < minx) minx = tmpx;
	// 		}
	// 		// printf("[INFO] VboatsRos::remove_ground() --- Scan Row %d -- minX = %d |  maxIdx = %d | xLimit = %d \r\n", row, minx, maxx, xlim);
	// 		mins[row] = minx;
	// 		maxs[row] = maxx;
	// 	}
	// }
	// ForEachGndMaskGenerator<uchar> initializer(mins, maxs, img.rows);//, disparity.size());
	// testImg.forEach<uchar>(initializer);
	// cv::Mat mask2 = initializer.m_mask;

	// dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
	// printf("[INFO] VboatsRos::remove_ground() ---- Ground Mask Generation 1 took %.4lf ms (%.2lf Hz)\r\n", dt*1000.0, (1.0/dt));

	t = (double)cv::getTickCount();
	for(int v = y0; v < img.rows; ++v){
	     refRow = refImg.row(v);
		refMask = refRow > 0;
		cv::findNonZero(refMask, nonzero);
		maxx = 0;
		minx = 1000;
		for(int i = 0; i < nonzero.total(); i++ ){
			tmpx = nonzero.at<cv::Point>(i).x;
			int xlim = (int)((float)(v - y2) / slope);
			if(tmpx > xlim) continue;
			if(tmpx > maxx) maxx = tmpx;
			if(tmpx < minx) minx = tmpx;
		}
		// printf("[INFO] VboatsRos::remove_ground() --- Scan Row %d :\r\n", v);
		// cvinfo(nonzero,"nonzero");
		// cout << "nonzero " << endl << " "  << nonzero << endl << endl;
		// printf(" ----------------- \r\n");
		// printf("[INFO] VboatsRos::remove_ground() --- Scan Row %d -- minX = %d |  maxIdx = %d | TestminX = %d |  TestmaxIdx = %d\r\n", v, minx, maxx, mins.at<int>(v), maxs.at<int>(v));
		// printf("[INFO] VboatsRos::remove_ground() --- Scan Row %d -- minX = %d |  maxIdx = %d\r\n", v, minx, maxx);

		pix = img.ptr<uchar>(v);
	     for(int u = 0; u < img.cols; ++u){
	          int dvalue = pix[u];
	          if( (dvalue >= minx) && (dvalue <= maxx) ) mask.at<uchar>(v, u) = 255;
	     }
	}


	cv::Mat maskInv;
	cv::bitwise_not(mask,maskInv);
	// cv::Mat maskInv2;
	// cv::bitwise_not(mask2,maskInv2);
	// cvinfo(maskInv,"maskInv");
	// cvinfo(mask,"mask");
	cv::Mat gndFilteredImg;
	depthImg.copyTo(gndFilteredImg, maskInv);
	{
		// dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
		// printf("[INFO] VboatsRos::remove_ground() ---- Ground Mask Generation 2 took %.4lf ms (%.2lf Hz)\r\n", dt*1000.0, (1.0/dt));
	}

	this->_filteredImgHeader.stamp = ros::Time::now();
	this->_filteredImgHeader.seq = this->_count;
	this->_filteredImgHeader.frame_id = this->_camera_tf;

	sensor_msgs::ImagePtr filteredImgMsg;
	if(gndFilteredImg.type() == CV_8UC1) filteredImgMsg = cv_bridge::CvImage(this->_filteredImgHeader, "8UC1", gndFilteredImg).toImageMsg();
	else if(gndFilteredImg.type() == CV_16UC1) filteredImgMsg = cv_bridge::CvImage(this->_filteredImgHeader, "16UC1", gndFilteredImg).toImageMsg();
	else if(gndFilteredImg.type() == CV_32F) filteredImgMsg = cv_bridge::CvImage(this->_filteredImgHeader, "32FC1", gndFilteredImg).toImageMsg();
	else if(gndFilteredImg.type() == CV_8UC3) filteredImgMsg = cv_bridge::CvImage(this->_filteredImgHeader, "bgr8", gndFilteredImg).toImageMsg();
	this->_new_img_pub.publish(filteredImgMsg);

	if((this->_flag_pub_cloud) || (this->_flag_pub_filtered_cloud)) this->generate_pointcloud(gndFilteredImg);

	if(this->_visualize_images){
	// if(true)/{
          cv::Mat display;
		/** Visualize segmented image */
		cv::applyColorMap(mask, display, cv::COLORMAP_JET);
		cv::imshow("Ground Segmentation 1", display);
		// cv::applyColorMap(testImg, display, cv::COLORMAP_JET);
		// cv::imshow("Ground Segmentation 2", testImg);
          // cv::waitKey(10);
     }

	return 0;
}

int VboatsRos::process(const cv::Mat& disparity, const cv::Mat& umap, const cv::Mat& vmap, vector<Obstacle>* obstacles, const cv::Mat& depth){
	int nObs = 0;
	vector<float> vthreshs = {0.3, 0.3,0.25,0.4};
	vector<float> uthreshs = {0.3,0.295,0.3,0.35};

	/** Pre-filter Umap */
	cv::Mat uTmp, uProcessed;
     cv::cvtColor(umap, uTmp, cv::COLOR_GRAY2BGR);
     cv::rectangle(uTmp, cv::Point(0,0), cv::Point(umap.cols,3), cv::Scalar(0, 0, 0), -1);
     cv::cvtColor(uTmp, uTmp, cv::COLOR_BGR2GRAY);
	cv::boxFilter(uTmp,uTmp,-1, cv::Size(2,2));
	this->vb->filter_disparity_umap(uTmp, &uProcessed, &uthreshs);

	/** Pre-filter Vmap: Approach 1 */
     cv::Mat vTmp, vProcessed;
     cv::cvtColor(vmap, vTmp, cv::COLOR_GRAY2BGR);
     cv::rectangle(vTmp, cv::Point(0,0), cv::Point(3,vmap.rows), cv::Scalar(0, 0, 0), -1);
     cv::cvtColor(vTmp, vTmp, cv::COLOR_BGR2GRAY);
     // this->vb->filter_disparity_vmap(vTmp, &vProcessed, &vthreshs);

	/** Pre-filter Vmap: Approach 2 - better highlight useful lines*/
	cv::Mat tmpSobel, sobelV;
	double minVal, maxVal;
     cv::Sobel(vmap, tmpSobel, CV_64F, 0, 1, 3);
     cv::minMaxLoc(tmpSobel, &minVal, &maxVal);
     tmpSobel = tmpSobel * (255.0/maxVal);
     cv::convertScaleAbs(tmpSobel, tmpSobel, 1, 0);
     threshold(tmpSobel, sobelV, 30, 255, cv::THRESH_TOZERO);

	/** Extract ground line parameters (if ground is present) */
	float* line_params;
     float gndM; int gndB;
     bool gndPresent = this->vb->find_ground_line(sobelV, &gndM,&gndB);
     if(gndPresent){
          float tmpParams[] = {gndM, (float) gndB};
          line_params = &tmpParams[0];
     } else line_params = nullptr;

	/** Ground segmentation */
	if((gndPresent) && (this->_filter_ground)) this->remove_ground(disparity,vTmp, depth, line_params);

     vector<vector<cv::Point>> contours;
	vector<Obstacle> _obstacles;
	if(this->_detect_obstacles){
		/** Find contours in Umap useful for obstacle filtering */
		this->vb->find_contours(uProcessed, &contours, 1, 50, nullptr, -1, false, this->_visualize_images);
		/** Extract obstacles */
		nObs = this->vb->find_obstacles_disparity(vmap, contours, &_obstacles, line_params);
		this->publish_obstacle_data(_obstacles, disparity);
	}

	if(this->_visualize_images){
          cv::Mat display;
		/** Visualize pre-filtered uv-maps */
		cv::applyColorMap(uProcessed, display, cv::COLORMAP_JET);
		cv::imshow("Pre-filtered Umap", display);
          cv::applyColorMap(vTmp, display, cv::COLORMAP_JET);
		cv::imshow("Pre-filtered Vmap", display);

		/** Visualize estimated ground line */
          cv::cvtColor(sobelV, display, cv::COLOR_GRAY2BGR);
		int band = 25;
          int yk = int(sobelV.cols * gndM) + gndB;
          int yu = int(sobelV.cols * gndM) + (gndB-band);
          int yl = int(sobelV.cols * gndM) + (gndB+band);
		cv::line(display, cv::Point(0, gndB), cv::Point(sobelV.cols, yk), cv::Scalar(0,255,0), 2, cv::LINE_AA);
		cv::line(display, cv::Point(0, (gndB-band)), cv::Point(sobelV.cols, yu), cv::Scalar(255,0,0), 2, cv::LINE_AA);
		cv::line(display, cv::Point(0, (gndB+band)), cv::Point(sobelV.cols, yl), cv::Scalar(0,0,255), 2, cv::LINE_AA);
		cv::imshow("Estimated Gnd", display);

          cv::waitKey(10);
     }

	if(obstacles) *obstacles = _obstacles;
	return nObs;
}

// void VboatsRos::update(const cv::Mat& image, const cv::Mat& umap, const cv::Mat& vmap, float conversion_gain, bool verbose, bool debug_timing){
int VboatsRos::update(bool verbose, bool debug_timing){
	int nObs = 0;
	vector<Obstacle> obs;
	cv::Mat curDepth, curDisparity, umap, vmap, display;
	/** Don't do any proessing if we haven't received valid camera info */
	if(!this->_recvd_cam_info) return -1;

	this->_lock.lock();
	if(this->_flag_depth_based){
		curDepth = this->_depth;
		this->depth_to_disparity(curDepth,&curDisparity, this->_depth2disparityFactor);
	} else curDisparity = this->_disparity;
	this->_lock.unlock();
	/** Don't do any proessing if disparity is empty */
	if(curDisparity.empty()) return -1;

	double t = (double)cv::getTickCount();
	genUVMapThreaded(curDisparity,&umap,&vmap, 2.0);
	nObs = this->process(curDisparity, umap, vmap, &obs, curDepth);

	if(debug_timing){
		double dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
		// printf("[INFO] VboatsRos::update() ---- Found %d obstacles in %.4lf ms (%.2lf Hz)\r\n", nObs, dt*1000.0, (1.0/dt));
	}

	// if(this->_publish_images) cv::cvtColor(curDisparity, display, cv::COLOR_GRAY2BGR);
	// swanson_msgs::VboatsObstacles obsMsg;
	// obsMsg.header.stamp = ros::Time::now();
	// obsMsg.header.frame_id = this->_camera_tf;
	// int n = 0;
	// for(Obstacle ob : obs){
     //      if(this->_verbose_obstacles) printf("Obstacle [%d]: ", n+1);
     //      ob.update(false,this->_baseline, this->_dscale, this->_focal, this->_principle, 1.0, 1.0, this->_verbose_obstacles);
	//
	// 	swanson_msgs::VboatsObstacle tmpOb;
	// 	tmpOb.header.seq = n;
	// 	tmpOb.header.stamp = obsMsg.header.stamp;
	// 	tmpOb.header.frame_id = obsMsg.header.frame_id;
	// 	tmpOb.distance = ob.distance;
	// 	tmpOb.angle = ob.angle;
	// 	tmpOb.position.x = ob.location.x;
	// 	tmpOb.position.y = ob.location.y;
	// 	tmpOb.position.z = ob.location.z;
	// 	obsMsg.obstacles.push_back(tmpOb);
	//
     //      if(this->_publish_images) cv::rectangle(display, ob.minXY, ob.maxXY, cv::Scalar(255, 0, 255), 1);
	// 	n++;
     // }
	// // if(this->_verbose_obstacles) printf(" --------- \r\n");
	// this->_detected_obstacle_info_pub.publish(obsMsg);
	if(this->_publish_aux_images){
		// this->publish_obstacle_image(display);
		this->publish_images(umap, vmap, cv::Mat());
	}

	this->_count++;
	return 0;
}

int VboatsRos::run(bool verbose){
	double t = (double)cv::getTickCount();
     while(ros::ok()){
		this->update();
          ros::spinOnce();
          this->_loop_rate->sleep();
     }

     return 0;
}

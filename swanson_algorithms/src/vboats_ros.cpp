#include <iostream>
#include "swanson_algorithms/vboats_ros.h"

#include <RoboCommander/algorithms/vboats/uvmap_utils.h>
#include <RoboCommander/utilities/cv_utils.h>
#include <swanson_msgs/VboatsObstacle.h>
#include <swanson_msgs/VboatsObstacles.h>

using namespace std;

template<typename Pixel>
struct ForEachPclOperator{
     Pixel m_gain;
     ForEachPclOperator(Pixel gain){
          m_gain = gain;
     }
     void operator()(Pixel& pixel, const int * idx) const {
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
	this->_ns = m_nh.getNamespace();

	/** Flag Configuration */
	bool flag_verbose_obstacles = false;
	bool flag_verbose_timings = false;
	bool flag_use_tf_prefix = false;
	bool flag_publish_imgs = true;
	bool visualize_imgs = false;
	int update_rate = 30;

	p_nh.getParam("verbose_obstacles",flag_verbose_obstacles);
	p_nh.getParam("verbose_timings",flag_verbose_timings);
	p_nh.getParam("publish_images",flag_publish_imgs);
	p_nh.getParam("show_images",visualize_imgs);
	p_nh.getParam("update_rate",update_rate);

	this->_verbose_obstacles = flag_verbose_obstacles;
	this->_verbose_timings = flag_verbose_timings;
	this->_publish_images = flag_publish_imgs;
	this->_visualize_images = visualize_imgs;
	this->_update_rate = update_rate;

	/** ROS Object Configuration */
	std::string camera_info_topic = "/camera/depth/camera_info";
	std::string disparity_image_topic = "/camera/disparity/image_raw";
	std::string umap_topic = "/vboats/umap/image_raw";
	std::string vmap_topic = "/vboats/vmap/image_raw";
	std::string filtered_image_topic = "/vboats/disparity/image_filtered";
	std::string obstacles_image_topic = "/vboats/obstacles/image_raw";
	std::string detected_obstacles_info_topic = "/vboats/obstacles/data";
	p_nh.getParam("camera_info_topic",camera_info_topic);
	p_nh.getParam("disparity_image_topic",disparity_image_topic);
	p_nh.getParam("umap_topic",umap_topic);
	p_nh.getParam("vmap_topic",vmap_topic);
	p_nh.getParam("filtered_image_topic",filtered_image_topic);
	p_nh.getParam("obstacles_image_topic",obstacles_image_topic);
	p_nh.getParam("obstacles_info_topic",detected_obstacles_info_topic);

	/** Initialize ROS-Objects */
	this->_cam_info_sub = m_nh.subscribe<sensor_msgs::CameraInfo>(camera_info_topic, 1, boost::bind(&VboatsRos::infoCallback,this,_1,1));
	this->_disparity_sub = m_nh.subscribe<sensor_msgs::Image>(disparity_image_topic, 30, boost::bind(&VboatsRos::imageCallback,this,_1,100));
	this->_umap_pub = m_nh.advertise<sensor_msgs::Image>(umap_topic, 1);
	this->_vmap_pub = m_nh.advertise<sensor_msgs::Image>(vmap_topic, 1);
	this->_new_img_pub = m_nh.advertise<sensor_msgs::Image>(filtered_image_topic, 1);
	this->_obstacles_img_pub = m_nh.advertise<sensor_msgs::Image>(obstacles_image_topic, 1);
	this->_detected_obstacle_info_pub = m_nh.advertise<swanson_msgs::VboatsObstacles>(detected_obstacles_info_topic, 100);

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
	this->_disparity2depth = 1.0;

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
		this->_recvd_cam_info = true;
	}
	this->_info_count++;
}
void VboatsRos::imageCallback(const sensor_msgs::Image::ConstPtr& msg, const int value){
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
	if(!filtered.empty()){
		this->_filteredImgHeader.stamp = time;
		this->_filteredImgHeader.seq = this->_count;
		this->_filteredImgHeader.frame_id = this->_camera_tf;

		sensor_msgs::ImagePtr filteredImgMsg = cv_bridge::CvImage(this->_filteredImgHeader, "8UC1", filtered).toImageMsg();
		this->_new_img_pub.publish(filteredImgMsg);
	}
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

	// if(this->_img_count == value){
	// 	cvinfo(this->_disparity,"disparity");
	// } else if(this->_img_count >= value){
	// 	cv::Mat display;
	// 	cv::applyColorMap(disparity, display, cv::COLORMAP_JET);
	//
	// 	this->_filteredImgHeader.stamp = ros::Time::now();
	// 	this->_filteredImgHeader.seq = this->_img_count;
	// 	this->_filteredImgHeader.frame_id = this->_camera_tf;
	// 	sensor_msgs::ImagePtr newImgMsg = cv_bridge::CvImage(this->_filteredImgHeader, "bgr8", display).toImageMsg();
	// 	this->_new_img_pub.publish(newImgMsg);
	// }
}
void VboatsRos::publish_obstacle_data(vector<Obstacle> obstacles){}

int VboatsRos::remove_ground(const cv::Mat& disparity, const cv::Mat& vmap, const vector<Obstacle>& obstacles, float* line_params){
	cv::Mat mask = cv::Mat::ones(disparity.size(), CV_8UC1);
	int dband = 25;
	int y0 = (int)(0 * line_params[0] + line_params[1]);
	int y1 = y0 + dband;
	int y1f = (int)(vmap.cols * line_params[0] + (line_params[1]+dband));
	int y2 = y0 - dband - 10;
	int y2f = (int)(vmap.cols * line_params[0] + (line_params[1]-dband-10));
	printf("[INFO] VboatsRos::remove_ground() --- slope = %.2f, intercept = %d\r\n", line_params[0], (int)line_params[1]);

	cv::Mat refImg = vmap.clone();
	std::vector<cv::Point> cnt1;
	cnt1.push_back(cv::Point(0,0));
	cnt1.push_back(cv::Point(refImg.cols,0));
	cnt1.push_back(cv::Point(refImg.cols,y2f));
	cnt1.push_back(cv::Point(0,y2));
	cnt1.push_back(cv::Point(0,0));

	std::vector<cv::Point> cnt2;
	cnt2.push_back(cv::Point(0,y1));
	cnt2.push_back(cv::Point(refImg.cols,y1f));
	cnt2.push_back(cv::Point(0,y1f));
	cnt2.push_back(cv::Point(0,y1));

	std::vector<std::vector<cv::Point> > fillContAll;
	fillContAll.push_back(cnt1);
	fillContAll.push_back(cnt2);
	cv::fillPoly( refImg, fillContAll, cv::Scalar(0));

	cv::Mat img = disparity.clone();
	uchar* pix;
	cv::Mat refRow;
	cv::Mat refMask;
	double minVal, maxVal;
	cv::Point minValIdx, maxValIdx;

	cv::Mat nonzero;
	for(int v = y0; v < img.rows; ++v){
	     pix = img.ptr<uchar>(v);
	     refRow = refImg.row(v);
		refMask = refRow > 0;
		cv::findNonZero(refMask, nonzero);
		int tmpx;
		int minx = 1000, maxx = 0;
		for(int i = 0; i < nonzero.total(); i++ ){
			tmpx = nonzero.at<cv::Point>(i).x;
			if(tmpx > maxx) maxx = tmpx;
			if(tmpx < minx) minx = tmpx;
		}
		// cv::minMaxLoc(refRow, &minVal, &maxVal, &minValIdx, &maxValIdx,refMask);
		// printf("[INFO] VboatsRos::remove_ground() --- Scan Row %d -- minX = %d |  maxIdx = %d\r\n", v, minx, maxx);
		// printf("[INFO] VboatsRos::remove_ground() --- Scan Row %d -- minVal = %.2f (at %d) |  maxVal = %.2f (at %d)\r\n",
		// 	v, minVal, minValIdx.x, maxVal, maxValIdx.x);
	     for(int u = 0; u < img.cols; ++u){
	          int dvalue = pix[u];
	          // if( (dvalue >= maxx) && (dvalue <= minx) ){
	          if( (dvalue >= minx) && (dvalue <= maxx) ){
				mask.at<uchar>(v, u) = 0;
				// pix[u] = ((float) gain / dvalue);
			} else{}
	     }
	}

	// ForEachPclOperator<float> initializer((float)gain);
	// tmpMat.forEach<float>(initializer);

	cv::Mat gndFilteredImg;
	img.copyTo(gndFilteredImg, mask);
	this->_filteredImgHeader.stamp = ros::Time::now();
	this->_filteredImgHeader.seq = this->_count;
	this->_filteredImgHeader.frame_id = this->_camera_tf;

	sensor_msgs::ImagePtr filteredImgMsg = cv_bridge::CvImage(this->_filteredImgHeader, "8UC1", gndFilteredImg).toImageMsg();
	this->_new_img_pub.publish(filteredImgMsg);


	if(this->_visualize_images){
          cv::Mat display;
		/** Visualize segmented image */
		cv::applyColorMap(mask, display, cv::COLORMAP_JET);
		cv::imshow("Ground Segmentation", display);

		// cv::Mat nonzeroMask = refImg>0;
		// cv::applyColorMap(nonzeroMask, display, cv::COLORMAP_JET);
		// cv::imshow("Nonzero Mask", display);

          cv::waitKey(10);
     }

	return 0;
}

int VboatsRos::process(const cv::Mat& disparity, const cv::Mat& umap, const cv::Mat& vmap, vector<Obstacle>* obstacles){
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

	/** Find contours in Umap useful for obstacle filtering */
     vector<vector<cv::Point>> contours;
     this->vb->find_contours(uProcessed, &contours, 1, 50, nullptr, -1, false, this->_visualize_images);

	/** Extract obstacles */
     vector<Obstacle> _obstacles;
     int nObs = this->vb->find_obstacles_disparity(vmap, contours, &_obstacles, line_params);

	/** Ground segmentation */
	if(gndPresent) this->remove_ground(disparity,vTmp, _obstacles,line_params);

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
	cv::Mat curDisparity, umap, vmap, display;
	this->_lock.lock();
	curDisparity = this->_disparity;
	this->_lock.unlock();
	/** Don't do any proessing if disparity is empty */
	if(curDisparity.empty()) return -1;

	double t = (double)cv::getTickCount();

	genUVMapThreaded(curDisparity,&umap,&vmap, 2.0);
	// nObs = this->vb->pipeline_disparity(curDisparity, umap, vmap, &obs);
	nObs = this->process(curDisparity, umap, vmap, &obs);

	if(debug_timing){
		double dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
		printf("[INFO] VboatsRos::update() ---- Found %d obstacles in %.4lf ms (%.2lf Hz)\r\n", nObs, dt*1000.0, (1.0/dt));
	}

	if(this->_publish_images) cv::cvtColor(curDisparity, display, cv::COLOR_GRAY2BGR);
	swanson_msgs::VboatsObstacles obsMsg;
	obsMsg.header.stamp = ros::Time::now();
	obsMsg.header.frame_id = this->_camera_tf;
	int n = 0;
	for(Obstacle ob : obs){
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
	// if(this->_verbose_obstacles) printf(" --------- \r\n");
	this->_detected_obstacle_info_pub.publish(obsMsg);
	if(this->_publish_images){
		this->publish_obstacle_image(display);
		this->publish_images(umap, vmap, cv::Mat());
	}

	this->_count++;
	return 0;
}

int VboatsRos::run(bool verbose){
	double t = (double)cv::getTickCount();
     while(ros::ok()){
		this->update();
		// this->_lock.lock();
		// // disparity = this->_disparity.clone();
		// // umap = this->_umap.clone();
		// // vmap = this->_vmap.clone();
		// // cvt_gain = this->_disparity2depth;
		// this->_lock.unlock();
		// genUVMapThreaded(disparity,&umap,&vmap, 2.0);
		// this->_umap = umap.clone();
		// this->_vmap = vmap.clone();
		// // this->update(disparity,umap, vmap,cvt_gain);
		// // cvinfo(depth,"depth");
		//
		// if(this->_verbose_timings){
		// 	double dt = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
		// 	printf("[INFO] VboatsRos::run() ---- pipeline_disparity took %.4lf ms (%.2lf Hz)\r\n", dt*1000.0, (1.0/dt));
		// 	t = (double)cv::getTickCount();
		// }
		// if(this->_visualize_images){
			// if(!rgb.empty()) cv::imshow("RGB", rgb);
			// if(!depth.empty()) cv::imshow("Depth", depth);
			// if(!disparity.empty()) cv::imshow("Disparity", disparity);
			// if(!umap.empty()) cv::imshow("Umap", umap);
			// if(!vmap.empty()) cv::imshow("Vmap", vmap);
			// cv::waitKey(10);
		// }

          ros::spinOnce();
          this->_loop_rate->sleep();
     }

     return 0;
}

#include <iostream>
#include "swanson_teleop/android_ros_interface_listener.h"

using namespace std;

/** SECTION:
     CONSTRUCTOR & DECONSTRUCTOR
*/
AndroidRosInterface::AndroidRosInterface(ros::NodeHandle nh, ros::NodeHandle _nh) : m_nh(nh), p_nh(_nh){

     // Declare constants
	_count = 0;
     _rc_msg_count = 0;
     _header_count = 0;
	_gimbal_msg_count = 0;
	timeoutCnt = 0;
	maxTimeout = 100;

	std::string prefix;
	std::string cmdTopic = "cmd_vel";
	std::string frontTarget = "target_angle/front";
	std::string rightTarget = "target_angle/right";
	std::string leftTarget = "target_angle/left";
	std::string backTarget = "target_angle/back";

	_max_speed = 1.5;
	int port_in = 14500;
	float max_turn_radius = 0.381;
	float update_rate = 30;

	p_nh.getParam("prefix",prefix);
	p_nh.getParam("cmd_topic",cmdTopic);
	p_nh.getParam("frontGimbalTargetTopic",frontTarget);
	p_nh.getParam("rightGimbalTargetTopic",rightTarget);
	p_nh.getParam("leftGimbalTargetTopic",leftTarget);
	p_nh.getParam("backGimbalTargetTopic",backTarget);
	p_nh.getParam("update_rate",update_rate);
	p_nh.getParam("app_port",port_in);
	p_nh.getParam("max_speed",_max_speed);
	p_nh.getParam("max_turn_radius",max_turn_radius);

	_max_omega = _max_speed / max_turn_radius;

	cmd_pub = m_nh.advertise<geometry_msgs::Twist>(cmdTopic, 1000);
	angF_pub = m_nh.advertise<std_msgs::Float32>(frontTarget, 100);
	angR_pub = m_nh.advertise<std_msgs::Float32>(rightTarget, 100);
	angL_pub = m_nh.advertise<std_msgs::Float32>(leftTarget, 100);
	angB_pub = m_nh.advertise<std_msgs::Float32>(backTarget, 100);

	// rc_in = new UDP(35555,NULL);
	mRelay = new AndroidAppInterface(port_in);
	_loop_rate = new ros::Rate(update_rate);
}

AndroidRosInterface::~AndroidRosInterface(){
	delete mRelay;
	delete _loop_rate;
}

void AndroidRosInterface::update(){
	AndroidInterfaceData tmpData;

	int err = this->mRelay->receiveUdpMessage();
	if(err < 0){
		this->timeoutCnt++;
		// printf("[INFO] AndroidAppInterface_Test() --- Timeout Count = %d\r\n",this->timeoutCnt);
	}else{ this->timeoutCnt = 0; }
	// printf("[INFO] face->receiveUdpMessage() --- Returned code \'%d\'\r\n",err);
	tmpData = this->mRelay->getReceivedData();

	float v = tmpData.normalized_speed * this->_max_speed;
	float w = tmpData.normalized_turn_rate * this->_max_omega;

	// Set velocities to zero in case of communication timeouts
	if(this->timeoutCnt >= this->maxTimeout){
		v = 0;
		w = 0;
	}

	geometry_msgs::Twist commands;
	commands.linear.x = v; commands.linear.y = 0; commands.linear.z = 0;
	commands.angular.x = 0; commands.angular.y = 0; commands.angular.z = -w;
	cmd_pub.publish(commands);

	float angleF = tmpData.front_cam_angle;
	float angleB = tmpData.back_cam_angle;
	float angleR = tmpData.right_cam_angle;
	float angleL = tmpData.left_cam_angle;

	std_msgs::Float32 targetFront;
	std_msgs::Float32 targetBack;
	std_msgs::Float32 targetRight;
	std_msgs::Float32 targetLeft;

	targetFront.data = angleF;
	targetBack.data = angleB;
	targetRight.data = angleR;
	targetLeft.data = angleL;

	angF_pub.publish(targetFront);
	angB_pub.publish(targetBack);
	angR_pub.publish(targetRight);
	angL_pub.publish(targetLeft);
}

int AndroidRosInterface::run(){
     cout << "Looping..." << endl;

     while(ros::ok()){
		this->update();

		// printUdpHeader(&_header);

		// Update ROS index
		_count++;
          ros::spinOnce();
          _loop_rate->sleep();
     }

     return 0;
}

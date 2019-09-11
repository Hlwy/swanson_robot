#include <iostream>
#include <boost/bind.hpp>
#include "swanson_controls/camera_gimbal_controller_ros.h"

using namespace std;

/** SECTION:
     CONSTRUCTOR & DECONSTRUCTOR
*/
CameraGimbalControllerRos::CameraGimbalControllerRos(ros::NodeHandle nh, ros::NodeHandle _nh) : m_nh(nh), p_nh(_nh){
     // Declare constants
	_count = 0;
     _maxAngle = 180.0;
     _targetAngle = 90.0;
	_nullPulse = 1590;
	_kp = 10.0;
	_ki = 3.0;
	_kd = 0.05;
	_control_dt = 0.05;

	int actuator_addr = 0x41;
	int actuator_bus = 1;
	int act_channel = 0;
	int actuator_freq = 50;

	std::string prefix;
	std::string target_angle_topic = "gimbal/fr/target";
	std::string imu_topic = "imu/fr";
	std::string data_topic = "gimbal/info";
	float update_rate = 20;

	p_nh.getParam("actuator_addr",actuator_addr);
	p_nh.getParam("actuator_bus",actuator_bus);
	p_nh.getParam("actuator_channel",act_channel);
	p_nh.getParam("actuator_frequency",actuator_freq);

	p_nh.getParam("max_angle",_maxAngle);
	p_nh.getParam("start_angle",_targetAngle);
	p_nh.getParam("neutral_cmd",_nullPulse);
	p_nh.getParam("kp",_kp);
	p_nh.getParam("ki",_ki);
	p_nh.getParam("kd",_kd);

	p_nh.getParam("prefix",prefix);
	p_nh.getParam("target_topic",target_angle_topic);
	p_nh.getParam("imu_topic",imu_topic);
	p_nh.getParam("data_topic",data_topic);
	p_nh.getParam("update_rate",update_rate);
	_control_dt = 1.0 / (float) update_rate;

	/* Connect to Pi. */
	int _pi = pigpio_start(NULL, NULL);
	if(_pi < 0){
		printf("[ERROR] Could not initialize with the pigpiod \r\n");
		exit(0);
	}else{ this->_pi = _pi; }

	/** Initialize Controller */
	cg = new CameraGimbal();
	cg->set_p_gain(_kp);			// Set P Gain
	cg->set_i_gain(_ki);			// Set I Gain
	cg->set_d_gain(_kd);			// Set D Gain
	cg->set_max_state(_maxAngle);
	cg->set_null_cmd((float)_nullPulse);
	cg->set_dt(_control_dt);

	/** Initialize Actuator */
	_actCommCfg.platform = this->_pi;
	_actCommCfg.i2c.address = actuator_addr;
	_actCommCfg.i2c.bus = actuator_bus;
	int err = cg->init_actuator(_actCommCfg, act_channel);
	cg->gimbal->setFrequency(actuator_freq);

	_f = boost::bind(&CameraGimbalControllerRos::cfgCallback, this, _1, _2);
  	_cfg_server.setCallback(_f);

	cmd_sub = m_nh.subscribe<std_msgs::Float32>(target_angle_topic, 50, boost::bind(&CameraGimbalControllerRos::cmdCallback,this,_1,0));
	imu_sub = m_nh.subscribe<geometry_msgs::Vector3>(imu_topic, 1000, boost::bind(&CameraGimbalControllerRos::imuCallback,this,_1,0));
	// data_pub = m_nh.advertise<swanson_msgs::DualClawInfo>(data_topic, 1000);

	_loop_rate = new ros::Rate(update_rate);
	usleep(2 * 1000000);
}

CameraGimbalControllerRos::~CameraGimbalControllerRos(){
	this->stop();
	delete this->cg;
	delete this->_loop_rate;
	usleep(1 * 10000000);
	// pigpio_stop(this->pi);
}

void CameraGimbalControllerRos::stop(){
	cg->goto_neutral_state();
}

void CameraGimbalControllerRos::cmdCallback(const std_msgs::Float32::ConstPtr& msg, const int topic_index){
	float target = msg->data / _maxAngle;
	cg->set_target_state(target);
	// printf("[INFO] CameraGimbalControllerRos::cmdCallback() ---- Recieved Cmds V,W: %.3f, %.3f\r\n",target_v,target_w);
}

void CameraGimbalControllerRos::imuCallback(const geometry_msgs::Vector3::ConstPtr& msg, const int topic_index){
	_curAngles[0] = msg->x;
	_curAngles[1] = msg->y;
	_curAngles[2] = msg->z;
	// printf("[INFO] CameraGimbalControllerRos::cmdCallback() ---- Recieved Cmds V,W: %.3f, %.3f\r\n",target_v,target_w);
	if(!_flag_start) _flag_start = true;
}

void CameraGimbalControllerRos::cfgCallback(swanson_controls::GimbalConfig &config, uint32_t level) {
	float maxAngle = config.max_angle;
     float targetAngle = config.target_angle;
	int nullPulse = config.null_cmd;
	float kp = config.kp;
	float ki = config.ki;
	float kd = config.kd;
	float target = targetAngle / maxAngle;
	ROS_INFO("Reconfigure Request: %.2f %.2f %.2f %d %.2f %.2f", kp, ki, kd, nullPulse, maxAngle, target);

	_maxAngle = maxAngle;
     _targetAngle = targetAngle;
	_nullPulse = nullPulse;
	_kp = kp;
	_ki = ki;
	_kd = kd;

	cg->set_p_gain(kp);			// Set P Gain
	cg->set_i_gain(ki);			// Set I Gain
	cg->set_d_gain(kd);			// Set D Gain
	cg->set_max_state(maxAngle);
	cg->set_null_cmd((float)nullPulse);
	cg->set_target_state(target);
}

void CameraGimbalControllerRos::update(bool verbose){
	_count++;
	/** Update Control Output */
	float angle = this->_curAngles[1];
	float normalized = angle / cg->get_max_state();
     float command = cg->calculate(normalized);
     command = command * (600.0) + cg->_params.null_cmd;
     cg->gimbal->setPulsewidth(cg->get_actuator_channel(),(int)command);
     float error = cg->get_integral_error();
     if(verbose) printf("[INFO] CameraGimbalControllerRos::update() --- Angle = %.2f | Command = %.2f | Error = %.3f\r\n",angle, command, error);

	// geometry_msgs::Pose2D poseMsg;
	// poseMsg.x = pose[0];
	// poseMsg.y = pose[1];
	// poseMsg.theta = pose[5];
	// dataMsg.estimated_pose = poseMsg;
	//
	// data_pub.publish(dataMsg);
	//
	// if(verbose){
		// 	printf("Motor Speeds (m/s):  %.3f | %.3f  | %.3f  | %.3f \r\n",spds[0],spds[1],spds[2],spds[3]);
		// 	printf("Encoder Positions (qpps): %d | %d | %d | %d\r\n",positions[0],positions[1],positions[2],positions[3]);
		// 	printf("Δdistance, ΔX, ΔY, ΔYaw: %.3f, %.3f, %.3f, %.3f\r\n",dOdom[0], dOdom[1],dOdom[2],dOdom[3]);
		// 	printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",pose[0],pose[1],pose[5]);
		// 	printf("Battery Voltages:     %.3f |    %.3f\r\n",voltages[0], voltages[1]);
		//      printf("Motor Currents:     %.3f |    %.3f |    %.3f |    %.3f\r\n",currents[0], currents[1], currents[2], currents[3]);
		// 	printf(" =========================================== \r\n");
		// }
	}

int CameraGimbalControllerRos::run(bool verbose){
     cout << "Looping..." << endl;
	this->stop();

     while(ros::ok()){
		if(_flag_start) this->update(verbose);
		// Update ROS index
          ros::spinOnce();
          _loop_rate->sleep();
     }

	this->stop();
     return 0;
}

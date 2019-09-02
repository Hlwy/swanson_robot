#include <iostream>
#include "swanson_base/dual_roboclaw_skidsteer_drivetrain.h"

using namespace std;

/** SECTION:
     CONSTRUCTOR & DECONSTRUCTOR
*/
DualClawSkidsteerDrivetrainInterface::DualClawSkidsteerDrivetrainInterface(ros::NodeHandle nh, ros::NodeHandle _nh) : m_nh(nh), p_nh(_nh){

     // Declare constants
	_count = 0;

	std::string ser_dev = "/dev/ttyS0";
	int ser_baud = 115200;
	float max_turn_radius = 0.381;
	float base_width = 0.219;
     float max_speed = 2.0;
	int qpps_per_meter = 9596;
	float wheel_diameter = 0.1905;

	std::string prefix;
	std::string cmd_vel_topic = "cmd_vel";
	std::string data_topic = "dualclaw/info";
	int left_claw_addr = 128;
	int right_claw_addr = 129;
	float update_rate = 20;

	p_nh.getParam("serial_device",ser_dev);
	p_nh.getParam("serial_baud",ser_baud);
	p_nh.getParam("left_claw_addr",left_claw_addr);
	p_nh.getParam("right_claw_addr",right_claw_addr);

	p_nh.getParam("max_speed",max_speed);
	p_nh.getParam("max_turn_radius",max_turn_radius);
	p_nh.getParam("base_width",base_width);
	p_nh.getParam("qpps_per_meter",qpps_per_meter);
	p_nh.getParam("wheel_diameter",wheel_diameter);

	p_nh.getParam("prefix",prefix);
	p_nh.getParam("velocity_topic",cmd_vel_topic);
	p_nh.getParam("data_topic",data_topic);
	p_nh.getParam("update_rate",update_rate);

	/* Connect to Pi. */
	int _pi = pigpio_start(NULL, NULL);
	if(_pi < 0){
		printf("[ERROR] Could not initialize with the pigpiod \r\n");
		exit(0);
	}else{ this->pi = _pi; }

	/** Initialize Dual RoboClaws */
	this->claws = new DualClaw(this->pi);
	int err = this->claws->init(ser_dev.c_str(), ser_baud, left_claw_addr, right_claw_addr);
	if(err < 0){
		printf("[ERROR] Could not establish serial communication with DualClaws. Error Code = %d\r\n", err);
		exit(0);
	}

	this->claws->set_base_width(base_width);
	this->claws->set_max_speed(max_speed);
	this->claws->set_qpps_per_meter(qpps_per_meter);
	this->claws->set_wheel_diameter(wheel_diameter);

	cmd_sub = m_nh.subscribe<geometry_msgs::Twist>(cmd_vel_topic, 50, boost::bind(&DualClawSkidsteerDrivetrainInterface::cmdCallback,this,_1,0));
	data_pub = m_nh.advertise<swanson_msgs::DualClawInfo>(data_topic, 1000);
	_loop_rate = new ros::Rate(update_rate);
	usleep(2 * 1000000);
}

DualClawSkidsteerDrivetrainInterface::~DualClawSkidsteerDrivetrainInterface(){
	delete this->claws;
	delete this->_loop_rate;
	usleep(1 * 10000000);
	pigpio_stop(this->pi);
}

void DualClawSkidsteerDrivetrainInterface::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg, const int topic_index){
	float target_v = msg->linear.x;
	float target_w = msg->angular.z;
	// printf("[INFO] DualClawSkidsteerDrivetrainInterface::cmdCallback() ---- Recieved Cmds V,W: %.3f, %.3f\r\n",target_v,target_w);
	claws->drive(target_v,target_w);
}


void DualClawSkidsteerDrivetrainInterface::update(bool verbose){
	_count++;
	claws->update_status();
     claws->update_encoders();

	vector<float> currents = claws->get_currents();
     vector<float> voltages = claws->get_voltages();
     vector<uint32_t> positions = claws->get_encoder_positions();
     vector<float> spds = claws->get_encoder_speeds();
     vector<float> dOdom = claws->get_odom_deltas();
     vector<float> pose = claws->get_pose();

	swanson_msgs::DualClawInfo dataMsg;
	dataMsg.header.stamp = ros::Time::now();
	dataMsg.header.seq = _count;

	dataMsg.left_roboclaw_voltage = voltages[0];
	dataMsg.right_roboclaw_voltage = voltages[1];
	dataMsg.qpps_per_meter = claws->get_qpps_per_meter();
	dataMsg.max_speed = claws->get_max_speed();
	dataMsg.wheel_diameter = claws->get_wheel_diameter();
	dataMsg.base_width = claws->get_base_width();
	dataMsg.left_motor_currents[0] = currents[0];
	dataMsg.left_motor_currents[1] = currents[1];
	dataMsg.right_motor_currents[0] = currents[2];
	dataMsg.right_motor_currents[1] = currents[3];
	dataMsg.left_motor_speeds[0] = spds[0];
	dataMsg.left_motor_speeds[1] = spds[1];
	dataMsg.right_motor_speeds[0] = spds[2];
	dataMsg.right_motor_speeds[1] = spds[3];
	dataMsg.left_motor_positions[0] = positions[0];
	dataMsg.left_motor_positions[1] = positions[1];
	dataMsg.right_motor_positions[0] = positions[2];
	dataMsg.right_motor_positions[1] = positions[3];

	geometry_msgs::Pose2D poseMsg;
	poseMsg.x = pose[0];
	poseMsg.y = pose[1];
	poseMsg.theta = pose[5];
	dataMsg.estimated_pose = poseMsg;

	data_pub.publish(dataMsg);

	if(verbose){
		printf("Motor Speeds (m/s):  %.3f | %.3f  | %.3f  | %.3f \r\n",spds[0],spds[1],spds[2],spds[3]);
		printf("Encoder Positions (qpps): %d | %d | %d | %d\r\n",positions[0],positions[1],positions[2],positions[3]);
		printf("Δdistance, ΔX, ΔY, ΔYaw: %.3f, %.3f, %.3f, %.3f\r\n",dOdom[0], dOdom[1],dOdom[2],dOdom[3]);
		printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",pose[0],pose[1],pose[5]);
		printf("Battery Voltages:     %.3f |    %.3f\r\n",voltages[0], voltages[1]);
	     printf("Motor Currents:     %.3f |    %.3f |    %.3f |    %.3f\r\n",currents[0], currents[1], currents[2], currents[3]);
		printf(" =========================================== \r\n");
	}
}

int DualClawSkidsteerDrivetrainInterface::run(bool verbose){
     cout << "Looping..." << endl;

     while(ros::ok()){
		this->update(verbose);

		// Update ROS index

          ros::spinOnce();
          _loop_rate->sleep();
     }

     return 0;
}

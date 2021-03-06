#include <iostream>
#include "swanson_base/dual_roboclaw_skidsteer_drivetrain.h"

using namespace std;

/** SECTION: Constructors / Deconstructors
*/
DualClawSkidsteerDrivetrainInterface::DualClawSkidsteerDrivetrainInterface(ros::NodeHandle nh, ros::NodeHandle _nh) : m_nh(nh), p_nh(_nh){
     // Declare constants
	_count = 0;
	_cmd_count = 0;
	_target_vel = 0.0;
     _target_rot = 0.0;
	_was_cmd_dir_flipped = false;
	_was_odom_dir_flipped = false;
	_ns = m_nh.getNamespace();

	/** Robot Hardware Interface */
	bool single_com_dev = false;
	std::string ser_dev_left = "/dev/ttyS0";
	std::string ser_dev_right = "/dev/ttyS1";
	int ser_baud = 115200;
	int left_claw_addr = 128;
	int right_claw_addr = 129;
	p_nh.getParam("use_single_serial_device",single_com_dev);
	p_nh.getParam("serial_baud",ser_baud);
	p_nh.getParam("left_serial_device",ser_dev_left);
	p_nh.getParam("right_serial_device",ser_dev_right);
	p_nh.getParam("left_claw_addr",left_claw_addr);
	p_nh.getParam("right_claw_addr",right_claw_addr);

	/** Robot Geometry Config */
	float max_speed = 2.0;
	float base_width = 0.219;
	int qpps_per_meter = 9596;
	float wheel_diameter = 0.1905;
	float max_turn_radius = 0.381;
	p_nh.getParam("max_speed",max_speed);
	p_nh.getParam("base_width",base_width);
	p_nh.getParam("qpps_per_meter",qpps_per_meter);
	p_nh.getParam("wheel_diameter",wheel_diameter);
	p_nh.getParam("max_turn_radius",max_turn_radius);
	this->_max_speed = max_speed;
	this->_base_width = base_width;
	this->_wheel_diameter = wheel_diameter;
	this->_max_turn_radius = max_turn_radius;
	this->_qpps_per_meter = qpps_per_meter;

	/** ROS Topic Config */
	std::string cmd_vel_topic = "cmd_vel";
	std::string data_topic = "dualclaw/info";
	std::string pose_topic = "dualclaw/pose";
	std::string odom_topic = "dead_reckoning";
	p_nh.getParam("velocity_topic",cmd_vel_topic);
	p_nh.getParam("data_topic",data_topic);
	p_nh.getParam("pose_topic",pose_topic);
	p_nh.getParam("odom_topic",odom_topic);

	/** ROS tf Config */
	bool verbose = false;
	bool flag_publish_tf = true;
	bool flag_use_tf_prefix = true;
	std::string tf_prefix = "";
	std::string odom_frame = "odom";
	std::string base_frame = "base_link";
	p_nh.getParam("verbose",verbose);
	p_nh.getParam("publish_tf",flag_publish_tf);
	p_nh.getParam("use_tf_prefix",flag_use_tf_prefix);
	p_nh.getParam("tf_prefix",tf_prefix);
	p_nh.getParam("odom_tf",odom_frame);
	p_nh.getParam("base_tf",base_frame);

	this->_verbose = verbose;
	this->_tf_prefix = tf_prefix;
	this->_publishTf = flag_publish_tf;
	if(flag_use_tf_prefix){
		this->_tf_odom = tf_prefix + odom_frame;
		this->_tf_base = tf_prefix + base_frame;
	} else{
		this->_tf_odom = odom_frame;
		this->_tf_base = base_frame;
	}

	/** ROS Misc Elements Config */
	int update_rate = 20;
	p_nh.getParam("update_rate",update_rate);

	/* Connect to Pi. */
	int _pi = pigpio_start(NULL, NULL);
	if(_pi < 0){
		printf("[ERROR] Could not initialize with the pigpiod \r\n");
		exit(0);
	}else{ this->pi = _pi; }

	/** Initialize Dual RoboClaws */
	this->claws = new DualClaw(this->pi);
	int err = 0;
	if(single_com_dev) err = this->claws->init(ser_dev_left.c_str(), ser_baud, left_claw_addr, right_claw_addr);
	else err = this->claws->init(ser_baud, ser_dev_left.c_str(), ser_dev_right.c_str(), left_claw_addr, right_claw_addr);

	if(err < 0){
		printf("[ERROR] Could not establish serial communication with DualClaws. Error Code = %d\r\n", err);
		exit(0);
	}

	this->claws->set_max_speed(max_speed);
	this->claws->set_base_width(base_width);
	this->claws->set_qpps_per_meter(qpps_per_meter);
	this->claws->set_wheel_diameter(wheel_diameter);
	// this->claws->set_max_turn_radius(max_turn_radius);

	/** Pre-initialize ROS messages */
	this->_odom_tf.header.frame_id = this->_tf_odom;
	this->_odom_tf.child_frame_id = this->_tf_base;
	this->_odom_tf.transform.translation.z = 0.0;

	this->_poseMsg.header.frame_id = this->_tf_odom;
	this->_poseMsg.pose.position.z = 0.0;

	this->_odomMsg.header.frame_id = this->_tf_odom;
	this->_odomMsg.child_frame_id = this->_tf_base;
	this->_odomMsg.pose.covariance[0] = 0.01;
	this->_odomMsg.pose.covariance[7] = 0.01;
	this->_odomMsg.pose.covariance[14] = 99999;
	this->_odomMsg.pose.covariance[21] = 99999;
	this->_odomMsg.pose.covariance[28] = 99999;
	this->_odomMsg.pose.covariance[35] = 0.01;
	this->_odomMsg.twist.twist.linear.y = 0.0;
	this->_odomMsg.twist.covariance = this->_odomMsg.pose.covariance;
	/** Initialize ROS Objects */
	cmd_sub = m_nh.subscribe<geometry_msgs::Twist>(cmd_vel_topic, 10, boost::bind(&DualClawSkidsteerDrivetrainInterface::cmdCallback,this,_1,0));
	data_pub = m_nh.advertise<swanson_msgs::DualClawInfo>(data_topic, 1);
	pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 1);
	odom_pub = m_nh.advertise<nav_msgs::Odometry>(odom_topic, 1);

	_reset_enc = m_nh.advertiseService("reset_odom", &DualClawSkidsteerDrivetrainInterface::reset_odometry, this);
	this->_cfg_f = boost::bind(&DualClawSkidsteerDrivetrainInterface::cfgCallback, this, _1, _2);
	this->_cfg_server.setCallback(this->_cfg_f);

	_loop_rate = new ros::Rate(update_rate);
}
DualClawSkidsteerDrivetrainInterface::~DualClawSkidsteerDrivetrainInterface(){
	printf("[INFO] Shutting Down DualClawSkidsteerDrivetrainInterface...\r\n");
	delete this->claws;
	delete this->_loop_rate;
	// usleep(1 * 10000000);
	pigpio_stop(this->pi);
}

/** SECTION: ROS Callbacks
*
*/
bool DualClawSkidsteerDrivetrainInterface::reset_odometry(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	ROS_DEBUG("Resetting Encoders");
	this->claws->reset_odometry();
	return true;
}
void DualClawSkidsteerDrivetrainInterface::cfgCallback(swanson_base::DrivetrainConfig &config, uint32_t level){
	int cmd_dir = 1, odom_dir = 1;
	/** Flip the commanded angular direction only if previously flipped */
	if(config.flip_cmd_turn_direction){
		cmd_dir = -1;
		this->claws->set_command_turn_direction(cmd_dir);
		this->_was_cmd_dir_flipped = true;
	} else{
		if(this->_was_cmd_dir_flipped){
			cmd_dir = 1;
			this->claws->set_command_turn_direction(cmd_dir);
			this->_was_cmd_dir_flipped = false;
		}
	}

	/** Flip the sensed angular direction only if previously flipped */
	if(config.flip_sensed_turn_direction){
		odom_dir = -1;
		this->claws->set_odom_turn_direction(odom_dir);
		this->_was_odom_dir_flipped = true;
	} else{
		if(this->_was_odom_dir_flipped){
			odom_dir = 1;
			this->claws->set_odom_turn_direction(odom_dir);
			this->_was_odom_dir_flipped = false;
		}
	}

	/** Update Variables only if they are different from current settings */
	if(config.max_speed != this->_max_speed){
		this->claws->set_max_speed(config.max_speed);
		this->_max_speed = config.max_speed;
	}
	if(config.base_width != this->_base_width){
		this->claws->set_base_width(config.base_width);
		this->_base_width = config.base_width;
	}
	if(config.wheel_diameter != this->_wheel_diameter){
		this->claws->set_wheel_diameter(config.wheel_diameter);
		this->_wheel_diameter = config.wheel_diameter;
	}
	if(config.max_turn_radius != this->_max_turn_radius){
		// this->claws->set_max_turn_radius(config.max_turn_radius);
		this->_max_turn_radius = config.max_turn_radius;
	}
	if(config.qpps_per_meter != this->_qpps_per_meter){
		this->claws->set_qpps_per_meter(config.qpps_per_meter);
		this->_qpps_per_meter = config.qpps_per_meter;
	}

	ROS_DEBUG("Reconfigure Request: \r\n\tMax Vel = %.3f \r\n\tBaseWidth = %.3f \r\n\tWheel Diameter = %.3f \r\n\tMax Turn Radius = %.3f \r\n\tQPPS/m = %d \r\n\tCmd Dir = %d \r\n\tSensed Dir = %d",
               config.max_speed, config.base_width,
               config.wheel_diameter, config.max_turn_radius,
               config.qpps_per_meter, cmd_dir, odom_dir
	);
}
void DualClawSkidsteerDrivetrainInterface::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg, const int topic_index){
	std::lock_guard<std::mutex> lock(this->_lock);
	float target_v = msg->linear.x;
	float target_w = msg->angular.z;
	this->claws->drive(target_v,target_w);
	this->_cmd_count++;
}

/** SECTION: Exection Functions
*
*/
void DualClawSkidsteerDrivetrainInterface::update(){
	_count++;
	ros::Time curTime = ros::Time::now();

	this->_lock.lock();
	this->claws->update_status();
	this->claws->update_odometry();
	float ppm = this->claws->get_qpps_per_meter();
	float wheelbase = this->claws->get_base_width();
	float wheel_diamter = this->claws->get_wheel_diameter();
	float max_claw_speed = this->claws->get_max_speed();
	vector<uint32_t> positions = this->claws->get_encoder_positions();
	vector<float> spds = this->claws->get_motor_speeds();
	vector<float> dOdom = this->claws->get_odom_deltas();
	vector<float> pose = this->claws->get_pose();
	vector<float> vels = this->claws->get_velocities();
	vector<float> currents = this->claws->get_currents();
	vector<float> voltages = this->claws->get_voltages();
	this->_lock.unlock();

	/** Update Roboclaw Data */
	swanson_msgs::DualClawInfo dataMsg;
	dataMsg.header.seq = _count;
	dataMsg.header.stamp = curTime;
	dataMsg.left_roboclaw_voltage = voltages[0];
	dataMsg.right_roboclaw_voltage = voltages[1];
	dataMsg.qpps_per_meter = ppm;
	dataMsg.base_width = wheelbase;
	dataMsg.wheel_diameter = wheel_diamter;
	dataMsg.max_speed = max_claw_speed;
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
	dataMsg.estimated_pose.x = pose[0];
	dataMsg.estimated_pose.y = pose[1];
	dataMsg.estimated_pose.theta = pose[2];
	data_pub.publish(dataMsg);

	/** Update Robot's body transformations */
	geometry_msgs::Quaternion quats = tf::createQuaternionMsgFromYaw(pose[2]);

	this->_odom_tf.header.seq = _count;
	this->_odom_tf.header.stamp = curTime;
	this->_odom_tf.transform.translation.x = pose[0];
	this->_odom_tf.transform.translation.y = pose[1];
	this->_odom_tf.transform.rotation = quats;
	if(this->_publishTf) this->_br.sendTransform(this->_odom_tf);

	/** Update Robot's dead-reckoned pose */
	this->_poseMsg.header.seq = _count;
	this->_poseMsg.header.stamp = curTime;
	this->_poseMsg.pose.position.x = pose[0];
	this->_poseMsg.pose.position.y = pose[1];
	this->_poseMsg.pose.orientation = quats;
	this->pose_pub.publish(this->_poseMsg);

	/** Update Robot's dead-reckoned odometry */
	this->_odomMsg.header.seq = _count;
	this->_odomMsg.header.stamp = curTime;
	this->_odomMsg.pose.pose = this->_poseMsg.pose;
	this->_odomMsg.twist.twist.linear.x = vels[0];
	this->_odomMsg.twist.twist.angular.z = vels[1];
	this->odom_pub.publish(this->_odomMsg);

	if(this->_verbose){
		printf("Motor Speeds (m/s):  %.3f | %.3f  | %.3f  | %.3f \r\n",spds[0],spds[1],spds[2],spds[3]);
		printf("Encoder Positions (qpps): %d | %d | %d | %d\r\n",positions[0],positions[1],positions[2],positions[3]);
		printf("Δdistance, ΔYaw, ΔX, ΔY,: %.3f, %.3f, %.3f, %.3f\r\n",dOdom[0], dOdom[1],dOdom[2],dOdom[3]);
		printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",pose[0],pose[1],pose[2]);
		printf("Battery Voltages:     %.3f |    %.3f\r\n",voltages[0], voltages[1]);
		printf("Motor Currents:     %.3f |    %.3f |    %.3f |    %.3f\r\n",currents[0], currents[1], currents[2], currents[3]);
		printf(" =========================================== \r\n");
	}
}
int DualClawSkidsteerDrivetrainInterface::run(bool verbose){
	cout << "Looping..." << endl;
	int curCount = 0;
	while(ros::ok()){
		this->update();
		ros::spinOnce();
		this->_loop_rate->sleep();
	}
	return 0;
}

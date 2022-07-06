#include <iostream>
#include "swanson_base/diffdrive_roboclaw_skidsteer_drivetrain.h"

using namespace std;

/** SECTION: Constructors / Deconstructors
*/
DiffDriveClawSkidsteerDrivetrainInterface::DiffDriveClawSkidsteerDrivetrainInterface(ros::NodeHandle nh, ros::NodeHandle _nh) : m_nh(nh), p_nh(_nh){
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
    /**
    std::string ser_dev_left = "/dev/ttyS0";
    std::string ser_dev_right = "/dev/ttyS1";
     **/
    std::string ser_dev = "/dev/ttyACM0";
    int ser_baud = 115200;
    /**
    int left_claw_addr = 128;
    int right_claw_addr = 129;
     **/
    int claw_addr = 128;
    int doack = false;
    int timeout = 1000;
    p_nh.getParam("use_single_serial_device",single_com_dev);
    p_nh.getParam("serial_baud",ser_baud);
    /**
    p_nh.getParam("left_serial_device",ser_dev_left);
    p_nh.getParam("right_serial_device",ser_dev_right);
    p_nh.getParam("left_claw_addr",left_claw_addr);
    p_nh.getParam("right_claw_addr",right_claw_addr);
     **/
    p_nh.getParam("serial_device",ser_dev);
    p_nh.getParam("claw_addr",claw_addr);
    p_nh.getParam("doack", doack);
    p_nh.getParam("timeout", timeout);

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
    /**
    std::string data_topic = "dualclaw/info";
    std::string pose_topic = "dualclaw/pose";
     **/
    std::string data_topic = "diffdrive_claw/info";
    std::string pose_topic = "diffdrive_claw/pose";
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
//    int _pi = pigpio_start(NULL, NULL);
//    if(_pi < 0){
//        printf("[ERROR] Could not initialize with the pigpiod \r\n");
//        exit(0);
//    }else{ this->pi = _pi; }

    /** Initialize DiffDrive RoboClaw */
    /**this->claws = new DualClaw(this->pi);*/
    this->claw = new DiffDriveClaw();
    int err = 0;
    if(single_com_dev) err = this->claw->init(doack, ser_dev.c_str(), claw_addr, timeout);
    else err = this->claw->init(doack, ser_dev.c_str(), claw_addr, timeout);

    if(err < 0){
        printf("[ERROR] Could not establish serial communication with DualClaws. Error Code = %d\r\n", err);
        exit(0);
    }

    this->claw->set_max_speed(max_speed);
    this->claw->set_base_width(base_width);
    this->claw->set_qpps_per_meter(qpps_per_meter);
    this->claw->set_wheel_diameter(wheel_diameter);
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
    cmd_sub = m_nh.subscribe<geometry_msgs::Twist>(cmd_vel_topic, 10, boost::bind(&DiffDriveClawSkidsteerDrivetrainInterface::cmdCallback,this,_1,0));
    data_pub = m_nh.advertise<swanson_msgs::DiffDriveClawInfo>(data_topic, 1);
    pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 1);
    odom_pub = m_nh.advertise<nav_msgs::Odometry>(odom_topic, 1);

    _reset_enc = m_nh.advertiseService("reset_odom", &DiffDriveClawSkidsteerDrivetrainInterface::reset_odometry, this);
    this->_cfg_f = boost::bind(&DiffDriveClawSkidsteerDrivetrainInterface::cfgCallback, this, _1, _2);
    this->_cfg_server.setCallback(this->_cfg_f);

    _loop_rate = new ros::Rate(update_rate);
}
DiffDriveClawSkidsteerDrivetrainInterface::~DiffDriveClawSkidsteerDrivetrainInterface(){
    printf("[INFO] Shutting Down DiffDriveClawSkidsteerDrivetrainInterface...\r\n");
    delete this->claw;
    delete this->_loop_rate;
    // usleep(1 * 10000000);
    //pigpio_stop(this->pi);
}

/** SECTION: ROS Callbacks
*
*/
bool DiffDriveClawSkidsteerDrivetrainInterface::reset_odometry(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ROS_DEBUG("Resetting Encoders");
    this->claw->reset_odometry();
    return true;
}
void DiffDriveClawSkidsteerDrivetrainInterface::cfgCallback(swanson_base::DrivetrainConfig &config, uint32_t level){
    int cmd_dir = 1, odom_dir = 1;
    /** Flip the commanded angular direction only if previously flipped */
    if(config.flip_cmd_turn_direction){
        cmd_dir = -1;
        this->claw->set_command_turn_direction(cmd_dir);
        this->_was_cmd_dir_flipped = true;
    } else{
        if(this->_was_cmd_dir_flipped){
            cmd_dir = 1;
            this->claw->set_command_turn_direction(cmd_dir);
            this->_was_cmd_dir_flipped = false;
        }
    }

    /** Flip the sensed angular direction only if previously flipped */
    if(config.flip_sensed_turn_direction){
        odom_dir = -1;
        this->claw->set_odom_turn_direction(odom_dir);
        this->_was_odom_dir_flipped = true;
    } else{
        if(this->_was_odom_dir_flipped){
            odom_dir = 1;
            this->claw->set_odom_turn_direction(odom_dir);
            this->_was_odom_dir_flipped = false;
        }
    }

    /** Update Variables only if they are different from current settings */
    if(config.max_speed != this->_max_speed){
        this->claw->set_max_speed(config.max_speed);
        this->_max_speed = config.max_speed;
    }
    if(config.base_width != this->_base_width){
        this->claw->set_base_width(config.base_width);
        this->_base_width = config.base_width;
    }
    if(config.wheel_diameter != this->_wheel_diameter){
        this->claw->set_wheel_diameter(config.wheel_diameter);
        this->_wheel_diameter = config.wheel_diameter;
    }
    if(config.max_turn_radius != this->_max_turn_radius){
        // this->claws->set_max_turn_radius(config.max_turn_radius);
        this->_max_turn_radius = config.max_turn_radius;
    }
    if(config.qpps_per_meter != this->_qpps_per_meter){
        this->claw->set_qpps_per_meter(config.qpps_per_meter);
        this->_qpps_per_meter = config.qpps_per_meter;
    }

    ROS_DEBUG("Reconfigure Request: \r\n\tMax Vel = %.3f \r\n\tBaseWidth = %.3f \r\n\tWheel Diameter = %.3f \r\n\tMax Turn Radius = %.3f \r\n\tQPPS/m = %d \r\n\tCmd Dir = %d \r\n\tSensed Dir = %d",
              config.max_speed, config.base_width,
              config.wheel_diameter, config.max_turn_radius,
              config.qpps_per_meter, cmd_dir, odom_dir
    );
}
void DiffDriveClawSkidsteerDrivetrainInterface::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg, const int topic_index){
    std::lock_guard<std::mutex> lock(this->_lock);
    float target_v = msg->linear.x;
    float target_w = msg->angular.z;
    this->claw->drive(target_v,target_w);
    this->_cmd_count++;
}

/** SECTION: Exection Functions
*
*/
void DiffDriveClawSkidsteerDrivetrainInterface::update(){
    _count++;
    ros::Time curTime = ros::Time::now();

    this->_lock.lock();
    this->claw->update_status(true);
//    cout<<"Claw Update Status Complete"<<endl;
    this->claw->update_odometry(true);
//    cout<<"Claw Update Odometry Complete"<<endl;
    float ppm = this->claw->get_qpps_per_meter();
    float wheelbase = this->claw->get_base_width();
    float wheel_diamter = this->claw->get_wheel_diameter();
    float max_claw_speed = this->claw->get_max_speed();
    vector<uint32_t> positions = this->claw->get_encoder_positions();
    vector<float> spds = this->claw->get_motor_speeds();
    vector<float> dOdom = this->claw->get_odom_deltas();
    vector<float> pose = this->claw->get_pose();
    vector<float> vels = this->claw->get_velocities();
    vector<float> currents = this->claw->get_currents();
//    for (float i: currents) {
//        std::cout << i << ' ';
//    }
    float voltage = this->claw->get_voltage();
//    cout<<"Claw Param Get Complete"<<endl;
    this->_lock.unlock();

    /** Update Roboclaw Data */
    swanson_msgs::DiffDriveClawInfo dataMsg;
    dataMsg.header.seq = _count;
    dataMsg.header.stamp = curTime;
//    cout<<"Header Configured"<<endl;
    /**
    dataMsg.left_roboclaw_voltage = voltages[0];
    dataMsg.right_roboclaw_voltage = voltages[1];**/
    dataMsg.roboclaw_voltage = voltage;
    dataMsg.qpps_per_meter = ppm;
    dataMsg.base_width = wheelbase;
    dataMsg.wheel_diameter = wheel_diamter;
    dataMsg.max_speed = max_claw_speed;
//    cout<<"Robot Status Set"<<endl;
    /**
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
     **/
    dataMsg.left_motor_current = currents[0];
    dataMsg.right_motor_current = currents[1];
//    cout << "Currents Configured" << endl;
    dataMsg.left_motor_speed = spds[0];
    dataMsg.right_motor_speed =spds[1];
//    cout << "Motor Speeds Configured" << endl;
    dataMsg.left_motor_position = positions[0];
    dataMsg.right_motor_position = positions[1];
//    cout << "Motor Positions Configured" << endl;
    dataMsg.estimated_pose.x = pose[0];
    dataMsg.estimated_pose.y = pose[1];
    dataMsg.estimated_pose.theta = pose[2];
//    cout << "Finished constructing message" << endl;
    data_pub.publish(dataMsg);

//    cout << "Published DiffDriveClaw Info" << endl;

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
        printf("Motor Speeds (m/s):  %.3f | %.3f\r\n",spds[0],spds[1]);
        printf("Encoder Positions (qpps): %d | %d\r\n",positions[0],positions[1]);
        printf("Δdistance, ΔX, ΔY, ΔYaw: %.3f, %.3f, %.3f, %.3f\r\n",dOdom[0], dOdom[1],dOdom[2],dOdom[3]);
        printf("Current Pose [X (m), Y (m), Yaw (rad)]: %.3f     |    %.3f   |       %.3f\r\n",pose[0],pose[1],pose[2]);
        printf("Battery Voltages:     %.3f\r\n",voltage);
        printf("Motor Currents:     %.3f |    %.3f\r\n",currents[0], currents[1]);
        printf(" =========================================== \r\n");
    }
}
int DiffDriveClawSkidsteerDrivetrainInterface::run(bool verbose){
    cout << "Looping..." << endl;
    int curCount = 0;
    while(ros::ok()){
//        cout <<"In Loop" << endl;
        this->update();
//        cout<<"Finish Update"<<endl;
        ros::spinOnce();
//        cout<<"Spinning..."<<endl;
        this->_loop_rate->sleep();
//        cout<<"Sleep"<<endl;
    }
    return 0;
}

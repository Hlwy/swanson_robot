#include <iostream>
#include "swanson_algorithms/angle_test.h"
#include <Eigen/Geometry>

using namespace std;

/** SECTION:
     CONSTRUCTOR & DECONSTRUCTOR
*/
AngleTestRos::AngleTestRos(ros::NodeHandle nh, ros::NodeHandle _nh) : m_nh(nh), p_nh(_nh){
     // Flag Configuration
     int update_rate = 30;
     bool flag_use_tf_prefix = false;
     std::string namespaced = m_nh.getNamespace();
     p_nh.getParam("namespace",namespaced);
     p_nh.getParam("update_rate",update_rate);
     this->_ns = namespaced;

     std::string imu_topic, pose_topic, pose_stamped_topic;
     p_nh.param<std::string>("imu_topic",          imu_topic, "mavros/imu/data");
     p_nh.param<std::string>("pose_topic",         pose_topic, "");
     p_nh.param<std::string>("pose_stamped_topic", pose_stamped_topic, "");
     this->_imu_sub = m_nh.subscribe<sensor_msgs::Imu>(imu_topic, 30, &AngleTestRos::imuCallback,this);
     // Initialize ROS-Objects
     // if(strcmp(pose_stamped_topic.c_str(), "") != 0){
     //      // this->_pose_stamped_sub = m_nh.subscribe<geometry_msgs::PoseStamped>(pose_stamped_topic, 30, &AngleTestRos::poseStampedCallback,this);
     // } else if(strcmp(imu_topic.c_str(), "") != 0){
     // }

     this->_loop_rate = new ros::Rate(update_rate);
}
AngleTestRos::~AngleTestRos(){ delete this->_loop_rate; }

void AngleTestRos::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
     std::lock_guard<std::mutex> lock(_lock);

     double roll, pitch, yaw;
     // tf::Quaternion quat;
     // geometry_msgs::Quaternion orientMsg = msg->orientation;
     // tf::quaternionMsgToTF(orientMsg, quat);
	// tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

     // Eigen::Quaternion<double> quat(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
     // Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2,1,0);
     // roll  = (double) euler[2];
     // pitch = (double) euler[1];
     // yaw   = (double) euler[0];

     Eigen::Quaternion<double> q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
     // double y2 = q.y() * q.y();
     // double rz = std::atan2(2*(q.w()*q.z() + q.x()*q.y()), (1 - 2*(y2 + q.z()*q.z())));
     // double ry = std::asin( 2*(q.w()*q.y() - q.z()*q.x()));
     // double rx = std::atan2(2*(q.w()*q.x() + q.y()*q.z()), (1 - 2*(q.x()*q.x() + y2)));
     // Eigen::Vector3d euler(rx, ry, rz);

     std::vector<double> euler = toEulerAngles(q.x(), q.y(), q.z(), q.w());

     roll  = (double) euler[0];
     pitch = (double) euler[1];
     yaw   = (double) euler[2];

     double pi2 = (2.0*M_PI);
     double roll_wrapped  = roll;
     double pitch_wrapped = pitch;
     double yaw_wrapped   = yaw;

     roll_wrapped   -= pi2 * std::floor(roll_wrapped * (1.0 / pi2) );
     pitch_wrapped  -= pi2 * std::floor(pitch_wrapped * (1.0 / pi2) );
     yaw_wrapped    -= pi2 * std::floor(yaw_wrapped * (1.0 / pi2) );

     printf("[INFO] RPY (deg):\r\n"
            "      Raw     --- (%.2lf, %.2lf, %.2lf)\r\n", roll*M_RAD2DEG, pitch*M_RAD2DEG, yaw*M_RAD2DEG);
     printf("      Wrapped --- (%.2lf, %.2lf, %.2lf)\r\n", roll_wrapped*M_RAD2DEG, pitch_wrapped*M_RAD2DEG, yaw_wrapped*M_RAD2DEG);
}

int AngleTestRos::run(){
     while(ros::ok()){
          ros::spinOnce();
          this->_loop_rate->sleep();
     }
     return 0;
}

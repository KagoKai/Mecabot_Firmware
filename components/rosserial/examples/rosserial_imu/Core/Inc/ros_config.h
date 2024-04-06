/*
 * ros_config.h
 *
 *  Created on: Mar 29, 2024
 *      Author: ADMIN
 */

#ifndef INC_ROS_CONFIG_H_
#define INC_ROS_CONFIG_H_

#include "ros.h"
#include "ros/time.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

#define IMU_PUBLISH_FREQUENCY		50
#define MOTOR_CONTROL_FREQUENCY		10

/* CALLBACK FUNCTIONS START */
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
/* CALLBACK FUNCTIONS END */

/* MSG INITIALIZATION FUNCTIONS START */
void initOdom(void);
void initJointStates(void);
/* MSG INITIALIZATION FUNCTIONS END */

/* MSG UPDATE FUNCTIONS START */
void updateImu(void);
void updateOdometry(void);
void updateJointStates(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
bool calculateOdometry(float diff_time);
/* MSG UPDATE FUNCTIONS END */

/* PUBLISHING FUNCTIONS START */
void publishImuMsg(void);
void publishDriveInfo(void);
/* PUBLISHING FUNCTIONS END */

/* SETUP FUNCTIONS START */
void ros_setup(void);
/* SETUP FUNCTIONS END */

/* GLOBAL VARIABLES START */

/*
 * ROS NodeHandle
 */
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*
 * ROS parameters
 */
char odom_header_frame_id[] = "/odom";
char odom_child_frame_id[] = "/base_link";
char imu_frame_id[] = "/imu";
char joint_states_frame_id[] = "/joint_states"
char *joint_states_name = {
		"front_left_wheel_joint",
		"front_right_wheel_joint",
		"back_left_wheel_joint",
		"back_right_wheel_joint"
};

/*
 * Subscribers
 */
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &commandVelocityCallback);

/*
 * Publishers
 */
sensor_msgs::Imu imu_msg;
ros::Publisher pub_imu("imu", &imu_msg);
nav_msgs::Odometry odom_msg;
ros::Publisher pub_odom("odom", &odom_msg);
sensor_msgs::JointState joint_states_msg;
ros::Publisher pub_joint_states("joint_states", &joint_states_msg);

/*
 * TF broadcaster
 */
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/*
 * Odometry
 */
float odom_pose[3];
float odom_vel[3];

/* GLOBAL VARIABLES END */

#endif /* INC_ROS_CONFIG_H_ */

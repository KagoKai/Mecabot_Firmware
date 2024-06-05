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
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float64.h"

#define MOTOR_CONTROL_FREQUENCY		10
#define RPM_PUBLISH_FREQUENCY		20

/* CALLBACK FUNCTIONS START */
void motorVelCallback(const std_msgs::UInt8& motor_speed_msg);
void pGainCallback(const std_msgs::Float64& kp_msg);
void iGainCallback(const std_msgs::Float64& ki_msg);
void dGainCallback(const std_msgs::Float64& kd_msg);
/* CALLBACK FUNCTIONS END */

/* MSG INITIALIZATION FUNCTIONS START */
/* MSG INITIALIZATION FUNCTIONS END */

/* MSG UPDATE FUNCTIONS START */
void updateRpm(void);
/* MSG UPDATE FUNCTIONS END */

/* PUBLISHING FUNCTIONS START */
void publishRpmMsg(void);
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

/*
 * Subscribers
 */
ros::Subscriber<std_msgs::UInt8> sub_motor_speed("/motor_speed", &motorVelCallback);
ros::Subscriber<std_msgs::Float64> sub_P_gain("/controller_kp", &pGainCallback);
ros::Subscriber<std_msgs::Float64> sub_I_gain("/controller_ki", &pGainCallback);
ros::Subscriber<std_msgs::Float64> sub_D_gain("/controller_kd", &pGainCallback);

/*
 * Publishers
 */
std_msgs::UInt32 rpm_msg;
ros::Publisher pub_motor_rpm("/motor_rpm", &rpm_msg);

/* GLOBAL VARIABLES END */

#endif /* INC_ROS_CONFIG_H_ */
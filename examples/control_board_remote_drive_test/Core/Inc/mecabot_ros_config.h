/*
 * mecabot_ros_config.h
 *
 *  Created on: May 25, 2024
 *      Author: ADMIN
 */

#ifndef INC_MECABOT_ROS_CONFIG_H_
#define INC_MECABOT_ROS_CONFIG_H_

#include "ros.h"
#include "ros/time.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "geometry_msgs/Twist.h"

#define NUM_OF_MOTOR                	4
#define MOTOR_CONTROL_FREQUENCY		    10
#define VEL_FEEDBACK_FREQUENCY        	20

/* CALLBACK FUNCTIONS START */
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void motorDutyCallback(const std_msgs::UInt8& motor_duty_msg);
/* CALLBACK FUNCTIONS END */

/* SETUP FUNCTIONS START */
void ros_setup(void);
/* SETUP FUNCTIONS END */

/* DATA HANDLE FUNCTIONS START */
void calculateWheelVelocity(void);     // Calculate individual wheel velocity (rad/s) using inverse kinematics.
void calculateRpm(void);			   // Calculate the motors' velocity
/* DATA HANDLE FUNCTION END */

/* CONTROL FUNCTIONS START */
void controlMotors(void);
/* CONTROL FUNCTIONS START */

/* PUBLISH FUNCTIONS START */
void publishRpm(void);
/* PUBLISH FUNCTIONS START */

/* GLOBAL VARIABLES START */

/*
 * ROS NodeHandle
 */
ros::NodeHandle nh;

/*
 * ROS parameters
 */

/*
 * Subscribers
 */
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", &commandVelocityCallback);
ros::Subscriber<std_msgs::UInt8> sub_duty("/motor_duty", &motorDutyCallback);
/*
 * Publishers
 */
std_msgs::UInt16 rpm_msg[NUM_OF_MOTOR];
//ros::Publisher pub_FL_rpm("/FL_rpm", &rpm_msg[front_left]);
//ros::Publisher pub_FR_rpm("/FR_rpm", &rpm_msg[front_right]);
//ros::Publisher pub_BL_rpm("/BL_rpm", &rpm_msg[back_left]);
//ros::Publisher pub_BR_rpm("/BR_rpm", &rpm_msg[back_right]);

/*
* Control variables
*/
typedef enum
{
    linear_x = 0,
    linear_y,
    angular_z
}odom_component_t;

float goal_vel[3] = { 0.0f, 0.0f, 0.0f };						   	// Robot velocity received from cmd_vel (m/s)
float wheel_angular_vel[NUM_OF_MOTOR] = { 0.0f, 0.0f, 0.0f, 0.0f };      	// Calculated from goal velocity (rad/s)
float meas_wheel_angular_vel[NUM_OF_MOTOR] = { 0.0f }; 	// Actual speed calculated from encoder ticks

uint8_t motor_duty[NUM_OF_MOTOR] = { 0 };

/*
* Time-related variables
*/
typedef enum
{
    motor_control_event = 0,
    wheel_velocity_feedback_event,
}time_index_t;
uint32_t t_previous[2] = { 0 };
uint32_t dt[2] = { 1000 / MOTOR_CONTROL_FREQUENCY, 1000 / VEL_FEEDBACK_FREQUENCY };

/* GLOBAL VARIABLES END */

#endif /* INC_MECABOT_ROS_CONFIG_H_ */
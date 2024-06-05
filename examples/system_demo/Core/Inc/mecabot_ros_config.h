/*
 * mecabot_ros_config.h
 *
 *  Created on: May 15, 2024
 *      Author: ADMIN
 */

#ifndef INC_MECABOT_ROS_CONFIG_H_
#define INC_MECABOT_ROS_CONFIG_H_


#include <cstdint>
#include "ros.h"
#include "ros/time.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt32.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_OF_MOTOR                4

#define MOTOR_CONTROL_FREQUENCY		    100
#define VEL_FEEDBACK_FREQUENCY        5
#define IMU_PUBLISH_FREQUENCY           10
#define ROBOT_INFO_PUBLISH_FREQUENCY    5
#define RPM_PUBLISH_FREQUENCY		    5

/* CALLBACK FUNCTIONS START */
void motorVelCallback(const std_msgs::UInt8& motor_speed_msg);
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
/* CALLBACK FUNCTIONS END */

/* SETUP FUNCTIONS START */
void ros_setup(void);
/* SETUP FUNCTIONS END */

/* MSG INITIALIZATION FUNCTIONS START */
void initOdom(void);
void initJointStates(void);
void initTF(void);
/* MSG INITIALIZATION FUNCTIONS END */

/* MSG UPDATE FUNCTIONS START */
void updateImu(void);
void updateOdometry(void);
void updateJointStates(void);
void updateTF(void);
/* MSG UPDATE FUNCTIONS END */

/* PUBLISHING FUNCTIONS START */
void publishImu(void);
void publishRobotState(void);  // Update odometry, tf, and joint states
/* PUBLISHING FUNCTIONS END */

/* DATA HANDLE FUNCTIONS START */
void calculateOdometry(void);       // Calculate odometry from motor speed using kinematics.
void calculateWheelVelocity(void);     // Calculate individual wheel velocity (rad/s) using inverse kinematics.
/* DATA HANDLE FUNCTION END */

void controlMotors(void);

/* GLOBAL VARIABLES START */

/*
 * ROS NodeHandle
 */
ros::NodeHandle nh;

/*
 * ROS parameters
 */
char *joint_states_name[NUM_OF_MOTOR] {"front_left_wheel_joint", "front_right_wheel_joint",
                 	 	 	 	 	   "back_left_wheel_joint" , "back_right_wheel_joint"};

/*
 * Subscribers
 */
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", &commandVelocityCallback);

/*
 * Publishers
 */
sensor_msgs::Imu imu_msg;
ros::Publisher pub_imu("/imu", &imu_msg);
sensor_msgs::JointState joint_states_msg;
ros::Publisher pub_joint_states("/joint_states", &joint_states_msg);
nav_msgs::Odometry odom_msg;
ros::Publisher pub_odom("/odom", &odom_msg);

/*
* TF broadcaster
*/
geometry_msgs::TransformStamped odom_tf_msg; // tf information between "/odom" and "/baselink"
tf::TransformBroadcaster tf_broadcaster;

/*
* Control variables
*/
typedef enum
{
    linear_x = 0,
    linear_y,
    angular_z
}odom_component_t;
// Calculated from odometry
float odom_pose[3] = { 0.0 };
float odom_vel[3] = { 0.0 };
// Receive from cmd_vel
float goal_vel[3] = { 0.0 };
// Actual motor speed measurement

float wheel_angular_vel[NUM_OF_MOTOR] = { 0.0f };      // Calculated from goal velocity (rad/s)
float meas_wheel_angular_vel[NUM_OF_MOTOR] = { 0.0f }; // Actual speed calculated from encoder ticks

uint16_t prev_encoder_tick[NUM_OF_MOTOR] = { 0 };
uint16_t diff_tick[NUM_OF_MOTOR] = { 0 };


uint8_t motor_duty[NUM_OF_MOTOR] = { 0 };

/*
* Time-related variables
*/
typedef enum
{
    motor_control_event = 0,
    wheel_velocity_feedback_event,
    imu_publish_event,
    robot_info_publish_event
}time_index_t;
uint32_t t_previous[4] = { 0 };
uint32_t dt[4] = { 1 / MOTOR_CONTROL_FREQUENCY, 1 / VEL_FEEDBACK_FREQUENCY, 1 / IMU_PUBLISH_FREQUENCY, 1 / ROBOT_INFO_PUBLISH_FREQUENCY };

/* GLOBAL VARIABLES END */

#ifdef __cplusplus
}
#endif

#endif /* INC_MECABOT_ROS_CONFIG_H_ */

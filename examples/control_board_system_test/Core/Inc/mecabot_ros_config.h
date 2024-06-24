/*
 * mecabot_ros_config.h
 *
 *  Created on: May 28, 2024
 *      Author: ADMIN
 */

#ifndef INC_MECABOT_ROS_CONFIG_H_
#define INC_MECABOT_ROS_CONFIG_H_

#include <cstdint>
#include "ros.h"
#include "ros/time.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_OF_MOTOR                	4

#define MOTOR_CONTROL_FREQUENCY		    10
#define VEL_FEEDBACK_FREQUENCY        	10
#define IMU_PUBLISH_FREQUENCY			20
#define ODOM_PUBLISH_FREQUENCY			10

/* CALLBACK FUNCTIONS START */
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void enablePidCallback(const std_msgs::UInt8& use_pid_msg);
void kpTuneCallback(const std_msgs::Float32& kp_msg);
void kiTuneCallback(const std_msgs::Float32& ki_msg);
void kdTuneCallback(const std_msgs::Float32& kd_msg);
void testIdCallback(const std_msgs::Int8& test_id_msg);
/* CALLBACK FUNCTIONS END */

/* SETUP FUNCTIONS START */
void ros_setup(void);
/* SETUP FUNCTIONS END */

/* MSG INITIALIZATION FUNCTIONS START */
void initOdom(void);				// Initialize the Odometry message.
void initJointStates(void);			// Initialize the joint states (for </robot_state_publisher>).
/* MSG INITIALIZATION FUNCTIONS END */

/* MSG UPDATE FUNCTIONS START */
void updateImu(void);				// Update the IMU sensor data.
void updateYaw(float gz, float dt);	// Update the yaw angle.
void updateOdom(void);				// Update the Odometry message.
void updateJointStates(void);		// Update the Joint States data.
/* MSG UPDATE FUNCTIONS END */

/* DATA HANDLE FUNCTIONS START */
void calculateRpm(void);			// Calculate the motors' velocity using encoder.
void calculateOdometry(void);		// Calculate the Odometry data using encoders & IMU.
void calculateWheelVelocity(void);	// Calculate individual wheel velocity (rad/s) using inverse kinematics.
/* DATA HANDLE FUNCTION END */

/* PUBLISHING FUNCTIONS START */
void publishImu(void);				// Publish the IMU message.
void publishRobotState(void);		// Publish the Odometry, Odom TF, and Joint States.
/* PUBLISHING FUNCTIONS END */

/* CONTROL FUNCTIONS START */
void controlMotors(void);
/* CONTROL FUNCTIONS START */

/* GLOBAL VARIABLES START */

/*
 * ROS NodeHandle
 */
ros::NodeHandle nh;

/*
 * ROS parameters
 */
const char* base_frame_id = "base_link";
const char* odom_frame_id = "odom";
const char* imu_frame_id = "imu_frame";

/*
 * Subscribers
 */
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", &commandVelocityCallback);
ros::Subscriber<std_msgs::UInt8> sub_use_pid("/use_pid", &enablePidCallback);
ros::Subscriber<std_msgs::Float32> sub_kp("/kp", &kpTuneCallback);
ros::Subscriber<std_msgs::Float32> sub_ki("/ki", &kiTuneCallback);
ros::Subscriber<std_msgs::Float32> sub_kd("/kd", &kdTuneCallback);
ros::Subscriber<std_msgs::Int8> sub_test_id("/test_id", &testIdCallback);

/*
 * Publishers
 */
sensor_msgs::Imu imu_msg;
ros::Publisher pub_imu("/imu/data", &imu_msg);

nav_msgs::Odometry odom_msg;
ros::Publisher pub_odom("/encoder/odom", &odom_msg);	// Encoder odometry

sensor_msgs::JointState joint_states_msg;
ros::Publisher pub_joint_states("/joint_states", &joint_states_msg);

/*
* TF broadcaster
*/

/*
* Control variables
*/
typedef enum
{
    linear_x = 0,
    linear_y,
    angular_z
}odom_component_t;

/*
* Odometric variables
*/
float odom_pose[3] = { 0.0 };							// Odometry position (in the odom frame)
float odom_vel[3] = { 0.0 };							// Odometry velocity (in the base link frame)
float goal_vel[3] = { 0.0 };							// Goal velocity received from </cmd_vel>

/*
 * Wheel velocity (rad/s) variables
 */
uint16_t prev_tick[NUM_OF_MOTOR] = { 0 };				// Last read encoder tick (to calculate diff tick)
float wheel_angular_vel[NUM_OF_MOTOR] = { 0.0f };      	// Calculated from goal velocity (rad/s). Used as set points.
float meas_wheel_angular_vel[NUM_OF_MOTOR] = { 0.0f }; 	// Actual speed calculated from encoder ticks (rad/s)
float meas_theta[NUM_OF_MOTOR] = {0.0f };

/*
 * Joint states variable
 */
double joint_states_pos[NUM_OF_MOTOR] = { 0.0 };
double joint_states_vel[NUM_OF_MOTOR] = { 0.0 };

/*
* Time-related variables
*/
typedef enum
{
    motor_control_event = 0,
    wheel_velocity_feedback_event,
    imu_publish_event,
    odom_publish_event
}time_index_t;
uint32_t t_previous[4] = { 0 };
uint32_t dt[4] = { 1000 / MOTOR_CONTROL_FREQUENCY, 1000 / VEL_FEEDBACK_FREQUENCY
				, 1000 / IMU_PUBLISH_FREQUENCY, 1000 / ODOM_PUBLISH_FREQUENCY };

/* GLOBAL VARIABLES END */

#ifdef __cplusplus
}
#endif

#endif /* INC_MECABOT_ROS_CONFIG_H_ */

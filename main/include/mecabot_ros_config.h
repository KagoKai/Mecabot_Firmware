#ifndef _MECABOT_ROS_CONFIG_H_
#define _MECABOT_ROS_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

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

#define NUM_OF_MOTOR                4

#define MOTOR_CONTROL_FREQUENCY		100
#define RPM_PUBLISH_FREQUENCY		5

/* CALLBACK FUNCTIONS START */
void motorSpeedCallback(const std_msgs::UInt8& motor_speed_msg);
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
/* CALLBACK FUNCTIONS END */

/* SETUP FUNCTIONS START */
void ros_setup(void);
/* SETUP FUNCTIONS END */

/* MSG INITIALIZATION FUNCTIONS START */
void initOdom(void);
void initJointStates(void);
/* MSG INITIALIZATION FUNCTIONS END */

/* MSG UPDATE FUNCTIONS START */
void updateImu(void);
void updateJointStates(void);
void updateOdometry(void);
void updateTF(void);
/* MSG UPDATE FUNCTIONS END */

/* PUBLISHING FUNCTIONS START */
void publishImu(void);
void publishRobotState(void);  // Update odometry, tf, and joint states
/* PUBLISHING FUNCTIONS END */

/* DATA HANDLE FUNCTIONS START */
void calculateOdometry(void); // Calculate odometry from motor speed
void calculateMotorSpeed(void); // Calculate individual motor speed from linear & angular speed
/* DATA HANDLE FUNCTION END */

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
const char odom_header_frame_id[] = "/odom";
const char odom_child_frame_id[] = "/base_link";
const char imu_frame_id[] = "/imu";
const char joint_state_header_frame_id[] = "/joint_states";

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
ros::Publisher pub_joint_state("/joint_states", &joint_states_msg);
nav_msgs::Odometry odom_msg;
ros::Publisher pub_odom("/odom", &odom_msg);

/*
* TF broadcaster
*/
geometry_msgs::TransformStamped tf_msg; // tf information between "/odom" and "/baselink"
tf::TransformBroadcaster tf_broadcaster;

/*
* Control variables
*/
float odom_pose[3] = {0.0, 0.0, 0.0};
float odom_vel[3] = {0.0, 0.0, 0.0};

uint16_t prev_encoder_tick[NUM_OF_MOTOR] = {0};
uint8_t motor_speed[NUM_OF_MOTOR] = {0};

/* GLOBAL VARIABLES END */
#ifdef __cplusplus
}
#endif

#endif
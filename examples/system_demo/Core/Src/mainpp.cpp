#include "mainpp.h"
#include "mecabot_hardware.h"
#include "mecabot_ros_config.h"

#define ENCODER_RESOLUTION		20

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

float constraint(float var, float min, float max)
{
	if (var < min)	return min;
	if (var > max) 	return max;
	else 			return var;
}

uint32_t t;
uint16_t last_tick[NUM_OF_MOTOR] = { 0 };

extern Motor mecabot_motor[NUM_OF_MOTOR];
extern Encoder encoder[NUM_OF_MOTOR];

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	comm_rosserial.set_tx_cplt();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	comm_rosserial.reset_rbuf();
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	switch (htim->Channel)
	{
	case HAL_TIM_ACTIVE_CHANNEL_3:
		encoder[front_left]->tick++;
		break;
	case HAL_TIM_ACTIVE_CHANNEL_1:
		encoder[front_right]->tick++;
		break;
	case HAL_TIM_ACTIVE_CHANNEL_4:
		encoder[back_left]->tick++;
		break;
	case HAL_TIM_ACTIVE_CHANNEL_2:
		encoder[back_right]->tick++;
	default:
		break;
	}
}

/* CALLBACK FUNCTIONS START */
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
	goal_vel[linear_x] = cmd_vel_msg.linear.x;
	goal_vel[linear_y] = cmd_vel_msg.linear.y;
	goal_vel[angular_z] = cmd_vel_msg.angular.z;
}
/* CALLBACK FUNCTIONS END */

void ros_setup()
{
	nh.initNode();

	nh.subscribe(sub_cmd_vel);

	nh.advertise(pub_imu);
    nh.advertise(pub_odom);
	nh.advertise(pub_joint_states);

	tf_broadcaster

	initOdom();
	initJointStates();
}

void setup()
{
	ros_setup();

	mecabot_motor_init();
	mecabot_encoder_init();
	// mecabot_mpu_init();

	for (int i=0; i<NUM_OF_MOTOR; i++)
	{
		mecabot_motor_start(mecabot_motor[i]);
	}
}

void loop()
{
	t = millis();

	/* Motor control */
	if ((t - t_previous[motor_control_event]) >= dt[motor_control_event])
	{
		calculateWheelVelocity();
		controlMotors();
		t_previous[motor_control_event] = t;
	}
	/* Wheel velocity feedback */
    if ((t - t_previous[wheel_velocity_feedback_event]) >= dt[wheel_velocity_feedback_event])
	{
		t_previous[wheel_velocity_feedback_event] = t;
	}
	/* Imu publish */
    if ((t - t_previous[imu_publish_event]) >= dt[imu_publish_event])
	{
		updateImu();
		publishImu();
		t_previous[imu_publish_event] = t;
	}
	/* Robot states (Odometry, TF, joint states) publish */
    if ((t - t_previous[robot_info_publish_event]) >= dt[robot_info_publish_event])
	{
		for (int i = 0; i < NUM_OF_MOTOR; i++)
		{
			uint16_t tick = mecabot_encoder_read(encoder[i]);

			diff_tick[i] = tick - prev_encoder_tick[i];
			prev_encoder_tick[i] = tick;
		}

		updateOdometry();
		updateJointStates();
		updateTF();
		publishRobotState();
		t_previous[robot_info_publish_event] = t;
	}

	nh.spinOnce();
}

/* MSG INITIALIZATION FUNCTIONS START */
void initOdom(void)
{
	for (int i = 0; i < 3; i++)
    {
        odom_pose[i] = 0.0;
        odom_vel[i]  = 0.0;
    }

	odom_msg.header.frame_id = odom_frame_id;
	odom_msg.child_frame_id = base_frame_id;

    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;

    odom_msg.twist.twist.linear.x  = 0.0;
	odom_msg.twist.twist.linear.y  = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;
}
void initJointStates(void)
{
	static char *joint_states_name[NUM_OF_MOTOR] =
	                {"front_left_wheel_joint", "front_right_wheel_joint",
	                 "back_left_wheel_joint" , "back_right_wheel_joint"};

	joint_states_msg.header.frame_id = "joint_states";
	joint_states_msg.name = joint_states_name;

	joint_states_msg.name_length 	 = NUM_OF_MOTOR;
	joint_states_msg.position_length = NUM_OF_MOTOR;
	joint_states_msg.velocity_length = NUM_OF_MOTOR;
	joint_states_msg.effort_length	 = NUM_OF_MOTOR;
}
void initTF(void)
{
	odom_tf_msg.header.frame_id = odom_msg.header.frame_id;
	odom_tf_msg.child_frame_id = odom_msg.child_frame_id;

	odom_tf_msg.transform.translation.x = 0;
	odom_tf_msg.transform.translation.y = 0;
	odom_tf_msg.transform.translation.z = 0;

	odom_tf_msg.transform.rotation.w = 1;
	odom_tf_msg.transform.rotation.x = 0;
	odom_tf_msg.transform.rotation.y = 0;
	odom_tf_msg.transform.rotation.z = 0;
}
/* MSG INITIALIZATION FUNCTIONS END */

/* MSG UPDATE FUNCTIONS START */
void updateImu(void)
{
	float gyro_buffer[3], accel_buffer[3];
	Quaternion_t quat_buffer;

	mecabot_imu_read_gyro(gyro_buffer);
	mecabot_imu_read_accel(accel_buffer);
	mecabot_imu_get_quaternion(&quat_buffer);

	imu_msg.angular_velocity.x = gyro_buffer[0];
    imu_msg.angular_velocity.y = gyro_buffer[1];
    imu_msg.angular_velocity.z = gyro_buffer[2];

    imu_msg.linear_acceleration.x = accel_buffer[0];
    imu_msg.linear_acceleration.y = accel_buffer[1];
    imu_msg.linear_acceleration.z = accel_buffer[2];

    imu_msg.orientation.w = quat_buffer.q0;
    imu_msg.orientation.x = quat_buffer.q1;
    imu_msg.orientation.y = quat_buffer.q2;
    imu_msg.orientation.z = quat_buffer.q3;

    imu_msg.angular_velocity_covariance[1] = 0;
    imu_msg.angular_velocity_covariance[2] = 0;
    imu_msg.angular_velocity_covariance[3] = 0;
    imu_msg.angular_velocity_covariance[4] = 0.02;
    imu_msg.angular_velocity_covariance[5] = 0;
    imu_msg.angular_velocity_covariance[6] = 0;
    imu_msg.angular_velocity_covariance[7] = 0;
    imu_msg.angular_velocity_covariance[8] = 0.02;

    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[1] = 0;
    imu_msg.linear_acceleration_covariance[2] = 0;
    imu_msg.linear_acceleration_covariance[3] = 0;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[5] = 0;
    imu_msg.linear_acceleration_covariance[6] = 0;
    imu_msg.linear_acceleration_covariance[7] = 0;
    imu_msg.linear_acceleration_covariance[8] = 0.04;

    imu_msg.orientation_covariance[0] = 0.0025;
    imu_msg.orientation_covariance[1] = 0;
    imu_msg.orientation_covariance[2] = 0;
    imu_msg.orientation_covariance[3] = 0;
    imu_msg.orientation_covariance[4] = 0.0025;
    imu_msg.orientation_covariance[5] = 0;
    imu_msg.orientation_covariance[6] = 0;
    imu_msg.orientation_covariance[7] = 0;
    imu_msg.orientation_covariance[8] = 0.0025;
}
void updateJointStates(void)
{
	static float joint_states_pos[NUM_OF_MOTOR] = { 0.0 };
	static float joint_states_vel[NUM_OF_MOTOR] = { 0.0 };

	for (int i = 0; i < NUM_OF_MOTOR; i++)
	{
		joint_states_pos[i] += TICK2RAD(diff_tick[i]);
		joint_states_pos[i] = fmod(joint_states_pos[i], 2*PI);

		joint_states_vel[i] = TICK2RAD(diff_tick[i] / dt[robot_info_publish_event]);
	}
}
void updateOdometry(void)
{
	calculateOdometry();

	odom_msg.pose.pose.position.x = odom_pose[linear_x];
    odom_msg.pose.pose.position.y = odom_pose[linear_y];
	odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[angular_z]);

    odom_msg.twist.twist.linear.x  = odom_vel[linear_x];
	odom_msg.twist.twist.linear.y  = odom_vel[linear_y];
    odom_msg.twist.twist.angular.z = odom_vel[angular_z];
}
void updateTF(void)
{
	odom_tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
	odom_tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
	odom_tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
	odom_tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
}
/* MSG UPDATE FUNCTIONS END */

/* PUBLISHING FUNCTIONS START */
void publishImu(void)
{
	imu_msg.header.frame_id = "/imu";
	imu_msg.header.stamp = nh.now();
	pub_imu.publish(&imu_msg);
}
void publishRobotState(void)
{
	ros::Time now = nh.now();

	odom_msg.header.stamp = now;
	pub_odom.publish(&odom_msg);

	joint_states_msg.header.stamp = now;
	pub_joint_states.publish(&joint_states_msg);

	odom_tf_msg.header.stamp = now;
	tf_broadcaster.sendTransform(odom_tf_msg);
}
/* PUBLISHING FUNCTIONS END */

/* DATA HANDLE FUNCTIONS START */
void calculateOdometry(void)
{
	float w[NUM_OF_MOTOR] = { 0.0 };
	float new_yaw, diff_yaw;

	for (int i = 0; i < NUM_OF_MOTOR; i++)
	{
		w[i] = mecabot_motor[i]->direction * TICK2RAD(diff_tick[i] / dt[robot_info_publish_event]);
	}

	float C = 2 / (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y);
	odom_vel[linear_x]  = (WHEEL_RADIUS / 4) * ( w[0] + w[1] + w[2] + w[3]);
	odom_vel[linear_y]  = (WHEEL_RADIUS / 4) * (-w[0] + w[1] + w[2] - w[3]);
	odom_vel[angular_z] = (WHEEL_RADIUS / 4) * C * (-w[0] + w[1] - w[2] + w[3]);

	Quaternion_t quat_buffer;
	mecabot_imu_get_quaternion(&quat_buffer);
	new_yaw = quat_getYaw(quat_buffer);
	diff_yaw = new_yaw - odom_pose[angular_z];

	odom_pose[linear_x] += (odom_vel[linear_x] * cos(new_yaw) + odom_vel[linear_y] * sin(new_yaw)) * dt[robot_info_publish_event];
	odom_pose[linear_y] += (odom_vel[linear_x] * sin(new_yaw) + odom_vel[linear_y] * cos(new_yaw)) * dt[robot_info_publish_event];
	odom_pose[angular_z] = new_yaw;
}
void calculateWheelVelocity(void)
{
	float C = (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) / 2;
	// Inverse kinematics
	wheel_angular_vel[front_left] = INV_WHEEL_RADIUS *
									(goal_vel[linear_x] - goal_vel[linear_y] - C * goal_vel[angular_z]);
	wheel_angular_vel[front_right] = INV_WHEEL_RADIUS *
									(goal_vel[linear_x] + goal_vel[linear_y] + C * goal_vel[angular_z]);
	wheel_angular_vel[back_left] = INV_WHEEL_RADIUS *
									(goal_vel[linear_x] + goal_vel[linear_y] - C * goal_vel[angular_z]);
	wheel_angular_vel[back_right] = INV_WHEEL_RADIUS *
									(goal_vel[linear_x] - goal_vel[linear_y] + C * goal_vel[angular_z]);
}
/* DATA HANDLE FUNCTIONS END */

/* CONTROL FUNCTIONS START */
void controlMotors(void)
{
	for (int i = 0; i < NUM_OF_MOTOR; i++)
	{
		wheel_angular_vel[i] = constraint(wheel_angular_vel[i], WHEEL_MIN_ANGULAR_VELOCITY, WHEEL_MAX_ANGULAR_VELOCITY);
		mecabot_motor_set_angular_velocity(mecabot_motor[i], wheel_angular_vel[i]);
	}
}
/* CONTROL FUNCTIONS END */

/*
 * mainpp.cpp
 *
 *  Created on: May 25, 2024
 *      Author: ADMIN
 */

#include "mainpp.h"
#include "mecabot_hardware.h"
#include "mecabot_ros_config.h"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

float constraint(float var, float min, float max)
{
	if (var < min)	return min;
	if (var > max) 	return max;
	else 			return var;
}

uint32_t t = 0; // Elapsed system time

uint16_t d_tick = 0;

/* DEBUG PARAMETERS START */
const float odom_test[9][3] =
{
		{ 0.0, 0.0, 0.0},			// Stop
		{ 0.4, 0.0, 0.0},			// Di tien
		{ 0.4, 0.0, 0.0},			// Di di lui
		{ 0.0, 0.4, 0.0},			// Di sang trai
		{ 0.0, -0.4, 0.0},			// Di sang phai
		{ 0.28284, 0.28284, 0.0}, 	// Di cheo trai
		{ 0.28284, -0.28284, 0.0}, 	// Di cheo phai
		{ 0.0, 0.0, PI * 0.4 }, 	// Quay CCW
		{ 0.0, 0.0, - PI * 0.4 } 	// Quay CW
};
bool is_testing = false;
uint32_t test_start_time = 0;
/* DEBUG PARAMETERS START */

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
void enablePidCallback(const std_msgs::UInt8& use_pid_msg)
{
	if (use_pid_msg.data == 1)
	{
		use_pid = true;
	}
	else
	{
		use_pid = false;
	}
}
void kpTuneCallback(const std_msgs::Float32& kp_msg)
{
	kp = kp_msg.data;
	for (int i = 0; i < NUM_OF_MOTOR; i++)
	{
		PID_SetGain(controller[i], kp, ki, kd);
	}
}
void kiTuneCallback(const std_msgs::Float32& ki_msg)
{
	ki = ki_msg.data;
	for (int i = 0; i < NUM_OF_MOTOR; i++)
	{
		PID_SetGain(controller[i], kp, ki, kd);
	}
}
void kdTuneCallback(const std_msgs::Float32& kd_msg)
{
	kd = kd_msg.data;
	for (int i = 0; i < NUM_OF_MOTOR; i++)
	{
		PID_SetGain(controller[i], kp, ki, kd);
	}
}
void testIdCallback(const std_msgs::Int8& test_id_msg)
{
	int i = test_id_msg.data;
	if (i != 0)
	{
		goal_vel[linear_x] = odom_test[i][linear_x];
		goal_vel[linear_y] = odom_test[i][linear_y];
		goal_vel[angular_z] = odom_test[i][angular_z];

		is_testing = true;
		test_start_time = millis();
	}
	else
	{
		is_testing = false;
	}
}
/* CALLBACK FUNCTIONS END */

void ros_setup()
{
	nh.initNode();

	nh.subscribe(sub_cmd_vel);
	nh.subscribe(sub_use_pid);
	nh.subscribe(sub_kp);
	nh.subscribe(sub_ki);
	nh.subscribe(sub_kd);
	nh.subscribe(sub_test_id);

	//nh.advertise(pub_imu);
    nh.advertise(pub_odom);
	nh.advertise(pub_joint_states);

	initOdom();
	initJointStates();
}

void setup()
{
	ros_setup();

	mecabot_motor_init();
	mecabot_encoder_init();
	mecabot_pid_init();

	//while (mecabot_mpu_init() != STATUS_OK);
	//MPU6050_Calibrate(&hi2c1, &my_mpu);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	for (int i=0; i<NUM_OF_MOTOR; i++)
	{
		mecabot_motor_start(mecabot_motor[i]);
		mecabot_encoder_start(encoder[i]);
	}
}

void loop()
{
	t = millis();

	/* Wheel velocity feedback */
    if ((t - t_previous[wheel_velocity_feedback_event]) >= dt[wheel_velocity_feedback_event])
	{
    	calculateRpm();
		t_previous[wheel_velocity_feedback_event] = t;
	}
	/* Motor control */
	if ((t - t_previous[motor_control_event]) >= dt[motor_control_event])
	{
		calculateWheelVelocity();
		controlMotors();
		t_previous[motor_control_event] = t;
	}
	/* Imu publish */
    if ((t - t_previous[imu_publish_event]) >= dt[imu_publish_event])
	{
		//updateImu();
		//publishImu();
		t_previous[imu_publish_event] = t;
	}
	/* Robot states (Odometry, TF, joint states) publish */
    if ((t - t_previous[odom_publish_event]) >= dt[odom_publish_event])
	{
		updateOdom();
		updateJointStates();

		publishRobotState();

		t_previous[odom_publish_event] = t;
	}

    /* Odom navigation testing */
    if (is_testing)
    {
    	if ((t - test_start_time) >= 5200) // Run the test for 5 seconds
    	{
    		goal_vel[linear_x] = 0;
    		goal_vel[linear_y] = 0;
    		goal_vel[angular_z] = 0;
    		is_testing = false;
    		test_start_time = 0;
     	}
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
	static char *joint_states_name[4] {(char*)"front_left_wheel_joint", (char*)"front_right_wheel_joint",
												(char*)"back_left_wheel_joint" , (char*)"back_right_wheel_joint"};

	joint_states_msg.header.frame_id = base_frame_id;
	joint_states_msg.name = joint_states_name;

	joint_states_msg.name_length 	 = NUM_OF_MOTOR;
	joint_states_msg.position_length = NUM_OF_MOTOR;
	joint_states_msg.velocity_length = NUM_OF_MOTOR;
	joint_states_msg.effort_length	 = NUM_OF_MOTOR;
}
/* MSG INITIALIZATION FUNCTIONS END */

/* MSG UPDATE FUNCTIONS START */
void updateImu(void)
{
	mecabot_imu_read_gyro(gyro_buffer);
	mecabot_imu_read_accel(accel_buffer);

	updateYaw(gyro_buffer[2], (float)dt[imu_publish_event]/1000.0f);

	imu_msg.header.stamp = nh.now();
	imu_msg.header.frame_id = imu_frame_id;

	imu_msg.angular_velocity.x = 0;
    imu_msg.angular_velocity.y = 0;
    imu_msg.angular_velocity.z = gyro_buffer[2];

    imu_msg.linear_acceleration.x = accel_buffer[0];
    imu_msg.linear_acceleration.y = accel_buffer[1];
    imu_msg.linear_acceleration.z = accel_buffer[2];

    imu_msg.orientation = tf::createQuaternionFromYaw(imu_yaw);
}
void updateYaw(float gz, float dt)
{
	imu_yaw += gz * dt;
}
void updateOdom(void)
{
	calculateOdometry();

	odom_msg.header.stamp = nh.now();

	odom_msg.header.frame_id = odom_frame_id;
	odom_msg.child_frame_id = base_frame_id;

	odom_msg.pose.pose.position.x = odom_pose[linear_x];
    odom_msg.pose.pose.position.y = odom_pose[linear_y];
	odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[angular_z]);

    odom_msg.twist.twist.linear.x  = odom_vel[linear_x];
	odom_msg.twist.twist.linear.y  = odom_vel[linear_y];
    odom_msg.twist.twist.angular.z = odom_vel[angular_z];
}
void updateJointStates(void)
{
	joint_states_msg.header.stamp = nh.now();

	joint_states_msg.header.frame_id = base_frame_id;
	for (int i = 0; i < NUM_OF_MOTOR; i++)
	{
		joint_states_pos[i] = meas_theta[i];
		joint_states_vel[i] = meas_wheel_angular_vel[i];
	}

	joint_states_msg.position = reinterpret_cast<double *>(joint_states_pos);
	joint_states_msg.velocity = reinterpret_cast<double *>(joint_states_vel);
}
/* MSG UPDATE FUNCTIONS END */

/* DATA HANDLE FUNCTIONS START */
void calculateRpm(void)
{
	for (int i = 0; i < NUM_OF_MOTOR; ++i)
	{
		if (encoder[i]->tick < prev_tick[i])	// Over-flow
		{
			d_tick = encoder[i]->tick + (encoder[i]->max_count - prev_tick[i]);
		}
		else if (encoder[i]->tick > prev_tick[i])
		{
			d_tick = encoder[i]->tick - prev_tick[i];
		}
		else
		{
			d_tick = 0;
		}

		prev_tick[i] = encoder[i]->tick;

		// Get the number of rotation
		meas_wheel_angular_vel[i] = static_cast<float>(d_tick) / ENCODER_RESOLUTION;
		// Get the rotation per second
		meas_wheel_angular_vel[i] /= (float)dt[wheel_velocity_feedback_event]/1000;
		// Get the radians per second
		meas_wheel_angular_vel[i] *= 2 * PI;

		// Apply Low Pass filter
		meas_wheel_angular_vel[i] = mecabot_motor[i]->direction * FO_IIR_Compute(encoder_filter[i], meas_wheel_angular_vel[i]);
		if ((meas_wheel_angular_vel[i] > -0.001f) && (meas_wheel_angular_vel[i] < 0.001f))	meas_wheel_angular_vel[i] = 0;

		meas_theta[i] += meas_wheel_angular_vel[i] * (float)dt[wheel_velocity_feedback_event] / 1000.0f;
	}
}
void calculateOdometry(void)
{
	float w[NUM_OF_MOTOR] = { 0.0 };

	for (int i = 0; i < NUM_OF_MOTOR; i++)
	{
		// Calculate filtered wheel velocity (rad/s) from d_tick
		w[i] = meas_wheel_angular_vel[i];
	}

	// Robot velocity in the base frame (calculated from Inverse Kinematics)
	float C = 2 / (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y);
	odom_vel[linear_x]  = (WHEEL_RADIUS / 4) * ( w[0] + w[1] + w[2] + w[3]);
	odom_vel[linear_y]  = (WHEEL_RADIUS / 4) * (-w[0] + w[1] + w[2] - w[3]);
	odom_vel[angular_z] = (WHEEL_RADIUS / 4) * C * (-w[0] + w[1] - w[2] + w[3]);

	encoder_yaw += odom_vel[angular_z] *  ((float)dt[odom_publish_event])/1000.0f;

	// Robot position in the world frame
	odom_pose[linear_x] += (odom_vel[linear_x] * cos(encoder_yaw) - odom_vel[linear_y] * sin(encoder_yaw)) * ((float)dt[odom_publish_event]) / 1000.0f;
	odom_pose[linear_y] += (odom_vel[linear_x] * sin(encoder_yaw) + odom_vel[linear_y] * cos(encoder_yaw)) * ((float)dt[odom_publish_event]) / 1000.0f;
	odom_pose[angular_z] = encoder_yaw;
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

	for (int i = 0; i < NUM_OF_MOTOR; i++)
	{
		controller[i]->set_point = wheel_angular_vel[i];
	}
}
/* DATA HANDLE FUNCTIONS END */

/* PUBLISH FUNCTIONS START */
void publishImu(void)
{
	imu_msg.header.frame_id = imu_frame_id;
	imu_msg.header.stamp = nh.now();
	pub_imu.publish(&imu_msg);
}
void publishRobotState(void)
{
	pub_odom.publish(&odom_msg);
	pub_joint_states.publish(&joint_states_msg);
}
/* PUBLISH FUNCTIONS START */

/* CONTROL FUNCTIONS START */
void controlMotors(void)
{
	for (int i = 0; i < NUM_OF_MOTOR; i++)
	{
		if (mecabot_motor[i]->state != STATE_STOP)
		{
			if ((meas_wheel_angular_vel[i] > -WHEEL_MIN_ANGULAR_VELOCITY) && (meas_wheel_angular_vel[i] < WHEEL_MIN_ANGULAR_VELOCITY))
			{
				duty[i] = 0;
				Motor_ChangeState(mecabot_motor[i], STATE_STOP);
				continue;
			}
		}

	    if (use_pid)
	    {
	    	duty[i] = PID_Compute(controller[i], meas_wheel_angular_vel[i], dt[motor_control_event]);
	    }
	    else
	    {
	    	duty[i] = constraint(wheel_angular_vel[i]/WHEEL_MAX_ANGULAR_VELOCITY * 255, -255, 255);
	    }
	}

	for (int i = 0; i < NUM_OF_MOTOR; i++)
	{
		Motor_Set_PWM_Duty(mecabot_motor[i], duty[i]);
	}
}
/* CONTROL FUNCTIONS END */

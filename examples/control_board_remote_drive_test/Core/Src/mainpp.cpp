/*
 * mainpp.cpp
 *
 *  Created on: May 25, 2024
 *      Author: ADMIN
 */

#include <cstdint>
#include <stdlib.h>
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

extern Motor mecabot_motor[4];
extern Encoder encoder[4];
extern FO_IIR_Filter_t *filter[4];

uint16_t prev_tick[NUM_OF_MOTOR] = { 0, 0, 0, 0 };
float rpm[NUM_OF_MOTOR] = { 0.0f };

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
uint8_t duty[NUM_OF_MOTOR] = { 0 };
void motorDutyCallback(const std_msgs::UInt8& motor_duty_msg)
{
	for (int i = 0; i < 4; i++)
	{
		duty[i] = motor_duty_msg.data;
	}
}
/* CALLBACK FUNCTIONS END */

void ros_setup()
{
	nh.initNode();

	nh.subscribe(sub_cmd_vel);
	nh.subscribe(sub_duty);

	//nh.advertise(pub_FL_rpm);
	//nh.advertise(pub_FR_rpm);
	//nh.advertise(pub_BL_rpm);
	//nh.advertise(pub_BR_rpm);
}

void setup()
{
	ros_setup();

	mecabot_motor_init();
	mecabot_encoder_init();

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
		//calculateWheelVelocity();
		controlMotors();
		t_previous[motor_control_event] = t;
	}
	/* Wheel velocity feedback */
    if ((t - t_previous[wheel_velocity_feedback_event]) >= dt[wheel_velocity_feedback_event])
	{
    	calculateRpm();
    	publishRpm();
		t_previous[wheel_velocity_feedback_event] = t;
	}

	nh.spinOnce();
}

/* DATA HANDLE FUNCTIONS START */
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
void calculateRpm(void)
{
	for (int i = 0; i < 1; i++)
	{
		if (encoder[i]->tick < prev_tick[i])
		{
			d_tick = encoder[i]->tick + (encoder[i]->max_count - prev_tick[i]);
		}
		else
		{
			d_tick = encoder[i]->tick - prev_tick[i];
		}
		prev_tick[i] = encoder[i]->tick;

		// Get the number of rotation
		rpm[i] = (float)d_tick / ENCODER_RESOLUTION;
		// Get the rotation per second
		rpm[i] /= (dt[wheel_velocity_feedback_event] / 1000.0f);
		// Get the rotation per minute
		rpm[i] *= 60;

		// Apply Low Pass filter
		rpm[i] = FO_IIR_Compute(filter[i], rpm[i]);

		rpm_msg[i].data = static_cast<uint16_t>(rpm[i]);
	}
}
/* DATA HANDLE FUNCTIONS END */

/* CONTROL FUNCTIONS START */
void controlMotors(void)
{
	for (int i = 0; i < NUM_OF_MOTOR; i++)
	{
		//wheel_angular_vel[i] = constraint(wheel_angular_vel[i], WHEEL_MIN_ANGULAR_VELOCITY, WHEEL_MAX_ANGULAR_VELOCITY);
		//mecabot_motor_set_angular_velocity(mecabot_motor[i], wheel_angular_vel[i]);
		Motor_Set_PWM_Duty(mecabot_motor[i], duty[i]);
	}
}
/* CONTROL FUNCTIONS END */

/* PUBLISH FUNCTIONS START */
void publishRpm(void)
{
	//pub_FL_rpm.publish(&rpm_msg[front_left]);
	//pub_FR_rpm.publish(&rpm_msg[front_right]);
	//pub_BL_rpm.publish(&rpm_msg[back_left]);
	//pub_BR_rpm.publish(&rpm_msg[back_right]);
}
/* PUBLISH FUNCTIONS START */
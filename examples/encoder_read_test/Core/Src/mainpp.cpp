#include <cstdint>
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
	case HAL_TIM_ACTIVE_CHANNEL_1:
		encoder[front_left]->tick++;
		break;
	case HAL_TIM_ACTIVE_CHANNEL_2:
		encoder[front_right]->tick++;
		break;
	case HAL_TIM_ACTIVE_CHANNEL_3:
		encoder[back_left]->tick++;
		break;
	case HAL_TIM_ACTIVE_CHANNEL_4:
		encoder[back_right]->tick++;
	default:
		break;
	}
}

void motorSpeedCallback(const std_msgs::UInt8& motor_speed_msg)
{
}

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
	goal_vel[0] = cmd_vel_msg.linear.x;
	goal_vel[1] = cmd_vel_msg.linear.y;
	goal_vel[2] = cmd_vel_msg.angular.z;
}

void ros_setup()
{
	nh.initNode();

	nh.subscribe(sub_cmd_vel);
	nh.advertise(pub_left_motor_rpm);
	nh.advertise(pub_right_motor_rpm);
}

uint32_t millis()
{
	return HAL_GetTick();
}

void setup()
{
	ros_setup();

	mecabot_encoder_init();

	mecabot_motor_init();
	/*
	for (int i=0; i<NUM_OF_MOTOR; i++)
	{
		mecabot_motor_start(mecabot_motor[i]);
	}
	*/
	Motor_SetDirection(mecabot_motor[front_left], mecabot_motor[front_left]->direction);
	Motor_SetDirection(mecabot_motor[front_right], mecabot_motor[front_right]->direction);
	HAL_TIM_PWM_Start(&htim_motor, mecabot_motor[front_left]->pwm_channel);
	HAL_TIM_PWM_Start(&htim_motor, mecabot_motor[front_right]->pwm_channel);

	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void loop()
{
	t = millis();

	/* Motor control */
	if ((t - t_previous[MOTOR_CONTROL_ID]) >= dt[MOTOR_CONTROL_ID])
	{
		calculateMotorSpeed();
		t_previous[MOTOR_CONTROL_ID] = t;
	}
	/* RPM publish */
	if ((t - t_previous[RPM_PUBLISH_ID]) >= dt[RPM_PUBLISH_ID])
	{
		updateRpm();
		publishRpmMsg();
		t_previous[RPM_PUBLISH_ID] = t;
	}
	/* Odometry publish */

	nh.spinOnce();
}

uint16_t d_tick = 0;
float data = 0;

void updateRpm()
{
	for (int i=0; i<NUM_OF_MOTOR; i++)
	{
		d_tick = 0;
		// Encoder_UpdateTick(my_encoder);

		if (encoder[i]->tick < last_tick[i])
		{
			d_tick = encoder[i]->tick + (0xFFFF - last_tick[i]);
		}
		else
		{
			d_tick = encoder[i]->tick - last_tick[i];
		}
		last_tick[i] = encoder[i]->tick;

		// Get the number of rotation
		data = (float)d_tick / ENCODER_RESOLUTION;
		// Get the rotation per second
		data = (data * 1000) / dt[RPM_PUBLISH_ID];
		// Get the rotation per minute
		//data *= 60;

		rpm_msg[i].data = (uint32_t)(data*60);
	}
}

void publishRpmMsg()
{
	pub_left_motor_rpm.publish(&rpm_msg[front_left]);
	pub_right_motor_rpm.publish(&rpm_msg[front_right]);
}

void calculateMotorSpeed()
{
	float linear_vel = goal_vel[0];
	float angular_vel = goal_vel[2];

	float left_motor_vel = linear_vel - (angular_vel * WHEEL_SEPARATION_Y / 2);
	float right_motor_vel = linear_vel + (angular_vel * WHEEL_SEPARATION_Y / 2);

	left_motor_vel = constraint(left_motor_vel, -WHEEL_MAX_VELOCITY, WHEEL_MAX_VELOCITY);
	right_motor_vel = constraint(right_motor_vel, -WHEEL_MAX_VELOCITY, WHEEL_MAX_VELOCITY);

	mecabot_motor_set_velocity(mecabot_motor[front_left], left_motor_vel);
	mecabot_motor_set_velocity(mecabot_motor[front_right], right_motor_vel);
}

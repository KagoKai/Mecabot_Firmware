/*
 * mainpp.cpp
 *
 *  Created on: Apr 3, 2024
 *      Author: ADMIN
 */

#include <cstdint>
#include "mainpp.h"
#include "ros_config.h"
#include "encoder.h"
#include "dc_motor.h"

#define ENCODER_RESOLUTION		20

Encoder_t *my_encoder = NULL;
Motor_t *my_motor = NULL;

uint32_t dt = 1000 / RPM_PUBLISH_FREQUENCY; // RPM publish period (in millisec).

uint32_t t = 0, t_motor_control_prev = 0, t_motor_rpm_prev = 0;
uint16_t last_tick = 0;

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
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		my_encoder->tick++;
	}
}

void motorSpeedCallback(const std_msgs::UInt8& motor_speed_msg)
{
	uint8_t duty_val = motor_speed_msg.data;
	Motor_Set_PWM_Duty(my_motor, duty_val);
}

void ros_setup()
{
	nh.initNode();

	nh.subscribe(sub_motor_speed);
	nh.advertise(pub_motor_rpm);
}

uint32_t millis()
{
	return HAL_GetTick();
}

void setup()
{
	ros_setup();

	Encoder_Handle_t encoder_handle =
	{
			.max_count = 0xFFFFFFFF,
			.tick_read_channel = TIM_CHANNEL_2
	};
	my_encoder = Encoder_Init(encoder_handle);
	Encoder_Start(my_encoder);

	Motor_Handle_t motor_handle =
	{
			.pwm_channel = TIM_CHANNEL_1,
			.direction_port = GPIOA,
			.direction_pin_A = GPIO_PIN_11,
			.direction_pin_B = GPIO_PIN_12,
			.pwm_frequency = 1000
	};
	my_motor = Motor_Init(motor_handle);
	Motor_Start(my_motor);
}

void loop()
{
	t = millis();

	if ((t - t_motor_rpm_prev) >= dt)
	{
		updateRpm();
		publishRpmMsg();

		t_motor_rpm_prev = t;
	}

	nh.spinOnce();
}

uint16_t d_tick = 0;
float data = 0;

void updateRpm()
{
	d_tick = 0;

	// Encoder_UpdateTick(my_encoder);

	if (my_encoder->tick < last_tick)
	{
		d_tick = my_encoder->tick + (0xFFFF - last_tick);
	}
	else
	{
		d_tick = my_encoder->tick - last_tick;
	}
	last_tick = my_encoder->tick;

	// Get the number of rotation
	data = (float)d_tick / ENCODER_RESOLUTION;
	// Get the rotation per second
	data = (data * 1000) / dt;
	// Get the rotation per minute
	//data *= 60;

	rpm_msg.data = (uint32_t)data*60;
}

void publishRpmMsg()
{
	pub_motor_rpm.publish(&rpm_msg);
}
/*
 * mainpp.cpp
 *
 *  Created on: Apr 3, 2024
 *      Author: ADMIN
 */

#include <cstdint>
#include <stdlib.h>
#include "mainpp.h"
#include "ros_config.h"
#include "encoder.h"
#include "IIR_filter.h"
#include "dc_motor.h"
#include "pid.h"

#define ENCODER_RESOLUTION		20

#define KP		10
#define KI		0
#define KD 		0

#define PI		3.14159265359
#define WHEEL_MAX_ANGULAR_VELOCITY     240                             /*!< Wheel angular velocity rpm */
#define WHEEL_MIN_ANGULAR_VELOCITY     -WHEEL_MAX_ANGULAR_VELOCITY

Encoder_t *my_encoder = NULL;
Motor_t *my_motor = NULL;
PID_t *my_controller = NULL;
FO_IIR_Filter_t *encoder_filter = NULL;

double dt[2] = { 1000.0f / MOTOR_CONTROL_FREQUENCY, 1000.0f / RPM_PUBLISH_FREQUENCY }; // Time interval between events (in millisec).

double t = 0, t_motor_control_prev = 0, t_motor_rpm_prev = 0;
uint16_t last_tick = 0;

uint16_t d_tick;
double rpm_data; 			// Raw Motor Velocity calculated from Encoder
uint32_t rpm_filtered; 		// Motor Velocity with Moving Average Filter
float rpm_LPF;				// Motor Velocity with First Order IIR Filter

uint8_t duty_pwm = 0;

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

void motorVelCallback(const std_msgs::UInt8& motor_speed_msg)
{
	my_controller->set_point = static_cast<double>(motor_speed_msg.data);
	duty_pwm = motor_speed_msg.data;
}
void pGainCallback(const std_msgs::Float64& kp_msg)
{
	my_controller->kp = static_cast<double>(kp_msg.data);
}
void iGainCallback(const std_msgs::Float64& ki_msg)
{
	my_controller->ki = static_cast<double>(ki_msg.data);
}
void dGainCallback(const std_msgs::Float64& kd_msg)
{
	my_controller->kd = static_cast<double>(kd_msg.data);
}

void ros_setup()
{
	nh.initNode();

	nh.subscribe(sub_motor_speed);
	nh.subscribe(sub_P_gain);
	nh.subscribe(sub_I_gain);
	nh.subscribe(sub_D_gain);

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
			.max_count = 0xFFFF,
			.tick_read_channel = TIM_CHANNEL_1
	};
	my_encoder = Encoder_Init(encoder_handle);
	Encoder_Start(my_encoder);

	float b_coeff[] = { 0.7548f, 0.7548f };
	float a_coeff[] = {    1.0f, 0.5095f };
	encoder_filter = reinterpret_cast<FO_IIR_Filter_t*>(calloc(1, sizeof(FO_IIR_Filter_t)));
	FO_IIR_Init(encoder_filter, b_coeff, a_coeff);

	Motor_Handle_t motor_handle =
	{
			.pwm_channel = TIM_CHANNEL_1,
			.direction_port = GPIOC,
			.direction_pin = GPIO_PIN_3,
			.pwm_frequency = 1000
	};
	my_motor = Motor_Init(motor_handle);
	Motor_Start(my_motor);

	my_controller = PID_Init(KP, KI, KD);
}

void loop()
{
	t = static_cast<double>(millis());

	if ((t - t_motor_control_prev) >= dt[0])
	{
		//duty_pwm = PID_Compute(my_controller, rpm_data, dt[0]);
		Motor_Set_PWM_Duty(my_motor, duty_pwm);

		t_motor_control_prev = t;
	}
	if ((t - t_motor_rpm_prev) >= dt[1])
	{
		updateRpm();
		publishRpmMsg();

		t_motor_rpm_prev = t;
	}

	nh.spinOnce();
}

void updateRpm()
{
	if (my_encoder->tick < last_tick)
	{
		d_tick = my_encoder->tick + (my_encoder->max_count - last_tick);
	}
	else
	{
		d_tick = my_encoder->tick - last_tick;
	}
	last_tick = my_encoder->tick;

	// Get the number of rotation
	rpm_data = (double)d_tick / ENCODER_RESOLUTION;
	// Get the rotation per second
	rpm_data /= (dt[1] / 1000.0f);
	// Get the rotation per minute
	rpm_data *= 60;

	// Apply Low Pass filter
	rpm_LPF = FO_IIR_Compute(encoder_filter, rpm_data);

	rpm_msg.data = rpm_LPF;
}

void publishRpmMsg()
{
	pub_motor_rpm.publish(&rpm_msg);
}
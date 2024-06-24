/*
 * mainpp.cpp
 *
 *  Created on: Jun 16, 2024
 *      Author: ADMIN
 */

#include <cstdint>
#include <stdlib.h>
#include "mainpp.h"
#include "encoder.h"
#include "IIR_filter.h"
#include "dc_motor_2.h"
#include "pid.h"

#define ENCODER_RESOLUTION		20

#define KP		8.75
#define KI		25
#define KD 		0

#define PI		3.14159265359
#define WHEEL_MAX_ANGULAR_VELOCITY    	240                             /*!< Wheel angular velocity rpm */
#define WHEEL_MIN_ANGULAR_VELOCITY     -WHEEL_MAX_ANGULAR_VELOCITY
#define MOTOR_PWM_FREQUENCY				1000


#define MOTOR_CONTROL_FREQUENCY		10
#define VEL_FEEDBACK_FREQUENCY		10

#define RPM_2_RAD_PER_S(rpm) 			((rpm) / 60.0f * 2 * PI)

/*
 * DEBUG VARIABLE
 */
float raw_FL_rpm = 0, raw_FR_rpm = 0, raw_BL_rpm = 0, raw_BR_rpm = 0;	// Raw rpm
float FL_rpm = 0, FR_rpm = 0, BL_rpm = 0, BR_rpm = 0;					// rpm
float raw_FL_vel = 0, raw_FR_vel = 0, raw_BL_vel = 0, raw_BR_vel = 0;	// Raw rad/sec
float FL_vel = 0, FR_vel = 0, BL_vel = 0, BR_vel = 0;					// rad/sec
float FL_dtick = 0, FR_dtick = 0, BL_dtick = 0, BR_dtick = 0;

bool use_pid = true;

typedef enum
{
	front_left = 0,
	front_right,
	back_left,
	back_right
};

Encoder_t *encoder[4] = { NULL };
Motor_t *mecabot_motor[4] = { NULL };
FO_IIR_Filter_t *encoder_filter[4] = { NULL };
PID_t *my_controller[4] = { NULL };
int16_t duty[4] = { 0 };

uint16_t dt[2] = { 1000 / VEL_FEEDBACK_FREQUENCY, 1000 / MOTOR_CONTROL_FREQUENCY }; // Time interval between events (in millisec).

uint32_t t = 0, t_motor_control_prev = 0, t_motor_rpm_prev = 0;
uint16_t last_tick = 0;

uint16_t d_tick;
uint16_t prev_tick[4];			// Last recorded encoder tick
float raw_rpm_data[4]; 			// Raw Motor Velocity calculated from Encoder (rpm)
float filtered_rpm_data[4]; 	// Motor Velocity with First Order IIR Filter (rpm)
float raw_vel_data[4]; 			// Raw Motor Velocity calculated from Encoder (rad/s)
float filtered_vel_data[4]; 	// Motor Velocity with First Order IIR Filter (rad/s)

int16_t motor_duty[4] = { 0, 0, 0, 0 }; 		// Motor duty [-255, 255]
int16_t vel_setpoint = RPM_2_RAD_PER_S(0); 			// Setpoint value for PID controller test (rad/s)

#define SET_POINT_100	RPM_2_RAD_PER_S(100)
#define SET_POINT_175	RPM_2_RAD_PER_S(175)
#define SET_POINT_250	RPM_2_RAD_PER_S(250)

float set_point = 0;
bool is_pid = true;			// Set to true to test PID
bool speed_changed_100 = false, speed_changed_175 = false, speed_changed_250 = false, speed_changed_stop = false;


status_t mecabot_motor_init(void)
{
    /* Hard-coded value for the Control Board */

	// Front Left motor setting
    Motor_Handle_t handle = {
        .pwm_channel = TIM_CHANNEL_3,
        .direction_port = GPIOC,
        .direction_pin = GPIO_PIN_2,
        .pwm_frequency = MOTOR_PWM_FREQUENCY
    };
    mecabot_motor[front_left] = Motor_Init(handle);
    // Front Right motor setting
    handle.pwm_channel = TIM_CHANNEL_1;
    handle.direction_port = GPIOC;
    handle.direction_pin = GPIO_PIN_3;
    mecabot_motor[front_right] = Motor_Init(handle);
    // Back Left motor setting
    handle.pwm_channel = TIM_CHANNEL_4;
    handle.direction_port = GPIOC;
    handle.direction_pin = GPIO_PIN_0;
    mecabot_motor[back_left] = Motor_Init(handle);
    // Back Right motor setting
    handle.pwm_channel = TIM_CHANNEL_2;
    handle.direction_port = GPIOC;
    handle.direction_pin = GPIO_PIN_1;
    mecabot_motor[back_right] = Motor_Init(handle);

    // TODO: Check if all motors were initialized correctly
    return STATUS_OK;
}

status_t mecabot_encoder_init(void)
{
	/* Hard-coded value for the Control Board */

	// Front Left encoder setting
    Encoder_Handle_t handle_encoder = {
        .max_count = 0xFFFF,
        .tick_read_channel = TIM_CHANNEL_3
    };
    encoder[front_left] = Encoder_Init(handle_encoder);
    // Front Right encoder setting
    handle_encoder.tick_read_channel = TIM_CHANNEL_1;
    encoder[front_right] = Encoder_Init(handle_encoder);
    // Back Left encoder setting
    handle_encoder.tick_read_channel = TIM_CHANNEL_4;
    encoder[back_left] = Encoder_Init(handle_encoder);
    // Back Right encoder setting
    handle_encoder.tick_read_channel = TIM_CHANNEL_2;
    encoder[back_right] = Encoder_Init(handle_encoder);

    // Initialize the encoder Low-pass Filter
    float b_coeff[] = { 0.2452,	0.2452 };
    float a_coeff[] = {    1.0, -0.5095 };
	for (int i = 0; i < 4; i++)
	{
		encoder_filter[i] = new FO_IIR_Filter_t;
		FO_IIR_Init(encoder_filter[i], b_coeff, a_coeff);
	}
    // TODO: Check if all encoders were initialized correctly
    return STATUS_OK;
}

status_t mecabot_pid_init(void)
{
	for (int i = 0; i < 4; i++)
	{
		my_controller[i] = PID_Init(KP, KI, KD);
		my_controller[i]->set_point = vel_setpoint;
	}

	return STATUS_OK;
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

uint32_t millis()
{
	return HAL_GetTick();
}

void updateRpm(void);

void setup()
{
	mecabot_motor_init();
	mecabot_encoder_init();
	mecabot_pid_init();

	for (int i=0; i<4; i++)
	{
		Motor_Start(mecabot_motor[i]);
		Encoder_Start(encoder[i]);
	}
}

int8_t direction;
uint8_t duty_debug;
void loop()
{
	t = millis();

	if ((t - t_motor_rpm_prev) >= dt[0])
	{
		updateRpm();
		//publishRpmMsg();

		t_motor_rpm_prev = t;
	}

	if ((t - t_motor_control_prev) >= dt[1])
	{
		int8_t direction;
		for (int i = 0; i < 4; i++)
		{
			if (mecabot_motor[i]->state == STATE_BRAKE)
			{
				if ((filtered_vel_data[i] > -0.001f) && (filtered_vel_data[i] < 0.001f))
				{
					Motor_ChangeState(mecabot_motor[i], STATE_STOP);
				}
			}

			if (use_pid) { duty[i] = PID_Compute(my_controller[i], filtered_vel_data[i], dt[1]); }
			else		 { duty[i] = filtered_vel_data[i]/WHEEL_MAX_ANGULAR_VELOCITY * 255; }
		}

		for (int i = 0; i < 4; i++)
		{
			Motor_Set_PWM_Duty(mecabot_motor[i], duty[i]);
		}

		t_motor_control_prev = t;
	}

	if (is_pid)
	{
		if ((t >= 5000) && (speed_changed_100 == false))
		{
			for (int i=0; i<4; i++)
			{
				my_controller[i]->set_point = SET_POINT_100;
				//motor_duty[i] = 60;
			}
			set_point = 100;
			speed_changed_100 = true;
		}
		if ((t >= 15000) && (speed_changed_175 == false))
		{
			for (int i=0; i<4; i++)
			{
				my_controller[i]->set_point = SET_POINT_175;
				//motor_duty[i] = 80;
			}
			set_point = 175;
			speed_changed_175 = true;
		}
		if ((t >= 20000) && (speed_changed_250 == false))
		{
			for (int i=0; i<4; i++)
			{
				my_controller[i]->set_point = SET_POINT_250;
				//motor_duty[i] = 130;
			}
			set_point = 250;
			speed_changed_250 = true;
		}
		if ((t >= 25000) && (speed_changed_stop == false))
		{
			for (int i=0; i<4; i++)
			{
				my_controller[i]->set_point = 0;
				//motor_duty[i] = 0;
			}
			set_point = 0;
			speed_changed_stop = true;
		}
	}
}

void updateRpm()
{
	for (int i = 0; i < 4; ++i)
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

		switch (i) // Debug
		{
			case 0:
				FL_dtick = d_tick;
				break;
			case 1:
				FR_dtick = d_tick;
				break;
			case 2:
				BL_dtick = d_tick;
				break;
			case 3:
				BR_dtick = d_tick;
				break;
			default:
				break;
		}

		prev_tick[i] = encoder[i]->tick;

		// Get the number of rotation
		raw_vel_data[i] = static_cast<float>(d_tick) / ENCODER_RESOLUTION;
		// Get the rotation per second
		raw_vel_data[i] *= VEL_FEEDBACK_FREQUENCY;
		// Get the radians per second
		raw_vel_data[i] *= 2 * PI;

		// Apply Low Pass filter
		filtered_vel_data[i] = mecabot_motor[i]->direction * FO_IIR_Compute(encoder_filter[i], raw_vel_data[i]);

		raw_rpm_data[i] = raw_vel_data[i] / (2 * PI) * 60;
		filtered_rpm_data[i] = filtered_vel_data[i] / (2 * PI) * 60;
	}

	raw_FL_rpm = raw_rpm_data[front_left]; raw_FR_rpm = raw_rpm_data[front_right];
	raw_BL_rpm = raw_rpm_data[back_left];  raw_BR_rpm = raw_rpm_data[back_right];

	FL_rpm = filtered_rpm_data[front_left]; FR_rpm = filtered_rpm_data[front_right];
	BL_rpm = filtered_rpm_data[back_left];  BR_rpm = filtered_rpm_data[back_right];
}

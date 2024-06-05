/*
 * mecabot_hardware.cpp
 *
 *  Created on: May 25, 2024
 *      Author: ADMIN
 */

#include "mecabot_hardware.h"

Motor mecabot_motor[4] = { NULL, NULL, NULL, NULL };
Encoder encoder[4] = { NULL, NULL, NULL, NULL };
FO_IIR_Filter_t *filter[4] = { NULL, NULL, NULL, NULL };

uint32_t millis(void)
{
    return HAL_GetTick();
}

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
        .max_count = 0xFFFFFFFF,
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

    // Initialize the rpm filter
    float b_coeff[] = { 0.7548f, 0.7548f };
    float a_coeff[] = {    1.0f, 0.5095f };
    for (int i = 0; i < NUM_OF_MOTOR; i++)
    {
    	filter[i] = reinterpret_cast<FO_IIR_Filter_t*>(calloc(1, sizeof(FO_IIR_Filter_t)));
    	FO_IIR_Init(filter[i], b_coeff, a_coeff);
    }

    // TODO: Check if all encoders were initialized correctly
    return STATUS_OK;
}

status_t mecabot_motor_start(Motor motor)
{
    return Motor_Start(motor);
}

status_t mecabot_motor_stop(Motor motor)
{
    return Motor_Stop(motor);
}

status_t mecabot_motor_set_angular_velocity(Motor motor, float velocity)
{
    int8_t direction = (velocity >= 0) ? DIRECTION_FORWARD : DIRECTION_BACKWARD;
    if (direction != motor->direction)
    {
        Motor_SetDirection(motor, direction);
    }

    uint8_t duty = (uint8_t)(255 * fabs(velocity) / WHEEL_MAX_ANGULAR_VELOCITY);
    Motor_Set_PWM_Duty(motor, duty);

    return STATUS_OK;
}
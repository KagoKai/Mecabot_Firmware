/*
 * mecabot_hardware.cpp
 *
 *  Created on: May 25, 2024
 *      Author: ADMIN
 */

#include "mecabot_hardware.h"

Motor mecabot_motor[4] = { NULL, NULL, NULL, NULL };
Encoder encoder[4] = { NULL, NULL, NULL, NULL };
FO_IIR_Filter_t *encoder_filter[4] = { NULL, NULL, NULL, NULL };
PID_t *controller[4] = { NULL, NULL, NULL, NULL };
int16_t duty[NUM_OF_MOTOR] = { 0 };

MPU6050_t my_mpu = { .address = MPU6050_ADDR_LOW };
float gyro_buffer[3] = { 0.0f };					// Buffer to hold scaled gyro measurement.
float accel_buffer[3] = { 0.0f };					// Buffer to hold scaled accelerometer measurement.
float encoder_yaw = 0.0f, imu_yaw = 0.0f;			// Orientation (in radians);
bool use_pid = true;
float kp = 5.0, ki = 0.0, kd = 0;

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

    // Initialize the encoder Low-pass Filter
    float b_coeff[] = { 0.2452,	0.2452 };
    float a_coeff[] = {    1.0, -0.5095 };
	for (int i = 0; i < NUM_OF_MOTOR; i++)
	{
		encoder_filter[i] = new FO_IIR_Filter_t;
		FO_IIR_Init(encoder_filter[i], b_coeff, a_coeff);
	}
    // TODO: Check if all encoders were initialized correctly
    return STATUS_OK;
}

status_t mecabot_pid_init(void)
{
	for (int i = 0; i < NUM_OF_MOTOR; i++)
	{
		controller[i] = PID_Init(kp, ki, kd);
	}

	return STATUS_OK;
}

status_t mecabot_mpu_init(void)
{
    MPU6050_Handle_t mpu_handle =
    {
        .rate_div = Rate_1KHz_Div,
        .gyro_range = Gyro_Range_250s,
        .accel_range = Accel_Range_2g,
		.filter_bandwidth = DLPF_21A_20G_Hz
    };

    return MPU6050_Init(&hi2c1, &my_mpu, mpu_handle, 0);
}

status_t mecabot_motor_start(Motor motor)
{
    return Motor_Start(motor);
}

status_t mecabot_motor_stop(Motor motor)
{
    return Motor_Stop(motor);
}

status_t mecabot_encoder_start(Encoder encoder)
{
	return Encoder_Start(encoder);
}

uint16_t mecabot_encoder_read(Encoder encoder)
{
	return encoder->tick;
}

status_t mecabot_imu_read_gyro(float *gyro_buffer)
{
    MPU6050_ReadGyroscope(&hi2c1, &my_mpu);

    gyro_buffer[0] = my_mpu.gyro_scaled.x;
    gyro_buffer[1] = my_mpu.gyro_scaled.y;
    gyro_buffer[2] = my_mpu.gyro_scaled.z;

    return STATUS_OK;
}

status_t mecabot_imu_read_accel(float *accel_buffer)
{
    MPU6050_ReadAccelerometer(&hi2c1, &my_mpu);

    accel_buffer[0] = my_mpu.accel_scaled.x;
    accel_buffer[1] = my_mpu.accel_scaled.y;
    accel_buffer[2] = my_mpu.accel_scaled.z;

    return STATUS_OK;
}

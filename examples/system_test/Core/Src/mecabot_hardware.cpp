#include "mecabot_hardware.h"

Motor mecabot_motor[4] = { NULL, NULL, NULL, NULL };
Encoder encoder[4] = { NULL, NULL, NULL, NULL };

MPU6050_t my_mpu = {.address = MPU6050_ADDR_LOW};
MadgwickFilter_t my_filter;

uint32_t millis(void)
{
    return HAL_GetTick();
}

status_t mecabot_motor_init(void)
{
    /* Hard-coded value for STM32F103C8T6 */

    Motor_Handle_t handle = {
        .pwm_channel = TIM_CHANNEL_1,
        .direction_port = GPIOB,
        .direction_pin = GPIO_PIN_12,
        .pwm_frequency = MOTOR_PWM_FREQUENCY
    };
    mecabot_motor[front_left] = Motor_Init(handle);

    handle.pwm_channel = TIM_CHANNEL_2;
    handle.direction_port = GPIOB;
    handle.direction_pin = GPIO_PIN_13;
    mecabot_motor[front_right] = Motor_Init(handle);

    handle.pwm_channel = TIM_CHANNEL_3;
    handle.direction_port = GPIOB;
    handle.direction_pin = GPIO_PIN_14;
    mecabot_motor[back_left] = Motor_Init(handle);

    handle.pwm_channel = TIM_CHANNEL_4;
    handle.direction_port = GPIOB;
    handle.direction_pin = GPIO_PIN_15;
    mecabot_motor[back_right] = Motor_Init(handle);

    // TODO: Check if all motors were initialized correctly
    return STATUS_OK;
}

status_t mecabot_encoder_init(void)
{
    Encoder_Handle_t handle_encoder = {
        .max_count = 0xFFFFFFFF,
        .tick_read_channel = TIM_CHANNEL_1
    };
    encoder[front_left] = Encoder_Init(handle_encoder);

    handle_encoder.tick_read_channel = TIM_CHANNEL_2;
    encoder[front_right] = Encoder_Init(handle_encoder);

    handle_encoder.tick_read_channel = TIM_CHANNEL_3;
    encoder[back_left] = Encoder_Init(handle_encoder);

    handle_encoder.tick_read_channel = TIM_CHANNEL_4;
    encoder[back_right] = Encoder_Init(handle_encoder);

    // TODO: Check if all encoders were initialized correctly
    return STATUS_OK;
}

status_t mecabot_mpu_init(void)
{
    MPU6050_Handle_t mpu_handle = 
    {
        .rate_div = Rate_1KHz_Div,
        .gyro_range = Gyro_Range_250s,
        .accel_range = Accel_Range_2g
    };

    return MPU6050_Init(&hi2c1, &my_mpu, mpu_handle, 0);
}

status_t mecabot_madgwick_init(void)
{
    MadgwickFilter_Handle_t filter_handle = 
    {
        .beta = BETA,
        .sample_rate = 1000/100
    };

    return MadgwickFilter_Init(&my_filter, filter_handle);
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
    int8_t direction = (velocity > 0) ? DIRECTION_FORWARD : DIRECTION_BACKWARD;
    if (direction != motor->direction) 
        Motor_SetDirection(motor, direction);

    uint8_t duty = (uint8_t)(255 * fabs(velocity) / WHEEL_MAX_ANGULAR_VELOCITY);
    Motor_Set_PWM_Duty(motor, duty);

    return STATUS_OK;
}

status_t mecabot_motor_set_linear_velocity(Motor motor, float velocity)
{
    int8_t direction = (velocity > 0) ? DIRECTION_FORWARD : DIRECTION_BACKWARD;
    if (direction != motor->direction) 
        Motor_SetDirection(motor, direction);

    uint8_t duty = (uint8_t)(255 * fabs(velocity) / WHEEL_MAX_LINEAR_VELOCITY);
    Motor_Set_PWM_Duty(motor, duty);

    return STATUS_OK;
}

uint32_t mecabot_encoder_read(Encoder encoder)
{
    return encoder->tick;
}

status_t mecabot_imu_read_gyro(float *gyro_buffer)
{
    MPU6050_ReadGyroscope(&hi2c1, &my_mpu);

    gyro_buffer[0] = DEG2RAD(my_mpu.gyro_scaled.x);
    gyro_buffer[1] = DEG2RAD(my_mpu.gyro_scaled.y);
    gyro_buffer[2] = DEG2RAD(my_mpu.gyro_scaled.z);

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

status_t mecabot_imu_get_quaternion(Quaternion_t *quat_buffer)
{
    MPU6050_ReadAccelerometer(&hi2c1, &my_mpu);
	MPU6050_ReadGyroscope(&hi2c1, &my_mpu);

	float 	gx = DEG2RAD(my_mpu.gyro_scaled.x),
			gy = DEG2RAD(my_mpu.gyro_scaled.y),
			gz = DEG2RAD(my_mpu.gyro_scaled.z),
			ax = my_mpu.accel_scaled.x,
			ay = my_mpu.accel_scaled.y,
			az = my_mpu.accel_scaled.z;

	MadgwickFilter_Update_IMU(&my_filter, gx, gy, gz, ax, ay, az);
	
    quat_buffer->q0 = my_filter.q.q0;
    quat_buffer->q1 = my_filter.q.q1;
    quat_buffer->q2 = my_filter.q.q2;
    quat_buffer->q3 = my_filter.q.q3;
}

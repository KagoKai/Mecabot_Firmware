#include "mecabot_hardware.h"

Motor motor[4];
Encoder encoder[4];

MPU6050_t my_mpu = {.address = MPU6050_ADDR_LOW};
MadgwickFilter_t my_filter;

status_t mecabot_motor_init(void)
{
    Motor_Handle_t handle_FL = {
      // TODO: Setup handle  
    };
    motor[front_left] = Motor_Init(handle_FL);

    Motor_Handle_t handle_FR = {
      // TODO: Setup handle  
    };
    motor[front_right] = Motor_Init(handle_FR);

    Motor_Handle_t handle_BL = {
      // TODO: Setup handle  
    };
    motor[back_left] = Motor_Init(handle_BL);

    Motor_Handle_t handle_BR = {
      // TODO: Setup handle  
    };
    motor[back_right] = Motor_Init(handle_BR);
}

status_t mecabot_encoder_init(void)
{
    Encoder_Handle_t handle_encoder = {
        .max_count = 0xFFFFFFFF,
        .tick_read_channel = TIM_CHANNEL_1
    };

    for (int i=0; i < NUM_OF_MOTOR; i++)
    {
        
    }
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

status_t mecabot_motor_start(Motor_t *motor);
status_t mecabot_motor_stop(Motor_t *motor);
status_t mecabot_motor_set_speed(Motor_t *motor, float speed);

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

status_t mecabot_encoder_read(Motor_t *motor);
#ifndef _MECABOT_HARDWARE_H_
#define _MECABOT_HARDWARE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "operation_status.h"
#include "dc_motor.h"
#include "encoder.h"
#include "mpu6050.h"
#include "madgwick_filter.h"

#define PI      3.14159265359

#define NUM_OF_MOTOR    4

/* Convert constant */
#define DEG2RAD(x)      (x * PI / 180.0f)     /*!< Convert from degree to radian (PI/180) */
#define RAD2DEG(x)      (x * 180.0f / PI)     /*!< convert from radian to degree (180/PI) */

/*
* Madgwick filter parameters
*/
#define GYRO_MEAN_ERROR     PI * (5.0f / 180.0f) // 5 deg/s gyroscope measurement error (in rad/s)  *from paper*
#define BETA                sqrt(3.0f/4.0f) * GYRO_MEAN_ERROR    //*from paper*
#define FILTER_RATE         100

/*
* Robot chassis parameters
*/
#define WHEEL_RADIUS            0.03
#define WHEEL_SEPARATION_X      0.010   
#define WHEEL_SEPARATION_Y      0.007

typedef enum
{
    front_left  = 0,
    front_right = 1,
    back_left   = 2,
    back_right  = 3
};

status_t mecabot_motor_init(void);
status_t mecabot_encoder_init(void);
status_t mecabot_mpu_init(void);
status_t mecabot_madgwick_init(void);

status_t mecabot_motor_start(Motor_t *motor);
status_t mecabot_motor_stop(Motor_t *motor);
status_t mecabot_motor_set_speed(Motor_t *motor, float speed);

status_t mecabot_encoder_read(Motor_t *motor);

status_t mecabot_imu_read_gyro(float *gyro_buffer);
status_t mecabot_imu_read_accel(float *accel_buffer);
status_t mecabot_imu_get_quaternion(Quaternion_t *quat_buffer);

#ifdef __cplusplus
}
#endif

#endif
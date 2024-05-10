#ifndef _MECABOT_HARDWARE_H_
#define _MECABOT_HARDWARE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include "operation_status.h"
#include "dc_motor.h"
#include "encoder.h"
#include "mpu6050.h"
#include "madgwick_filter.h"

#define PI      3.14159265359

#define NUM_OF_MOTOR        4
#define ENCODER_RESOLUTION  20

/* Convert constant */
#define DEG2RAD(x)      ((x) * PI / 180.0f)                     /*!< Convert from degree to radian (PI/180) */
#define RAD2DEG(x)      ((x) * 180.0f / PI)                     /*!< Convert from radian to degree (180/PI) */
#define TICK2RAD(x)     ((x) / ENCODER_RESOLUTION * 2 * PI)     /*!< Convert from encoder tick to radians */

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
#define INV_WHEEL_RADIUS        (1 / WHEEL_RADIUS)
#define WHEEL_SEPARATION_X      0.115
#define WHEEL_SEPARATION_Y      0.138   

#define WHEEL_MAX_ANGULAR_VELOCITY     2 * PI                                          /*!< Wheel angular velocity rad/s */
#define WHEEL_MIN_ANGULAR_VELOCITY     -WHEEL_MAX_ANGULAR_VELOCITY
#define WHEEL_MAX_LINEAR_VELOCITY      (WHEEL_RADIUS * WHEEL_MAX_ANGULAR_VELOCITY)     /*!< Wheel linear velocity m/s */
#define WHEEL_MIN_LINEAR_VELOCITY      -WHEEL_MAX_LINEAR_VELOCITY


#define MOTOR_PWM_FREQUENCY     1000

typedef enum
{
    front_left  = 0,
    front_right = 1,
    back_left   = 2,
    back_right  = 3
}motor_id_t;

/**
* @brief Return the elapsed time (in miliseconds) since the MCU starts.
* 
* @return Elapsed time.
*/
uint32_t millis(void);

/**
* @brief Initialize the robot's motors. 
*        Modified according to the robot structure and the MCU used.
* 
* @return Operation status.
*/
status_t mecabot_motor_init(void);

/**
* @brief Initialize the robot's encoders. 
*        Modified according to the robot structure and the MCU used.
* 
* @return Operation status.
*/
status_t mecabot_encoder_init(void);

/**
* @brief Initialize the robot's MPU. 
*        Modified according to the robot structure and the sensor used.
* 
* @return Operation status.
*/
status_t mecabot_mpu_init(void);

/**
* @brief Initialize the madgwick filter to get orientation from MPU.
* 
* @return Operation status.
*/
status_t mecabot_madgwick_init(void);

/**
* @brief Start a robot motor.
*
* @param motor The motor object.
* 
* @return Operation status.
*/
status_t mecabot_motor_start(Motor motor);

/**
* @brief Stop a robot motor.
*
* @param motor The motor object.
* 
* @return Operation status.
*/
status_t mecabot_motor_stop(Motor motor);

/**
* @brief Set a motor angular velocity (rad/s).
*
* @param motor The motor object.
* @param speed The angular velocity value (rad/s).
* 
* @return Operation status.
*/
status_t mecabot_motor_set_angular_velocity(Motor motor, float velocity);

/**
* @brief Set a motor linear velocity (m/s).
*
* @param motor The motor object.
* @param speed The linear velocity value (m/s).
* 
* @return Operation status.
*/
status_t mecabot_motor_set_linear_velocity(Motor motor, float velocity);

/**
 * @brief Get the current encoder tick.
 * 
 * @param encoder The encoder object to read from.
 * 
 * @return The tick value (uint32_t).
*/
uint32_t mecabot_encoder_read(Encoder encoder);

/**
 * @brief Read the angular velocities (in rad/s) from the IMU.
 * 
 * @param gyro_buffer The pointer to the buffer used to stored the data.
 * 
 * @return Operation status. 
*/
status_t mecabot_imu_read_gyro(float *gyro_buffer);

/**
 * @brief Read the linear acceleration (in m/s^2) from the IMU.
 * 
 * @param accel_buffer The pointer to the buffer used to stored the data.
 * 
 * @return Operation status. 
*/
status_t mecabot_imu_read_accel(float *accel_buffer);

/**
 * @brief Calculate the robot orientation (as quaterniion) using the IMU measurement.
 * 
 * @param quat_buffer The pointer to the buffer used to stored the data.
 * 
 * @return Operation status.
*/
status_t mecabot_imu_get_quaternion(Quaternion_t *quat_buffer);

#ifdef __cplusplus
}
#endif

#endif
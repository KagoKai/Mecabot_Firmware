/*
 * mecabot_hardware.h
 *
 *  Created on: May 25, 2024
 *      Author: ADMIN
 */

#ifndef INC_MECABOT_HARDWARE_H_
#define INC_MECABOT_HARDWARE_H_

#include <math.h>
#include "operation_status.h"
#include "dc_motor_2.h"
#include "encoder.h"
#include "IIR_filter.h"
#include "pid.h"
#include "mpu6050.h"

#define NUM_OF_MOTOR        4
#define ENCODER_RESOLUTION  20

/* Convert constant */
#define TICK2RAD(x)     (static_cast<float>((x)) / ENCODER_RESOLUTION * 2 * PI)     /*!< Convert from encoder tick to radians */

/*
 * PID controller parameters
 */
#define KP		8
#define KI		20
#define KD 		0

/*
* Robot chassis parameters
*/
#define WHEEL_RADIUS            0.028
#define INV_WHEEL_RADIUS        (1 / WHEEL_RADIUS)
#define WHEEL_SEPARATION_X      0.115
#define WHEEL_SEPARATION_Y      0.138

#define WHEEL_MAX_ANGULAR_VELOCITY     	(5 * 2 * PI)    	/*!< Wheel angular velocity rad/s */
#define WHEEL_MIN_ANGULAR_VELOCITY		(PI / 6)			/*!< Slowest motor velocity. If set lower the motor will stop */

#define MOTOR_PWM_FREQUENCY     1000

extern I2C_HandleTypeDef hi2c1;

typedef enum
{
    front_left  = 0,
    front_right = 1,
    back_left   = 2,
    back_right  = 3
}motor_id_t;

extern Motor mecabot_motor[4];
extern Encoder encoder[4];
extern FO_IIR_Filter_t *encoder_filter[4];
extern PID_t *controller[4];
extern int16_t duty[NUM_OF_MOTOR];

extern MPU6050_t my_mpu;
extern float gyro_buffer[3];					// Buffer to hold scaled gyro measurement.
extern float accel_buffer[3];					// Buffer to hold scaled accelerometer measurement.
extern float encoder_yaw, imu_yaw;			// Orientation (in radians);
extern bool use_pid;
extern float kp, ki, kd;

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
* @brief Initialize the robot's controllers for each wheel.
*        Modified according to the robot structure and the MCU used.
*
* @return Operation status.
*/
status_t mecabot_pid_init(void);

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
* @brief Start a robot encoder.
*
* @param encoder The encoder object.
*
* @return Operation status.
*/
status_t mecabot_encoder_start(Encoder encoder);

/**
 * @brief Return the tick value read by the encoder.
 *
 * @param encoder The encoder object.
 *
 * @return The tick value.
 */
uint16_t mecabot_encoder_read(Encoder encoder);


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
#endif /* INC_MECABOT_HARDWARE_H_ */

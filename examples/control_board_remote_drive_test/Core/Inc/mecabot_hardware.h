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
#include "dc_motor.h"
#include "encoder.h"
#include "IIR_filter.h"

#define PI      3.14159265359f

#define NUM_OF_MOTOR        4
#define ENCODER_RESOLUTION  20

/* Convert constant */
#define DEG2RAD(x)      ((x) * PI / 180.0f)                     /*!< Convert from degree to radian (PI/180) */
#define RAD2DEG(x)      ((x) * 180.0f / PI)                     /*!< Convert from radian to degree (180/PI) */
#define TICK2RAD(x)     ((x) / ENCODER_RESOLUTION * 2 * PI)     /*!< Convert from encoder tick to radians */

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

#endif /* INC_MECABOT_HARDWARE_H_ */
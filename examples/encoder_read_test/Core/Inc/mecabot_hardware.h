#ifndef _MECABOT_HARDWARE_H_
#define _MECABOT_HARDWARE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include "operation_status.h"
#include "dc_motor.h"
#include "encoder.h"

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
#define WHEEL_SEPARATION_X      0.007
#define WHEEL_SEPARATION_Y      0.0135
#define TURNING_RADIUS          0.08

/*
 * Robot speed parameters
 */
#define ROBOT_MAX_LINEAR_VELOCITY		(WHEEL_RADIUS * 2 * PI)
#define ROBOT_MIN_LINEAR_VELOCITY		-ROBOT_MAX_LINEAR_VELOCITY
#define ROBOT_MAX_ANGULAR_VELOCITY		(ROBOT_MAX_LINEAR_VELOCITY / TURNING_RADIUS)  /*!< Max angular velocity */
#define ROBOT_MIN_ANGULAR_VELOCITY		-ROBOT_MAX_ANGULAR_VELOCITY

#define WHEEL_MAX_VELOCITY				(WHEEL_RADIUS * 2 * PI)
#define WHEEL_MIN_VELOCITY				0

typedef enum
{
    front_left  = 0,
    front_right = 1,
    back_left   = 2,
    back_right  = 3
};

status_t mecabot_motor_init(void);
status_t mecabot_encoder_init(void);

status_t mecabot_motor_start(Motor motor);
status_t mecabot_motor_stop(Motor motor);
status_t mecabot_motor_set_velocity(Motor motor, float velocity);

uint32_t mecabot_encoder_read(Encoder encoder);

#ifdef __cplusplus
}
#endif

#endif
#ifndef _MECABOT_PID_H_
#define _MECABOT_PID_H_

#define NUM_OF_MOTOR        4

#define MAX_INTEGRAL_SUM    30
#define MIN_INTEGRAL_SUM   -MAX_INTEGRAL_SUM

#define MAX_PID_OUTPUT          255          // The controller output is the PWM signal: Limited to [0, 255]
#define MIN_PID_OUTPUT          0

#include <stdlib.h>

typedef struct
{
    float set_point;
    float kp, ki, kd;
    float integral_sum[NUM_OF_MOTOR];
    float last_input[NUM_OF_MOTOR];
}Mecabot_PID_t;

/** @brief Initialize a PID controller object.
 * 
 * @param p The proportional gain.
 * @param i The integral gain.
 * @param d The derivative gain.
 * 
 * @return The controller object.
*/
Mecabot_PID_t* PID_Init(float p, float i, float d);

/** @brief Set the gain values for the controller.
 * 
 * @param p The proportional gain.
 * @param i The integral gain.
 * @param d The derivative gain.
 * 
 * @return Operation status.
*/
void Mecabot_PID_SetGain(Mecabot_PID_t *controller, float p, float i, float d);

/** @brief Compute the output value.
 * 
 * @param controller The pointer to the PID controller object.
 * @param input The feedback input to the PID controller.
 * @param dt The time period between 2 calls (should be constant).
 * 
 * @return The output value.
*/
void Mecabot_PID_Compute(Mecabot_PID_t *controller, const float (&input)[NUM_OF_MOTOR], float (&output)[NUM_OF_MOTOR], double dt);

#endif

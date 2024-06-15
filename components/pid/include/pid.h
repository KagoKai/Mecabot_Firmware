#ifndef _PID_H_
#define _PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_INTEGRAL    160
#define MIN_INTEGRAL   -MAX_INTEGRAL

#define MAX_PID_OUTPUT          255          // The controller output is the PWM signal: Limited to [0, 255]
#define MIN_PID_OUTPUT          -255

#include <stdlib.h>

typedef struct
{
    double set_point;
    double kp, ki, kd;
    double P, I, D;
    double integral_sum;
    double last_input;
}PID_t;

/** @brief Initialize a PID controller object.
 * 
 * @param p The proportional gain.
 * @param i The integral gain.
 * @param d The derivative gain.
 * 
 * @return The controller object.
*/
PID_t* PID_Init(double p, double i, double d);

/** @brief Set the gain values for the controller.
 * 
 * @param p The proportional gain.
 * @param i The integral gain.
 * @param d The derivative gain.
 * 
 * @return Operation status.
*/
void PID_SetGain(PID_t *controller, double p, double i, double d);

/** @brief Compute the output value.
 * 
 * @param controller The pointer to the PID controller object.
 * @param input The feedback input to the PID controller.
 * @param dt The time period between 2 calls (should be constant).
 * 
 * @return The output value.
*/
double PID_Compute(PID_t *controller, double input, double dt);

#ifdef __cplusplus
}
#endif

#endif

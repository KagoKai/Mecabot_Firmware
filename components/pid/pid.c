#include "pid.h"

PID_t* PID_Init(double p, double i, double d)
{
    PID_t *object = calloc(1, sizeof(PID_t));

    object->set_point = 0;
    
    object->kp = p;
    object->ki = i;
    object->kd = d;

    object->integral_sum = 0;
    object->last_input = 0;

    return object;
}

void PID_SetGain(PID_t *controller, double p, double i, double d)
{
    controller->kp = p;
    controller->ki = i;
    controller->kd = d;
}

double PID_Compute(PID_t *controller, double input, double dt)
{
    // Error calculation
    double err = controller->set_point - input;

    // Integral component
    controller->integral_sum += err * (dt/1000);
    if (controller->integral_sum >= MAX_INTEGRAL_SUM)
    {
        controller->integral_sum = MAX_INTEGRAL_SUM;
    }
    if (controller->integral_sum <= MIN_INTEGRAL_SUM)
    {
        controller->integral_sum = MIN_INTEGRAL_SUM;
    }
    double iError = controller->integral_sum;

    // Derivative component
    double dError = (input - controller->last_input) / (dt/1000);
    controller->last_input = input;

    // Calculate the output
    double output = (controller->kp * err + controller->ki * iError + controller->kd * dError);
    if (output >= MAX_PID_OUTPUT)
    {
        output = MAX_PID_OUTPUT;
    }
    else if (output <= MIN_PID_OUTPUT)
    {
        output = MIN_PID_OUTPUT;
    }

    return output;
}
#include "pid.h"

PID_t* PID_Init(double p, double i, double d)
{
    PID_t *object = calloc(1, sizeof(PID_t));

    object->set_point = 0;
    
    object->kp = p;
    object->ki = i;
    object->kd = d;

    object->P = 0;
    object->I = 0;
    object->D = 0;

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

    // P
    controller->P = err * controller->kp;

    // I
    controller->integral_sum += err * (dt/1000);
    controller->I = controller->integral_sum * controller->ki;
    if (controller->I >= MAX_INTEGRAL)
    {
        controller->I = MAX_INTEGRAL;
    }
    if (controller->I <= MIN_INTEGRAL)
    {
        controller->I = MIN_INTEGRAL;
    }

    // D
    double dError = (input - controller->last_input) / (dt/1000);
    controller->last_input = input;
    controller->D = dError  * controller->kd;

    // Calculate the output
    double output = controller->P + controller->I + controller->D;
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
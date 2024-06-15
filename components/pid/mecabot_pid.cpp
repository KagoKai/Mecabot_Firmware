#include "mecabot_pid.h"

Mecabot_PID_t* PID_Init(float p, float i, float d)
{
    Mecabot_PID_t *object = new Mecabot_PID_t;

    object->set_point = 0;
    
    object->kp = p;
    object->ki = i;
    object->kd = d;

    for (int i = 0; i < NUM_OF_MOTOR; i++)
    {
        object->integral_sum[i] = 0;
        object->last_input[i] = 0;
    }
    
    return object;
}

void Mecabot_PID_SetGain(Mecabot_PID_t *controller, float p, float i, float d)
{
    controller->kp = p;
    controller->ki = i;
    controller->kd = d;
}

void Mecabot_PID_Compute(Mecabot_PID_t *controller, const float (&input)[NUM_OF_MOTOR], float (&output)[NUM_OF_MOTOR], double dt)
{
    
}
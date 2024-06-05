#include "IIR_filter.h"

void FO_IIR_Init(FO_IIR_Filter_t *instance, float *b_coefficient, float *a_coefficient)
{
    for (int i = 0; i < 2; i++)
    {
        instance->b[i] = b_coefficient[i];
        instance->a[i] = a_coefficient[i];
    }

    instance->prev_y = 0.0f;
    instance->prev_x = 0.0f;
}

float FO_IIR_Compute(FO_IIR_Filter_t *instance, float input)
{
    float output = -instance->a[1] * instance->prev_y + (instance->b[0] * input + instance->b[1] * instance->prev_x);

    instance->prev_y = output;
    instance->prev_x = input;

    return output;
}

void SO_IIR_Init(SO_IIR_Filter_t *instance, float *b_coefficient, float *a_coefficient)
{
    for (int i = 0; i < 3; i++)
    {
        instance->b[i] = b_coefficient[i];
        instance->a[i] = a_coefficient[i];
        
        if (i < 2)
        {
            instance->prev_y[i] = 0;
            instance->prev_x[i] = 0;
        }
    }
}

float SO_IIR_Compute(SO_IIR_Filter_t *instance, float input)
{
    float output = instance->b[0] * input;

    for (int i = 0; i < 2; i ++)
    {
        output += -instance->a[i+1] * instance->prev_y[i] + instance->b[i+1] * instance->prev_x[i];
    }

    instance->prev_y[1] = instance->prev_y[0];
    instance->prev_y[0] = output;
    instance->prev_x[1] = instance->prev_x[0];
    instance->prev_x[0] = input;

    return output;
}
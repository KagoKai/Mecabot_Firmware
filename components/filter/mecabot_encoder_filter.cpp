#include "mecabot_encoder_filter.h"

void Mecabot_FO_IIR_Init(FO_IIR_Filter_t *instance, const float (&b_coefficient)[2], const float (&a_coefficient)[2])
{
    for (int i = 0; i < 2; i++)
    {
        instance->b[i] = b_coefficient[i];
        instance->a[i] = a_coefficient[i];
    }

    for (int i = 0; i < NUM_OF_MOTOR; i++)
    {
        instance->prev_y[i] = 0.0f;
        instance->prev_x[i] = 0.0f;
    }
}

void Mecabot_FO_IIR_Compute(FO_IIR_Filter_t *instance, const float (&input)[NUM_OF_MOTOR], float (&output_buffer)[NUM_OF_MOTOR])
{
    for (int i = 0; i < NUM_OF_MOTOR; i++)
    {
        output_buffer[i] = -instance->a[1] * instance->prev_y[i] + (instance->b[0] * input[i] + instance->b[1] * instance->prev_x[i]);

        instance->prev_y[i] = output_buffer[i];
        instance->prev_x[i] = input[i];
    }
}
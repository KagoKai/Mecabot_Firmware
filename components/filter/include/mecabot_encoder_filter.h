#ifndef _MECABOT_ENCODER_FILTER_H_
#define _MECABOT_ENCODER_FILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_OF_MOTOR        4

#include <stdint.h>

typedef struct 
{
    float b[2];         // Numerator coefficient
    float a[2];         // Denominator coefficient
    float prev_y[NUM_OF_MOTOR];
    float prev_x[NUM_OF_MOTOR];
}FO_IIR_Filter_t;

/**
 * @brief Initialize a first order IIR filter instance for the mecabot wheel encoders.
 * 
 * @param instance The pointer to the filter instance.
 * @param b_coefficient The numerator coefficients array.
 * @param a_coefficient The denumerator coefficients array.
 * 
 * @return Void.
*/
void Mecabot_FO_IIR_Init(FO_IIR_Filter_t *instance, const float (&b_coefficient)[2], const float (&a_coefficient)[2]);

/**
 * @brief Compute the filtered value from the newest raw encoder inputs.
 * 
 * @param instance The pointer to the filter instance.
 * @param input The raw filter input data.
 * @param output_buffer The buffer that hold the filtered value.
 * 
 * @return Void.
*/
void Mecabot_FO_IIR_Compute(FO_IIR_Filter_t *instance, const float (&input)[NUM_OF_MOTOR], float (&output_buffer)[NUM_OF_MOTOR]);

#ifdef __cplusplus
}
#endif

#endif
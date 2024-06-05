#ifndef _IIR_FILTER_H_
#define _IIR_FILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct 
{
    float b[2];         // Numerator coefficient
    float a[2];         // Denominator coefficient
    float prev_y;
    float prev_x;
}FO_IIR_Filter_t;

/**
 * @brief Initialize a first order IIR filter instance.
 * 
 * @param instance The pointer to the filter instance.
 * @param b_coefficient The numerator coefficients array.
 * @param a_coefficient The to the denumerator coefficients array.
 * 
 * @return Void.
*/
void FO_IIR_Init(FO_IIR_Filter_t *instance, const float (&b_coefficient)[2], const float (&a_coefficient)[2]);

/**
 * @brief Compute the filtered value from the newest raw input data.
 * 
 * @param instance The pointer to the filter instance.
 * @param input The raw filter input data.
 * 
 * @return The filtered data.
*/
float FO_IIR_Compute(FO_IIR_Filter_t *instance, float input);

typedef struct 
{
    float a[3];
    float b[3];
    float prev_y[2];
    float prev_x[2];
}SO_IIR_Filter_t;

/**
 * @brief Initialize a second order IIR filter instance.
 * 
 * @param instance The pointer to the filter instance.
 * @param b_coefficient The numerator coefficients array.
 * @param a_coefficient The to the denumerator coefficients array.
 * 
 * @return Void.
*/
void SO_IIR_Init(SO_IIR_Filter_t *instance, const float (&b_coefficient)[3], const float (&a_coefficient)[3]);

/**
 * @brief Compute the filtered value from the newest raw input data.
 * 
 * @param instance The pointer to the filter instance.
 * @param input The raw filter input data.
 * 
 * @return The filtered data.
*/
float SO_IIR_Compute(SO_IIR_Filter_t *instance, float input);

#ifdef __cplusplus
}
#endif

#endif
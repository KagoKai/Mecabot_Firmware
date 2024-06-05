#ifndef _MOVING_AVG_FILTER_H_
#define _MOVING_AVG_FILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#define WINDOW_LENGTH   50

#include <stdint.h>

/**
 * @brief Implementation of the Simple Moving Average Filter Algorithm
*/
typedef struct 
{
    /* data */
    uint32_t history[WINDOW_LENGTH];
    uint32_t sum;
    uint32_t last_elem_cnt;
}MA_Filter_t;

/** @brief Initialize a Filter object.
 * 
 * @param instance The pointer to a filter instance.
 * 
 * @return Void.
*/
void Moving_Avg_Init(MA_Filter_t *instance);

/** @brief Apply the filter algorithm to the lastest data point.
 * 
 * @param instance The pointer to a filter instance.
 * @param raw_data The lastest raw data point.
 * 
 * @return The filtered value.
*/
uint32_t Moving_Avg_Compute(MA_Filter_t *instance, uint32_t raw_data);

#ifdef __cplusplus
}
#endif

#endif


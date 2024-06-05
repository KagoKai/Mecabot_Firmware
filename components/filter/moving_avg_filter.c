#include "moving_avg_filter.h"

void Moving_Avg_Init(MA_Filter_t *instance)
{
    instance->last_elem_cnt = 0;
    instance->sum = 0;

    for (int i = 0; i < WINDOW_LENGTH; i++)
    {
        instance->history[i] = 0;
    }
    
}

uint32_t Moving_Avg_Compute(MA_Filter_t *instance, uint32_t raw_data)
{
    instance->sum += raw_data;
    instance->sum -= instance->history[instance->last_elem_cnt];
    instance->history[instance->last_elem_cnt] = raw_data;

    instance->last_elem_cnt++;
    if (instance->last_elem_cnt == WINDOW_LENGTH)
    {
        instance->last_elem_cnt = 0;
    }
    
    return instance->sum / WINDOW_LENGTH;
}
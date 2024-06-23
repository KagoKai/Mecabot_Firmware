#include "encoder.h"

Encoder_t* Encoder_Init(Encoder_Handle_t handle)
{
    Encoder_t* object = calloc(1, sizeof(Encoder_t));

    object->tick = 0;
    object->max_count = handle.max_count;
    object->tick_read_channel = handle.tick_read_channel;

    return object;
}

status_t Encoder_Start(Encoder_t *encoder)
{
    HAL_StatusTypeDef ret = HAL_TIM_IC_Start_IT(&htim_encoder, encoder->tick_read_channel);

    if (ret != HAL_OK) return STATUS_FAIL;

    return STATUS_OK;
}

status_t Encoder_Stop(Encoder_t *encoder)
{
    HAL_StatusTypeDef ret = HAL_TIM_IC_Stop_IT(&htim_encoder, encoder->tick_read_channel);

    if (ret != HAL_OK) return STATUS_FAIL;

    return STATUS_OK;
}

status_t Encoder_GetTick(Encoder_t *encoder, uint16_t *buffer)
{
    uint32_t tmp = __HAL_TIM_GET_COMPARE(&htim_encoder, encoder->tick_read_channel);

    // Check for valid reading
    if (tmp > encoder->max_count) return STATUS_FAIL;

    *buffer = tmp;

    return STATUS_OK;
}

status_t Encoder_UpdateTick(Encoder_t *encoder)
{
    uint32_t tmp = __HAL_TIM_GET_COMPARE(&htim_encoder, encoder->tick_read_channel);

    // Check for valid reading
    if (tmp > encoder->max_count) return STATUS_FAIL;

    encoder->tick = tmp;

    return STATUS_OK;
}

status_t Encoder_Destroy(Encoder_t *encoder)
{
    free(encoder);
    encoder = NULL;

    return STATUS_OK;
}
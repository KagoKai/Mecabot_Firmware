#ifndef _ENCODER_H_
#define _ENCODER_H_

#ifdef __cplusplus
extern "C" {
#endif

#define STM32F103RCT6 // Change to BLUEPILL for Module Test

#include "stdlib.h"
#include "stdint.h"

#ifdef STM32F103RCT6
#include "stm32f103xe.h"
#endif /* Control Board */
#ifdef BLUEPILL
#include "stm32f103xb.h"
#endif /* Bluepill Module */

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include "operation_status.h"

#define ENCODER_MAX_COUNT       1000

extern TIM_HandleTypeDef htim_encoder;

typedef struct
{
    uint16_t tick;
    uint32_t max_count;
    uint32_t tick_read_channel;
}Encoder_t;

typedef Encoder_t* Encoder;

typedef struct 
{
    uint32_t max_count;
    uint16_t tick_read_channel;
}Encoder_Handle_t;

typedef Encoder_t* Encoder;

/** @brief Initialize an encoder object.
 * 
 * @param handle The encoder setting handle.
 * 
 * @return The encoder object.
*/
Encoder_t* Encoder_Init(Encoder_Handle_t handle);

/** @brief Start the encoder object (enable reading encoder ticks in Timer).
 * 
 * @param encoder The pointer to the encoder object.
 * 
 * @return Operation status.
*/
status_t Encoder_Start(Encoder_t *encoder);

/** @brief Stop the encoder object (disable reading encoder ticks in Timer).
 * 
*/
status_t Encoder_Stop(Encoder_t *encoder);

/** @brief Get the current tick value (from the Timer register) and write it to a buffer.
 * 
 * @param encoder The pointer to the encoder object.
 * @param buffer The buffer to store the value.
 * 
 * @return Operation status.
*/
status_t Encoder_GetTick(Encoder_t *encoder, uint16_t *buffer);

status_t Encoder_UpdateTick(Encoder_t *encoder);

/** @brief Destroy an encoder object - free its memory.
 * 
 * @param encoder The pointer to the encoder object.
 * 
 * @return Operation status.
*/
status_t Encoder_Destroy(Encoder_t *encoder);

#ifdef __cplusplus
}
#endif

#endif
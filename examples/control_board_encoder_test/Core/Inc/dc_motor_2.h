#ifndef _DC_MOTOR_H_
#define _DC_MOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#define STM32F103RCT6 // Change to BLUEPILL for Module Test

#include <stdlib.h>

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

//#define DEFAULT_CLOCK_FREQ       8000000U
#define DEFAULT_CLOCK_FREQ       32000000U

#define STATE_FORWARD        ((int8_t)1)
#define STATE_STOP           ((int8_t)0)
#define STATE_BACKWARD       ((int8_t)(-1))
#define STATE_BRAKE          ((int8_t)2)

#define DIRECTION_FORWARD        ((int8_t)1)
#define DIRECTION_STOP           ((int8_t)0)
#define DIRECTION_BACKWARD       ((int8_t)(-1))

#define TRUE    1
#define FALSE   0

#define MIN_DUTY				  0

extern TIM_HandleTypeDef htim_motor;

typedef uint8_t bool;

typedef struct 
{
    int8_t state;
    int8_t direction;
    uint8_t pwm_duty;

     /* Motor pin setting */
    uint32_t pwm_channel;
    GPIO_TypeDef *direction_port;
    uint16_t direction_pin;
}Motor_t;

typedef Motor_t* Motor;

typedef struct 
{
    /* Motor pin setting */
    uint32_t pwm_channel;
    GPIO_TypeDef *direction_port;
    uint16_t direction_pin;
    /* data */
    uint32_t pwm_frequency;
}Motor_Handle_t;

typedef Motor_t* Motor;

/** @brief Initialize a motor object.
 * 
 * @param handle The encoder setting handle.
 * 
 * @return The motor object.
*/
Motor_t* Motor_Init(Motor_Handle_t handle);

status_t Motor_Set_PWM_Frequency(uint32_t freq);

status_t Motor_Set_PWM_Duty(Motor_t *motor, int16_t duty);

status_t Motor_Start(Motor_t *motor);

status_t Motor_Stop(Motor_t *motor);

status_t Motor_ChangeState(Motor_t *motor, int8_t state);

status_t Motor_SetDirection(Motor_t *motor, int8_t direction);

/** @brief Destroy a motor object - free its memory.
 * 
 * @param motor The pointer to the motor object.
 * 
 * @return Operation status.
*/
status_t Motor_Destroy(Motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif
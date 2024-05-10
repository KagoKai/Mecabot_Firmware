#ifndef _DC_MOTOR_H_
#define _DC_MOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include "operation_status.h"

#define DEFAULT_CLOCK_FREQ       8000000U
#define DIRECTION_FORWARD        (int8_t)1
#define DIRECTION_BACKWARD       (int8_t)(-1)

extern TIM_HandleTypeDef htim_motor;

typedef struct 
{
    /* data */
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

status_t Motor_Set_PWM_Duty(Motor_t *motor, uint8_t duty);

status_t Motor_Start(Motor_t *motor);

status_t Motor_Stop(Motor_t *motor);

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
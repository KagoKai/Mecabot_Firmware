#include "dc_motor_2.h"

uint32_t pwm_freq;

Motor_t* Motor_Init(Motor_Handle_t handle)
{
    Motor_t *object = calloc(1, sizeof(Motor_t));

    object->state = STATE_STOP;
    object->direction = DIRECTION_STOP;
    object->pwm_duty = 0;

    object->pwm_channel = handle.pwm_channel;
    object->direction_port = handle.direction_port;
    object->direction_pin = handle.direction_pin;

    pwm_freq = handle.pwm_frequency;

    // Duty cycle Setting
    Motor_Set_PWM_Duty(object, 0);
    // Frequency Setting
    Motor_Set_PWM_Frequency(pwm_freq);

    return object;
}

status_t Motor_Set_PWM_Frequency(uint32_t freq)
{
    // The PWM frequency depends on: The clock frequency, the ARR value, the PSC value. 
    //              PWM frequency = Clock frequency / [ (ARR + 1) * (PSC + 1) ]
    pwm_freq = freq;

    uint32_t tim_arr_val = __HAL_TIM_GET_AUTORELOAD(&htim_motor);
    uint32_t tim_psc_val = (uint32_t)(DEFAULT_CLOCK_FREQ / (pwm_freq * (tim_arr_val + 1)) - 1);

    __HAL_TIM_SET_PRESCALER(&htim_motor, tim_psc_val);
    
    return STATUS_OK;
}

status_t Motor_Set_PWM_Duty(Motor_t *motor, int16_t duty)
{
    switch (motor->state)
    {
        case STATE_FORWARD:
        {
            if (duty > 0)
                motor->pwm_duty = duty;            // duty: 0 - Min, 255 - Max
            else if (duty <= 0)
            {
                Motor_ChangeState(motor, STATE_BRAKE);
                motor->pwm_duty =  0;
            } 
        }
            break;
        case STATE_BACKWARD:
            {
                if (duty >= 0)
                {
                    Motor_ChangeState(motor, STATE_BRAKE);
                    motor->pwm_duty = 0;
                }
                else if (duty < 0)
                    motor->pwm_duty = 255 + duty;
            }
            break;
        case STATE_STOP:
            {
                if (duty > MIN_DUTY)
                {
                    Motor_ChangeState(motor, STATE_FORWARD);
                    motor->pwm_duty = duty;
                }
                else if (duty < -MIN_DUTY)
                {
                    Motor_ChangeState(motor, STATE_BACKWARD);
                    motor->pwm_duty = 255 + duty;
                }
                else
                {
                    motor->pwm_duty = 0;
                }
            }
            break;
        case STATE_BRAKE:
            {
                motor->pwm_duty = 0;
            }
        default:
            break;
    }

    float duty_percentage = motor->pwm_duty / 255.0f;

    uint32_t tim_arr_val = __HAL_TIM_GET_AUTORELOAD(&htim_motor);
    uint32_t tim_ccr_val = (uint32_t)(duty_percentage * tim_arr_val);

    __HAL_TIM_SET_COMPARE(&htim_motor, motor->pwm_channel, tim_ccr_val);

    return STATUS_OK;
}

status_t Motor_Start(Motor_t *motor)
{
    Motor_ChangeState(motor, motor->state);
    HAL_StatusTypeDef ret = HAL_TIM_PWM_Start(&htim_motor, motor->pwm_channel);

    if (ret != HAL_OK) return STATUS_FAIL;

    return STATUS_OK;
}

status_t Motor_Stop(Motor_t *motor)
{
    HAL_StatusTypeDef ret = HAL_TIM_PWM_Stop(&htim_motor, motor->pwm_channel);

    motor->pwm_duty = 0;
    Motor_ChangeState(motor, STATE_STOP);
    Motor_Set_PWM_Duty(motor, 0);

    if (ret != HAL_OK) return STATUS_FAIL;

    return STATUS_OK;
}

status_t Motor_ChangeState(Motor_t *motor, int8_t state)
{
    motor->state = state;

    if (state != STATE_BRAKE)
    {
        Motor_SetDirection(motor, state);
    }

    return STATUS_OK;
}

status_t Motor_SetDirection(Motor_t *motor, int8_t direction)
{
    motor->direction = direction;

    switch (direction)
    {
    case DIRECTION_FORWARD:
    	HAL_GPIO_WritePin(motor->direction_port, motor->direction_pin, GPIO_PIN_RESET);
        break;
    case DIRECTION_STOP:
        HAL_GPIO_WritePin(motor->direction_port, motor->direction_pin, GPIO_PIN_RESET);
        break;
    case DIRECTION_BACKWARD:
        HAL_GPIO_WritePin(motor->direction_port, motor->direction_pin, GPIO_PIN_SET);
        break;
    default:
        break;
    }

    return STATUS_OK;
}

status_t Motor_Destroy(Motor_t *motor)
{
    free(motor);
    motor = NULL;

    return STATUS_OK;
}
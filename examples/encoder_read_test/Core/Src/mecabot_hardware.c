#include "mecabot_hardware.h"

Motor mecabot_motor[NUM_OF_MOTOR];
Encoder encoder[NUM_OF_MOTOR];

status_t mecabot_motor_init(void)
{
    Motor_Handle_t handle_FL = {
      .pwm_channel = TIM_CHANNEL_1,
	  .direction_port = GPIOB,
	  .direction_pin_A = GPIO_PIN_12,
	  .direction_pin_B = GPIO_PIN_12,
	  .pwm_frequency = 1000
    };
    mecabot_motor[front_left] = Motor_Init(handle_FL);

    Motor_Handle_t handle_FR = {
	  .pwm_channel = TIM_CHANNEL_2,
	  .direction_port = GPIOB,
	  .direction_pin_A = GPIO_PIN_13,
	  .direction_pin_B = GPIO_PIN_13,
	  .pwm_frequency = 1000
    };
    mecabot_motor[front_right] = Motor_Init(handle_FR);

    /*
    Motor_Handle_t handle_BL = {
	  .pwm_channel = TIM_CHANNEL_3,
	  .direction_port = GPIOB,
	  .direction_pin_A = GPIO_PIN_0,
	  .direction_pin_B = GPIO_PIN_0,
	  .pwm_frequency = 1000
    };
    mecabot_motor[back_left] = Motor_Init(handle_BL);

    Motor_Handle_t handle_BR = {
	  .pwm_channel = TIM_CHANNEL_4,
	  .direction_port = GPIOB,
	  .direction_pin_A = GPIO_PIN_1,
	  .direction_pin_B = GPIO_PIN_1,
	  .pwm_frequency = 1000
    };
    mecabot_motor[back_right] = Motor_Init(handle_BR);
    */

    return STATUS_OK;
}

status_t mecabot_encoder_init(void)
{
    Encoder_Handle_t handle_encoder = {
        .max_count = 0xFFFFFFFF,
        .tick_read_channel = TIM_CHANNEL_1
    };
    encoder[front_left] = Encoder_Init(handle_encoder);

    handle_encoder.tick_read_channel = TIM_CHANNEL_2;
    encoder[front_right] = Encoder_Init(handle_encoder);

    /*
    handle_encoder.tick_read_channel = TIM_CHANNEL_3;
    encoder[back_left] = Encoder_Init(handle_encoder);
        
    handle_encoder.tick_read_channel = TIM_CHANNEL_4;
	encoder[back_right] = Encoder_Init(handle_encoder);
	*/

	return STATUS_OK;
}

status_t mecabot_motor_start(Motor motor)
{
	Motor_Start(motor);

	return STATUS_OK;
}
status_t mecabot_motor_stop(Motor motor)
{
	Motor_Stop(motor);

	return STATUS_OK;
}

int8_t direction;
float velocity_debug;
status_t mecabot_motor_set_velocity(Motor motor, float velocity)
{
	direction = (velocity >= 0) ? DIRECTION_FORWARD : DIRECTION_BACKWARD;
	velocity_debug = velocity;
	if (direction != motor->direction)
	{
		motor->direction = direction;
		Motor_SetDirection(motor, direction);
	}

	uint8_t duty = (uint8_t)(255 * fabs(velocity) / WHEEL_MAX_VELOCITY);
	Motor_Set_PWM_Duty(motor, duty);

	return STATUS_OK;
}

uint32_t mecabot_encoder_read(Encoder encoder)
{
	return encoder->tick;
}

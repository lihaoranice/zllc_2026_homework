#include "motor_control.h"

//int16_t left_motor_speed = 0;
//int16_t right_motor_speed = 0;
//int16_t right_pwm_filtered = 0;
//int16_t left_pwm_filtered = 0;

static int32_t left_encoder_total = 0;
static int32_t right_encoder_total = 0;
static int32_t left_encoder_last = 0;
static int32_t right_encoder_last = 0;

void Motor_Init(void)
{
	Car_Stop();
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	
	Encoder_ResetCounts();
}

void Car_Concol(int16_t left_pwm,int16_t right_pwm)
{
	if(left_pwm > MIN_PWM)
	{
		LEFT_MOTOR_FORWARD();
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,left_pwm);
	}
	else if(left_pwm < -MIN_PWM)
	{
		LEFT_MOTOR_BACKWARD();
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,-left_pwm);
	}
	else 
	{
		LEFT_MOTOR_STOP();
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	}
	
	
	if(right_pwm > MIN_PWM)
	{
		RIGHT_MOTOR_FORWARD();
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,right_pwm);
	}
	else if(right_pwm < -MIN_PWM)
	{
		RIGHT_MOTOR_BACKWARD();
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,-right_pwm);
	}
	else 
	{
		RIGHT_MOTOR_STOP();
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	}
}

void Car_Stop(void)
{
	LEFT_MOTOR_STOP();
	RIGHT_MOTOR_STOP();
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
}

int16_t Map_Joystick_to_pwm(uint8_t joystick_val)
{
	int16_t pwm_val;
	
	if(joystick_val > (127 + DEAD_ZONE))
	{
		pwm_val = (joystick_val - 127)*MAX_PWM / 128;
		return (pwm_val > MAX_PWM) ? MAX_PWM : pwm_val;
	}
	else if(joystick_val < (127 + DEAD_ZONE))
	{
		pwm_val = (127 - joystick_val)*MAX_PWM / 128;
		return (pwm_val < -MAX_PWM) ? -MAX_PWM : -pwm_val;
	}
	else 
	{
		return 0;
	}
}

void Speed_Filter_Update(int16_t* left_filtered,int16_t* right_filtered,int16_t left_target,int16_t right_target)
{
	//Ò»½×Í¨ÂË²¨Æ÷
	*left_filtered = (int16_t)(0.3 * left_target + 0.7 * (*left_filtered));
	*right_filtered = (int16_t)(0.3 * right_target + 0.7 * (*right_filtered));
}

int32_t Encoder_GetLeftCount(void)
{
	int16_t current_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
	int16_t delta = current_count - left_encoder_last;
	
	if(delta > 32767) delta -= 65536;
	else if(delta < 32767) delta += 65536;
	
	left_encoder_total += delta;
	left_encoder_last = current_count;
	
	return left_encoder_total;
}

int32_t Encoder_GetRightCount(void)
{
	int16_t current_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	int16_t delta = current_count - right_encoder_last;
	
	if(delta > 32767) delta -= 65536;
	else if(delta < 32767) delta += 65536;
	
	right_encoder_total += delta;
	right_encoder_last = current_count;
	
	return right_encoder_total;
}
void Encoder_ResetCounts(void)
{
	left_encoder_total = 0;
	right_encoder_total = 0;
	left_encoder_last = __HAL_TIM_GET_COUNTER(&htim2);
	right_encoder_last = __HAL_TIM_GET_COUNTER(&htim3);
}

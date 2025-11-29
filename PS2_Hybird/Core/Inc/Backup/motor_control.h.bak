#ifndef __MOTOR_CONTROL_H
#define __MPTOR_CONTROL_H

#include "main.h"
#include "tim.h"

#define LEFT_MOTOR_FORWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); \
}while(0)

#define LEFT_MOTOR_BACKWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET); \
}while(0)

#define LEFT_MOTOR_STOP() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); \
}while(0)

#define RIGHT_MOTOR_FORWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET); \
}while(0)

#define RIGHT_MOTOR_BACKWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET); \
}while(0)

#define RIGHT_MOTOR_STOP() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET); \
}while(0)

#define MAX_PWM 900		//最大占空比
#define MIN_PWM 150		//最小占空比
#define DEAD_ZONE 15 	//死区

void Motor_Init(void);
void Car_Concol(int16_t left_pwm,int16_t right_pwm);
void Car_Stop(void);
int16_t Map_Joystick_to_pwm(uint8_t joystick_val);
void Speed_Filter_Update(int16_t* left_filtered,int16_t* right_filtered,int16_t left_target,int16_t right_target);
int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);
void Encoder_ResetCounts(void);

//extern int16_t left_motor_speed;
//extern int16_t right_motor_speed;
//extern int16_t left_pwm_filtered;
//extern int16_t right_pwm_filtered;

#endif

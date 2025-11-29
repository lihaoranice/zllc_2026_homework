#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"
#include "tim.h"
#include <stdint.h>

/**
 * @brief 麦克纳姆轮电机驱动系统配置说明
 * 
 * 硬件配置:
 * - 4个麦克纳姆轮 + JGB37-520电机 (左前, 左后, 右前, 右后)
 * - 2个L298N双路H桥驱动器
 * 
 * 麦克纳姆轮布局 (俯视图):
 *   左前●────────●右前
 *      │  ╱  ╲  │
 *      │ ╱    ╲ │     主辊轴方向: / 和 \
 *      │╱      ╲│
 *   左后●────────●右后
 * 
 * 驱动分配:
 * - L298N#1: 控制左前+左后电机 (并联驱动)
 *   - ENA: PA8 (TIM1_CH1 PWM)
 *   - IN1: PA0 (方向控制1)
 *   - IN2: PA1 (方向控制2)
 *   - OUT1+OUT3: 左前+左后 M+
 *   - OUT2+OUT4: 左前+左后 M-
 * 
 * - L298N#2: 控制右前+右后电机 (并联驱动)
 *   - ENB: PA9 (TIM1_CH2 PWM)
 *   - IN3: PA4 (方向控制1)
 *   - IN4: PA5 (方向控制2)
 *   - OUT1+OUT3: 右前+右后 M+
 *   - OUT2+OUT4: 右前+右后 M-
 * 
 * ⚠️ 硬件限制说明:
 * 由于左右两侧电机并联驱动，只有2个独立控制通道，
 * 无法实现麦克纳姆轮的纯平移功能（需要4电机独立控制）。
 * 当前可实现: 前后移动、原地旋转、斜向移动(速度差实现)
 * 
 * 编码器配置:
 * - TIM2: 左侧编码器 (左前+左后并联) -> PA15/PB3
 * - TIM3: 右侧编码器 (右前+右后并联) -> PB4/PB5
 */

// L298N#1控制左前+左后电机 (PA0/PA1)
#define LEFT_MOTORS_FORWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); \
}while(0)

#define LEFT_MOTORS_BACKWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET); \
}while(0)

#define LEFT_MOTORS_STOP() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); \
}while(0)

// L298N#2控制右前+右后电机 (PA4/PA5)
#define RIGHT_MOTORS_FORWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET); \
}while(0)

#define RIGHT_MOTORS_BACKWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET); \
}while(0)

#define RIGHT_MOTORS_STOP() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET); \
}while(0)

#define MAX_PWM 900		//最大占空比
#define MIN_PWM 150		//最小占空比
#define DEAD_ZONE 15 	//摇杆死区

// 电机初始化和控制函数
void Motor_Init(void);
void Car_Control(int16_t left_pwm, int16_t right_pwm);
void Car_Stop(void);
void Car_Forward(int16_t speed);
void Car_Backward(int16_t speed);
void Car_TurnLeft(int16_t speed);
void Car_TurnRight(int16_t speed);
void Car_Move_Left(int16_t speed);   // 左斜移（麦轮模式）
void Car_Move_Right(int16_t speed);  // 右斜移（麦轮模式）
void Mecanum_Move(int16_t vx, int16_t vy, int16_t vw);  // 麦轮运动学控制

// 摇杆映射和滤波函数
int16_t Map_Joystick_To_PWM(uint8_t joystick_val);
void Speed_Filter_Update(int16_t* left_filtered, int16_t* right_filtered, int16_t left_target, int16_t right_target);

// 编码器函数
int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);
void Encoder_ResetCounts(void);

// PS2手柄控制函数（麦克纳姆轮版本）
void PS2_Control_Car(uint8_t ljoy_lr, uint8_t ljoy_ud, uint8_t btn1);

#endif

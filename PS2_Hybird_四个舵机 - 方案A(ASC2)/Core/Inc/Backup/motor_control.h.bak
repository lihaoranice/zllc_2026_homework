#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"
#include "tim.h"
#include <stdint.h>

/**
 * @brief 麦克纳姆轮电机驱动系统配置说明
 * 
 * 硬件配置:
 * - 4个麦克纳姆轮电机 (左前LF, 左后LB, 右前RF, 右后RB)
 * - 2个L298N双路H桥驱动器
 * 
 * 麦克纳姆轮布局 (俯视图):
 *   LF ↗     ↖ RF
 *        车体
 *   LB ↖     ↗ RB
 * 
 * 驱动分配 (四轮独立控制):
 * - L298N#1: 控制左前+左后电机
 *   - OUT1/2: 左前轮 (LF) - 方向: PA0/PA1
 *   - OUT3/4: 左后轮 (LB) - 方向: PA2/PA3
 *   - ENA: PA8 (TIM1_CH1 PWM) - 左前轮速度
 *   - ENB: PA10 (TIM1_CH3 PWM) - 左后轮速度
 * 
 * - L298N#2: 控制右前+右后电机
 *   - OUT1/2: 右前轮 (RF) - 方向: PA4/PA5
 *   - OUT3/4: 右后轮 (RB) - 方向: PA6/PA7
 *   - ENA: PA9 (TIM1_CH2 PWM) - 右前轮速度
 *   - ENB: PA11 (TIM1_CH4 PWM) - 右后轮速度
 * 
 * 编码器配置:
 * - TIM2: 左侧编码器 -> PA15/PB3
 * - TIM3: 右侧编码器 -> PB4/PB5
 *
 * 麦克纳姆轮运动学:
 * - 前进: LF+ RF+ LB+ RB+
 * - 后退: LF- RF- LB- RB-
 * - 左平移: LF- RF+ LB+ RB-
 * - 右平移: LF+ RF- LB- RB+
 * - 原地左转: LF- RF+ LB- RB+
 * - 原地右转: LF+ RF- LB+ RB-
 */

// 左前轮 (LF) - L298N#1 OUT1/2 (PA0/PA1)
#define LF_FORWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); \
}while(0)

#define LF_BACKWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET); \
}while(0)

#define LF_STOP() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); \
}while(0)

// 左后轮 (LB) - L298N#1 OUT3/4 (PA2/PA3)
#define LB_FORWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET); \
}while(0)

#define LB_BACKWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET); \
}while(0)

#define LB_STOP() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET); \
}while(0)

// 右前轮 (RF) - L298N#2 OUT1/2 (PA4/PA5)
#define RF_FORWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET); \
}while(0)

#define RF_BACKWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET); \
}while(0)

#define RF_STOP() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET); \
}while(0)

// 右后轮 (RB) - L298N#2 OUT3/4 (PA6/PA7)
#define RB_FORWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET); \
}while(0)

#define RB_BACKWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET); \
}while(0)

#define RB_STOP() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET); \
}while(0)

#define MAX_PWM 900		//最大占空比
#define MIN_PWM 150		//最小占空比
#define DEAD_ZONE 15 	//摇杆死区

// 电机初始化和控制函数
void Motor_Init(void);
void Mecanum_Control(int16_t lf_pwm, int16_t rf_pwm, int16_t lb_pwm, int16_t rb_pwm);  // 四轮独立控制
void Car_Stop(void);
void Car_Forward(int16_t speed);      // 前进
void Car_Backward(int16_t speed);     // 后退
void Car_Move_Left(int16_t speed);    // 左平移
void Car_Move_Right(int16_t speed);   // 右平移
void Car_TurnLeft(int16_t speed);     // 原地左转
void Car_TurnRight(int16_t speed);    // 原地右转
void Car_Move_FL(int16_t speed);      // 左前斜移
void Car_Move_FR(int16_t speed);      // 右前斜移
void Car_Move_BL(int16_t speed);      // 左后斜移
void Car_Move_BR(int16_t speed);      // 右后斜移

// 摇杆映射和滤波函数
int16_t Map_Joystick_To_PWM(uint8_t joystick_val);
void Speed_Filter_Update(int16_t* left_filtered, int16_t* right_filtered, int16_t left_target, int16_t right_target);

// 编码器函数
int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);
void Encoder_ResetCounts(void);

// PS2手柄控制函数（新版：支持平移和按键旋转）
void PS2_Control_Car(uint8_t ljoy_lr, uint8_t ljoy_ud, uint8_t btn1);

#endif

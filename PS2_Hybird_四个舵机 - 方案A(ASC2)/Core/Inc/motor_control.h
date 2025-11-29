#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"
#include "tim.h"
#include <stdint.h>

/**
 * @brief 电机控制系统的说明
 * 
 * 电机控制:
 * - 4个电机的控制 (前LF, 后LB, 前RF, 后RB)
 * - 2个L298N双H桥驱动器
 * 
 * 电机布局 (俯视图):
 *   LF ┐     ┌ RF
 *        ┴
 *   LB └     ┘ RB
 * 
 * 电机驱动 (引脚分配):
 * - L298N#1: 前+后
 *   - OUT1/2: 前进 (LF) - 引脚: PA0/PA1
 *   - OUT3/4: 后退 (LB) - 引脚: PA2/PA3
 *   - ENA: PA8 (TIM1_CH1 PWM) - 前进速度
 *   - ENB: PA10 (TIM1_CH3 PWM) - 后退速度
 * 
 * - L298N#2: 前+后
 *   - OUT1/2: 前进 (RF) - 引脚: PA4/PA5
 *   - OUT3/4: 后退 (RB) - 引脚: PA6/PA7
 *   - ENA: PA9 (TIM1_CH2 PWM) - 前进速度
 *   - ENB: PA11 (TIM1_CH4 PWM) - 后退速度
 * 
 * 编码器引脚:
 * - TIM2: 左侧编码器 -> PA15/PB3
 * - TIM3: 右侧编码器 -> PB4/PB5
 *
 * 电机控制示例:
 * - 前进: LF+ RF+ LB+ RB+
 * - 后退: LF- RF- LB- RB-
 * - 左移: LF- RF+ LB+ RB-
 * - 右移: LF+ RF- LB- RB+
 * - 原地左转: LF- RF+ LB- RB+
 * - 原地右转: LF+ RF- LB+ RB-
 */

// 前进 (LF) - L298N#1 OUT1/2 (PA0/PA1)
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

// 后退 (LB) - L298N#1 OUT3/4 (PA2/PA3)
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

// 前进 (RF) - L298N#2 OUT1/2 (PA4/PA5)
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

// 后退 (RB) - L298N#2 OUT3/4 (PA6/PA7)
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

// 编码器和路程测量参数
#define ENCODER_PPR 1560.0f      // 编码器每圈脉冲数(根据实际编码器修改)
#define WHEEL_DIAMETER 60.0f     // 车轮直径(mm)
#define WHEEL_PERIMETER (3.14159f * WHEEL_DIAMETER)  // 车轮周长(mm)
#define PULSES_PER_MM (ENCODER_PPR / WHEEL_PERIMETER) // 每毫米脉冲数

// 电机初始化函数
void Motor_Init(void);
void Mecanum_Control(int16_t lf_pwm, int16_t rf_pwm, int16_t lb_pwm, int16_t rb_pwm);  // 速度控制
void Car_Stop(void);
void Car_Forward(int16_t speed);      // 前进
void Car_Backward(int16_t speed);     // 后退
void Car_Move_Left(int16_t speed);    // 左移
void Car_Move_Right(int16_t speed);   // 右移
void Car_TurnLeft(int16_t speed);     // 原地左转
void Car_TurnRight(int16_t speed);    // 原地右转
void Car_Move_FL(int16_t speed);      // 前左
void Car_Move_FR(int16_t speed);      // 前右
void Car_Move_BL(int16_t speed);      // 后左
void Car_Move_BR(int16_t speed);      // 后右

// 摇杆映射和速度滤波函数
int16_t Map_Joystick_To_PWM(uint8_t joystick_val);
void Speed_Filter_Update(int16_t* left_filtered, int16_t* right_filtered, int16_t left_target, int16_t right_target);

// 编码器函数
int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);
void Encoder_ResetCounts(void);

// 里程测量函数
float Odometer_GetLeftDistance(void);   // 获取左侧行驶距离(mm)
float Odometer_GetRightDistance(void);  // 获取右侧行驶距离(mm)
float Odometer_GetTotalDistance(void);  // 获取总行驶距离(mm)
void Odometer_Reset(void);               // 重置里程计

// 速度测量函数
float Speed_GetLeft(void);               // 获取左侧速度(mm/s)
float Speed_GetRight(void);              // 获取右侧速度(mm/s)
void Speed_Update(void);                 // 更新速度测量(定时调用)

// PS2手柄控制函数
void PS2_Control_Car(uint8_t ljoy_lr, uint8_t ljoy_ud, uint8_t btn1);

// 电机测试函数
void Motor_Test_Individual(void);  // 逐个测试电机转向

#endif

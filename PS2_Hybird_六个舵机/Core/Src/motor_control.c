#include "motor_control.h"
#include <stdlib.h>  // for abs() function

// 电机速度和编码器变量
static int32_t left_encoder_total = 0;
static int32_t right_encoder_total = 0;
static int32_t left_encoder_last = 0;
static int32_t right_encoder_last = 0;

/**
  * @brief  初始化电机和编码器
  * @param  无
  * @retval 无
  */
void Motor_Init(void)
{
	Car_Stop();
	
	// 启动PWM TIM1 CH1?CH2??????L298N???ENA?ENB?
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	
	// 启动编码器TIM2?TIM3?
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	
	Encoder_ResetCounts();
}

/**
  * @brief  控制小车速度
  * @param  left_pwm: 左侧电机PWM值 (-MAX_PWM ~ MAX_PWM)
  * @param  right_pwm: 右侧电机PWM值 (-MAX_PWM ~ MAX_PWM)
  * @retval 无
  * @说明  L298N#1控制左前+左后电机,L298N#2控制右前+右后电机
  */
void Car_Control(int16_t left_pwm, int16_t right_pwm)
{
	// 左侧电机控制 (左前+左后)
	if(left_pwm > MIN_PWM)
	{
		LEFT_MOTORS_FORWARD();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, left_pwm);
	}
	else if(left_pwm < -MIN_PWM)
	{
		LEFT_MOTORS_BACKWARD();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -left_pwm);
	}
	else 
	{
		LEFT_MOTORS_STOP();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	}
	
	// 右侧电机控制 (右前+右后)
	if(right_pwm > MIN_PWM)
	{
		RIGHT_MOTORS_FORWARD();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, right_pwm);
	}
	else if(right_pwm < -MIN_PWM)
	{
		RIGHT_MOTORS_BACKWARD();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -right_pwm);
	}
	else 
	{
		RIGHT_MOTORS_STOP();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	}
}

/**
  * @brief  停止小车
  * @param  无
  * @retval 无
  */
void Car_Stop(void)
{
	LEFT_MOTORS_STOP();
	RIGHT_MOTORS_STOP();
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

/**
  * @brief  小车前进
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_Forward(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	Car_Control(speed, speed);
}

/**
  * @brief  小车后退
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_Backward(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	Car_Control(-speed, -speed);
}

/**
  * @brief  小车原地左转
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_TurnLeft(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	Car_Control(-speed, speed);
}

/**
  * @brief  小车原地右转
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_TurnRight(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	Car_Control(speed, -speed);
}

/**
  * @brief  小车左斜移（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  * @说明  由于硬件限制，实际为左前方斜移（左侧慢、右侧快）
  */
void Car_Move_Left(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 左斜移：左侧低速，右侧高速
	Car_Control(speed/2, speed);
}

/**
  * @brief  小车右斜移（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  * @说明  由于硬件限制，实际为右前方斜移（左侧快、右侧慢）
  */
void Car_Move_Right(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 右斜移：左侧高速，右侧低速
	Car_Control(speed, speed/2);
}

/**
  * @brief  麦克纳姆轮运动学控制（简化版，2自由度）
  * @param  vx: X轴速度(前后) -MAX_PWM ~ MAX_PWM, 正值=前进
  * @param  vy: Y轴速度(左右) -MAX_PWM ~ MAX_PWM, 正值=右移 (硬件限制，实际为斜移)
  * @param  vw: 角速度(旋转) -MAX_PWM ~ MAX_PWM, 正值=顺时针
  * @retval 无
  * @说明  由于左右两侧电机并联，只能近似实现麦轮运动
  *        标准麦轮公式：
  *        V_LF = Vx + Vy + Vw
  *        V_RF = Vx - Vy - Vw
  *        V_LB = Vx - Vy + Vw
  *        V_RB = Vx + Vy - Vw
  *        
  *        简化版（并联后）：
  *        V_Left  = (V_LF + V_LB)/2 = Vx + Vw
  *        V_Right = (V_RF + V_RB)/2 = Vx - Vw
  */
void Mecanum_Move(int16_t vx, int16_t vy, int16_t vw)
{
	int16_t left_speed, right_speed;
	
	// 简化的麦轮公式（忽略vy，因为无法真正平移）
	left_speed = vx + vw;
	right_speed = vx - vw;
	
	// 限幅保护
	if(left_speed > MAX_PWM) left_speed = MAX_PWM;
	if(left_speed < -MAX_PWM) left_speed = -MAX_PWM;
	if(right_speed > MAX_PWM) right_speed = MAX_PWM;
	if(right_speed < -MAX_PWM) right_speed = -MAX_PWM;
	
	Car_Control(left_speed, right_speed);
}

/**
  * @brief  将摇杆值映射为PWM值
  * @param  joystick_val: 摇杆值 (0~255, 中点127)
  * @retval PWM值 (-MAX_PWM ~ MAX_PWM)
  */
int16_t Map_Joystick_To_PWM(uint8_t joystick_val)
{
	int16_t pwm_val;
	
	// 摇杆值 > 127 + 死区
	if(joystick_val > (127 + DEAD_ZONE))
	{
		pwm_val = (joystick_val - 127) * MAX_PWM / 128;
		return (pwm_val > MAX_PWM) ? MAX_PWM : pwm_val;
	}
	// 摇杆值 < 127 - 死区
	else if(joystick_val < (127 - DEAD_ZONE))
	{
		pwm_val = (127 - joystick_val) * MAX_PWM / 128;
		return (pwm_val > MAX_PWM) ? -MAX_PWM : -pwm_val;
	}
	// 死区
	else 
	{
		return 0;
	}
}

/**
  * @brief  速度滤波器更新
  * @param  left_filtered: 左侧滤波后速度
  * @param  right_filtered: 右侧滤波后速度
  * @param  left_target: 左侧目标速度
  * @param  right_target: 右侧目标速度
  * @retval 无
  */
void Speed_Filter_Update(int16_t* left_filtered, int16_t* right_filtered, int16_t left_target, int16_t right_target)
{
	// 一阶滤波
	*left_filtered = (int16_t)(0.3 * left_target + 0.7 * (*left_filtered));
	*right_filtered = (int16_t)(0.3 * right_target + 0.7 * (*right_filtered));
}

/**
  * @brief  获取左侧编码器计数值
  * @param  无
  * @retval 编码器计数值
  */
int32_t Encoder_GetLeftCount(void)
{
	int16_t current_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
	int16_t delta = current_count - left_encoder_last;
	
	if(delta > 32767) delta -= 65536;
	else if(delta < -32767) delta += 65536;
	
	left_encoder_total += delta;
	left_encoder_last = current_count;
	
	return left_encoder_total;
}

/**
  * @brief  获取右侧编码器计数值
  * @param  无
  * @retval 编码器计数值
  */
int32_t Encoder_GetRightCount(void)
{
	int16_t current_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	int16_t delta = current_count - right_encoder_last;
	
	if(delta > 32767) delta -= 65536;
	else if(delta < -32767) delta += 65536;
	
	right_encoder_total += delta;
	right_encoder_last = current_count;
	
	return right_encoder_total;
}

/**
  * @brief  重置编码器计数值
  * @param  无
  * @retval 无
  */
void Encoder_ResetCounts(void)
{
	left_encoder_total = 0;
	right_encoder_total = 0;
	left_encoder_last = __HAL_TIM_GET_COUNTER(&htim2);
	right_encoder_last = __HAL_TIM_GET_COUNTER(&htim3);
}

/**
  * @brief  PS2手柄控制小车（麦克纳姆轮版本）
  * @param  ljoy_lr: 左摇杆左右值 (0~255)
  * @param  ljoy_ud: 左摇杆上下值 (0~255)
  * @param  btn1: 按键1状态（Bit1=JOYR, Bit2=JOYL）
  * @retval 无
  * @说明  控制逻辑：
  *        1. 优先级最高：按键原地旋转
  *           - 按住JOYL（Bit2=1）：原地左转
  *           - 按住JOYR（Bit1=1）：原地右转
  *        2. 其次：左摇杆控制移动
  *           - 前后推：前进/后退
  *           - 左右推：左斜移/右斜移（由于硬件限制）
  */
void PS2_Control_Car(uint8_t ljoy_lr, uint8_t ljoy_ud, uint8_t btn1)
{
	// 定义按键位：Bit1=JOYR, Bit2=JOYL
	#define BTN_JOYR  (1<<1)  // Bit1
	#define BTN_JOYL  (1<<2)  // Bit2
	
	int16_t vx = 0;  // 前后速度
	int16_t vy = 0;  // 左右速度（当前硬件无法实现纯平移）
	int16_t vw = 0;  // 旋转速度
	
	// 优先级最高：检查按键原地旋转
	if(btn1 & BTN_JOYL)  // 按下JOYL：原地左转
	{
		Car_TurnLeft(400);  // 使用固定速度旋转
		return;
	}
	else if(btn1 & BTN_JOYR)  // 按下JOYR：原地右转
	{
		Car_TurnRight(400);
		return;
	}
	
	// 将摇杆值转换为PWM值
	// ljoy_ud: 0=上(前进), 127=中点, 255=下(后退)
	vx = -Map_Joystick_To_PWM(ljoy_ud);  // 反转，因为上是0，下是255
	
	// ljoy_lr: 0=左, 127=中点, 255=右
	vy = Map_Joystick_To_PWM(ljoy_lr);
	
	// 判断主要移动方向
	if(vx == 0 && vy == 0)
	{
		// 摇杆在中点，停止
		Car_Stop();
	}
	else if(abs(vx) > abs(vy))
	{
		// 前后移动为主
		Mecanum_Move(vx, 0, 0);
	}
	else
	{
		// 左右移动为主（由于硬件限制，实际为斜移）
		if(vy > 0)  // 右斜移
		{
			Car_Move_Right(vy);
		}
		else  // 左斜移
		{
			Car_Move_Left(-vy);
		}
	}
}

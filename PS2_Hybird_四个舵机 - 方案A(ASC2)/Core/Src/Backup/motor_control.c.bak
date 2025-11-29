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
	
	// 启动PWM TIM1的4个通道，分别控制四个麦克纳姆轮
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // PA8 - 左前轮
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // PA9 - 右前轮
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // PA10 - 左后轮
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // PA11 - 右后轮
	
	// 启动编码器TIM2和TIM3
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	
	Encoder_ResetCounts();
}

/**
  * @brief  麦克纳姆轮四轮独立控制
  * @param  lf_pwm: 左前轮PWM值 (-MAX_PWM ~ MAX_PWM)
  * @param  rf_pwm: 右前轮PWM值 (-MAX_PWM ~ MAX_PWM)
  * @param  lb_pwm: 左后轮PWM值 (-MAX_PWM ~ MAX_PWM)
  * @param  rb_pwm: 右后轮PWM值 (-MAX_PWM ~ MAX_PWM)
  * @retval 无
  */
void Mecanum_Control(int16_t lf_pwm, int16_t rf_pwm, int16_t lb_pwm, int16_t rb_pwm)
{
	// 左前轮控制
	if(lf_pwm > MIN_PWM)
	{
		LF_FORWARD();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, lf_pwm);
	}
	else if(lf_pwm < -MIN_PWM)
	{
		LF_BACKWARD();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -lf_pwm);
	}
	else 
	{
		LF_STOP();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	}
	
	// 右前轮控制
	if(rf_pwm > MIN_PWM)
	{
		RF_FORWARD();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, rf_pwm);
	}
	else if(rf_pwm < -MIN_PWM)
	{
		RF_BACKWARD();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -rf_pwm);
	}
	else 
	{
		RF_STOP();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	}
	
	// 左后轮控制
	if(lb_pwm > MIN_PWM)
	{
		LB_FORWARD();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, lb_pwm);
	}
	else if(lb_pwm < -MIN_PWM)
	{
		LB_BACKWARD();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -lb_pwm);
	}
	else 
	{
		LB_STOP();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	}
	
	// 右后轮控制
	if(rb_pwm > MIN_PWM)
	{
		RB_FORWARD();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, rb_pwm);
	}
	else if(rb_pwm < -MIN_PWM)
	{
		RB_BACKWARD();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -rb_pwm);
	}
	else 
	{
		RB_STOP();
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	}
}

/**
  * @brief  停止小车
  * @param  无
  * @retval 无
  */
void Car_Stop(void)
{
	LF_STOP();
	RF_STOP();
	LB_STOP();
	RB_STOP();
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}

/**
  * @brief  小车前进（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_Forward(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 前进: LF+ RF+ LB+ RB+
	Mecanum_Control(speed, speed, speed, speed);
}

/**
  * @brief  小车后退（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_Backward(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 后退: LF- RF- LB- RB-
	Mecanum_Control(-speed, -speed, -speed, -speed);
}

/**
  * @brief  小车原地左转（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_TurnLeft(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 原地左转: LF- RF+ LB- RB+
	Mecanum_Control(-speed, speed, -speed, speed);
}

/**
  * @brief  小车原地右转（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_TurnRight(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 原地右转: LF+ RF- LB+ RB-
	Mecanum_Control(speed, -speed, speed, -speed);
}

/**
  * @brief  小车左平移（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_Move_Left(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 左平移: LF- RF+ LB+ RB-
	Mecanum_Control(-speed, speed, speed, -speed);
}

/**
  * @brief  小车右平移（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_Move_Right(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 右平移: LF+ RF- LB- RB+
	Mecanum_Control(speed, -speed, -speed, speed);
}

/**
  * @brief  小车左前斜移（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_Move_FL(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 左前斜移: LF0 RF+ LB+ RB0
	Mecanum_Control(0, speed, speed, 0);
}

/**
  * @brief  小车右前斜移（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_Move_FR(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 右前斜移: LF+ RF0 LB0 RB+
	Mecanum_Control(speed, 0, 0, speed);
}

/**
  * @brief  小车左后斜移（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_Move_BL(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 左后斜移: LF- RF0 LB0 RB-
	Mecanum_Control(-speed, 0, 0, -speed);
}

/**
  * @brief  小车右后斜移（麦克纳姆轮）
  * @param  speed: 速度 (MIN_PWM ~ MAX_PWM)
  * @retval 无
  */
void Car_Move_BR(int16_t speed)
{
	if(speed > MAX_PWM) speed = MAX_PWM;
	if(speed < MIN_PWM) speed = MIN_PWM;
	// 右后斜移: LF0 RF- LB- RB0
	Mecanum_Control(0, -speed, -speed, 0);
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
  * @brief  PS2手柄控制麦克纳姆轮小车（支持全向移动和按键旋转）
  * @param  ljoy_lr: 左摇杆左右值 (0~255)
  * @param  ljoy_ud: 左摇杆上下值 (0~255)
  * @param  btn1: 按键1状态（Bit1=JOYR, Bit2=JOYL）
  * @retval 无
  * @说明  控制逻辑：
  *        1. 优先级最高：按键原地旋转
  *           - 按住JOYL（Bit2=1）：原地左转
  *           - 按住JOYR（Bit1=1）：原地右转
  *        2. 其次：左摇杆控制全向移动
  *           - 前后推：前进/后退
  *           - 左右推：左平移/右平移
  *           - 斜向推：斜向移动（麦克纳姆轮特性）
  */
void PS2_Control_Car(uint8_t ljoy_lr, uint8_t ljoy_ud, uint8_t btn1)
{
	// 定义按键位：Bit1=JOYR, Bit2=JOYL
	#define BTN_JOYR  (1<<1)  // Bit1
	#define BTN_JOYL  (1<<2)  // Bit2
	
	int16_t forward_speed = 0;  // 前后速度
	int16_t strafe_speed = 0;   // 平移速度
	int16_t lf_pwm = 0, rf_pwm = 0, lb_pwm = 0, rb_pwm = 0;
	
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
	forward_speed = -Map_Joystick_To_PWM(ljoy_ud);  // 反转，因为上是0，下是255
	
	// ljoy_lr: 0=左, 127=中点, 255=右
	strafe_speed = Map_Joystick_To_PWM(ljoy_lr);
	
	// 麦克纳姆轮运动学公式：
	// LF = forward - strafe
	// RF = forward + strafe
	// LB = forward + strafe
	// RB = forward - strafe
	
	lf_pwm = forward_speed - strafe_speed;
	rf_pwm = forward_speed + strafe_speed;
	lb_pwm = forward_speed + strafe_speed;
	rb_pwm = forward_speed - strafe_speed;
	
	// PWM限幅
	if(lf_pwm > MAX_PWM) lf_pwm = MAX_PWM;
	if(lf_pwm < -MAX_PWM) lf_pwm = -MAX_PWM;
	if(rf_pwm > MAX_PWM) rf_pwm = MAX_PWM;
	if(rf_pwm < -MAX_PWM) rf_pwm = -MAX_PWM;
	if(lb_pwm > MAX_PWM) lb_pwm = MAX_PWM;
	if(lb_pwm < -MAX_PWM) lb_pwm = -MAX_PWM;
	if(rb_pwm > MAX_PWM) rb_pwm = MAX_PWM;
	if(rb_pwm < -MAX_PWM) rb_pwm = -MAX_PWM;
	
	// 控制四个轮子
	Mecanum_Control(lf_pwm, rf_pwm, lb_pwm, rb_pwm);
}

#include "motor_control.h"

static float NormalizeAngle(float angle);
static float CalculateAngleDifference(float target, float current);

void MotorControl_Init(MotorControl_t *motor, uint8_t motor_id)
{
    motor->motor_id = motor_id;

    PID_Init(&motor->angle_pid, 10.0, 0.05, 0.2, 2000.0f, -2000.0f);

    PID_Init(&motor->speed_pid, 5.0, 0.1, 0.05, 16384.0f, -16384.0f);
    
    motor->feedback.rotor_angle = 0;
    motor->feedback.rotor_speed = 0;
    motor->feedback.torque_current = 0;
    motor->feedback.temperature = 0;
    motor->feedback.last_update_time = 0;
    
    motor->target_output_angle = 0.0f;
    motor->current_output_angle = 0.0f;
    motor->target_rotor_speed = 0.0f;
    motor->current_rotor_speed = 0.0f;
    
    motor->total_output_angle = 0.0f;
    motor->last_rotor_angle = 0.0f;
    motor->rotation_count = 0;
    
    motor->control_enable = 0;
}

void MotorControl_UpdateFeedback(MotorControl_t *motor, uint16_t rotor_angle, 
                                  int16_t rotor_speed, int16_t torque_current, 
                                  int8_t temperature)
{
    float rotor_angle_deg;
    float angle_diff;
    float output_angle_deg;
    
    motor->feedback.rotor_angle = rotor_angle;
    motor->feedback.rotor_speed = rotor_speed;
    motor->feedback.torque_current = torque_current;
    motor->feedback.temperature = temperature;
    
    rotor_angle_deg = (float)rotor_angle * 360.0f / ROTOR_ANGLE_MAX;
    
    angle_diff = rotor_angle_deg - motor->last_rotor_angle;

    if (angle_diff > 180.0f)
    {
        motor->rotation_count--;
    } 
    else if (angle_diff < -180.0f)
    {
        motor->rotation_count++;
    }
    
    float total_rotor_angle_deg = rotor_angle_deg + motor->rotation_count * 360.0f;
    
    // 计算累计输出轴角度
    // 转子角度转输出轴角度：输出轴角度 = 转子角度 / 减速比
    output_angle_deg = total_rotor_angle_deg / GEAR_RATIO;
    motor->total_output_angle = output_angle_deg;
    
    // 计算当前输出轴角度（0-360度归一化）
    motor->current_output_angle = NormalizeAngle(output_angle_deg);
    
    // 更新当前转子速度
    motor->current_rotor_speed = (float)rotor_speed;
    
    // 保存当前转子角度
    motor->last_rotor_angle = rotor_angle_deg;
}


void MotorControl_SetTargetAngle(MotorControl_t *motor, float target_angle_deg)
{
    // 如果输入的是相对角度（0-360），转换为累计角度
    // 这里假设输入是相对于当前位置的角度增量
    // 如果是绝对角度，直接使用累计角度
    motor->target_output_angle = motor->total_output_angle + target_angle_deg;
}

int16_t MotorControl_Calculate(MotorControl_t *motor)
{
    float angle_error;
    float speed_target;
    float current_output;
    
    if (motor->control_enable == 0) 
    {
        return 0;
    }
    
    // 外环：角度环PID计算
    // 目标角度已经是累计角度，直接计算误差
    angle_error = motor->target_output_angle - motor->total_output_angle;
    
    // 角度环PID计算：输入角度误差，输出速度目标（rpm）
    // PID计算：target设为0，current设为-angle_error
    // 这样error = 0 - (-angle_error) = angle_error
    // 输出速度与角度误差成正比，误差为正时输出正速度
    motor->angle_pid.target = 0.0f;
    speed_target = PID_Calculate(&motor->angle_pid, -angle_error);
    
    // 速度目标限制（转子速度，单位rpm）
    if (speed_target > 2000.0f) 
    {
        speed_target = 2000.0f;
    }
    else if (speed_target < -2000.0f)
    {
        speed_target = -2000.0f;
    }
    
    motor->target_rotor_speed = speed_target;
    
    // 内环：速度环PID计算（维持设定速度）
    // 速度环输入是当前速度，输出是电流值
    PID_SetTarget(&motor->speed_pid, motor->target_rotor_speed);
    current_output = PID_Calculate(&motor->speed_pid, motor->current_rotor_speed);
    
    // 输出限幅（电流值范围，根据实际电机调整）
    if (current_output > 16384.0f) 
    {
        current_output = 16384.0f;
    } 
    else if (current_output < -16384.0f) 
    {
        current_output = -16384.0f;
    }
    
    return (int16_t)current_output;
}

void MotorControl_Reset(MotorControl_t *motor)
{
    PID_Reset(&motor->angle_pid);
    PID_Reset(&motor->speed_pid);
    motor->target_output_angle = 0.0f;
    motor->target_rotor_speed = 0.0f;
    motor->rotation_count = 0;
    motor->total_output_angle = 0.0f;
    motor->last_rotor_angle = (float)motor->feedback.rotor_angle * 360.0f / ROTOR_ANGLE_MAX;
}

/**
  * @brief  设置控制使能
  * @param  motor: 电机控制结构体指针
  * @param  enable: 使能标志（1-使能，0-禁用）
  * @retval None
  */
void MotorControl_SetEnable(MotorControl_t *motor, uint32_t enable)
{
    motor->control_enable = enable;

    if (enable == 0) 
    {
        MotorControl_Reset(motor);
    }
}

float MotorControl_GetCurrentAngle(MotorControl_t *motor)
{
    return motor->current_output_angle;
}

/**
  * @brief  根据输出轴角度计算需要的转子角度
  * @param  output_angle_deg: 输出轴角度（度）
  * @retval 转子角度（度）
  */
float MotorControl_CalculateRotorAngleFromOutput(float output_angle_deg)
{
    return output_angle_deg * GEAR_RATIO;
}

void MotorControl_RotateTwoRoundsForward(MotorControl_t *motor)
{
    // 目标角度 = 当前累计角度 + 720度（两圈）
    motor->target_output_angle = motor->total_output_angle + OUTPUT_SHAFT_TWO_ROTATION;
    // 使能控制
    motor->control_enable = 1;
}

void MotorControl_RotateTwoRoundsReverse(MotorControl_t *motor)
{
    // 目标角度 = 当前累计角度 - 720度（两圈）
    motor->target_output_angle = motor->total_output_angle - OUTPUT_SHAFT_TWO_ROTATION;
    // 使能控制
    motor->control_enable = 1;
}

/**
  * @brief  角度归一化到0-360度
  * @param  angle: 输入角度（度）
  * @retval 归一化后的角度（0-360度）
  */
static float NormalizeAngle(float angle)
{
    while (angle < 0.0f) 
    {
        angle += 360.0f;
    }
    while (angle >= 360.0f) 
    {
        angle -= 360.0f;
    }
    return angle;
}

static float CalculateAngleDifference(float target, float current)
{
    float diff = target - current;
    
    // 将角度差归一化到-180到180度
    while (diff > 180.0f) 
    {
        diff -= 360.0f;
    }
    while (diff < -180.0f) 
    {
        diff += 360.0f;
    }
    
    return diff;
}

/* USER CODE END 1 */


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "usart.h"
#include <stdio.h>
/* USER CODE BEGIN 0 */

motor_feedback_t motor_feedback[4] = {{0}};

float motor_continuous_angle = 0.0f;
uint16_t motor_last_angle_raw = 0;
uint8_t motor_angle_inited = 0;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

    CAN_FilterTypeDef can_filter;                                   //过滤器初始化
    can_filter.FilterBank           = 0;
    can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh         = 0x0000;
    can_filter.FilterIdLow          = 0x0000;
    can_filter.FilterMaskIdHigh     = 0x0000;
    can_filter.FilterMaskIdLow      = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation     = ENABLE;
    can_filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK) 
    {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan) != HAL_OK) 
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void can_dji_motor_control(uint8_t motor_id, int16_t current)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;

    tx_header.StdId = 0x200;
    tx_header.ExtId = 0;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 8;

    for (int i = 0; i < 8; i++) 
    {
        tx_data[i] = 0;
    }

    switch (motor_id) 
    {
        case 1:
            tx_data[0] = (current >> 8) & 0xFF;
            tx_data[1] = current & 0xFF;
            break;
        case 2:
            tx_data[2] = (current >> 8) & 0xFF;
            tx_data[3] = current & 0xFF;
            break;
        case 3:
            tx_data[4] = (current >> 8) & 0xFF;
            tx_data[5] = current & 0xFF;
            break;
        case 4:
            tx_data[6] = (current >> 8) & 0xFF;
            tx_data[7] = current & 0xFF;
            break;
        default:
            return;
    }

    if (HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox) != HAL_OK) 
    {
        Error_Handler();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rx_data) == HAL_OK) 
    {
        can_feedcallback(&rxHeader, rx_data);
    }
}

void can_feedcallback(CAN_RxHeaderTypeDef *rx_header, uint8_t rx_data[])
{
    if (rx_header->StdId >= 0x201 && rx_header->StdId <= (0x201 + 3)) 
    {
        uint8_t id = (uint8_t)(rx_header->StdId - 0x201);

        if (id < 4)
        {
            uint16_t angle_u  = ((uint16_t)rx_data[0] << 8) | (uint16_t)rx_data[1];      // 角度范围为 0 ~ 8191 (13 位) -- 0 ~ 360 度
            int16_t speed_i   = ((uint16_t)rx_data[2] << 8) | (uint16_t)rx_data[3];      // 速度，单位 rpm
            int16_t current_i = ((uint16_t)rx_data[4] << 8) | (uint16_t)rx_data[5];      // 电流，单位 mA
            uint8_t temp_u8   = rx_data[6];

            float raw_angle_deg = ((float)angle_u) * 360.0f / 8192.0f;

            //跳变检测
            if (!motor_angle_inited)
            {
                motor_continuous_angle = raw_angle_deg;
                motor_last_angle_raw = angle_u;
                motor_angle_inited = 1;
            }
            else
            {
                int32_t diff = (int32_t)angle_u - (int32_t)motor_last_angle_raw;

                if (diff > 4096) 
                {
                    diff -= 8192;
                }
                else if (diff < -4096) 
                {
                    diff += 8192;
                }

                float diff_deg = ((float)diff) * 360.0f / 8192.0f;
                motor_continuous_angle += diff_deg;
                motor_last_angle_raw = angle_u;
            }
            
            motor_feedback[id].angle   = (int16_t)(raw_angle_deg + 0.5f);
            motor_feedback[id].speed   = speed_i;
            motor_feedback[id].current = current_i;
            motor_feedback[id].temp    = (int16_t)temp_u8;
            
            uint8_t uart_buffer[128];
            int len = snprintf((char*)uart_buffer, sizeof(uart_buffer), 
                               "M%d:%d,%d,%d,%d\r\n", 
                               id + 1, 
                               motor_feedback[id].angle, 
                               motor_feedback[id].speed,
                               motor_feedback[id].current,
                               motor_feedback[id].temp);

            if (len > 0 && len < (int)sizeof(uart_buffer)) 
            {
                HAL_UART_Transmit_IT(&huart1, uart_buffer, (uint16_t)len);
            }
        }
    }
}

void motor_rotate_by(uint8_t motor_id, float delta_angle, float speed_limit)
{
    if (motor_id < 1 || motor_id > 4)
    {
        return;
    }

    if (!motor_angle_inited)
    {
        return;
    }

    float current_continuous = motor_continuous_angle / 19.2037f;  // 输出轮当前角度
    float target_continuous = current_continuous + delta_angle;    // 输出轮目标角度
    float angle_diff = target_continuous - current_continuous;     // 输出轮角度差值

    float target_speed = 0.0f;
    if (angle_diff > 0.0f) 
    {
        target_speed = speed_limit;
    } else if (angle_diff < 0.0f) 
    {
        target_speed = -speed_limit;
    }

    // 发送命令（使用 can_dji_motor_control，参数为电流，当前用速度映射到电流）
    // target_speed 是输出轮的速度，需要转换为转子速度
    can_dji_motor_control(motor_id, (int16_t)(target_speed * 19.2037f));
}

void pid_init(pid_controller_t *pid, float kp, float ki, float kd)
{
    pid->kp         = kp;
    pid->ki         = ki;
    pid->kd         = kd;
    pid->target     = 0.0f;
    pid->current    = 0.0f;
    pid->error      = 0.0f;
    pid->last_error = 0.0f;
    pid->integral   = 0.0f;
    pid->output     = 0.0f;
}

float pid_calculate(pid_controller_t *pid, float target, float current, float dt)
{
    pid->target  = target;
    pid->current = current;
    pid->error   = target - current;

    if (dt < 1e-4f) dt =1e-4f; // 防止除以零或过小的 dt 导致微分项过大

    // 积分项（增量积分），并暂存以便在发生饱和时回退
    float integral_before = pid->integral;
    pid->integral += pid->error * dt;

    const float INTEGRAL_LIMIT = 2000.0f;
    if (pid->integral > INTEGRAL_LIMIT) pid->integral = INTEGRAL_LIMIT;
    else if (pid->integral < -INTEGRAL_LIMIT) pid->integral = -INTEGRAL_LIMIT;

    float derivative = (pid->error - pid->last_error) / dt;

    float out = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * derivative;

    // 输出限幅
    const float OUT_LIMIT = 16384.0f;
    if (out > OUT_LIMIT) 
    {
        pid->integral = integral_before;
        out = OUT_LIMIT;
    } 
    else if (out < -OUT_LIMIT) 
    {
        pid->integral = integral_before;
        out = -OUT_LIMIT;
    }

    pid->output = out;
    pid->last_error = pid->error;

    return pid->output;
}

void cascade_pid_init(cascade_pid_controller_t *cascade_pid, float angle_kp, float angle_ki, float angle_kd, float speed_kp, float speed_ki, float speed_kd)
{
    // 初始化角度环
    pid_init(&cascade_pid->angle_pid, angle_kp, angle_ki, angle_kd);
    
    // 初始化速度环
    pid_init(&cascade_pid->speed_pid, speed_kp, speed_ki, speed_kd);
    
    cascade_pid->target_angle = 0.0f;
    cascade_pid->target_speed = 0.0f;
    cascade_pid->current_angle = 0.0f;
    cascade_pid->current_speed = 0.0f;
}

float cascade_pid_calculate(cascade_pid_controller_t *cascade_pid, float target_angle, float current_angle, float current_speed, float dt)
{
    // target_angle 是转子角度，current_angle 是输出轮角度
    cascade_pid->target_angle = target_angle;
    cascade_pid->current_angle = current_angle;
    cascade_pid->current_speed = current_speed;
    
    // 角度环计算，输出作为速度环的目标值
    // 需要将转子目标角度转换为输出轮角度进行比较
    cascade_pid->target_speed = pid_calculate(&cascade_pid->angle_pid, target_angle / 19.2037f, current_angle, dt);
    
    // 速度环计算，输出作为最终的控制量
    // 需要将输出轮速度目标值转换为转子速度进行控制
    float output_current = pid_calculate(&cascade_pid->speed_pid, cascade_pid->target_speed * 19.2037f, current_speed * 19.2037f, dt);
    
    return output_current;
}

//
void motor_rotate_to_angle(uint8_t motor_id, float target_angle, float speed_limit)
{
    // 确保电机ID在有效范围内
    if (motor_id < 1 || motor_id > 4) 
    {
        return;
    }
    
    // 限制目标角度在-720到720度范围内（输出轮角度）
    if (target_angle > 720.0f) 
    {
        target_angle = 720.0f;
    }
    else if (target_angle < -720.0f) 
    {
        target_angle = -720.0f;
    }
    
    // 使用连续角度进行计算
    if (!motor_angle_inited) 
    {
        // 如果角度未初始化，直接返回
        return;
    }
    

    float current_angle = motor_continuous_angle / 19.2037f;  // 输出轮当前角度
    
    // 计算输出轮角度差值
    float angle_diff = target_angle - current_angle;
    
    // 角度差值超过180度时，选择反向旋转路径以减少旋转角度
    if (angle_diff > 180.0f) 
    {
        angle_diff -= 360.0f;
    }
    else if (angle_diff < -180.0f) 
    {
        angle_diff += 360.0f;
    }
    
    // 根据角度差值确定旋转方向和速度，使用更平滑的控制策略
    float target_speed = 0.0f;
    if (angle_diff > 5.0f) 
    {  // 正向旋转
        // 使用比例控制，角度差值越大速度越快
        target_speed = (angle_diff > speed_limit) ? speed_limit : angle_diff;
    }
    else if (angle_diff < -5.0f) 
    {  // 反向旋转
        // 使用比例控制，角度差值越大速度越快
        target_speed = (angle_diff < -speed_limit) ? -speed_limit : angle_diff;
    }
    else
    {
        // 角度差值较小时，缓慢调整
        target_speed = angle_diff * 0.5f;  // 比例系数可调
    }
    
    // target_speed 是输出轮的速度，需要转换为转子速度
    can_dji_motor_control(motor_id, (int16_t)(target_speed * 19.2037f));
}

/* USER CODE END 1 */

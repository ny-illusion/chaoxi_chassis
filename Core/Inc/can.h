/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file can.h
  * @brief CAN1 外设、引脚与中断配置的声明
  * @board 底盘板
  * @vehicle_function 连接板间遥控链路和电机反馈总线
  * @inputs APB1 时钟、CAN1 初始化参数、PD0/PD1
  * @outputs 板级外设句柄与初始化函数声明
  * @dependencies STM32F4 HAL CAN、RCC、GPIO、NVIC
  * @note 重构说明：本文件行为与原代码保持一致。main 调用 MX_CAN1_Init，应用层另行配置接收过滤器。
  *
  * This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __CAN_H__
#define __CAN_H__
#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
    extern CAN_HandleTypeDef hcan1;
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */
    void MX_CAN1_Init(void);  // CubeMX 生成，配置 CAN1 为 1 Mbit/s 标准工作模式
/* USER CODE BEGIN Prototypes */
/* USER CODE END Prototypes */
#ifdef __cplusplus
}
#endif
#endif

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file tim.h
  * @brief TIM6 基本定时器和中断配置的声明
  * @board 底盘板
  * @vehicle_function 提供底盘控制时基
  * @inputs 定时器时钟、预分频和自动重载配置
  * @outputs 板级外设句柄与初始化函数声明
  * @dependencies STM32F4 HAL TIM、RCC 和 NVIC
  * @note 重构说明：本文件行为与原代码保持一致。底盘初始化完成后启动周期中断。
  *
  * This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__
#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
    extern TIM_HandleTypeDef htim6;
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */
    void MX_TIM6_Init(void);  // CubeMX 生成，配置 TIM6 为 1 ms 更新周期
/* USER CODE BEGIN Prototypes */
/* USER CODE END Prototypes */
#ifdef __cplusplus
}
#endif
#endif

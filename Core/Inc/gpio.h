/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file gpio.h
  * @brief GPIO 输出与端口时钟配置的声明
  * @board 底盘板
  * @vehicle_function 控制指示灯并提供传感器的片选引脚
  * @inputs 引脚初始电平与 GPIO 端口时钟
  * @outputs MX_GPIO_Init 初始化函数声明
  * @dependencies STM32F4 HAL GPIO 和 RCC
  * @note 重构说明：本文件行为与原代码保持一致。main 在其他板级外设之前调用 MX_GPIO_Init。
  *
  * This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__
#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */
    void MX_GPIO_Init(void);  // CubeMX 生成，配置板上使用的 GPIO 引脚
/* USER CODE BEGIN Prototypes */
/* USER CODE END Prototypes */
#ifdef __cplusplus
}
#endif
#endif

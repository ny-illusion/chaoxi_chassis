/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file spi.h
  * @brief SPI1 主机与复用引脚配置的声明
  * @board 底盘板
  * @vehicle_function 为底盘陀螺仪提供 SPI 寄存器读通路
  * @inputs SPI1 初始化参数、SPI 引脚和电源时钟
  * @outputs 板级外设句柄与初始化函数声明
  * @dependencies STM32F4 HAL SPI、GPIO 和 RCC
  * @note 重构说明：本文件行为与原代码保持一致。main 在传感器探测前调用 MX_SPI1_Init。
  *
  * This file contains all the function prototypes for
  *          the spi.c file
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
#ifndef __SPI_H__
#define __SPI_H__
#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
    extern SPI_HandleTypeDef hspi1;
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */
    void MX_SPI1_Init(void);  // CubeMX 生成，配置 BMI088 使用的 SPI1 主机
/* USER CODE BEGIN Prototypes */
/* USER CODE END Prototypes */
#ifdef __cplusplus
}
#endif
#endif

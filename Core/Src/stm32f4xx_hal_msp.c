/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file stm32f4xx_hal_msp.c
  * @brief HAL 全局底层初始化
  * @board 底盘板
  * @vehicle_function 为板级外设提供基础系统时钟
  * @inputs HAL_Init 调用和芯片复位状态
  * @outputs SYSCFG 与 PWR 时钟配置
  * @dependencies STM32F4 HAL、RCC
  * @note 重构说明：本文件行为与原代码保持一致。HAL_Init 调用 HAL_MspInit，各外设的 MSP 配置位于对应驱动文件。
  *
  * This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
#include "main.h"
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
/* USER CODE BEGIN TD */
/* USER CODE END TD */
/* USER CODE BEGIN Define */
/* USER CODE END Define */
/* USER CODE BEGIN Macro */
/* USER CODE END Macro */
/* USER CODE BEGIN PV */
/* USER CODE END PV */
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* USER CODE BEGIN ExternalFunctions */
/* USER CODE END ExternalFunctions */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

void HAL_MspInit(void)  // HAL 硬件初始化钩子，CubeMX 生成实现
{
  /* USER CODE BEGIN MspInit 0 */
  /* USER CODE END MspInit 0 */
    __HAL_RCC_SYSCFG_CLK_ENABLE();  // HAL 宏，打开SYSCFG 时钟
    __HAL_RCC_PWR_CLK_ENABLE();  // HAL 宏，打开PWR 时钟
  /* USER CODE BEGIN MspInit 1 */
  /* USER CODE END MspInit 1 */
}
/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

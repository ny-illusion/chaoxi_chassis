/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file main.h
  * @brief 底盘 HAL 入口与故障处理声明
  * @board 底盘板
  * @vehicle_function 向本板各外设模块提供一致的 HAL 类型和错误入口
  * @inputs 编译器目标宏及 STM32F4 HAL 配置
  * @outputs HAL 类型和 Error_Handler 声明
  * @dependencies stm32f4xx_hal.h
  * @note 重构说明：本文件行为与原代码保持一致。由 CubeMX 外设头与底盘应用模块间接使用。
  *
  * Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
/* USER CODE BEGIN ET */
/* USER CODE END ET */
/* USER CODE BEGIN EC */
/* USER CODE END EC */
/* USER CODE BEGIN EM */
/* USER CODE END EM */
    void Error_Handler(void);  // CubeMX 生成的错误处理入口
/* USER CODE BEGIN EFP */
/* USER CODE END EFP */
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */
#ifdef __cplusplus
}
#endif
#endif

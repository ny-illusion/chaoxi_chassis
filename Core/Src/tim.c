/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file tim.c
  * @brief TIM6 基本定时器和中断配置
  * @board 底盘板
  * @vehicle_function 提供底盘控制时基
  * @inputs 定时器时钟、预分频和自动重载配置
  * @outputs htim6 及 TIM6_DAC 中断配置
  * @dependencies STM32F4 HAL TIM、RCC 和 NVIC
  * @note 重构说明：本文件行为与原代码保持一致。底盘初始化完成后启动周期中断。
  *
  * This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
TIM_HandleTypeDef htim6;

void MX_TIM6_Init(void)  // CubeMX 生成，配置 TIM6 为 1 ms 更新周期
{
  /* USER CODE BEGIN TIM6_Init 0 */
  /* USER CODE END TIM6_Init 0 */
    TIM_MasterConfigTypeDef sMasterConfig = {0};
  /* USER CODE BEGIN TIM6_Init 1 */
  /* USER CODE END TIM6_Init 1 */
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 8399;  // 84 MHz 定时器时钟先除以 8400
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 9;  // 计满 10 个计数产生一次更新，即 1 ms
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)  // HAL 库，装载 TIM6 的计数配置
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)  // HAL 库，设置定时器主模式
    {
        Error_Handler();
    }
  /* USER CODE BEGIN TIM6_Init 2 */
  /* USER CODE END TIM6_Init 2 */
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle)  // HAL 硬件初始化钩子，CubeMX 生成实现
{
    if (tim_baseHandle->Instance == TIM6)
    {
  /* USER CODE BEGIN TIM6_MspInit 0 */
  /* USER CODE END TIM6_MspInit 0 */
        __HAL_RCC_TIM6_CLK_ENABLE();  // HAL 宏，打开TIM6 时钟
        HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);  // HAL 库，设置中断抢占和子优先级
        HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);  // HAL 库，允许此中断进入 NVIC
  /* USER CODE BEGIN TIM6_MspInit 1 */
  /* USER CODE END TIM6_MspInit 1 */
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_baseHandle)  // HAL 硬件反初始化钩子，CubeMX 生成实现
{
    if (tim_baseHandle->Instance == TIM6)
    {
  /* USER CODE BEGIN TIM6_MspDeInit 0 */
  /* USER CODE END TIM6_MspDeInit 0 */
        __HAL_RCC_TIM6_CLK_DISABLE();  // HAL 宏，关闭TIM6 时钟
        HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);  // HAL 库，屏蔽此中断
  /* USER CODE BEGIN TIM6_MspDeInit 1 */
  /* USER CODE END TIM6_MspDeInit 1 */
    }
}
/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

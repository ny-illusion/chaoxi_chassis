/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file gpio.c
  * @brief GPIO 输出与端口时钟配置
  * @board 底盘板
  * @vehicle_function 控制指示灯并提供传感器的片选引脚
  * @inputs 引脚初始电平与 GPIO 端口时钟
  * @outputs GPIO 模式、输出电平和端口时钟
  * @dependencies STM32F4 HAL GPIO 和 RCC
  * @note 重构说明：本文件行为与原代码保持一致。main 在其他板级外设之前调用 MX_GPIO_Init。
  *
  * This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

void MX_GPIO_Init(void)  // CubeMX 生成，配置板上使用的 GPIO 引脚
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();  // HAL 宏，打开 GPIOB 时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();  // HAL 宏，打开 GPIOA 时钟
    __HAL_RCC_GPIOD_CLK_ENABLE();  // HAL 宏，打开 GPIOD 时钟
    __HAL_RCC_GPIOH_CLK_ENABLE();  // HAL 宏，打开 GPIOH 时钟
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_10, GPIO_PIN_RESET);  // HAL 库，设置输出引脚的上电初值
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // HAL 库，设置输出引脚的上电初值
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // HAL 库，设置输出引脚的上电初值
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);  // HAL 库，装载引脚模式和复用配置
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  // HAL 库，装载引脚模式和复用配置
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  // HAL 库，装载引脚模式和复用配置
}
/* USER CODE BEGIN 2 */
/* USER CODE END 2 */

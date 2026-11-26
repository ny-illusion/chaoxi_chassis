/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file spi.c
  * @brief SPI1 主机与复用引脚配置
  * @board 底盘板
  * @vehicle_function 为底盘陀螺仪提供 SPI 寄存器读通路
  * @inputs SPI1 初始化参数、SPI 引脚和电源时钟
  * @outputs SPI1 句柄、驱动时钟与引脚配置
  * @dependencies STM32F4 HAL SPI、GPIO 和 RCC
  * @note 重构说明：本文件行为与原代码保持一致。main 在传感器探测前调用 MX_SPI1_Init。
  *
  * This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
SPI_HandleTypeDef hspi1;

void MX_SPI1_Init(void)  // CubeMX 生成，配置 BMI088 使用的 SPI1 主机
{
  /* USER CODE BEGIN SPI1_Init 0 */
  /* USER CODE END SPI1_Init 0 */
  /* USER CODE BEGIN SPI1_Init 1 */
  /* USER CODE END SPI1_Init 1 */
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)  // HAL 库，装载 SPI 主机参数
    {
        Error_Handler();
    }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle)  // HAL 硬件初始化钩子，CubeMX 生成实现
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (spiHandle->Instance == SPI1)
    {
  /* USER CODE BEGIN SPI1_MspInit 0 */
  /* USER CODE END SPI1_MspInit 0 */
        __HAL_RCC_SPI1_CLK_ENABLE();  // HAL 宏，打开SPI1 时钟
        __HAL_RCC_GPIOB_CLK_ENABLE();  // HAL 宏，打开 GPIOB 时钟
        __HAL_RCC_GPIOA_CLK_ENABLE();  // HAL 宏，打开 GPIOA 时钟
        GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  // HAL 库，装载引脚模式和复用配置
        GPIO_InitStruct.Pin = GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  // HAL 库，装载引脚模式和复用配置
  /* USER CODE BEGIN SPI1_MspInit 1 */
  /* USER CODE END SPI1_MspInit 1 */
    }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spiHandle)  // HAL 硬件反初始化钩子，CubeMX 生成实现
{
    if (spiHandle->Instance == SPI1)
    {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */
  /* USER CODE END SPI1_MspDeInit 0 */
        __HAL_RCC_SPI1_CLK_DISABLE();  // HAL 宏，关闭SPI1 时钟
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4 | GPIO_PIN_3);  // HAL 库，释放外设引脚
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);  // HAL 库，释放外设引脚
  /* USER CODE BEGIN SPI1_MspDeInit 1 */
  /* USER CODE END SPI1_MspDeInit 1 */
    }
}
/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

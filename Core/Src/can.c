/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file can.c
  * @brief CAN1 外设、引脚与中断配置
  * @board 底盘板
  * @vehicle_function 连接板间遥控链路和电机反馈总线
  * @inputs APB1 时钟、CAN1 初始化参数、PD0/PD1
  * @outputs CAN1 句柄及 RX0 中断配置
  * @dependencies STM32F4 HAL CAN、RCC、GPIO、NVIC
  * @note 重构说明：本文件行为与原代码保持一致。main 调用 MX_CAN1_Init，应用层另行配置接收过滤器。
  *
  * This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
CAN_HandleTypeDef hcan1;

void MX_CAN1_Init(void)  // CubeMX 生成，配置 CAN1 为 1 Mbit/s 标准工作模式
{
  /* USER CODE BEGIN CAN1_Init 0 */
  /* USER CODE END CAN1_Init 0 */
  /* USER CODE BEGIN CAN1_Init 1 */
  /* USER CODE END CAN1_Init 1 */
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 3;  // 与 BS1、BS2 一起决定 CAN1 的 1 Mbit/s 位速率
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)  // HAL 库，初始化 bxCAN 外设
    {
        Error_Handler();
    }
  /* USER CODE BEGIN CAN1_Init 2 */
  /* USER CODE END CAN1_Init 2 */
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)  // HAL 硬件初始化钩子，CubeMX 生成实现
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (canHandle->Instance == CAN1)
    {
  /* USER CODE BEGIN CAN1_MspInit 0 */
  /* USER CODE END CAN1_MspInit 0 */
        __HAL_RCC_CAN1_CLK_ENABLE();  // HAL 宏，打开 CAN1 时钟
        __HAL_RCC_GPIOD_CLK_ENABLE();  // HAL 宏，打开 GPIOD 时钟
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;  // PD0 接 CAN1_RX，PD1 接 CAN1_TX
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);  // HAL 库，装载引脚模式和复用配置
        HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);  // HAL 库，设置中断抢占和子优先级
        HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);  // HAL 库，允许此中断进入 NVIC
  /* USER CODE BEGIN CAN1_MspInit 1 */
  /* USER CODE END CAN1_MspInit 1 */
    }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle)  // HAL 硬件反初始化钩子，CubeMX 生成实现
{
    if (canHandle->Instance == CAN1)
    {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */
  /* USER CODE END CAN1_MspDeInit 0 */
        __HAL_RCC_CAN1_CLK_DISABLE();  // HAL 宏，关闭 CAN1 时钟
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1);  // HAL 库，释放外设引脚
        HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);  // HAL 库，屏蔽此中断
  /* USER CODE BEGIN CAN1_MspDeInit 1 */
  /* USER CODE END CAN1_MspDeInit 1 */
    }
}
/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

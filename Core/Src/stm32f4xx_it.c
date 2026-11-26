/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file stm32f4xx_it.c
  * @brief Cortex-M 异常与外设中断入口
  * @board 底盘板
  * @vehicle_function 把系统滴答、CAN1 和 TIM6 事件送给 HAL
  * @inputs 向量表触发的异常或中断、对应外设句柄
  * @outputs HAL 滴答更新、外设 IRQ 分派及故障处理
  * @dependencies 启动向量表、CMSIS 和 STM32F4 HAL
  * @note 重构说明：本文件行为与原代码保持一致。向量表按处理函数的原始符号名绑定。
  *
  * Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
/* USER CODE BEGIN TD */
/* USER CODE END TD */
/* USER CODE BEGIN PD */
/* USER CODE END PD */
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* USER CODE BEGIN PV */
/* USER CODE END PV */
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim6;
/* USER CODE BEGIN EV */
/* USER CODE END EV */

void NMI_Handler(void)  // 中断向量入口，CubeMX 生成；不可屏蔽中断
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  /* USER CODE END NonMaskableInt_IRQn 0 */
    HAL_RCC_NMI_IRQHandler();  // HAL 库，处理时钟安全系统中断
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1)
    {
    }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

void HardFault_Handler(void)  // 中断向量入口，CubeMX 生成；硬故障
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  /* USER CODE END HardFault_IRQn 0 */
    while (1)
    {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

void MemManage_Handler(void)  // 中断向量入口，CubeMX 生成；存储管理故障
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  /* USER CODE END MemoryManagement_IRQn 0 */
    while (1)
    {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

void BusFault_Handler(void)  // 中断向量入口，CubeMX 生成；总线故障
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  /* USER CODE END BusFault_IRQn 0 */
    while (1)
    {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

void UsageFault_Handler(void)  // 中断向量入口，CubeMX 生成；用法故障
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  /* USER CODE END UsageFault_IRQn 0 */
    while (1)
    {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

void SVC_Handler(void)  // 中断向量入口，CubeMX 生成；系统服务调用
{
  /* USER CODE BEGIN SVCall_IRQn 0 */
  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */
  /* USER CODE END SVCall_IRQn 1 */
}

void DebugMon_Handler(void)  // 中断向量入口，CubeMX 生成；调试监控
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */
  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */
  /* USER CODE END DebugMonitor_IRQn 1 */
}

void PendSV_Handler(void)  // 中断向量入口，CubeMX 生成；可挂起系统服务
{
  /* USER CODE BEGIN PendSV_IRQn 0 */
  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */
  /* USER CODE END PendSV_IRQn 1 */
}

void SysTick_Handler(void)  // 中断向量入口，CubeMX 生成；系统滴答
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();  // HAL 库，递增 HAL 毫秒计数
  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */
}

void CAN1_RX0_IRQHandler(void)  // 中断向量入口，CubeMX 生成；CAN1 接收 FIFO0
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
  /* USER CODE END CAN1_RX0_IRQn 0 */
    HAL_CAN_IRQHandler(&hcan1);  // HAL 库，分派 CAN 中断
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */
  /* USER CODE END CAN1_RX0_IRQn 1 */
}

void TIM6_DAC_IRQHandler(void)  // 中断向量入口，CubeMX 生成；TIM6 更新与 DAC 共用
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
  /* USER CODE END TIM6_DAC_IRQn 0 */
    HAL_TIM_IRQHandler(&htim6);  // HAL 库，分派定时器中断
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
  /* USER CODE END TIM6_DAC_IRQn 1 */
}
/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

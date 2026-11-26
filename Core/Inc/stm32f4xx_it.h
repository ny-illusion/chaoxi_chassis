/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file stm32f4xx_it.h
  * @brief Cortex-M 异常与外设中断入口的声明
  * @board 底盘板
  * @vehicle_function 把系统滴答、CAN1 和 TIM6 事件送给 HAL
  * @inputs 向量表触发的异常或中断、对应外设句柄
  * @outputs 异常与中断处理函数的原始声明
  * @dependencies 启动向量表、CMSIS 和 STM32F4 HAL
  * @note 重构说明：本文件行为与原代码保持一致。向量表按处理函数的原始符号名绑定。
  *
  * This file contains the headers of the interrupt handlers.
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
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H
#ifdef __cplusplus
extern "C"
{
#endif
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
/* USER CODE BEGIN ET */
/* USER CODE END ET */
/* USER CODE BEGIN EC */
/* USER CODE END EC */
/* USER CODE BEGIN EM */
/* USER CODE END EM */
    void NMI_Handler(void);  // 中断向量入口，CubeMX 生成；不可屏蔽中断
    void HardFault_Handler(void);  // 中断向量入口，CubeMX 生成；硬故障
    void MemManage_Handler(void);  // 中断向量入口，CubeMX 生成；存储管理故障
    void BusFault_Handler(void);  // 中断向量入口，CubeMX 生成；总线故障
    void UsageFault_Handler(void);  // 中断向量入口，CubeMX 生成；用法故障
    void SVC_Handler(void);  // 中断向量入口，CubeMX 生成；系统服务调用
    void DebugMon_Handler(void);  // 中断向量入口，CubeMX 生成；调试监控
    void PendSV_Handler(void);  // 中断向量入口，CubeMX 生成；可挂起系统服务
    void SysTick_Handler(void);  // 中断向量入口，CubeMX 生成；系统滴答
    void CAN1_RX0_IRQHandler(void);  // 中断向量入口，CubeMX 生成；CAN1 接收 FIFO0
    void TIM6_DAC_IRQHandler(void);  // 中断向量入口，CubeMX 生成；TIM6 更新与 DAC 共用
/* USER CODE BEGIN EFP */
/* USER CODE END EFP */
#ifdef __cplusplus
}
#endif
#endif

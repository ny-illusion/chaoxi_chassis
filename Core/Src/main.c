/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file main.c
  * @brief 底盘启动顺序、主循环与定时调度入口
  * @board 底盘板
  * @vehicle_function 按板级顺序启用反馈和控制周期
  * @inputs 复位、TIM6 更新中断、HAL 外设初始化
  * @outputs 板级外设启动、前台 IMU 轮询和每毫秒控制调度
  * @dependencies CubeMX 外设接口、chassis_imu_feedback、chassis_can_bus、chassis_control_loop
  * @note 重构说明：本文件行为与原代码保持一致。先启动 CAN，再初始化控制状态，然后开启接收通知和周期定时器。
  *
  * Main program body
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
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
/* USER CODE BEGIN Includes */
#include "chassis_control_loop.h"
#include "chassis_can_bus.h"
#include "chassis_imu_feedback.h"
/* USER CODE END Includes */
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* USER CODE BEGIN PD */
/* USER CODE END PD */
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* USER CODE BEGIN PV */
/* USER CODE END PV */

void SystemClock_Config(void);  // CubeMX 生成，配置 HSE、PLL 和总线分频
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)  // CubeMX 生成入口，项目在用户区轮询 IMU 并接收板间遥控
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
    HAL_Init();  // HAL 库，初始化 HAL 和系统滴答
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
    SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */
    MX_GPIO_Init();  // CubeMX 生成，配置板上使用的 GPIO 引脚
    MX_CAN1_Init();  // CubeMX 生成，配置 CAN1 为 1 Mbit/s 标准工作模式
    MX_TIM6_Init();  // CubeMX 生成，配置 TIM6 为 1 ms 更新周期
    MX_SPI1_Init();  // CubeMX 生成，配置 BMI088 使用的 SPI1 主机
  /* USER CODE BEGIN 2 */
    chassis_imu_read_configuration();
    chassis_can_start();
    ChassisApp_Init();  // 项目函数，清零四轮控制状态并初始化指示灯

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  // HAL 库，开启 FIFO0 待接收消息通知
    HAL_TIM_Base_Start_IT(&htim6);  // HAL 库，启动 TIM6 周期中断
  /* USER CODE END 2 */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
        chassis_imu_poll();
    }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)  // CubeMX 生成，配置 HSE、PLL 和总线分频
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();  // HAL 宏，打开PWR 时钟
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)  // HAL 库，应用振荡器和 PLL 配置
    {
        Error_Handler();
    }
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)  // HAL 库，切换时钟源并设置总线分频
    {
        Error_Handler();
    }
    HAL_RCC_EnableCSS();  // HAL 库，启用时钟安全系统
}
/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  // HAL 回调，项目实现；TIM6 每 1 ms 调度底盘
{
    if (htim->Instance == TIM6)
    {
        ChassisApp_Run1ms();  // 项目函数，读取遥控和反馈，计算并下发四轮与 yaw 命令
    }
}

/* USER CODE END 4 */

void Error_Handler(void)  // CubeMX 生成的错误处理入口
{
  /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();  // CMSIS 内核接口，错误处理时关闭可屏蔽中断
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)  // HAL 断言钩子，CubeMX 生成实现
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif

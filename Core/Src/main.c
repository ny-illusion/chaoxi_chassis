/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h" /* 引入芯片库公共类型和错误处理声明。 */
#include "can.h" /* 引入CAN 外设句柄和初始化声明。 */
#include "tim.h" /* 引入定时器句柄和初始化声明。 */
#include "usart.h" /* 引入串口句柄和初始化声明。 */
#include "gpio.h" /* 引入引脚初始化声明。 */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassis_loop.h" /* 引入底盘整体调度接口。 */
#include "chassis_motor.h" /* 引入电机反馈、PI 和发送接口。 */
#include "rc_sbus.h" /* 引入遥控数据类型和接收接口。 */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* 函数作用：配置晶振、PLL 和总线时钟；没有输入参数，也不返回数值。 */
void SystemClock_Config(void); /* 提前声明时钟配置函数，下面的 main 才能调用它。 */
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * 主程序：初始化硬件，把定时器、CAN、串口的回调转交给对应模块。
 * Callback=回调（事件发生后由芯片库调用）；Handle=句柄（保存外设信息）。
 * FIFO=先入先出的接收队列；Filter=筛选规则；Notification=中断通知。
 * HAL=芯片硬件操作库；Init=初始化；IT=中断方式；Rx=接收；Tx=发送。
 * C 语法：static 全局量只在本文件可见，局部 static 跨调用保留值；const 表示不能通过它修改数据。
 * volatile 提醒编译器每次访问实际内存，不保证多字段读写不被打断。
 * -> 访问指针所指结构体的成员；& 取地址；! 取反；&& 且；|| 或。
 */


/* USER CODE END 0 */


/**
  * @brief  程序入口：初始化硬件，再等待中断运行控制任务。
  * @retval int
  */
/* 函数作用：程序入口：上电初始化外设，启动接收和定时器；后续控制在中断中执行。 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* 初始化芯片库、Flash 接口和系统滴答计时。 */

  HAL_Init(); /* 初始化芯片库、系统毫秒计时和中断分组。 */

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* 设置系统时钟。 */
  SystemClock_Config(); /* 配置系统和外设总线使用的时钟。 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* 初始化所需外设。 */
  MX_GPIO_Init(); /* 初始化引脚，包括运行灯和停止灯。 */
  MX_CAN1_Init(); /* 配置 CAN1 的通信速率等参数，稍后再启动通信。 */
  MX_TIM6_Init(); /* 配置 TIM6，使它每隔 1 ms 产生一次更新事件。 */
  MX_USART1_UART_Init(); /* 配置 USART1，供遥控接收机传入 SBUS 数据。 */
  /* USER CODE BEGIN 2 */
  
CAN_FilterTypeDef filter = {0};                  /* 先全清零，避免垃圾值 */
filter.FilterBank           = 0;                 /* 用 0 号滤波器 */
filter.FilterMode           = CAN_FILTERMODE_IDMASK;  /* 掩码模式 */
filter.FilterScale          = CAN_FILTERSCALE_32BIT;  /* 32 位宽 */
filter.FilterIdHigh         = 0x0000; /* 设置匹配编号的高 16 位；本例掩码为 0，不限制编号。 */
filter.FilterIdLow          = 0x0000; /* 设置匹配编号的低 16 位。 */
filter.FilterMaskIdHigh     = 0x0000; /* 高 16 位掩码全为 0，表示这些位都不检查。 */
filter.FilterMaskIdLow      = 0x0000; /* 低 16 位也不检查，所以先接收所有编号，再由软件筛选。 */
filter.FilterFIFOAssignment = CAN_RX_FIFO0;      /* 接收数据进入队列 0；FIFO 是先到先读的队列 */
filter.FilterActivation     = ENABLE;            /* 启用 */
HAL_CAN_ConfigFilter(&hcan1, &filter); /* 把 filter 中的筛选规则写入 CAN1 硬件；& 表示取变量地址。 */
HAL_CAN_Start(&hcan1); /* 启动 CAN1，让它开始参与总线通信。 */

ChassisApp_Init(); /* 清空电机状态，并设置底盘的初始指示灯。 */

RC_SBUS_Start(&huart1); /* 让 USART1 开始通过中断逐字节接收遥控数据。 */

HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); /* 允许 CAN 接收队列 0 有数据时触发中断。 */

HAL_TIM_Base_Start_IT(&htim6);      /* ★ 启动 TIM6，每 1ms 进一次中断 */
  /* USER CODE END 2 */

  /* 主循环：这里暂时为空，控制任务在中断里执行。 */
  /* USER CODE BEGIN WHILE */
   while (1) /* 一直循环等待；这里控制任务由中断调用，循环中暂无操作。 */
  {
    /* USER CODE END WHILE */






    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief 系统时钟配置：设置晶振、PLL 和各总线分频。
  * @retval None
  */
/* 函数作用：配置晶振、PLL 和总线时钟；没有输入参数，也不返回数值。 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; /* 准备晶振和倍频器配置，先把各字段清零。 */
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; /* 准备 CPU 和各总线的时钟配置。 */

  /** 设置芯片内部稳压器的工作档位。
  */
  __HAL_RCC_PWR_CLK_ENABLE(); /* 打开电源控制模块的时钟，才能修改它的设置。 */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1); /* 选择芯片高速运行所需的内部电压档位。 */

  /** 按照下面填写的参数配置晶振和 PLL。
  * RCC_OscInitTypeDef 是保存这些配置的结构体类型。
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE; /* 选择外部高速时钟 HSE 作为本次配置对象。 */
  RCC_OscInitStruct.HSEState = RCC_HSE_ON; /* 打开外部高速晶振。 */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; /* 打开 PLL 倍频器，用来得到更高频率。 */
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; /* PLL 的输入来自外部晶振。 */
  RCC_OscInitStruct.PLL.PLLM = 6; /* PLL 输入先除以 6；与外部晶振频率配套使用。 */
  RCC_OscInitStruct.PLL.PLLN = 168; /* 再把频率乘以 168。 */
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; /* 主时钟输出再除以 2；最终频率为 HSE ÷ 6 × 168 ÷ 2。 */
  RCC_OscInitStruct.PLL.PLLQ = 4; /* PLL 的另一条输出支路除以 4，供有关外设使用。 */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) /* 应用晶振和 PLL 配置；失败就进入下面的错误处理。 */
  {
    Error_Handler(); /* 进入错误处理函数，关闭可屏蔽中断并停在循环中。 */
  }

  /** 设置 CPU、AHB 和 APB 外设总线的时钟。
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK /* 用按位或 | 同时选择 AHB 和系统时钟配置项。 */
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2; /* 继续选择 APB1、APB2 两条外设总线的时钟配置项。 */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; /* 系统主时钟采用 PLL 的输出。 */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; /* AHB 总线使用系统时钟，不再分频。 */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4; /* APB1 总线频率是 AHB 的四分之一。 */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; /* APB2 总线频率是 AHB 的二分之一。 */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) /* 应用总线时钟，并设置 Flash 读取等待周期；失败则处理错误。 */
  {
    Error_Handler(); /* 进入错误处理函数，关闭可屏蔽中断并停在循环中。 */
  }

  /** 打开外部时钟故障检测。
  */
  HAL_RCC_EnableCSS(); /* 启用外部时钟故障检测，CSS 表示时钟安全系统。 */
}

/* USER CODE BEGIN 4 */

//TIM6 中断回调，每 1ms 调用一次
/* 函数作用：定时器计时回调：htim 指向触发事件的定时器；只在 TIM6 事件中运行底盘。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) /* Instance 表示外设实例；只处理 TIM6 的计时事件。 */
    {
        ChassisApp_Run1ms();  /* 每 1ms 调一次 */
    }
}



/* CAN 接收回调。 */
/* 函数作用：CAN 接收回调：hcan 指向触发事件的 CAN；取出 CAN1 队列中的帧，交给电机解析。 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    /* 保存帧头和八个字节。 */
    CAN_RxHeaderTypeDef header; /* header 表示帧头，用来保存编号、长度和帧类型。 */
    uint8_t data[8]; /* 准备 8 个字节，存放这一帧 CAN 的实际数据。 */

    /* 只处理 CAN1。 */
    if (hcan->Instance != CAN1) return;

    /* 一次取完 FIFO 中的所有帧。 */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        /* 读取一帧。 */
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) break;

        /* 交给电机模块解析。 */
        ChassisMotor_ReadCAN(&header, data);
    }
}

/* 串口接收回调。 */
/* 函数作用：串口接收完成回调：huart 指向接收串口；把新字节交给 SBUS 拼帧函数。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* 交给 SBUS 模块解析。 */
    RC_SBUS_ReadByte(huart);
}

/* USER CODE END 4 */

/**
  * @brief  初始化出错时执行：关闭可屏蔽中断，停在循环内。
  * @retval None
  */
/* 函数作用：硬件初始化失败时调用；关闭可屏蔽中断并永久等待。这里没有主动发送零电流。 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 可在这里补充错误提示；当前实现只停住程序。 */
  __disable_irq(); /* 暂时禁止可屏蔽中断，避免复制或更新数据时被打断。 */
  while (1) /* 一直循环等待；这里控制任务由中断调用，循环中暂无操作。 */
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  断言失败回调：提供出错文件名和代码行号。
  * assert_param 表示检查函数参数是否符合要求。
  * @param  file：指向出错文件名的指针。
  * @param  line：断言失败处的代码行号。
  * @retval None
  */
/* 函数作用：完整断言开启时报告参数错误；file 是文件名，line 是出错行号。目前函数体为空。 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

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
#include "main.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

volatile uint8_t imu_gyro_id = 0;
volatile int32_t imu_spi_status = -1;
volatile uint8_t imu_gyro_range = 0xFFU;
volatile int32_t imu_gyro_range_status = -1;
volatile int16_t imu_gyro_z_raw = 0;
volatile float imu_gyro_z_dps = 0.0f;
volatile int32_t imu_gyro_read_status = -1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

uint8_t tx[2] = {0x80, 0x00};  // 读寄存器 0x00，最高位置 1 表示读
uint8_t rx[2] = {0};

HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);    // 加速度计不占用总线
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    // 陀螺仪片选先拉高
HAL_Delay(100);                                       // 等传感器上电稳定

HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // 选中陀螺仪
imu_spi_status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 10);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    // 结束本次读取

if (imu_spi_status == HAL_OK) {
    imu_gyro_id = rx[1];  // 第一个字节发地址，第二个字节收到 ID
}
  
if (imu_gyro_id == 0x0FU) {
    tx[0] = 0x8FU;  /* 读取 GYRO_RANGE（0x0F），tx[1] 仍为 0。 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    imu_gyro_range_status = (int32_t)HAL_SPI_TransmitReceive(
        &hspi1, tx, rx, 2U, 10U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    if (imu_gyro_range_status == HAL_OK) {
        imu_gyro_range = rx[1];
    }
}

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

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1) /* 一直循环等待；这里控制任务由中断调用，循环中暂无操作。 */
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    static uint32_t last_gyro_poll_ms = 0U;
    const uint32_t now_ms = HAL_GetTick();

    if (imu_gyro_range == 0x00U &&
        (uint32_t)(now_ms - last_gyro_poll_ms) >= 5U) {
        last_gyro_poll_ms = now_ms;

        const uint8_t z_tx[3] = {0x86U, 0x00U, 0x00U};
        uint8_t z_rx[3] = {0};

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        const HAL_StatusTypeDef status =
            HAL_SPI_TransmitReceive(&hspi1, z_tx, z_rx, 3U, 2U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

        if (status == HAL_OK) {
            const int16_t raw = (int16_t)(((uint16_t)z_rx[2] << 8) | z_rx[1]);
            const float dps = (float)raw * (2000.0f / 32768.0f);
            imu_gyro_z_raw = raw;
            imu_gyro_z_dps = dps;
            ChassisApp_SetGyroZ(dps);
        }
        imu_gyro_read_status = (int32_t)status;
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

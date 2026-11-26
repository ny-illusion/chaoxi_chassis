/**
 * @file chassis_imu_feedback.c
 * @brief BMI088 陀螺仪探测与 Z 轴采样
 * @board 底盘板
 * @vehicle_function 提供底盘角速度测量和传感器诊断值
 * @inputs SPI1 读数、芯片 ID、量程和 HAL 毫秒计数
 * @outputs 最近一次读数、事务状态及传给控制环的角速度
 * @dependencies spi.h、gpio.h、chassis_control_loop.h 和 BMI088
 * @note 重构说明：本文件行为与原代码保持一致。main 在 SPI 初始化后探测，在前台循环轮询。
 */

#include "chassis_imu_feedback.h"
#include "chassis_control_loop.h"
#include "gpio.h"
#include "spi.h"

volatile uint8_t imu_gyro_id = 0;
volatile int32_t imu_spi_status = -1;
volatile uint8_t imu_gyro_range = 0xFFU;
volatile int32_t imu_gyro_range_status = -1;
volatile int16_t imu_gyro_z_raw = 0;
volatile float imu_gyro_z_dps = 0.0f;
volatile int32_t imu_gyro_read_status = -1;

void chassis_imu_read_configuration(void)
{
    uint8_t register_request[2] = {0x80, 0x00};  // 读取 BMI088 陀螺仪的芯片 ID
    uint8_t register_response[2] = {0};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // HAL 库，拉高 PA4 释放加速度计片选
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // HAL 库，拉高 PB0 释放陀螺仪片选
    HAL_Delay(100);  // HAL 库，等待 BMI088 上电稳定
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // HAL 库，拉低 PB0 选中 BMI088 陀螺仪
    imu_spi_status = HAL_SPI_TransmitReceive(&hspi1, register_request, register_response, 2, 10);  // HAL 库，同步交换寄存器地址和数据
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // HAL 库，拉高 PB0 释放陀螺仪片选
    if (imu_spi_status == HAL_OK)
    {
        imu_gyro_id = register_response[1];
    }

    if (imu_gyro_id == 0x0FU)
    {
        register_request[0] = 0x8FU;  // 读取 GYRO_RANGE 以确认量程
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // HAL 库，拉低 PB0 选中 BMI088 陀螺仪
        imu_gyro_range_status = (int32_t)HAL_SPI_TransmitReceive(  // HAL 库，同步交换寄存器地址和数据
            &hspi1, register_request, register_response, 2U, 10U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // HAL 库，拉高 PB0 释放陀螺仪片选
        if (imu_gyro_range_status == HAL_OK)
        {
            imu_gyro_range = register_response[1];
        }
    }
}

void chassis_imu_poll(void)
{
    static uint32_t last_gyro_poll_ms = 0U;  // 主循环每 5 ms 轮询 BMI088，控制计算由 TIM6 触发
    const uint32_t now_ms = HAL_GetTick();  // HAL 库，读取本板的毫秒计数
    if (imu_gyro_range == 0x00U &&  // 仅在确认为 ±2000 °/s 量程时按 5 ms 周期采样
        (uint32_t)(now_ms - last_gyro_poll_ms) >= 5U)
    {
        last_gyro_poll_ms = now_ms;

        const uint8_t z_axis_request[3] = {0x86U, 0x00U, 0x00U};  // 从 BMI088 的 0x06 开始连续读取 Z 轴低、高字节
        uint8_t z_axis_response[3] = {0};
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // HAL 库，拉低 PB0 选中 BMI088 陀螺仪
        const HAL_StatusTypeDef transfer_status =
            HAL_SPI_TransmitReceive(&hspi1, z_axis_request, z_axis_response, 3U, 2U);  // HAL 库，同步交换寄存器地址和数据
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // HAL 库，拉高 PB0 释放陀螺仪片选
        if (transfer_status == HAL_OK)
        {
            const int16_t z_axis_counts = (int16_t)(((uint16_t)z_axis_response[2] << 8) | z_axis_response[1]);
            const float z_axis_dps = (float)z_axis_counts * (2000.0f / 32768.0f);  // 量程寄存器为 0 时按 ±2000 °/s 换算
            imu_gyro_z_raw = z_axis_counts;
            imu_gyro_z_dps = z_axis_dps;
            ChassisApp_SetGyroZ(z_axis_dps);  // 项目函数，用静止样本校零，将底盘 Z 轴角速度转换为 rpm
        }
        imu_gyro_read_status = (int32_t)transfer_status;
    }
}

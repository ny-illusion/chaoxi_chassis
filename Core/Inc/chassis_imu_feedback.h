/**
 * @file chassis_imu_feedback.h
 * @brief BMI088 陀螺仪探测与 Z 轴采样的接口声明
 * @board 底盘板
 * @vehicle_function 提供底盘角速度测量和传感器诊断值
 * @inputs SPI1 读数、芯片 ID、量程和 HAL 毫秒计数
 * @outputs 最近一次读数、事务状态及传给控制环的角速度
 * @dependencies spi.h、gpio.h、chassis_control_loop.h 和 BMI088
 * @note 重构说明：本文件行为与原代码保持一致。main 在 SPI 初始化后探测，在前台循环轮询。
 */

#ifndef CHASSIS_IMU_FEEDBACK_H
#define CHASSIS_IMU_FEEDBACK_H

void chassis_imu_read_configuration(void);
void chassis_imu_poll(void);

#endif

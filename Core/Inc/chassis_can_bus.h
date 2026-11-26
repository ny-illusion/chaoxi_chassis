/**
 * @file chassis_can_bus.h
 * @brief CAN1 滤波启动与 FIFO0 接收分派的接口声明
 * @board 底盘板
 * @vehicle_function 把板间遥控和电机反馈送到各自的接收模块
 * @inputs CAN1 FIFO0 帧头及载荷
 * @outputs CAN1 启动状态、遥控快照更新和电机反馈更新
 * @dependencies can.h、chassis_remote_communication.h、chassis_motor_driver.h
 * @note 重构说明：本文件行为与原代码保持一致。main 启动 CAN1 后才初始化控制状态和开启接收通知。
 */

#ifndef CHASSIS_CAN_BUS_H
#define CHASSIS_CAN_BUS_H

void chassis_can_start(void);

#endif

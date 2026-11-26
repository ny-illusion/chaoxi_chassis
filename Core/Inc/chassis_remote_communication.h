/**
 * @file chassis_remote_communication.h
 * @brief 板间 CAN 遥控帧解包与快照发布的接口声明
 * @board 底盘板
 * @vehicle_function 为底盘控制周期提供来自云台板的操纵输入
 * @inputs 来自接收中断的八字节遥控载荷和本机时间
 * @outputs 带通道、有效位、帧计数及时间戳的快照
 * @dependencies chassis_remote_snapshot.h 的快照类型、HAL 滴答和 CMSIS 中断屏蔽接口
 * @note 重构说明：本文件行为与原代码保持一致。在 CAN 接收路径发布，在 TIM6 中复制整帧。
 */

#ifndef CHASSIS_REMOTE_COMMUNICATION_H
#define CHASSIS_REMOTE_COMMUNICATION_H

#include "chassis_remote_snapshot.h"

#define CHASSIS_REMOTE_CAN_ID 0x301U  // 两板约定的遥控标准数据帧
void ChassisRemote_ReceiveCAN(const uint8_t data[8]);  // 项目函数，解包云台 0x301，并记录底盘本机接收时间
void ChassisRemote_CopyData(RC_SBUS_Snapshot *snapshot);  // 项目函数，为 TIM6 复制同一帧的遥控通道和状态

#endif

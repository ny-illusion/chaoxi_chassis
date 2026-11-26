/**
 * @file chassis_motor_driver.h
 * @brief 声明底盘电机驱动接口和轮电机 CAN 标识。
 * @board 底盘板
 * @vehicle_function 刷新四轮转速和 yaw 编码器反馈，按控制周期向电调下发指令。
 * @inputs CAN1 电机反馈、期望转速、使能、控制周期与制动状态。
 * @outputs 四轮电流帧、yaw 电机电压帧，以及提供给控制循环的相对角。
 * @dependencies main.h 中的 CAN 类型与 chassis_motion_types.h。
 * @note 重构说明：本文件行为与原代码保持一致。由 CAN 接收回调和底盘控制循环调用。
 */
#ifndef CHASSIS_MOTOR_DRIVER_H
#define CHASSIS_MOTOR_DRIVER_H

#include "main.h"
#include "chassis_motion_types.h"

#define CHASSIS_MOTOR_CAN_TX_ID 0x200U
#define CHASSIS_MOTOR_CAN_RX_FIRST_ID 0x201U
#define CHASSIS_MOTOR_CAN_RX_LAST_ID 0x204U
void ChassisMotor_Init(void);  // 项目函数，复位四轮反馈、积分和电流指令
void ChassisMotor_ReadCAN(const CAN_RxHeaderTypeDef *header, const uint8_t *data);  // 项目函数，分派 0x201～0x204 轮速和 0x206 yaw 反馈

void ChassisMotor_Update(const float target_rpm[CHASSIS_WHEEL_COUNT],  // 项目函数，用速度 PI 计算四轮电流并发送 0x200
                         uint8_t enabled, float dt_s);  // 项目函数，用速度 PI 计算四轮电流并发送 0x200

void GM6020_Update(float target_rpm, uint8_t enabled, float dt_s,  // 项目函数，根据 yaw 目标与转速反馈计算电压，发送 0x1FF
                   uint8_t stick_centered);  // 项目函数，根据 yaw 目标与转速反馈计算电压，发送 0x1FF
#define GM6020_YAW_DIR 1.0f  // 统一编码器和 yaw 控制的正方向
#endif

/**
 * @file chassis_control_loop.h
 * @brief 声明底盘控制循环的启动、反馈写入与周期入口。
 * @board 底盘板
 * @vehicle_function 每个 TIM6 周期更新车轮和 yaw 电机命令，两次指示灯刷新至少间隔 100 ms。
 * @inputs 板间遥控快照、底盘角速度、yaw 编码器反馈和本机毫秒计数。
 * @outputs 底盘运动命令、四轮目标转速与 yaw 电机目标转速。
 * @dependencies 标准整数类型 stdint.h 与 chassis_control_loop.c。
 * @note 重构说明：本文件行为与原代码保持一致。启动入口由 main 调用，周期入口由 TIM6 回调调用。
 */
#ifndef CHASSIS_CONTROL_LOOP_H
#define CHASSIS_CONTROL_LOOP_H

#include <stdint.h>

void ChassisApp_Init(void);  // 项目函数，清零四轮控制状态并初始化指示灯
void ChassisApp_SetGimbalFeedback(float relative_yaw_rad);  // 项目函数，保存 yaw 编码器相对角和本机接收时间
void ChassisApp_SetGyroZ(float gyro_z_dps);  // 项目函数，用静止样本校零，将底盘 Z 轴角速度转换为 rpm
void ChassisApp_Run1ms(void);  // 项目函数，读取遥控和反馈，计算并下发四轮与 yaw 命令
#endif

/**
 * @file chassis_motion_control.h
 * @brief 声明底盘运动控制接口和模式参数。
 * @board 底盘板
 * @vehicle_function 将遥控平移与旋转请求转换为四轮运动学使用的输入。
 * @inputs ChassisInput、ChassisGimbalFeedback 和本机毫秒计数。
 * @outputs 带模式与三轴轮速分量的 ChassisCommand、折回到单圈的角度。
 * @dependencies chassis_motion_types.h。
 * @note 重构说明：本文件行为与原代码保持一致。控制循环和主机测试通过这些声明调用运动算法。
 */
#ifndef CHASSIS_MOTION_CONTROL_H
#define CHASSIS_MOTION_CONTROL_H

#include "chassis_motion_types.h"

#define CHASSIS_GIMBAL_INSTALLED 1U  // 当前装有 yaw 云台，按有效反馈进行坐标变换和稳向
#define CHASSIS_GIMBAL_TIMEOUT_MS 100U  // 运动模式层检查相对角反馈年龄，单位 ms
#define CHASSIS_SPIN_RPM_LOW 1500.0f  // SC1 低速自旋的轮速分量，单位 rpm
#define CHASSIS_SPIN_RPM_HIGH 3000.0f  // SC2 高速自旋的轮速分量，单位 rpm
#define CHASSIS_FOLLOW_KP 3000.0f  // 相对角误差换算为轮速旋转分量，单位 rpm/rad
#define CHASSIS_FOLLOW_RPM_MAX 2250.0f  // SC0 角度跟随输出的轮速上限，单位 rpm
#define CHASSIS_FOLLOW_DEADBAND_RAD 0.035f  // 跟随误差死区，单位 rad

float ChassisControl_WrapAngle(float angle);  // 项目函数，将弧度折回 −π 至 +π

void ChassisControl_Step(const ChassisInput *input,  // 项目函数，根据使能、SC 和云台方向计算底盘的三轴命令
                         const ChassisGimbalFeedback *feedback,
                         uint32_t now_ms, ChassisCommand *command);  // 项目函数，根据使能、SC 和云台方向计算底盘的三轴命令

#endif

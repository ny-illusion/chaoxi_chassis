/**
 * @file chassis_wheel_kinematics.h
 * @brief 声明四轮目标转速分配接口。
 * @board 底盘板
 * @vehicle_function 按安装方向和转速上限生成每个轮电机的目标转速。
 * @inputs 底盘运动命令和单轮转速上限，单位 rpm。
 * @outputs 按左前、右前、右后、左后顺序排列的四个目标转速。
 * @dependencies chassis_motion_types.h。
 * @note 重构说明：本文件行为与原代码保持一致。调用方提供可写的四轮转速数组。
 */
#ifndef CHASSIS_WHEEL_KINEMATICS_H
#define CHASSIS_WHEEL_KINEMATICS_H

#include "chassis_motion_types.h"

void ChassisControl_Mix(const ChassisCommand *command, float limit_rpm,  // 项目函数，分配四轮目标转速，超限时四轮同比缩放
                        float targets[CHASSIS_WHEEL_COUNT]);  // 项目函数，分配四轮目标转速，超限时四轮同比缩放

#endif

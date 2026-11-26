/**
 * @file chassis_motion.h
 * @brief 保留底盘运动控制与四轮运动学原有的头文件路径。
 * @board 底盘板
 * @vehicle_function 使已有调用方继续使用底盘运动控制与四轮运动学接口。
 * @inputs 原工程中的 include 指令。
 * @outputs 底盘运动控制与四轮运动学的公开声明。
 * @dependencies chassis_motion_control.h、chassis_wheel_kinematics.h。
 * @note 重构说明：本文件行为与原代码保持一致。该头文件只转发声明。
 */
#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

#include "chassis_motion_control.h"
#include "chassis_wheel_kinematics.h"

#endif

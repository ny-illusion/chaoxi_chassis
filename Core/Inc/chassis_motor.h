/**
 * @file chassis_motor.h
 * @brief 保留底盘电机驱动原有的头文件路径。
 * @board 底盘板
 * @vehicle_function 使已有调用方继续使用底盘电机驱动接口。
 * @inputs 原工程中的 include 指令。
 * @outputs 底盘电机驱动的公开声明。
 * @dependencies chassis_motor_driver.h、chassis_motion.h。
 * @note 重构说明：本文件行为与原代码保持一致。该头文件只转发声明。
 */
#ifndef CHASSIS_MOTOR_H
#define CHASSIS_MOTOR_H

#include "chassis_motor_driver.h"
#include "chassis_motion.h"

#endif

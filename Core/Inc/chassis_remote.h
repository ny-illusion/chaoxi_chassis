/**
 * @file chassis_remote.h
 * @brief 原有包含路径的兼容入口
 * @board 底盘板
 * @vehicle_function 向原有调用方提供遥控快照与驾驶输入接口
 * @inputs 使用旧文件名的已有调用方
 * @outputs 原有函数声明、类型和宏
 * @dependencies chassis_remote_communication.h、chassis_remote_input.h、chassis_motion.h
 * @note 重构说明：本文件行为与原代码保持一致。新模块可直接包含对应功能头。
 */

#ifndef CHASSIS_REMOTE_H
#define CHASSIS_REMOTE_H

#include "chassis_remote_communication.h"
#include "chassis_remote_input.h"
#include "chassis_motion.h"

#endif

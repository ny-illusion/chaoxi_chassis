/**
 * @file chassis_remote_input.h
 * @brief 声明从遥控快照计算驾驶输入的接口
 * @board 底盘板
 * @vehicle_function 把摇杆、档位和使能开关转换为底盘控制输入
 * @inputs 遥控快照、底盘本地毫秒计数
 * @outputs ChassisInput 和现有遥控调试变量
 * @dependencies chassis_remote_snapshot.h、chassis_motion_types.h
 * @note 重构说明：本文件行为与原代码保持一致。由底盘控制环每周期调用，通道顺序、失联门限及死区沿用原实现。
 */

#ifndef CHASSIS_REMOTE_INPUT_H
#define CHASSIS_REMOTE_INPUT_H

#include "chassis_remote_snapshot.h"
#include "chassis_motion_types.h"

ChassisInput ChassisRemote_Read(const RC_SBUS_Snapshot *rc, uint32_t now_ms);  // 项目函数，用板间遥控快照计算平移、yaw、SC 档位及 SD 使能

#endif

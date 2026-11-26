/**
 * @file chassis_remote_snapshot.h
 * @brief 定义底盘从板间 CAN 帧取得的遥控快照。
 * @board 底盘板
 * @vehicle_function 为底盘控制提供云台转发的驾驶输入。
 * @inputs CAN 帧中的五路通道、有效位以及底盘本机接收时间。
 * @outputs 控制周期读取的通道、计数、时间戳和失效状态。
 * @dependencies stdint.h
 * @note 通道下标沿用云台 SBUS 编号，仅更新板间协议传输的通道。
 */
#ifndef CHASSIS_REMOTE_SNAPSHOT_H
#define CHASSIS_REMOTE_SNAPSHOT_H

#include <stdint.h>

#define RC_SBUS_CHANNEL_COUNT 16U

typedef struct
{
    uint16_t channels[RC_SBUS_CHANNEL_COUNT];  // 按 SBUS 通道号索引，每路原始值占 11 位
    uint32_t last_frame_ms;  // 生成该快照的板子最近收到帧的时间，单位 ms
    uint32_t frame_count;  // 已接收的格式正确的帧数，包括遥控标记失控的帧
    uint8_t valid;  // 已经收到格式正确的帧，仍需结合 failsafe 与帧龄判断
    uint8_t failsafe;  // 源遥控丢帧、失控或被发送端判为过期
} RC_SBUS_Snapshot;
#endif

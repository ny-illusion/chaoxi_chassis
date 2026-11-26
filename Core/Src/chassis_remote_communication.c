/**
 * @file chassis_remote_communication.c
 * @brief 板间 CAN 遥控帧解包与快照发布
 * @board 底盘板
 * @vehicle_function 为底盘控制周期提供来自云台板的操纵输入
 * @inputs 来自接收中断的八字节遥控载荷和本机时间
 * @outputs 带通道、有效位、帧计数及时间戳的快照
 * @dependencies chassis_remote_snapshot.h 的快照类型、HAL 滴答和 CMSIS 中断屏蔽接口
 * @note 重构说明：本文件行为与原代码保持一致。在 CAN 接收路径发布，在 TIM6 中复制整帧。
 */

#include "chassis_remote_communication.h"
#include "main.h"

static volatile RC_SBUS_Snapshot received_remote_snapshot;  // CAN 接收中断发布，TIM6 临界区读取

void ChassisRemote_ReceiveCAN(const uint8_t data[8])  // 项目函数，解包云台 0x301，并记录底盘本机接收时间
{
    static const uint8_t sbus_channel_order[] = {0, 2, 3, 4, 5};  // 按 yaw、前后、横移、SC、SD 的顺序传五路原始值
    uint64_t packed_channels = 0;
    for (unsigned i = 0; i < 7; ++i)
    {
        packed_channels |= (uint64_t)data[i] << (8U * i);  // 将低字节在前的 7 个字节还原为 55 位通道数据
    }
    for (unsigned i = 0; i < sizeof(sbus_channel_order); ++i)
    {
        received_remote_snapshot.channels[sbus_channel_order[i]] = (packed_channels >> (11U * i)) & 0x7FFU;  // 按约定的通道号写回 11 位原始值
    }
    received_remote_snapshot.valid = 1U;  // 底盘已收到至少一帧格式正确的 0x301
    received_remote_snapshot.failsafe = (data[7] & 0x01U) == 0U;  // 只取 bit0，预留位不参与遥控有效性判断
    received_remote_snapshot.last_frame_ms = HAL_GetTick();  // HAL 毫秒计时，用底盘本地时间判断板间通信超时
    ++received_remote_snapshot.frame_count;  // Watch 可用此值区分板间断线与源遥控失联
}

void ChassisRemote_CopyData(RC_SBUS_Snapshot *snapshot)  // 项目函数，为 TIM6 复制同一帧的遥控通道和状态
{
    uint32_t irq_state = __get_PRIMASK();  // CMSIS 内核接口，保存当前中断屏蔽状态
    __disable_irq();  // CMSIS 内核接口，暂时屏蔽可屏蔽中断
    *snapshot = received_remote_snapshot;
    __set_PRIMASK(irq_state);  // CMSIS 内核接口，恢复进入临界区前的中断状态
}

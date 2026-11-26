/**
 * @file chassis_motion_types.h
 * @brief 定义底盘运动模式、反馈、输入和输出数据类型。
 * @board 底盘板
 * @vehicle_function 让遥控、运动控制和四轮分配使用相同的模式编号、单位和车轮数量。
 * @inputs 调用方使用的速度分量、相对角、时间戳与模式选择。
 * @outputs ChassisInput、ChassisCommand、ChassisGimbalFeedback 以及模式枚举。
 * @dependencies 标准整数类型 stdint.h。
 * @note 重构说明：本文件行为与原代码保持一致。保留原有字段顺序、枚举值和轮数。
 */
#ifndef CHASSIS_MOTION_TYPES_H
#define CHASSIS_MOTION_TYPES_H

#include <stdint.h>

#define CHASSIS_WHEEL_COUNT 4U

typedef enum
{
    CHASSIS_SPIN_OFF = 0,
    CHASSIS_SPIN_LOW,
    CHASSIS_SPIN_HIGH
} ChassisSpinMode;

typedef enum
{
    CHASSIS_ZERO_FORCE = 0,
    CHASSIS_MANUAL,
    CHASSIS_FOLLOW_GIMBAL,
    CHASSIS_ROTATE_LOW,
    CHASSIS_ROTATE_HIGH
} ChassisMode;

typedef struct
{
    float relative_yaw_rad;  // 云台相对车头的 yaw 角，单位 rad
    uint32_t last_update_ms;  // 本板最近一次更新时间，单位 ms
    uint8_t valid;  // 反馈生产者已发布测量值，使用时另查时间戳
} ChassisGimbalFeedback;

typedef struct
{
    float vx_rpm;  // 向前为正的平移轮速分量，单位 rpm
    float vy_rpm;  // 向左为正的平移轮速分量，单位 rpm
    ChassisSpinMode spin;
    uint8_t enabled;
    uint8_t use_gimbal;  // 允许使用 yaw 编码器角度进行跟随、稳向和坐标变换
    float yaw_target_rpm;  // 右杆给定的 yaw 转速，采用电机正方向
    float gimbal_hold_error_rad;  // 松杆时锁定航向与当前对地航向之差，单位 rad
} ChassisInput;

typedef struct
{
    float vx_rpm;  // 向前为正的平移轮速分量，单位 rpm
    float vy_rpm;  // 向左为正的平移轮速分量，单位 rpm
    float wz_rpm;  // 混控中的旋转轮速分量，单位 rpm
    ChassisMode mode;
} ChassisCommand;

#endif

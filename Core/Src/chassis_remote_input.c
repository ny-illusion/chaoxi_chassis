/**
 * @file chassis_remote_input.c
 * @brief 遥控通道到驾驶输入的换算
 * @board 底盘板
 * @vehicle_function 把摇杆、档位和使能开关转换为底盘控制输入
 * @inputs 遥控快照、底盘本地毫秒计数
 * @outputs ChassisInput 和现有遥控调试变量
 * @dependencies chassis_remote_input.h、chassis_motion_control.h
 * @note 重构说明：本文件行为与原代码保持一致。由底盘控制环每周期调用，通道顺序、失联门限及死区沿用原实现。
 */

#include "chassis_remote_input.h"
#include "chassis_motion_control.h"

#define REMOTE_STICK_CENTER 1024.0f  // SBUS 摇杆回中原始值
#define REMOTE_STICK_HALF_RANGE 671.0f  // 中位到满杆的标定差值
#define REMOTE_RC_ENABLE_THRESHOLD 1400U  // SD 原始值超过此阈值才允许输出
#define REMOTE_RC_LOST_MS 100U  // 底盘板间遥控帧的超时门限，单位 ms

#define REMOTE_TARGET_RPM_MAX 8000.0f
#define REMOTE_SC_LOW_THRESHOLD 690.0f
#define REMOTE_SC_VALID_MIN 300U  // 低于此值的 SC 输入按异常处理
#define REMOTE_SC_HIGH_THRESHOLD 1360.0f
#define YAW_PREVIEW_MAX_RPM 50.0f  // 右杆满量程对应的 yaw 速度，单位 rpm

uint8_t rc_mode = CHASSIS_SPIN_OFF;
float rc_vx;
float rc_vy;
static volatile float rc_yaw_stick;
static volatile float yaw_preview_rpm;
uint8_t rc_enable;
uint8_t rc_lost = 1;

static float normalize_stick(uint16_t raw_channel)  // 项目函数，将 SBUS 摇杆归一化至 ±1，并应用中位死区
{
    float normalized_stick = (REMOTE_STICK_CENTER - (float)raw_channel) / REMOTE_STICK_HALF_RANGE;
    if (normalized_stick > 1.0f) normalized_stick = 1.0f;
    if (normalized_stick < -1.0f) normalized_stick = -1.0f;
    if (normalized_stick > -0.05f && normalized_stick < 0.05f) normalized_stick = 0.0f;
    return normalized_stick;
}

ChassisInput ChassisRemote_Read(const RC_SBUS_Snapshot *rc, uint32_t now_ms)  // 项目函数，用板间遥控快照计算平移、yaw、SC 档位及 SD 使能
{
    rc_vx = normalize_stick(rc->channels[2]);  // 项目函数，将通道 2 换算为前后摇杆比例
    rc_vy = normalize_stick(rc->channels[3]);  // 项目函数，将通道 3 换算为横移摇杆比例
    rc_yaw_stick = normalize_stick(rc->channels[0]);  // 项目函数，将右杆通道 0 换算为 yaw 转速比例
    rc_enable = (uint8_t)(rc->channels[5] > REMOTE_RC_ENABLE_THRESHOLD);  // SD 超过阈值表示操作者允许输出
    rc_lost = (uint8_t)(!rc->valid || rc->failsafe ||  // 检查板间帧、源遥控状态与本地接收时间
                        (uint32_t)(now_ms - rc->last_frame_ms) > REMOTE_RC_LOST_MS);
    if (rc->channels[4] < REMOTE_SC_VALID_MIN)
        rc_mode = CHASSIS_SPIN_OFF;
    else if ((float)rc->channels[4] < REMOTE_SC_LOW_THRESHOLD)
        rc_mode = CHASSIS_SPIN_HIGH;
    else if ((float)rc->channels[4] < REMOTE_SC_HIGH_THRESHOLD)
        rc_mode = CHASSIS_SPIN_LOW;
    else
        rc_mode = CHASSIS_SPIN_OFF;
    ChassisInput input = {
        .vx_rpm = rc_vx * REMOTE_TARGET_RPM_MAX,
        .vy_rpm = rc_vy * REMOTE_TARGET_RPM_MAX,
        .spin = (ChassisSpinMode)rc_mode,
        .enabled = (uint8_t)(rc_enable && !rc_lost),  // 遥控链路有效且 SD 开启时才允许底盘输出
        .use_gimbal = (uint8_t)(CHASSIS_GIMBAL_INSTALLED &&
                                rc->channels[4] >= REMOTE_SC_VALID_MIN)};
    input.yaw_target_rpm = input.enabled
                               ? rc_yaw_stick * YAW_PREVIEW_MAX_RPM
                               : 0.0f;
    yaw_preview_rpm = input.yaw_target_rpm;
    return input;
}

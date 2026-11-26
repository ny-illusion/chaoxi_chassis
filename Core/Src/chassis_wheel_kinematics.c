/**
 * @file chassis_wheel_kinematics.c
 * @brief 将底盘平移与旋转分量分配到四个麦克纳姆轮。
 * @board 底盘板
 * @vehicle_function 按安装方向和转速上限生成每个轮电机的目标转速。
 * @inputs 底盘运动命令和单轮转速上限，单位 rpm。
 * @outputs 按左前、右前、右后、左后顺序排列的四个目标转速。
 * @dependencies 底盘运动类型与 C 数学库。
 * @note 重构说明：本文件行为与原代码保持一致。由底盘控制循环在更新轮电机前调用。
 */
#include "chassis_wheel_kinematics.h"
#include <math.h>

void ChassisControl_Mix(  // 项目函数，分配四轮目标转速，超限时四轮同比缩放
    const ChassisCommand *command,
    float limit_rpm,
    float targets[CHASSIS_WHEEL_COUNT])  // 项目函数，分配四轮目标转速，超限时四轮同比缩放
{
    static const int8_t wheel_mount_sign[CHASSIS_WHEEL_COUNT] = {+1, -1, -1, +1};  // 轮序为左前、右前、右后、左后，符号补偿电机安装方向
    static const int8_t lateral_sign[CHASSIS_WHEEL_COUNT] = {-1, +1, -1, +1};
    static const int8_t rotation_sign[CHASSIS_WHEEL_COUNT] = {-1, +1, +1, -1};
    float peak_rpm = 0.0f;
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i) targets[i] = 0.0f;
    if (command->mode == CHASSIS_ZERO_FORCE) return;
    if (!isfinite(limit_rpm) || limit_rpm <= 0.0f) return;  // C 数学库，检查数值是否有限
    if (!isfinite(command->vx_rpm)) return;  // C 数学库，检查数值是否有限
    if (!isfinite(command->vy_rpm)) return;  // C 数学库，检查数值是否有限
    if (!isfinite(command->wz_rpm)) return;  // C 数学库，检查数值是否有限
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        targets[i] = command->vx_rpm + lateral_sign[i] * command->vy_rpm + rotation_sign[i] * command->wz_rpm;
        targets[i] *= wheel_mount_sign[i];
        if (fabsf(targets[i]) > peak_rpm) peak_rpm = fabsf(targets[i]);  // C 数学库，取绝对值
    }
    if (peak_rpm <= limit_rpm) return;

    float speed_scale = limit_rpm / peak_rpm;  // 四轮同比缩小，保持平移与旋转的比例
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i) targets[i] *= speed_scale;
}

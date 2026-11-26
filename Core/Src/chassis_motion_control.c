/**
 * @file chassis_motion_control.c
 * @brief 根据遥控模式和反馈计算车身坐标系下的运动命令。
 * @board 底盘板
 * @vehicle_function 将遥控平移与旋转请求转换为四轮运动学使用的输入。
 * @inputs ChassisInput、ChassisGimbalFeedback 和本机毫秒计数。
 * @outputs 带模式与三轴轮速分量的 ChassisCommand、折回到单圈的角度。
 * @dependencies 底盘运动类型与 C 数学库。
 * @note 重构说明：本文件行为与原代码保持一致。每次计算从零力命令开始，反馈和模式检查沿用原返回路径。
 */
#include "chassis_motion_control.h"
#include <math.h>

#define CHASSIS_PI 3.14159265f
#define CHASSIS_TWO_PI 6.28318531f

static float ClampSymmetric(float value, float limit)  // 项目函数，在正负 limit 之间限制指令
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

float ChassisControl_WrapAngle(float angle)  // 项目函数，将弧度折回 −π 至 +π
{
    angle = fmodf(angle, CHASSIS_TWO_PI);  // C 数学库，求浮点余数
    if (angle > CHASSIS_PI) angle -= CHASSIS_TWO_PI;
    if (angle < -CHASSIS_PI) angle += CHASSIS_TWO_PI;
    return angle;
}

void ChassisControl_Step(  // 项目函数，根据使能、SC 和云台方向计算底盘的三轴命令
    const ChassisInput *input,
    const ChassisGimbalFeedback *feedback,
    uint32_t now_ms,
    ChassisCommand *command)  // 项目函数，根据使能、SC 和云台方向计算底盘的三轴命令
{
    float yaw = 0.0f;
    *command = (ChassisCommand){0};
    if (!input->enabled) return;
    if (!isfinite(input->vx_rpm)) return;  // C 数学库，检查数值是否有限
    if (!isfinite(input->vy_rpm)) return;  // C 数学库，检查数值是否有限

    if (input->use_gimbal)
    {
        if (!feedback->valid) return;
        if (!isfinite(feedback->relative_yaw_rad)) return;  // C 数学库，检查数值是否有限
        if ((uint32_t)(now_ms - feedback->last_update_ms) > CHASSIS_GIMBAL_TIMEOUT_MS)
            return;
        yaw = ChassisControl_WrapAngle(feedback->relative_yaw_rad);  // 项目函数，将弧度折回 −π 至 +π
    }

    switch (input->spin)
    {
    case CHASSIS_SPIN_LOW:
        command->mode = CHASSIS_ROTATE_LOW;
        command->wz_rpm = CHASSIS_SPIN_RPM_LOW;
        break;
    case CHASSIS_SPIN_HIGH:
        command->mode = CHASSIS_ROTATE_HIGH;
        command->wz_rpm = CHASSIS_SPIN_RPM_HIGH;
        break;
    case CHASSIS_SPIN_OFF:

        if (input->use_gimbal)
        {
            const uint8_t stick_centered = input->yaw_target_rpm == 0.0f;
            if (stick_centered && !isfinite(input->gimbal_hold_error_rad)) return;  // C 数学库，检查数值是否有限
            const float follow_error = stick_centered  // 回中跟随锁定的航向，推杆时跟随编码器相对角
                                           ? ChassisControl_WrapAngle(yaw + input->gimbal_hold_error_rad)  // 项目函数，将弧度折回 −π 至 +π
                                           : yaw;
            command->mode = CHASSIS_FOLLOW_GIMBAL;
            const float outside_deadband = fabsf(follow_error) - CHASSIS_FOLLOW_DEADBAND_RAD;  // 减去跟随死区，小误差时停止底盘旋转
            if (outside_deadband > 0.0f)
            {
                const float turn_error = stick_centered
                                             ? copysignf(outside_deadband, follow_error)  // C 数学库，为幅值赋予转向符号
                                             : follow_error;
                command->wz_rpm = ClampSymmetric(CHASSIS_FOLLOW_KP * turn_error, CHASSIS_FOLLOW_RPM_MAX);  // 项目函数，在正负 limit 之间限制指令
            }
        }
        else
        {
            command->mode = CHASSIS_MANUAL;
        }
        break;
    default:
        return;
    }

    const float cos_yaw = cosf(yaw);  // C 数学库，弧度角的余弦
    const float sin_yaw = sinf(yaw);  // C 数学库，弧度角的正弦
    command->vx_rpm = input->vx_rpm * cos_yaw - input->vy_rpm * sin_yaw;  // 按云台相对角把平移指令投影到底盘坐标系
    command->vy_rpm = input->vx_rpm * sin_yaw + input->vy_rpm * cos_yaw;
}

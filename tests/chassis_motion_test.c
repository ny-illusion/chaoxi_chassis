#include "chassis_motion.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PI_F 3.14159265f

static void Near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.02f);
}

static void ZeroForce(const ChassisCommand *command)
{
    assert(command->mode == CHASSIS_ZERO_FORCE);
    Near(command->vx_rpm, 0.0f);
    Near(command->vy_rpm, 0.0f);
    Near(command->wz_rpm, 0.0f);
}

int main(void)
{
    ChassisInput input = {
        /* vx_rpm、vy_rpm 分别给出前后和横移的轮速分量。
         * spin 选择自旋档位，enabled 决定能否输出，use_gimbal 决定是否使用云台反馈。 */
        .vx_rpm = 1000.0f, .vy_rpm = 200.0f,
        .spin = CHASSIS_SPIN_OFF, .enabled = 1, .use_gimbal = 0
    };
    ChassisGimbalFeedback feedback = {0};
    /* feedback.relative_yaw_rad 是相对角（弧度），last_update_ms 是更新时间（毫秒），valid 是有效标志。 */
    ChassisCommand command;
    /* command.vx_rpm、vy_rpm、wz_rpm 保存算出的三个轮速分量，mode 保存算出的工作模式。 */
    float wheels[CHASSIS_WHEEL_COUNT];

    /* 无云台且关闭自旋时，只控制平移。 */
    ChassisControl_Step(&input, &feedback, 1000, &command);
    assert(command.mode == CHASSIS_MANUAL);
    Near(command.vx_rpm, 1000.0f);
    Near(command.vy_rpm, 200.0f);
    Near(command.wz_rpm, 0.0f);

    /* 两个自旋档位取固定速度。 */
    input.spin = CHASSIS_SPIN_LOW;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    assert(command.mode == CHASSIS_ROTATE_LOW);
    Near(command.wz_rpm, CHASSIS_SPIN_RPM_LOW);
    input.spin = CHASSIS_SPIN_HIGH;
    for (uint32_t t = 0; t < 10000; ++t)
        ChassisControl_Step(&input, &feedback, t, &command);
    assert(command.mode == CHASSIS_ROTATE_HIGH);
    Near(command.wz_rpm, CHASSIS_SPIN_RPM_HIGH);
    Near(command.vx_rpm, 1000.0f);
    input.spin = CHASSIS_SPIN_OFF;
    ChassisControl_Step(&input, &feedback, 10000, &command);
    Near(command.wz_rpm, 0.0f); /* 退出高速档后旋转指令归零。 */
    Near(command.vx_rpm, 1000.0f);
    Near(command.vy_rpm, 200.0f);

    /* 有云台：未接反馈不能运动；新鲜反馈允许跟随，过期反馈零力。 */
    input.use_gimbal = 1;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    ZeroForce(&command);
    feedback = (ChassisGimbalFeedback){.valid = 1, .last_update_ms = 1000};
    ChassisControl_Step(&input, &feedback, 1100, &command);
    assert(command.mode == CHASSIS_FOLLOW_GIMBAL);
    Near(command.wz_rpm, 0.0f);
    ChassisControl_Step(&input, &feedback, 1101, &command);
    ZeroForce(&command);
    input.spin = CHASSIS_SPIN_HIGH;
    ChassisControl_Step(&input, &feedback, 1101, &command);
    ZeroForce(&command);
    input.spin = CHASSIS_SPIN_OFF;

    /* 正负 90 度验证坐标方向与跟随方向，而不是只验证公式自身。 */
    input.vy_rpm = 0.0f;
    feedback.relative_yaw_rad = PI_F / 2.0f;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    Near(command.vx_rpm, 0.0f);
    Near(command.vy_rpm, 1000.0f);
    Near(command.wz_rpm, CHASSIS_FOLLOW_RPM_MAX);
    feedback.relative_yaw_rad = -PI_F / 2.0f;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    Near(command.vx_rpm, 0.0f);
    Near(command.vy_rpm, -1000.0f);
    Near(command.wz_rpm, -CHASSIS_FOLLOW_RPM_MAX);

    /* 跟随死区、比例段、跨 360 度回绕。 */
    feedback.relative_yaw_rad = 0.01f;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    Near(command.wz_rpm, 0.0f);
    const float follow_yaw = CHASSIS_FOLLOW_DEADBAND_RAD + 0.05f;
    const float follow_rpm = CHASSIS_FOLLOW_KP * 0.05f;
    feedback.relative_yaw_rad = follow_yaw;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    Near(command.wz_rpm, follow_rpm);
    feedback.relative_yaw_rad = 2.0f * PI_F - follow_yaw;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    Near(command.wz_rpm, -follow_rpm);

    /* 虚拟云台朝向固定，底盘正转 90 度：相对角 -90 度，前进变右移。
     * 这是电脑上的输入场景，不将虚拟角度接到实际电机控制中。
     */
    input.spin = CHASSIS_SPIN_HIGH;
    feedback.relative_yaw_rad = -PI_F / 2.0f;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    Near(command.vx_rpm, 0.0f);
    Near(command.vy_rpm, -1000.0f);
    Near(command.wz_rpm, CHASSIS_SPIN_RPM_HIGH);

    /* tick 溢出仍正确判定新鲜度。 */
    feedback.last_update_ms = UINT32_MAX - 49U;
    ChassisControl_Step(&input, &feedback, 50, &command);
    assert(command.mode == CHASSIS_ROTATE_HIGH);
    ChassisControl_Step(&input, &feedback, 51, &command);
    ZeroForce(&command);

    feedback.last_update_ms = 1000;
    feedback.relative_yaw_rad = NAN;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    ZeroForce(&command);
    feedback.relative_yaw_rad = INFINITY;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    ZeroForce(&command);
    feedback.relative_yaw_rad = 0.0f;
    input.enabled = 0;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    ZeroForce(&command);
    input.enabled = 1;
    input.spin = (ChassisSpinMode)99;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    ZeroForce(&command);
    input.spin = CHASSIS_SPIN_OFF;
    input.vx_rpm = NAN;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    ZeroForce(&command);

    /* 四轮方向沿用当前底盘实测表；混合超限时四轮同比缩放。 */
    command = (ChassisCommand){.vx_rpm = 1000, .mode = CHASSIS_MANUAL};
    ChassisControl_Mix(&command, 8000, wheels);
    Near(wheels[0], 1000); Near(wheels[1], -1000);
    Near(wheels[2], -1000); Near(wheels[3], 1000);
    command = (ChassisCommand){.vy_rpm = 1000, .mode = CHASSIS_MANUAL};
    ChassisControl_Mix(&command, 8000, wheels);
    Near(wheels[0], -1000); Near(wheels[1], -1000);
    Near(wheels[2], 1000); Near(wheels[3], 1000);
    command = (ChassisCommand){.wz_rpm = 1000, .mode = CHASSIS_ROTATE_LOW};
    ChassisControl_Mix(&command, 8000, wheels);
    Near(wheels[0], -1000); Near(wheels[1], -1000);
    Near(wheels[2], -1000); Near(wheels[3], -1000);
    command = (ChassisCommand){.vx_rpm = 8000, .vy_rpm = 8000,
                               .wz_rpm = 3000, .mode = CHASSIS_ROTATE_HIGH};
    ChassisControl_Mix(&command, 8000, wheels);
    const float expected[4] = {-3000, -19000, -3000, 13000};
    for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        Near(wheels[i], expected[i] * 8000.0f / 19000.0f);
        assert(fabsf(wheels[i]) <= 8000.0f);
    }
    command.mode = CHASSIS_ZERO_FORCE;
    ChassisControl_Mix(&command, 8000, wheels);
    for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i) Near(wheels[i], 0);
    command.mode = CHASSIS_MANUAL;
    ChassisControl_Mix(&command, 0, wheels);
    for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i) Near(wheels[i], 0);

    puts("PASS: chassis modes, yaw transform, follow, timeout, disable and wheel scaling");
    return 0;
}

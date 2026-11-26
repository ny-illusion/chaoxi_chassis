/**
 * @file chassis_motion_test.c
 * @brief 在主机上检查运动模式和四轮分配结果。
 * @board 底盘板
 * @vehicle_function 通过已知输入核对车身运动指令、反馈边界和轮速限制。
 * @inputs 测试构造的遥控输入、角度反馈、模拟时间和轮速上限。
 * @outputs 断言结果和测试通过信息。
 * @dependencies 底盘运动控制、四轮运动学、标准断言、数学库与标准输出。
 * @note 重构说明：本文件行为与原代码保持一致。由 run_chassis_tests.cmd 使用主机 GCC 编译执行。
 */
#include "chassis_motion_control.h"
#include "chassis_wheel_kinematics.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PI_F 3.14159265f

static void AssertNear(float actual, float expected)  // 项目函数，断言实际值与期望值相差小于 0.02
{
    assert(fabsf(actual - expected) < 0.02f);  // C 数学库，取绝对值
}

static void AssertZeroForce(const ChassisCommand *command)  // 项目函数，检查底盘处于零力模式且三个速度分量均为零
{
    assert(command->mode == CHASSIS_ZERO_FORCE);
    AssertNear(command->vx_rpm, 0.0f);
    AssertNear(command->vy_rpm, 0.0f);
    AssertNear(command->wz_rpm, 0.0f);
}

int main(void)  // 项目测试入口，运行底盘运动断言
{
    ChassisInput input = {
        .vx_rpm = 1000.0f, .vy_rpm = 200.0f, .spin = CHASSIS_SPIN_OFF, .enabled = 1, .use_gimbal = 0};
    ChassisGimbalFeedback feedback = {0};
    ChassisCommand command;
    float wheels[CHASSIS_WHEEL_COUNT];
    ChassisControl_Step(&input, &feedback, 1000, &command);
    assert(command.mode == CHASSIS_MANUAL);
    AssertNear(command.vx_rpm, 1000.0f);
    AssertNear(command.vy_rpm, 200.0f);
    AssertNear(command.wz_rpm, 0.0f);
    input.spin = CHASSIS_SPIN_LOW;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    assert(command.mode == CHASSIS_ROTATE_LOW);
    AssertNear(command.wz_rpm, CHASSIS_SPIN_RPM_LOW);
    input.spin = CHASSIS_SPIN_HIGH;
    for (uint32_t t = 0; t < 10000; ++t)
        ChassisControl_Step(&input, &feedback, t, &command);
    assert(command.mode == CHASSIS_ROTATE_HIGH);
    AssertNear(command.wz_rpm, CHASSIS_SPIN_RPM_HIGH);
    AssertNear(command.vx_rpm, 1000.0f);
    input.spin = CHASSIS_SPIN_OFF;
    ChassisControl_Step(&input, &feedback, 10000, &command);
    AssertNear(command.wz_rpm, 0.0f);
    AssertNear(command.vx_rpm, 1000.0f);
    AssertNear(command.vy_rpm, 200.0f);

    input.use_gimbal = 1;  // 进入云台跟随，先检查反馈缺失，再检查 100 ms 时效边界
    ChassisControl_Step(&input, &feedback, 1000, &command);
    AssertZeroForce(&command);
    feedback = (ChassisGimbalFeedback){.valid = 1, .last_update_ms = 1000};
    ChassisControl_Step(&input, &feedback, 1100, &command);
    assert(command.mode == CHASSIS_FOLLOW_GIMBAL);
    AssertNear(command.wz_rpm, 0.0f);
    ChassisControl_Step(&input, &feedback, 1101, &command);
    AssertZeroForce(&command);
    input.spin = CHASSIS_SPIN_HIGH;
    ChassisControl_Step(&input, &feedback, 1101, &command);
    AssertZeroForce(&command);
    input.spin = CHASSIS_SPIN_OFF;

    input.vy_rpm = 0.0f;  // 分别验证 yaw 为正负 90° 时的底盘平移方向
    feedback.relative_yaw_rad = PI_F / 2.0f;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    AssertNear(command.vx_rpm, 0.0f);
    AssertNear(command.vy_rpm, 1000.0f);
    AssertNear(command.wz_rpm, CHASSIS_FOLLOW_RPM_MAX);
    feedback.relative_yaw_rad = -PI_F / 2.0f;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    AssertNear(command.vx_rpm, 0.0f);
    AssertNear(command.vy_rpm, -1000.0f);
    AssertNear(command.wz_rpm, -CHASSIS_FOLLOW_RPM_MAX);

    feedback.relative_yaw_rad = 0.01f;  // 先验证跟随死区，再验证比例区和整圈回绕
    ChassisControl_Step(&input, &feedback, 1000, &command);
    AssertNear(command.wz_rpm, 0.0f);
    const float follow_yaw = CHASSIS_FOLLOW_DEADBAND_RAD + 0.05f;
    const float follow_rpm = CHASSIS_FOLLOW_KP * 0.05f;
    feedback.relative_yaw_rad = follow_yaw;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    AssertNear(command.wz_rpm, follow_rpm);
    feedback.relative_yaw_rad = 2.0f * PI_F - follow_yaw;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    AssertNear(command.wz_rpm, -follow_rpm);
    input.spin = CHASSIS_SPIN_HIGH;
    feedback.relative_yaw_rad = -PI_F / 2.0f;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    AssertNear(command.vx_rpm, 0.0f);
    AssertNear(command.vy_rpm, -1000.0f);
    AssertNear(command.wz_rpm, CHASSIS_SPIN_RPM_HIGH);

    feedback.last_update_ms = UINT32_MAX - 49U;  // 毫秒计数回绕时仍按实际间隔判断超时
    ChassisControl_Step(&input, &feedback, 50, &command);
    assert(command.mode == CHASSIS_ROTATE_HIGH);
    ChassisControl_Step(&input, &feedback, 51, &command);
    AssertZeroForce(&command);
    feedback.last_update_ms = 1000;

    feedback.relative_yaw_rad = NAN;  // 非有限的反馈角触发零力输出
    ChassisControl_Step(&input, &feedback, 1000, &command);
    AssertZeroForce(&command);
    feedback.relative_yaw_rad = INFINITY;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    AssertZeroForce(&command);
    feedback.relative_yaw_rad = 0.0f;
    input.enabled = 0;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    AssertZeroForce(&command);
    input.enabled = 1;
    input.spin = (ChassisSpinMode)99;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    AssertZeroForce(&command);
    input.spin = CHASSIS_SPIN_OFF;
    input.vx_rpm = NAN;
    ChassisControl_Step(&input, &feedback, 1000, &command);
    AssertZeroForce(&command);

    command = (ChassisCommand){.vx_rpm = 1000, .mode = CHASSIS_MANUAL};  // 依轮序验证前后、横移和旋转的电机符号
    ChassisControl_Mix(&command, 8000, wheels);
    AssertNear(wheels[0], 1000);
    AssertNear(wheels[1], -1000);
    AssertNear(wheels[2], -1000);
    AssertNear(wheels[3], 1000);
    command = (ChassisCommand){.vy_rpm = 1000, .mode = CHASSIS_MANUAL};
    ChassisControl_Mix(&command, 8000, wheels);
    AssertNear(wheels[0], -1000);
    AssertNear(wheels[1], -1000);
    AssertNear(wheels[2], 1000);
    AssertNear(wheels[3], 1000);
    command = (ChassisCommand){.wz_rpm = 1000, .mode = CHASSIS_ROTATE_LOW};
    ChassisControl_Mix(&command, 8000, wheels);
    AssertNear(wheels[0], -1000);
    AssertNear(wheels[1], -1000);
    AssertNear(wheels[2], -1000);
    AssertNear(wheels[3], -1000);
    command = (ChassisCommand){.vx_rpm = 8000, .vy_rpm = 8000, .wz_rpm = 3000, .mode = CHASSIS_ROTATE_HIGH};
    ChassisControl_Mix(&command, 8000, wheels);
    const float expected[4] = {-3000, -19000, -3000, 13000};
    for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        AssertNear(wheels[i], expected[i] * 8000.0f / 19000.0f);
        assert(fabsf(wheels[i]) <= 8000.0f);  // C 数学库，取绝对值
    }
    command.mode = CHASSIS_ZERO_FORCE;
    ChassisControl_Mix(&command, 8000, wheels);
    for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i) AssertNear(wheels[i], 0);
    command.mode = CHASSIS_MANUAL;
    ChassisControl_Mix(&command, 0, wheels);
    for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i) AssertNear(wheels[i], 0);
    puts("PASS: chassis modes, yaw transform, follow, timeout, disable and wheel scaling");
    return 0;
}

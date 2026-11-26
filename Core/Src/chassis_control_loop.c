/**
 * @file chassis_control_loop.c
 * @brief 协调底盘定时控制、IMU 零偏校准和反馈快照。
 * @board 底盘板
 * @vehicle_function 每个 TIM6 周期更新车轮和 yaw 电机命令，两次指示灯刷新至少间隔 100 ms。
 * @inputs 板间遥控快照、底盘角速度、yaw 编码器反馈和本机毫秒计数。
 * @outputs 底盘运动命令、四轮目标转速与 yaw 电机目标转速。
 * @dependencies 遥控输入、运动控制、四轮运动学、电机驱动、GPIO 与 CMSIS 中断接口。
 * @note 重构说明：本文件行为与原代码保持一致。反馈写入和定时读取沿用原有临界区。
 */
#include "chassis_control_loop.h"
#include "chassis_motion_control.h"
#include "chassis_wheel_kinematics.h"
#include "chassis_remote_communication.h"
#include "chassis_remote_input.h"
#include "chassis_motor_driver.h"
#include "gpio.h"
#include "main.h"
#include <math.h>

#define APP_ROTOR_RPM_LIMIT 8000.0f  // 混控后的单轮转速上限，单位 rpm
#define CHASSIS_CONTROL_PERIOD_S 0.001f  // TIM6 底盘周期，单位 s
#define APP_IMU_YAW_DIR 1.0f  // 使 IMU 的正方向与底盘逆时针 yaw 一致
#define APP_IMU_BIAS_SAMPLES 200U  // 上电静止校零所需的成功采样数
#define APP_YAW_FEEDBACK_AGE_MS 20U  // 稳向要求 IMU 和 yaw 反馈均在此时限内，单位 ms
#define APP_RPM_TO_RAD_S 0.104719755f  // 将 rpm 换算为 rad/s 后积分底盘航向
#define APP_YAW_HOLD_KP 12.0f  // 对地航向误差到转速的增益，单位 rpm/rad
#define APP_YAW_HOLD_MAX_RPM 15.0f  // 松杆航向修正的最大转速，单位 rpm

typedef struct
{
    float yaw_rpm;  // 底盘实测 yaw 角速度，单位 rpm
    uint32_t last_update_ms;  // 本板最近一次更新时间，单位 ms
    uint8_t valid;  // 反馈生产者已发布测量值，使用时另查时间戳
} ChassisGyroFeedback;
static volatile ChassisGyroFeedback gyro_feedback;
ChassisCommand chassis_command;
static volatile ChassisGimbalFeedback gimbal_feedback;

void ChassisApp_SetGimbalFeedback(float relative_yaw_rad)  // 项目函数，保存 yaw 编码器相对角和本机接收时间
{
    uint32_t saved_primask = __get_PRIMASK();  // CMSIS 内核接口，保存当前中断屏蔽状态
    __disable_irq();  // CMSIS 内核接口，暂时屏蔽可屏蔽中断
    gimbal_feedback.relative_yaw_rad = relative_yaw_rad;
    gimbal_feedback.last_update_ms = HAL_GetTick();  // HAL 库，读取本板的毫秒计数
    gimbal_feedback.valid = (uint8_t)isfinite(relative_yaw_rad);  // C 数学库，检查数值是否有限
    __set_PRIMASK(saved_primask);  // CMSIS 内核接口，恢复进入临界区前的中断状态
}

void ChassisApp_SetGyroZ(float gyro_z_dps)  // 项目函数，用静止样本校零，将底盘 Z 轴角速度转换为 rpm
{
    static uint32_t sample_count;
    static float bias_sum_dps;
    static float bias_dps;
    if (sample_count < APP_IMU_BIAS_SAMPLES)  // 静止时收集成功的 Z 轴样本，估算上电零偏
    {
        bias_sum_dps += gyro_z_dps;
        sample_count++;
        if (sample_count < APP_IMU_BIAS_SAMPLES) return;  // 校零完成前暂不发布角速度
        bias_dps = bias_sum_dps / (float)APP_IMU_BIAS_SAMPLES;
    }

    const float yaw_rpm = APP_IMU_YAW_DIR * (gyro_z_dps - bias_dps) / 6.0f;  // 扣除零偏并统一方向，6 °/s 换算为 1 rpm
    const uint32_t sample_ms = HAL_GetTick();  // HAL 库，读取本板的毫秒计数
    const uint32_t saved_primask = __get_PRIMASK();  // CMSIS 内核接口，保存当前中断屏蔽状态
    __disable_irq();  // CMSIS 内核接口，暂时屏蔽可屏蔽中断
    gyro_feedback.yaw_rpm = yaw_rpm;
    gyro_feedback.last_update_ms = sample_ms;
    gyro_feedback.valid = 1U;
    __set_PRIMASK(saved_primask);  // CMSIS 内核接口，恢复进入临界区前的中断状态
}

void ChassisApp_Init(void)  // 项目函数，清零四轮控制状态并初始化指示灯
{
    ChassisMotor_Init();  // 项目函数，复位四轮反馈、积分和电流指令
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);  // HAL 库，设置 GPIO 输出电平
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);  // HAL 库，设置 GPIO 输出电平
}

typedef struct
{
    float target_rpm;
    float hold_error_rad;
} GimbalYawCommand;

static GimbalYawCommand GimbalYawControl(float stick_target_rpm, float relative_yaw_rad,  // 项目函数，合成对地航向，计算 yaw 相对底盘的目标转速
                                         float chassis_yaw_rpm, uint8_t stabilize_yaw)  // 项目函数，合成对地航向，计算 yaw 相对底盘的目标转速
{
    static float chassis_yaw_rad;
    static float hold_yaw_rad;
    static uint8_t gimbal_holding;
    GimbalYawCommand command = {.target_rpm = stick_target_rpm};
    if (!stabilize_yaw)
    {
        chassis_yaw_rad = 0.0f;
        gimbal_holding = 0U;
        return command;
    }
    chassis_yaw_rad = ChassisControl_WrapAngle(  // 项目函数，将弧度折回 −π 至 +π
        chassis_yaw_rad + chassis_yaw_rpm * APP_RPM_TO_RAD_S * CHASSIS_CONTROL_PERIOD_S);
    const float gimbal_world_yaw_rad = chassis_yaw_rad + relative_yaw_rad;  // 底盘累计转角加编码器相对角，得到云台对地航向

    const uint8_t stick_centered = (stick_target_rpm == 0.0f);
    float world_target_rpm = GM6020_YAW_DIR * stick_target_rpm;
    if (stick_centered)
    {
        if (!gimbal_holding) hold_yaw_rad = gimbal_world_yaw_rad;  // 摇杆回中时记下对地航向
        command.hold_error_rad = ChassisControl_WrapAngle(hold_yaw_rad - gimbal_world_yaw_rad);  // 项目函数，将弧度折回 −π 至 +π
        const float hold_rpm = APP_YAW_HOLD_KP * command.hold_error_rad;
        world_target_rpm = fmaxf(-APP_YAW_HOLD_MAX_RPM, fminf(APP_YAW_HOLD_MAX_RPM, hold_rpm));  // C 数学库，将航向修正限制在 ±APP_YAW_HOLD_MAX_RPM
    }
    gimbal_holding = stick_centered;

    command.target_rpm = GM6020_YAW_DIR * (world_target_rpm - chassis_yaw_rpm);  // 扣除底盘实测角速度，得到 yaw 电机的相对目标转速
    return command;
}

void ChassisApp_Run1ms(void)  // 项目函数，读取遥控和反馈，计算并下发四轮与 yaw 命令
{
    static uint32_t last_led_update_ms;
    float wheel_targets[CHASSIS_WHEEL_COUNT];

    RC_SBUS_Snapshot remote_snapshot;
    ChassisGimbalFeedback feedback;
    ChassisRemote_CopyData(&remote_snapshot);  // 项目函数，为 TIM6 复制同一帧的遥控通道和状态
    uint32_t saved_primask = __get_PRIMASK();  // CMSIS 内核接口，保存当前中断屏蔽状态
    __disable_irq();  // CMSIS 内核接口，暂时屏蔽可屏蔽中断
    feedback.relative_yaw_rad = gimbal_feedback.relative_yaw_rad;
    feedback.last_update_ms = gimbal_feedback.last_update_ms;
    feedback.valid = gimbal_feedback.valid;
    const ChassisGyroFeedback gyro = {
        .yaw_rpm = gyro_feedback.yaw_rpm,
        .last_update_ms = gyro_feedback.last_update_ms,
        .valid = gyro_feedback.valid,
    };
    __set_PRIMASK(saved_primask);  // CMSIS 内核接口，恢复进入临界区前的中断状态
    uint32_t now_ms = HAL_GetTick();  // HAL 库，读取本板的毫秒计数
    ChassisInput input = ChassisRemote_Read(&remote_snapshot, now_ms);  // 项目函数，用板间遥控快照计算平移、yaw、SC 档位及 SD 使能

    const uint8_t yaw_feedback_fresh = gyro.valid && feedback.valid &&  // 陀螺仪和 yaw 编码器都需在 20 ms 内更新
                                       (uint32_t)(now_ms - gyro.last_update_ms) <= APP_YAW_FEEDBACK_AGE_MS &&
                                       (uint32_t)(now_ms - feedback.last_update_ms) <= APP_YAW_FEEDBACK_AGE_MS;
    if ((input.use_gimbal && !yaw_feedback_fresh) ||
        (input.spin != CHASSIS_SPIN_OFF && !input.use_gimbal))
    {
        input.enabled = 0U;
    }

    const uint8_t stabilize_yaw = input.enabled && input.use_gimbal;
    const GimbalYawCommand gimbal = GimbalYawControl(  // 项目函数，合成对地航向，计算 yaw 相对底盘的目标转速
        input.yaw_target_rpm, feedback.relative_yaw_rad, gyro.yaw_rpm, stabilize_yaw);
    input.gimbal_hold_error_rad = gimbal.hold_error_rad;

    ChassisControl_Step(&input, &feedback, now_ms, &chassis_command);  // 项目函数，根据使能、SC 和云台方向计算底盘的三轴命令
    const uint8_t drive_enabled = (chassis_command.mode != CHASSIS_ZERO_FORCE);
    ChassisControl_Mix(&chassis_command, APP_ROTOR_RPM_LIMIT, wheel_targets);  // 项目函数，分配四轮目标转速，超限时四轮同比缩放
    ChassisMotor_Update(wheel_targets, drive_enabled, CHASSIS_CONTROL_PERIOD_S);  // 项目函数，用速度 PI 计算四轮电流并发送 0x200
    const uint8_t speed_brake = !stabilize_yaw && input.yaw_target_rpm == 0.0f;  // 退出稳向且目标转速为零时使用比例制动
    GM6020_Update(gimbal.target_rpm, drive_enabled, CHASSIS_CONTROL_PERIOD_S, speed_brake);  // 项目函数，以 yaw 相对目标转速计算 0x1FF 电压命令
    if ((uint32_t)(now_ms - last_led_update_ms) >= 100U)
    {
        last_led_update_ms = now_ms;
        uint8_t active = (uint8_t)(chassis_command.mode != CHASSIS_ZERO_FORCE);
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11,  // HAL 库，设置 GPIO 输出电平
                          active ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12,  // HAL 库，设置 GPIO 输出电平
                          active ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

/**
 * @file chassis_motor_driver.c
 * @brief 接收 CAN1 电机反馈并计算和发送电机驱动指令。
 * @board 底盘板
 * @vehicle_function 刷新四轮转速和 yaw 编码器反馈，按控制周期向电调下发指令。
 * @inputs CAN1 电机反馈、期望转速、使能、控制周期与制动状态。
 * @outputs 四轮电流帧、yaw 电机电压帧，以及提供给控制循环的相对角。
 * @dependencies STM32 HAL CAN、CMSIS 中断接口、底盘控制循环和数学库。
 * @note 重构说明：本文件行为与原代码保持一致。CAN 接收回调与 TIM6 共享反馈，保留快照的屏蔽与恢复顺序。
 */
#include "chassis_motor_driver.h"
#include "can.h"
#include <math.h>
#include "chassis_control_loop.h"

typedef struct
{
    volatile int16_t rpm;
    int16_t cmd;  // 四轮速度环算出的电流指令
    float target;
    float err;
    float integ;  // 速度 PI 中已累计的输出分量
} ChassisWheelMotorState;
ChassisWheelMotorState motor[CHASSIS_WHEEL_COUNT];
static volatile uint32_t gm6020_rx_count;
static volatile uint16_t gm6020_angle_raw;
static volatile int16_t gm6020_rpm;  // 相对底盘的电机转速，单位 rpm
static volatile int16_t gm6020_current_raw;
#define GM6020_KP 160.0f
#define GM6020_BRAKE_KP 160.0f  // 使用速度制动分支时的比例增益
#define GM6020_KI 160.0f
#define GM6020_VOLT_LIMIT 9000.0f  // 0x1FF 电压指令的绝对值上限
#define GM6020_TIMEOUT_MS 20U  // yaw 速度控制允许的最大反馈年龄，单位 ms
#define GM6020_ENCODER_COUNTS 8192U
static uint16_t gm6020_yaw_zero_raw = 4250;  // 实测车头对齐时的 yaw 单圈编码器值

static float gm6020_integral;
static volatile uint8_t gm6020_seen;
static volatile uint32_t gm6020_last_rx_ms;
static volatile uint32_t gm6020_tx_fail_count;
#define MOTOR_PID_KP 5.0f
#define MOTOR_PID_KI 20.0f
#define MOTOR_OUTPUT_MAX 4000.0f  // 四轮电流指令绝对值上限
#define MOTOR_INTEGRAL_MAX 8000.0f  // 四轮速度 PI 的积分项限幅
#define MOTOR_FEEDBACK_DIR 1.0f

void ChassisMotor_Init(void)  // 项目函数，复位四轮反馈、积分和电流指令
{
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        motor[i].rpm = 0;
        motor[i].cmd = 0;
        motor[i].target = 0.0f;
        motor[i].err = 0.0f;
        motor[i].integ = 0.0f;
    }
}

void ChassisMotor_ReadCAN(const CAN_RxHeaderTypeDef *header, const uint8_t *data)  // 项目函数，分派 0x201～0x204 轮速和 0x206 yaw 反馈
{
    if (header == 0 || data == 0) return;
    if (header->IDE != CAN_ID_STD) return;
    if (header->RTR != CAN_RTR_DATA) return;
    if (header->DLC != 8U) return;
    if (header->StdId == 0x206U)  // 底盘板上 2 号 GM6020 的 yaw 反馈
    {
        gm6020_angle_raw = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
        gm6020_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
        gm6020_current_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);  // 实际转矩电流的有符号反馈值
        gm6020_last_rx_ms = HAL_GetTick();  // HAL 库，读取本板的毫秒计数
        gm6020_seen = 1U;
        gm6020_rx_count++;
        if (gm6020_angle_raw < GM6020_ENCODER_COUNTS)
        {
            if (gm6020_yaw_zero_raw == GM6020_ENCODER_COUNTS)  // 仅当零位设为 8192 时，才采用首帧角度作为零位
                gm6020_yaw_zero_raw = gm6020_angle_raw;
            int32_t delta = (int32_t)gm6020_angle_raw - gm6020_yaw_zero_raw;
            float relative_yaw_rad = GM6020_YAW_DIR * (float)delta *
                                     (6.283185307f / (float)GM6020_ENCODER_COUNTS);
            ChassisApp_SetGimbalFeedback(relative_yaw_rad);  // 项目函数，保存 yaw 编码器相对角和本机接收时间
        }
        return;
    }
    if (header->StdId < CHASSIS_MOTOR_CAN_RX_FIRST_ID) return;
    if (header->StdId > CHASSIS_MOTOR_CAN_RX_LAST_ID) return;
    uint8_t wheel_index = (uint8_t)(header->StdId - CHASSIS_MOTOR_CAN_RX_FIRST_ID);  // 四轮反馈 0x201～0x204 映射为下标 0～3
    motor[wheel_index].rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
}

void ChassisMotor_Update(const float target_rpm[CHASSIS_WHEEL_COUNT],  // 项目函数，用速度 PI 计算四轮电流并发送 0x200
                         uint8_t enabled, float dt_s)  // 项目函数，用速度 PI 计算四轮电流并发送 0x200
{
    int16_t rpm[CHASSIS_WHEEL_COUNT];
    uint32_t saved_primask = __get_PRIMASK();  // CMSIS 内核接口，保存当前中断屏蔽状态
    __disable_irq();  // CMSIS 内核接口，暂时屏蔽可屏蔽中断
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
        rpm[i] = motor[i].rpm;
    __set_PRIMASK(saved_primask);  // CMSIS 内核接口，恢复进入临界区前的中断状态

    if (target_rpm == 0) enabled = 0;
    if (!isfinite(dt_s) || dt_s <= 0.0f) enabled = 0;  // C 数学库，检查数值是否有限
    if (target_rpm != 0)
    {
        for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
        {
            if (!isfinite(target_rpm[i])) enabled = 0;  // C 数学库，检查数值是否有限
        }
    }
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        if (enabled)
        {
            motor[i].target = target_rpm[i];
            motor[i].err = motor[i].target - MOTOR_FEEDBACK_DIR * (float)rpm[i];
            float current_command = MOTOR_PID_KP * motor[i].err + motor[i].integ;  // 用当前积分估算输出，饱和时仅积累有助于退出饱和的误差
            if ((current_command < MOTOR_OUTPUT_MAX || motor[i].err < 0.0f) &&
                (current_command > -MOTOR_OUTPUT_MAX || motor[i].err > 0.0f))
            {
                motor[i].integ += MOTOR_PID_KI * motor[i].err * dt_s;
                if (motor[i].integ > MOTOR_INTEGRAL_MAX)
                    motor[i].integ = MOTOR_INTEGRAL_MAX;
                if (motor[i].integ < -MOTOR_INTEGRAL_MAX)
                    motor[i].integ = -MOTOR_INTEGRAL_MAX;
            }
            current_command = MOTOR_PID_KP * motor[i].err + motor[i].integ;
            if (current_command > MOTOR_OUTPUT_MAX) current_command = MOTOR_OUTPUT_MAX;
            if (current_command < -MOTOR_OUTPUT_MAX) current_command = -MOTOR_OUTPUT_MAX;
            motor[i].cmd = (int16_t)current_command;
        }
        else
        {
            motor[i].target = 0.0f;
            motor[i].err = 0.0f;
            motor[i].integ = 0.0f;
            motor[i].cmd = 0;
        }
    }
    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        int16_t current = motor[i].cmd;
        data[2U * i] = (uint8_t)((uint16_t)current >> 8);
        data[2U * i + 1U] = (uint8_t)((uint16_t)current & 0xFFU);
    }

    CAN_TxHeaderTypeDef header = {0};
    header.StdId = CHASSIS_MOTOR_CAN_TX_ID;  // 0x200 帧依次装入四个轮电机的电流
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    uint32_t mailbox;
    HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);  // HAL 库，将一帧交给硬件发送邮箱
}

void GM6020_Update(float target_rpm, uint8_t enabled, float dt_s,  // 项目函数，根据 yaw 目标与转速反馈计算电压，发送 0x1FF
                   uint8_t stick_centered)  // 项目函数，根据 yaw 目标与转速反馈计算电压，发送 0x1FF
{
    uint32_t saved_primask = __get_PRIMASK();  // CMSIS 内核接口，保存当前中断屏蔽状态
    __disable_irq();  // CMSIS 内核接口，暂时屏蔽可屏蔽中断
    int16_t rpm = gm6020_rpm;
    uint8_t seen = gm6020_seen;
    uint32_t last_feedback_ms = gm6020_last_rx_ms;
    __set_PRIMASK(saved_primask);  // CMSIS 内核接口，恢复进入临界区前的中断状态

    uint32_t age_ms = HAL_GetTick() - last_feedback_ms;  // HAL 库，读取本板的毫秒计数
    int16_t cmd = 0;
    if (enabled && seen && age_ms <= GM6020_TIMEOUT_MS &&
        isfinite(target_rpm) && isfinite(dt_s) && dt_s > 0.0f)  // C 数学库，检查数值是否有限
    {
        float measured_rpm = (float)rpm;
        float voltage;
        if (stick_centered)
        {
            gm6020_integral = 0.0f;
            voltage = GM6020_BRAKE_KP * (target_rpm - measured_rpm);
        }
        else
        {
            float error = target_rpm - measured_rpm;
            float proportional_voltage = GM6020_KP * error;
            float candidate_integral = gm6020_integral + GM6020_KI * error * dt_s;
            float candidate_voltage = proportional_voltage + candidate_integral;  // 候选电压达到限幅后，仅允许减轻饱和的积分更新
            if ((candidate_voltage <= GM6020_VOLT_LIMIT || error < 0.0f) &&
                (candidate_voltage >= -GM6020_VOLT_LIMIT || error > 0.0f))
            {
                gm6020_integral = candidate_integral;
            }
            voltage = proportional_voltage + gm6020_integral;
        }
        if (voltage > GM6020_VOLT_LIMIT) voltage = GM6020_VOLT_LIMIT;
        if (voltage < -GM6020_VOLT_LIMIT) voltage = -GM6020_VOLT_LIMIT;
        cmd = (int16_t)voltage;
    }
    else
    {
        gm6020_integral = 0.0f;
    }

    CAN_TxHeaderTypeDef header = {0};
    header.StdId = 0x1FFU;  // 2 号 yaw 的电压占 data[2] 和 data[3]
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    uint16_t bits = (uint16_t)cmd;
    uint8_t data[8] = {0};
    data[2] = (uint8_t)(bits >> 8);
    data[3] = (uint8_t)(bits & 0xFFU);
    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox) != HAL_OK)  // HAL 库，将一帧交给硬件发送邮箱
    {
        gm6020_tx_fail_count++;  // 记录 yaw 指令入邮箱失败，并在本分支清除积分
        gm6020_integral = 0.0f;
    }
}

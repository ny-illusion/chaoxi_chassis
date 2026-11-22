/*
 * 底盘调度：遥控输入 -> 模式和速度计算 -> 四轮分配 -> 电机输出。
 * Loop=循环；每 1 毫秒运行一次底盘控制。
 * Gimbal=云台；Feedback=反馈；Relative=相对；Yaw=水平转角。
 * Snapshot=某一时刻的数据副本，复制期间防止中断改写，以免新旧数据混用。
 * C 语法：static 全局量只在本文件可见，局部 static 跨调用保留值；const 表示不能通过它修改数据。
 * volatile 提醒编译器每次访问实际内存，不保证多字段读写不被打断。
 * -> 访问指针所指结构体的成员；& 取地址；! 取反；&& 且；|| 或。
 */
#include "chassis_loop.h" /* 引入底盘整体调度接口。 */
#include "chassis_motion.h" /* 引入底盘运动计算接口和数据类型。 */
#include "chassis_remote.h" /* 遥控通道与底盘动作的对应规则。 */
#include "chassis_motor.h" /* 引入电机反馈、PI 和发送接口。 */
#include "rc_sbus.h" /* 引入遥控数据类型和接收接口。 */
#include "gpio.h" /* 引入引脚初始化声明。 */
#include "main.h" /* 引入芯片库公共类型和错误处理声明。 */
#include <math.h> /* 本文件用 isfinite 判断收到的角度是否为有限数。 */

/* 四轮合成后的最大转速，单位 rpm。 */
#define APP_ROTOR_RPM_LIMIT 8000.0f

/* 控制周期，单位 s。 */
#define APP_CONTROL_DT 0.001f

/* 从上往下看，逆时针读数为正时用 +1，读数为负时用 -1。 */
#define APP_IMU_YAW_DIR           1.0f
#define APP_IMU_BIAS_SAMPLES      200U
#define APP_YAW_FEEDBACK_AGE_MS   20U
#define APP_RPM_TO_RAD_S          0.104719755f
#define APP_YAW_HOLD_KP           12.0f /* rpm/rad */
#define APP_YAW_HOLD_MAX_RPM      15.0f

typedef struct
{
    float yaw_rpm;
    uint32_t last_update_ms;
    uint8_t valid;
} ChassisGyroFeedback;

static volatile ChassisGyroFeedback gyro_feedback;

/* 底盘最后生成的命令，方便调试器 Watch。 */
ChassisCommand chassis_command;
/* chassis_command.vx_rpm、vy_rpm、wz_rpm 分别是前后、横移、旋转的轮速分量（转/分）。
 * chassis_command.mode 是本轮选定的底盘工作模式。 */

/* 保存最近的云台反馈。 */
static volatile ChassisGimbalFeedback gimbal_feedback;
/* gimbal_feedback.relative_yaw_rad 保存云台相对车头的水平角（弧度）。
 * gimbal_feedback.last_update_ms 保存底盘最近更新反馈的时刻（毫秒）。
 * gimbal_feedback.valid 为 1 表示角度数值有效；是否超时还要看更新时间。 */

/* 写入云台反馈。 */
/* 函数作用：写入云台相对底盘的水平角 relative_yaw_rad（弧度），同时记录接收时间和有效状态。 */
void ChassisApp_SetGimbalFeedback(float relative_yaw_rad)
{
    /* 保存中断状态。 */
    uint32_t irq_state = __get_PRIMASK(); /* irq 是中断；保存进入本段代码前的中断屏蔽状态。 */

    /* 写入时关闭中断。 */
    __disable_irq(); /* 暂时禁止可屏蔽中断，避免复制或更新数据时被打断。 */

    /* 保存相对角度。 */
    gimbal_feedback.relative_yaw_rad = relative_yaw_rad;

    /* 记录更新时间。 */
    gimbal_feedback.last_update_ms = HAL_GetTick();

    /* 数值正常时标记有效。 */
    gimbal_feedback.valid = (uint8_t)isfinite(relative_yaw_rad);

    /* 恢复中断状态。 */
    __set_PRIMASK(irq_state); /* 恢复之前的中断屏蔽状态，不能简单地一律打开中断。 */
}

void ChassisApp_SetGyroZ(float gyro_z_dps)
{
    static uint32_t sample_count;
    static float bias_sum_dps;
    static float bias_dps;

    /* 开机保持底盘静止，用前 200 个成功样本估计零偏。 */
    if (sample_count < APP_IMU_BIAS_SAMPLES)
    {
        bias_sum_dps += gyro_z_dps;
        sample_count++;
        if (sample_count < APP_IMU_BIAS_SAMPLES) return;
        bias_dps = bias_sum_dps / (float)APP_IMU_BIAS_SAMPLES;
    }

    const float yaw_rpm = APP_IMU_YAW_DIR * (gyro_z_dps - bias_dps) / 6.0f;
    const uint32_t sample_ms = HAL_GetTick();

    const uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    gyro_feedback.yaw_rpm = yaw_rpm;
    gyro_feedback.last_update_ms = sample_ms;
    gyro_feedback.valid = 1U;
    __set_PRIMASK(irq_state);
}

/* 初始化底盘应用。 */
/* 函数作用：初始化电机软件状态和指示灯；不负责初始化 CAN、串口或定时器硬件。 */
void ChassisApp_Init(void)
{
    /* 清空电机状态。 */
    ChassisMotor_Init();

    /* 关闭运行灯。 */
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);

    /* 点亮停止灯。 */
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);
}

typedef struct
{
    float target_rpm;
    float hold_error_rad;
} GimbalYawCommand;

/* 跟随档和自旋档共用对地稳向，将摇杆目标换算为 GM6020 相对底盘的转速目标。 */
static GimbalYawCommand GimbalYawControl(float stick_target_rpm, float relative_yaw_rad,
                                        float chassis_yaw_rpm, uint8_t stabilize_yaw)
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

    chassis_yaw_rad = ChassisControl_WrapAngle(
        chassis_yaw_rad + chassis_yaw_rpm * APP_RPM_TO_RAD_S * APP_CONTROL_DT);
    const float gimbal_world_yaw_rad = chassis_yaw_rad + relative_yaw_rad;

    const uint8_t stick_centered = (stick_target_rpm == 0.0f);
    float world_target_rpm = GM6020_YAW_DIR * stick_target_rpm;
    if (stick_centered)
    {
        if (!gimbal_holding) hold_yaw_rad = gimbal_world_yaw_rad;
        command.hold_error_rad = ChassisControl_WrapAngle(hold_yaw_rad - gimbal_world_yaw_rad);
        const float hold_rpm = APP_YAW_HOLD_KP * command.hold_error_rad;
        world_target_rpm = fmaxf(-APP_YAW_HOLD_MAX_RPM, fminf(APP_YAW_HOLD_MAX_RPM, hold_rpm));
    }
    gimbal_holding = stick_centered;

    command.target_rpm = GM6020_YAW_DIR * (world_target_rpm - chassis_yaw_rpm);
    return command;
}

/* 1 ms 执行一次。 */
/* 函数作用：每隔 1 ms 调用一次：读取遥控和云台副本，判断失控，计算四轮目标并发送电流。 */
void ChassisApp_Run1ms(void)
{
    /* LED 刷新计时。 */
    static uint32_t led_ms;

    /* 四轮目标转速。 */
    float wheel_targets[CHASSIS_WHEEL_COUNT];

    /* 遥控数据快照。 */
    RC_SBUS_Snapshot rc;
    /* 下面这些成员由 RC_SBUS_CopyData(&rc) 一次填好，供本轮控制使用。
     * rc.channels[0..15]  存放遥控器 16 个通道的原始值，channels[0] 对应第 1 通道。
     * rc.last_frame_ms    最近收到一帧有效数据的时刻，单位毫秒，用来判断超时。
     * rc.frame_count      已累计收到多少帧格式正确的数据。
     * rc.valid            1 表示曾收到格式正确的数据，当前是否失联还要看时间。
     * rc.failsafe         1 表示接收机报告丢帧或失控。 */

    /* 云台反馈快照。 */
    ChassisGimbalFeedback feedback;
    /* feedback 是本轮使用的副本，下面会从 gimbal_feedback 复制这三个成员。
     * feedback.relative_yaw_rad  云台相对车头的水平角，单位弧度，对齐时为 0。
     * feedback.last_update_ms    底盘最近更新这份反馈的时刻，单位毫秒。
     * feedback.valid             1 表示角度数值有效，0 表示无效。 */

    /* 获取遥控快照。 */
    RC_SBUS_CopyData(&rc);

    /* 保存中断状态。 */
    uint32_t irq_state = __get_PRIMASK(); /* irq 是中断；保存进入本段代码前的中断屏蔽状态。 */

    /* 一起复制云台和陀螺仪反馈。 */
    __disable_irq(); /* 暂时禁止可屏蔽中断，避免复制或更新数据时被打断。 */

    /* 复制相对角度。 */
    feedback.relative_yaw_rad = gimbal_feedback.relative_yaw_rad;

    /* 复制更新时间。 */
    feedback.last_update_ms = gimbal_feedback.last_update_ms;

    /* 复制有效标志。 */
    feedback.valid = gimbal_feedback.valid;
    const ChassisGyroFeedback gyro = {
        .yaw_rpm = gyro_feedback.yaw_rpm,
        .last_update_ms = gyro_feedback.last_update_ms,
        .valid = gyro_feedback.valid,
    };

    /* 恢复中断状态。 */
    __set_PRIMASK(irq_state); /* 恢复之前的中断屏蔽状态，不能简单地一律打开中断。 */

    /* 保存当前时间。 */
    uint32_t now = HAL_GetTick();

    /* 根据本轮的遥控快照，算出平移速度、自旋档位和使能。 */
    ChassisInput input = ChassisRemote_Read(&rc, now);

    const uint8_t yaw_feedback_fresh = gyro.valid && feedback.valid &&
        (uint32_t)(now - gyro.last_update_ms) <= APP_YAW_FEEDBACK_AGE_MS &&
        (uint32_t)(now - feedback.last_update_ms) <= APP_YAW_FEEDBACK_AGE_MS;
    if ((input.use_gimbal && !yaw_feedback_fresh) ||
        (input.spin != CHASSIS_SPIN_OFF && !input.use_gimbal))
    {
        input.enabled = 0U;
    }

    const uint8_t stabilize_yaw = input.enabled && input.use_gimbal;
    const GimbalYawCommand gimbal = GimbalYawControl(
        input.yaw_target_rpm, feedback.relative_yaw_rad, gyro.yaw_rpm, stabilize_yaw);
    input.gimbal_hold_error_rad = gimbal.hold_error_rad;

    ChassisControl_Step(&input, &feedback, now, &chassis_command);
    const uint8_t drive_enabled = (chassis_command.mode != CHASSIS_ZERO_FORCE);
    ChassisControl_Mix(&chassis_command, APP_ROTOR_RPM_LIMIT, wheel_targets);
    ChassisMotor_Update(wheel_targets, drive_enabled, APP_CONTROL_DT);
    /* 稳向期间持续使用速度 PI 和 IMU 反转补偿。 */
    const uint8_t speed_brake = !stabilize_yaw && input.yaw_target_rpm == 0.0f;
    GM6020_Update(gimbal.target_rpm, drive_enabled, APP_CONTROL_DT, speed_brake);

    /* 每 100 ms 更新一次 LED。 */
    if ((uint32_t)(now - led_ms) >= 100U)
    {
        /* 记录本次刷新时间。 */
        led_ms = now;

        /* 判断是否允许输出。 */
        uint8_t active = (uint8_t)(chassis_command.mode != CHASSIS_ZERO_FORCE);

        /* 更新运行灯。 */
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11,
                          active ? GPIO_PIN_SET : GPIO_PIN_RESET); /* 三目运算：允许运行就输出高电平，否则输出低电平。 */

        /* 更新停止灯。 */
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12,
                          active ? GPIO_PIN_RESET : GPIO_PIN_SET); /* 停止灯与运行灯相反：禁止输出时点亮。 */
    }
}

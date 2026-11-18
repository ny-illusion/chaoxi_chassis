/*
 * 底盘调度：遥控输入 -> 模式和速度计算 -> 四轮分配 -> 电机输出。
 * App=应用调度；Stick=摇杆；Scale=缩放；Threshold=判断用的分界值。
 * Gimbal=云台；Feedback=反馈；Relative=相对；Yaw=水平转角。
 * Snapshot=某一时刻的数据副本，复制期间防止中断改写，以免新旧数据混用。
 * C 语法：static 全局量只在本文件可见，局部 static 跨调用保留值；const 表示不能通过它修改数据。
 * volatile 提醒编译器每次访问实际内存，不保证多字段读写不被打断。
 * -> 访问指针所指结构体的成员；& 取地址；! 取反；&& 且；|| 或。
 */
#include "chassis_app.h" /* 引入底盘整体调度接口。 */
#include "chassis_control.h" /* 引入底盘运动计算接口和数据类型。 */
#include "chassis_motor.h" /* 引入电机反馈、PI 和发送接口。 */
#include "rc_sbus.h" /* 引入遥控数据类型和接收接口。 */
#include "gpio.h" /* 引入引脚初始化声明。 */
#include "main.h" /* 引入芯片库公共类型和错误处理声明。 */
#include <math.h> /* 引入数学函数：isfinite 检查有限数，fabsf 求绝对值，cosf/sinf 求余弦/正弦，fmodf 求余数。 */

/* 摇杆中位。 */
#define APP_RC_MID 1024.0f

/* 摇杆中位到一端的距离。 */
#define APP_RC_HALF 671.0f

/* SD 总使能阈值。 */
#define APP_RC_ENABLE_THRESHOLD 1400U

/* 遥控超时时间，单位 ms。 */
#define APP_RC_LOST_MS 100U

/* 满杆目标转速，单位 rpm。 */
#define APP_TARGET_RPM_MAX 8000.0f

/* 四轮合成后的最大转速，单位 rpm。 */
#define APP_ROTOR_RPM_LIMIT 8000.0f

/* 自旋档位前阈值。 */
#define APP_SC_TH_LOW 690.0f

/* 自旋档位后阈值。 */
#define APP_SC_TH_HIGH 1360.0f

/* 控制周期，单位 s。 */
#define APP_CONTROL_DT 0.001f

/* 底盘最后生成的命令，方便调试器 Watch。 */
ChassisCommand chassis_command;

/* 当前自旋档位，方便调试器 Watch。 */
uint8_t rc_mode = CHASSIS_SPIN_OFF;

/* 当前摇杆值，方便调试器 Watch。 */
float rc_vx; /* 保存前后摇杆的比例，范围为 -1 到 +1。 */
float rc_vy; /* 保存左右摇杆的比例，范围为 -1 到 +1。 */
float rc_wz; /* 保存转向摇杆的比例，范围为 -1 到 +1。 */

/* 当前使能和失控状态，方便调试器 Watch。 */
uint8_t rc_enable; /* enable 表示允许；记录 SD 是否允许底盘输出。 */
uint8_t rc_lost = 1; /* lost 表示丢失；上电先认为遥控不可用，等待有效数据。 */

/* 保存最近的云台反馈。 */
static volatile ChassisGimbalFeedback gimbal_feedback;

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
/* 把 SBUS 原始值变成 -1 到 +1。 */
/* 函数作用：输入 raw 是 SBUS 原始通道值；返回 -1 到 +1 的摇杆比例，并把中位附近的小抖动归零。 */
static float App_ScaleStick(uint16_t raw)
{
    /* 转成相对中位的比例。 */
    float value = (APP_RC_MID - (float)raw) / APP_RC_HALF;

    /* 限制最大值。 */
    if (value > 1.0f) value = 1.0f;

    /* 限制最小值。 */
    if (value < -1.0f) value = -1.0f;

    /* 把中位附近 ±0.05 的输入归零，设置摇杆死区以避免轻微抖动。 */
    if (value > -0.05f && value < 0.05f) value = 0.0f;

    /* 返回归一化结果。 */
    return value;
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

    /* 云台反馈快照。 */
    ChassisGimbalFeedback feedback;

    /* 获取遥控快照。 */
    RC_SBUS_CopyData(&rc);

    /* 保存中断状态。 */
    uint32_t irq_state = __get_PRIMASK(); /* irq 是中断；保存进入本段代码前的中断屏蔽状态。 */

    /* 复制云台反馈时关闭中断。 */
    __disable_irq(); /* 暂时禁止可屏蔽中断，避免复制或更新数据时被打断。 */

    /* 复制相对角度。 */
    feedback.relative_yaw_rad = gimbal_feedback.relative_yaw_rad;

    /* 复制更新时间。 */
    feedback.last_update_ms = gimbal_feedback.last_update_ms;

    /* 复制有效标志。 */
    feedback.valid = gimbal_feedback.valid;

    /* 恢复中断状态。 */
    __set_PRIMASK(irq_state); /* 恢复之前的中断屏蔽状态，不能简单地一律打开中断。 */

    /* 保存当前时间。 */
    uint32_t now = HAL_GetTick();

    /* 前后摇杆。 */
    rc_vx = App_ScaleStick(rc.channels[2]);

    /* 左右摇杆。 */
    rc_vy = App_ScaleStick(rc.channels[3]);

    /* 转向摇杆。 */
    rc_wz = App_ScaleStick(rc.channels[0]);

    /* 读取 SD 总使能。 */
    rc_enable = (uint8_t)(rc.channels[5] > APP_RC_ENABLE_THRESHOLD);

    /* 判断遥控是否失控。 */
    rc_lost = (uint8_t)(!rc.valid || rc.failsafe ||
                        (uint32_t)(now - rc.last_frame_ms) > APP_RC_LOST_MS); /* 距离上一帧超过 100 ms 也算失控；无符号相减可处理毫秒计数回绕。 */

    /* 通道值异常时不自旋。 */
    if (rc.channels[4] < 300U) rc_mode = CHASSIS_SPIN_OFF;

    /* SC 上档对应高速自旋。 */
    else if ((float)rc.channels[4] < APP_SC_TH_LOW) rc_mode = CHASSIS_SPIN_HIGH;

    /* SC 中档对应低速自旋。 */
    else if ((float)rc.channels[4] < APP_SC_TH_HIGH) rc_mode = CHASSIS_SPIN_LOW;

    /* SC 下档对应不自旋。 */
    else rc_mode = CHASSIS_SPIN_OFF;

    /* 组装底盘输入。 */
    ChassisInput input = {
        .vx_rpm = rc_vx * APP_TARGET_RPM_MAX, /* 前后摇杆比例乘以满杆轮速分量，得到前后输入。 */
        .vy_rpm = rc_vy * APP_TARGET_RPM_MAX, /* 左右摇杆比例乘以满杆轮速分量，得到横移输入。 */
        .manual_wz_rpm = rc_wz * CHASSIS_MANUAL_RPM_MAX, /* 转向摇杆比例乘以手动转向上限。 */
        .spin = (ChassisSpinMode)rc_mode, /* spin 表示自旋；把档位数值转换为自旋档位类型。 */
        .enabled = (uint8_t)(rc_enable && !rc_lost), /* SD 允许并且遥控没失控时，才允许输出。 */
        .use_gimbal = CHASSIS_GIMBAL_INSTALLED /* 根据配置决定是否使用云台角度；0 不用，1 使用。 */
    };

    /* 计算底盘命令。 */
    ChassisControl_Step(&input, &feedback, now, &chassis_command);

    /* 把命令分配到四个轮子。 */
    ChassisControl_Mix(&chassis_command, APP_ROTOR_RPM_LIMIT, wheel_targets);

    /* 计算速度 PI 并发送电流。 */
    ChassisMotor_Update(wheel_targets,
                        (uint8_t)(chassis_command.mode != CHASSIS_ZERO_FORCE), /* 模式不是零力时，才让电机模块计算输出电流。 */
                        APP_CONTROL_DT); /* 传入控制间隔 0.001 秒，供 PI 积分计算使用。 */

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

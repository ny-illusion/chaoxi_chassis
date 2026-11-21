/* 把遥控通道换算成底盘输入：平移速度、自旋档位和总使能。 */
#include "chassis_remote.h"

/* 摇杆中位。 */
#define REMOTE_RC_MID 1024.0f

/* 摇杆中位到一端的距离。 */
#define REMOTE_RC_HALF 671.0f

/* SD 总使能阈值。 */
#define REMOTE_RC_ENABLE_THRESHOLD 1400U

/* 遥控超时时间，单位 ms。 */
#define REMOTE_RC_LOST_MS 100U

/* 满杆目标转速，单位 rpm。 */
#define REMOTE_TARGET_RPM_MAX 8000.0f

/* 自旋档位前阈值。 */
#define REMOTE_SC_TH_LOW 690.0f

/* 低于此值的 SC 读数按异常处理。 */
#define REMOTE_SC_VALID_MIN 300U

/* 自旋档位后阈值。 */
#define REMOTE_SC_TH_HIGH 1360.0f

#define YAW_PREVIEW_MAX_RPM 50.0f

/* 当前自旋档位，方便调试器 Watch。 */
uint8_t rc_mode = CHASSIS_SPIN_OFF;

/* 当前摇杆值，方便调试器 Watch。 */
float rc_vx; /* 保存前后摇杆的比例，范围为 -1 到 +1。 */
float rc_vy; /* 保存左右摇杆的比例，范围为 -1 到 +1。 */

static volatile float rc_yaw_stick;    /* 云台左右摇杆比例，左正右负。 */
static volatile float yaw_preview_rpm; /* 由摇杆换算出的目标转速，单位 rpm。 */

/* 当前使能和失控状态，方便调试器 Watch。 */
uint8_t rc_enable; /* enable 表示允许；记录 SD 是否允许底盘输出。 */
uint8_t rc_lost = 1; /* lost 表示丢失；上电先认为遥控不可用，等待有效数据。 */

/* 把 SBUS 原始值变成 -1 到 +1。 */
/* 函数作用：输入 raw 是 SBUS 原始通道值；返回 -1 到 +1 的摇杆比例，并把中位附近的小抖动归零。 */
static float ScaleStick(uint16_t raw)
{
    /* 转成相对中位的比例。 */
    float value = (REMOTE_RC_MID - (float)raw) / REMOTE_RC_HALF;

    /* 限制最大值。 */
    if (value > 1.0f) value = 1.0f;

    /* 限制最小值。 */
    if (value < -1.0f) value = -1.0f;

    /* 把中位附近 ±0.05 的输入归零，设置摇杆死区以避免轻微抖动。 */
    if (value > -0.05f && value < 0.05f) value = 0.0f;

    /* 返回归一化结果。 */
    return value;
}

ChassisInput ChassisRemote_Read(const RC_SBUS_Snapshot *rc, uint32_t now)
{
    /* 前后摇杆。 */
    rc_vx = ScaleStick(rc->channels[2]);

    /* 底盘横移摇杆。 */
    rc_vy = ScaleStick(rc->channels[3]);

    /* 云台左右摇杆。 */
    rc_yaw_stick = ScaleStick(rc->channels[0]);

    /* 通道 [0] 和 [1] 留给云台。 */

    /* 读取 SD 总使能。 */
    rc_enable = (uint8_t)(rc->channels[5] > REMOTE_RC_ENABLE_THRESHOLD);

    /* 判断遥控是否失控。 */
    rc_lost = (uint8_t)(!rc->valid || rc->failsafe ||
                        (uint32_t)(now - rc->last_frame_ms) > REMOTE_RC_LOST_MS); /* 距离上一帧超过 100 ms 也算失控；无符号相减可处理毫秒计数回绕。 */

    /* 通道值异常时不自旋。 */
    if (rc->channels[4] < REMOTE_SC_VALID_MIN) rc_mode = CHASSIS_SPIN_OFF;

    /* SC 上档对应高速自旋。 */
    else if ((float)rc->channels[4] < REMOTE_SC_TH_LOW) rc_mode = CHASSIS_SPIN_HIGH;

    /* SC 中档对应低速自旋。 */
    else if ((float)rc->channels[4] < REMOTE_SC_TH_HIGH) rc_mode = CHASSIS_SPIN_LOW;

    /* SC 下档对应不自旋。 */
    else rc_mode = CHASSIS_SPIN_OFF;

    /* 组装底盘输入。 */
    ChassisInput input = {
        .vx_rpm = rc_vx * REMOTE_TARGET_RPM_MAX, /* 前后摇杆比例乘以满杆轮速分量，得到前后输入。 */
        .vy_rpm = rc_vy * REMOTE_TARGET_RPM_MAX, /* 左右摇杆比例乘以满杆轮速分量，得到横移输入。 */
        .spin = (ChassisSpinMode)rc_mode, /* spin 表示自旋；把档位数值转换为自旋档位类型。 */
        .enabled = (uint8_t)(rc_enable && !rc_lost), /* SD 允许并且遥控没失控时，才允许输出。 */
        .use_gimbal = (uint8_t)(CHASSIS_GIMBAL_INSTALLED &&
                               rc->channels[4] >= REMOTE_SC_VALID_MIN)
    };

    input.yaw_target_rpm = input.enabled
                         ? rc_yaw_stick * YAW_PREVIEW_MAX_RPM
                         : 0.0f;

    yaw_preview_rpm = input.yaw_target_rpm;  /* 保留给 Watch 观察。 */
    return input;
}

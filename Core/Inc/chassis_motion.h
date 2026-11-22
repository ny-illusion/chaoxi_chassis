#ifndef CHASSIS_MOTION_H /* 如果本头文件还没加载过，就继续加载。 */
#define CHASSIS_MOTION_H /* 标记已经加载，避免重复定义。 */

#include <stdint.h> /* 引入固定宽度整数，如 uint8_t 是 8 位无符号整数。 */

/* 命名缩写：rpm=每分钟转数，rad=弧度，ms=毫秒，wz=绕 z 轴旋转。 */

/* 0 表示还没装云台。 */
#define CHASSIS_GIMBAL_INSTALLED 1U

/* 云台反馈超过 100 ms 没更新就认为失效。 */
#define CHASSIS_GIMBAL_TIMEOUT_MS 100U

/* 低速自旋的轮速分量，单位 rpm。 */
#define CHASSIS_SPIN_RPM_LOW 1500.0f

/* 高速自旋的轮速分量，单位 rpm。 */
#define CHASSIS_SPIN_RPM_HIGH 3000.0f

/* 云台跟随的比例系数，角度越大转向越快。 */
#define CHASSIS_FOLLOW_KP 3000.0f

/* 云台跟随的最大转向轮速分量，单位 rpm。 */
#define CHASSIS_FOLLOW_RPM_MAX 2250.0f

/* 云台跟随的角度死区，单位 rad。 */
#define CHASSIS_FOLLOW_DEADBAND_RAD 0.035f

/* 底盘轮子数量。 */
#define CHASSIS_WHEEL_COUNT 4U

/* 自旋档位。 */
typedef enum /* 定义一组有名字的整数，用来表示不同档位或模式。 */
{
    CHASSIS_SPIN_OFF = 0,  /* 不自旋。 */
    CHASSIS_SPIN_LOW,      /* 低速自旋。 */
    CHASSIS_SPIN_HIGH      /* 高速自旋。 */
} ChassisSpinMode;

/* 本周期底盘所处的工作模式。 */
typedef enum /* 定义一组有名字的整数，用来表示不同档位或模式。 */
{
    CHASSIS_ZERO_FORCE = 0,  /* 零力输出。 */
    CHASSIS_MANUAL,          /* 无云台且关闭自旋：只控制平移。 */
    CHASSIS_FOLLOW_GIMBAL,   /* 跟随云台。 */
    CHASSIS_ROTATE_LOW,      /* 低速自旋。 */
    CHASSIS_ROTATE_HIGH      /* 高速自旋。 */
} ChassisMode;

/* 云台相对底盘的反馈。 */
typedef struct /* 把相关数据放到同一个结构体中，并为类型起名。 */
{
    float relative_yaw_rad;   /* 云台减底盘的角度，单位 rad。 */
    uint32_t last_update_ms;  /* 最近更新时间，单位 ms。 */
    uint8_t valid;            /* 1 表示反馈有效，0 表示无效。 */
} ChassisGimbalFeedback;

/* 底盘控制的输入数据。 */
typedef struct /* 把相关数据放到同一个结构体中，并为类型起名。 */
{
    float vx_rpm;             /* x 方向速度，前方为正，单位 rpm。 */
    float vy_rpm;             /* y 方向速度，左方为正，单位 rpm。 */
    ChassisSpinMode spin;     /* 自旋档位。 */
    uint8_t enabled;          /* 1 表示允许输出，0 表示零力。 */
    uint8_t use_gimbal;       /* 1 表示使用云台角度，0 表示不用。 */
    float yaw_target_rpm;     /* yaw 摇杆目标转速，采用 GM6020 的正方向。 */
    float gimbal_hold_error_rad; /* 松杆锁定的对地航向减去云台当前对地航向；推杆时为零。 */
} ChassisInput;

/* 底盘控制的输出数据。 */
typedef struct /* 把相关数据放到同一个结构体中，并为类型起名。 */
{
    float vx_rpm;             /* 底盘 x 方向速度分量，单位 rpm。 */
    float vy_rpm;             /* 底盘 y 方向速度分量，单位 rpm。 */
    float wz_rpm;             /* 底盘旋转速度分量，单位 rpm。 */
    ChassisMode mode;         /* 本周期工作模式。 */
} ChassisCommand;

/* 将角度折回 [-π, π]，单位 rad。 */
float ChassisControl_WrapAngle(float angle);

/* 函数名：Step = 计算一个控制周期。 */
/* input：输入数据指针。 */
/* feedback：云台反馈数据指针。 */
/* now_ms：当前时间，单位 ms。 */
/* command：输出命令指针。 */
/* 函数作用：计算底盘命令：input 是输入，feedback 是云台反馈，now_ms 是当前毫秒，command 接收结果。指针须有效。 */
void ChassisControl_Step(const ChassisInput *input,
                         const ChassisGimbalFeedback *feedback,
                         uint32_t now_ms, ChassisCommand *command);

/* 函数名：Mix = 把三个速度分量分配给四个轮子。 */
/* command：底盘速度命令。 */
/* limit_rpm：单个轮子的最大目标转速，单位 rpm。 */
/* targets：输出四轮目标转速，顺序为左前、右前、右后、左后。 */
/* 函数作用：把 command 的前后、横移、旋转分配到 targets 四轮数组；超过 limit_rpm 时四轮等比例缩小。指针须有效。 */
void ChassisControl_Mix(const ChassisCommand *command, float limit_rpm,
                        float targets[CHASSIS_WHEEL_COUNT]);


#endif

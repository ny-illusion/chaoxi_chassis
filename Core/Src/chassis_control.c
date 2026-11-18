/*
 * 底盘算法：根据输入选择运动模式，并计算四轮各自的目标转速。
 * Step=计算一轮控制；Mix=分配到四轮；Clamp=限制数值范围。
 * WrapAngle=把角度折回一圈内；Peak=最大绝对值；Limit=上限；Target=目标。
 * Spin/Rotate=旋转；Follow=跟随；Manual=手动；Deadband=不响应的小范围。
 * vx/vy/wz 在本工程都是转子轮速分量，不是米每秒或弧度每秒。
 * C 语法：static 全局量只在本文件可见，局部 static 跨调用保留值；const 表示不能通过它修改数据。
 * volatile 提醒编译器每次访问实际内存，不保证多字段读写不被打断。
 * -> 访问指针所指结构体的成员；& 取地址；! 取反；&& 且；|| 或。
 */
#include "chassis_control.h" /* 引入底盘运动计算接口和数据类型。 */

#include <math.h>          /* cosf、sinf、fabsf、isfinite、fmodf */

#define CHASSIS_PI 3.14159265f /* 圆周率 π；末尾 f 表示单精度小数。 */
#define CHASSIS_TWO_PI 6.28318531f /* 2π 弧度是一整圈，也就是 360 度。 */

/* 坐标：x 向前，y 向左，逆时针为正。 */
/* 单位：vx/vy/wz 都是轮速分量，最终折算到电机转子 rpm。 */
/* 轮子顺序：左前、右前、右后、左后。 */

/* 把数值限制在 [-limit, limit] 内。 */
/* 函数作用：输入 value 和正的 limit；返回限制在 -limit 到 +limit 范围内的数值。 */
static float Clamp(float value, float limit)
{
    /* 超过上限时取上限。 */
    if (value > limit) return limit;

    /* 低于下限时取下限。 */
    if (value < -limit) return -limit;

    /* 没超限就保持原值。 */
    return value;
}

/* 把角度折叠到 -π 到 π 之间。 */
/* 函数作用：输入 angle（弧度）；返回 -π 到 +π 内的等效角，避免跨一圈时跟随方向突变。 */
static float WrapAngle(float angle)
{
    /* 先去掉整圈，避免角度越来越大。 */
    angle = fmodf(angle, CHASSIS_TWO_PI);

    /* 大于 π 时减一圈。 */
    if (angle > CHASSIS_PI) angle -= CHASSIS_TWO_PI;

    /* 小于 -π 时加一圈。 */
    if (angle < -CHASSIS_PI) angle += CHASSIS_TWO_PI;

    /* 返回折叠后的角度。 */
    return angle;
}

/* 函数名：Step = 计算一个控制周期。 */
/* 函数作用：计算底盘命令：input 是输入，feedback 是云台反馈，now_ms 是当前毫秒，command 接收结果。指针须有效。 */
void ChassisControl_Step(
    const ChassisInput *input,             /* 输入：摇杆、模式和使能。 */
    const ChassisGimbalFeedback *feedback, /* 输入：云台角度反馈。 */
    uint32_t now_ms,                       /* 输入：当前时间，单位 ms。 */
    ChassisCommand *command)               /* 输出：底盘速度命令。 */
{
    /* 云台相对底盘的角度，没有云台时为 0。 */
    float yaw = 0.0f;

    /* 把输出结构体各字段清零；模式 0 正好是零力，因此异常时直接 return 就不会留下旧命令。 */
    *command = (ChassisCommand){0};

    /* 总使能关闭时零力。 */
    if (!input->enabled) return;

    /* 前后速度非法时零力。 */
    if (!isfinite(input->vx_rpm)) return;

    /* 平移速度非法时零力。 */
    if (!isfinite(input->vy_rpm)) return;

    /* 手动转向非法时零力。 */
    if (!isfinite(input->manual_wz_rpm)) return;

    /* 配置了云台才检查云台反馈。 */
    if (input->use_gimbal)
    {
        /* 没收到过云台反馈时零力。 */
        if (!feedback->valid) return;

        /* 云台角度非法时零力。 */
        if (!isfinite(feedback->relative_yaw_rad)) return;

        /* 云台反馈超过 100 ms 没更新时零力。 */
        if ((uint32_t)(now_ms - feedback->last_update_ms) > CHASSIS_GIMBAL_TIMEOUT_MS)
            return; /* 立即结束本次函数调用，不再执行后面的代码。 */

        /* 把云台角度折叠到 -π 到 π。 */
        yaw = WrapAngle(feedback->relative_yaw_rad);
    }

    /* 根据自旋档位选择底盘旋转方式。 */
    switch (input->spin)
    {
    case CHASSIS_SPIN_LOW: /* 自旋档位为低速时，执行这个分支。 */
        /* 标记为低速自旋模式。 */
        command->mode = CHASSIS_ROTATE_LOW;

        /* 使用固定的低速自旋分量。 */
        command->wz_rpm = CHASSIS_SPIN_RPM_LOW;
        break; /* 结束当前 switch 分支，避免接着执行下一个档位。 */

    case CHASSIS_SPIN_HIGH: /* 自旋档位为高速时，执行这个分支。 */
        /* 标记为高速自旋模式。 */
        command->mode = CHASSIS_ROTATE_HIGH;

        /* 使用固定的高速自旋分量。 */
        command->wz_rpm = CHASSIS_SPIN_RPM_HIGH;
        break; /* 结束当前 switch 分支，避免接着执行下一个档位。 */

    case CHASSIS_SPIN_OFF: /* 自旋关闭时，改用手动转向或云台跟随。 */
        /* 不开启自旋时，有云台就跟随云台。 */
        if (input->use_gimbal)
        {
            /* 标记为云台跟随模式。 */
            command->mode = CHASSIS_FOLLOW_GIMBAL;

            /* 角度超过死区才转动，减少原地抖动。 */
            if (fabsf(yaw) > CHASSIS_FOLLOW_DEADBAND_RAD)
            {
                /* 比例控制：角度越大，跟随转向越大。 */
                command->wz_rpm = CHASSIS_FOLLOW_KP * yaw;

                /* 限制跟随转向的最大轮速分量。 */
                command->wz_rpm = Clamp(command->wz_rpm, CHASSIS_FOLLOW_RPM_MAX);
            }
        }
        else /* 前面的条件不满足时，执行下面的分支。 */
        {
            /* 没装云台时使用手动转向模式。 */
            command->mode = CHASSIS_MANUAL;

            /* 摇杆转向乘以倍率，并限制最大值。 */
            command->wz_rpm = Clamp(input->manual_wz_rpm, CHASSIS_MANUAL_RPM_MAX);
        }
        break; /* 结束当前 switch 分支，避免接着执行下一个档位。 */

    default: /* 处理不属于已知档位的输入。 */
        /* 未知档位时保持零力。 */
        return; /* 立即结束本次函数调用，不再执行后面的代码。 */
    }

    /* 计算角度的余弦。 */
    float c = cosf(yaw);

    /* 计算角度的正弦。 */
    float s = sinf(yaw);

    /* 把云台坐标系的前后速度转到底盘 x 轴。 */
    command->vx_rpm = input->vx_rpm * c - input->vy_rpm * s;

    /* 把云台坐标系的平移速度转到底盘 y 轴。 */
    command->vy_rpm = input->vx_rpm * s + input->vy_rpm * c;
}

/* 函数名：Mix = 把三个速度分量分配给四个轮子。 */
/* 函数作用：把 command 的前后、横移、旋转分配到 targets 四轮数组；超过 limit_rpm 时四轮等比例缩小。指针须有效。 */
void ChassisControl_Mix(
    const ChassisCommand *command,        /* 输入：底盘速度命令。 */
    float limit_rpm,                      /* 输入：单轮最大转速，单位 rpm。 */
    float targets[CHASSIS_WHEEL_COUNT])   /* 输出：四轮目标转速，单位 rpm。 */
{
    /* 电机安装方向：+1 同向，-1 反向。 */
    static const int8_t wheel_dir[CHASSIS_WHEEL_COUNT] = {+1, -1, -1, +1};

    /* 横移时四个轮子的方向系数。 */
    static const int8_t k_vy[CHASSIS_WHEEL_COUNT] = {-1, +1, -1, +1};

    /* 自旋时四个轮子的方向系数。 */
    static const int8_t k_wz[CHASSIS_WHEEL_COUNT] = {-1, +1, +1, -1};

    /* 记录四轮中最大的目标转速。 */
    float peak = 0.0f;

    /* 先把四个轮子的目标清零。 */
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i) targets[i] = 0.0f;

    /* 零力模式直接返回。 */
    if (command->mode == CHASSIS_ZERO_FORCE) return;

    /* 轮速上限非法时返回。 */
    if (!isfinite(limit_rpm) || limit_rpm <= 0.0f) return;

    /* 前后分量非法时返回。 */
    if (!isfinite(command->vx_rpm)) return;

    /* 平移分量非法时返回。 */
    if (!isfinite(command->vy_rpm)) return;

    /* 自旋分量非法时返回。 */
    if (!isfinite(command->wz_rpm)) return;

    /* 逐个计算四个轮子的目标转速。 */
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        /* 前后、平移、自旋相加，再乘安装方向。 */
        targets[i] = command->vx_rpm + k_vy[i] * command->vy_rpm
                   + k_wz[i] * command->wz_rpm; /* 再叠加这个轮子承担的旋转分量。 */
        targets[i] *= wheel_dir[i]; /* 乘上电机安装方向；*= 表示乘完再存回原变量。 */

        /* 找出绝对值最大的轮速。 */
        if (fabsf(targets[i]) > peak) peak = fabsf(targets[i]);
    }

    /* 没有超过上限就不需要缩放。 */
    if (peak <= limit_rpm) return;

    /* 比如最大值为 10000、上限为 8000，则 scale=0.8；四轮都乘它，保持运动方向。 */
    float scale = limit_rpm / peak;

    /* 四轮一起缩放，保持原来的运动比例。 */
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i) targets[i] *= scale;
}

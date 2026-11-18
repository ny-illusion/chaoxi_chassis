/*
 * 电机模块：读取实际转速，用速度 PI 计算电流，再通过 CAN 发送。
 * PI：P 比例项立即响应误差；I 积分项累加误差，补偿持续偏差。
 * Error/err=误差；Integral/integ=积分；Output/out=输出；Command/cmd=指令。
 * 电流指令是 C620 协议的数值，不直接等于安培；rpm 是每分钟转数。
 * C 语法：static 全局量只在本文件可见，局部 static 跨调用保留值；const 表示不能通过它修改数据。
 * volatile 提醒编译器每次访问实际内存，不保证多字段读写不被打断。
 * -> 访问指针所指结构体的成员；& 取地址；! 取反；&& 且；|| 或。
 */
#include "chassis_motor.h" /* 引入电机反馈、PI 和发送接口。 */
#include "can.h" /* 引入CAN 外设句柄和初始化声明。 */
#include <math.h> /* 引入数学函数：isfinite 检查有限数，fabsf 求绝对值，cosf/sinf 求余弦/正弦，fmodf 求余数。 */

/* 单个电机的数据。 */
typedef struct /* 把相关数据放到同一个结构体中，并为类型起名。 */
{
    volatile int16_t rpm; /* 转子转速，单位 rpm。 */
    int16_t cmd;          /* 发送给电调的电流指令。 */
    float target;         /* 目标转速，单位 rpm。 */
    float err;            /* 速度误差。 */
    float integ;          /* 积分项。 */
} Motor_t;

/* 四个电机的状态，保留为全局方便调试器 Watch。 */
Motor_t motor[CHASSIS_WHEEL_COUNT];

/* P 比例系数：当前转速误差乘以它，得到比例输出；这里只用 PI，没有 D 微分项。 */
#define MOTOR_PID_KP 5.0f

/* 速度 PI 积分系数。 */
#define MOTOR_PID_KI 20.0f

/* 单轮输出上限。 */
#define MOTOR_OUTPUT_MAX 6000.0f

/* 积分项上限。 */
#define MOTOR_INTEGRAL_MAX 8000.0f

/* 速度反馈方向修正。 */
#define MOTOR_FEEDBACK_DIR 1.0f

/* 清空四个电机。 */
/* 函数作用：清空四个电机的反馈、目标、误差、积分和电流指令；此函数本身不发送 CAN。 */
void ChassisMotor_Init(void)
{
    /* 逐个清空。 */
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        motor[i].rpm = 0; /* 清零第 i 个电机的反馈转速。 */
        motor[i].cmd = 0; /* 清零第 i 个电机的电流指令；cmd 是 command 的缩写。 */
        motor[i].target = 0.0f; /* 清零目标转速；target 表示希望达到的值。 */
        motor[i].err = 0.0f; /* 清零转速误差；err 是 error 的缩写。 */
        motor[i].integ = 0.0f; /* 清零积分累加值，防止重新使能时残留旧输出。 */
    }
}

/* 解析一个 CAN 反馈帧。 */
/* 函数作用：header 是接收到的帧头，data 是 8 字节内容；验证编号和格式后更新对应电机的转速。 */
void ChassisMotor_ReadCAN(const CAN_RxHeaderTypeDef *header, const uint8_t *data)
{
    /* 空指针直接返回。 */
    if (header == 0 || data == 0) return;

    /* 只接收标准数据帧。 */
    if (header->IDE != CAN_ID_STD) return; /* IDE 表示编号格式；不是标准帧就退出。 */
    if (header->RTR != CAN_RTR_DATA) return; /* RTR 表示帧类型；只处理数据帧，不处理远程请求帧。 */
    if (header->DLC != 8U) return; /* DLC 表示数据长度；必须刚好有 8 个字节。 */

    /* 只接收四个电调的 0x201~0x204。 */
    if (header->StdId < CHASSIS_MOTOR_CAN_RX_FIRST_ID) return;
    if (header->StdId > CHASSIS_MOTOR_CAN_RX_LAST_ID) return;

    /* 把 ID 换算成 0~3 的数组下标。 */
    uint8_t index = (uint8_t)(header->StdId - CHASSIS_MOTOR_CAN_RX_FIRST_ID);

    /* 拼接大端转速，data[2] 是高字节。 */
    motor[index].rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
}

/* 计算速度 PI 并发送电流。 */
/* 函数作用：输入四轮 target_rpm、允许标志 enabled 和周期 dt_s（秒）；计算 PI 并尝试发送 0x200 电流帧。 */
void ChassisMotor_Update(const float target_rpm[CHASSIS_WHEEL_COUNT],
                         uint8_t enabled, float dt_s)
{
    /* 保存四个电机转速。 */
    int16_t rpm[CHASSIS_WHEEL_COUNT];

    /* 保存中断状态。 */
    uint32_t irq_state = __get_PRIMASK(); /* irq 是中断；保存进入本段代码前的中断屏蔽状态。 */

    /* 复制转速时关闭中断。 */
    __disable_irq(); /* 暂时禁止可屏蔽中断，避免复制或更新数据时被打断。 */

    /* 逐个复制转速。 */
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
        rpm[i] = motor[i].rpm; /* 把第 i 个电机的反馈转速复制到本次计算用的数组。 */

    /* 恢复中断状态。 */
    __set_PRIMASK(irq_state); /* 恢复之前的中断屏蔽状态，不能简单地一律打开中断。 */

    /* 目标数组为空时禁止输出。 */
    if (target_rpm == 0) enabled = 0;

    /* 控制周期非法时禁止输出。 */
    if (!isfinite(dt_s) || dt_s <= 0.0f) enabled = 0;

    /* 先检查全部目标，避免只处理一半电机。 */
    if (target_rpm != 0)
    {
        /* 逐个检查目标转速。 */
        for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
        {
            /* 任一目标非法时全部禁止输出。 */
            if (!isfinite(target_rpm[i])) enabled = 0;
        }
    }

    /* 逐个电机计算。 */
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        /* 允许输出时计算速度 PI。 */
        if (enabled)
        {
            /* 保存目标转速。 */
            motor[i].target = target_rpm[i];

            /* 计算速度误差。 */
            motor[i].err = motor[i].target - MOTOR_FEEDBACK_DIR * (float)rpm[i];

            /* 计算比例项和积分项。 */
            float out = MOTOR_PID_KP * motor[i].err + motor[i].integ;

            /* 输出未超限，或误差能把已超限的输出拉回时，才允许累加积分。 */
            if ((out < MOTOR_OUTPUT_MAX || motor[i].err < 0.0f) &&
                (out > -MOTOR_OUTPUT_MAX || motor[i].err > 0.0f)) /* 负向也检查：允许能让输出回到范围内的积分；两行条件必须同时满足。 */
            {
                /* 累加积分。 */
                motor[i].integ += MOTOR_PID_KI * motor[i].err * dt_s;

                /* 限制积分最大值。 */
                if (motor[i].integ > MOTOR_INTEGRAL_MAX)
                    motor[i].integ = MOTOR_INTEGRAL_MAX; /* 超过积分上限时，固定在上限。 */

                /* 限制积分最小值。 */
                if (motor[i].integ < -MOTOR_INTEGRAL_MAX)
                    motor[i].integ = -MOTOR_INTEGRAL_MAX; /* 低于积分下限时，固定在下限。 */
            }

            /* 加上积分后重算输出。 */
            out = MOTOR_PID_KP * motor[i].err + motor[i].integ;

            /* 限制输出最大值。 */
            if (out > MOTOR_OUTPUT_MAX) out = MOTOR_OUTPUT_MAX;

            /* 限制输出最小值。 */
            if (out < -MOTOR_OUTPUT_MAX) out = -MOTOR_OUTPUT_MAX;

            /* 保存最终电流。 */
            motor[i].cmd = (int16_t)out;
        }
        else /* 前面的条件不满足时，执行下面的分支。 */
        {
            /* 禁止输出时清空状态。 */
            motor[i].target = 0.0f; /* 清零目标转速；target 表示希望达到的值。 */
            motor[i].err = 0.0f; /* 清零转速误差；err 是 error 的缩写。 */
            motor[i].integ = 0.0f; /* 清零积分累加值，防止重新使能时残留旧输出。 */
            motor[i].cmd = 0; /* 清零第 i 个电机的电流指令；cmd 是 command 的缩写。 */
        }
    }

    /* 准备八个字节的 CAN 数据。 */
    uint8_t data[8] = {0};

    /* 填入四路电流。 */
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        /* 取出电流指令。 */
        int16_t current = motor[i].cmd;

        /* 高字节在前。 */
        data[2U * i] = (uint8_t)((uint16_t)current >> 8);

        /* 低字节在后。 */
        data[2U * i + 1U] = (uint8_t)((uint16_t)current & 0xFFU);
    }

    /* 设置发送帧头。 */
    CAN_TxHeaderTypeDef header = {0};
    header.StdId = CHASSIS_MOTOR_CAN_TX_ID; /* StdId 是标准帧编号；0x200 用来给四个电调发送电流。 */
    header.IDE = CAN_ID_STD; /* 使用 11 位标准 CAN 编号。 */
    header.RTR = CAN_RTR_DATA; /* 发送的是含有实际数据的数据帧。 */
    header.DLC = 8; /* 数据长度为 8 字节，每个电机占 2 字节。 */

    /* 发送电流帧。 */
    uint32_t mailbox; /* mailbox 表示发送邮箱；芯片库会写回使用的邮箱编号。 */
    HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
}

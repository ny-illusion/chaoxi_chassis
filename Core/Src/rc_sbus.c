/*
 * 遥控模块：串口每收一个字节就拼帧，25 字节齐后解析 16 路通道。
 * SBUS=遥控接收机使用的数据协议；Frame=一帧完整数据；Channel=通道。
 * Buffer=暂存数组；Index=数组下标；Offset=偏移量；Valid=有效。
 * Failsafe=接收机失控保护；本程序把丢帧位和失控位任一置位都视为不可用。
 * C 语法：static 全局量只在本文件可见，局部 static 跨调用保留值；const 表示不能通过它修改数据。
 * volatile 提醒编译器每次访问实际内存，不保证多字段读写不被打断。
 * -> 访问指针所指结构体的成员；& 取地址；! 取反；&& 且；|| 或。
 */
#include "rc_sbus.h" /* 引入遥控数据类型和接收接口。 */

/* 中断收到的当前字节。 */
static uint8_t rx_byte;

/* 正在拼接的一帧数据。 */
static uint8_t rx_buffer[RC_SBUS_FRAME_LENGTH];

/* 当前写到第几个字节。 */
static uint8_t rx_index;

/* 下面几个变量保留为全局，方便调试器 Watch。 */
volatile uint16_t rc_ch[RC_SBUS_CHANNEL_COUNT]; /* 16 个通道原始值。 */
volatile uint32_t rc_last_ms;                   /* 最后一帧时间。 */
volatile uint32_t rc_cnt;                       /* 有效帧计数。 */
volatile uint8_t rc_ok;                         /* 1 表示收到过有效帧。 */
volatile uint8_t rc_failsafe = 1;               /* 1 表示接收机失控。 */

/* 启动 SBUS 接收。 */
/* 函数作用：huart 指定接收串口；只接受 USART1，清除有效状态后启动单字节中断接收。 */
void RC_SBUS_Start(UART_HandleTypeDef *huart)
{
    /* 只处理 USART1。 */
    if (huart == 0 || huart->Instance != USART1) return;

    /* 从帧首开始接收。 */
    rx_index = 0;

    /* 上电先按失控处理。 */
    rc_ok = 0;
    rc_failsafe = 1; /* 上电先把接收机状态当作失控，避免使用未接收的数据。 */

    /* 开启单字节中断接收。 */
    HAL_UART_Receive_IT(huart, &rx_byte, 1);
}

/* 收到一个 SBUS 字节。 */
/* 函数作用：串口收完一个字节后调用；拼接、校验、解出 16 路通道，最后再次安排下一个字节的接收。 */
void RC_SBUS_ReadByte(UART_HandleTypeDef *huart)
{
    /* 只处理 USART1。 */
    if (huart == 0 || huart->Instance != USART1) return;

    /* 保存当前字节。 */
    rx_buffer[rx_index] = rx_byte;

    /* 接收下标加一，下一个字节放到下一个数组位置。 */
    rx_index++;

    /* 攒满 25 字节后开始检查。 */
    if (rx_index >= RC_SBUS_FRAME_LENGTH)
    {
        /* 检查帧头和帧尾。 */
        if (rx_buffer[0] == 0x0F && (rx_buffer[24] & 0x0F) == 0x00)
        {
            /* 逐个解析 16 个通道。 */
            for (uint8_t i = 0; i < RC_SBUS_CHANNEL_COUNT; ++i)
            {
                /* 每路占 11 个二进制位，第 i 路从第 i×11 位开始；U 表示无符号整数常量。 */
                uint32_t bit = (uint32_t)i * 11U;

                /* >>3 相当于整数除以 8，算出所在字节；加 1 是跳过帧头字节。 */
                uint32_t byte_index = 1U + (bit >> 3);

                /* &7 只保留低 3 位，相当于对 8 取余，得到字节内第几位。 */
                uint32_t bit_offset = bit & 7U;

                /* 取出三个字节。 */
                uint32_t word = (uint32_t)rx_buffer[byte_index]
                              | ((uint32_t)rx_buffer[byte_index + 1] << 8) /* 第二个字节左移 8 位，再用按位或 | 合并。 */
                              | ((uint32_t)rx_buffer[byte_index + 2] << 16); /* 第三个字节左移 16 位，补齐可能跨字节的通道数据。 */

                /* 先右移到最低位，再用 &0x07FF 只保留 11 位；强制转成 16 位无符号整数。 */
                rc_ch[i] = (uint16_t)((word >> bit_offset) & 0x07FFU);
            }

            /* 0x0C 检查 bit2 丢帧、bit3 失控；任一位为 1 就标记不可用，并非只检查 failsafe 位。 */
            rc_failsafe = (uint8_t)((rx_buffer[23] & 0x0CU) != 0U);

            /* 标记已经收到有效帧。 */
            rc_ok = 1;

            /* 记录本帧接收时间。 */
            rc_last_ms = HAL_GetTick();

            /* 有效帧计数加一。 */
            rc_cnt++;

            /* 下一帧重新从 0 开始。 */
            rx_index = 0;
        }
        else /* 前面的条件不满足时，执行下面的分支。 */
        {
            /* 帧头或帧尾校验失败时，丢掉最前面的一个字节，重新寻找完整帧。 */
            for (uint8_t i = 0; i < RC_SBUS_FRAME_LENGTH - 1U; ++i)
                rx_buffer[i] = rx_buffer[i + 1]; /* 每个字节向前挪一格，丢掉最早字节，重新寻找帧边界。 */

            /* 保留最后 24 字节继续找帧头。 */
            rx_index = RC_SBUS_FRAME_LENGTH - 1U;
        }
    }

    /* 继续接收下一个字节。 */
    HAL_UART_Receive_IT(huart, &rx_byte, 1);
}

/* 复制一份遥控数据。 */
/* 函数作用：把通道、接收时刻、计数和状态复制到 snapshot；复制期间暂时屏蔽中断，保证数据一致。 */
void RC_SBUS_CopyData(RC_SBUS_Snapshot *snapshot)
{
    /* 空指针直接返回。 */
    if (snapshot == 0) return;

    /* 保存中断状态。 */
    uint32_t irq_state = __get_PRIMASK(); /* irq 是中断；保存进入本段代码前的中断屏蔽状态。 */

    /* 复制时关闭中断。 */
    __disable_irq(); /* 暂时禁止可屏蔽中断，避免复制或更新数据时被打断。 */

    /* 逐个复制通道。 */
    for (uint8_t i = 0; i < RC_SBUS_CHANNEL_COUNT; ++i)
        snapshot->channels[i] = rc_ch[i]; /* 把第 i 路原始通道复制到调用者提供的结构体。 */

    /* 复制时间、计数和状态。 */
    snapshot->last_frame_ms = rc_last_ms; /* 复制最近一帧的接收时刻，单位毫秒。 */
    snapshot->frame_count = rc_cnt; /* 复制累计收到的有效帧数。 */
    snapshot->valid = rc_ok; /* 复制是否曾收到有效帧的标记。 */
    snapshot->failsafe = rc_failsafe; /* 复制接收机的失控或丢帧标记。 */

    /* 恢复中断状态。 */
    __set_PRIMASK(irq_state); /* 恢复之前的中断屏蔽状态，不能简单地一律打开中断。 */
}

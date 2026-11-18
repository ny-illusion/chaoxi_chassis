#ifndef RC_SBUS_H /* 如果本头文件还没加载过，就继续加载。 */
#define RC_SBUS_H /* 标记已经加载，避免重复定义。 */

#include "main.h" /* 引入芯片库公共类型和错误处理声明。 */

/* SBUS 一帧的长度。 */
#define RC_SBUS_FRAME_LENGTH 25U

/* SBUS 的通道数量。 */
#define RC_SBUS_CHANNEL_COUNT 16U

/* 一帧遥控数据的快照。 */
typedef struct /* 把相关数据放到同一个结构体中，并为类型起名。 */
{
    uint16_t channels[RC_SBUS_CHANNEL_COUNT]; /* 16 个通道原始值。 */
    uint32_t last_frame_ms;                   /* 最后一帧的接收时间。 */
    uint32_t frame_count;                     /* 已接收的有效帧数。 */
    uint8_t valid;                            /* 1 表示收到过有效帧。 */
    uint8_t failsafe;                         /* 1 表示接收机报告失控。 */
} RC_SBUS_Snapshot;

/* 启动 SBUS 中断接收。 */
/* 函数作用：huart 指定接收串口；只接受 USART1，清除有效状态后启动单字节中断接收。 */
void RC_SBUS_Start(UART_HandleTypeDef *huart);

/* 收到一个字节后调用。 */
/* 函数作用：串口收完一个字节后调用；拼接、校验、解出 16 路通道，最后再次安排下一个字节的接收。 */
void RC_SBUS_ReadByte(UART_HandleTypeDef *huart);

/* 复制一份遥控数据。 */
/* 函数作用：把通道、接收时刻、计数和状态复制到 snapshot；复制期间暂时屏蔽中断，保证数据一致。 */
void RC_SBUS_CopyData(RC_SBUS_Snapshot *snapshot);

#endif

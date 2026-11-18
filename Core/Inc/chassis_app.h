#ifndef CHASSIS_APP_H /* 如果本头文件还没加载过，就继续加载。 */
#define CHASSIS_APP_H /* 标记已经加载，避免重复定义。 */

#include <stdint.h> /* 引入固定宽度整数，如 uint8_t 是 8 位无符号整数。 */

/* 初始化底盘应用。 */
/* 函数作用：初始化电机软件状态和指示灯；不负责初始化 CAN、串口或定时器硬件。 */
void ChassisApp_Init(void);

/* 写入云台相对角度。 */
/* relative_yaw_rad：云台相对底盘的角度，单位 rad。 */
/* 函数作用：写入云台相对底盘的水平角 relative_yaw_rad（弧度），同时记录接收时间和有效状态。 */
void ChassisApp_SetGimbalFeedback(float relative_yaw_rad);
/* 1 ms 执行一次底盘应用。 */
/* 函数作用：每隔 1 ms 调用一次：读取遥控和云台副本，判断失控，计算四轮目标并发送电流。 */
void ChassisApp_Run1ms(void);

#endif

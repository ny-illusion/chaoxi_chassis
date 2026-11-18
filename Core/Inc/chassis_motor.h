#ifndef CHASSIS_MOTOR_H /* 如果本头文件还没加载过，就继续加载。 */
#define CHASSIS_MOTOR_H /* 标记已经加载，避免重复定义。 */

#include "main.h" /* 引入芯片库公共类型和错误处理声明。 */
#include "chassis_motion.h" /* 引入底盘运动计算接口和数据类型。 */

/* 四个底盘电机共用这一帧发送电流。 */
#define CHASSIS_MOTOR_CAN_TX_ID 0x200U

/* 四个电调的反馈帧编号。 */
#define CHASSIS_MOTOR_CAN_RX_FIRST_ID 0x201U
#define CHASSIS_MOTOR_CAN_RX_LAST_ID 0x204U

/* 清空电机状态。 */
/* 函数作用：清空四个电机的反馈、目标、误差、积分和电流指令；此函数本身不发送 CAN。 */
void ChassisMotor_Init(void);

/* 解析一个电调反馈帧。 */
/* 函数作用：header 是接收到的帧头，data 是 8 字节内容；验证编号和格式后更新对应电机的转速。 */
void ChassisMotor_ReadCAN(const CAN_RxHeaderTypeDef *header, const uint8_t *data);

/* 计算速度 PI 并发送电流。 */
/* 函数作用：输入四轮 target_rpm、允许标志 enabled 和周期 dt_s（秒）；计算 PI 并尝试发送 0x200 电流帧。 */
void ChassisMotor_Update(const float target_rpm[CHASSIS_WHEEL_COUNT],
                         uint8_t enabled, float dt_s);

#endif

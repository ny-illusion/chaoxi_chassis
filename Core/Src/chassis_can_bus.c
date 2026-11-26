/**
 * @file chassis_can_bus.c
 * @brief CAN1 滤波启动与 FIFO0 接收分派
 * @board 底盘板
 * @vehicle_function 把板间遥控和电机反馈送到各自的接收模块
 * @inputs CAN1 FIFO0 帧头及载荷
 * @outputs CAN1 启动状态、遥控快照更新和电机反馈更新
 * @dependencies can.h、chassis_remote_communication.h、chassis_motor_driver.h
 * @note 重构说明：本文件行为与原代码保持一致。main 启动 CAN1 后才初始化控制状态和开启接收通知。
 */

#include "chassis_can_bus.h"
#include "can.h"
#include "chassis_motor_driver.h"
#include "chassis_remote_communication.h"

void chassis_can_start(void)
{
    CAN_FilterTypeDef filter = {0};
    filter.SlaveStartFilterBank = 14;  // CAN1 使用滤波器组 0～13
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;  // 接收所有编号，回调再按 ID 分派
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(&hcan1, &filter);  // HAL 库，写入 CAN 硬件滤波规则
    HAL_CAN_Start(&hcan1);  // HAL 库，启动 CAN 控制器
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)  // HAL 回调，项目实现；取出 CAN1 消息并分派遥控和电机帧
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    if (hcan->Instance != CAN1) return;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)  // HAL 库，读取 FIFO0 中待处理的帧数
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) break;  // HAL 库，从 FIFO0 取出帧头和数据
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC != 8U)
            continue;
        if (header.StdId == CHASSIS_REMOTE_CAN_ID)  // 0x301 是云台板每 10 ms 发送的遥控帧
        {
            ChassisRemote_ReceiveCAN(data);  // 项目函数，解包云台 0x301，并记录底盘本机接收时间
        }
        else
        {
            ChassisMotor_ReadCAN(&header, data);  // 项目函数，分派 0x201～0x204 轮速和 0x206 yaw 反馈
        }
    }
}

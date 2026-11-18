#ifndef CHASSIS_REMOTE_H
#define CHASSIS_REMOTE_H

#include "rc_sbus.h"
#include "chassis_motion.h"

/* rc 是本轮的遥控副本，now 是当前毫秒数；返回底盘要使用的输入。 */
ChassisInput ChassisRemote_Read(const RC_SBUS_Snapshot *rc, uint32_t now);

#endif

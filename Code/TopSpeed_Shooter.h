#ifndef _TOPSPEED_SHOOTER_H_
#define _TOPSPEED_SHOOTER_H_

#include "stm32h7xx_hal.h"
#include "TopSpeed_Typedef.h"

/*
    弹弓指令：
    1. 复位：SHOOTER_Reset，弹弓上升到上限位
    2. 蓄力: SHOOTER_set_mission,弹弓吸引磁铁并向下到达指定位置
    3. 释放: SHOOTER_inposition_callback
    由于复位需要较长时间，随后应当尽快进行复位

*/

void TS_Shooter_init(void);
void TS_Shooter_uplimit_trig_poll(void);
void TS_Shooter_inposition_callback(void);
void TS_Shooter_set_mission(int32_t force);

void TS_Shooter_Reset(void);
void TS_Shooter_enable_mag(void);
void TS_Shooter_disable_mag(void);

void TS_Shooter_mission_poll(void);
void TS_Shooter_response_callback(void);

extern SHOOTER_Handle_struct SHOOTER_Handle;

#endif // !_TOPSPEED_SHOOTER_H_
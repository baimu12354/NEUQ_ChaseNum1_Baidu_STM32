#ifndef _TOPSPEED_ACTION_H_
#define _TOPSPEED_ACTION_H_

#include "stm32h7xx_hal.h"
#include "TopSpeed_Typedef.h"

/*
    动作指令流程：
        维护一个状态机，储存每个动作模块的状态：待命、忙、超时；
        接收到上位机指令并解析；
        传入TS_set_Action分类讨论；
        判断目标模块的状态，若空闲，则下发指令，若忙，则返回错误，若超时，则返回错误；
        对于模块内部的通信错误，比如确认指令回复超时，由模块内部自动重传解决，丢包超过一定数量可以锁死模块
*/



void TS_set_Action(Action_CMD_struct Action,uint8_t action_check_get);
void TS_Action_in_position(SM_Handle_struct *H_SM);
void TS_Action_init(void);

void TS_Action_Store(void);
void TS_Action_Reset(void);

#endif // !_TOPSPEED_ACTION_H_
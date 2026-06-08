#ifndef _CHASENUM1_PUMP_H_
#define _CHASENUM1_PUMP_H_

#include "stm32h7xx_hal.h"
#include "ChaseNum1_Typedef.h"

// 气泵 - 全局句柄
extern Pump_Handle_Struct Pump_Handle;

// 气泵 - 初始化
void CN_Pump_init(void);
// 气泵 - 定时器初始化（PWM输出配置、脉冲输入捕获配置）
void CN_Pump_TIM_init(void);
// 气泵 - 任务回调
void CN_Pump_Task_Loop(void);
// 气泵 - 开启
void CN_Pump_Start(void);
// 气泵 - 关闭
void CN_Pump_Stop(void);

float CN_Pump_Get_Feedback_Freq_Hz(void);
uint32_t CN_Pump_Get_Feedback_Period_Ticks(void);
void CN_Pump_TIM15_PeriodElapsedCallback(void);




#endif 

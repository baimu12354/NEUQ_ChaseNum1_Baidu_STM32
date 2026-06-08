#ifndef _CHASENUM1_MENU_H_
#define _CHASENUM1_MENU_H_

#include "stm32h7xx_hal.h"

// 屏幕状态页初始化
void Menu_init(void);
// 屏幕状态页轮询刷新（需要在主循环中周期调用）。
void Menu_poll(void);

#endif

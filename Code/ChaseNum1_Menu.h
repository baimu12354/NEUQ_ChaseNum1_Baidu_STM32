#ifndef _CHASENUM1_MENU_H_
#define _CHASENUM1_MENU_H_

#include "stm32h7xx_hal.h"
#include "ChaseNum1_Typedef.h"

/*
    按键引脚定义：
        ENCA:PH2
        ENCB:PG12
        IN:PC1
        OUT:PB5

*/

extern uint8_t flag_server_started;
extern uint8_t flag_menu_refresh;

void Key_init(void);
void Key_poll(void);
void Menu_show(uint8_t highlight);
void Menu_poll(void);



#endif // !_CHASENUM1_MENU_H_
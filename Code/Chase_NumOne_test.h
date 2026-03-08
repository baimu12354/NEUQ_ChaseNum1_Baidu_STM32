#ifndef CHASE_NUMONE_TEST_H_
#define CHASE_NUMONE_TEST_H_

#include "stm32h7xx_hal.h"
#include "TopSpeed_Typedef.h"


typedef enum {
    position_loop,
    angle_loop,
    speed_loop
}ClosedLoop_Type;



void CNO_Chassis_closedloop_test(ClosedLoop_Type loopType);
void CNO_closedloop_show(ClosedLoop_Type loopType);
void CNO_GPIOC9_Low(void);

#endif

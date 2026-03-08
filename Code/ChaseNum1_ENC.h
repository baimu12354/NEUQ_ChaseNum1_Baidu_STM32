#ifndef _CHASENUM1_ENC_H_
#define _CHASENUM1_ENC_H_

#include "stm32h7xx_hal.h"

#include "ChaseNum1_Typedef.h"

extern int32_t ENC_data[4];

void TS_TIM2345_ENC_init(void);
void TS_read_ENC_INC(void);


#endif // !_CHASENUM1_ENC_H_
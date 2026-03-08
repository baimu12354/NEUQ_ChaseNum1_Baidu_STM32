#ifndef _TOPSPEED_ENC_H_
#define _TOPSPEED_ENC_H_

#include "stm32h7xx_hal.h"

#include "TopSpeed_Typedef.h"

extern int32_t ENC_data[4];

void TS_TIM2345_ENC_init(void);
void TS_read_ENC_INC(void);


#endif // !_TOPSPEED_ENC_H_
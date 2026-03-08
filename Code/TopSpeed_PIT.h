#ifndef _TOPSPEED_PIT_H_
#define _TOPSPEED_PIT_H_

#include "stm32h7xx_hal.h"
#include "TopSpeed_Typedef.h"

void TS_PIT1_CALLBACK(void);
void TS_PIT2_CALLBACK(void);
void TS_PIT_1_init_ms(uint16_t period,uint8_t priority);
void TS_PIT_2_init_ms(uint16_t period,uint8_t priority);
void TS_PIT_3_init_us(uint16_t period,uint8_t priority);
void TS_delay_us(uint32_t nus);
void TS_delay_ms(uint16_t nms);

extern volatile uint32_t systime_us;


#endif // !_TOPSPEED_PIT_H_
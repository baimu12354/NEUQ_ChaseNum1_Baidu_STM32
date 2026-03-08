#ifndef _CHASE_NUM1_PIT_H_
#define _CHASE_NUM1_PIT_H_

#include "stm32h7xx_hal.h"
#include "ChaseNum1_Typedef.h"

void CN_PIT1_CALLBACK(void);
void CN_PIT2_CALLBACK(void);
void CN_PIT_1_init_ms(uint16_t period,uint8_t priority);
void CN_PIT_2_init_ms(uint16_t period,uint8_t priority);
void CN_PIT_3_init_us(uint16_t period,uint8_t priority);
void CN_delay_us(uint32_t nus);
void CN_delay_ms(uint16_t nms);

extern volatile uint32_t systime_us;


#endif // !_CHASE_NUM1_PIT_H_
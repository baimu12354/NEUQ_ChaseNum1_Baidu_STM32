#ifndef _TOPSPEED_PUMP_H_
#define _TOPSPEED_PUMP_H_

#include "stm32h7xx_hal.h"
#include "TopSpeed_Typedef.h"

void TS_Pump_ADC_init(void);
uint16_t Read_Pump_ADC(void);
void TS_Pump_init(void);
void Pump_ADC_Read(void);
uint8_t TS_get_Suck(void);

void TS_Pump_Start(void);
void TS_Pump_Stop(void);

extern Pump_Handle_Struct Pump_Handle;


#endif // !_TOPSPEED_PUMP_H_
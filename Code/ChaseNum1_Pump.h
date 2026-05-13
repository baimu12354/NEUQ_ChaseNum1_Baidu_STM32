#ifndef _CHASENUM1_PUMP_H_
#define _CHASENUM1_PUMP_H_

#include "stm32h7xx_hal.h"
#include "ChaseNum1_Typedef.h"

typedef struct{
    uint8_t Pump_on;
    uint8_t IfSucked;
    uint32_t SuckStrength;
}Pump_Handle_Struct;

void CN_Pump_ADC_init(void);
uint16_t Read_Pump_ADC(void);
void CN_Pump_init(void);
void Pump_ADC_Read(void);
uint8_t CN_get_Suck(void);

void CN_Pump_Start(void);
void CN_Pump_Stop(void);

extern Pump_Handle_Struct Pump_Handle;


#endif 

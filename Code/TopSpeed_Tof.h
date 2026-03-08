#ifndef _TOPSPEED_TOF_H_
#define _TOPSPEED_TOF_H_

#include "stm32h7xx_hal.h"
#include "TopSpeed_Typedef.h"

#define TOF_USE_VL53L1X     (0)
#define TOF_USE_WT_VL53     (1)

void Tof_init(void);

extern TOF_Handle_struct TOF_Handle;

#endif // !_TOPSPEED_TOF_H_
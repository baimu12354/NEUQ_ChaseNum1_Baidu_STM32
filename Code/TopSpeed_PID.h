#ifndef _TOPSPEED_PID_H_
#define _TOPSPEED_PID_H_

#include "stm32h7xx_hal.h"
#include "TopSpeed_Typedef.h"



void init_PID_ctrler(PID_ctrler *ctrler,
                    float Kp,
                    float Ki,
                    float Kd,

                    float i_max,
                    float out_max);
void PID_ctrler_update(PID_ctrler *ctrler,float err);
void PID_ctrler_update_with_time(PID_ctrler *ctrler,float err,uint32_t us_time);
void PID_ctrler_reset(PID_ctrler *ctrler);

#endif // !_TOPSPEED_PID_H_
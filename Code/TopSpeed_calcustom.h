#ifndef _TOPSPEED_CALCUSTOM_H_
#define _TOPSPEED_CALCUSTOM_H_

#include "stm32h7xx_hal.h"

static inline int TS_linear_map_int(int input,int in_min,int in_max,int out_min,int out_max){
    return (int)((float)out_min + ((float)(input - in_min) * (out_max - out_min)) / (float)(in_max - in_min));
}

static inline float TS_linear_map_float(float input,
                                        float in_min,
                                        float in_max,
                                        float out_min,
                                        float out_max)
{
    return out_min
           + ((input - in_min) * (out_max - out_min))
             / (in_max - in_min);
}

static inline float TS_linear_map_with_mid_float(float input,
                                                 float in_min,
                                                 float in_mid,
                                                 float in_max,
                                                 float out_min,
                                                 float out_mid,
                                                 float out_max)
{
    if (input <= in_mid) {
        // 映射区间：in_min → in_mid 到 out_min → out_mid
        return out_min + ((input - in_min) * (out_mid - out_min)) / (in_mid - in_min);
    } else {
        // 映射区间：in_mid → in_max 到 out_mid → out_max
        return out_mid + ((input - in_mid) * (out_max - out_mid)) / (in_max - in_mid);
    }
}

float degree_to_rad(float degree);
float rad_to_degree(float rad);
float get_relative_angle(float now,float aim);

#endif // !_TOPSPEED_CALCUSTOM_H_
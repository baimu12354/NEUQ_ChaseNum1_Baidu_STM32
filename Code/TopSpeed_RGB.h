#ifndef _TOPSPEED_RGB_H_
#define _TOPSPEED_RGB_H_

#include "stm32h7xx_hal.h"
#include "TopSpeed_Typedef.h"

typedef enum{
    RGB_COLOR_NONE,
    RGB_COLOR_White,
    RGB_COLOR_Blue,
    RGB_COLOR_Green,
    RGB_COLOR_Yellow,
    RGB_COLOR_Orange,
}RGB_ColorTypeDef;

void TS_RGB_init(void);
void TS_RGB_show(RGB_ColorTypeDef color);

extern RGB_ColorTypeDef Color_Now;


#endif // !_TOPSPEED_RGB_H_
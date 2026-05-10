#include "stm32h7xx_hal.h"

#include "ChaseNum1_Typedef.h"
#include "ChaseNum1_calcustom.h"
#include "ChaseNum1_COM.h"
#include "ChaseNum1_ENC.h"
#include "ChaseNum1_IMU.h"
#include "ChaseNum1_Motor.h"
#include "ChaseNum1_Movement.h"
#include "ChaseNum1_PID.h"
#include "ChaseNum1_PIT.h"
#include "ChaseNum1_Trigger.h"
#include "ChaseNum1_TFT.h"
#include "ChaseNum1_Uart.h"
#include "ChaseNum1_FIFO.h"
#include "ChaseNum1_BusServo.h"
#include "ChaseNum1_MecArm.h"
#include "ChaseNum1_IMUcan.h"


#include "ChaseNum1_Menu.h"
#include "ChaseNum1_Mission.h"

#include "3rd_Party_SDK/wit_c_sdk/wit_c_sdk.h"
#include "3rd_Party_SDK/wit_c_sdk/REG.h"
#include "ChaseNum1_TFT180/lcd_init.h"
#include "ChaseNum1_TFT180/lcd.h"
#include "ChaseNum1_RC.h"

extern SYS_STATUS_STRUCT SYS_STATUS;


/*
**************************************************************************************
    timer分配如下：
    TIM1/8/16用于底盘电机控制(所有引脚全部可以重排，不可改变)
    // FL  C8  A9  TIM8CH3 TIM1CH2
    // FR  C9  A10 TIM8CH4 TIM1CH3
    // BL  C6  B8  TIM8CH1 TIM16CH1
    // BR  C7  A8  TIM8CH2 TIM1CH1

    FL  A10  A8  TIM1CH3 TIM1CH1
    FR  A9  C9  TIM1CH2 TIM8CH4
    BL  C6  C7  TIM8CH1 TIM8CH2
    BR  C8  B8  TIM8CH3 TIM16CH1

    TIM2/3/4/5用于底盘电机编码器解码(电机顺序可以重排，引脚不可改变)
    TIM2    PA0,1       //BR
    TIM3    PA6,7       //BL
    TIM4    PD12,13     //FL
    TIM5    PH10,11     //FR

    TIM6/7/13无输出通道，用于PIT中断

    TIM12/15用于pwm舵机驱动(所有引脚全部可以重排)
    TIM12   PH6 PH9
    TIM15   PE5 PE6
**************************************************************************************

**************************************************************************************
    UART分配如下：(所有模块可以重排，引脚不可改变)
    typedef enum{
        TS_UART_1,  //TX:PB14 RX:PB15   //调试口
        TS_UART_2,  //TX:PD5  RX:PD6    //陀螺仪通信
        TS_UART_3,  //TX:PB10 RX:PB11   //上位机通信
        TS_UART_4,  //TX:PC10 RX:PC11   //步进电机1通信
        TS_UART_5,  //TX:PB12 RX:PB13   //步进电机2通信
        TS_UART_6,  //TX:PG14 RX:PG9    //步进电机3通信/无刷驱动板通信
        TS_UART_7,  //TX:PB4  RX:PB3    //其他传感器/核心板
    }TS_UART_ENUM;
**************************************************************************************

**************************************************************************************
    机械限位触发io：任意io模拟，最好使用不同数字编号的io，字母编号可以相同
**************************************************************************************

**************************************************************************************
    硬件iic：SCL:PH4,SDA:PH5
**************************************************************************************

**************************************************************************************
    负压泵控制：与驱动配合，任意一个io即可
**************************************************************************************

**************************************************************************************
    可能的拓展：
    硬件spi：SCK:PA5 MISO:PA6 MOSI:PD7，还有其他选项
    软件spi：任意io模拟
    屏幕：硬件SPI，其余控制IO若干，由屏幕驱动决定
    按键、旋转编码器：任意IO模拟，最好使用外部中断（要求同机械限位触发io），旋转编码器可以由LPTIM1：PG12/PH2解码
**************************************************************************************

*/
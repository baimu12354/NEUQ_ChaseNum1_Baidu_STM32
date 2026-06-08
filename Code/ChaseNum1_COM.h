#ifndef _CHASENUM1_COM_H_
#define _CHASENUM1_COM_H_

#include "stm32h7xx_hal.h"
#include "ChaseNum1_Typedef.h"
#include "stdint.h"
#include "stdbool.h"

// NANO 协议使用 UART2。
#define NANO_PROTOCOL_UART2             2
// NANO 协议使用 UART7。
#define NANO_PROTOCOL_UART7             7
// NANO 协议串口选择；只改该宏即可切换 NANO 协议收包和回包串口。
#define NANO_PROTOCOL_UART_SELECT       NANO_PROTOCOL_UART2
// NANO 协议发送串口枚举，必须和 NANO_PROTOCOL_UART_SELECT 保持一致。
#if NANO_PROTOCOL_UART_SELECT == NANO_PROTOCOL_UART2
#define NANO_PROTOCOL_UART_PORT         CN_UART_2
#elif NANO_PROTOCOL_UART_SELECT == NANO_PROTOCOL_UART7
#define NANO_PROTOCOL_UART_PORT         CN_UART_7
#else
#error "NANO_PROTOCOL_UART_SELECT must be NANO_PROTOCOL_UART2 or NANO_PROTOCOL_UART7"
#endif

// Debug 初始化
void Debug_init(void);

// 通信初始化
void Main_COM_init(void);

//----------------------------------------vofa+ 上位机调试----------------------------------------//
typedef struct {
    // 角度环 (位置环)
    float target_angle;
    float Kp_angle;
    float Ki_angle;
    float Kd_angle;
    
    // 速度环
    float target_vel;
    float Kp_vel;
    float Ki_vel;
    float Kd_vel;
} Motor_Tuning_Params_t;

// 用于字节转浮点数的共用体
typedef union {
    float f_data;
    uint8_t bytes[4];
} VOFA_Float_Union;

extern volatile Motor_Tuning_Params_t motor_tune;
extern volatile bool param_updated_flag;  
extern uint8_t motor_tun_flag;

void VOFA_Single_Byte_Process(uint8_t rx_byte);
//----------------------------------------vofa+ 上位机调试----------------------------------------//


#endif // !_CHASENUM1_COM_H_

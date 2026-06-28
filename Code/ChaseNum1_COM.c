#include "stm32h7xx_hal.h"
#include "ChaseNum1_Include.h"
#include <string.h>
#include <stdio.h>

// 初始化下位机通信模块。
void Main_COM_init(void)
{
    // NANO 协议初始化会根据 NANO_PROTOCOL_UART_SELECT 自动选择 UART2 或 UART7。
    Nano_COM_Init();
}

#if NANO_PROTOCOL_UART_SELECT == NANO_PROTOCOL_UART2

// UART2 接收回调，用于 NANO 协议通信。
void CN_UART2_RX_CALLBACK(uint8_t dat)
{
    // UART2 收到的每个字节交给 NANO 协议状态机解析。
    Nano_COM_RX_Callback(dat);
}

// UART7 接收回调，用于串口助手调试。
void CN_UART7_RX_CALLBACK(uint8_t dat)
{
    VOFA_Single_Byte_Process(dat);
}

#elif NANO_PROTOCOL_UART_SELECT == NANO_PROTOCOL_UART7

// UART2 接收回调，当前未作为 NANO 协议入口。
void CN_UART2_RX_CALLBACK(uint8_t dat){}

// UART7 接收回调，用于 NANO 协议通信。
void CN_UART7_RX_CALLBACK(uint8_t dat)
{
    // UART7 收到的每个字节交给 NANO 协议状态机解析。
    Nano_COM_RX_Callback(dat);
}
#endif


//----------------------------------------vofa+ 上位机调试----------------------------------------//
// Debug调试初始化
void Debug_init(void){
#if NANO_PROTOCOL_UART_SELECT != NANO_PROTOCOL_UART7
   CN_UART7_init();
#endif
}

volatile Motor_Tuning_Params_t motor_tune = {0};
volatile bool param_updated_flag = 0;
uint8_t motor_tun_flag = 0;

static uint8_t vofa_receive_buffer[9];
static uint8_t vofa_receive_count = 0;

// VOFA+ 上位机解包函数
void VOFA_Single_Byte_Process(uint8_t rx_byte)
{
    if (vofa_receive_count == 0u) {
        if (rx_byte == 0xAAu) {
            vofa_receive_buffer[0] = rx_byte;
            vofa_receive_count = 1;
        }
        return;
    }

    if (vofa_receive_count == 1u) {
        if (rx_byte == 0x55u) {
            vofa_receive_buffer[1] = rx_byte;
            vofa_receive_count = 2;
        } else if (rx_byte == 0xAAu) {
            vofa_receive_count = 1;
        } else {
            vofa_receive_count = 0;
        }
        return;
    }

    if ((vofa_receive_count >= 2u) && (vofa_receive_count < 9u)) {
        vofa_receive_buffer[vofa_receive_count++] = rx_byte;
    }

    if (vofa_receive_count == 9u) {
        if ((vofa_receive_buffer[7] == 0x5Au) && (vofa_receive_buffer[8] == 0xA5u)) {
            uint8_t rx_id = vofa_receive_buffer[2];
            VOFA_Float_Union rx_float;

            rx_float.bytes[0] = vofa_receive_buffer[3];
            rx_float.bytes[1] = vofa_receive_buffer[4];
            rx_float.bytes[2] = vofa_receive_buffer[5];
            rx_float.bytes[3] = vofa_receive_buffer[6];

            switch (rx_id) {
                case 0: {
                    motor_tune.target_angle = rx_float.f_data;
                    motor_tun_flag = 1;
                    break;
                }
                case 1:
                    motor_tune.Kp_angle = rx_float.f_data;
                    break;
                case 2:
                    motor_tune.Ki_angle = rx_float.f_data;
                    break;
                case 3:
                    motor_tune.Kd_angle = rx_float.f_data;
                    break;
                case 4:
                    motor_tune.target_vel = rx_float.f_data;
                    break;
                case 5:
                    motor_tune.Kp_vel = rx_float.f_data;
                    break;
                case 6:
                    motor_tune.Ki_vel = rx_float.f_data;
                    break;
                case 7:
                    motor_tune.Kd_vel = rx_float.f_data;
                    break;
                default:
                    break;
            }

            param_updated_flag = true;
        }

        vofa_receive_count = 0;
    }
}
//----------------------------------------vofa+ 上位机调试----------------------------------------//

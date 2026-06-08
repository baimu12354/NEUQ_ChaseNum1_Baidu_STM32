#ifndef __CHASENUM1_UART_H_
#define __CHASENUM1_UART_H_

#include "stm32h7xx_hal.h"
#include "ChaseNum1_Typedef.h"

typedef enum{
    CN_UART_1,  //TX:PB14 RX:PB15   //调试口
    CN_UART_2,  //TX:PD5  RX:PD6    //NANO上位机通信
    CN_UART_3,  //TX:PB10 RX:PB11   //上位机通信
    CN_UART_4,  //TX:PC10 RX:PC11   //步进电机1通信
    CN_UART_5,  //TX:PB12 RX:PB13   //步进电机2通信
    CN_UART_6,  //TX:PG14 RX:PG9    //步进电机3通信
    CN_UART_7,  //TX:PB4  RX:PB3    //Tof通信
    CN_UART_8,  //TX:PE0  RX:PE1    // 舵机通信
}CN_UART_ENUM;

typedef enum{
    RX_WAITING_HEAD,
    RX_WAITING_TAIL,
}CN_UART_RX_STATUS_ENUM;

//+UART 诊断信息，用于屏幕显示通信是否发生硬件错误或发送 FIFO 堵塞。
typedef struct{
    uint32_t tx_fifo_write_fail_count;
    uint32_t uart_error_count;
    uint32_t uart_overrun_count;
    uint32_t uart_frame_error_count;
    uint32_t uart_noise_error_count;
    uint16_t tx_fifo_occupied;
    uint16_t tx_fifo_space;
    uint32_t hal_error_code;
}CN_UART_Diag_t;

void CN_UART1_init(void);
void CN_UART2_init(void);
void CN_UART3_init(void);
void CN_UART4_init(uint32_t baud);
void CN_UART5_init(void);
void CN_UART6_init(void);
void CN_UART7_init(void);
void CN_UART8_init(void);
void CN_UART_deinit(CN_UART_ENUM Moudle);

void CN_UART1_RX_CALLBACK(uint8_t dat);
void CN_UART2_RX_CALLBACK(uint8_t dat);
void CN_UART3_RX_CALLBACK(uint8_t dat);
void CN_UART4_RX_CALLBACK(uint8_t dat);
void CN_UART5_RX_CALLBACK(uint8_t dat);
void CN_UART6_RX_CALLBACK(uint8_t dat);
void CN_UART7_RX_CALLBACK(uint8_t dat);
void CN_UART8_RX_CALLBACK(uint8_t dat);

// 串口阻塞发送字符串
void CN_UART_send_string_block(CN_UART_ENUM Moudle,const uint8_t *str);
// 串口非阻塞发送字符串
void CN_UART_send_string_IT(CN_UART_ENUM Moudle,uint8_t *str);
// 串口非阻塞发送数据
void CN_UART_send_data_IT(CN_UART_ENUM Moudle,uint8_t *data,uint32_t len);
// 串口FIFO发送数据
void CN_UART_write_fifo(CN_UART_ENUM Moudle,uint8_t *data,uint32_t len);
// 串口FIFO发送字符串
void CN_UART_write_string_FIFO(CN_UART_ENUM Moudle,uint8_t *str);
//+读取指定 UART 的诊断信息。
void CN_UART_Get_Diag(CN_UART_ENUM Moudle,CN_UART_Diag_t *diag);

#endif // __CHASENUM1_UART_H_

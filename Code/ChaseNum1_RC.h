#ifndef CHASENUM1_RC_H_
#define CHASENUM1_RC_H_

#include "stm32h7xx_hal.h"
#include "ChaseNum1_Typedef.h"

#define SBUS_UART_BAUDRATE                  (100000)            // 指定 SBUS 串口所使用的的串口波特率 (用户不可修改)
#define UART_RECEVIER_CHANNEL_NUM           ( 6 )               // 定义遥控器通道数量 (用户不可修改)

#define REV_DATA_LEN                        ( 25   )            // SBUS帧长
#define FRAME_STAR                          ( 0X0F )            // 帧头信息
#define FRAME_END                           ( 0X00 )            // 帧尾信息
#define SBUS_NORMAL_STATE                   ( 0X00 )            // 正常状态
#define SBUS_ABNORMAL_STATE                 ( 0X04 )            // 失控状态

typedef struct
{
    uint16_t channel[UART_RECEVIER_CHANNEL_NUM];                  // CH1-CH6通道数据
    uint8_t  state;                                               // 遥控器状态(1表示正常，否则表示失控)
    uint8_t  finsh_flag;                                          // 1：表示成功接收到一帧遥控器数据
}uart_receiver_struct;

extern uart_receiver_struct uart_receiver;                      // 串口接收机通道数据与状态

extern uint8_t debug_sbus_buffer[100];
extern uint8_t stop_record;

void TIM17_RC_Init(void);
uint32_t TIM17_RC_Get(void);
void TIM17_RC_Clear(void);

// RC接收机初始化
void uart_receiver_init(void);

// 获取两次调用uart_receiver_interval_time函数时间间隔  (us)
static uint32_t uart_receiver_interval_time (void);

// SBUS数据解析
static void uart_receiver_analysis (uart_receiver_struct *remote_data,uint8_t * buffer);

// 串口接收机模块 串口中断回调函数
void uart_receiver_callback(uint8_t dat);

#endif
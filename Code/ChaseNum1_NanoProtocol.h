#ifndef _CHASE_NUM1_NANO_PROTOCOL_H_
#define _CHASE_NUM1_NANO_PROTOCOL_H_

#include "stm32h7xx_hal.h"

//+NANO 协议诊断快照，用于屏幕显示通信链路和主循环状态。
typedef struct {
    uint32_t ms_tick;
    uint32_t last_loop_ms;
    uint32_t task_loop_count;
    uint32_t rx_byte_count;
    uint32_t rx_frame_count;
    uint32_t rx_process_count;
    uint32_t rx_drop_count;
    uint32_t rx_busy_drop_count;
    uint32_t rx_bad_check_count;
    uint32_t rx_bad_tail_count;
    uint32_t rx_bad_len_count;
    uint32_t tx_frame_count;
    uint16_t rx_ring_occupied;
    uint8_t package_ready;
    uint8_t rx_state;
    uint8_t system_state;
    uint8_t chassis_state;
    uint8_t arm_state;
    uint8_t pump_state;
} Nano_COM_Diag_t;

// NANO 协议初始化，内部使用 UART2。
void Nano_COM_Init(void);

// NANO 协议主循环任务，负责处理完整命令和模块完成事件。
void Nano_COM_Task_Loop(void);

// NANO 协议 1ms 任务，负责超时计时和释放等待计时。
void Nano_COM_1ms_Tick(void);

// UART2 每收到一个字节后调用该函数写入软件缓冲区，协议解析在主循环中完成。
void Nano_COM_RX_Callback(uint8_t dat);

// 主流程确认全部外设初始化完成后调用，状态转为空闲并主动上报。
void Nano_COM_Notify_Init_Done(void);
//+读取 NANO 协议诊断快照。
void Nano_COM_Get_Diag(Nano_COM_Diag_t *diag);

#endif

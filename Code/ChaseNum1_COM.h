#ifndef _CHASENUM1_COM_H_
#define _CHASENUM1_COM_H_

#include "stm32h7xx_hal.h"
#include "ChaseNum1_Typedef.h"
#include "stdint.h"
#include "stdbool.h"

/*
 * Chassis test protocol for the host computer.
 *
 * UART3: 115200, 8 data bits, no parity, 1 stop bit.
 * Frame: HEAD(AA 55) VERSION SEQ CMD LEN PAYLOAD CHECK TAIL(0D 0A)
 * CHECK is uint8 sum of VERSION, SEQ, CMD, LEN and PAYLOAD bytes.
 * Multi-byte numbers are little-endian. float is IEEE-754 float32.
 */
#define MAIN_COM_FRAME_HEAD1             (0xAAu)
#define MAIN_COM_FRAME_HEAD2             (0x55u)
#define MAIN_COM_FRAME_TAIL1             (0x0Du)
#define MAIN_COM_FRAME_TAIL2             (0x0Au)
#define MAIN_COM_FRAME_VERSION           (0x01u)
#define MAIN_COM_MAX_PAYLOAD_LEN         (64u)
#define MAIN_COM_FRAME_OVERHEAD          (9u)

#define MAIN_COM_CMD_HELLO               (0x01u)
#define MAIN_COM_CMD_BRAKE               (0x02u)
#define MAIN_COM_CMD_RUN                 (0x03u)
#define MAIN_COM_CMD_GET_STATUS          (0x04u)
#define MAIN_COM_CMD_SET_YAW             (0x05u)
#define MAIN_COM_CMD_SET_FEEDBACK        (0x06u)

#define MAIN_COM_RESP_ACK                (0x81u)
#define MAIN_COM_RESP_ERR                (0x82u)
#define MAIN_COM_RESP_STATUS             (0x83u)

#define MAIN_COM_ERR_NONE                (0x00u)
#define MAIN_COM_ERR_BAD_CMD             (0x01u)
#define MAIN_COM_ERR_BAD_LEN             (0x02u)
#define MAIN_COM_ERR_BAD_CHECK           (0x03u)
#define MAIN_COM_ERR_BUSY                (0x04u)
#define MAIN_COM_ERR_TIMEOUT             (0x05u)

#define MAIN_COM_RUN_PAYLOAD_LEN         (12u)
#define MAIN_COM_SET_YAW_PAYLOAD_LEN     (4u)
#define MAIN_COM_SET_FEEDBACK_PAYLOAD_LEN (2u)
#define MAIN_COM_STATUS_PAYLOAD_LEN      (21u)

#define MAIN_COM_CMD_TIMEOUT_MS          (500u)
#define MAIN_COM_FEEDBACK_DEFAULT_MS     (0u)
#define MAIN_COM_FEEDBACK_MIN_MS         (20u)
#define MAIN_COM_FEEDBACK_MAX_MS         (1000u)

#define MAIN_COM_MAX_VX                  (50.0f)
#define MAIN_COM_MAX_VY                  (50.0f)
#define MAIN_COM_MAX_WZ_DEG_S            (200.0f)

// 通信初始化
void Main_COM_init(void);
// 上位机通信回调
void Main_COM_Response_CALLBACK(void);
// 下位机通用发送帧函数
void Main_COM_send_frame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len);
// 上位机通信接收解包函数
void Main_COM_RX_CALLBACK(uint8_t dat);

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

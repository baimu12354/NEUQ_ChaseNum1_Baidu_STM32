#ifndef __MECARM_PROTOCOL_H__
#define __MECARM_PROTOCOL_H__

#include "stm32h7xx_hal.h"

// 定义一个全局结构体用于存放上位机下发的命令
typedef struct {
    float th0;
    float x;
    float y;
    float th3;
    uint8_t run_flag; // 1: 正常运行, 0: 急停
    uint8_t new_cmd_flag; // 1表示收到了新指令，主循环处理完后清0
} HostCmd_t;

// 解析状态机枚举
typedef enum {
    STATE_WAIT_H1,
    STATE_WAIT_H2,
    STATE_WAIT_CMD,
    STATE_RECEIVE_DATA,
    STATE_WAIT_CHECK,
    STATE_WAIT_T1,
    STATE_WAIT_T2
} ProtocolState_t;

extern HostCmd_t g_HostCmd;

// 解析比解包函数
void ARM_RX_Callback(uint8_t dat);

// 机械臂初始化
void Arm_Init(void);

// 机械臂解算
uint8_t Arm_Inverse_Kinematics(float plane_x, float plane_y, float *out_th1, float *out_th2);

// 物理映射
void Arm_Execute_Move(float v_th0, float v_th1, float v_th2, float v_th3);

// 机械臂指令测试
void Arm_Standalone_Test(void);

// 机械臂指令轮询
void Arm_Task_Loop(void); 

#endif

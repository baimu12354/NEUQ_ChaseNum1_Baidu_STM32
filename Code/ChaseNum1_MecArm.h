#ifndef __MECARM_PROTOCOL_H__
#define __MECARM_PROTOCOL_H__

#include "stm32h7xx_hal.h"

// 定义一个全局结构体用于存放上位机下发的命令
// 0x01 运动命令载荷：world_x、world_y、world_z、suc_dir_deg，共 4 个 float，单位为 cm/deg
typedef struct {
    float world_x;     // 吸盘末端世界 X 坐标，单位 cm
    float world_y;     // 吸盘末端世界 Y 坐标，单位 cm
    float world_z;     // 吸盘末端世界 Z 坐标，单位 cm
    float suc_dir_deg; // 吸盘在目标竖直平面内的绝对方向，平面+深度=90deg, +Z=180deg
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

// 机械臂解算：输入吸盘末端世界坐标与吸盘竖直平面内绝对方向，输出四个虚拟角度
uint8_t Arm_Inverse_Kinematics(float world_x, float world_y, float world_z, float suc_dir_deg,
                               float *out_th0, float *out_th1, float *out_th2, float *out_th3);

// 物理映射
void Arm_Execute_Move(float v_th0, float v_th1, float v_th2, float v_th3);

// 机械臂指令测试
void Arm_Standalone_Test(void);

// 机械臂指令轮询
void Arm_Task_Loop(void); 

#endif

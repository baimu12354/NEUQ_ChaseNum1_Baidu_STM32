#ifndef __MECARM_PROTOCOL_H__
#define __MECARM_PROTOCOL_H__

#include "stm32h7xx_hal.h"

// 机械臂内部停止命令字：停止当前规划并锁持全部舵机。
#define ARM_HOST_CMD_STOP 0x00U
// 机械臂内部运动命令字：携带吸盘末端世界坐标和吸盘方向。
#define ARM_HOST_CMD_MOVE 0x01U
// 机械臂内部默认休息命令字：回到默认休息姿态。
#define ARM_HOST_CMD_FOLD 0x02U

//+机械臂没有新的内部事件。
#define ARM_STATUS_EVENT_NONE      0U
//+机械臂目标动作已经完成。
#define ARM_STATUS_EVENT_DONE      1U
//+机械臂默认休息动作已经完成。
#define ARM_STATUS_EVENT_FOLD_DONE 2U
//+机械臂命令被拒绝。
#define ARM_STATUS_EVENT_REJECTED  3U
//+机械臂已经停止并锁持。
#define ARM_STATUS_EVENT_STOPPED   4U

// 定义一个全局结构体用于存放外部调度层投递给机械臂任务的命令。
// 0x01 运动命令载荷：world_x、world_y、world_z、suc_dir_deg，共 4 个 float，单位为 cm/deg
typedef struct {
    float world_x;     // 吸盘末端世界 X 坐标，单位 cm
    float world_y;     // 吸盘末端世界 Y 坐标，单位 cm
    float world_z;     // 吸盘末端世界 Z 坐标，单位 cm
    float suc_dir_deg; // 吸盘在目标竖直平面内的绝对方向，平面+深度=90deg, +Z=180deg
    uint8_t cmd_type; // 最近一次收到的机械臂内部命令字
    uint8_t run_flag; // 兼容保留字段，1 表示非急停命令，0 表示急停命令
    uint8_t new_cmd_flag; // 1表示收到了新指令，主循环处理完后清0
} HostCmd_t;

extern HostCmd_t g_HostCmd;

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

//+获取机械臂规划器是否仍在执行。
uint8_t Arm_Is_Planner_Active(void);

//+读取机械臂当前逻辑末端位置和关节角。
void Arm_Get_Logical_Pose(float *x, float *y, float *z, float *dir,
                          float *th0, float *th1, float *th2, float *th3);

//+读取并清除机械臂内部事件。
uint8_t Arm_Fetch_Status_Event(uint8_t *event);

#endif

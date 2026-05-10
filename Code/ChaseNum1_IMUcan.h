#ifndef __CHASENUM1_IMUCAN_H
#define __CHASENUM1_IMUCAN_H

#include "stm32h7xx_hal.h"

//----------------------------- 通信协议 ----------------------------- //
#define ACCEL_CAN_MAX   (235.2f)
#define ACCEL_CAN_MIN   (-235.2f)
#define GYRO_CAN_MAX    (34.88f)
#define GYRO_CAN_MIN    (-34.88f)
#define PITCH_CAN_MAX   (90.0f)
#define PITCH_CAN_MIN   (-90.0f)
#define ROLL_CAN_MAX    (180.0f)
#define ROLL_CAN_MIN    (-180.0f)
#define YAW_CAN_MAX     (180.0f)
#define YAW_CAN_MIN     (-180.0f)
#define Quaternion_MIN  (-1.0f)
#define Quaternion_MAX  (1.0f)

#define CMD_READ  0
#define CMD_WRITE 1

// IMU指令 寄存器 ID 定义
typedef enum 
{
    REBOOT_IMU = 0,
    ACCEL_DATA,
    GYRO_DATA,
    EULER_DATA,
    QUAT_DATA,
    SET_ZERO,
    ACCEL_CALI,
    GYRO_CALI,
    MAG_CALI,
    CHANGE_COM,
    SET_DELAY,
    CHANGE_ACTIVE,
    SET_BAUD,
    SET_CAN_ID,
    SET_MST_ID,
    DATA_OUTPUT_SELECTION,
    SAVE_PARAM = 254,
    RESTORE_SETTING = 255
} reg_id_e;
//----------------------------- 通信协议 ----------------------------- //

//----------------------------- 对外接口 ----------------------------- //
// IMU 数据结构体，用来存放解析后的姿态数据
typedef struct
{
    uint8_t can_id;         // 陀螺仪的 CAN ID (通常为 0x01)
    uint8_t mst_id;         // 主机的 CAN ID (通常为 0x11)
    
    FDCAN_HandleTypeDef *can_handle; // 绑定的 CAN 句柄
    
    float pitch;            // 俯仰角
    float roll;             // 横滚角
    float yaw;              // 偏航角

    float gyro[3];          // 存放三轴角速度
    float accel[3];         // 存放三轴加速度
    float q[4];             // 存放四元数
} imu_t;

// 全局变量和句柄
extern imu_t imu;
extern FDCAN_HandleTypeDef hfdcan1;

// 初始化模块：包含 FDCAN1 硬件初始化、引脚配置、中断开启以及 IMU 对象绑定 
void IMU_CAN_Init(void);

// 请求数据的指令函数 
void imu_request_accel(void);
void imu_request_gyro(void);
void imu_request_euler(void);
void imu_request_quat(void);
//----------------------------- 对外接口 ----------------------------- //

#endif
#ifndef __CHASENUM1_SERVO_H__
#define __CHASENUM1_SERVO_H__

#include "ChaseNum1_Typedef.h" // 包含你的底层串口定义头文件

#include <stdint.h>
#include <stdbool.h>

#define BUS_SERVO_UART CN_UART_7

/* ---------------- 之前的发送宏定义保持不变 ---------------- */
#define SERVO_CMD_PING                  0x01 // 通讯检测
#define SERVO_CMD_SIMPLE_SINGLE_ANGLE   0x08 // 简易单圈角度控制
#define SERVO_CMD_ADV_SINGLE_TIME       0x0b // 高级单圈角度控制 (基于时间)
#define SERVO_CMD_ADV_SINGLE_SPEED      0x0c // 高级单圈角度控制 (基于速度)
#define SERVO_CMD_READ_SINGLE_ANGLE     0x0a // 当前单圈角度读取
#define SERVO_CMD_SIMPLE_MULTI_ANGLE    0x0d // 简易多圈角度控制
#define SERVO_CMD_ADV_MULTI_TIME        0x0e // 高级多圈角度控制 (基于时间)
#define SERVO_CMD_ADV_MULTI_SPEED       0x0f // 高级多圈角度控制 (基于速度)
#define SERVO_CMD_READ_MULTI_ANGLE      0x10 // 当前多圈角度读取
#define SERVO_CMD_RESET_TURNS           0x11 // 重置圈数
#define SERVO_CMD_DAMPING               0x09 // 阻尼控制
#define SERVO_CMD_STOP                  0x18 // 停止指令
#define SERVO_CMD_SYNC                  0x19 // 同步指令
#define SERVO_CMD_ASYNC_WRITE           0x12 // 异步写入指令
#define SERVO_CMD_ASYNC_EXEC            0x13 // 异步执行指令
#define SERVO_CMD_READ_DATA             0x03 // 数据读取
#define SERVO_CMD_MONITOR_DATA          0x16 // 数据监控
#define SERVO_CMD_SET_ORIGIN            0x17 // 设置原点
#define SERVO_CMD_CUSTOM_CONFIG         0x04 // 自定义配置参数

#define SERVO_STOP_MODE_RELEASE         0x10 // 停止后释放锁力
#define SERVO_STOP_MODE_HOLD            0x11 // 停止后保持锁力
#define SERVO_STOP_MODE_DAMPING         0x12 // 停止后进入阻尼控制

#define SERVO_ASYNC_ACTION_EXEC         0x00 // 执行
#define SERVO_ASYNC_ACTION_CANCEL       0x01 // 取消

/* ---------------- 接收状态机枚举 ---------------- */
typedef enum {
    SERVO_RX_STATE_HEADER1 = 0,
    SERVO_RX_STATE_HEADER2,
    SERVO_RX_STATE_CMD,
    SERVO_RX_STATE_LEN,
    SERVO_RX_STATE_CONTENT,
    SERVO_RX_STATE_CHECKSUM
} Servo_Rx_State_e;

/* ---------------- 舵机状态数据结构体 ---------------- */
// 用于存放解析后的舵机数据，方便应用层直接读取
typedef struct {
    uint8_t  is_online;     // 是否在线
    int32_t  position;      // 当前角度 (0.1°)
    int16_t  turns;         // 当前圈数
    uint16_t voltage;       // 电压 (mV)
    uint16_t current;       // 电流 (mA)
    uint16_t power;         // 功率 (mW)
    uint16_t temperature;   // 温度 (ADC值)
    uint8_t  status_err;    // 状态标志位
} Servo_Info_t;

extern Servo_Info_t g_ServoInfo[255]; // 假设最多支持255个ID的舵机信息记录

void Servo_Init(void);
void Servo_Send_Packet(uint8_t cmd_id, uint8_t *content, uint8_t length);
void Servo_Ping(uint8_t servo_id);
void Servo_Simple_Single_Turn(uint8_t servo_id, int16_t position, uint16_t time, uint16_t power);
void Servo_Advanced_Single_Turn_Time(uint8_t servo_id, int16_t position, uint16_t time, uint16_t accel, uint16_t decel, uint16_t power);
void Servo_Advanced_Single_Turn_Speed(uint8_t servo_id, int16_t position, uint16_t speed, uint16_t accel, uint16_t decel, uint16_t power);
void Servo_Read_Single_Turn_Angle(uint8_t servo_id);
void Servo_Simple_Multi_Turn(uint8_t servo_id, int32_t position, uint32_t time, uint16_t power);
void Servo_Advanced_Multi_Turn_Time(uint8_t servo_id, int32_t position, uint32_t time, uint16_t accel, uint16_t decel, uint16_t power);
void Servo_Advanced_Multi_Turn_Speed(uint8_t servo_id, int32_t position, uint16_t speed, uint16_t accel, uint16_t decel, uint16_t power);
void Servo_Read_Multi_Turn_Angle(uint8_t servo_id);
void Servo_Reset_Turns(uint8_t servo_id);
void Servo_Damping_Control(uint8_t servo_id, uint16_t power);
void Servo_Stop(uint8_t servo_id, uint8_t mode, uint16_t power);
void Servo_Async_Write(void);
void Servo_Async_Execute(uint8_t action);
void Servo_Read_Data(uint8_t servo_id, uint8_t data_id);
void Servo_Monitor_Data(uint8_t servo_id);
void Servo_Set_Origin(uint8_t servo_id, uint8_t reset_mode);
void Servo_Custom_Config(uint8_t servo_id, uint8_t data_id, uint8_t *data_content, uint8_t content_len);

/* ---------------- 接收与解析函数声明 ---------------- */
void Servo_Receive_Byte(uint8_t byte);
void Servo_Parse_Packet(uint8_t cmd_id, uint8_t length, uint8_t *content);


/* ---------------- 同步指令专用的结构体定义 ---------------- */

// 简易单圈角度控制 (0x08) 同步参数结构体
typedef struct {
    uint8_t  servo_id;  // 舵机ID
    int16_t  position;  // 目标位置 (0.1°)
    uint16_t time;      // 运动时间 (ms)
    uint16_t power;     // 执行功率 (mW)
} Sync_Simple_Single_t;

// 高级单圈角度控制-基于时间 (0x0B) 同步参数结构体
typedef struct {
    uint8_t  servo_id;
    int16_t  position;
    uint16_t time;
    uint16_t accel;     // 加速时间 (ms)
    uint16_t decel;     // 减速时间 (ms)
    uint16_t power;
} Sync_Adv_Single_Time_t;

// 高级单圈角度控制-基于速度 (0x0C) 同步参数结构体
typedef struct {
    uint8_t  servo_id;
    int16_t  position;
    uint16_t speed;     // 运动速度 (0.1°/s)
    uint16_t accel;
    uint16_t decel;
    uint16_t power;
} Sync_Adv_Single_Speed_t;

// 简易多圈角度控制 (0x0D) 同步参数结构体
typedef struct {
    uint8_t  servo_id;
    int32_t  position;  // 目标位置 (0.1°) 32位
    uint32_t time;      // 运动时间 (ms) 32位
    uint16_t power;
} Sync_Simple_Multi_t;

// 高级多圈角度控制-基于时间 (0x0E) 同步参数结构体
typedef struct {
    uint8_t  servo_id;
    int32_t  position;
    uint32_t time;
    uint16_t accel;
    uint16_t decel;
    uint16_t power;
} Sync_Adv_Multi_Time_t;

// 高级多圈角度控制-基于速度 (0x0F) 同步参数结构体
typedef struct {
    uint8_t  servo_id;
    int32_t  position;
    uint16_t speed;
    uint16_t accel;
    uint16_t decel;
    uint16_t power;
} Sync_Adv_Multi_Speed_t;

/* ---------------- 同步指令高级封装函数声明 ---------------- */
void Servo_Sync_Simple_Single_Turn(Sync_Simple_Single_t *targets, uint8_t count);
void Servo_Sync_Adv_Single_Turn_Time(Sync_Adv_Single_Time_t *targets, uint8_t count);
void Servo_Sync_Adv_Single_Turn_Speed(Sync_Adv_Single_Speed_t *targets, uint8_t count);
void Servo_Sync_Simple_Multi_Turn(Sync_Simple_Multi_t *targets, uint8_t count);
void Servo_Sync_Adv_Multi_Turn_Time(Sync_Adv_Multi_Time_t *targets, uint8_t count);
void Servo_Sync_Adv_Multi_Turn_Speed(Sync_Adv_Multi_Speed_t *targets, uint8_t count);


#endif /* __CHASENUM1_SERVO_H__ */

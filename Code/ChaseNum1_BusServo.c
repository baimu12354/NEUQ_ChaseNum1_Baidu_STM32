#include "ChaseNum1_Include.h"

#include <string.h>

//--------------------------------发送模块--------------------------------//
// 舵机初始化
void Servo_Init(void){
    CN_UART3_init();
}

// 舵机串口组包发送函数
void Servo_Send_Packet(uint8_t cmd_id, uint8_t *content, uint8_t length) {
    uint8_t buffer[128]; // 保证足够容纳同步指令等较长包
    uint32_t sum = 0;
    
    buffer[0] = 0x12; // Header 1
    buffer[1] = 0x4C; // Header 2
    buffer[2] = cmd_id;
    buffer[3] = length;
    
    if (length > 0 && content != NULL) {
        memcpy(&buffer[4], content, length);
    }
    
    // 计算校验和: Σ(Byte[0..4+length-1]) % 256
    for (int i = 0; i < 4 + length; i++) {
        sum += buffer[i];
    }
    buffer[4 + length] = sum & 0xFF;
    
    // 写入串口 FIFO 发送
    CN_UART_write_fifo(BUS_SERVO_UART, buffer, 4 + length + 1);
}

// 通讯检测
void Servo_Ping(uint8_t servo_id) {
    uint8_t content[1] = {servo_id};
    Servo_Send_Packet(SERVO_CMD_PING, content, 1);
}

// 简易单圈控制
void Servo_Simple_Single_Turn(uint8_t servo_id, int16_t position, uint16_t time, uint16_t power) {
    uint8_t content[7];
    content[0] = servo_id;
    content[1] = (uint8_t)(position & 0xFF);
    content[2] = (uint8_t)((position >> 8) & 0xFF);
    content[3] = (uint8_t)(time & 0xFF);
    content[4] = (uint8_t)((time >> 8) & 0xFF);
    content[5] = (uint8_t)(power & 0xFF);
    content[6] = (uint8_t)((power >> 8) & 0xFF);
    Servo_Send_Packet(SERVO_CMD_SIMPLE_SINGLE_ANGLE, content, 7);
}

// 高级单圈控制（基于时间）
void Servo_Advanced_Single_Turn_Time(uint8_t servo_id, int16_t position, uint16_t time, uint16_t accel, uint16_t decel, uint16_t power) {
    uint8_t content[11];
    content[0] = servo_id;
    content[1] = (uint8_t)(position & 0xFF);
    content[2] = (uint8_t)((position >> 8) & 0xFF);
    content[3] = (uint8_t)(time & 0xFF);
    content[4] = (uint8_t)((time >> 8) & 0xFF);
    content[5] = (uint8_t)(accel & 0xFF);
    content[6] = (uint8_t)((accel >> 8) & 0xFF);
    content[7] = (uint8_t)(decel & 0xFF);
    content[8] = (uint8_t)((decel >> 8) & 0xFF);
    content[9] = (uint8_t)(power & 0xFF);
    content[10] = (uint8_t)((power >> 8) & 0xFF);
    Servo_Send_Packet(SERVO_CMD_ADV_SINGLE_TIME, content, 11);
}

// 高级单圈控制（基于速度）
void Servo_Advanced_Single_Turn_Speed(uint8_t servo_id, int16_t position, uint16_t speed, uint16_t accel, uint16_t decel, uint16_t power) {
    uint8_t content[11];
    content[0] = servo_id;
    content[1] = (uint8_t)(position & 0xFF);
    content[2] = (uint8_t)((position >> 8) & 0xFF);
    content[3] = (uint8_t)(speed & 0xFF);
    content[4] = (uint8_t)((speed >> 8) & 0xFF);
    content[5] = (uint8_t)(accel & 0xFF);
    content[6] = (uint8_t)((accel >> 8) & 0xFF);
    content[7] = (uint8_t)(decel & 0xFF);
    content[8] = (uint8_t)((decel >> 8) & 0xFF);
    content[9] = (uint8_t)(power & 0xFF);
    content[10] = (uint8_t)((power >> 8) & 0xFF);
    Servo_Send_Packet(SERVO_CMD_ADV_SINGLE_SPEED, content, 11);
}

// 单圈角度读取
void Servo_Read_Single_Turn_Angle(uint8_t servo_id) {
    uint8_t content[1] = {servo_id};
    Servo_Send_Packet(SERVO_CMD_READ_SINGLE_ANGLE, content, 1);
}

// 简易多圈控制
void Servo_Simple_Multi_Turn(uint8_t servo_id, int32_t position, uint32_t time, uint16_t power) {
    uint8_t content[11];
    content[0] = servo_id;
    content[1] = (uint8_t)(position & 0xFF);
    content[2] = (uint8_t)((position >> 8) & 0xFF);
    content[3] = (uint8_t)((position >> 16) & 0xFF);
    content[4] = (uint8_t)((position >> 24) & 0xFF);
    content[5] = (uint8_t)(time & 0xFF);
    content[6] = (uint8_t)((time >> 8) & 0xFF);
    content[7] = (uint8_t)((time >> 16) & 0xFF);
    content[8] = (uint8_t)((time >> 24) & 0xFF);
    content[9] = (uint8_t)(power & 0xFF);
    content[10] = (uint8_t)((power >> 8) & 0xFF);
    Servo_Send_Packet(SERVO_CMD_SIMPLE_MULTI_ANGLE, content, 11);
}

// 高级多圈控制（基于时间）
void Servo_Advanced_Multi_Turn_Time(uint8_t servo_id, int32_t position, uint32_t time, uint16_t accel, uint16_t decel, uint16_t power) {
    uint8_t content[15];
    content[0] = servo_id;
    content[1] = (uint8_t)(position & 0xFF);
    content[2] = (uint8_t)((position >> 8) & 0xFF);
    content[3] = (uint8_t)((position >> 16) & 0xFF);
    content[4] = (uint8_t)((position >> 24) & 0xFF);
    content[5] = (uint8_t)(time & 0xFF);
    content[6] = (uint8_t)((time >> 8) & 0xFF);
    content[7] = (uint8_t)((time >> 16) & 0xFF);
    content[8] = (uint8_t)((time >> 24) & 0xFF);
    content[9] = (uint8_t)(accel & 0xFF);
    content[10] = (uint8_t)((accel >> 8) & 0xFF);
    content[11] = (uint8_t)(decel & 0xFF);
    content[12] = (uint8_t)((decel >> 8) & 0xFF);
    content[13] = (uint8_t)(power & 0xFF);
    content[14] = (uint8_t)((power >> 8) & 0xFF);
    Servo_Send_Packet(SERVO_CMD_ADV_MULTI_TIME, content, 15);
}

// 高级多圈控制（基于速度）
void Servo_Advanced_Multi_Turn_Speed(uint8_t servo_id, int32_t position, uint16_t speed, uint16_t accel, uint16_t decel, uint16_t power) {
    uint8_t content[13];
    content[0] = servo_id;
    content[1] = (uint8_t)(position & 0xFF);
    content[2] = (uint8_t)((position >> 8) & 0xFF);
    content[3] = (uint8_t)((position >> 16) & 0xFF);
    content[4] = (uint8_t)((position >> 24) & 0xFF);
    content[5] = (uint8_t)(speed & 0xFF);
    content[6] = (uint8_t)((speed >> 8) & 0xFF);
    content[7] = (uint8_t)(accel & 0xFF);
    content[8] = (uint8_t)((accel >> 8) & 0xFF);
    content[9] = (uint8_t)(decel & 0xFF);
    content[10] = (uint8_t)((decel >> 8) & 0xFF);
    content[11] = (uint8_t)(power & 0xFF);
    content[12] = (uint8_t)((power >> 8) & 0xFF);
    Servo_Send_Packet(SERVO_CMD_ADV_MULTI_SPEED, content, 13);
}

// 多圈角度读取
void Servo_Read_Multi_Turn_Angle(uint8_t servo_id) {
    uint8_t content[1] = {servo_id};
    Servo_Send_Packet(SERVO_CMD_READ_MULTI_ANGLE, content, 1);
}

// 圈数重置
void Servo_Reset_Turns(uint8_t servo_id) {
    uint8_t content[1] = {servo_id};
    Servo_Send_Packet(SERVO_CMD_RESET_TURNS, content, 1);
}

// 阻尼控制
void Servo_Damping_Control(uint8_t servo_id, uint16_t power) {
    uint8_t content[3];
    content[0] = servo_id;
    content[1] = (uint8_t)(power & 0xFF);
    content[2] = (uint8_t)((power >> 8) & 0xFF);
    Servo_Send_Packet(SERVO_CMD_DAMPING, content, 3);
}

// 停止指令
void Servo_Stop(uint8_t servo_id, uint8_t mode, uint16_t power) {
    uint8_t content[4];
    content[0] = servo_id;
    content[1] = mode;
    content[2] = (uint8_t)(power & 0xFF);
    content[3] = (uint8_t)((power >> 8) & 0xFF);
    Servo_Send_Packet(SERVO_CMD_STOP, content, 4);
}

// 同步指令1：简易单圈角度控制 (0x08)
void Servo_Sync_Simple_Single_Turn(Sync_Simple_Single_t *targets, uint8_t count) {
    if (count == 0 || targets == NULL) return;

    uint8_t sync_cmd_id = SERVO_CMD_SIMPLE_SINGLE_ANGLE; // 0x08
    uint8_t sync_cmd_len = 0x07; // 真实长度: 7
    uint16_t total_content_len = 3 + (sync_cmd_len * count);
    if (total_content_len > 255) return; // 协议限制一帧数据内容最大255字节

    uint8_t content[256]; 
    content[0] = sync_cmd_id;
    content[1] = sync_cmd_len;
    content[2] = count;
    
    uint16_t idx = 3;
    for (uint8_t i = 0; i < count; i++) {
        content[idx++] = targets[i].servo_id;
        content[idx++] = (uint8_t)(targets[i].position & 0xFF);
        content[idx++] = (uint8_t)((targets[i].position >> 8) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].time & 0xFF);
        content[idx++] = (uint8_t)((targets[i].time >> 8) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].power & 0xFF);
        content[idx++] = (uint8_t)((targets[i].power >> 8) & 0xFF);
    }
    Servo_Send_Packet(SERVO_CMD_SYNC, content, (uint8_t)total_content_len);
}

// 同步指令2：高级单圈角度控制-基于时间 (0x0B)
void Servo_Sync_Adv_Single_Turn_Time(Sync_Adv_Single_Time_t *targets, uint8_t count) {
    if (count == 0 || targets == NULL) return;

    uint8_t sync_cmd_id = SERVO_CMD_ADV_SINGLE_TIME; // 0x0B
    uint8_t sync_cmd_len = 0x0B; // 真实长度: 11
    uint16_t total_content_len = 3 + (sync_cmd_len * count);
    if (total_content_len > 255) return;

    uint8_t content[256];
    content[0] = sync_cmd_id;
    content[1] = sync_cmd_len;
    content[2] = count;
    
    uint16_t idx = 3;
    for (uint8_t i = 0; i < count; i++) {
        content[idx++] = targets[i].servo_id;
        content[idx++] = (uint8_t)(targets[i].position & 0xFF);
        content[idx++] = (uint8_t)((targets[i].position >> 8) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].time & 0xFF);
        content[idx++] = (uint8_t)((targets[i].time >> 8) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].accel & 0xFF);
        content[idx++] = (uint8_t)((targets[i].accel >> 8) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].decel & 0xFF);
        content[idx++] = (uint8_t)((targets[i].decel >> 8) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].power & 0xFF);
        content[idx++] = (uint8_t)((targets[i].power >> 8) & 0xFF);
    }
    Servo_Send_Packet(SERVO_CMD_SYNC, content, (uint8_t)total_content_len);
}

// 同步指令3：高级单圈角度控制-基于速度 (0x0C)
void Servo_Sync_Adv_Single_Turn_Speed(Sync_Adv_Single_Speed_t *targets, uint8_t count) {
    if (count == 0 || targets == NULL) return;

    uint8_t sync_cmd_id = SERVO_CMD_ADV_SINGLE_SPEED; // 0x0C
    uint8_t sync_cmd_len = 0x0B; // 真实长度: 11 
    uint16_t total_content_len = 3 + (sync_cmd_len * count);
    if (total_content_len > 255) return;

    uint8_t content[256];
    content[0] = sync_cmd_id;
    content[1] = sync_cmd_len;
    content[2] = count;
    
    uint16_t idx = 3;
    for (uint8_t i = 0; i < count; i++) {
        content[idx++] = targets[i].servo_id;
        content[idx++] = (uint8_t)(targets[i].position & 0xFF);
        content[idx++] = (uint8_t)((targets[i].position >> 8) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].speed & 0xFF);
        content[idx++] = (uint8_t)((targets[i].speed >> 8) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].accel & 0xFF);
        content[idx++] = (uint8_t)((targets[i].accel >> 8) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].decel & 0xFF);
        content[idx++] = (uint8_t)((targets[i].decel >> 8) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].power & 0xFF);
        content[idx++] = (uint8_t)((targets[i].power >> 8) & 0xFF);
    }
    Servo_Send_Packet(SERVO_CMD_SYNC, content, (uint8_t)total_content_len);
}
 
// 同步指令4：简易多圈角度控制 (0x0D)
void Servo_Sync_Simple_Multi_Turn(Sync_Simple_Multi_t *targets, uint8_t count) {
    if (count == 0 || targets == NULL) return;

    uint8_t sync_cmd_id = SERVO_CMD_SIMPLE_MULTI_ANGLE; // 0x0D
    uint8_t sync_cmd_len = 0x0B; // 真实长度: 11
    uint16_t total_content_len = 3 + (sync_cmd_len * count);
    if (total_content_len > 255) return;

    uint8_t content[256];
    content[0] = sync_cmd_id;
    content[1] = sync_cmd_len;
    content[2] = count;
    
    uint16_t idx = 3;
    for (uint8_t i = 0; i < count; i++) {
        content[idx++] = targets[i].servo_id;
        content[idx++] = (uint8_t)(targets[i].position & 0xFF);
        content[idx++] = (uint8_t)((targets[i].position >> 8) & 0xFF);
        content[idx++] = (uint8_t)((targets[i].position >> 16) & 0xFF);
        content[idx++] = (uint8_t)((targets[i].position >> 24) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].time & 0xFF);
        content[idx++] = (uint8_t)((targets[i].time >> 8) & 0xFF);
        content[idx++] = (uint8_t)((targets[i].time >> 16) & 0xFF);
        content[idx++] = (uint8_t)((targets[i].time >> 24) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].power & 0xFF);
        content[idx++] = (uint8_t)((targets[i].power >> 8) & 0xFF);
    }
    Servo_Send_Packet(SERVO_CMD_SYNC, content, (uint8_t)total_content_len);
}

// 同步指令5：高级多圈角度控制-基于时间 (0x0E)
void Servo_Sync_Adv_Multi_Turn_Time(Sync_Adv_Multi_Time_t *targets, uint8_t count) {
    if (count == 0 || targets == NULL) return;

    uint8_t sync_cmd_id = SERVO_CMD_ADV_MULTI_TIME; // 0x0E
    uint8_t sync_cmd_len = 0x0F; // 真实长度: 15
    uint16_t total_content_len = 3 + (sync_cmd_len * count);
    if (total_content_len > 255) return;

    uint8_t content[256];
    content[0] = sync_cmd_id;
    content[1] = sync_cmd_len;
    content[2] = count;
    
    uint16_t idx = 3;
    for (uint8_t i = 0; i < count; i++) {
        content[idx++] = targets[i].servo_id;
        content[idx++] = (uint8_t)(targets[i].position & 0xFF);
        content[idx++] = (uint8_t)((targets[i].position >> 8) & 0xFF);
        content[idx++] = (uint8_t)((targets[i].position >> 16) & 0xFF);
        content[idx++] = (uint8_t)((targets[i].position >> 24) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].time & 0xFF);
        content[idx++] = (uint8_t)((targets[i].time >> 8) & 0xFF);
        content[idx++] = (uint8_t)((targets[i].time >> 16) & 0xFF);
        content[idx++] = (uint8_t)((targets[i].time >> 24) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].accel & 0xFF);
        content[idx++] = (uint8_t)((targets[i].accel >> 8) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].decel & 0xFF);
        content[idx++] = (uint8_t)((targets[i].decel >> 8) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].power & 0xFF);
        content[idx++] = (uint8_t)((targets[i].power >> 8) & 0xFF);
    }
    Servo_Send_Packet(SERVO_CMD_SYNC, content, (uint8_t)total_content_len);
}

// 同步指令6：高级多圈角度控制-基于速度 (0x0F)
void Servo_Sync_Adv_Multi_Turn_Speed(Sync_Adv_Multi_Speed_t *targets, uint8_t count) {
    if (count == 0 || targets == NULL) return;

    uint8_t sync_cmd_id = SERVO_CMD_ADV_MULTI_SPEED; // 0x0F
    uint8_t sync_cmd_len = 0x0D; // 真实长度: 13
    uint16_t total_content_len = 3 + (sync_cmd_len * count);
    if (total_content_len > 255) return;

    uint8_t content[256];
    content[0] = sync_cmd_id;
    content[1] = sync_cmd_len;
    content[2] = count;
    
    uint16_t idx = 3;
    for (uint8_t i = 0; i < count; i++) {
        content[idx++] = targets[i].servo_id;
        content[idx++] = (uint8_t)(targets[i].position & 0xFF);
        content[idx++] = (uint8_t)((targets[i].position >> 8) & 0xFF);
        content[idx++] = (uint8_t)((targets[i].position >> 16) & 0xFF);
        content[idx++] = (uint8_t)((targets[i].position >> 24) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].speed & 0xFF);
        content[idx++] = (uint8_t)((targets[i].speed >> 8) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].accel & 0xFF);
        content[idx++] = (uint8_t)((targets[i].accel >> 8) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].decel & 0xFF);
        content[idx++] = (uint8_t)((targets[i].decel >> 8) & 0xFF);
        content[idx++] = (uint8_t)(targets[i].power & 0xFF);
        content[idx++] = (uint8_t)((targets[i].power >> 8) & 0xFF);
    }
    Servo_Send_Packet(SERVO_CMD_SYNC, content, (uint8_t)total_content_len);
}

// 异步写入指令（还未封装）
void Servo_Async_Write(void) {
    Servo_Send_Packet(SERVO_CMD_ASYNC_WRITE, NULL, 0);
}

// 异步执行指令
void Servo_Async_Execute(uint8_t action) {
    uint8_t content[1] = {action};
    Servo_Send_Packet(SERVO_CMD_ASYNC_EXEC, content, 1);
}

// 数据读取
void Servo_Read_Data(uint8_t servo_id, uint8_t data_id) {
    uint8_t content[2];
    content[0] = servo_id;
    content[1] = data_id;
    Servo_Send_Packet(SERVO_CMD_READ_DATA, content, 2);
}

// 数据监控
void Servo_Monitor_Data(uint8_t servo_id) {
    uint8_t content[1] = {servo_id};
    Servo_Send_Packet(SERVO_CMD_MONITOR_DATA, content, 1);
}

// 设置原点
void Servo_Set_Origin(uint8_t servo_id, uint8_t reset_mode) {
    uint8_t content[2];
    content[0] = servo_id;
    content[1] = reset_mode;
    Servo_Send_Packet(SERVO_CMD_SET_ORIGIN, content, 2);
}
//--------------------------------发送模块--------------------------------//

//--------------------------------接收与解析模块--------------------------------//
// 全局数组 用于保存各个ID舵机的最新状态
Servo_Info_t g_ServoInfo[255] = {0}; 

// 接收缓存与状态机变量
static Servo_Rx_State_e rx_state = SERVO_RX_STATE_HEADER1;
static uint8_t rx_cmd_id = 0;
static uint8_t rx_length = 0;
static uint8_t rx_content[128];
static uint8_t rx_content_idx = 0;
static uint32_t rx_checksum_calc = 0;

// 接收状态机
void Servo_Receive_Byte(uint8_t byte) {
    switch (rx_state) {
        case SERVO_RX_STATE_HEADER1:
            if (byte == 0x05) { // 响应封包固定标识 1: 0x05
                rx_checksum_calc = byte;
                rx_state = SERVO_RX_STATE_HEADER2;
            }
            break;
            
        case SERVO_RX_STATE_HEADER2:
            if (byte == 0x1C) { // 响应封包固定标识 2: 0x1C
                rx_checksum_calc += byte;
                rx_state = SERVO_RX_STATE_CMD;
            } else {
                rx_state = SERVO_RX_STATE_HEADER1; // 错误，重置状态机
            }
            break;
            
        case SERVO_RX_STATE_CMD:
            rx_cmd_id = byte;
            rx_checksum_calc += byte;
            rx_state = SERVO_RX_STATE_LEN;
            break;
            
        case SERVO_RX_STATE_LEN:
            rx_length = byte;
            rx_checksum_calc += byte;
            rx_content_idx = 0;
            if (rx_length > 0) {
                rx_state = SERVO_RX_STATE_CONTENT;
            } else {
                rx_state = SERVO_RX_STATE_CHECKSUM; // 长度为0直接等校验和
            }
            break;
            
        case SERVO_RX_STATE_CONTENT:
            rx_content[rx_content_idx++] = byte;
            rx_checksum_calc += byte;
            if (rx_content_idx >= rx_length) {
                rx_state = SERVO_RX_STATE_CHECKSUM;
            }
            break;
            
        case SERVO_RX_STATE_CHECKSUM:
            if (byte == (rx_checksum_calc & 0xFF)) {
                // 校验成功，解析数据包
                Servo_Parse_Packet(rx_cmd_id, rx_length, rx_content);
            }
            // 无论校验成功与否，重置状态机准备接收下一帧
            rx_state = SERVO_RX_STATE_HEADER1; 
            break;
            
        default:
            rx_state = SERVO_RX_STATE_HEADER1;
            break;
    }
}

// 数据包解析函数
void Servo_Parse_Packet(uint8_t cmd_id, uint8_t length, uint8_t *content) {
    if (length == 0) return;
    
    uint8_t id = content[0]; // 响应包第一个字节必然是 servo_id
    
    switch (cmd_id) {
        case SERVO_CMD_PING: // 0x01 通讯检测
            // 示例: 0x05 0x1c 0x01 0x01 0x00 0x23
            g_ServoInfo[id].is_online = 1;
            printf("servo %d is online\n",id);
            break;
            
        case SERVO_CMD_READ_SINGLE_ANGLE: // 0x0A 单圈角度读取
            // 示例: 0x05 0x1C 0x0a 0x03 0x00 0x86 0x03 0xb7 -> 0x0386 = 902 (90.2度)
            if (length >= 3) {
                int16_t pos = (int16_t)(content[1] | (content[2] << 8));
                g_ServoInfo[id].position = pos;
            }
            printf("the angle of servo %d is %.1f \n",id,(float)g_ServoInfo[id].position/10);
            break;
            
        case SERVO_CMD_READ_MULTI_ANGLE: // 0x10 多圈角度读取
            // 示例: 0x05 0x1c 0x10 0x07 0x00 0x23 0x13 0x00 0x00 0x01 0x00 0x6f
            if (length >= 7) {
                int32_t pos = (int32_t)(content[1] | (content[2] << 8) | (content[3] << 16) | (content[4] << 24));
                int16_t turns = (int16_t)(content[5] | (content[6] << 8));
                g_ServoInfo[id].position = pos;
                g_ServoInfo[id].turns = turns;
                printf("the angle of servo %d is %.1f \n",id,(float)g_ServoInfo[id].position/10);
                printf("the turn of servo %d is %d \n",id,g_ServoInfo[id].turns);
            }
            break;
            
        case SERVO_CMD_MONITOR_DATA: // 0x16 数据监控
            // 示例: 0x05 0x1c 0x16 0x10 0x00 0x83 0x1e 0x1e 0x00 0xea 0x00 0x2c 0x07 0x01 0xaf 0x0b 0x00 0x00 0x00 0x00 0xde
            if (length >= 16) {
                g_ServoInfo[id].voltage     = (uint16_t)(content[1] | (content[2] << 8));
                g_ServoInfo[id].current     = (uint16_t)(content[3] | (content[4] << 8));
                g_ServoInfo[id].power       = (uint16_t)(content[5] | (content[6] << 8));
                g_ServoInfo[id].temperature = (uint16_t)(content[7] | (content[8] << 8));
                g_ServoInfo[id].status_err  = content[9];
                g_ServoInfo[id].position    = (int32_t)(content[10] | (content[11] << 8) | (content[12] << 16) | (content[13] << 24));
                g_ServoInfo[id].turns       = (int16_t)(content[14] | (content[15] << 8));
            }
            break;
            
        case SERVO_CMD_SIMPLE_SINGLE_ANGLE:
        case SERVO_CMD_ADV_SINGLE_TIME:
        case SERVO_CMD_ADV_SINGLE_SPEED:
        case SERVO_CMD_SIMPLE_MULTI_ANGLE:
        case SERVO_CMD_ADV_MULTI_TIME:
        case SERVO_CMD_ADV_MULTI_SPEED:
        case SERVO_CMD_RESET_TURNS:
        case SERVO_CMD_DAMPING:
        case SERVO_CMD_STOP:
        case SERVO_CMD_SET_ORIGIN:
            // 以上指令若开启了"可配置响应"，返回: servo_id(1) + result(1)
            // 示例: 0x05 0x1C 0x08 0x02 0x00 0x01 0x2c (result 0x01 成功)
            if (length >= 2) {
                uint8_t result = content[1];
                // 你可以在这里添加对 result 的处理，比如打印警告或设置标志位
                (void)result; 
            }
            break;
            
        default:
            // 其他如 0x03(数据读取) 等可以根据需求在这里继续扩展
            break;
    }
}

// 串口回调函数
void CN_UART3_RX_CALLBACK(uint8_t dat){
    Servo_Receive_Byte(dat);
}
//--------------------------------接收与解析模块--------------------------------//
#include "stm32h7xx_hal.h"
#include "ChaseNum1_Include.h"
#include <string.h>
#include <stdio.h>
// #include <stdlib.h>

Main_COM_Package_t Package_get={0};
uint8_t mission_check=0;

/******************************************************************应用层*********************************************************/
void Main_COM_init(void){
    CN_UART3_init();
    Package_get.done=1;//接收缓冲空闲状态
}

void Main_COM_send_pack(Main_CMD_ENUM CMD,uint8_t *load,uint8_t load_len){
}

void Main_COM_Response_CALLBACK(void){
}

void CN_send_Response(Main_CMD_ENUM CMD,uint8_t* response){
}

// 接收拼包
void Main_COM_RX_CALLBACK(uint8_t dat){
}

// // 串口3接收回调
// void CN_UART3_RX_CALLBACK(uint8_t dat){
//     Main_COM_RX_CALLBACK(dat);
// }
/******************************************************************应用层*********************************************************/
// 定义全局/静态变量用于状态记录和缓存
// 这些变量也可以放在文件开头作为 static 全局变量
static uint8_t vofa_receive_buffer[9];
static uint8_t vofa_receive_count = 0;

uint8_t motor_tun_flag=0;

void CN_UART1_RX_CALLBACK(uint8_t dat){
    VOFA_Single_Byte_Process(dat);
}

volatile Motor_Tuning_Params_t motor_tune = {0};
volatile bool param_updated_flag = 0;; 

void VOFA_Single_Byte_Process(uint8_t rx_byte)
{
    // 1. 寻找帧头 1：0xAA
    if (vofa_receive_count == 0) 
    {
        if (rx_byte == 0xAA) 
        {
            vofa_receive_buffer[0] = rx_byte;
            vofa_receive_count = 1;
        }
        return; // 不是 0xAA 就直接丢弃，继续等
    }
    
    // 2. 寻找帧头 2：0x55
    if (vofa_receive_count == 1) 
    {
        if (rx_byte == 0x55) 
        {
            vofa_receive_buffer[1] = rx_byte;
            vofa_receive_count = 2;
        } 
        else if (rx_byte == 0xAA) 
        {
            // 防错机制：遇到连续的 AA (比如丢包导致的 AA AA 55)，停留在状态 1
            vofa_receive_count = 1;
        } 
        else 
        {
            // 校验失败，重新开始找帧头
            vofa_receive_count = 0;
        }
        return;
    }

    // 3. 接收中间的数据和帧尾（第3~9个字节）
    if (vofa_receive_count >= 2 && vofa_receive_count < 9)
    {
        vofa_receive_buffer[vofa_receive_count++] = rx_byte;
    }

    // 4. 接收满 9 个字节，进行帧尾校验并解析
    if (vofa_receive_count == 9)
    {
        // 严格校验双帧尾 0x5A 0xA5
        if (vofa_receive_buffer[7] == 0x5A && vofa_receive_buffer[8] == 0xA5)
        {
            // 校验成功，提取数据
            uint8_t rx_id = vofa_receive_buffer[2];
            VOFA_Float_Union rx_float;
            
            // 将 4 个数据字节塞入共用体
            rx_float.bytes[0] = vofa_receive_buffer[3];
            rx_float.bytes[1] = vofa_receive_buffer[4];
            rx_float.bytes[2] = vofa_receive_buffer[5];
            rx_float.bytes[3] = vofa_receive_buffer[6];
            
            // 根据 ID 赋值给对应的参数
            switch (rx_id) {
                case 0: {motor_tune.target_angle = rx_float.f_data; motor_tun_flag=1;break;}
                case 1: motor_tune.Kp_angle     = rx_float.f_data; break;
                case 2: motor_tune.Ki_angle     = rx_float.f_data; break;
                case 3: motor_tune.Kd_angle     = rx_float.f_data; break;
                case 4: motor_tune.target_vel   = rx_float.f_data; break;
                case 5: motor_tune.Kp_vel       = rx_float.f_data; break;
                case 6: motor_tune.Ki_vel       = rx_float.f_data; break;
                case 7: motor_tune.Kd_vel       = rx_float.f_data; break;
            }
            
            param_updated_flag = true; // 标记参数已更新
        }
        
        // 解析完毕（无论校验成功还是失败），重置计数器准备迎接下一帧
        vofa_receive_count = 0;
    }
}
/******************************************************************数据链路层*********************************************************/

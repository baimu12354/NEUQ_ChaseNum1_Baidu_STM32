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

/******************************************************************数据链路层*********************************************************/

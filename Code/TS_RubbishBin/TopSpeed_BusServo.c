// /* ds_servo.c */
// #include "TopSpeed_BusServo.h"
// #include <string.h>

// static void DS_SendPacket(UART_HandleTypeDef *huart, uint8_t *data, uint8_t len) {
//     HAL_UART_Transmit(huart, data, len, HAL_MAX_DELAY);
// }

// static uint8_t DS_Checksum(uint8_t *data, uint8_t len) {
//     uint16_t sum = 0;
//     for (int i = 2; i < len - 1; i++) sum += data[i];
//     return ~sum & 0xFF;
// }

// void DS_Servo_WritePos(UART_HandleTypeDef *huart, uint8_t id, uint16_t pos, uint16_t time) {
//     uint8_t buf[11];
//     buf[0] = DS_SERVO_HEADER_1;
//     buf[1] = DS_SERVO_HEADER_2;
//     buf[2] = id;
//     buf[3] = 7; // 长度 = 指令+地址+数据
//     buf[4] = 0x03;
//     buf[5] = DS_REG_GOAL_POSITION;
//     buf[6] = (pos >> 8) & 0xFF;
//     buf[7] = pos & 0xFF;
//     buf[8] = (time >> 8) & 0xFF;
//     buf[9] = time & 0xFF;
//     buf[10] = DS_Checksum(buf, 11);
//     DS_SendPacket(huart, buf, 11);
// }

// void DS_Servo_RegWritePos(UART_HandleTypeDef *huart, uint8_t id, uint16_t pos, uint16_t time) {
//     uint8_t buf[11];
//     buf[0] = DS_SERVO_HEADER_1;
//     buf[1] = DS_SERVO_HEADER_2;
//     buf[2] = id;
//     buf[3] = 0x07;
//     buf[4] = 0x04;
//     buf[5] = DS_REG_GOAL_POSITION;
//     buf[6] = (pos >> 8) & 0xFF;
//     buf[7] = pos & 0xFF;
//     buf[8] = (time >> 8) & 0xFF;
//     buf[9] = time & 0xFF;
//     buf[10] = DS_Checksum(buf, 11);
//     DS_SendPacket(huart, buf, 11);
// }

// void DS_Servo_Action(UART_HandleTypeDef *huart) {
//     uint8_t buf[6];
//     buf[0] = DS_SERVO_HEADER_1;
//     buf[1] = DS_SERVO_HEADER_2;
//     buf[2] = DS_SERVO_BROADCAST_ID;
//     buf[3] = 2;
//     buf[4] = 0x05;
//     buf[5] = DS_Checksum(buf, 6);
//     DS_SendPacket(huart, buf, 6);
// }

// void DS_Servo_Reset(UART_HandleTypeDef *huart,uint8_t id) {
//     uint8_t buf[6];
//     buf[0] = DS_SERVO_HEADER_1;
//     buf[1] = DS_SERVO_HEADER_2;
//     buf[2] = id;
//     buf[3] = 2;
//     buf[4] = 0x06;
//     buf[5] = DS_Checksum(buf, 6);
//     DS_SendPacket(huart, buf, 6);
// }

// void DS_Servo_SetID(UART_HandleTypeDef *huart,uint8_t id,uint8_t id_aim) {
//     uint8_t buf[8];
//     buf[0] = DS_SERVO_HEADER_1;
//     buf[1] = DS_SERVO_HEADER_2;
//     buf[2] = id;
//     buf[3] = 0x04;
//     buf[4] = 0x03;
//     buf[5] = 0x05;
//     buf[6] = id_aim;
//     buf[7] = DS_Checksum(buf, 8);
//     DS_SendPacket(huart, buf, 8);
// }


// bool DS_Servo_ReadByte(UART_HandleTypeDef *huart, uint8_t id, uint8_t addr, uint8_t *value) {
//     uint8_t buf[8] = {DS_SERVO_HEADER_1, DS_SERVO_HEADER_2, id, 4, 0x02, addr, 1, 0};
//     buf[7] = DS_Checksum(buf, 8);
//     DS_SendPacket(huart, buf, 8);

//     uint8_t resp[7];
//     if (HAL_UART_Receive(huart, resp, 7, 10) == HAL_OK && resp[1] == DS_SERVO_RESP_HEADER_2) {
//         *value = resp[5];
//         return true;
//     }
//     return false;
// }

// bool DS_Servo_ReadPos(UART_HandleTypeDef *huart, uint8_t id, uint16_t *pos) {
//     uint8_t buf[8] = {DS_SERVO_HEADER_1, DS_SERVO_HEADER_2, id, 4, 0x02, DS_REG_PRESENT_POSITION, 2, 0};
//     buf[7] = DS_Checksum(buf, 8);
//     DS_SendPacket(huart, buf, 8);

//     uint8_t resp[8];
//     if (HAL_UART_Receive(huart, resp, 8, 10) == HAL_OK && resp[1] == DS_SERVO_RESP_HEADER_2) {
//         *pos = (resp[5] << 8) | resp[6];
//         return true;
//     }
//     return false;
// }

// bool DS_Servo_Ping(UART_HandleTypeDef *huart, uint8_t id) {
//     uint8_t buf[6] = {DS_SERVO_HEADER_1, DS_SERVO_HEADER_2, id, 0x02, 0x01};
//     buf[5] = DS_Checksum(buf, 6);
//     DS_SendPacket(huart, buf, 6);

//     // printf("send\n");
//     // for(int i=0;i<6;i++){
//     //     printf("%X\n",buf[i]);
//     // }

//     uint8_t resp[6]={0};

//     uint8_t res=HAL_UART_Receive(huart, resp, 6, 1000);

//     printf("get\n");
//     for(int i=0;i<6;i++){
//         printf("%X\n",resp[i]);
//     }

//     return  res==HAL_OK;// && resp[1] == 0x17;
// }


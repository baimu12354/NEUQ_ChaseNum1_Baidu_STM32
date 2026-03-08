// /* ds_servo.h */
// #ifndef __DS_SERVO_H
// #define __DS_SERVO_H

// #include "stm32h7xx_hal.h"
// #include <stdint.h>
// #include <stdbool.h>

// #define DS_SERVO_BROADCAST_ID 254
// #define DS_SERVO_HEADER_1 0xFF
// #define DS_SERVO_HEADER_2 0xFF
// #define DS_SERVO_RESP_HEADER_2 0xF5

// // 常用寄存器地址
// #define DS_REG_GOAL_POSITION     0x2A
// #define DS_REG_MOVE_TIME         0x2C
// #define DS_REG_PRESENT_POSITION  0x38
// #define DS_REG_TEMPERATURE       0x3F

// // API 函数
// void DS_Servo_WritePos(UART_HandleTypeDef *huart, uint8_t id, uint16_t pos, uint16_t time);
// void DS_Servo_RegWritePos(UART_HandleTypeDef *huart, uint8_t id, uint16_t pos, uint16_t time);
// void DS_Servo_Action(UART_HandleTypeDef *huart);
// void DS_Servo_Reset(UART_HandleTypeDef *huart,uint8_t id);
// void DS_Servo_SetID(UART_HandleTypeDef *huart,uint8_t id,uint8_t id_aim);
// bool DS_Servo_ReadPos(UART_HandleTypeDef *huart, uint8_t id, uint16_t *pos);
// bool DS_Servo_ReadByte(UART_HandleTypeDef *huart, uint8_t id, uint8_t addr, uint8_t *value);
// bool DS_Servo_Ping(UART_HandleTypeDef *huart, uint8_t id);

// #endif
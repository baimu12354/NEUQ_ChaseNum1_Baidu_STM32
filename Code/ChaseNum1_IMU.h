#ifndef __CHASE_NUM1_IMU_H_
#define __CHASE_NUM1_IMU_H_

#include "stm32h7xx_hal.h"
#include "ChaseNum1_Typedef.h"

extern float YAW_now;
extern float yaw_bias;
extern float YAW_aim;

void CN_3rd_Party_JY61_IMU_init(void);//初始化陀螺仪
uint8_t CN_auto_scan_JY61(void);//自动检测陀螺仪的波特率
void CN_3rd_Party_JY61_IMU_update_CALLBACK(uint32_t uiReg, uint32_t uiRegNum);//陀螺仪数据更新回调函数
void CN_Set_YAW(float yaw);//规定当前的逻辑角度
float CN_get_logical_yaw(void);//获取经过校准后的陀螺仪角度

#endif // !__CHASE_NUM1_IMU_H_
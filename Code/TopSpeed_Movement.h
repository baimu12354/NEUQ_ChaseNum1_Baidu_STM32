#ifndef _TOPSPEED_MOVEMENT_H_
#define _TOPSPEED_MOVEMENT_H_

#include "stm32h7xx_hal.h"
#include "TopSpeed_Typedef.h"

extern float YAW_now;

extern uint8_t Movement_Precise_Mode;

void TS_3rd_Party_JY61_IMU_init(void);//初始化陀螺仪
uint8_t TS_auto_scan_JY61(void);//自动检测陀螺仪的波特率
void TS_Set_YAW(float yaw);//规定当前的逻辑角度
float TS_get_logical_yaw(void);//获取经过校准后的陀螺仪角度

void TS_Movement_Change_To_Precise_Mode(void);//高精度模式
void TS_Movement_Change_To_Normal_Mode(void);//普通模式
void TS_move_init(void);//运动模块初始化

void TS_set_movement(Car_Movement_struct movement,Movement_type_ENUM movement_type,uint8_t cmd_check_get);//外部调用函数的接口
void TS_Movement_done_check_CALLBACK(void);//检查运动是否完成

void TS_Movement_ctrl_CALLBACK(uint32_t us_delta_time);//运动控制入口
void TS_Intr_Movement(void);//停止运动

void TS_CAR_yaw_CL_init(void);//初始化角度环参数
void TS_CAR_yaw_CL_CALLBACK(uint32_t us_time);//执行角度环函数
void TS_CAR_yaw_CL_set_aim(float yaw);//设定目标角度
float TS_get_relative_angle(float now,float aim);//解决过零点问题

void TS_CAR_vel_CL_init(void);//初始化速度环
void TS_CAR_vel_CL_CALLBACK(uint32_t us_time);//执行速度环
void TS_CAR_vel_CL_set_aim(Car_Movement_struct C_vel);//设定目标速度

void TS_CAR_pos_CL_init(void);//初始化位置环
void TS_CAR_pos_CL_set_aim(Car_Movement_struct C_pos_aim);//设定目标位置
void TS_CAR_pos_CL_CALLBACK(uint32_t us_time);//位置环执行函数

#endif // !_TOPSPEED_MOVEMENT_H_
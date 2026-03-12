#ifndef _CHASE_NUM1_MOVEMENT_H_
#define _CHASE_NUM1_MOVEMENT_H_

#include "stm32h7xx_hal.h"
#include "ChaseNum1_Typedef.h"

#define M_TO_ENC_FACTOR (1.0f)

#define TRANSLATION_TOLERANCE_PRECISE   (50)
#define ROTATION_TOLERANCE_PRECISE      (2)
#define CONFIRM_TIME_PRECISE            (50)//*5ms
#define TRANSLATION_TOLERANCE   (100)
#define ROTATION_TOLERANCE      (2)
#define CONFIRM_TIME            (10)//*5ms

#define Pos_Err_LPF_LEN (20)

extern uint8_t Movement_Precise_Mode;
extern float distance_to_go;

void CN_Movement_Change_To_Precise_Mode(void);//高精度模式
void CN_Movement_Change_To_Normal_Mode(void);//普通模式
void CN_move_init(void);//运动模块初始化

void CN_set_movement(Car_Movement_struct movement,Movement_type_ENUM movement_type,uint8_t cmd_check_get);//外部调用函数的接口
void CN_Movement_done_check_CALLBACK(void);//检查运动是否完成

void CN_Movement_ctrl_CALLBACK(uint32_t us_delta_time);//运动控制入口
void CN_Intr_Movement(void);//停止运动

void CN_CAR_yaw_CL_init(void);//初始化角度环参数
void CN_CAR_yaw_CL_CALLBACK(uint32_t us_time);//执行角度环函数
void CN_CAR_yaw_CL_set_aim(float yaw);//设定目标角度
float CN_get_relative_angle(float now,float aim);//解决过零点问题

void CN_CAR_vel_CL_init(void);//初始化速度环
void CN_CAR_vel_CL_CALLBACK(uint32_t us_time);//执行速度环
void CN_CAR_vel_CL_set_aim(Car_Movement_struct C_vel);//设定目标速度

void CN_CAR_pos_CL_init(void);//初始化位置环
void CN_CAR_pos_CL_set_aim(Car_Movement_struct C_pos_aim);//设定目标位置
void CN_CAR_pos_CL_CALLBACK(uint32_t us_time);//位置环执行函数

#endif // !_CHASE_NUM1_MOVEMENT_H_
#ifndef _CHASE_NUM1_MOVEMENT_H_
#define _CHASE_NUM1_MOVEMENT_H_

#include "stm32h7xx_hal.h"
#include "ChaseNum1_Typedef.h"

#define M_TO_ENC_FACTOR (1.0f)

extern PID_ctrler Car_anv_PID_ctrler;
extern PID_ctrler Car_yaw_PID_ctrler;
extern PID_ctrler Car_yaw_moving_PID_ctrler;

// 用于旋转
extern float target_yaw;    // 目标旋转角度
extern float YAW_inital;    // 初始角度（用于记录运动之前的车身姿态）
extern float YAW_gain;  // 累计角度
extern float YAW_pre;   // 上一次的角度

// 用于运动
extern float YAW_acc;       // 角度增量（用于驱动车身以固定的角速度旋转）
extern float target_anv;    // 目标角速度
extern float target_VelX;   // 目标横向速度
extern float target_VelY;   // 目标纵向速度

// 用于定位
extern float distance_to_go;

void CN_move_init(void);//运动模块初始化
void CN_set_movement(Car_Movement_struct movement,Movement_type_ENUM movement_type);//外部调用函数的接口
void CN_Movement_ctrl_CALLBACK(uint32_t ms);//运动控制入口
void CN_Intr_Movement(void);//停止运动
 
// 设置目标角度增量（连续的、模拟角速度）
void CN_CAR_rotation_set_aim(float oumiga);
// 车身角度环初始化
void CN_CAR_yaw_CL_init(void);
// 车身角度环回调函数
void CN_CAR_yaw_CL_CALLBACK(uint32_t us_time);
// 获取两个绝对方位角之间的相对方位角
float CN_get_relative_angle(float now,float aim);
// 获取目标逻辑角度
float CN_yaw_add(float now,float aim);
// 设置目标角度
void CN_CAR_yaw_CL_set_aim(float yaw);

// 角速度环初始化
void CN_CAR_anv_CL_init(void);
// 设置目标角速度
void CN_CAR_anv_CL_set_aim(float anv);
// 角速度环控制回调
void CN_CAR_anv_CL_CALLBACK(uint32_t us_time);

// 速度环初始化
void CN_CAR_vel_CL_init(void);
// 设置目标速度
void CN_CAR_vel_CL_set_aim(float VelX, float VelY);
// 速度环控制回调
void CN_CAR_vel_CL_CALLBACK(uint32_t us_time);


void CN_CAR_pos_CL_init(void);//初始化位置环
void CN_CAR_pos_CL_set_aim(Car_Movement_struct C_pos_aim);//设定目标位置
void CN_CAR_pos_CL_CALLBACK(uint32_t us_time);//位置环执行函数

#endif // !_CHASE_NUM1_MOVEMENT_H_
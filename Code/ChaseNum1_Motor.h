#ifndef _CHASENUM1_MOTOR_H_
#define _CHASENUM1_MOTOR_H_

#include "stm32h7xx_hal.h"
#include "ChaseNum1_Typedef.h"

/*
    引脚定义：
    FL  C8  A9  TIM8CH3 TIM1CH2
    FR  C9  A10 TIM8CH4 TIM1CH3
    BL  C6  B8  TIM8CH1 TIM16CH1
    BR  C7  A8  TIM8CH2 TIM1Ch1
*/



extern Motor_duty_struct Duty_now;
extern Motor_vel_struct Vel_now;
extern Motor_vel_struct Vel_aim;
extern Motor_pos_struct Pos_now;
extern Motor_pos_struct Pos_aim;

extern uint8_t Motor_Precise_Mode;
/****************************电机功率环***************************/
void CN_TIM8_Motor_init(void);
void CN_TIM1_Motor_init(void);
void CN_TIM14_Motor_init(void);
void CN_TIM16_Motor_init(void);
void CN_TIM4_Motor_init(void);
void CN_Motor_init(void);
void CN_TIM_Motor_PWMOUT_ENA(void);
void CN_TIM_Motor_PWMOUT_DISA(void);
void CN_TIM_Motor_PWM_duty_set(uint32_t CH1_8[8]);
void CN_set_Motor_Duty(void);
/****************************电机功率环***************************/

void CN_Motor_Change_To_Precise_Mode(void);
void CN_Motor_Change_To_Normal_Mode(void);

/****************************电机转速环***************************/
void CN_Motor_vel_CL_init(void);
void CN_Motor_vel_CL_set_aim(Motor_vel_struct aim);
void CN_Motor_vel_CL_CALLBACK(uint32_t us_time);
void CN_Motor_outline(void);

void CN_Milemeter_set(float milemeter_value);
float CN_get_Milemeter(void);
/****************************电机转速环***************************/

/****************************电机位置环***************************/
void CN_Motor_pos_CL_init(void);
void CN_Motor_pos_CL_CALLBACK(uint32_t us_time);
void CN_Motor_pos_CL_set_aim(Motor_pos_struct aim);
/****************************电机位置环***************************/

extern PID_ctrler FL_pos_PID_ctrler;
extern PID_ctrler FR_pos_PID_ctrler;
extern PID_ctrler BL_pos_PID_ctrler;
extern PID_ctrler BR_pos_PID_ctrler;

#endif // !_CHASENUM1_MOTOR_H_
#ifndef _CHASENUM1_MOTOR_H_
#define _CHASENUM1_MOTOR_H_

#include "stm32h7xx_hal.h"
#include "ChaseNum1_Typedef.h"

extern Motor_duty_struct Duty_now;
extern Motor_vel_struct Vel_now;
extern Motor_vel_struct Vel_aim;

extern PID_ctrler FL_vel_PID_ctrler;
extern PID_ctrler FR_vel_PID_ctrler;
extern PID_ctrler BL_vel_PID_ctrler;
extern PID_ctrler BR_vel_PID_ctrler;

/****************************电机配置***************************/
void CN_TIM8_Motor_init(void);
void CN_TIM1_Motor_init(void);
void CN_TIM12_Motor_init(void);
void CN_Motor_init(void);
void CN_TIM_Motor_PWMOUT_ENA(void);
void CN_TIM_Motor_PWMOUT_DISA(void);
void CN_TIM_Motor_PWM_duty_set(uint32_t CH1_8[8]);
void CN_set_Motor_Duty(void);
void CN_stop_Motor_Duty(void);
/****************************电机配置***************************/

/****************************电机转速环***************************/
void CN_Motor_vel_CL_init(void);
void CN_Motor_vel_CL_set_aim(Motor_vel_struct aim);
void CN_Motor_vel_CL_CALLBACK(uint32_t us_time);
void CN_Motor_outline(void);
/****************************电机转速环***************************/



#endif // !_CHASENUM1_MOTOR_H_
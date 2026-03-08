#ifndef _TOPSPEED_SERVO_H_
#define _TOPSPEED_SERVO_H_

#include "stm32h7xx_hal.h"
#include "TopSpeed_Typedef.h"

// #define TS_SERVO_GRIPPER_YAW_90_DUTY    (2030)
// #define TS_SERVO_GRIPPER_YAW_0_DUTY    (3750)
// #define TS_SERVO_GRIPPER_YAW_090_DUTY    (5400)

#define TS_SERVO_GRIPPER_YAW_90_DUTY    (2200)
#define TS_SERVO_GRIPPER_YAW_0_DUTY    (3850)
#define TS_SERVO_GRIPPER_YAW_090_DUTY    (5550)

#define TS_SERVO_GRIPPER_PITCH1_HORIZONTAL_DUTY    (3700)
#define TS_SERVO_GRIPPER_PITCH1_OVERLOOK_DUTY    (2100)

#define TS_SERVO_GRIPPER_PITCH2_HORIZONTAL_DUTY    (5300)
#define TS_SERVO_GRIPPER_PITCH2_OVERLOOK_DUTY    (1200)

#define TS_SERVO_GRIPPER_PITCH1_TEACHING1_DUTY    (3800)
#define TS_SERVO_GRIPPER_PITCH2_TEACHING1_DUTY    (5000)

#define TS_SERVO_GRIPPER_PITCH1_TEACHING2_DUTY    (3650)
#define TS_SERVO_GRIPPER_PITCH2_TEACHING2_DUTY    (5600)

#define TS_SERVO_GRIPPER_PITCH1_GRIP_DUTY    (6000)
#define TS_SERVO_GRIPPER_PITCH2_GRIP_DUTY    (5200)

#define TS_SERVO_SHOOTER_RELOAD_ON_DUTY    (6200)       //装填设备打开
#define TS_SERVO_SHOOTER_RELOAD_OFF_DUTY    (4750)      //装填设备关闭

#define TS_SERVO_GRIPPER_YAW_INPUT_MIN  (-90)
#define TS_SERVO_GRIPPER_YAW_INPUT_MAX  (90)

#define TS_SERVO_GRIPPER_YAW_OUTPUT_MIN  (TS_SERVO_GRIPPER_YAW_090_DUTY)
#define TS_SERVO_GRIPPER_YAW_OUTPUT_MID  (TS_SERVO_GRIPPER_YAW_0_DUTY)
#define TS_SERVO_GRIPPER_YAW_OUTPUT_MAX  (TS_SERVO_GRIPPER_YAW_90_DUTY)

/*
    Servo1:PH6  //吸盘yaw轴舵机
    Servo2:PH9  //吸盘pitch1舵机
    Servo3:PE5  //吸盘pitch2舵机
    Servo4:PE6  //弹弓装填舵机
*/


uint8_t TS_SERVO_operate(SERVO_Handle_struct *H_SERVO,Action_method_ENUM OPE_SELECT,int data);

void TS_Set_Servo_Gripper_yaw_slow(float yaw);
void TS_SET_Servo_Gripper_yaw(float yaw);
void TS_Servo_init(void);
void TS_Servo_check(void);

void TS_Servo_timer_init(void);
void TS_TIM_Servo_PWMOUT_ENA(void);
void TS_TIM_Servo_PWMOUT_DISA(void);
// void TS_TIM_Servo_PWM_duty_set(uint32_t Servo_1,uint32_t Servo_2,uint32_t Servo_3,uint32_t Servo_4);
void TS_Servo_Duty_Set(Servo_enum Servo,uint32_t duty);
void TS_Servo_Duty_Set_slow(Servo_enum Servo,uint32_t duty,uint32_t step);
void TS_Servo_Duty_Done_poll(void);

void TS_Change_to_Horizontal(void);
void TS_Change_to_OverLook(void);
void TS_Change_to_Teaching_1(void);
void TS_Change_to_Teaching_2(void);
void TS_Change_to_Grip(void);

void TS_Servo_Shooter_reload_ON(void);
void TS_Servo_Shooter_reload_OFF(void);

extern SERVO_Handle_struct SERVO_Gripper_yaw_Handle;
extern SERVO_Handle_struct SERVO_Gripper_grip_Handle;

#endif // !_TOPSPEED_SERVO_H_
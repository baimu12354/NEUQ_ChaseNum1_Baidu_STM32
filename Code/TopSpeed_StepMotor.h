#ifndef _TOPSPEED_STEPMOTOR_H_
#define _TOPSPEED_STEPMOTOR_H_

#include "stm32h7xx_hal.h"
#include "TopSpeed_Typedef.h"

/*******驱动层定义(脉冲数)*******/
#define SM_Z_LOW    (0)
#define SM_Z_HIGH    (9000)

#define SM_X_LEFT    (0)
#define SM_X_RIGHT    (18000)


/*******应用层定义(位置)*******/
#define LIFT_Z_LOW    (0)
#define LIFT_Z_HIGH    (10000)

#define LIFT_X_LEFT    (-10500)
#define LEFT_X_RIGHT    (10500)

//     Trig_XLeft,     //Trigger1:PB9
//     Trig_XRight,    //Trigger2:PB7
//     Trig_ZUp,       //Trigger3:PI5
//     Trig_ZDown,     //Trigger4:PI4
//     Trig_Shooter,   //Trigger6:PI6


#define TS_TEST_WAIT_TILL_SM1_READY()   while(1){if(SM_1_Handle.SM_status==SM_NONE){break;}}
#define TS_TEST_WAIT_TILL_SM2_READY()   while(1){if(SM_2_Handle.SM_status==SM_NONE){break;}}
#define TS_TEST_WAIT_TILL_SM3_READY()   while(1){if(SM_3_Handle.SM_status==SM_NONE){break;}}


void TS_SM_init(void);
void TS_CHECK_SM(void);
void TS_SM_SET_CMD(SM_NO_ENUM SM_SELECT,uint8_t *cmd,uint32_t cmd_len);
void TS_SM_COM_RX_CALLBACK(SM_Handle_struct *H_SM,uint8_t dat);
void TS_Parse_SM_1_STATUS(void);
void TS_Parse_SM_2_STATUS(void);

uint8_t TS_SM_operate(SM_Handle_struct *H_SM,Action_method_ENUM OPE_SELECT,int data);


void TS_SM_ENA(void);
uint8_t TS_SM_Set_spd(SM_Handle_struct *H_SM,int spd);
uint8_t TS_SM_Set_pos_inc(SM_Handle_struct *H_SM,int pos,uint32_t spd);
uint8_t TS_SM_Set_pos(SM_Handle_struct *H_SM,int pos,uint32_t spd);
uint8_t TS_SM_Set_ZERO(SM_Handle_struct *H_SM,SM_NO_ENUM SM_SELECT);
uint8_t TS_SM_Set_ENA(SM_Handle_struct *H_SM,uint8_t ena);
uint8_t TS_SM_Set_PARA(SM_Handle_struct *H_SM,uint8_t save);

void TS_SM_Trig_init(void);
void TS_SM_Trig_poll(void);
void TS_SM_inpos_poll(void);


void TS_Reset_SM_COM_FIAL_counter(SM_Handle_struct *H_SM,uint32_t timeout_ms);
void TS_SM_timeout_countdown_1ms(void);
void TS_SM_timeout_polling(void);
void TS_SM_resend_message(SM_Handle_struct *H_SM);

extern SM_Handle_struct SM_1_Handle;
extern SM_Handle_struct SM_2_Handle;
extern SM_Handle_struct SM_3_Handle;

#endif // !_TOPSPEED_STEPMOTOR_H_
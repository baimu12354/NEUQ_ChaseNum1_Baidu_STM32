#include "ChaseNum1_Include.h"
#include <math.h>
    
#define USE_WHEELTEC_DRIVER (0)
#define USE_CHERRY_DRIVER   (1)

#define CN_PWM_PRELOAD  (10000)
#define CN_PWM_MAX  (4000-1)

TIM_HandleTypeDef TIM8_Motor_Handle={0};
TIM_HandleTypeDef TIM1_Motor_Handle={0};
TIM_HandleTypeDef TIM14_Motor_Handle={0};
TIM_HandleTypeDef TIM16_Motor_Handle={0};

Motor_duty_struct Duty_now={0};
Motor_vel_struct Vel_now={0};
Motor_vel_struct Vel_aim={0};
Motor_pos_struct Pos_now={0};
Motor_pos_struct Pos_aim={0};

PID_ctrler FL_vel_PID_ctrler={0};
PID_ctrler FR_vel_PID_ctrler={0};
PID_ctrler BL_vel_PID_ctrler={0};
PID_ctrler BR_vel_PID_ctrler={0};

PID_ctrler FL_pos_PID_ctrler={0};
PID_ctrler FR_pos_PID_ctrler={0};
PID_ctrler BL_pos_PID_ctrler={0};
PID_ctrler BR_pos_PID_ctrler={0};


#define MOTOR_VEL_CL_PID_KP (200)
#define MOTOR_VEL_CL_PID_KI (0)
#define MOTOR_VEL_CL_PID_KD (150)
#define MOTOR_VEL_CL_PID_IMAX (0)
#define MOTOR_VEL_CL_PID_OUTMAX (500)

#define MOTOR_POS_CL_PID_KP (0.01)
#define MOTOR_POS_CL_PID_KI (0.0001)
#define MOTOR_POS_CL_PID_KD (0.01)
#define MOTOR_POS_CL_PID_IMAX (2)
#define MOTOR_POS_CL_PID_OUTMAX (50)

#define MOTOR_POS_CL_SLOW_PID_KP (0.005)
#define MOTOR_POS_CL_SLOW_PID_KI (0.0001)
#define MOTOR_POS_CL_SLOW_PID_KD (0.0)
#define MOTOR_POS_CL_SLOW_PID_IMAX (2)
#define MOTOR_POS_CL_SLOW_PID_OUTMAX (10)

#define MOTOR_VEL_CL_PID_KP_PRECISE (100)
#define MOTOR_VEL_CL_PID_KI_PRECISE (0)
#define MOTOR_VEL_CL_PID_KD_PRECISE (100)
#define MOTOR_VEL_CL_PID_IMAX_PRECISE (0)
#define MOTOR_VEL_CL_PID_OUTMAX_PRECISE (200)

#define MOTOR_POS_CL_PID_KP_PRECISE (0.01)
#define MOTOR_POS_CL_PID_KI_PRECISE (0.00002)
#define MOTOR_POS_CL_PID_KD_PRECISE (0.05)
#define MOTOR_POS_CL_PID_IMAX_PRECISE (0)
#define MOTOR_POS_CL_PID_OUTMAX_PRECISE (15)

/****************************电机配置***************************/
#if USE_WHEELTEC_DRIVER

/*
    M1:PWM PC6;IO:PI0、1;ENC:PA0、1
    M2:PWM PC7;IO:PI2、3;ENC:PA6、7
    M3:PWM PC8;IO:PI4、5;ENC:PD12、13
    M4:PWM PC9;IO:PI6、7;ENC:PH10、11
*/
/****************************电机功率环***************************/

void TS_WHEELTEC_Driver_IO_init(void);
void TS_Motor_init(void){
    TS_WHEELTEC_Driver_IO_init();
    TS_TIM8_Motor_init();
}

void TS_TIM8_Motor_init(void){
    GPIO_InitTypeDef GPIO_Initure={0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;  
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用模式
	GPIO_Initure.Pull=GPIO_NOPULL;              //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //高速
    GPIO_Initure.Alternate=GPIO_AF3_TIM8;
	HAL_GPIO_Init(GPIOC,&GPIO_Initure);

    __HAL_RCC_TIM8_CLK_ENABLE(); 
    TIM8_Motor_Handle.Instance=TIM8;
    TIM8_Motor_Handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//200mhz
    TIM8_Motor_Handle.Init.Prescaler=20-1;//10mhz
    TIM8_Motor_Handle.Init.Period=TS_PWM_PRELOAD-1;//1khz
    TIM8_Motor_Handle.Init.CounterMode=TIM_COUNTERMODE_UP;
    TIM8_Motor_Handle.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&TIM8_Motor_Handle);

    TIM_OC_InitTypeDef sConfigOC={0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;  // PWM 模式 1
    sConfigOC.Pulse = 0;  // 设置占空比
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;  // 输出极性
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;  // 禁用快速模式

    
    HAL_TIM_PWM_ConfigChannel(&TIM8_Motor_Handle, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&TIM8_Motor_Handle, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&TIM8_Motor_Handle, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&TIM8_Motor_Handle, &sConfigOC, TIM_CHANNEL_4);

}

void TS_WHEELTEC_Driver_IO_init(void){
    GPIO_InitTypeDef GPIO_Initure={0};

    __HAL_RCC_GPIOI_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10;  
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      //复用模式
	GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //高速
	HAL_GPIO_Init(GPIOI,&GPIO_Initure);

    HAL_GPIO_WritePin(GPIOI,GPIO_PIN_0,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOI,GPIO_PIN_1,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOI,GPIO_PIN_2,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOI,GPIO_PIN_3,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOI,GPIO_PIN_4,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOI,GPIO_PIN_5,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOI,GPIO_PIN_7,GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOI,GPIO_PIN_10,GPIO_PIN_RESET);
}

void TS_TIM_Motor_PWMOUT_ENA(void){
    HAL_TIM_PWM_Start(&TIM8_Motor_Handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TIM8_Motor_Handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TIM8_Motor_Handle, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&TIM8_Motor_Handle, TIM_CHANNEL_4);

    HAL_GPIO_WritePin(GPIOI,GPIO_PIN_10,GPIO_PIN_SET);
}

void TS_TIM_Motor_PWMOUT_DISA(void){
    HAL_TIM_PWM_Stop(&TIM8_Motor_Handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&TIM8_Motor_Handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&TIM8_Motor_Handle, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&TIM8_Motor_Handle, TIM_CHANNEL_4);

    HAL_GPIO_WritePin(GPIOI,GPIO_PIN_10,GPIO_PIN_RESET);
}

void TS_TIM_Motor_PWM_duty_set(uint32_t CH1_8[8]){
    CH1_8[0]=CH1_8[0]>(TS_PWM_PRELOAD-1)?(TS_PWM_PRELOAD-1):CH1_8[0];
    CH1_8[1]=CH1_8[1]>(TS_PWM_PRELOAD-1)?(TS_PWM_PRELOAD-1):CH1_8[1];
    CH1_8[2]=CH1_8[2]>(TS_PWM_PRELOAD-1)?(TS_PWM_PRELOAD-1):CH1_8[2];
    CH1_8[3]=CH1_8[3]>(TS_PWM_PRELOAD-1)?(TS_PWM_PRELOAD-1):CH1_8[3];
    __HAL_TIM_SET_COMPARE(&TIM8_Motor_Handle, TIM_CHANNEL_1, CH1_8[0]);
    __HAL_TIM_SET_COMPARE(&TIM8_Motor_Handle, TIM_CHANNEL_2, CH1_8[1]);
    __HAL_TIM_SET_COMPARE(&TIM8_Motor_Handle, TIM_CHANNEL_3, CH1_8[2]);
    __HAL_TIM_SET_COMPARE(&TIM8_Motor_Handle, TIM_CHANNEL_4, CH1_8[3]);
}

#define DUTY_FILTER_TIME    (3)
void TS_set_Motor_Duty(void){
    static int32_t FL_Filter=0;
    static int32_t FR_Filter=0;
    static int32_t BL_Filter=0;
    static int32_t BR_Filter=0;

    int32_t FL_Duty=(int32_t)Duty_now.M_FL_Duty;
    int32_t FR_Duty=(int32_t)Duty_now.M_FR_Duty;
    int32_t BL_Duty=(int32_t)Duty_now.M_BL_Duty;
    int32_t BR_Duty=(int32_t)Duty_now.M_BR_Duty;

    // FL_Duty=FL_Duty>TS_PWM_PRELOAD?TS_PWM_PRELOAD:FL_Duty;
    // FL_Duty=FL_Duty<-TS_PWM_PRELOAD?-TS_PWM_PRELOAD:FL_Duty;
    // FR_Duty=FR_Duty>TS_PWM_PRELOAD?TS_PWM_PRELOAD:FR_Duty;
    // FR_Duty=FR_Duty<-TS_PWM_PRELOAD?-TS_PWM_PRELOAD:FR_Duty;
    // BL_Duty=BL_Duty>TS_PWM_PRELOAD?TS_PWM_PRELOAD:BL_Duty;
    // BL_Duty=BL_Duty<-TS_PWM_PRELOAD?-TS_PWM_PRELOAD:BL_Duty;
    // BR_Duty=BR_Duty>TS_PWM_PRELOAD?TS_PWM_PRELOAD:BR_Duty;
    // BR_Duty=BR_Duty<-TS_PWM_PRELOAD?-TS_PWM_PRELOAD:BR_Duty;

    FL_Duty=FL_Duty>TS_PWM_MAX?TS_PWM_MAX:FL_Duty;
    FL_Duty=FL_Duty<-TS_PWM_MAX?-TS_PWM_MAX:FL_Duty;
    FR_Duty=FR_Duty>TS_PWM_MAX?TS_PWM_MAX:FR_Duty;
    FR_Duty=FR_Duty<-TS_PWM_MAX?-TS_PWM_MAX:FR_Duty;
    BL_Duty=BL_Duty>TS_PWM_MAX?TS_PWM_MAX:BL_Duty;
    BL_Duty=BL_Duty<-TS_PWM_MAX?-TS_PWM_MAX:BL_Duty;
    BR_Duty=BR_Duty>TS_PWM_MAX?TS_PWM_MAX:BR_Duty;
    BR_Duty=BR_Duty<-TS_PWM_MAX?-TS_PWM_MAX:BR_Duty;

    if(fabs(FL_Duty)<50){
        FL_Duty=0;
    }    

    FL_Filter=FL_Filter*(DUTY_FILTER_TIME-1)/DUTY_FILTER_TIME+FL_Duty;
    FR_Filter=FR_Filter*(DUTY_FILTER_TIME-1)/DUTY_FILTER_TIME+FR_Duty;
    BL_Filter=BL_Filter*(DUTY_FILTER_TIME-1)/DUTY_FILTER_TIME+BL_Duty;
    BR_Filter=BR_Filter*(DUTY_FILTER_TIME-1)/DUTY_FILTER_TIME+BR_Duty;

    FL_Duty=FL_Filter/DUTY_FILTER_TIME;
    FR_Duty=FR_Filter/DUTY_FILTER_TIME;
    BL_Duty=BL_Filter/DUTY_FILTER_TIME;
    BR_Duty=BR_Filter/DUTY_FILTER_TIME;    

    uint32_t CH1_8[8]={0};

    // printf("%d\n",FL_Duty);
    if(BR_Duty>=0){
        CH1_8[0]=BR_Duty;
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_0,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_1,GPIO_PIN_RESET);
    }
    else{
        CH1_8[0]=-BR_Duty;
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_0,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_1,GPIO_PIN_SET);
    }

    if(BL_Duty>=0){
        CH1_8[1]=BL_Duty;
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_2,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_3,GPIO_PIN_RESET);
    }
    else{
        CH1_8[1]=-BL_Duty;
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_2,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_3,GPIO_PIN_SET);
    }

    if(FL_Duty>=0){
        CH1_8[2]=FL_Duty;
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_4,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_5,GPIO_PIN_SET);
    }
    else{
        CH1_8[2]=-FL_Duty;
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_4,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_5,GPIO_PIN_RESET);
    }

    if(FR_Duty>=0){
        CH1_8[3]=FR_Duty;
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_7,GPIO_PIN_SET);
    }
    else{
        CH1_8[3]=-FR_Duty;
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOI,GPIO_PIN_7,GPIO_PIN_RESET);        
    }

    // CH1_8[0]=1000;
    // CH1_8[1]=2000;
    // CH1_8[2]=3000;
    // CH1_8[3]=4000;

    CN_TIM_Motor_PWM_duty_set(CH1_8);
}
#endif 
#if USE_CHERRY_DRIVER
void CN_Motor_init(void){
    CN_TIM8_Motor_init();
    CN_TIM1_Motor_init();
    CN_TIM16_Motor_init();
}

void CN_TIM8_Motor_init(void){
    GPIO_InitTypeDef GPIO_Initure={0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;  
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用模式
	GPIO_Initure.Pull=GPIO_NOPULL;              //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //高速
    GPIO_Initure.Alternate=GPIO_AF3_TIM8;
	HAL_GPIO_Init(GPIOC,&GPIO_Initure);

    __HAL_RCC_TIM8_CLK_ENABLE(); 
    TIM8_Motor_Handle.Instance=TIM8;
    TIM8_Motor_Handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//200mhz
    TIM8_Motor_Handle.Init.Prescaler=10-1;//10mhz
    TIM8_Motor_Handle.Init.Period=CN_PWM_PRELOAD-1;//1khz
    TIM8_Motor_Handle.Init.CounterMode=TIM_COUNTERMODE_UP;
    TIM8_Motor_Handle.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&TIM8_Motor_Handle);

    TIM_OC_InitTypeDef sConfigOC={0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;  // PWM 模式 1
    sConfigOC.Pulse = 0;  // 设置占空比
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;  // 输出极性
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;  // 禁用快速模式
  
    HAL_TIM_PWM_ConfigChannel(&TIM8_Motor_Handle, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&TIM8_Motor_Handle, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&TIM8_Motor_Handle, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&TIM8_Motor_Handle, &sConfigOC, TIM_CHANNEL_4);

}

void CN_TIM1_Motor_init(void){
    GPIO_InitTypeDef GPIO_Initure={0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;  
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用推挽
	GPIO_Initure.Pull=GPIO_NOPULL;              //
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //高速
    GPIO_Initure.Alternate=GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);

    __HAL_RCC_TIM1_CLK_ENABLE(); 
    TIM1_Motor_Handle.Instance=TIM1;
    TIM1_Motor_Handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//200mhz
    TIM1_Motor_Handle.Init.Prescaler=10-1;//10mhz
    TIM1_Motor_Handle.Init.Period=CN_PWM_PRELOAD-1;//1khz
    TIM1_Motor_Handle.Init.CounterMode=TIM_COUNTERMODE_UP;
    TIM1_Motor_Handle.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&TIM1_Motor_Handle);

    TIM_OC_InitTypeDef sConfigOC={0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;  // PWM 模式 1
    sConfigOC.Pulse = 0;  // 设置占空比
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;  // 输出极性
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;  // 禁用快速模式

    
    HAL_TIM_PWM_ConfigChannel(&TIM1_Motor_Handle, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&TIM1_Motor_Handle, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&TIM1_Motor_Handle, &sConfigOC, TIM_CHANNEL_3);

}

void CN_TIM14_Motor_init(void){
    GPIO_InitTypeDef GPIO_Initure={0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_7;  
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用模式
	GPIO_Initure.Pull=GPIO_NOPULL;              //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //高速
    GPIO_Initure.Alternate=GPIO_AF9_TIM14;
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);

    __HAL_RCC_TIM14_CLK_ENABLE(); 
    TIM14_Motor_Handle.Instance=TIM14;
    TIM14_Motor_Handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//200mhz
    TIM14_Motor_Handle.Init.Prescaler=10-1;//10mhz
    TIM14_Motor_Handle.Init.Period=CN_PWM_PRELOAD-1;//1khz
    TIM14_Motor_Handle.Init.CounterMode=TIM_COUNTERMODE_UP;
    TIM14_Motor_Handle.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&TIM14_Motor_Handle);

    TIM_OC_InitTypeDef sConfigOC={0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;  // PWM 模式 1
    sConfigOC.Pulse = 0;  // 设置占空比
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;  // 输出极性
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;  // 禁用快速模式
    
    HAL_TIM_PWM_ConfigChannel(&TIM14_Motor_Handle, &sConfigOC, TIM_CHANNEL_1);

}

void CN_TIM16_Motor_init(void){
    GPIO_InitTypeDef GPIO_Initure={0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_8;  
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用模式
	GPIO_Initure.Pull=GPIO_NOPULL;              //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //高速
    GPIO_Initure.Alternate=GPIO_AF1_TIM16;
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);

    __HAL_RCC_TIM16_CLK_ENABLE(); 
    TIM16_Motor_Handle.Instance=TIM16;
    TIM16_Motor_Handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//200mhz
    TIM16_Motor_Handle.Init.Prescaler=10-1;//10mhz
    TIM16_Motor_Handle.Init.Period=CN_PWM_PRELOAD-1;//1khz
    TIM16_Motor_Handle.Init.CounterMode=TIM_COUNTERMODE_UP;
    TIM16_Motor_Handle.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&TIM16_Motor_Handle);

    TIM_OC_InitTypeDef sConfigOC={0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;  // PWM 模式 1
    sConfigOC.Pulse = 0;  // 设置占空比
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;  // 输出极性
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;  // 禁用快速模式

    
    HAL_TIM_PWM_ConfigChannel(&TIM16_Motor_Handle, &sConfigOC, TIM_CHANNEL_1);
}

void CN_TIM_Motor_PWMOUT_ENA(void){
    HAL_TIM_PWM_Start(&TIM8_Motor_Handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TIM8_Motor_Handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TIM8_Motor_Handle, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&TIM8_Motor_Handle, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&TIM1_Motor_Handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TIM1_Motor_Handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TIM1_Motor_Handle, TIM_CHANNEL_3);

    HAL_TIM_PWM_Start(&TIM16_Motor_Handle, TIM_CHANNEL_1);
}

void CN_TIM_Motor_PWMOUT_DISA(void){
    HAL_TIM_PWM_Stop(&TIM8_Motor_Handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&TIM8_Motor_Handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&TIM8_Motor_Handle, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&TIM8_Motor_Handle, TIM_CHANNEL_4);

    HAL_TIM_PWM_Stop(&TIM1_Motor_Handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&TIM1_Motor_Handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&TIM1_Motor_Handle, TIM_CHANNEL_3);

    HAL_TIM_PWM_Stop(&TIM16_Motor_Handle, TIM_CHANNEL_1);
}

void CN_TIM_Motor_PWM_duty_set(uint32_t CH1_8[8]){
    CH1_8[0]=CH1_8[0]>(CN_PWM_PRELOAD-1)?(CN_PWM_PRELOAD-1):CH1_8[0];
    CH1_8[1]=CH1_8[1]>(CN_PWM_PRELOAD-1)?(CN_PWM_PRELOAD-1):CH1_8[1];
    CH1_8[2]=CH1_8[2]>(CN_PWM_PRELOAD-1)?(CN_PWM_PRELOAD-1):CH1_8[2];
    CH1_8[3]=CH1_8[3]>(CN_PWM_PRELOAD-1)?(CN_PWM_PRELOAD-1):CH1_8[3];
    CH1_8[4]=CH1_8[4]>(CN_PWM_PRELOAD-1)?(CN_PWM_PRELOAD-1):CH1_8[4];
    CH1_8[5]=CH1_8[5]>(CN_PWM_PRELOAD-1)?(CN_PWM_PRELOAD-1):CH1_8[5];
    CH1_8[6]=CH1_8[6]>(CN_PWM_PRELOAD-1)?(CN_PWM_PRELOAD-1):CH1_8[6];
    CH1_8[7]=CH1_8[7]>(CN_PWM_PRELOAD-1)?(CN_PWM_PRELOAD-1):CH1_8[7];

    __HAL_TIM_SET_COMPARE(&TIM1_Motor_Handle, TIM_CHANNEL_3, CH1_8[0]);
    __HAL_TIM_SET_COMPARE(&TIM1_Motor_Handle, TIM_CHANNEL_1, CH1_8[1]);
    __HAL_TIM_SET_COMPARE(&TIM1_Motor_Handle, TIM_CHANNEL_2, CH1_8[2]);
    __HAL_TIM_SET_COMPARE(&TIM8_Motor_Handle, TIM_CHANNEL_4, CH1_8[3]);
    __HAL_TIM_SET_COMPARE(&TIM8_Motor_Handle, TIM_CHANNEL_1, CH1_8[4]);
    __HAL_TIM_SET_COMPARE(&TIM8_Motor_Handle, TIM_CHANNEL_2,CH1_8[5]);
    __HAL_TIM_SET_COMPARE(&TIM8_Motor_Handle, TIM_CHANNEL_3, CH1_8[6]);
    __HAL_TIM_SET_COMPARE(&TIM16_Motor_Handle, TIM_CHANNEL_1, CH1_8[7]);

    // printf("%d,%d\n",CH1_8[4],CH1_8[5]);

    // __HAL_TIM_SET_COMPARE(&TIM1_Motor_Handle, TIM_CHANNEL_3, 5000);
    // __HAL_TIM_SET_COMPARE(&TIM1_Motor_Handle, TIM_CHANNEL_1, 3000);
}

void CN_set_Motor_Duty(void){
    int32_t FL_Duty=(int32_t)Duty_now.M_FL_Duty;
    int32_t FR_Duty=(int32_t)Duty_now.M_FR_Duty;
    int32_t BL_Duty=(int32_t)Duty_now.M_BL_Duty;
    int32_t BR_Duty=(int32_t)Duty_now.M_BR_Duty;

    FL_Duty=FL_Duty>CN_PWM_MAX?CN_PWM_MAX:FL_Duty;
    FL_Duty=FL_Duty<-CN_PWM_MAX?-CN_PWM_MAX:FL_Duty;
    FR_Duty=FR_Duty>CN_PWM_MAX?CN_PWM_MAX:FR_Duty;
    FR_Duty=FR_Duty<-CN_PWM_MAX?-CN_PWM_MAX:FR_Duty;
    BL_Duty=BL_Duty>CN_PWM_MAX?CN_PWM_MAX:BL_Duty;
    BL_Duty=BL_Duty<-CN_PWM_MAX?-CN_PWM_MAX:BL_Duty;
    BR_Duty=BR_Duty>CN_PWM_MAX?CN_PWM_MAX:BR_Duty;
    BR_Duty=BR_Duty<-CN_PWM_MAX?-CN_PWM_MAX:BR_Duty;

    uint32_t CH1_8[8]={0};
    if(FL_Duty>=0){
        CH1_8[0]=FL_Duty;
        CH1_8[1]=0;
    }
    else{
        CH1_8[0]=0;
        CH1_8[1]=-FL_Duty;
    }

    if(FR_Duty>=0){
        CH1_8[2]=0;
        CH1_8[3]=FR_Duty;
    }
    else{
        CH1_8[2]=-FR_Duty;
        CH1_8[3]=0;
    }

    if(BL_Duty>=0){
        CH1_8[4]=BL_Duty;
        CH1_8[5]=0;
    }
    else{
        CH1_8[4]=0;
        CH1_8[5]=-BL_Duty;
    }

    if(BR_Duty>=0){
        CH1_8[6]=0;
        CH1_8[7]=BR_Duty;
    }
    else{
        CH1_8[6]=-BR_Duty;
        CH1_8[7]=0;
    }
    CN_TIM_Motor_PWM_duty_set(CH1_8);
}
#endif 
/****************************电机配置***************************/

/****************************电机功能函数***************************/
uint8_t Motor_Precise_Mode=0;

// 电机切换精确模式（高精度 低响应）
void CN_Motor_Change_To_Precise_Mode(void){
    Motor_Precise_Mode=1;
    init_PID_ctrler(&FL_vel_PID_ctrler,MOTOR_VEL_CL_PID_KP_PRECISE,MOTOR_VEL_CL_PID_KI_PRECISE,MOTOR_VEL_CL_PID_KD_PRECISE,MOTOR_VEL_CL_PID_IMAX_PRECISE,MOTOR_VEL_CL_PID_OUTMAX_PRECISE);
    init_PID_ctrler(&FR_vel_PID_ctrler,MOTOR_VEL_CL_PID_KP_PRECISE,MOTOR_VEL_CL_PID_KI_PRECISE,MOTOR_VEL_CL_PID_KD_PRECISE,MOTOR_VEL_CL_PID_IMAX_PRECISE,MOTOR_VEL_CL_PID_OUTMAX_PRECISE);
    init_PID_ctrler(&BL_vel_PID_ctrler,MOTOR_VEL_CL_PID_KP_PRECISE,MOTOR_VEL_CL_PID_KI_PRECISE,MOTOR_VEL_CL_PID_KD_PRECISE,MOTOR_VEL_CL_PID_IMAX_PRECISE,MOTOR_VEL_CL_PID_OUTMAX_PRECISE);
    init_PID_ctrler(&BR_vel_PID_ctrler,MOTOR_VEL_CL_PID_KP_PRECISE,MOTOR_VEL_CL_PID_KI_PRECISE,MOTOR_VEL_CL_PID_KD_PRECISE,MOTOR_VEL_CL_PID_IMAX_PRECISE,MOTOR_VEL_CL_PID_OUTMAX_PRECISE);

    init_PID_ctrler(&FL_pos_PID_ctrler,MOTOR_POS_CL_PID_KP_PRECISE,MOTOR_POS_CL_PID_KI_PRECISE,MOTOR_POS_CL_PID_KD_PRECISE,MOTOR_POS_CL_PID_IMAX_PRECISE,MOTOR_POS_CL_PID_OUTMAX_PRECISE);
    init_PID_ctrler(&FR_pos_PID_ctrler,MOTOR_POS_CL_PID_KP_PRECISE,MOTOR_POS_CL_PID_KI_PRECISE,MOTOR_POS_CL_PID_KD_PRECISE,MOTOR_POS_CL_PID_IMAX_PRECISE,MOTOR_POS_CL_PID_OUTMAX_PRECISE);
    init_PID_ctrler(&BL_pos_PID_ctrler,MOTOR_POS_CL_PID_KP_PRECISE,MOTOR_POS_CL_PID_KI_PRECISE,MOTOR_POS_CL_PID_KD_PRECISE,MOTOR_POS_CL_PID_IMAX_PRECISE,MOTOR_POS_CL_PID_OUTMAX_PRECISE);
    init_PID_ctrler(&BR_pos_PID_ctrler,MOTOR_POS_CL_PID_KP_PRECISE,MOTOR_POS_CL_PID_KI_PRECISE,MOTOR_POS_CL_PID_KD_PRECISE,MOTOR_POS_CL_PID_IMAX_PRECISE,MOTOR_POS_CL_PID_OUTMAX_PRECISE);

    printf("Motor change to precise mode\n");
}
// 电机切换正常模式（低精度 高响应）
void CN_Motor_Change_To_Normal_Mode(void){
    Motor_Precise_Mode=0;
    init_PID_ctrler(&FL_vel_PID_ctrler,MOTOR_VEL_CL_PID_KP,MOTOR_VEL_CL_PID_KI,MOTOR_VEL_CL_PID_KD,MOTOR_VEL_CL_PID_IMAX,MOTOR_VEL_CL_PID_OUTMAX);
    init_PID_ctrler(&FR_vel_PID_ctrler,MOTOR_VEL_CL_PID_KP,MOTOR_VEL_CL_PID_KI,MOTOR_VEL_CL_PID_KD,MOTOR_VEL_CL_PID_IMAX,MOTOR_VEL_CL_PID_OUTMAX);
    init_PID_ctrler(&BL_vel_PID_ctrler,MOTOR_VEL_CL_PID_KP,MOTOR_VEL_CL_PID_KI,MOTOR_VEL_CL_PID_KD,MOTOR_VEL_CL_PID_IMAX,MOTOR_VEL_CL_PID_OUTMAX);
    init_PID_ctrler(&BR_vel_PID_ctrler,MOTOR_VEL_CL_PID_KP,MOTOR_VEL_CL_PID_KI,MOTOR_VEL_CL_PID_KD,MOTOR_VEL_CL_PID_IMAX,MOTOR_VEL_CL_PID_OUTMAX);

    init_PID_ctrler(&FL_pos_PID_ctrler,MOTOR_POS_CL_PID_KP,MOTOR_POS_CL_PID_KI,MOTOR_POS_CL_PID_KD,MOTOR_POS_CL_PID_IMAX,MOTOR_POS_CL_PID_OUTMAX);
    init_PID_ctrler(&FR_pos_PID_ctrler,MOTOR_POS_CL_PID_KP,MOTOR_POS_CL_PID_KI,MOTOR_POS_CL_PID_KD,MOTOR_POS_CL_PID_IMAX,MOTOR_POS_CL_PID_OUTMAX);
    init_PID_ctrler(&BL_pos_PID_ctrler,MOTOR_POS_CL_PID_KP,MOTOR_POS_CL_PID_KI,MOTOR_POS_CL_PID_KD,MOTOR_POS_CL_PID_IMAX,MOTOR_POS_CL_PID_OUTMAX);
    init_PID_ctrler(&BR_pos_PID_ctrler,MOTOR_POS_CL_PID_KP,MOTOR_POS_CL_PID_KI,MOTOR_POS_CL_PID_KD,MOTOR_POS_CL_PID_IMAX,MOTOR_POS_CL_PID_OUTMAX);

    printf("Motor change to normal mode\n");
}
/****************************电机功能函数***************************/

/****************************电机转速环***************************/
static float milemeter=0.0f;

// 初始化电机转速环PID控制器
void CN_Motor_vel_CL_init(void){
    init_PID_ctrler(&FL_vel_PID_ctrler,MOTOR_VEL_CL_PID_KP,MOTOR_VEL_CL_PID_KI,MOTOR_VEL_CL_PID_KD,MOTOR_VEL_CL_PID_IMAX,MOTOR_VEL_CL_PID_OUTMAX);
    init_PID_ctrler(&FR_vel_PID_ctrler,MOTOR_VEL_CL_PID_KP,MOTOR_VEL_CL_PID_KI,MOTOR_VEL_CL_PID_KD,MOTOR_VEL_CL_PID_IMAX,MOTOR_VEL_CL_PID_OUTMAX);
    init_PID_ctrler(&BL_vel_PID_ctrler,MOTOR_VEL_CL_PID_KP,MOTOR_VEL_CL_PID_KI,MOTOR_VEL_CL_PID_KD,MOTOR_VEL_CL_PID_IMAX,MOTOR_VEL_CL_PID_OUTMAX);
    init_PID_ctrler(&BR_vel_PID_ctrler,MOTOR_VEL_CL_PID_KP,MOTOR_VEL_CL_PID_KI,MOTOR_VEL_CL_PID_KD,MOTOR_VEL_CL_PID_IMAX,MOTOR_VEL_CL_PID_OUTMAX);
}

// 设置目标转速
void CN_Motor_vel_CL_set_aim(Motor_vel_struct aim){
    Vel_aim=aim;
}

// 电机转速环输出更新
void CN_Motor_vel_CL_CALLBACK(uint32_t us_time){
    Vel_now.M_FL_Vel=ENC_data[0];
    Vel_now.M_FR_Vel=ENC_data[1];
    Vel_now.M_BL_Vel=ENC_data[2];
    Vel_now.M_BR_Vel=ENC_data[3];

    // 里程数更新（相对于上位机之前的设定值）
    milemeter+=(Vel_now.M_FL_Vel+
               Vel_now.M_FR_Vel+
               Vel_now.M_BL_Vel+
               Vel_now.M_BR_Vel)/4.0f;

    // 增量式PID控制器
    PID_ctrler_update_with_time(&FL_vel_PID_ctrler,Vel_aim.M_FL_Vel-Vel_now.M_FL_Vel,us_time);
    PID_ctrler_update_with_time(&FR_vel_PID_ctrler,Vel_aim.M_FR_Vel-Vel_now.M_FR_Vel,us_time);
    PID_ctrler_update_with_time(&BL_vel_PID_ctrler,Vel_aim.M_BL_Vel-Vel_now.M_BL_Vel,us_time);
    PID_ctrler_update_with_time(&BR_vel_PID_ctrler,Vel_aim.M_BR_Vel-Vel_now.M_BR_Vel,us_time);
    Duty_now.M_FL_Duty+=FL_vel_PID_ctrler.output;
    Duty_now.M_FR_Duty+=FR_vel_PID_ctrler.output;
    Duty_now.M_BL_Duty+=BL_vel_PID_ctrler.output;
    Duty_now.M_BR_Duty+=BR_vel_PID_ctrler.output;
    
    // 限制输出
    Duty_now.M_FL_Duty=Duty_now.M_FL_Duty>CN_PWM_MAX?CN_PWM_MAX:Duty_now.M_FL_Duty;
    Duty_now.M_FL_Duty=Duty_now.M_FL_Duty<-CN_PWM_MAX?-CN_PWM_MAX:Duty_now.M_FL_Duty;
    Duty_now.M_FR_Duty=Duty_now.M_FR_Duty>CN_PWM_MAX?CN_PWM_MAX:Duty_now.M_FR_Duty;
    Duty_now.M_FR_Duty=Duty_now.M_FR_Duty<-CN_PWM_MAX?-CN_PWM_MAX:Duty_now.M_FR_Duty;
    Duty_now.M_BL_Duty=Duty_now.M_BL_Duty>CN_PWM_MAX?CN_PWM_MAX:Duty_now.M_BL_Duty;
    Duty_now.M_BL_Duty=Duty_now.M_BL_Duty<-CN_PWM_MAX?-CN_PWM_MAX:Duty_now.M_BL_Duty;
    Duty_now.M_BR_Duty=Duty_now.M_BR_Duty>CN_PWM_MAX?CN_PWM_MAX:Duty_now.M_BR_Duty;
    Duty_now.M_BR_Duty=Duty_now.M_BR_Duty<-CN_PWM_MAX?-CN_PWM_MAX:Duty_now.M_BR_Duty;
    printf("%d,%d,%d,%d\n",(int)Duty_now.M_FL_Duty,(int)Duty_now.M_FR_Duty,(int)Duty_now.M_BL_Duty,(int)Duty_now.M_BR_Duty);
    
    // 电机输出
    CN_set_Motor_Duty();
}

// 失控全部清空
void CN_Motor_outline(void){
    Vel_aim.M_BL_Vel=0;
    Vel_aim.M_BR_Vel=0;
    Vel_aim.M_FL_Vel=0;
    Vel_aim.M_FR_Vel=0;
    Duty_now.M_BL_Duty=0;
    Duty_now.M_BR_Duty=0;
    Duty_now.M_FL_Duty=0;
    Duty_now.M_FR_Duty=0;
    CN_set_Motor_Duty();
    SYS_STATUS.bits.SYS_INLINE=0;
}

// 上位机设置里程数
void CN_Milemeter_set(float milemeter_value){
    milemeter=milemeter_value;
}

// 上位机获取里程数
float CN_get_Milemeter(void){
    return milemeter;
}
/****************************电机转速环***************************/

/****************************电机位置环***************************/
void CN_Motor_pos_CL_init(void){//初始化电机位置环PID控制器
    CN_Motor_vel_CL_init();//需要首先初始化内环
    init_PID_ctrler(&FL_pos_PID_ctrler,MOTOR_POS_CL_PID_KP,MOTOR_POS_CL_PID_KI,MOTOR_POS_CL_PID_KD,MOTOR_POS_CL_PID_IMAX,MOTOR_POS_CL_PID_OUTMAX);
    init_PID_ctrler(&FR_pos_PID_ctrler,MOTOR_POS_CL_PID_KP,MOTOR_POS_CL_PID_KI,MOTOR_POS_CL_PID_KD,MOTOR_POS_CL_PID_IMAX,MOTOR_POS_CL_PID_OUTMAX);
    init_PID_ctrler(&BL_pos_PID_ctrler,MOTOR_POS_CL_PID_KP,MOTOR_POS_CL_PID_KI,MOTOR_POS_CL_PID_KD,MOTOR_POS_CL_PID_IMAX,MOTOR_POS_CL_PID_OUTMAX);
    init_PID_ctrler(&BR_pos_PID_ctrler,MOTOR_POS_CL_PID_KP,MOTOR_POS_CL_PID_KI,MOTOR_POS_CL_PID_KD,MOTOR_POS_CL_PID_IMAX,MOTOR_POS_CL_PID_OUTMAX);
}

void CN_Motor_pos_CL_CALLBACK(uint32_t us_time){
    Pos_now.M_FL_Pos+=Vel_now.M_FL_Vel;
    Pos_now.M_FR_Pos+=Vel_now.M_FR_Vel;
    Pos_now.M_BL_Pos+=Vel_now.M_BL_Vel;
    Pos_now.M_BR_Pos+=Vel_now.M_BR_Vel;
    //对编码器速度积分得到位置(无损失)

    float FL_Pos_err=Pos_aim.M_FL_Pos-Pos_now.M_FL_Pos;
    float FR_Pos_err=Pos_aim.M_FR_Pos-Pos_now.M_FR_Pos;
    float BL_Pos_err=Pos_aim.M_BL_Pos-Pos_now.M_BL_Pos;
    float BR_Pos_err=Pos_aim.M_BR_Pos-Pos_now.M_BR_Pos;

    // FL_Pos_err=fabs(FL_Pos_err)<30?0:FL_Pos_err;
    // FR_Pos_err=fabs(FR_Pos_err)<30?0:FR_Pos_err;
    // BL_Pos_err=fabs(BL_Pos_err)<30?0:BL_Pos_err;
    // BR_Pos_err=fabs(BR_Pos_err)<30?0:BR_Pos_err;

    PID_ctrler_update_with_time(&FL_pos_PID_ctrler,FL_Pos_err,us_time);
    PID_ctrler_update_with_time(&FR_pos_PID_ctrler,FR_Pos_err,us_time);
    PID_ctrler_update_with_time(&BL_pos_PID_ctrler,BL_Pos_err,us_time);
    PID_ctrler_update_with_time(&BR_pos_PID_ctrler,BR_Pos_err,us_time);
    //更新位置环输出

    Vel_aim.M_FL_Vel=FL_pos_PID_ctrler.output;
    Vel_aim.M_FR_Vel=FR_pos_PID_ctrler.output;
    Vel_aim.M_BL_Vel=BL_pos_PID_ctrler.output;
    Vel_aim.M_BR_Vel=BR_pos_PID_ctrler.output;
    //将位置环输出作为转速环输入

    // printf("%.2f,%.2f,%.2f,%.2f\n",Pos_now.M_FL_Pos,Pos_now.M_FR_Pos,Pos_now.M_BL_Pos,Pos_now.M_BR_Pos);

    CN_Motor_vel_CL_CALLBACK(us_time);//调用速度环更新
}

#define SLOW_THRESHOLD (300)
void CN_Motor_pos_CL_set_aim(Motor_pos_struct aim){//设置位置环目标(一次性)
    if(Motor_Precise_Mode){
        init_PID_ctrler(&FL_pos_PID_ctrler,MOTOR_POS_CL_PID_KP_PRECISE,MOTOR_POS_CL_PID_KI_PRECISE,MOTOR_POS_CL_PID_KD_PRECISE,MOTOR_POS_CL_PID_IMAX_PRECISE,MOTOR_POS_CL_PID_OUTMAX_PRECISE);
        init_PID_ctrler(&FR_pos_PID_ctrler,MOTOR_POS_CL_PID_KP_PRECISE,MOTOR_POS_CL_PID_KI_PRECISE,MOTOR_POS_CL_PID_KD_PRECISE,MOTOR_POS_CL_PID_IMAX_PRECISE,MOTOR_POS_CL_PID_OUTMAX_PRECISE);
        init_PID_ctrler(&BL_pos_PID_ctrler,MOTOR_POS_CL_PID_KP_PRECISE,MOTOR_POS_CL_PID_KI_PRECISE,MOTOR_POS_CL_PID_KD_PRECISE,MOTOR_POS_CL_PID_IMAX_PRECISE,MOTOR_POS_CL_PID_OUTMAX_PRECISE);
        init_PID_ctrler(&BR_pos_PID_ctrler,MOTOR_POS_CL_PID_KP_PRECISE,MOTOR_POS_CL_PID_KI_PRECISE,MOTOR_POS_CL_PID_KD_PRECISE,MOTOR_POS_CL_PID_IMAX_PRECISE,MOTOR_POS_CL_PID_OUTMAX_PRECISE);
    }
    else{
        if(abs(aim.M_FL_Pos)<SLOW_THRESHOLD &&
        abs(aim.M_FR_Pos)<SLOW_THRESHOLD &&
        abs(aim.M_BL_Pos)<SLOW_THRESHOLD &&
        abs(aim.M_BR_Pos)<SLOW_THRESHOLD){
            init_PID_ctrler(&FL_pos_PID_ctrler,MOTOR_POS_CL_SLOW_PID_KP,MOTOR_POS_CL_SLOW_PID_KI,MOTOR_POS_CL_SLOW_PID_KD,MOTOR_POS_CL_SLOW_PID_IMAX,MOTOR_POS_CL_SLOW_PID_OUTMAX);
            init_PID_ctrler(&FR_pos_PID_ctrler,MOTOR_POS_CL_SLOW_PID_KP,MOTOR_POS_CL_SLOW_PID_KI,MOTOR_POS_CL_SLOW_PID_KD,MOTOR_POS_CL_SLOW_PID_IMAX,MOTOR_POS_CL_SLOW_PID_OUTMAX);
            init_PID_ctrler(&BL_pos_PID_ctrler,MOTOR_POS_CL_SLOW_PID_KP,MOTOR_POS_CL_SLOW_PID_KI,MOTOR_POS_CL_SLOW_PID_KD,MOTOR_POS_CL_SLOW_PID_IMAX,MOTOR_POS_CL_SLOW_PID_OUTMAX);
            init_PID_ctrler(&BR_pos_PID_ctrler,MOTOR_POS_CL_SLOW_PID_KP,MOTOR_POS_CL_SLOW_PID_KI,MOTOR_POS_CL_SLOW_PID_KD,MOTOR_POS_CL_SLOW_PID_IMAX,MOTOR_POS_CL_SLOW_PID_OUTMAX);
        }
        else{
            init_PID_ctrler(&FL_pos_PID_ctrler,MOTOR_POS_CL_PID_KP,MOTOR_POS_CL_PID_KI,MOTOR_POS_CL_PID_KD,MOTOR_POS_CL_PID_IMAX,MOTOR_POS_CL_PID_OUTMAX);
            init_PID_ctrler(&FR_pos_PID_ctrler,MOTOR_POS_CL_PID_KP,MOTOR_POS_CL_PID_KI,MOTOR_POS_CL_PID_KD,MOTOR_POS_CL_PID_IMAX,MOTOR_POS_CL_PID_OUTMAX);
            init_PID_ctrler(&BL_pos_PID_ctrler,MOTOR_POS_CL_PID_KP,MOTOR_POS_CL_PID_KI,MOTOR_POS_CL_PID_KD,MOTOR_POS_CL_PID_IMAX,MOTOR_POS_CL_PID_OUTMAX);
            init_PID_ctrler(&BR_pos_PID_ctrler,MOTOR_POS_CL_PID_KP,MOTOR_POS_CL_PID_KI,MOTOR_POS_CL_PID_KD,MOTOR_POS_CL_PID_IMAX,MOTOR_POS_CL_PID_OUTMAX);
        }        
    }

    Pos_aim=aim;//设置目标位置
    // printf("%f\n",Pos_aim.M_FL_Pos);
    Pos_now.M_FL_Pos=0;
    Pos_now.M_FR_Pos=0;
    Pos_now.M_BL_Pos=0;
    Pos_now.M_BR_Pos=0;
    //清空当前位置
}
/****************************电机位置环***************************/



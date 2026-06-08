#include "ChaseNum1_Include.h"
#include <math.h>

#define CN_PWM_PRELOAD  (10000)
#define CN_PWM_MAX  (8000-1)

Motor_duty_struct Duty_now={0};
Motor_vel_struct Vel_now={0};
Motor_vel_struct Vel_aim={0};

PID_ctrler FL_vel_PID_ctrler={0};
PID_ctrler FR_vel_PID_ctrler={0};
PID_ctrler BL_vel_PID_ctrler={0};
PID_ctrler BR_vel_PID_ctrler={0};


#define MOTOR_VEL_CL_PID_KP (50)
#define MOTOR_VEL_CL_PID_KI (0)
#define MOTOR_VEL_CL_PID_KD (0)
#define MOTOR_VEL_CL_PID_IMAX (0)
#define MOTOR_VEL_CL_PID_OUTMAX (800)

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
/*
    PWM1: PC8 TIM8CH3, PC7 TIM8CH2;
    PWM2: PC6 TIM8CH1, PH6 TIM12CH1;
    PWM3: PA10 TIM1CH3, PA8 TIM1Ch1;
    PWM4: PA9 TIM1CH2, PC9 TIM8CH4;
*/

// 电机 - 驱动初始化
void CN_Motor_init(void){
    CN_TIM8_Motor_init();
    CN_TIM1_Motor_init();
    CN_TIM12_Motor_init();
}

// 电机 - TIM1初始化
TIM_HandleTypeDef TIM1_Motor_Handle={0};
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

// 电机 - TIM8初始化
TIM_HandleTypeDef TIM8_Motor_Handle={0};
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

// 电机 - TIM12初始化
TIM_HandleTypeDef TIM12_Motor_Handle={0};
void CN_TIM12_Motor_init(void){
    GPIO_InitTypeDef GPIO_Initure={0};

    __HAL_RCC_GPIOH_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_6;  
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用模式
	GPIO_Initure.Pull=GPIO_NOPULL;              //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //高速
    GPIO_Initure.Alternate=GPIO_AF2_TIM12;
	HAL_GPIO_Init(GPIOH,&GPIO_Initure);

    __HAL_RCC_TIM12_CLK_ENABLE(); 
    TIM12_Motor_Handle.Instance=TIM12;
    TIM12_Motor_Handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//200mhz
    TIM12_Motor_Handle.Init.Prescaler=10-1;//10mhz
    TIM12_Motor_Handle.Init.Period=CN_PWM_PRELOAD-1;//1khz
    TIM12_Motor_Handle.Init.CounterMode=TIM_COUNTERMODE_UP;
    TIM12_Motor_Handle.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&TIM12_Motor_Handle);

    TIM_OC_InitTypeDef sConfigOC={0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;  // PWM 模式 1
    sConfigOC.Pulse = 0;  // 设置占空比
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;  // 输出极性
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;  // 禁用快速模式

    HAL_TIM_PWM_ConfigChannel(&TIM12_Motor_Handle, &sConfigOC, TIM_CHANNEL_1);
}

// 电机 - 输出使能
void CN_TIM_Motor_PWMOUT_ENA(void){
    HAL_TIM_PWM_Start(&TIM8_Motor_Handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TIM8_Motor_Handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TIM8_Motor_Handle, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&TIM8_Motor_Handle, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&TIM1_Motor_Handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TIM1_Motor_Handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TIM1_Motor_Handle, TIM_CHANNEL_3);

    HAL_TIM_PWM_Start(&TIM12_Motor_Handle, TIM_CHANNEL_1);
}

// 电机 - 输出失能
void CN_TIM_Motor_PWMOUT_DISA(void){
    HAL_TIM_PWM_Stop(&TIM8_Motor_Handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&TIM8_Motor_Handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&TIM8_Motor_Handle, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&TIM8_Motor_Handle, TIM_CHANNEL_4);

    HAL_TIM_PWM_Stop(&TIM1_Motor_Handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&TIM1_Motor_Handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&TIM1_Motor_Handle, TIM_CHANNEL_3);

    HAL_TIM_PWM_Stop(&TIM12_Motor_Handle, TIM_CHANNEL_1);
}

// 电机 - 双通道占空比设置
void CN_TIM_Motor_PWM_duty_set(uint32_t CH1_8[8]){
    CH1_8[0]=CH1_8[0]>(CN_PWM_PRELOAD-1)?(CN_PWM_PRELOAD-1):CH1_8[0];
    CH1_8[1]=CH1_8[1]>(CN_PWM_PRELOAD-1)?(CN_PWM_PRELOAD-1):CH1_8[1];
    CH1_8[2]=CH1_8[2]>(CN_PWM_PRELOAD-1)?(CN_PWM_PRELOAD-1):CH1_8[2];
    CH1_8[3]=CH1_8[3]>(CN_PWM_PRELOAD-1)?(CN_PWM_PRELOAD-1):CH1_8[3];
    CH1_8[4]=CH1_8[4]>(CN_PWM_PRELOAD-1)?(CN_PWM_PRELOAD-1):CH1_8[4];
    CH1_8[5]=CH1_8[5]>(CN_PWM_PRELOAD-1)?(CN_PWM_PRELOAD-1):CH1_8[5];
    CH1_8[6]=CH1_8[6]>(CN_PWM_PRELOAD-1)?(CN_PWM_PRELOAD-1):CH1_8[6];
    CH1_8[7]=CH1_8[7]>(CN_PWM_PRELOAD-1)?(CN_PWM_PRELOAD-1):CH1_8[7];

    /*
    PWM1: PC8 TIM8CH3, PC7 TIM8CH2;
    PWM2: PC6 TIM8CH1, PH6 TIM12CH1;
    PWM3: PA10 TIM1CH3, PA8 TIM1Ch1;
    PWM4: PA9 TIM1CH2, PC9 TIM8CH4;
    */
    __HAL_TIM_SET_COMPARE(&TIM8_Motor_Handle, TIM_CHANNEL_3, CH1_8[0]);
    __HAL_TIM_SET_COMPARE(&TIM8_Motor_Handle, TIM_CHANNEL_2, CH1_8[1]);
    __HAL_TIM_SET_COMPARE(&TIM8_Motor_Handle, TIM_CHANNEL_1, CH1_8[2]);
    __HAL_TIM_SET_COMPARE(&TIM12_Motor_Handle, TIM_CHANNEL_1, CH1_8[3]);
    __HAL_TIM_SET_COMPARE(&TIM1_Motor_Handle, TIM_CHANNEL_3, CH1_8[4]);
    __HAL_TIM_SET_COMPARE(&TIM1_Motor_Handle, TIM_CHANNEL_1,CH1_8[5]);
    __HAL_TIM_SET_COMPARE(&TIM1_Motor_Handle, TIM_CHANNEL_2, CH1_8[6]);
    __HAL_TIM_SET_COMPARE(&TIM8_Motor_Handle, TIM_CHANNEL_4, CH1_8[7]);
}

// 摩擦力死区补偿值：给多大的PWM值，轮胎刚好悬空微转/克服静摩擦 (需要实际上机测试)
#define MOTOR_DEADBAND_PWM 1500  

// 速度噪声死区容忍度：低于这个速度的编码器波动视为静止噪声
#define VEL_NOISE_MARGIN 1.0f

// 电机 - 设置占空比
void CN_set_Motor_Duty(void){
    int32_t FL_Duty=(int32_t)Duty_now.M_FL_Duty;
    int32_t FR_Duty=(int32_t)Duty_now.M_FR_Duty;
    int32_t BL_Duty=(int32_t)Duty_now.M_BL_Duty;
    int32_t BR_Duty=(int32_t)Duty_now.M_BR_Duty;

    // 静止状态防微震处理 
    // 逻辑：如果上位机/运动学解算下发的目标速度为0，且当前实际速度极小
    // 则强行掐断输出，并清零PID积累，防止传感器噪声驱动电机。
    if (Vel_aim.M_FL_Vel == 0 && fabs(Vel_now.M_FL_Vel) < VEL_NOISE_MARGIN) {
        FL_Duty = 0;
        Duty_now.M_FL_Duty = 0; 
    }
    if (Vel_aim.M_FR_Vel == 0 && fabs(Vel_now.M_FR_Vel) < VEL_NOISE_MARGIN) {
        FR_Duty = 0;
        Duty_now.M_FR_Duty = 0;
    }
    if (Vel_aim.M_BL_Vel == 0 && fabs(Vel_now.M_BL_Vel) < VEL_NOISE_MARGIN) {
        BL_Duty = 0;
        Duty_now.M_BL_Duty = 0;
    }
    if (Vel_aim.M_BR_Vel == 0 && fabs(Vel_now.M_BR_Vel) < VEL_NOISE_MARGIN) {
        BR_Duty = 0;
        Duty_now.M_BR_Duty = 0;
    }

    // 死区（静摩擦力）前馈补偿
    // 逻辑：如果经过上述判断后，仍需要输出力矩，则直接跨过物理死区
    if (FL_Duty > 0) FL_Duty += MOTOR_DEADBAND_PWM;
    else if (FL_Duty < 0) FL_Duty -= MOTOR_DEADBAND_PWM;

    if (FR_Duty > 0) FR_Duty += MOTOR_DEADBAND_PWM;
    else if (FR_Duty < 0) FR_Duty -= MOTOR_DEADBAND_PWM;

    if (BL_Duty > 0) BL_Duty += MOTOR_DEADBAND_PWM;
    else if (BL_Duty < 0) BL_Duty -= MOTOR_DEADBAND_PWM;

    if (BR_Duty > 0) BR_Duty += MOTOR_DEADBAND_PWM;
    else if (BR_Duty < 0) BR_Duty -= MOTOR_DEADBAND_PWM;

    FL_Duty=FL_Duty>CN_PWM_MAX?CN_PWM_MAX:FL_Duty;
    FL_Duty=FL_Duty<-CN_PWM_MAX?-CN_PWM_MAX:FL_Duty;
    FR_Duty=FR_Duty>CN_PWM_MAX?CN_PWM_MAX:FR_Duty;
    FR_Duty=FR_Duty<-CN_PWM_MAX?-CN_PWM_MAX:FR_Duty;
    BL_Duty=BL_Duty>CN_PWM_MAX?CN_PWM_MAX:BL_Duty;
    BL_Duty=BL_Duty<-CN_PWM_MAX?-CN_PWM_MAX:BL_Duty;
    BR_Duty=BR_Duty>CN_PWM_MAX?CN_PWM_MAX:BR_Duty;
    BR_Duty=BR_Duty<-CN_PWM_MAX?-CN_PWM_MAX:BR_Duty;

    uint32_t CH1_8[8]={0};
    if(BL_Duty>=0){
        CH1_8[0]=0;
        CH1_8[1]=BL_Duty;
    }
    else{
        CH1_8[0]=-BL_Duty;
        CH1_8[1]=0;
    }

    if(FL_Duty>=0){        
        CH1_8[2]=0;
        CH1_8[3]=FL_Duty;
    }
    else{
        CH1_8[2]=-FL_Duty;
        CH1_8[3]=0;
    }

    if(FR_Duty>=0){
        CH1_8[4]=FR_Duty;
        CH1_8[5]=0;
    }
    else{
        CH1_8[4]=0;
        CH1_8[5]=-FR_Duty;
    }

    if(BR_Duty>=0){
        CH1_8[6]=BR_Duty;
        CH1_8[7]=0;
    }
    else{
        CH1_8[6]=0;
        CH1_8[7]=-BR_Duty;
    }
    CN_TIM_Motor_PWM_duty_set(CH1_8);
}

// 电机 - 关闭输出
void CN_stop_Motor_Duty(void){
    uint32_t CH1_8[8]={0};
    
    CH1_8[0]=0;
    CH1_8[1]=0;
    CH1_8[2]=0;
    CH1_8[3]=0;
    CH1_8[4]=0;
    CH1_8[5]=0;
    CH1_8[6]=0;
    CH1_8[7]=0;

    CN_TIM_Motor_PWM_duty_set(CH1_8);
}
/****************************电机配置***************************/

/****************************电机转速环***************************/
static float milemeter=0.0f;

// 初始化电机转速环PID控制器
void CN_Motor_vel_CL_init(void){
    init_PID_ctrler(&FL_vel_PID_ctrler,MOTOR_VEL_CL_PID_KP,MOTOR_VEL_CL_PID_KI,MOTOR_VEL_CL_PID_KD,MOTOR_VEL_CL_PID_IMAX,MOTOR_VEL_CL_PID_OUTMAX);
    init_PID_ctrler(&FR_vel_PID_ctrler,MOTOR_VEL_CL_PID_KP,MOTOR_VEL_CL_PID_KI,MOTOR_VEL_CL_PID_KD,MOTOR_VEL_CL_PID_IMAX,MOTOR_VEL_CL_PID_OUTMAX);
    init_PID_ctrler(&BL_vel_PID_ctrler,MOTOR_VEL_CL_PID_KP,MOTOR_VEL_CL_PID_KI,MOTOR_VEL_CL_PID_KD,MOTOR_VEL_CL_PID_IMAX,MOTOR_VEL_CL_PID_OUTMAX);
    init_PID_ctrler(&BR_vel_PID_ctrler,MOTOR_VEL_CL_PID_KP,MOTOR_VEL_CL_PID_KI,MOTOR_VEL_CL_PID_KD,MOTOR_VEL_CL_PID_IMAX,MOTOR_VEL_CL_PID_OUTMAX);
    ENC_data[0] = 0;
    ENC_data[1] = 0;
    ENC_data[2] = 0;
    ENC_data[3] = 0;
}

// 设置目标转速
void CN_Motor_vel_CL_set_aim(Motor_vel_struct aim){
    Vel_aim=aim;
}

// 电机转速环输出更新
void CN_Motor_vel_CL_CALLBACK(uint32_t us_time){

    Vel_now.M_FL_Vel=(float)ENC_data[3];
    Vel_now.M_FR_Vel=(float)ENC_data[1];
    Vel_now.M_BL_Vel=(float)ENC_data[2];
    Vel_now.M_BR_Vel=(float)ENC_data[0];

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

   // printf("%d,%d,%d,%d\n",(int)Duty_now.M_FL_Duty,(int)Duty_now.M_FR_Duty,(int)Duty_now.M_BL_Duty,(int)Duty_now.M_BR_Duty);
    
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
}

/****************************电机转速环***************************/




#include "ChaseNum1_Include.h"

#define TS_SERVO_PWM_PRELOAD  (50000)

TIM_HandleTypeDef TIM12_Servo_Handle={0};
TIM_HandleTypeDef TIM15_Servo_Handle={0};


SERVO_Handle_struct SERVO_Gripper_yaw_Handle={0};
SERVO_Handle_struct SERVO_Gripper_pitch1_Handle={0};
SERVO_Handle_struct SERVO_Gripper_pitch2_Handle={0};
SERVO_Handle_struct SERVO_Shooter_reload={0};

uint8_t TS_SERVO_operate(SERVO_Handle_struct *H_SERVO,Action_method_ENUM OPE_SELECT,int data){
    switch(OPE_SELECT){
        case Action_method_absolute:{
            data=data>H_SERVO->input_max?H_SERVO->input_max:data;
            data=data<H_SERVO->input_min?H_SERVO->input_min:data;
            switch(H_SERVO->SERVO_SELECT){
                case Servo_Gripper_yaw:{
                    TS_Set_Servo_Gripper_yaw_slow(data);  
                    H_SERVO->pos_now=data;
                    break;
                }
            }
            break;
        }
        case Action_method_incremental:{
            int pos=data+H_SERVO->pos_now>H_SERVO->input_max?H_SERVO->input_max:data+H_SERVO->pos_now;
            pos=data+H_SERVO->pos_now<H_SERVO->input_min?H_SERVO->input_min:data+H_SERVO->pos_now;                    
            switch(H_SERVO->SERVO_SELECT){
                case Servo_Gripper_yaw:{
                    TS_Set_Servo_Gripper_yaw_slow(data);  
                    break;
                }
            }       
            H_SERVO->pos_now=pos;
            break;
        }
    }
}

/************************************************应用层*************************************************/
void TS_Set_Servo_Gripper_yaw_slow(float yaw){
    yaw=yaw>90.0f?90.0f:yaw;
    yaw=yaw<-90.0f?-90.0f:yaw;

    int duty=(int)TS_linear_map_with_mid_float(yaw,-90.0f,0.0f,90.0f,TS_SERVO_GRIPPER_YAW_090_DUTY,TS_SERVO_GRIPPER_YAW_0_DUTY,TS_SERVO_GRIPPER_YAW_90_DUTY);
    TS_Servo_Duty_Set_slow(Servo_Gripper_yaw,duty,50);
}

void TS_SET_Servo_Gripper_yaw(float yaw){//定义为-90-90
    yaw=yaw>90.0f?90.0f:yaw;
    yaw=yaw<-90.0f?-90.0f:yaw;

    int duty=(int)TS_linear_map_with_mid_float(yaw,-90.0f,0.0f,90.0f,TS_SERVO_GRIPPER_YAW_090_DUTY,TS_SERVO_GRIPPER_YAW_0_DUTY,TS_SERVO_GRIPPER_YAW_90_DUTY);
    TS_Servo_Duty_Set(Servo_Gripper_yaw,duty);
    SERVO_Gripper_yaw_Handle.Duty_now=duty;
    // printf("set:%d\n",duty);
}

void TS_Change_to_Horizontal(void){
    TS_Servo_Duty_Set_slow(Servo_Gripper_pitch1,TS_SERVO_GRIPPER_PITCH1_HORIZONTAL_DUTY,50);
    TS_Servo_Duty_Set_slow(Servo_Gripper_pitch2,TS_SERVO_GRIPPER_PITCH2_HORIZONTAL_DUTY,50);    
}

void TS_Change_to_OverLook(void){
    TS_Servo_Duty_Set_slow(Servo_Gripper_pitch1,TS_SERVO_GRIPPER_PITCH1_OVERLOOK_DUTY,50);
    TS_Servo_Duty_Set_slow(Servo_Gripper_pitch2,TS_SERVO_GRIPPER_PITCH2_OVERLOOK_DUTY,50);    
}

void TS_Change_to_Teaching_1(void){
    TS_Servo_Duty_Set_slow(Servo_Gripper_pitch1,TS_SERVO_GRIPPER_PITCH1_TEACHING1_DUTY,50);
    TS_Servo_Duty_Set_slow(Servo_Gripper_pitch2,TS_SERVO_GRIPPER_PITCH2_TEACHING1_DUTY,50);    
}

void TS_Change_to_Teaching_2(void){
    TS_Servo_Duty_Set_slow(Servo_Gripper_pitch1,TS_SERVO_GRIPPER_PITCH1_TEACHING2_DUTY,50);
    TS_Servo_Duty_Set_slow(Servo_Gripper_pitch2,TS_SERVO_GRIPPER_PITCH2_TEACHING2_DUTY,50);    
}

void TS_Change_to_Grip(void){
    TS_Servo_Duty_Set_slow(Servo_Gripper_pitch1,TS_SERVO_GRIPPER_PITCH1_GRIP_DUTY,50);
    TS_Servo_Duty_Set_slow(Servo_Gripper_pitch2,TS_SERVO_GRIPPER_PITCH2_GRIP_DUTY,50);
}

void TS_Servo_Reset(void){
    TS_Servo_Duty_Set(Servo_Gripper_yaw,TS_SERVO_GRIPPER_YAW_0_DUTY);
    TS_Servo_Duty_Set(Servo_Gripper_pitch1,TS_SERVO_GRIPPER_PITCH1_HORIZONTAL_DUTY);
    TS_Servo_Duty_Set(Servo_Gripper_pitch2,TS_SERVO_GRIPPER_PITCH2_HORIZONTAL_DUTY);  
    TS_Servo_Duty_Set(Servo_Shooter_reload,TS_SERVO_SHOOTER_RELOAD_OFF_DUTY);  


    // TS_Servo_Duty_Set(Servo_Gripper_pitch1,TS_SERVO_GRIPPER_PITCH1_GRIP_DUTY);
    // TS_Servo_Duty_Set(Servo_Gripper_pitch2,TS_SERVO_GRIPPER_PITCH2_GRIP_DUTY);  

    SERVO_Gripper_yaw_Handle.Duty_now=TS_SERVO_GRIPPER_YAW_0_DUTY;
    SERVO_Gripper_pitch1_Handle.Duty_now=TS_SERVO_GRIPPER_PITCH1_HORIZONTAL_DUTY;
    SERVO_Gripper_pitch2_Handle.Duty_now=TS_SERVO_GRIPPER_PITCH2_HORIZONTAL_DUTY;
}

void TS_Servo_init(void){
    TS_Servo_timer_init();
    TS_TIM_Servo_PWMOUT_ENA();

    TS_Servo_Reset();

    // while(1)

    SERVO_Gripper_yaw_Handle.SERVO_SELECT=Servo_Gripper_yaw;
    SERVO_Gripper_yaw_Handle.pos_now=0.0f;
    SERVO_Gripper_yaw_Handle.input_min=-90.0f;
    SERVO_Gripper_yaw_Handle.input_max=90.0f;

    SERVO_Gripper_pitch1_Handle.SERVO_SELECT=Servo_Gripper_pitch1;
    SERVO_Gripper_pitch2_Handle.SERVO_SELECT=Servo_Gripper_pitch2;

    SERVO_Gripper_yaw_Handle.Duty_Done=1;
    SERVO_Gripper_pitch1_Handle.Duty_Done=1;
    SERVO_Gripper_pitch2_Handle.Duty_Done=1;


    TS_Change_to_Horizontal();

    // TS_Servo_Duty_Set(Servo_Gripper_yaw,3750);

    // TS_Servo_Duty_Set(Servo_Gripper_pitch1,TS_SERVO_GRIPPER_PITCH1_TEACHING1_DUTY);
    // TS_Servo_Duty_Set(Servo_Gripper_pitch2,TS_SERVO_GRIPPER_PITCH2_TEACHING1_DUTY);

}


void TS_Servo_Shooter_reload_ON(void){
    TS_Servo_Duty_Set(Servo_Shooter_reload,TS_SERVO_SHOOTER_RELOAD_ON_DUTY);
}

void TS_Servo_Shooter_reload_OFF(void){
    TS_Servo_Duty_Set(Servo_Shooter_reload,TS_SERVO_SHOOTER_RELOAD_OFF_DUTY);
}


void TS_Servo_check(void){
    // TS_SET_Servo_Gripper_yaw(0);
    // TS_delay_ms(1000);
    // TS_SET_Servo_Gripper_yaw(-90);
    // TS_delay_ms(1000);
    // TS_SET_Servo_Gripper_yaw(90);
    // TS_delay_ms(1500);
    // TS_SET_Servo_Gripper_yaw(0);
    // TS_delay_ms(1000);
    // TS_SET_Servo_Gripper_grip(0);
    // TS_delay_ms(1000);
    // TS_SET_Servo_Gripper_grip(10000);
    // TS_delay_ms(1000);
    // TS_SET_Servo_Gripper_grip(3000);
}



/************************************************应用层*************************************************/

/*************************************************驱动层*************************************************/
void TS_Servo_timer_init(void){
    GPIO_InitTypeDef GPIO_Initure={0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_9;  
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用模式
	GPIO_Initure.Pull=GPIO_NOPULL;              //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //高速
    GPIO_Initure.Alternate=GPIO_AF2_TIM12;
	HAL_GPIO_Init(GPIOH,&GPIO_Initure);
    GPIO_Initure.Pin=GPIO_PIN_5|GPIO_PIN_6;  
    GPIO_Initure.Alternate=GPIO_AF4_TIM15;
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);
    

    __HAL_RCC_TIM12_CLK_ENABLE(); 
    TIM12_Servo_Handle.Instance=TIM12;
    TIM12_Servo_Handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//200mhz
    TIM12_Servo_Handle.Init.Prescaler=80-1;//10mhz
    TIM12_Servo_Handle.Init.Period=TS_SERVO_PWM_PRELOAD-1;//1khz
    TIM12_Servo_Handle.Init.CounterMode=TIM_COUNTERMODE_UP;
    TIM12_Servo_Handle.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&TIM12_Servo_Handle);

    __HAL_RCC_TIM15_CLK_ENABLE(); 
    TIM15_Servo_Handle.Instance=TIM15;
    TIM15_Servo_Handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//200mhz
    TIM15_Servo_Handle.Init.Prescaler=80-1;//10mhz
    TIM15_Servo_Handle.Init.Period=TS_SERVO_PWM_PRELOAD-1;//1khz
    TIM15_Servo_Handle.Init.CounterMode=TIM_COUNTERMODE_UP;
    TIM15_Servo_Handle.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&TIM15_Servo_Handle);

    TIM_OC_InitTypeDef sConfigOC={0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;  // PWM 模式 1
    sConfigOC.Pulse = 0;  // 设置占空比
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;  // 输出极性
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;  // 禁用快速模式

    
    HAL_TIM_PWM_ConfigChannel(&TIM12_Servo_Handle, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&TIM12_Servo_Handle, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&TIM15_Servo_Handle, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&TIM15_Servo_Handle, &sConfigOC, TIM_CHANNEL_2);

}

void TS_TIM_Servo_PWMOUT_ENA(void){
    HAL_TIM_PWM_Start(&TIM12_Servo_Handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TIM12_Servo_Handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TIM15_Servo_Handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TIM15_Servo_Handle, TIM_CHANNEL_2);
}

void TS_TIM_Servo_PWMOUT_DISA(void){
    HAL_TIM_PWM_Stop(&TIM12_Servo_Handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&TIM12_Servo_Handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&TIM15_Servo_Handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&TIM15_Servo_Handle, TIM_CHANNEL_2);
}

// void TS_TIM_Servo_PWM_duty_set(uint32_t Servo_1,uint32_t Servo_2,uint32_t Servo_3,uint32_t Servo_4){
//     Servo_1=Servo_1>TS_SERVO_PWM_PRELOAD?TS_SERVO_PWM_PRELOAD:Servo_1;
//     Servo_2=Servo_2>TS_SERVO_PWM_PRELOAD?TS_SERVO_PWM_PRELOAD:Servo_2;
//     Servo_3=Servo_3>TS_SERVO_PWM_PRELOAD?TS_SERVO_PWM_PRELOAD:Servo_3;
//     Servo_4=Servo_4>TS_SERVO_PWM_PRELOAD?TS_SERVO_PWM_PRELOAD:Servo_4;
//     __HAL_TIM_SET_COMPARE(&TIM12_Servo_Handle, TIM_CHANNEL_1, Servo_1);
//     __HAL_TIM_SET_COMPARE(&TIM12_Servo_Handle, TIM_CHANNEL_2, Servo_2);
//     __HAL_TIM_SET_COMPARE(&TIM15_Servo_Handle, TIM_CHANNEL_1, Servo_3);
//     __HAL_TIM_SET_COMPARE(&TIM15_Servo_Handle, TIM_CHANNEL_2, Servo_4);
// }

void TS_Servo_Duty_Set_slow(Servo_enum Servo,uint32_t duty,uint32_t step){
    duty=duty>TS_SERVO_PWM_PRELOAD-1?TS_SERVO_PWM_PRELOAD-1:duty;
    if(step<=1){
        step=1;
    }
    switch(Servo){
        case Servo_Gripper_yaw:{
            SERVO_Gripper_yaw_Handle.Duty_aim=duty;
            SERVO_Gripper_yaw_Handle.Duty_Done=0;
            SERVO_Gripper_yaw_Handle.Duty_inc=duty>SERVO_Gripper_yaw_Handle.Duty_now?step:-step;
            // printf("now:%d,aim:%d,inc:%d\n",SERVO_Gripper_yaw_Handle.Duty_now,SERVO_Gripper_yaw_Handle.Duty_aim,SERVO_Gripper_yaw_Handle.Duty_inc);
            break;
        }
        case Servo_Gripper_pitch1:{
            SERVO_Gripper_pitch1_Handle.Duty_aim=duty;
            SERVO_Gripper_pitch1_Handle.Duty_Done=0;
            SERVO_Gripper_pitch1_Handle.Duty_inc=duty>SERVO_Gripper_pitch1_Handle.Duty_now?step:-step;
            break;
        }
        case Servo_Gripper_pitch2:{
            SERVO_Gripper_pitch2_Handle.Duty_aim=duty;
            SERVO_Gripper_pitch2_Handle.Duty_Done=0;
            SERVO_Gripper_pitch2_Handle.Duty_inc=duty>SERVO_Gripper_pitch2_Handle.Duty_now?step:-step;
            break;
        }
    }
}

//50ms轮询
void TS_Servo_Duty_Done_poll(void){
    if(SERVO_Gripper_yaw_Handle.Duty_Done==0){
        if((abs(SERVO_Gripper_yaw_Handle.Duty_aim-SERVO_Gripper_yaw_Handle.Duty_now))<=abs(SERVO_Gripper_yaw_Handle.Duty_inc)){
            SERVO_Gripper_yaw_Handle.Duty_now=SERVO_Gripper_yaw_Handle.Duty_aim;
            SERVO_Gripper_yaw_Handle.Duty_Done=1;
        }
        else{
            SERVO_Gripper_yaw_Handle.Duty_now+=SERVO_Gripper_yaw_Handle.Duty_inc;
        }
        TS_Servo_Duty_Set(Servo_Gripper_yaw,SERVO_Gripper_yaw_Handle.Duty_now);
        printf("duty now:%d\n",SERVO_Gripper_yaw_Handle.Duty_now);
    }

    if(SERVO_Gripper_pitch1_Handle.Duty_Done==0){
        if((abs(SERVO_Gripper_pitch1_Handle.Duty_aim-SERVO_Gripper_pitch1_Handle.Duty_now))<abs(SERVO_Gripper_pitch1_Handle.Duty_inc)){
            SERVO_Gripper_pitch1_Handle.Duty_now=SERVO_Gripper_pitch1_Handle.Duty_aim;
        }
        else{
            SERVO_Gripper_pitch1_Handle.Duty_now+=SERVO_Gripper_pitch1_Handle.Duty_inc;
        }
        TS_Servo_Duty_Set(Servo_Gripper_pitch1,SERVO_Gripper_pitch1_Handle.Duty_now);
    }

    if(SERVO_Gripper_pitch2_Handle.Duty_Done==0){
        if((abs(SERVO_Gripper_pitch2_Handle.Duty_aim-SERVO_Gripper_pitch2_Handle.Duty_now))<abs(SERVO_Gripper_pitch2_Handle.Duty_inc)){
            SERVO_Gripper_pitch2_Handle.Duty_now=SERVO_Gripper_pitch2_Handle.Duty_aim;
        }
        else{
            SERVO_Gripper_pitch2_Handle.Duty_now+=SERVO_Gripper_pitch2_Handle.Duty_inc;
        }
        TS_Servo_Duty_Set(Servo_Gripper_pitch2,SERVO_Gripper_pitch2_Handle.Duty_now);
    }
}

void TS_Servo_Duty_Set(Servo_enum Servo,uint32_t duty){
    duty=duty>TS_SERVO_PWM_PRELOAD-1?TS_SERVO_PWM_PRELOAD-1:duty;
    switch(Servo){
        case Servo_Gripper_yaw:{
            __HAL_TIM_SET_COMPARE(&TIM15_Servo_Handle, TIM_CHANNEL_2, duty);
            break;
        }
        case Servo_Gripper_pitch1:{
            __HAL_TIM_SET_COMPARE(&TIM15_Servo_Handle, TIM_CHANNEL_1, duty);
            break;
        }
        case Servo_Gripper_pitch2:{
            __HAL_TIM_SET_COMPARE(&TIM12_Servo_Handle, TIM_CHANNEL_2, duty);
            break;
        }
        case Servo_Shooter_reload:{
            __HAL_TIM_SET_COMPARE(&TIM12_Servo_Handle, TIM_CHANNEL_1, duty);
            break;
        }
    }
}
/*************************************************驱动层*************************************************/



/******************************************************总线舵机测试*******************************************************/
// UART_HandleTypeDef UART6_Handle={0};
// void TS_Bus_Servo_init(void){
//     GPIO_InitTypeDef GPIO_Initure={0};

//     __HAL_RCC_GPIOC_CLK_ENABLE();
//     GPIO_Initure.Pin=GPIO_PIN_6;
// 	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用模式
// 	GPIO_Initure.Pull=GPIO_NOPULL;              //上拉
// 	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //高速
//     GPIO_Initure.Alternate=GPIO_AF7_USART6;
// 	HAL_GPIO_Init(GPIOC,&GPIO_Initure);

//     __HAL_RCC_USART6_CLK_ENABLE();
// 	UART6_Handle.Instance=USART6;					    //USART1
// 	UART6_Handle.Init.BaudRate=115200;				    //波特率
// 	UART6_Handle.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
// 	UART6_Handle.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
// 	UART6_Handle.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
// 	UART6_Handle.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
// 	UART6_Handle.Init.Mode=UART_MODE_TX_RX;		    //收发模式
//     UART6_Handle.Init.FIFOMode=UART_FIFOMODE_DISABLE;

//     HAL_HalfDuplex_Init(&UART6_Handle);
//     // HAL_NVIC_SetPriority(UART5_IRQn, 2, 0);// 设置中断优先级
//     // HAL_NVIC_EnableIRQ(UART5_IRQn);// 启用 TIM2 中断 

//     // TS_FIFO_init(&TS_UART5_FIFO,TS_UART5_temp,TS_UART_TEMP_SIZE);//挂载发送缓冲区
//     // __HAL_UART_ENABLE_IT(&UART5_Handle,UART_IT_TC);
//     // __HAL_UART_ENABLE_IT(&UART5_Handle,UART_IT_RXNE);
//     // HAL_UART_Receive_IT(&UART5_Handle,&UART_5_RX_dat,1);

    
// }


// void TS_Bus_Servo_Test(void){
//     TS_Bus_Servo_init();

//     // for(int i=1;i<10;i++){
//     //     if(DS_Servo_Ping(&UART6_Handle,i)){
//     //         printf("bus servo %d online\n",i);
//     //     }
//     //     else{
//     //         printf("id %d no servo\n",i);
//     //     }        
//     // }
//     // DS_Servo_Ping(&UART6_Handle,3);
//     // DS_Servo_WritePos(&UART6_Handle,6,1000,1000);

//     // ARM_Joint_ang_struct Joint_ang={0};
//     // Joint_ang.Joint_yaw_ang=0.0f;
//     // Joint_ang.Joint_pitch_1_ang=0.0f;
//     // Joint_ang.Joint_pitch_2_ang=0.0f;
//     // Joint_ang.Joint_pitch_3_ang=0.0f;
//     // Joint_ang.Joint_roll_ang=0.0f;
//     // Joint_ang.Joint_grip_ang=0.0f;
//     // Joint_ang.valid_solve=1;
//     // TS_6DOF_ARM_set_angle(Joint_ang);

// }

// void TS_BusServo_PreWrite(uint8_t id,uint16_t pos,uint16_t time){
//     // printf("write:no.%d pos:%d\n",id,pos);
//     DS_Servo_RegWritePos(&UART6_Handle,id,pos,time);
// }

// void TS_BusServo_Write(uint8_t id,uint16_t pos,uint16_t time){
//     printf("write:no.%d pos:%d\n",id,pos);
//     DS_Servo_WritePos(&UART6_Handle,id,pos,time);
// }

// void TS_BusServo_move(void){
//     DS_Servo_Action(&UART6_Handle);
// }


// void TS_BusServo_Reset(uint8_t id){
//     DS_Servo_Reset(&UART6_Handle,id);
// }

// void TS_BusServo_SetID(uint8_t id,uint8_t new_id){
//     DS_Servo_SetID(&UART6_Handle,id,new_id);
// }
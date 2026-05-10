#include "stm32h7xx_hal.h"

/*
    TIM2、5为32位定时器，3、4为16位定时器
    为防止行为不一致，确保16位定时器不溢出
*/

TIM_HandleTypeDef TIM_ENC_Handle[4]={0};
int32_t ENC_data[4]={0};

void CN_TIM2345_ENC_init(void){
    GPIO_InitTypeDef GPIO_Initure={0};

/***************************************GPIO初始化部分***************************************/
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    // __HAL_RCC_GPIOG_CLK_ENABLE();

    /****************公共部分***************/
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用模式
	GPIO_Initure.Pull=GPIO_NOPULL;              //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //高速

    /****************TIM2***************/
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1;     //PA0,1
    GPIO_Initure.Alternate=GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);

    /****************TIM3***************/
    GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_7;     //PA6,7
    GPIO_Initure.Alternate=GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);

    /****************TIM4***************/
    GPIO_Initure.Pin=GPIO_PIN_12|GPIO_PIN_13;     //PD12,13
    GPIO_Initure.Alternate=GPIO_AF2_TIM4;
	HAL_GPIO_Init(GPIOD,&GPIO_Initure);

    /****************TIM5***************/
    GPIO_Initure.Pin=GPIO_PIN_10|GPIO_PIN_11;     //PH10,11
    GPIO_Initure.Alternate=GPIO_AF2_TIM5;
	HAL_GPIO_Init(GPIOH,&GPIO_Initure);

    // /****************LPTIM1***************/
    // GPIO_Initure.Pin=GPIO_PIN_12;     //PG12,PH2
    // GPIO_Initure.Alternate=GPIO_AF1_LPTIM1;
	// HAL_GPIO_Init(GPIOG,&GPIO_Initure);
    // GPIO_Initure.Pin=GPIO_PIN_2; 
    // HAL_GPIO_Init(GPIOH,&GPIO_Initure);

/***************************************TIM初始化部分***************************************/
    __HAL_RCC_TIM2_CLK_ENABLE(); 
    __HAL_RCC_TIM3_CLK_ENABLE(); 
    __HAL_RCC_TIM4_CLK_ENABLE(); 
    __HAL_RCC_TIM5_CLK_ENABLE(); 

    /****************公共部分***************/
    for(int i=0;i<4;i++){
        TIM_ENC_Handle[i].Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
        TIM_ENC_Handle[i].Init.Prescaler=1-1;
        TIM_ENC_Handle[i].Init.CounterMode=TIM_COUNTERMODE_UP;
        TIM_ENC_Handle[i].Init.Period=0xFFFF;
        TIM_ENC_Handle[i].Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
    }
    TIM_ENC_Handle[0].Instance=TIM4;
    TIM_ENC_Handle[1].Instance=TIM5;
    TIM_ENC_Handle[2].Instance=TIM3;
    TIM_ENC_Handle[3].Instance=TIM2;
    //  TIM_ENC_Handle[0].Instance=TIM2;
    // TIM_ENC_Handle[1].Instance=TIM3;
    // TIM_ENC_Handle[2].Instance=TIM4;
    // TIM_ENC_Handle[3].Instance=TIM5;


    /****************编码器初始化部分***************/
	TIM_Encoder_InitTypeDef TIM_ENC_init;
	TIM_ENC_init.EncoderMode = TIM_ENCODERMODE_TI12;  // 设置为编码器模式
	TIM_ENC_init.IC1Polarity = TIM_ICPOLARITY_RISING; // 输入1极性，通常选择上升沿或下降沿
	TIM_ENC_init.IC1Selection = TIM_ICSELECTION_DIRECTTI; // 输入选择：直接TI（编码器模式）
	TIM_ENC_init.IC2Polarity = TIM_ICPOLARITY_RISING; // 输入2极性
	TIM_ENC_init.IC2Selection = TIM_ICSELECTION_DIRECTTI; // 输入选择：直接TI

    for(int i=0;i<4;i++){
        HAL_TIM_Encoder_Init(&TIM_ENC_Handle[i], &TIM_ENC_init);
        HAL_TIM_Encoder_Start(&TIM_ENC_Handle[i],TIM_CHANNEL_ALL);
        __HAL_TIM_SET_COUNTER(&TIM_ENC_Handle[i], 0xFFFF/2);
    }
}

void CN_read_ENC_INC(void){
    //printf("%d\n",(uint32_t)__HAL_TIM_GET_COUNTER(&TIM_ENC_Handle[1]));
    for(int i=0;i<4;i++){
       ENC_data[i]=(uint16_t)__HAL_TIM_GET_COUNTER(&TIM_ENC_Handle[i])-0xFFFF/2;
       __HAL_TIM_SET_COUNTER(&TIM_ENC_Handle[i],0xFFFF/2);
    }

    ENC_data[0]=ENC_data[0];
    ENC_data[1]=ENC_data[1];
    ENC_data[2]=ENC_data[2];
    ENC_data[3]=-ENC_data[3];

    // // printf("%d\n",ENC_data[2]);
    // printf("%d,%d,%d,%d\n",ENC_data[0],ENC_data[3],ENC_data[1],ENC_data[2]);
}

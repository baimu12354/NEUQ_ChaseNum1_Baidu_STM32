#include "ChaseNum1_Include.h"

volatile uint32_t us_delay_counter=0;
volatile uint32_t systime_us=0;

TIM_HandleTypeDef TIM6_PIT_Handle={0};
TIM_HandleTypeDef TIM7_PIT_Handle={0};
TIM_HandleTypeDef TIM13_PIT_Handle={0};

void CN_PIT1_CALLBACK(void);
void CN_PIT2_CALLBACK(void);
// void CN_PIT3_CALLBACK(void);

void CN_PIT_1_init_ms(uint16_t period,uint8_t priority){//初始化后自动启动定时器和中断
    //timer总线时钟速率200mhz
    __HAL_RCC_TIM6_CLK_ENABLE();

    TIM6_PIT_Handle.Instance=TIM6;
    TIM6_PIT_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;//分频后模块时钟为200mhz
    TIM6_PIT_Handle.Init.Prescaler = 20000-1;  // 预分频器，定时器频率 10kHz
    TIM6_PIT_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;  // 向上计数模式
    TIM6_PIT_Handle.Init.Period = period*10-1;  // 定时器周期
    TIM6_PIT_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;//自动复位

    HAL_TIM_Base_Init(&TIM6_PIT_Handle);
    HAL_TIM_Base_Start_IT(&TIM6_PIT_Handle);// 启用定时器更新中断  
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, priority, 0);// 设置中断优先级
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);// 启用 TIM2 中断 
}

void CN_PIT_2_init_ms(uint16_t period,uint8_t priority){//初始化后自动启动定时器和中断
    //timer总线时钟速率200mhz
    __HAL_RCC_TIM7_CLK_ENABLE();

    TIM7_PIT_Handle.Instance=TIM7;
    TIM7_PIT_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;//分频后模块时钟为200mhz
    TIM7_PIT_Handle.Init.Prescaler = 20000-1;  // 预分频器，定时器频率 10kHz
    TIM7_PIT_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;  // 向上计数模式
    TIM7_PIT_Handle.Init.Period = period*10-1;  // 定时器周期
    TIM7_PIT_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;//自动复位

    HAL_TIM_Base_Init(&TIM7_PIT_Handle);
    HAL_TIM_Base_Start_IT(&TIM7_PIT_Handle);// 启用定时器更新中断  
    HAL_NVIC_SetPriority(TIM7_IRQn, priority, 0);// 设置中断优先级 
    HAL_NVIC_EnableIRQ(TIM7_IRQn);// 启用 TIM2 中断 
}

void CN_PIT_3_init_us(uint16_t period,uint8_t priority){//初始化后自动启动定时器和中断
    //timer总线时钟速率200mhz
    __HAL_RCC_TIM13_CLK_ENABLE();

    TIM13_PIT_Handle.Instance=TIM13;
    TIM13_PIT_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;//分频后模块时钟为200mhz
    TIM13_PIT_Handle.Init.Prescaler = 20-1;  // 预分频器，定时器频率 10mHz
    TIM13_PIT_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;  // 向上计数模式
    TIM13_PIT_Handle.Init.Period = period*10-1;  // 定时器周期
    TIM13_PIT_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;//自动复位

    HAL_TIM_Base_Init(&TIM13_PIT_Handle);
    // HAL_TIM_Base_Start(&TIM13_PIT_Handle);
    HAL_TIM_Base_Start_IT(&TIM13_PIT_Handle);// 启用定时器更新中断  
    HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, priority, 0);// 设置中断优先级
    HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);// 启用 TIM2 中断 
}

void TIM6_DAC_IRQHandler(void){HAL_TIM_IRQHandler(&TIM6_PIT_Handle);}
void TIM7_IRQHandler(void){HAL_TIM_IRQHandler(&TIM7_PIT_Handle);}
void TIM8_UP_TIM13_IRQHandler(void){HAL_TIM_IRQHandler(&TIM13_PIT_Handle);}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        CN_PIT1_CALLBACK();
    }
    if (htim->Instance == TIM7)
    {
        CN_PIT2_CALLBACK();
    }
    if (htim->Instance == TIM13)
    {
        us_delay_counter++;
        systime_us+=10;
        // printf("intr\n");
    }
    if (htim->Instance == TIM15)
    {
        CN_Pump_TIM15_PeriodElapsedCallback();
    }
}

void CN_delay_us(uint32_t nus){
    uint32_t counter_start=us_delay_counter;
    while((uint32_t)(us_delay_counter-counter_start)<nus/10){
        // printf("%d\n",us_delay_counter);
    }
    us_delay_counter=0;
}

void CN_delay_ms(uint16_t nms){
    CN_delay_us(1000*nms);
}

#include "TopSpeed_Include.h"

void TS_Trig_init(void){
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);// 设置中断优先级
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);// 

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);// 设置中断优先级
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);// 
}

void EXTI4_IRQHandler(void){

}

void EXTI9_5_IRQHandler(void){
    
}

#include "TopSpeed_Include.h"

ADC_HandleTypeDef Pump_ADC_Handle={0};
Pump_Handle_Struct Pump_Handle={0};

void TS_Pump_init(void){
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2;          // PA0 = ADC1_INP0
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET);

    TS_Pump_ADC_init();
}

void TS_Pump_Start(void){
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET);
    Pump_Handle.Pump_on=1;
}

void TS_Pump_Stop(void){
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET);
    Pump_Handle.Pump_on=0;
}

uint8_t TS_get_Suck(void){
    return Pump_Handle.IfSucked;
}

void TS_Pump_ADC_init(void){
    __HAL_RCC_ADC3_CLK_ENABLE();      // 启用 ADC12（H7 系列 ADC1/2 共用时钟）
    __HAL_RCC_GPIOH_CLK_ENABLE();      // 启用 GPIOA 时钟

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_4;          // PA0 = ADC1_INP0
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    
    Pump_ADC_Handle.Instance = ADC3;
    Pump_ADC_Handle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV4; // 异步时钟分频
    Pump_ADC_Handle.Init.Resolution            = ADC_RESOLUTION_12B;
    Pump_ADC_Handle.Init.ScanConvMode          = DISABLE;
    Pump_ADC_Handle.Init.ContinuousConvMode    = DISABLE;
    Pump_ADC_Handle.Init.DiscontinuousConvMode = DISABLE;
    Pump_ADC_Handle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    Pump_ADC_Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    Pump_ADC_Handle.Init.NbrOfConversion       = 1;
    Pump_ADC_Handle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;

    HAL_ADC_Init(&Pump_ADC_Handle);

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = ADC_CHANNEL_15;       // ADC1_INP0 对应 PA0
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;

    HAL_ADC_ConfigChannel(&Pump_ADC_Handle, &sConfig);

    HAL_ADCEx_Calibration_Start(&Pump_ADC_Handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    HAL_NVIC_SetPriority(ADC3_IRQn, 5, 0);  // 设置优先级：主优先级5，子优先级0
    HAL_NVIC_EnableIRQ(ADC3_IRQn);         // 启用 ADC 全局中断

    // HAL_ADC_Start_IT(&Pump_ADC_Handle);
}

void Pump_ADC_Read(void){
    HAL_ADC_Start_IT(&Pump_ADC_Handle);
}

uint16_t Read_Pump_ADC(void){
    uint16_t value = HAL_ADC_GetValue(&Pump_ADC_Handle);      // 读取值
    return value;
}

#define BUFFER_SIZE 50

uint16_t adcBuffer[BUFFER_SIZE]={0};
uint8_t adcIndex = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
    if (hadc->Instance == ADC3){
        static uint16_t temp[128]={0};
        uint16_t adc_val = Read_Pump_ADC();
        static int suck_counter=0;
        // 你的处理逻辑，比如滤波、采集保存等
        // printf("%d\n",adc_val);
    // 存入环形缓冲区
        adcBuffer[adcIndex++] = adc_val;
        if (adcIndex >= BUFFER_SIZE) {
            adcIndex = 0;
        }

        // 找最大值
        uint16_t maxVal = 0;

        for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
            if (adcBuffer[i] > maxVal) {
                maxVal = adcBuffer[i];
            }
        }
        // printf("%d\n", maxVal);
        
        Pump_Handle.SuckStrength=maxVal;
        if(maxVal>=3800){
            suck_counter++;
        }
        else{
            suck_counter=0;
        }

        if(suck_counter>100){
            suck_counter=101;
            Pump_Handle.IfSucked=1;
        }
        else{
            Pump_Handle.IfSucked=0;
        }
    }
}

void ADC3_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&Pump_ADC_Handle);  // 通知 HAL 处理 ADC 中断
}
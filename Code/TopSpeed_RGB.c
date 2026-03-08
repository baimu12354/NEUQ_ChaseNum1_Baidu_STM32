#include "TopSpeed_Include.h"

/*
    驱动ws2812的RGB灯带，使用TIM17触发dma，向gpio的BSRR寄存器写入数据

    TIM17的时钟频率为200MHz,需要2.4mhz的位元率
    需要设置为200MHz/2.4MHz=83的预分频

    TIM17以2.4mhz也就是约1.24us的速度触发dma，每触发一次就根据先前设置的ws_buf向GPIOD->BSRR写入数据，也就是从gpio输出信号
    dma传输结束后，使用tim17产生一个100us的一次性中断，用于翻回允许下一次写入的Flag_WS_Reset，这段时间就是ws2812的reset保持时间，然后就可以进行下一次写入了

*/

TIM_HandleTypeDef TIM17_RGB_Handle={0};
DMA_HandleTypeDef RGB_DMA_Handle={0};

uint8_t Flag_WS_Reset=1;

#define LED_NUM      7
#define BIT_PER_LED  24
#define SYMBOLS_PER_BIT  3
#define WS_BUF_SIZE (LED_NUM * BIT_PER_LED * SYMBOLS_PER_BIT)

#define GPIO_BSRR_SET(pin)     (1U << (pin))
#define GPIO_BSRR_RESET(pin)   (1U << ((pin) + 16))

uint32_t ws_buf[WS_BUF_SIZE];

RGB_ColorTypeDef Color_Now=RGB_COLOR_NONE;

// uint8_t rgb_data[7][3] = {
//     {255,   0,   0}, // 红
//     {  0, 255,   0}, // 绿
//     {  0,   0, 255}, // 蓝
//     {255, 255,   0}, // 黄
//     {255,   0, 255}, // 品红
//     {  0, 255, 255}, // 青色
//     {255, 255, 255}  // 白色
// };

// uint8_t rgb_data[7][3] = {
//     {255,   0,   0}, // 红
//     {255,   0,   0}, // 红
//     {255,   0,   0}, // 红
//     {255,   0,   0}, // 红
//     {255,   0,   0}, // 红
//     {255,   0,   0}, // 红
//     {255,   0,   0}, // 红
// };

void WS2812_EncodeAll(uint8_t rgb_array[][3], uint16_t led_num)
{
    int i = 0;

    for (uint16_t led = 0; led < led_num; led++) {
        // 注意：WS2812 是 GRB 顺序
        uint8_t g = rgb_array[led][1];
        uint8_t r = rgb_array[led][0];
        uint8_t b = rgb_array[led][2];

        uint8_t grb[3] = {g, r, b};

        for (int byte = 0; byte < 3; byte++) {
            for (int bit = 7; bit >= 0; bit--) {
                if (grb[byte] & (1 << bit)) {
                    ws_buf[i++] = GPIO_BSRR_SET(12); // High
                    ws_buf[i++] = GPIO_BSRR_SET(12); // High
                    ws_buf[i++] = GPIO_BSRR_RESET(12); // Low
                } else {
                    ws_buf[i++] = GPIO_BSRR_SET(12); // High
                    ws_buf[i++] = GPIO_BSRR_RESET(12); // Low
                    ws_buf[i++] = GPIO_BSRR_RESET(12); // Low
                }
            }
        }
    }
}

void WS2812_Send(void){
    if(Flag_WS_Reset){
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_TIM_Base_Stop(&TIM17_RGB_Handle); // 保证定时器停下

        // 启动 DMA，从 ws_buf 搬运数据到 GPIOC->BSRR
        HAL_DMA_Start_IT(&RGB_DMA_Handle, (uint32_t)ws_buf, (uint32_t)((__IO uint32_t *)(&GPIOC->BSRRL)), WS_BUF_SIZE);

        // 允许定时器更新事件触发 DMA
        __HAL_TIM_ENABLE_DMA(&TIM17_RGB_Handle, TIM_DMA_UPDATE);

        // 启动 TIM17
        HAL_TIM_Base_Start(&TIM17_RGB_Handle);      
        Flag_WS_Reset=0;
    }
}

void WS2812_Show(uint8_t rgb_array[][3], uint16_t led_num){
    if (Flag_WS_Reset) {
        WS2812_EncodeAll(rgb_array, led_num);  // 编码颜色到 ws_buf[]
        WS2812_Send();                         // 启动 DMA 发送
    }
}

void TS_RGB_show(RGB_ColorTypeDef color){
    // WS2812_Show(rgb_data,7);
    uint8_t rgb_data[8][3];
    for(int i=0;i<7;i++){
        switch(color){
            case RGB_COLOR_NONE:
                rgb_data[i][0] = 0; // R
                rgb_data[i][1] = 0; // G
                rgb_data[i][2] = 0; // B
                break;
            case RGB_COLOR_White:
                rgb_data[i][0] = 255; // R
                rgb_data[i][1] = 255; // G
                rgb_data[i][2] = 255; // B
                break;
            case RGB_COLOR_Blue:
                rgb_data[i][0] = 0; // R
                rgb_data[i][1] = 0; // G
                rgb_data[i][2] = 255; // B
                break;
            case RGB_COLOR_Green:
                rgb_data[i][0] = 0; // R
                rgb_data[i][1] = 255; // G
                rgb_data[i][2] = 0; // B
                break;
            case RGB_COLOR_Yellow:
                rgb_data[i][0] = 255; // R
                rgb_data[i][1] = 150; // G
                rgb_data[i][2] = 0; // B
                break;
            case RGB_COLOR_Orange:
                rgb_data[i][0] = 255; // R
                rgb_data[i][1] = 50; // G
                rgb_data[i][2] = 0; // B
                break;
            default:
                rgb_data[i][0] = 0; // R
                rgb_data[i][1] = 0; // G
                rgb_data[i][2] = 0; // B
                break;
        }
    }
    WS2812_Show(rgb_data,7);
}

void Start_Delay_Event(uint16_t delay_us){
    __HAL_TIM_SET_COUNTER(&TIM17_RGB_Handle,0);
    uint32_t pulse = delay_us * (200); // 假设时钟单位是 MHz

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_TIMING;
    sConfigOC.Pulse = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_OC_ConfigChannel(&TIM17_RGB_Handle, &sConfigOC, TIM_CHANNEL_1);
    __HAL_TIM_DISABLE_DMA(&TIM17_RGB_Handle, TIM_DMA_UPDATE);
    __HAL_TIM_CLEAR_IT(&TIM17_RGB_Handle, TIM_IT_CC1);
    __HAL_TIM_ENABLE_IT(&TIM17_RGB_Handle, TIM_IT_CC1);
    HAL_TIM_OC_Start_IT(&TIM17_RGB_Handle, TIM_CHANNEL_1);
}

void DMA1_Stream0_IRQHandler(void) {
    HAL_DMA_IRQHandler(&RGB_DMA_Handle);
}

void RGB_DMA_Done_callback(DMA_HandleTypeDef *hdma){
    // printf("dma done\n");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
    Start_Delay_Event(100);
}

void TIM17_IRQHandler(void){
    // printf("reset done\n");
    HAL_TIM_IRQHandler(&TIM17_RGB_Handle);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM17 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
        // printf("reset done\n");
        Flag_WS_Reset=1;
        __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
        HAL_TIM_OC_Stop_IT(htim, TIM_CHANNEL_1);
        // WS2812_Send(); // 如果在reset时间内有新的数据要发送，就直接发送
    }
}


void TS_RGB_init(void){
   /*初始化时基时钟*/
    __HAL_RCC_TIM17_CLK_ENABLE();

    TIM17_RGB_Handle.Instance = TIM17;
    TIM17_RGB_Handle.Init.Prescaler = 0;                 // 不分频
    TIM17_RGB_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM17_RGB_Handle.Init.Period = 82;                   // 200MHz / (83+1) = 2.4MHz，416ns
    TIM17_RGB_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM17_RGB_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&TIM17_RGB_Handle);

    HAL_TIM_Base_Start(&TIM17_RGB_Handle); // 手动开启定时器

    HAL_NVIC_SetPriority(TIM17_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM17_IRQn);


    /*初始化DMA*/
    __HAL_RCC_DMA1_CLK_ENABLE();

    RGB_DMA_Handle.Instance = DMA1_Stream0;
    RGB_DMA_Handle.Init.Request = DMA_REQUEST_TIM17_UP;
    RGB_DMA_Handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    RGB_DMA_Handle.Init.PeriphInc = DMA_PINC_DISABLE;
    RGB_DMA_Handle.Init.MemInc = DMA_MINC_ENABLE;
    RGB_DMA_Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    RGB_DMA_Handle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    RGB_DMA_Handle.Init.Mode = DMA_NORMAL;
    RGB_DMA_Handle.Init.Priority = DMA_PRIORITY_HIGH;
    RGB_DMA_Handle.XferCpltCallback = RGB_DMA_Done_callback;

    HAL_DMA_Init(&RGB_DMA_Handle);

    __HAL_LINKDMA(&TIM17_RGB_Handle, hdma[TIM_DMA_ID_UPDATE], RGB_DMA_Handle);

    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

    /*初始化输出IO*/
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
    TS_delay_ms(5);
    Flag_WS_Reset=1;
}



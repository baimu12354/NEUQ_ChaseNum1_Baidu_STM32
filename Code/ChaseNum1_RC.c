#include "ChaseNum1_Include.h"

//--------------------------------RC遥控器 定时器配置---------------------------------//
TIM_HandleTypeDef TIM17_RC_Handle={0};

void TIM17_RC_Init(void)
{
    __HAL_RCC_TIM17_CLK_ENABLE();

    TIM17_RC_Handle.Instance = TIM17;
    // APB2 定时器时钟为 200MHz，分频到 1MHz (1us/tick)
    TIM17_RC_Handle.Init.Prescaler = 200 - 1; 
    TIM17_RC_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM17_RC_Handle.Init.Period = 0xFFFF; // 16位最大值
    TIM17_RC_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM17_RC_Handle.Init.RepetitionCounter = 0;
    TIM17_RC_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_Base_Init(&TIM17_RC_Handle);
    HAL_TIM_Base_Start(&TIM17_RC_Handle);
}

uint32_t TIM17_RC_Get(void){
   return __HAL_TIM_GET_COUNTER(&TIM17_RC_Handle);
}

void TIM17_RC_Clear(void){
  __HAL_TIM_SET_COUNTER(&TIM17_RC_Handle, 0);
}
//--------------------------------RC遥控器 定时器配置---------------------------------//

//--------------------------------RC遥控器 sbus协议----------------------------------//
uart_receiver_struct    uart_receiver ;             // 串口接收机通道数据与状态

uint8_t  uart_receiver_data[REV_DATA_LEN]  = {0};    // 接收器原始数据

// 串口接收机初始化
void uart_receiver_init(void)
{
    CN_UART3_init();   // 串口初始化
    TIM17_RC_Init();   // 定时器初始化（1Mhz）
}

// 获取两次调用uart_receiver_interval_time函数时间间隔  (us)
static uint32_t uart_receiver_interval_time (void)
{
    uint32_t interval_time;

    interval_time = TIM17_RC_Get();

    TIM17_RC_Clear();
    
    return interval_time;
}

// SBUS数据解析
static void uart_receiver_analysis (uart_receiver_struct *remote_data,uint8_t * buffer)
{
    uint8_t num = 0;
    remote_data->channel[num++] = (buffer[1] |buffer[ 2] << 8 ) & 0x07FF;
    remote_data->channel[num++] = (buffer[2] >> 3 | buffer[3] << 5 ) & 0x07FF;
    remote_data->channel[num++] = (buffer[3] >> 6 | buffer[4] << 2 | buffer[5] << 10 ) & 0x07FF;
    remote_data->channel[num++] = (buffer[5] >> 1 | buffer[6] << 7 ) & 0x07FF;
    remote_data->channel[num++] = (buffer[6] >> 4 | buffer[7] << 4 ) & 0x07FF;
    remote_data->channel[num++] = (buffer[7] >> 7 | buffer[8] << 1 | buffer[9] << 9 ) & 0x07FF;
    remote_data->state = (SBUS_ABNORMAL_STATE == (buffer[23] & SBUS_ABNORMAL_STATE)) ? 0 : 1;
   // printf("%d\n",remote_data->state);
    uart_receiver.finsh_flag  = 1;
}

// --- 在 ChaseNum1_RC.c 顶部定义全局抓包变量 ---
uint8_t debug_sbus_buffer[100]; // 抓取 100 个字节
uint8_t debug_count = 0;
uint8_t stop_record = 0;

// 串口接收机模块 串口中断回调函数
void uart_receiver_callback(uint8_t dat){

   static uint8_t length = 0;

    uint32_t in_time = uart_receiver_interval_time();
    // 超时断帧逻辑
    if (in_time > 3000)
    {
        length = 0;
    }
    
    // 防越界保护（如果因为噪点导致长度超限，强制丢弃并重新寻找帧头）
    if (length >= REV_DATA_LEN) 
    {
        length = 0;
    }
    
    uart_receiver_data[length] = dat;
    length++;

    // 帧校验与解析
    if((REV_DATA_LEN == length) && (FRAME_STAR == uart_receiver_data[0]) && (FRAME_END == uart_receiver_data[24]))
    {
        uart_receiver_analysis(&uart_receiver, uart_receiver_data);
        length = 0; // 解析成功后必须清零，准备接收下一帧
    }
}

// // 串口3接收回调
// void CN_UART3_RX_CALLBACK(uint8_t dat){
//     uart_receiver_callback(dat);
// }
//--------------------------------RC遥控器 sbus协议----------------------------------//
#include "ChaseNum1_Include.h"

/*
    串口通信模块，提供串口初始化、串口发送、串口接收等功能
    采用 中断+FIFO 的方式实现串口通信
    1.用户只需要将数据写入FIFO缓冲区，串口中断会自动从FIFO缓冲区读取数据并发送出去
    2.用户只需要在串口接收回调函数中处理接收到的数据

    需要注意的是：
    uart1 debug调试
    uart2 陀螺仪通信
    uart3 上位机通信
    uart7 舵机通信
*/

#define CN_UART_TEMP_SIZE   (256)

UART_HandleTypeDef UART1_Handle={0};
UART_HandleTypeDef UART2_Handle={0};
UART_HandleTypeDef UART3_Handle={0};
UART_HandleTypeDef UART4_Handle={0};
UART_HandleTypeDef UART5_Handle={0};
UART_HandleTypeDef UART6_Handle={0};
UART_HandleTypeDef UART7_Handle={0};
UART_HandleTypeDef UART8_Handle={0};

CN_FIFO_struct CN_UART1_FIFO={0};
uint8_t CN_UART1_temp[CN_UART_TEMP_SIZE]={0};
CN_FIFO_struct CN_UART2_FIFO={0};
uint8_t CN_UART2_temp[CN_UART_TEMP_SIZE]={0};
CN_FIFO_struct CN_UART3_FIFO={0};
uint8_t CN_UART3_temp[CN_UART_TEMP_SIZE]={0};
CN_FIFO_struct CN_UART4_FIFO={0};
uint8_t CN_UART4_temp[CN_UART_TEMP_SIZE]={0};
CN_FIFO_struct CN_UART5_FIFO={0};
uint8_t CN_UART5_temp[CN_UART_TEMP_SIZE]={0};
CN_FIFO_struct CN_UART6_FIFO={0};
uint8_t CN_UART6_temp[CN_UART_TEMP_SIZE]={0};
CN_FIFO_struct CN_UART7_FIFO={0};
uint8_t CN_UART7_temp[CN_UART_TEMP_SIZE]={0};
CN_FIFO_struct CN_UART8_FIFO={0};
uint8_t CN_UART8_temp[CN_UART_TEMP_SIZE]={0};

uint8_t UART_1_RX_dat=0;
uint8_t UART_2_RX_dat=0;
uint8_t UART_3_RX_dat=0;
uint8_t UART_4_RX_dat=0;
uint8_t UART_5_RX_dat=0;
uint8_t UART_6_RX_dat=0;
uint8_t UART_7_RX_dat=0;
uint8_t UART_8_RX_dat=0;

void CN_UART1_init(void){
    GPIO_InitTypeDef GPIO_Initure={0};

    //使用车载接口
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_14|GPIO_PIN_15;
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用模式
	GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //高速
    GPIO_Initure.Alternate=GPIO_AF4_USART1;
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);

    // //使用下载器串口
    // __HAL_RCC_GPIOA_CLK_ENABLE();
    // GPIO_Initure.Pin=GPIO_PIN_9|GPIO_PIN_10;
	// GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用模式
	// GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
	// GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //高速
    // GPIO_Initure.Alternate=GPIO_AF7_USART1;
	// HAL_GPIO_Init(GPIOA,&GPIO_Initure);

   __HAL_RCC_USART1_CLK_ENABLE();
	UART1_Handle.Instance=USART1;					    //USART1
	UART1_Handle.Init.BaudRate=115200;				    //波特率
	UART1_Handle.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART1_Handle.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART1_Handle.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART1_Handle.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART1_Handle.Init.Mode=UART_MODE_TX_RX;		    //收发模式
    UART1_Handle.Init.FIFOMode=UART_FIFOMODE_ENABLE;

    HAL_UART_Init(&UART1_Handle);
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);// 设置中断优先级
    HAL_NVIC_EnableIRQ(USART1_IRQn);// 启用 TIM2 中断 

    __HAL_UART_ENABLE_IT(&UART1_Handle,UART_IT_TC);
    __HAL_UART_ENABLE_IT(&UART1_Handle,UART_IT_RXNE);

    CN_FIFO_init(&CN_UART1_FIFO,CN_UART1_temp,CN_UART_TEMP_SIZE);//挂载发送缓冲区

    HAL_UART_Receive_IT(&UART1_Handle,&UART_1_RX_dat,1);
}

void CN_UART2_init(uint32_t baud){
    GPIO_InitTypeDef GPIO_Initure={0};

    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_5|GPIO_PIN_6;
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用模式
	GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //高速
    GPIO_Initure.Alternate=GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOD,&GPIO_Initure);

    __HAL_RCC_USART2_CLK_ENABLE();
	UART2_Handle.Instance=USART2;					    //USART1
	UART2_Handle.Init.BaudRate=baud;				    //波特率
	UART2_Handle.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART2_Handle.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART2_Handle.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART2_Handle.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART2_Handle.Init.Mode=UART_MODE_TX_RX;		    //收发模式
    UART2_Handle.Init.FIFOMode=UART_FIFOMODE_ENABLE;

    HAL_UART_Init(&UART2_Handle);
    HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);// 设置中断优先级
    HAL_NVIC_EnableIRQ(USART2_IRQn);// 启用 TIM2 中断 

    // __HAL_UART_ENABLE_IT(&UART2_Handle,UART_IT_TC);
    __HAL_UART_ENABLE_IT(&UART2_Handle,UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&UART2_Handle,UART_IT_TC);
    __HAL_UART_CLEAR_IT(&UART2_Handle,UART_IT_TC);

    HAL_UART_Receive_IT(&UART2_Handle,&UART_2_RX_dat,1);
}

void CN_UART3_init(void){
    GPIO_InitTypeDef GPIO_Initure={0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_10|GPIO_PIN_11;
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用模式
	GPIO_Initure.Pull=GPIO_NOPULL;              //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //高速
    GPIO_Initure.Alternate=GPIO_AF7_USART3;
    GPIO_Initure.Pull=GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);
    
    __HAL_RCC_USART3_CLK_ENABLE();
	UART3_Handle.Instance=USART3;

    // 普通 UART 协议：上位机通信使用 115200/8N1。
	UART3_Handle.Init.BaudRate=115200;
	UART3_Handle.Init.WordLength=UART_WORDLENGTH_8B;
	UART3_Handle.Init.StopBits=UART_STOPBITS_1;
	UART3_Handle.Init.Parity=UART_PARITY_NONE;
	UART3_Handle.Init.HwFlowCtl=UART_HWCONTROL_NONE;
	UART3_Handle.Init.Mode=UART_MODE_TX_RX;
    UART3_Handle.Init.FIFOMode=UART_FIFOMODE_DISABLE;
    UART3_Handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
    UART3_Handle.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_DISABLE;

    /*
     * SBUS 协议备用配置：
     * 如果下次要把 UART3 切回 SBUS，注释掉上面的普通 UART 配置，
     * 再打开下面这一段。SBUS 常用 100000/9E2，并且接收端可能需要 RX 反相。
     */
    // UART3_Handle.Init.BaudRate=SBUS_UART_BAUDRATE; // 100000
    // UART3_Handle.Init.WordLength=UART_WORDLENGTH_9B;
    // UART3_Handle.Init.StopBits=UART_STOPBITS_2;
    // UART3_Handle.Init.Parity=UART_PARITY_EVEN;
    // UART3_Handle.Init.HwFlowCtl=UART_HWCONTROL_NONE;
    // UART3_Handle.Init.Mode=UART_MODE_TX_RX;
    // UART3_Handle.Init.FIFOMode=UART_FIFOMODE_DISABLE;
    // UART3_Handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
    // UART3_Handle.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;

    __HAL_UART_CLEAR_IT(&UART3_Handle,UART_IT_TC);
    HAL_UART_Init(&UART3_Handle);
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);// 设置中断优先级
    HAL_NVIC_EnableIRQ(USART3_IRQn);// 启用 TIM2 中断 

    CN_FIFO_init(&CN_UART3_FIFO,CN_UART3_temp,CN_UART_TEMP_SIZE);//挂载发送缓冲区
    __HAL_UART_CLEAR_IT(&UART3_Handle,UART_IT_TC);
    __HAL_UART_ENABLE_IT(&UART3_Handle,UART_IT_TC);
    __HAL_UART_CLEAR_IT(&UART3_Handle,UART_IT_TC);
    __HAL_UART_ENABLE_IT(&UART3_Handle,UART_IT_RXNE);

    
    HAL_UART_Receive_IT(&UART3_Handle,&UART_3_RX_dat,1);
}

void CN_UART4_init(void){
    GPIO_InitTypeDef GPIO_Initure={0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_10|GPIO_PIN_11;
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用模式
	GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //高速
    GPIO_Initure.Alternate=GPIO_AF8_UART4;
	HAL_GPIO_Init(GPIOC,&GPIO_Initure);

    __HAL_RCC_UART4_CLK_ENABLE();
	UART4_Handle.Instance=UART4;					    //USART1
	UART4_Handle.Init.BaudRate=115200;				    //波特率
	UART4_Handle.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART4_Handle.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART4_Handle.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART4_Handle.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART4_Handle.Init.Mode=UART_MODE_TX_RX;		    //收发模式
    UART4_Handle.Init.FIFOMode=UART_FIFOMODE_DISABLE;

    HAL_UART_Init(&UART4_Handle);
    HAL_NVIC_SetPriority(UART4_IRQn, 2, 0);// 设置中断优先级
    HAL_NVIC_EnableIRQ(UART4_IRQn);// 启用 UART4 中断 

    CN_FIFO_init(&CN_UART4_FIFO,CN_UART4_temp,CN_UART_TEMP_SIZE);//挂载发送缓冲区
    __HAL_UART_ENABLE_IT(&UART4_Handle,UART_IT_TC);
    __HAL_UART_ENABLE_IT(&UART4_Handle,UART_IT_RXNE);

    HAL_UART_Receive_IT(&UART4_Handle,&UART_4_RX_dat,1);
}

void CN_UART5_init(void){
    GPIO_InitTypeDef GPIO_Initure={0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_12|GPIO_PIN_13;
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用模式
	GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //高速
    GPIO_Initure.Alternate=GPIO_AF14_UART5;
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);

    __HAL_RCC_UART5_CLK_ENABLE();
	UART5_Handle.Instance=UART5;					    //USART1
	UART5_Handle.Init.BaudRate=115200;				    //波特率
	UART5_Handle.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART5_Handle.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART5_Handle.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART5_Handle.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART5_Handle.Init.Mode=UART_MODE_TX_RX;		    //收发模式
    UART5_Handle.Init.FIFOMode=UART_FIFOMODE_DISABLE;

    HAL_UART_Init(&UART5_Handle);
    HAL_NVIC_SetPriority(UART5_IRQn, 2, 0);// 设置中断优先级
    HAL_NVIC_EnableIRQ(UART5_IRQn);// 启用 UART5 中断 

    CN_FIFO_init(&CN_UART5_FIFO,CN_UART5_temp,CN_UART_TEMP_SIZE);//挂载发送缓冲区
    __HAL_UART_ENABLE_IT(&UART5_Handle,UART_IT_TC);
    __HAL_UART_ENABLE_IT(&UART5_Handle,UART_IT_RXNE);

    HAL_UART_Receive_IT(&UART5_Handle,&UART_5_RX_dat,1);
}

void CN_UART6_init(void){
    GPIO_InitTypeDef GPIO_Initure={0};

    __HAL_RCC_GPIOG_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_9|GPIO_PIN_14;
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用模式
	GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //高速
    GPIO_Initure.Alternate=GPIO_AF7_USART6;
	HAL_GPIO_Init(GPIOG,&GPIO_Initure);

    __HAL_RCC_USART6_CLK_ENABLE();
	UART6_Handle.Instance=USART6;					    //USART1
	UART6_Handle.Init.BaudRate=115200;				    //波特率
	UART6_Handle.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART6_Handle.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART6_Handle.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART6_Handle.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART6_Handle.Init.Mode=UART_MODE_TX_RX;		    //收发模式
    UART6_Handle.Init.FIFOMode=UART_FIFOMODE_DISABLE;

    HAL_UART_Init(&UART6_Handle);
    HAL_NVIC_SetPriority(USART6_IRQn, 2, 0);// 设置中断优先级
    HAL_NVIC_EnableIRQ(USART6_IRQn);// 启用 USART6 中断 

    CN_FIFO_init(&CN_UART6_FIFO,CN_UART6_temp,CN_UART_TEMP_SIZE);//挂载发送缓冲区
    __HAL_UART_ENABLE_IT(&UART6_Handle,UART_IT_TC);
    __HAL_UART_ENABLE_IT(&UART6_Handle,UART_IT_RXNE);

    HAL_UART_Receive_IT(&UART6_Handle,&UART_6_RX_dat,1);
}

void CN_UART7_init(void){
    GPIO_InitTypeDef GPIO_Initure={0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_3|GPIO_PIN_4;
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用模式
	GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //高速
    GPIO_Initure.Alternate=GPIO_AF11_UART7;
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);

    __HAL_RCC_UART7_CLK_ENABLE();
	UART7_Handle.Instance=UART7;					    //USART1
	UART7_Handle.Init.BaudRate=115200;				    //波特率
	UART7_Handle.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART7_Handle.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART7_Handle.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART7_Handle.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART7_Handle.Init.Mode=UART_MODE_TX_RX;		    //收发模式
    UART7_Handle.Init.FIFOMode=UART_FIFOMODE_DISABLE;

    HAL_UART_Init(&UART7_Handle);
    HAL_NVIC_SetPriority(UART7_IRQn, 2, 0);// 设置中断优先级
    HAL_NVIC_EnableIRQ(UART7_IRQn);// 启用 UART7 中断 

    CN_FIFO_init(&CN_UART7_FIFO,CN_UART7_temp,CN_UART_TEMP_SIZE);//挂载发送缓冲区
    __HAL_UART_ENABLE_IT(&UART7_Handle,UART_IT_TC);
    __HAL_UART_ENABLE_IT(&UART7_Handle,UART_IT_RXNE);

    HAL_UART_Receive_IT(&UART7_Handle,&UART_7_RX_dat,1);
}

void CN_UART8_init(void){
    GPIO_InitTypeDef GPIO_Initure={0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1;
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用模式
	GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //高速
    GPIO_Initure.Alternate=GPIO_AF8_UART8;
	HAL_GPIO_Init(GPIOE,&GPIO_Initure);

    __HAL_RCC_UART8_CLK_ENABLE();
	UART8_Handle.Instance=UART8;					    //USART1
	UART8_Handle.Init.BaudRate=115200;				    //波特率
	UART8_Handle.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART8_Handle.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART8_Handle.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART8_Handle.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART8_Handle.Init.Mode=UART_MODE_TX_RX;		    //收发模式
    UART8_Handle.Init.FIFOMode=UART_FIFOMODE_DISABLE;

    HAL_UART_Init(&UART8_Handle);
    HAL_NVIC_SetPriority(UART8_IRQn, 2, 0);// 设置中断优先级
    HAL_NVIC_EnableIRQ(UART8_IRQn);// 启用 UART8 中断 

    CN_FIFO_init(&CN_UART8_FIFO,CN_UART8_temp,CN_UART_TEMP_SIZE);//挂载发送缓冲区
    __HAL_UART_ENABLE_IT(&UART8_Handle,UART_IT_TC);
    __HAL_UART_ENABLE_IT(&UART8_Handle,UART_IT_RXNE);

    HAL_UART_Receive_IT(&UART8_Handle,&UART_8_RX_dat,1);
}

// 串口反初始化
void CN_UART_deinit(CN_UART_ENUM Moudle){
    UART_HandleTypeDef *Moudle_use;
    switch(Moudle){
        case CN_UART_1:{
            Moudle_use=&UART1_Handle;
            break;
        }
        case CN_UART_2:{
            Moudle_use=&UART2_Handle;
            break;
        }
        case CN_UART_3:{
            Moudle_use=&UART3_Handle;
            break;
        }
        case CN_UART_4:{
            Moudle_use=&UART4_Handle;
            break;
        }
        case CN_UART_5:{
            Moudle_use=&UART5_Handle;
            break;
        }
        case CN_UART_6:{
            Moudle_use=&UART6_Handle;
            break;
        }
        case CN_UART_7:{
            Moudle_use=&UART7_Handle;
            break;
        }
        case CN_UART_8:{
            Moudle_use=&UART8_Handle;
            break;
        }
    }

    HAL_UART_DeInit(Moudle_use);
}

// 串口阻塞式发送字符串
void CN_UART_send_string_block(CN_UART_ENUM Moudle,const uint8_t *str){
    uint32_t len=strlen(str);
    USART_TypeDef *Moudle_use;
    switch(Moudle){
        case CN_UART_1:{
            Moudle_use=USART1;
            break;
        }
        case CN_UART_2:{
            Moudle_use=USART2;
            break;
        }
        case CN_UART_3:{
            Moudle_use=USART3;
            break;
        }
        case CN_UART_4:{
            Moudle_use=UART4;
            break;
        }
        case CN_UART_5:{
            Moudle_use=UART5;
            break;
        }
        case CN_UART_6:{
            Moudle_use=USART6;
            break;
        }
        case CN_UART_7:{
            Moudle_use=UART7;
            break;
        }
        case CN_UART_8:{
            Moudle_use=UART8;
            break;
        }
    }

    for(int i=0;i<len;i++){
        while((Moudle_use->ISR&0X40)==0);//循环发送,直到发送完毕   
        Moudle_use->TDR=(uint8_t)str[i];
    }    
}

// 串口非阻塞式发送字符串
void CN_UART_send_string_IT(CN_UART_ENUM Moudle,uint8_t *str){
    uint16_t len=strlen(str);
    UART_HandleTypeDef *Moudle_use;
    switch(Moudle){
        case CN_UART_1:{
            Moudle_use=&UART1_Handle;
            break;
        }
        case CN_UART_2:{
            Moudle_use=&UART2_Handle;
            break;
        }
        case CN_UART_3:{
            Moudle_use=&UART3_Handle;
            break;
        }
        case CN_UART_4:{
            Moudle_use=&UART4_Handle;
            break;
        }
        case CN_UART_5:{
            Moudle_use=&UART5_Handle;
            break;
        }
        case CN_UART_6:{
            Moudle_use=&UART6_Handle;
            break;
        }
        case CN_UART_7:{
            Moudle_use=&UART7_Handle;
            break;
        }
        case CN_UART_8:{
            Moudle_use=&UART8_Handle;
            break;
        }
    }
    HAL_UART_Transmit_IT(Moudle_use,str,len);
}

// 串口非阻塞式发送数据
void CN_UART_send_data_IT(CN_UART_ENUM Moudle,uint8_t *data,uint32_t len){
    UART_HandleTypeDef *Moudle_use;
    switch(Moudle){
        case CN_UART_1:{
            Moudle_use=&UART1_Handle;
            break;
        }
        case CN_UART_2:{
            Moudle_use=&UART2_Handle;
            break;
        }
        case CN_UART_3:{
            Moudle_use=&UART3_Handle;
            break;
        }
        case CN_UART_4:{
            Moudle_use=&UART4_Handle;
            break;
        }
        case CN_UART_5:{
            Moudle_use=&UART5_Handle;
            break;
        }
        case CN_UART_6:{
            Moudle_use=&UART6_Handle;
            break;
        }
        case CN_UART_7:{
            Moudle_use=&UART7_Handle;
            break;
        }
        case CN_UART_8:{
            Moudle_use=&UART8_Handle;
            break;
        }

    }
    HAL_UART_Transmit_IT(Moudle_use,data,len);
}

uint8_t U1_TX_temp_g[CN_UART_TEMP_SIZE]={0};//必须要通过这个数组发送，否则调用HAL_UART_Transmit_IT时找不到原局部变量数组就会报错
uint8_t U2_TX_temp_g[CN_UART_TEMP_SIZE]={0};//必须要通过这个数组发送，否则调用HAL_UART_Transmit_IT时找不到原局部变量数组就会报错
uint8_t U3_TX_temp_g[CN_UART_TEMP_SIZE]={0};//必须要通过这个数组发送，否则调用HAL_UART_Transmit_IT时找不到原局部变量数组就会报错
uint8_t U4_TX_temp_g[CN_UART_TEMP_SIZE]={0};//必须要通过这个数组发送，否则调用HAL_UART_Transmit_IT时找不到原局部变量数组就会报错
uint8_t U5_TX_temp_g[CN_UART_TEMP_SIZE]={0};//必须要通过这个数组发送，否则调用HAL_UART_Transmit_IT时找不到原局部变量数组就会报错
uint8_t U6_TX_temp_g[CN_UART_TEMP_SIZE]={0};//必须要通过这个数组发送，否则调用HAL_UART_Transmit_IT时找不到原局部变量数组就会报错
uint8_t U7_TX_temp_g[CN_UART_TEMP_SIZE]={0};//必须要通过这个数组发送，否则调用HAL_UART_Transmit_IT时找不到原局部变量数组就会报错
uint8_t U8_TX_temp_g[CN_UART_TEMP_SIZE]={0};//必须要通过这个数组发送，否则调用HAL_UART_Transmit_IT时找不到原局部变量数组就会报错

static void CN_UART_start_tx_from_fifo(UART_HandleTypeDef *huart,CN_FIFO_struct *fifo,uint8_t *tx_buf){
    uint16_t tx_len=CN_get_FIFO_occupied(fifo);

    if(tx_len==0u){
        return;
    }
    if(tx_len>CN_UART_TEMP_SIZE){
        tx_len=CN_UART_TEMP_SIZE;
    }
    if(CN_FIFO_read(fifo,tx_buf,tx_len)!=0u){
        HAL_UART_Transmit_IT(huart,tx_buf,tx_len);
    }
}

// 串口写入缓冲区
void CN_UART_write_fifo(CN_UART_ENUM Moudle,uint8_t *data,uint32_t len){
    UART_HandleTypeDef *Moudle_use=NULL;
    CN_FIFO_struct *FIFO_use=NULL;
    uint8_t *TX_temp_g_use=NULL;
    switch(Moudle){
        case CN_UART_1:{
            Moudle_use=&UART1_Handle;
            FIFO_use=&CN_UART1_FIFO;
            TX_temp_g_use=U1_TX_temp_g;
            break;
        }
        case CN_UART_2:{
            Moudle_use=&UART2_Handle;
            FIFO_use=&CN_UART2_FIFO;
            TX_temp_g_use=U2_TX_temp_g;
            break;
        }
        case CN_UART_3:{
            Moudle_use=&UART3_Handle;
            FIFO_use=&CN_UART3_FIFO;
            TX_temp_g_use=U3_TX_temp_g;
            break;
        }
        case CN_UART_4:{
            Moudle_use=&UART4_Handle;
            FIFO_use=&CN_UART4_FIFO;
            TX_temp_g_use=U4_TX_temp_g;
            break;
        }
        case CN_UART_5:{
            Moudle_use=&UART5_Handle;
            FIFO_use=&CN_UART5_FIFO;
            TX_temp_g_use=U5_TX_temp_g;
            break;
        }
        case CN_UART_6:{
            Moudle_use=&UART6_Handle;
            FIFO_use=&CN_UART6_FIFO;
            TX_temp_g_use=U6_TX_temp_g;
            break;
        }
        case CN_UART_7:{
            Moudle_use=&UART7_Handle;
            FIFO_use=&CN_UART7_FIFO;
            TX_temp_g_use=U7_TX_temp_g;
            break;
        }
        case CN_UART_8:{
            Moudle_use=&UART8_Handle;
            FIFO_use=&CN_UART8_FIFO;
            TX_temp_g_use=U8_TX_temp_g;
            break;
        }
    }
    if((Moudle_use == NULL) || (FIFO_use == NULL) || (TX_temp_g_use == NULL) || (data == NULL) || (len == 0u) || (len > CN_UART_TEMP_SIZE)){
        return;
    }

    uint32_t primask=__get_PRIMASK();
    __disable_irq();
    if(CN_FIFO_write(FIFO_use,data,(uint16_t)len)!=0u){
        if(Moudle_use->gState==HAL_UART_STATE_READY){
            CN_UART_start_tx_from_fifo(Moudle_use,FIFO_use,TX_temp_g_use);
        }
    }
    if(primask==0u){
        __enable_irq();
    }
}

// 串口写入缓冲区 发送字符串
void CN_UART_write_string_FIFO(CN_UART_ENUM Moudle,uint8_t *str){
    uint16_t len=strlen(str);
    CN_UART_write_fifo(Moudle,str,len);
}

void USART1_IRQHandler(void){HAL_UART_IRQHandler(&UART1_Handle);}
void USART2_IRQHandler(void){HAL_UART_IRQHandler(&UART2_Handle);}
void USART3_IRQHandler(void){HAL_UART_IRQHandler(&UART3_Handle);}
void UART4_IRQHandler(void){HAL_UART_IRQHandler(&UART4_Handle);}
void UART5_IRQHandler(void){HAL_UART_IRQHandler(&UART5_Handle);}
void USART6_IRQHandler(void){HAL_UART_IRQHandler(&UART6_Handle);}
void UART7_IRQHandler(void){HAL_UART_IRQHandler(&UART7_Handle);}
void UART8_IRQHandler(void){HAL_UART_IRQHandler(&UART8_Handle);}

// void CN_UART1_RX_CALLBACK(uint8_t dat){
//     ARM_RX_Callback(dat);
// }
// void CN_UART2_RX_CALLBACK(uint8_t dat){}
// void CN_UART3_RX_CALLBACK(uint8_t dat){}
void CN_UART4_RX_CALLBACK(uint8_t dat){}
void CN_UART5_RX_CALLBACK(uint8_t dat){}
void CN_UART6_RX_CALLBACK(uint8_t dat){}
// void CN_UART7_RX_CALLBACK(uint8_t dat){}
void CN_UART8_RX_CALLBACK(uint8_t dat){}

// 串口发送回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){

    if(huart->Instance==USART1){
        CN_UART_start_tx_from_fifo(&UART1_Handle,&CN_UART1_FIFO,U1_TX_temp_g);
    }
    else if(huart->Instance==USART2){
        CN_UART_start_tx_from_fifo(&UART2_Handle,&CN_UART2_FIFO,U2_TX_temp_g);
    }
    else if(huart->Instance==USART3){
        CN_UART_start_tx_from_fifo(&UART3_Handle,&CN_UART3_FIFO,U3_TX_temp_g);
    }
    else if(huart->Instance==UART4){
        CN_UART_start_tx_from_fifo(&UART4_Handle,&CN_UART4_FIFO,U4_TX_temp_g);
    }
    else if(huart->Instance==UART5){
        CN_UART_start_tx_from_fifo(&UART5_Handle,&CN_UART5_FIFO,U5_TX_temp_g);
    }
    else if(huart->Instance==USART6){
        CN_UART_start_tx_from_fifo(&UART6_Handle,&CN_UART6_FIFO,U6_TX_temp_g);
    }
    else if(huart->Instance==UART7){
        CN_UART_start_tx_from_fifo(&UART7_Handle,&CN_UART7_FIFO,U7_TX_temp_g);
    }
    else if(huart->Instance==UART8){
        CN_UART_start_tx_from_fifo(&UART8_Handle,&CN_UART8_FIFO,U8_TX_temp_g);
    }
}

// 串口接收回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance==USART1){
        // CN_Debug_UART_RX_CALLBACK(UART_1_RX_dat);
        CN_UART1_RX_CALLBACK(UART_1_RX_dat);
        HAL_UART_Receive_IT(&UART1_Handle,&UART_1_RX_dat,1);
    }
    if(huart->Instance==USART2){
        CN_UART2_RX_CALLBACK(UART_2_RX_dat);
        HAL_UART_Receive_IT(&UART2_Handle,&UART_2_RX_dat,1);
    }
    if(huart->Instance==USART3){
        CN_UART3_RX_CALLBACK(UART_3_RX_dat);
        HAL_UART_Receive_IT(&UART3_Handle,&UART_3_RX_dat,1);
    }
    if(huart->Instance==UART4){
        CN_UART4_RX_CALLBACK(UART_4_RX_dat);
        HAL_UART_Receive_IT(&UART4_Handle,&UART_4_RX_dat,1);
    }
    if(huart->Instance==UART5){
        CN_UART5_RX_CALLBACK(UART_5_RX_dat);
        HAL_UART_Receive_IT(&UART5_Handle,&UART_5_RX_dat,1);
    }
    if(huart->Instance==USART6){
        CN_UART6_RX_CALLBACK(UART_6_RX_dat);
        HAL_UART_Receive_IT(&UART6_Handle,&UART_6_RX_dat,1);
    }
    if(huart->Instance==UART7){
        CN_UART7_RX_CALLBACK(UART_7_RX_dat);
        HAL_UART_Receive_IT(&UART7_Handle,&UART_7_RX_dat,1);
    }
    if(huart->Instance==UART8){
        CN_UART8_RX_CALLBACK(UART_8_RX_dat);
        HAL_UART_Receive_IT(&UART8_Handle,&UART_8_RX_dat,1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3)
    {
        // 1. 清除溢出和帧错误标志
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        
        // 2. 强制重新开启单字节接收中断
        HAL_UART_Receive_IT(&UART3_Handle, &UART_3_RX_dat, 1);
    }
}
/******************************************************************数据链路层*********************************************************/

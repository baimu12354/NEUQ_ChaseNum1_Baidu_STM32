#include "stm32h7xx_hal.h"
#include "ChaseNum1_Include.h"
#include <string.h>
#include <stdio.h>
// #include <stdlib.h>

Main_COM_Package_t Package_get={0};
uint8_t mission_check=0;

/******************************************************************应用层*********************************************************/
void Main_COM_init(void){
    CN_UART3_init();
    Package_get.done=1;//接收缓冲空闲状态
}

void Main_COM_send_pack(Main_CMD_ENUM CMD,uint8_t *load,uint8_t load_len){
    uint8_t data[256]={0};
    data[0]=0xAA;
    data[1]=0xBB;
    data[2]=(uint8_t)CMD;
    data[3]=load_len;
    memcpy(&data[4],load,load_len);
    uint8_t check=0;
    for(int i=0;i<load_len;i++){
        check+=data[4+i];
    }
    data[4+load_len]=check;
    data[5+load_len]=0x0F;
    data[6+load_len]=0x0A;

    // TS_UART_write_fifo(TS_UART_1,data,7+load_len);

    CN_UART_write_fifo(CN_UART_3,data,7+load_len);
}

void Main_COM_Response_CALLBACK(void){
    if(Package_get.done==0){
        switch(Package_get.CMD){
            case Main_CMD_HELLO:{
                printf("get CMD_HELLO\n");
                uint8_t response[2]={Main_CMD_INLINE,0x00};
                CN_send_Response(Main_CMD_REPLY,response);  

                Package_get.done=1;
                break;
            }

            case Main_CMD_STATUS:{
                printf("get CMD_STATUS\n");
                uint8_t response[2]={Main_CMD_ASTATUS,SYS_STATUS.all};
                CN_send_Response(Main_CMD_ASTATUS,response);  

                Package_get.done=1;
                break;
            }

            case Main_CMD_SETMOVEMENT:{
                printf("get CMD_SETM\n");
                CMD_SETM_load_struct CMD_SETM_load;
                CMD_SETM_load.Movement_type=Package_get.load[0];
                memcpy(&CMD_SETM_load.Movement,&Package_get.load[1],Package_get.load_len-1);
                if(CMD_SETM_load.Movement_type==Movement_Set_speed){
                    if(SYS_STATUS.bits.SYS_INITED){
                        SYS_STATUS.bits.SYS_INLINE=1;
                        SYS_STATUS.bits.SYS_MOVING=0;
                        SYS_STATUS.bits.SYS_ROTATING=0;
                        CN_set_movement(CMD_SETM_load.Movement,(Movement_type_ENUM)CMD_SETM_load.Movement_type,Package_get.check);
                        mission_check=Package_get.check;
                        uint8_t response[2]={CMD_SETMOVEMENT,Package_get.check};
                        CN_send_Response(CMD_RECVCMD,response);                         
                    }
                    else{
                        printf("not inited \n");
                        uint8_t response[2]={CMD_SETMOVEMENT,SYS_STATUS.all};
                        CN_send_Response(Main_CMD_ERR,response);   
                    }                    
                }
                else if(CMD_SETM_load.Movement_type==Movement_Translation || CMD_SETM_load.Movement_type==Movement_Rotation){
                    if(Movement_Precise_Mode){
                        CN_Movement_Change_To_Normal_Mode();
                    }
                    if(Motor_Precise_Mode){
                        CN_Motor_Change_To_Normal_Mode();
                    }
                    SYS_STATUS.bits.SYS_INLINE=0;
                    CN_set_movement(CMD_SETM_load.Movement,(Movement_type_ENUM)CMD_SETM_load.Movement_type,Package_get.check);
                    mission_check=Package_get.check;
                    uint8_t response[2]={CMD_SETMOVEMENT,Package_get.check};
                    CN_send_Response(CMD_RECVCMD,response);   
                }
                else if(CMD_SETM_load.Movement_type==Movement_Translation_Precise || CMD_SETM_load.Movement_type==Movement_Rotation_Precise){
                    if(!Movement_Precise_Mode){
                        CN_Movement_Change_To_Precise_Mode();
                    }
                    if(!Motor_Precise_Mode){
                        CN_Motor_Change_To_Precise_Mode();
                    }
                    SYS_STATUS.bits.SYS_INLINE=0;
                    CN_set_movement(CMD_SETM_load.Movement,(Movement_type_ENUM)CMD_SETM_load.Movement_type,Package_get.check);
                    mission_check=Package_get.check;
                    uint8_t response[2]={CMD_SETMOVEMENT,Package_get.check};
                    CN_send_Response(CMD_RECVCMD,response);   
                }
                else{
                    printf("unknown movement type:%d\n",CMD_SETM_load.Movement_type);
                    // uint8_t response[2]={CMD_SETMOVEMENT,SYS_STATUS.all};
                    // CN_send_Response(Main_CMD_ERR,response);   
                }
                Package_get.done=1;
                break;
            }

            case Main_CMD_GETPARA:{
                printf("get CMD_REQ:%x\n",Package_get.load[0]);
                switch(Package_get.load[0]){
                    case REQ_type_YAW:{
                        float logical_yaw=CN_get_logical_yaw();
                        Main_COM_send_pack(CMD_APARA,(uint8_t*)&logical_yaw,4);
                        break;
                    }
                    case REQ_type_Mile:{
                        float mile=CN_get_Milemeter();
                        printf("send mile:%f\n",mile);
                        Main_COM_send_pack(CMD_APARA,(uint8_t*)&mile,4);
                        break;
                    }
                }
                Package_get.done=1;
                break;
            }

            case Main_CMD_INLINE:{
                printf("get CMD_INLINE\n");
                if(SYS_STATUS.bits.SYS_INITED==1){
                    SYS_STATUS.bits.SYS_INLINE=1;
                    uint8_t response[2]={Main_CMD_INLINE,Package_get.check};
                    CN_send_Response(Main_CMD_RECVCMD,response);                    
                }
                else{
                    uint8_t response[2]={Main_CMD_INLINE,SYS_STATUS.all};
                    CN_send_Response(Main_CMD_ERR,response);      
                }

                Package_get.done=1;
                break;
            }

            case Main_CMD_OUTLINE:{
                printf("get CMD_OUTLINE\n");
                CN_Motor_outline();
                uint8_t response[2]={Main_CMD_OUTLINE,Package_get.check};
                CN_send_Response(Main_CMD_RECVCMD,response);                    
                Package_get.done=1;
                break;
            }

            case Main_CMD_RESTART:{
                printf("get CMD_RST\n");
                uint8_t response[2]={Main_CMD_RESTART,Package_get.check};
                CN_send_Response(Main_CMD_RECVCMD,response);  
                CN_delay_ms(500);
                NVIC_SystemReset();
                Package_get.done=1;
                break;
            }

            case Main_CMD_INTERRUPT:{
                printf("get CMD_INT\n");
                CN_Intr_Movement();
                uint8_t response[2]={Main_CMD_INTERRUPT,Package_get.check};
                CN_send_Response(Main_CMD_RECVCMD,response);  
                Package_get.done=1;
                break;
            }

            case Main_CMD_SETPARA:{
                switch(Package_get.load[0]){
                    printf("get CMD_SET:%x\n",Package_get.load[0]);
                    case SET_YAW:{
                        float logical_yaw=0;
                        memcpy(&logical_yaw,&Package_get.load[1],4);
                        CN_Set_YAW(logical_yaw);
                        printf("set yaw:%d\n",logical_yaw);
                        uint8_t response[2]={Main_CMD_SETPARA,Package_get.check};
                        CN_send_Response(Main_CMD_RECVCMD,response);
                        break;
                    }
                    case SET_MILE:{
                        float mile=0;
                        memcpy(&mile,&Package_get.load[1],4);
                        CN_Milemeter_set(mile);
                        printf("set mile:%f\n",mile);
                        uint8_t response[2]={Main_CMD_SETPARA,Package_get.check};
                        CN_send_Response(Main_CMD_RECVCMD,response);
                        break;
                    }
                }
                Package_get.done=1;
                break;
            }

            // case Main_CMD_SETACTION:{
            //     printf("get CMD_SETA\n");
            //     // for(int i=0;i<6;i++){
            //     //     printf("%X\n",Package_get.load[i]);
            //     // }

            //     if(SYS_STATUS.bits.SYS_INITED){
            //         Action_CMD_struct Act_get;
            //         Act_get.Action_type=(Action_type_ENUM)Package_get.load[0];
            //         Act_get.Action_method=(Action_method_ENUM)Package_get.load[1];
            //         memcpy(&Act_get.value,&Package_get.load[2],4);
            //         CN_set_Action(Act_get,Package_get.check);
            //         // uint8_t response[2]={Main_CMD_SETACTION,Package_get.check};
            //         // CN_send_Response(Main_CMD_RECVCMD,response);                    
            //     }
            //     else{
            //         uint8_t response[2]={Main_CMD_SETACTION,SYS_STATUS.all};
            //         CN_send_Response(Main_CMD_ERR,response);  
            //     }

            //     Package_get.done=1;
            //     break;
            // }
        
            case Main_CMD_SERVERSTATUS:{
                printf("get CMD_SERVERSTATUS\n");
                if(Package_get.load[0]==1){
                    flag_server_started=1;
                }
                else{
                    flag_server_started=0;
                }
                
                flag_menu_refresh=1;
                Package_get.done=1;
                break;
            }
        }
    }
}

void CN_send_Response(Main_CMD_ENUM CMD,uint8_t* response){
    switch(CMD){
        case Main_CMD_REPLY:{
            printf("send CMD_REP\n");
            uint8_t data[7]={0xAA,0xBB,CMD,0x00,0x00,0x0F,0x0A};
            CN_UART_write_fifo(CN_UART_3,data,7);
            break;
        }
        
        case Main_CMD_ASTATUS:{
            printf("send CMD_SUC\n");
            uint8_t data[8]={0xAA,0xBB,CMD,0x01,SYS_STATUS.all,SYS_STATUS.all,0x0F,0x0A};
            CN_UART_write_fifo(CN_UART_3,data,8);
            break;                
        }

        case Main_CMD_APARA:{
            printf("send CMD_APARA\n");
            uint8_t data[11]={0xAA,0xBB,CMD,0x04,0x00,0x00,0x00,0x00,0x00,0x0F,0x0A};
            memcpy(&data[4],response,4);
            uint8_t check=0;
            for(int i=0;i<4;i++){
                check+=response[i];
            }
            data[8]=check;
            CN_UART_write_fifo(CN_UART_3,data,11);
            break;
        }

        case Main_CMD_RECVCMD:{
            printf("send CMD_RECVCMD\n");
            uint8_t data[9]={0xAA,0xBB,CMD,0x02,0x00,0x00,0x00,0x0F,0x0A};
            data[4]=response[0];
            data[5]=response[1];
            data[6]=(uint8_t)(data[4]+data[5]);
            CN_UART_write_fifo(CN_UART_3,data,9);
            break;
        }
        
        case Main_CMD_CMDDONE:{
            printf("send CMD_CMDDONE\n");
            uint8_t data[8]={0xAA,0xBB,CMD,0x01,response[0],response[0],0x0F,0x0A};
            CN_UART_write_fifo(CN_UART_3,data,8);
            break;
        }
        
        case Main_CMD_ERR:{
            printf("send CMD_ERR\n");
            uint8_t data[9]={0xAA,0xBB,CMD,0x02,response[0],response[1],(uint8_t)(response[0]+response[1]),0x0F,0x0A};
            CN_UART_write_fifo(CN_UART_3,data,9);
            break;
        }

        case Main_CMD_STARTSERVER:{
            printf("send CMD_STARTSERVER\n");
            uint8_t data[7]={0xAA,0xBB,CMD,0x00,0x00,0x0F,0x0A};
            CN_UART_write_fifo(CN_UART_3,data,7);
            break;
        }

        case Main_CMD_CLOSESERVER:{
            printf("send CMD_CLOSESERVER\n");
            uint8_t data[7]={0xAA,0xBB,CMD,0x00,0x00,0x0F,0x0A};
            CN_UART_write_fifo(CN_UART_3,data,7);
            break;
        }

        case Main_CMD_STARTMISSION:{
            printf("send CMD_STARTMISSION\n");
            uint8_t data[7]={0xAA,0xBB,CMD,0x00,0x00,0x0F,0x0A};
            CN_UART_write_fifo(CN_UART_3,data,7);
            break;
        }

        case Main_CMD_SUSPENDMISSION:{
            printf("send CMD_SUSPENDMISSION\n");
            uint8_t data[7]={0xAA,0xBB,CMD,0x00,0x00,0x0F,0x0A};
            CN_UART_write_fifo(CN_UART_3,data,7);
            break;
        }

        default:{
            break;
        }
    }
}

//接收拼包
void Main_COM_RX_CALLBACK(uint8_t dat){
    static int counter=0;
    static int load_len=0;
    static int tail_start=0;
    static uint8_t temp[256]={0};
    static uint8_t check_get=0;
    static Main_CMD_ENUM Command;

    // printf("get:%X\n",dat);


    /**********帧头#1***********/
    if(counter==0){
        if(dat==0xAA){
            counter++;
        }
        else{
            counter=0;
        }        
    }

    /**********帧头#2***********/
    else if(counter==1){
        if(dat==0xBB){
            counter++;
        }
        else{
            counter=0;
        }
    }

    /**********指令***********/
    else if(counter==2){
        if(IS_RX_MAIN_CMD(dat)){
            Command=(Main_CMD_ENUM)dat;
            // printf("cmd=%x\n",Command);
            counter++;                
        }
        else{
            counter=0;
        }
    }

    /**********载荷长度***********/
    else if(counter==3){
        load_len=dat;
        counter++;      
        tail_start=load_len+4;        
    }

    /**********载荷***********/
    else if(counter>3 && counter<tail_start){
        temp[counter-4]=dat;
        counter++;
    }

    /**********校验***********/
    else if(counter==tail_start){
        uint8_t check=0;
        for(int i=0;i<load_len;i++){
            check+=temp[i];
        }
        if(check==dat){
            counter++;
            check_get=check;
            // printf("check success\n");
        }
        else{
            printf("check fail %x %x\n",check,dat);
            counter=0;
        }
    }

    /**********帧尾#1***********/
    else if(counter==tail_start+1){
        if(dat==0x0F){
            counter++;
        }
        else{
            counter=0;
        }
    }

    /**********帧尾#2***********/
    else if(counter==tail_start+2){
        if(dat==0x0A){
            printf("get package %x with load %d\n",(uint8_t)Command,load_len);        
            if(Package_get.done==1){
                Package_get.CMD=Command;
                Package_get.load_len=load_len;
                Package_get.check=check_get;
                memcpy(Package_get.load,temp,load_len);
                Package_get.done=0;
                Main_COM_Response_CALLBACK();
            }
            else{
                printf("mission undone\n");
            }
        }
        counter=0;
    }

    else{
        counter=0;
    }
}

void CN_UART3_RX_CALLBACK(uint8_t dat){
    Main_COM_RX_CALLBACK(dat);
}

void CN_UART4_RX_CALLBACK(uint8_t dat){
}
void CN_UART5_RX_CALLBACK(uint8_t dat){
}
void CN_UART6_RX_CALLBACK(uint8_t dat){
}
void CN_UART7_RX_CALLBACK(uint8_t dat){
}

/******************************************************************应用层*********************************************************/

/******************************************************************数据链路层*********************************************************/
#define CN_UART_TEMP_SIZE   (256)

UART_HandleTypeDef UART1_Handle={0};
UART_HandleTypeDef UART2_Handle={0};
UART_HandleTypeDef UART3_Handle={0};
UART_HandleTypeDef UART4_Handle={0};
UART_HandleTypeDef UART5_Handle={0};
UART_HandleTypeDef UART6_Handle={0};
UART_HandleTypeDef UART7_Handle={0};

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

uint8_t UART_1_RX_dat=0;
uint8_t UART_2_RX_dat=0;
uint8_t UART_3_RX_dat=0;
uint8_t UART_4_RX_dat=0;
uint8_t UART_5_RX_dat=0;
uint8_t UART_6_RX_dat=0;
uint8_t UART_7_RX_dat=0;

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
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);

    __HAL_RCC_USART3_CLK_ENABLE();
	UART3_Handle.Instance=USART3;					    //USART1
	UART3_Handle.Init.BaudRate=115200;				    //波特率
	UART3_Handle.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART3_Handle.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART3_Handle.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART3_Handle.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART3_Handle.Init.Mode=UART_MODE_TX_RX;		    //收发模式
    UART3_Handle.Init.FIFOMode=UART_FIFOMODE_DISABLE;

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
    }

    HAL_UART_DeInit(Moudle_use);
}

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
    }

    for(int i=0;i<len;i++){
        while((Moudle_use->ISR&0X40)==0);//循环发送,直到发送完毕   
        Moudle_use->TDR=(uint8_t)str[i];
    }    
}

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
    }
    HAL_UART_Transmit_IT(Moudle_use,str,len);
}

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

void CN_UART_write_fifo(CN_UART_ENUM Moudle,uint8_t *data,uint32_t len){
    UART_HandleTypeDef *Moudle_use;
    CN_FIFO_struct *FIFO_use;
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
    }
    if(Moudle_use->gState==HAL_UART_STATE_READY){                   //如果串口空闲
        uint32_t FIFO_occupied=CN_get_FIFO_occupied(FIFO_use);      //计算FIFO剩余字节数
        if(FIFO_occupied>0){                                        //如果FIFO中还有数据
            CN_FIFO_read(FIFO_use,TX_temp_g_use,FIFO_occupied);              //读出FIFO剩余数据
            HAL_UART_Transmit_IT(Moudle_use,TX_temp_g_use,FIFO_occupied);    //发送剩余数据
            CN_FIFO_write(FIFO_use,data,len);                       //写入当前数据到FIFO
        }
        else{                                                       //如果FIFO中没有数据了
            // printf("send data\n");
            memcpy(TX_temp_g_use,data,len);
            HAL_UART_Transmit_IT(Moudle_use,TX_temp_g_use,len);              //直接发送当前数据
        }
    }
    else{                                                           //如果串口忙
        // printf("write fifo\n");
        CN_FIFO_write(FIFO_use,data,len);                           //写入当前数据到FIFO
    }
}

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

void CN_UART1_RX_CALLBACK(uint8_t dat){}
// void CN_UART2_RX_CALLBACK(uint8_t dat){}
// void CN_UART3_RX_CALLBACK(uint8_t dat){}
// void CN_UART4_RX_CALLBACK(uint8_t dat){}
// void CN_UART5_RX_CALLBACK(uint8_t dat){}
// void CN_UART6_RX_CALLBACK(uint8_t dat){}
// void CN_UART7_RX_CALLBACK(uint8_t dat){}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    
    if(huart->Instance==USART1){
        uint32_t FIFO_occupied=CN_get_FIFO_occupied(&CN_UART1_FIFO);      //计算FIFO剩余字节数
        if(FIFO_occupied>0){                                        //如果FIFO中还有数据  
            CN_FIFO_read(&CN_UART1_FIFO,U1_TX_temp_g,FIFO_occupied);              //读出FIFO剩余数据
            HAL_UART_Transmit_IT(&UART1_Handle,U1_TX_temp_g,FIFO_occupied);    //发送剩余数据
        }
    }
    if(huart->Instance==USART2){
        uint32_t FIFO_occupied=CN_get_FIFO_occupied(&CN_UART2_FIFO);      //计算FIFO剩余字节数
        if(FIFO_occupied>0){                                        //如果FIFO中还有数据
            CN_FIFO_read(&CN_UART2_FIFO,U2_TX_temp_g,FIFO_occupied);              //读出FIFO剩余数据
            HAL_UART_Transmit_IT(&UART2_Handle,U2_TX_temp_g,FIFO_occupied);    //发送剩余数据
        }
    }
    if(huart->Instance==USART3){
        uint32_t FIFO_occupied=CN_get_FIFO_occupied(&CN_UART3_FIFO);      //计算FIFO剩余字节数
        if(FIFO_occupied>0){               //如果FIFO中还有数据        
            CN_FIFO_read(&CN_UART3_FIFO,U3_TX_temp_g,FIFO_occupied);              //读出FIFO剩余数据
            HAL_UART_Transmit_IT(&UART3_Handle,U3_TX_temp_g,FIFO_occupied);    //发送剩余数据
        }
    }
    if(huart->Instance==UART4){
        uint32_t FIFO_occupied=CN_get_FIFO_occupied(&CN_UART4_FIFO);      //计算FIFO剩余字节数
        if(FIFO_occupied>0){               //如果FIFO中还有数据       
            CN_FIFO_read(&CN_UART4_FIFO,U4_TX_temp_g,FIFO_occupied);              //读出FIFO剩余数据
            HAL_UART_Transmit_IT(&UART4_Handle,U4_TX_temp_g,FIFO_occupied);    //发送剩余数据
        }
    }
    if(huart->Instance==UART5){
        uint32_t FIFO_occupied=CN_get_FIFO_occupied(&CN_UART5_FIFO);      //计算FIFO剩余字节数
        if(FIFO_occupied>0){               //如果FIFO中还有数据       
            CN_FIFO_read(&CN_UART5_FIFO,U5_TX_temp_g,FIFO_occupied);              //读出FIFO剩余数据
            HAL_UART_Transmit_IT(&UART5_Handle,U5_TX_temp_g,FIFO_occupied);    //发送剩余数据
        }
    }
    if(huart->Instance==USART6){
        uint32_t FIFO_occupied=CN_get_FIFO_occupied(&CN_UART6_FIFO);      //计算FIFO剩余字节数
        if(FIFO_occupied>0){               //如果FIFO中还有数据       
            CN_FIFO_read(&CN_UART6_FIFO,U6_TX_temp_g,FIFO_occupied);              //读出FIFO剩余数据
            HAL_UART_Transmit_IT(&UART6_Handle,U6_TX_temp_g,FIFO_occupied);    //发送剩余数据
        }
    }
    if(huart->Instance==UART7){
        uint32_t FIFO_occupied=CN_get_FIFO_occupied(&CN_UART7_FIFO);      //计算FIFO剩余字节数
        if(FIFO_occupied>0){               //如果FIFO中还有数据       
            CN_FIFO_read(&CN_UART7_FIFO,U7_TX_temp_g,FIFO_occupied);              //读出FIFO剩余数据
            HAL_UART_Transmit_IT(&UART7_Handle,U7_TX_temp_g,FIFO_occupied);    //发送剩余数据
        }
    }
}

//回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance==USART1){
        // CN_Debug_UART_RX_CALLBACK(UART_1_RX_dat);
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
}

/***************************FIFO驱动***************************/
void CN_FIFO_init(CN_FIFO_struct *FIFO,uint8_t *pdata,uint16_t templen){
    FIFO->temp=pdata;
    FIFO->head=0;
    FIFO->tail=0;
    FIFO->temp_len=templen;
    FIFO->is_FIFO_full=0;
}

uint8_t CN_FIFO_read(CN_FIFO_struct *FIFO,uint8_t *pdata,uint16_t readlen){
    if(CN_get_FIFO_occupied(FIFO)<readlen){
        return 0;
    }
    else{
        for(int i=0;i<readlen;i++){
            uint8_t byte=0;
            CN_FIFO_read_Byte(FIFO,&byte);
            pdata[i]=byte;
        }
        return 1;
    }
}

uint8_t CN_FIFO_write(CN_FIFO_struct *FIFO,uint8_t *pdata,uint16_t writelen){
    if(CN_get_FIFO_space(FIFO)<writelen){
        return 0;
    }
    else{
        for(int i=0;i<writelen;i++){
            CN_FIFO_write_Byte(FIFO,pdata[i]);
        }
        return 1;
    }
}

uint8_t CN_FIFO_read_Byte(CN_FIFO_struct *FIFO,uint8_t *pdata){//从FIFO尾部读出一个字节，如果失败则返回0
    if(CN_get_FIFO_occupied(FIFO)<1){
        return 0;
    }
    else{
        *pdata=FIFO->temp[FIFO->tail];//从尾部读出一个字节数据
        if(FIFO->tail++>=FIFO->temp_len){//尾指针位移
            FIFO->tail=0;
        }
        FIFO->is_FIFO_full=0;//清除满缓存状态
        return 1;
    }
}

uint8_t CN_FIFO_write_Byte(CN_FIFO_struct *FIFO,uint8_t data){//从FIFO头部写入一个字节，如果失败则返回0
    if(CN_get_FIFO_space(FIFO)<1){//检查剩余空间
        return 0;
    }
    else{
        FIFO->temp[FIFO->head]=data;//写入数据
        if(FIFO->head++>=FIFO->temp_len){//头指针位移
            FIFO->head=0;
        }
        if(FIFO->head==FIFO->tail){//检查满缓存状态
            FIFO->is_FIFO_full=1;
        }
        return 1;
    }
}

uint16_t CN_get_FIFO_space(CN_FIFO_struct *FIFO){
    return (FIFO->temp_len-CN_get_FIFO_occupied(FIFO));
}

uint16_t CN_get_FIFO_occupied(CN_FIFO_struct *FIFO){
    if(FIFO->head==FIFO->tail){
        if(FIFO->is_FIFO_full){
            return FIFO->temp_len;
        }
        else{
            return 0;
        }
    }
    else{
        if(FIFO->head>FIFO->tail){
            return (FIFO->head-FIFO->tail);
        }
        else{
            return (FIFO->temp_len-(FIFO->tail-FIFO->head));
        }        
    }
}
/***************************FIFO驱动***************************/

/******************************************************************数据链路层*********************************************************/
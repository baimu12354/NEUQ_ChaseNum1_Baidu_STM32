#include "TopSpeed_Include.h"

SM_Handle_struct SM_1_Handle={0};//Z轴
SM_Handle_struct SM_2_Handle={0};//X轴
SM_Handle_struct SM_3_Handle={0};

SM_SYSSTATUS_struct SM_1_STATUS_GET={0};
SM_SYSSTATUS_struct SM_2_STATUS_GET={0};
SM_SYSSTATUS_struct SM_3_STATUS_GET={0};

uint8_t SM_1_last_message_bytes[128]={0};
uint32_t SM_1_last_message_len=0;
uint32_t SM_1_RESEND_counter=0;

uint8_t SM_2_last_message_bytes[128]={0};
uint32_t SM_2_last_message_len=0;
uint32_t SM_2_RESEND_counter=0;

uint8_t SM_3_last_message_bytes[128]={0};
uint32_t SM_3_last_message_len=0;
uint32_t SM_3_RESEND_counter=0;

static uint32_t SM_1_COM_FAIL_COUNTER=0;
static uint32_t SM_2_COM_FAIL_COUNTER=0;
static uint32_t SM_3_COM_FAIL_COUNTER=0;

uint8_t SM_1_STATUS_bytes[31];
uint8_t SM_2_STATUS_bytes[31];
uint8_t SM_3_STATUS_bytes[31];


/*
定义SM_1为Z轴云台电机，SM_2为X轴云台电机，SM3为弹弓张紧电机

    在应用层，x轴以中点为零点，允许使用+-10000;z轴以最底端为零点，允许使用0-10000；弹弓以无张力为零点，张到最紧为10000
    在驱动层，x轴以最左端为零点，上电后先向左运动直到撞到限位，此时复位，最右端规定为15000脉冲
             z轴以最底端为零点，上电后，检查下侧限位是否被触发，此时复位，规定顶端为8000脉冲
             弹弓以最顶端为零点，上电后不带载向上运动到碰到限位，此时复位，
*/

/*
    错误处理：如果出现电机驱动的确认指令超时，自动重传，超过一定次数锁死模块并触发通信错误
        如果出现电机的到位指令超时，自动重传(排除接收到位指令错误)，超过一定次数触发硬件错误(无法闭环到指定位置)

*/

void TS_SM_init(void){
    TS_UART4_init();
    TS_UART5_init();
    TS_UART6_init();
    TS_SM_ENA();
    // TS_CHECK_SM();

    SM_1_Handle.SM_SELECT=SM_NO1;
    SM_1_Handle.SM_status=SM_NONE;
    SM_1_Handle.input_min=LIFT_Z_LOW;
    SM_1_Handle.input_max=LIFT_Z_HIGH;
    SM_1_Handle.output_min=SM_Z_LOW;
    SM_1_Handle.output_max=SM_Z_HIGH;
    SM_1_Handle.pos_now=0.0f;
    SM_1_Handle.pos_aim=0.0f;
    SM_1_Handle.timeout_counter=-1;
    SM_1_Handle.timeout_to_inpos=-1;

    SM_2_Handle.SM_SELECT=SM_NO2;
    SM_2_Handle.SM_status=SM_NONE;
    SM_2_Handle.input_min=LIFT_X_LEFT;
    SM_2_Handle.input_max=LEFT_X_RIGHT;
    SM_2_Handle.output_min=SM_X_LEFT;
    SM_2_Handle.output_max=SM_X_RIGHT;
    SM_2_Handle.pos_now=-2000.0f;
    SM_2_Handle.pos_aim=0.0f;
    SM_2_Handle.timeout_counter=-1;
    SM_2_Handle.timeout_to_inpos=-1;

    SM_3_Handle.SM_SELECT=SM_NO3;
    SM_3_Handle.SM_status=SM_NONE;
    SM_3_Handle.input_min=LIFT_Z_LOW;
    SM_3_Handle.input_max=LIFT_Z_HIGH;
    SM_3_Handle.output_min=SM_Z_LOW;
    SM_3_Handle.output_max=SM_Z_HIGH;
    SM_3_Handle.pos_now=0.0f;
    SM_3_Handle.pos_aim=0.0f;
    SM_3_Handle.timeout_counter=-1;
    SM_3_Handle.timeout_to_inpos=-1;


    TS_delay_ms(100);
    TS_SM_Set_PARA(&SM_2_Handle,1);
    TS_TEST_WAIT_TILL_SM2_READY();
    printf("SM_2 para set done\n");

    TS_SM_Set_PARA(&SM_1_Handle,1);
    TS_TEST_WAIT_TILL_SM1_READY();
    printf("SM_1 para set done\n");

    TS_SM_Set_PARA(&SM_3_Handle,1);
    TS_TEST_WAIT_TILL_SM3_READY();
    printf("SM_3 para set done\n");

    TS_delay_ms(100);

// TS_SM_operate(&SM_1_Handle,Action_method_absolute,10000);
}

//步进电机模块的接口，传入后检查取值、检查模块状态，如果成功，则设置模块状态并返回1，如果模块忙则返回0
uint8_t TS_SM_operate(SM_Handle_struct *H_SM,Action_method_ENUM OPE_SELECT,int data){
    switch(OPE_SELECT){
        case Action_method_absolute:{
            if(H_SM->SM_SELECT==SM_NO2){
                data=-data; //X轴的正方向为左侧
            }
            data=data>H_SM->input_max?H_SM->input_max:data;
            data=data<H_SM->input_min?H_SM->input_min:data;

            float timeout_to_inpos=2.0f*ABS((float)(H_SM->pos_now-data))/10000.0f;
            H_SM->timeout_to_inpos=timeout_to_inpos*1000; //转换为毫秒

            if(H_SM->SM_SELECT==SM_NO3){
                float timeout_to_inpos=5.0f*ABS((float)(H_SM->pos_now-data))/10000.0f;
                H_SM->timeout_to_inpos=timeout_to_inpos*1000; //转换为毫秒
            }
            TS_SM_Set_pos(H_SM,(int)TS_linear_map_float((float)data,H_SM->input_min,H_SM->input_max,H_SM->output_min,H_SM->output_max),300);
            H_SM->pos_aim=data;
            // if(TS_SM_Set_pos(H_SM,(int)TS_linear_map_float((float)data,H_SM->input_min,H_SM->input_max,H_SM->output_min,H_SM->output_max),300)==1){
            //     printf("set pos:%d\n",(int)TS_linear_map_float((float)data,H_SM->input_min,H_SM->input_max,H_SM->output_min,H_SM->output_max));
            //     H_SM->pos_aim=data;
            //     return 1;
            // }
            // else{
            //     return 0;
            // }             
            break;
        }
        case Action_method_incremental:{
            // printf("incr\n");
            if(H_SM->SM_SELECT==SM_NO2){
                data=-data; //X轴的正方向为左侧
            }
            float pos=data+H_SM->pos_now>H_SM->input_max?H_SM->input_max:data+H_SM->pos_now;
            pos=data+H_SM->pos_now<H_SM->input_min?H_SM->input_min:data+H_SM->pos_now;       
            float timeout_to_inpos=2.0f*ABS((float)data)/10000.0f;
            H_SM->timeout_to_inpos=timeout_to_inpos*1000; //转换为毫秒
            printf("posnow:%f\n",H_SM->pos_now);
            printf("incr %d\n",data);

            if(H_SM->SM_SELECT==SM_NO3){
                float timeout_to_inpos=7.0f*ABS((float)data)/10000.0f;
                H_SM->timeout_to_inpos=timeout_to_inpos*1000; //转换为毫秒
            }
            TS_SM_Set_pos(H_SM,(int)TS_linear_map_float((float)pos,H_SM->input_min,H_SM->input_max,H_SM->output_min,H_SM->output_max),300);
            H_SM->pos_aim=pos;
            // if(TS_SM_Set_pos(H_SM,(int)TS_linear_map_float((float)pos,H_SM->input_min,H_SM->input_max,H_SM->output_min,H_SM->output_max),300)==1){
            //     H_SM->pos_aim=pos;
            //     return 1;
            // }
            // else{
            //     return 0;
            // }       
            break;   
        }
    }
}

void TS_SM_ENA(void){
    TS_SM_Set_ENA(&SM_1_Handle,1);
    TS_SM_Set_ENA(&SM_2_Handle,1);
}

void TS_SM_DISA(void){
    TS_SM_Set_ENA(&SM_1_Handle,0);
    TS_SM_Set_ENA(&SM_2_Handle,0);
}

void TS_SM_inpos_poll(void){
    if(SM_1_Handle.timeout_to_inpos>0){
        SM_1_Handle.timeout_to_inpos--;
        if(SM_1_Handle.timeout_to_inpos==0){
            uint8_t response=CMDDONE_type_lift_z;
            TS_send_Response(Main_CMD_CMDDONE,&response);
            SM_1_Handle.timeout_to_inpos=-1;
            SM_1_Handle.SM_status=SM_NONE;
            SM_1_Handle.pos_now=SM_1_Handle.pos_aim;
            printf("z sm timeout inpos\n");
        }
    }

    if(SM_2_Handle.timeout_to_inpos>0){
        SM_2_Handle.timeout_to_inpos--;
        if(SM_2_Handle.timeout_to_inpos==0){
            uint8_t response=CMDDONE_type_lift_x;
            TS_send_Response(Main_CMD_CMDDONE,&response);
            SM_2_Handle.timeout_to_inpos=-1;
            SM_2_Handle.SM_status=SM_NONE;
            SM_2_Handle.pos_now=SM_2_Handle.pos_aim;
            printf("x sm timeout inpos\n");

        }
    }

    // if(SM_3_Handle.timeout_to_inpos>0){
    //     SM_3_Handle.timeout_to_inpos--;
    //     if(SM_3_Handle.timeout_to_inpos==0){
    //         TS_Shooter_inposition_callback();
    //         SM_3_Handle.timeout_to_inpos=-1;
    //         SM_3_Handle.SM_status=SM_NONE;
    //         SM_3_Handle.pos_now=SM_3_Handle.pos_aim;
    //         printf("shooter sm timeout inpos\n");

    //     }
    // }
}

static uint8_t Uplimit_Trig_Last=0;
static uint8_t Downlimit_Trig_Last=0;
static uint8_t Leftlimit_Trig_Last=0;
static uint8_t Rightlimit_Trig_Last=0;
static uint8_t Flag_Zcalibrated=0;
static uint8_t Flag_Xcalibrated=0;
void TS_SM_Trig_init(void){
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    uint8_t Uplimit_Trig_Last=(uint8_t)HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_5);
    uint8_t Downlimit_Trig_Last=(uint8_t)HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_4);
    uint8_t Leftlimit_Trig_Last=(uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9);
    uint8_t Rightlimit_Trig_Last=(uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7);

    if(!Downlimit_Trig_Last){
        Flag_Zcalibrated=1;
        SM_1_Handle.pos_now=LIFT_Z_LOW;
        Emm_V5_Reset_CurPos_To_Zero(SM_NO1,0x00);
        TS_delay_ms(200);
        TS_SM_operate(&SM_1_Handle,Action_method_absolute,2000);
    }

    if(!Leftlimit_Trig_Last ){
        Flag_Xcalibrated=1;
        SM_2_Handle.pos_now=LIFT_X_LEFT;
        Emm_V5_Reset_CurPos_To_Zero(SM_NO2,0x00);
        TS_delay_ms(100); 
        TS_SM_operate(&SM_2_Handle,Action_method_absolute,-2000);
    }

    if(!Flag_Zcalibrated){
        TS_SM_Set_spd(&SM_1_Handle,-10);
        while(1){
            if(!HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_4)){
                printf("hit downlimit\n");
                TS_SM_Set_spd(&SM_1_Handle,0);
                TS_delay_ms(100);
                SM_1_Handle.pos_now=LIFT_Z_LOW;
                Emm_V5_Reset_CurPos_To_Zero(SM_NO1,0x00);
                TS_delay_ms(100);
                Flag_Zcalibrated=1;
                TS_SM_operate(&SM_1_Handle,Action_method_absolute,2000);
                break;
            }
        }
    }

    if(!Flag_Xcalibrated){
        TS_SM_Set_spd(&SM_2_Handle,-10);
        while(1){
            if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9)){
                TS_SM_Set_spd(&SM_2_Handle,0);
                TS_delay_ms(100);
                SM_2_Handle.pos_now=LIFT_X_LEFT;
                Emm_V5_Reset_CurPos_To_Zero(SM_NO2,0x00);
                TS_delay_ms(100);
                Flag_Xcalibrated=1;
                TS_SM_operate(&SM_2_Handle,Action_method_absolute,-2000);
                break;
            }
        }
    }

    Uplimit_Trig_Last=(uint8_t)HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_5);
    Downlimit_Trig_Last=(uint8_t)HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_4);
    Leftlimit_Trig_Last=(uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9);
    Rightlimit_Trig_Last=(uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7);

    // TS_SM_operate(&SM_1_Handle,Action_method_absolute,0);
    // TS_SM_operate(&SM_2_Handle,Action_method_absolute,0);
}

void TS_SM_Trig_poll(void){
    uint8_t Uplimit_Trig=(uint8_t)HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_5);
    uint8_t Downlimit_Trig=(uint8_t)HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_4);
    uint8_t Leftlimit_Trig=(uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9);
    uint8_t Rightlimit_Trig=(uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7);

    // printf("%d,%d\n",Uplimit_Trig,Downlimit_Trig);

    if(Uplimit_Trig_Last && !Uplimit_Trig){
        TS_SM_Set_spd(&SM_1_Handle,0);
        TS_delay_ms(100);
        Emm_V5_Reset_CurPos_To_Zero(SM_NO2,0x00);
    }

}
/*************************************************驱动层*************************************************/
void TS_SM_SET_CMD(SM_NO_ENUM SM_SELECT,uint8_t *cmd,uint32_t cmd_len){
    // for(int i=0;i<cmd_len;i++){
    //     printf("send:%X\n",cmd[i]);
    // }
    if(SM_SELECT==SM_NO1){
        // memcpy(SM_1_last_message_bytes,cmd,cmd_len);
        // SM_1_last_message_len=cmd_len;
        // TS_Reset_SM_COM_FIAL_counter(SM_NO1,100);
        TS_UART_write_fifo(TS_UART_4,cmd,cmd_len);        
    }

    if(SM_SELECT==SM_NO2){
        // memcpy(SM_2_last_message_bytes,cmd,cmd_len);
        // SM_2_last_message_len=cmd_len;
        // TS_Reset_SM_COM_FIAL_counter(SM_NO2,100);
        TS_UART_write_fifo(TS_UART_5,cmd,cmd_len);        
    }

    if(SM_SELECT==SM_NO3){
        // memcpy(SM_2_last_message_bytes,cmd,cmd_len);
        // SM_2_last_message_len=cmd_len;
        // TS_Reset_SM_COM_FIAL_counter(SM_NO2,100);
        TS_UART_write_fifo(TS_UART_6,cmd,cmd_len);        
    }

}

void TS_CHECK_SM(void){
    TS_delay_ms(100);

    TS_SM_operate(&SM_1_Handle,Action_method_absolute,10000);
    TS_TEST_WAIT_TILL_SM1_READY();

    TS_SM_operate(&SM_1_Handle,Action_method_absolute,0);
    TS_TEST_WAIT_TILL_SM1_READY();

    TS_SM_operate(&SM_2_Handle,Action_method_absolute,10000);
    TS_TEST_WAIT_TILL_SM2_READY();

    TS_SM_operate(&SM_2_Handle,Action_method_absolute,0);
    TS_TEST_WAIT_TILL_SM2_READY();
 
    TS_SM_operate(&SM_2_Handle,Action_method_absolute,-10000);
    TS_TEST_WAIT_TILL_SM2_READY();

    TS_SM_operate(&SM_2_Handle,Action_method_absolute,0);
    TS_TEST_WAIT_TILL_SM2_READY();
}

void TS_SM_COM_RX_CALLBACK(SM_Handle_struct *H_SM,uint8_t dat){
//   printf("get:%X\n",dat);  
    static int SM_1_counter=0;
    static int SM_2_counter=0;
    static int SM_3_counter=0;
    int *counter_use=NULL;
    uint8_t *hello_load_use=NULL;

    switch(H_SM->SM_SELECT){
        case SM_NO1:{
            counter_use=&SM_1_counter;
            hello_load_use=SM_1_STATUS_bytes;
            break;
        }
        case SM_NO2:{
            counter_use=&SM_2_counter;
            hello_load_use=SM_2_STATUS_bytes;
            break;
        }
        case SM_NO3:{
            counter_use=&SM_3_counter;
            hello_load_use=SM_3_STATUS_bytes;
            break;
        }
    }

    switch(H_SM->SM_status){
        case SM_NONE:{
            return;
        }
        case SM_HELLO:{
            switch(*counter_use){
                case 0:{
                    if(dat==0x01){
                        hello_load_use[*counter_use]=dat;
                        (*counter_use)++;
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
                case 1:{
                    if(dat==0x00){
                        *counter_use=0;
                        //错误
                    }
                    else if(dat==0x43){
                        hello_load_use[*counter_use]=dat;
                        (*counter_use)++;
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
                case 2:{
                    if(dat==0x1F){
                        hello_load_use[*counter_use]=dat;
                        (*counter_use)++;
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
                case 3:{
                    if(dat==0x09){
                        hello_load_use[*counter_use]=dat;
                        (*counter_use)++;
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
                case 4:
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                case 10:
                case 11:
                case 12:
                case 13:
                case 14:
                case 15:
                case 16:
                case 17:
                case 18:
                case 19:
                case 20:
                case 21:
                case 22:
                case 23:
                case 24:
                case 25:
                case 26:
                case 27:
                case 28:
                case 29:{
                    hello_load_use[*counter_use]=dat;
                    (*counter_use)++;
                    break;
                }         
                case 30:{
                    if(dat==0x6B){
                        H_SM->SM_status=SM_NONE;
                        switch(H_SM->SM_SELECT){
                            case SM_NO1:{
                                TS_Parse_SM_1_STATUS();
                                break;
                            }
                            case SM_NO2:{
                                TS_Parse_SM_2_STATUS();
                                break;
                            }
                        }
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
            }
            break;
        }
        case SM_SET_POS:{
            switch(*counter_use){
                case 0:{
                    if(dat==0x01){
                        (*counter_use)++;
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
                case 1:{
                    if(dat==0xFD){
                        (*counter_use)++;
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
                case 2:{
                    if(dat==0x02){
                        (*counter_use)++;
                    }
                    else if(dat==0xE2){
                        printf("set pos fail\n");
                        (*counter_use)++;
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
                case 3:{
                    if(dat==0x6B){
                        *counter_use=0;
                        H_SM->SM_status=SM_WAITING_POS;
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
            }
            break;
        }
        case SM_ENA:{
            switch(*counter_use){
                case 0:{
                    if(dat==0x01){
                        (*counter_use)++;
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
                case 1:{
                    if(dat==0xF3){
                        (*counter_use)++;
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
                case 2:{
                    if(dat==0x02){
                        (*counter_use)++;
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
                case 3:{
                    if(dat==0x6B){
                        *counter_use=0;
                        H_SM->SM_status=SM_NONE;
                        printf("ena confirm\n");
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
            }
            break;
        }
        case SM_WAITING_POS:{
            switch(*counter_use){
                case 0:{
                    if(dat==0x01){
                        (*counter_use)++;
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
                case 1:{
                    if(dat==0xFD){
                        (*counter_use)++;
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
                case 2:{
                    if(dat==0x9F){
                        (*counter_use)++;
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
                case 3:{
                    if(dat==0x6B){
                        *counter_use=0;
                        H_SM->SM_status=SM_NONE;
                        TS_Action_in_position(H_SM);
                        printf("pos done\n");
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
            }
            break;
        }
        case SM_SET_PARA:{
            switch(*counter_use){
                case 0:{
                    if(dat==0x01){
                        (*counter_use)++;
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
                case 1:{
                    if(dat==0x48){
                        (*counter_use)++;
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
                case 2:{
                    if(dat==0x02){
                        (*counter_use)++;
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
                case 3:{
                    if(dat==0x6B){
                        *counter_use=0;
                        H_SM->SM_status=SM_NONE;
                        printf("clear ope\n");
                    }
                    else{
                        *counter_use=0;
                    }
                    break;
                }
                default:{
                    *counter_use=0;
                    break;
                }
            }
            break;
        }

    }
}

void TS_Parse_SM_1_STATUS(void){
    SM_1_STATUS_GET.SM_BUS_VOL=*(uint16_t*)(SM_1_STATUS_bytes+4);
    // printf("%d\n",SM_1_STATUS_GET.SM_BUS_VOL);
}

void TS_UART4_RX_CALLBACK(uint8_t dat){
    TS_SM_COM_RX_CALLBACK(&SM_1_Handle,dat);
    // if(SM_1_COM_OPE==SM_ENA){
    //     printf("get:%X\n",dat); 
    // }
    // printf("get:%X\n",dat); 
}

void TS_Parse_SM_2_STATUS(void){
    SM_2_STATUS_GET.SM_BUS_VOL=*(uint16_t*)(SM_2_STATUS_bytes+4);
    // printf("%d\n",SM_1_STATUS_GET.SM_BUS_VOL);
}
    
void TS_UART5_RX_CALLBACK(uint8_t dat){
    // printf("get:%X\n",dat); 
    TS_SM_COM_RX_CALLBACK(&SM_2_Handle,dat);
}

void TS_UART6_RX_CALLBACK(uint8_t dat){
    // printf("get:%X\n",dat); 
    TS_SM_COM_RX_CALLBACK(&SM_3_Handle,dat);
}

/************************************动作指令**********************************/
uint8_t TS_SM_Set_spd(SM_Handle_struct *H_SM,int spd){
    Emm_V5_Vel_Control(H_SM->SM_SELECT,0x00,spd>0,(uint16_t)(ABS(spd)),100,0);
    printf("setspd:%d\n",spd);
    // H_SM->SM_status=SM_SET_SPD;
}

uint8_t TS_SM_Set_pos(SM_Handle_struct *H_SM,int pos,uint32_t spd){
    Emm_V5_Pos_Control(H_SM->SM_SELECT,0x00,pos>0,spd,210,ABS(pos),1,0);
    H_SM->SM_status=SM_SET_POS;
    printf("abspos:%d\n",pos);
    // if(H_SM->SM_status==SM_NONE){
    //     H_SM->SM_status=SM_SET_POS;
    //     printf("pos:%d\n",pos);
        
    //     return 1;
    // }
    // else{
    //     printf("busy\n");
    //     return 0;
    // } 
}

uint8_t TS_SM_Set_pos_inc(SM_Handle_struct *H_SM,int pos,uint32_t spd){
    Emm_V5_Pos_Control(H_SM->SM_SELECT,0x00,pos>0,spd,210,ABS(pos),0,0);
    H_SM->SM_status=SM_SET_POS;
    printf("incpos:%d\n",pos);

    return 1;
    // if(H_SM->SM_status==SM_NONE){
    //     H_SM->SM_status=SM_SET_POS;
    //     printf("pos:%d\n",pos);
    //     Emm_V5_Pos_Control(H_SM->SM_SELECT,0x00,pos>0,spd,230,ABS(pos),0,0);
    //     return 1;
    // }
    // else{
    //     printf("busy\n");
    //     return 0;
    // } 
}

uint8_t TS_SM_Set_ZERO(SM_Handle_struct *H_SM,SM_NO_ENUM SM_SELECT){
    if(H_SM->SM_status==SM_NONE){
        H_SM->SM_status=SM_SET_ZERO;
        Emm_V5_Reset_CurPos_To_Zero(H_SM->SM_SELECT,0x00);
    }
    else{
        printf("busy\n");
    }
}

uint8_t TS_SM_Set_ENA(SM_Handle_struct *H_SM,uint8_t ena){
    if(H_SM->SM_status==SM_NONE){
        H_SM->SM_status=SM_ENA;
        Emm_V5_En_Control(H_SM->SM_SELECT,0x00,ena,0);
        return 1;
    }
    else{
        printf("busy\n");
        return 0;
    }
}

uint8_t TS_SM_Set_PARA(SM_Handle_struct *H_SM,uint8_t save){
    if(H_SM->SM_status==SM_NONE){
        H_SM->SM_status=SM_SET_PARA;
        TS_ZDT_SDK_Set_config(H_SM->SM_SELECT,0x00,save);
        return 1;
    }
    else{
        printf("busy\n");
        return 0;
    }
}

// uint8_t TS_SM_Read_Err(SM_NO_ENUM SM_SELECT){
//     switch(SM_SELECT){
//         case SM_NO1:{
//             if(SM_1_COM_OPE==SM_NONE){
//                 SM_1_COM_OPE=SM_SET_ZERO;
//                 Emm_V5_Reset_CurPos_To_Zero(SM_NO1,0x00);
//             }
//             else{
//                 printf("busy\n");
//             }
//             break;
//         }
//         case SM_NO2:{
//             SM_2_COM_OPE=SM_SET_POS;
//             break;
//         }
//     }
// }
/************************************动作指令**********************************/

/************************************通信超时重传**********************************/
void TS_Reset_SM_COM_FIAL_counter(SM_Handle_struct *H_SM,uint32_t timeout_ms){
    H_SM->timeout_counter=timeout_ms;
}

void TS_SM_timeout_countdown_1ms(void){
    if(SM_1_Handle.timeout_counter>0){
        SM_1_Handle.timeout_counter--;
    }
    if(SM_2_Handle.timeout_counter>0){
        SM_2_Handle.timeout_counter--;
    }
}

void TS_SM_timeout_polling(void){
    if(SM_1_Handle.SM_status!=SM_NONE && SM_1_Handle.timeout_counter==0){
        TS_SM_resend_message(&SM_1_Handle);
        printf("SM_COM_TIMEOUT:%d\n",++SM_1_RESEND_counter);
    }

    if(SM_2_Handle.SM_status!=SM_NONE && SM_2_Handle.timeout_counter==0){
        TS_SM_resend_message(&SM_2_Handle);
        printf("SM_COM_TIMEOUT:%d\n",++SM_2_RESEND_counter);
    }
}

void TS_SM_resend_message(SM_Handle_struct *H_SM){
    TS_Reset_SM_COM_FIAL_counter(H_SM,100);
    if(H_SM->SM_SELECT==SM_NO1){
        TS_UART_write_fifo(TS_UART_4,SM_1_last_message_bytes,SM_1_last_message_len);
    }
    else if(H_SM->SM_SELECT==SM_NO2){
        TS_UART_write_fifo(TS_UART_5,SM_2_last_message_bytes,SM_2_last_message_len);
    }
}
/************************************通信超时重传**********************************/

/*************************************************驱动层*************************************************/
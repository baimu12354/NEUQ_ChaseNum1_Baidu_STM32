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
            // 上位机握手请求
            case Main_CMD_HELLO:{
                printf("get CMD_HELLO\n");
                uint8_t response[2]={Main_CMD_INLINE,0x00};
                CN_send_Response(Main_CMD_REPLY,response);  

                Package_get.done=1;
                break;
            }

            // 上位机状态获取请求
            case Main_CMD_STATUS:{
                printf("get CMD_STATUS\n");
                uint8_t response[2]={Main_CMD_ASTATUS,SYS_STATUS.all};
                CN_send_Response(Main_CMD_ASTATUS,response);  

                Package_get.done=1;
                break;
            }

            // 上位机运动请求
            case Main_CMD_SETMOVEMENT:{
                printf("get CMD_SETM\n");
                CMD_SETM_load_struct CMD_SETM_load;
                CMD_SETM_load.Movement_type=Package_get.load[0];
                memcpy(&CMD_SETM_load.Movement,&Package_get.load[1],Package_get.load_len-1);
                // 设置速度
                if(CMD_SETM_load.Movement_type==Movement_Set_speed || CMD_SETM_load.Movement_type==Movement_Set_angvel){
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
                // 发起 平移 或者 旋转
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
                // 发起 精密平移 或者 精密旋转
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
            // 上位机参数获取请求（相对于此前上位机设定值、若上位机未设定则相对于车身初始值）
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

            // 开启
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

// 接收拼包
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

// 串口3接收回调
void CN_UART3_RX_CALLBACK(uint8_t dat){
    Main_COM_RX_CALLBACK(dat);
}
/******************************************************************应用层*********************************************************/

/******************************************************************数据链路层*********************************************************/

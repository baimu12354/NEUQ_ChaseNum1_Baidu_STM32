#include "TopSpeed_Include.h"

SHOOTER_Handle_struct SHOOTER_Handle={.Status=SHOOTER_Status_Err,
                                      .Flag_mag_ena=0,
                                      .Uplimit_trig=0,
                                      .Last_Uplimit_trig=0,
                                      .Force=0,
                                      .Flag_enable_hold=0,
                                      .Flag_send_cmdrecv=0,
                                      .Flag_send_cmddone=0};

void TS_Shooter_init(void){
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_5,GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    SHOOTER_Handle.Status=SHOOTER_Status_Idle;
}

void TS_Shooter_enable_mag(void){
    if(SHOOTER_Handle.Status!=SHOOTER_Status_Err){
        HAL_GPIO_WritePin(GPIOH,GPIO_PIN_5,GPIO_PIN_SET);
        SHOOTER_Handle.Flag_mag_ena=1;
    }
}

void TS_Shooter_disable_mag(void){
    if(SHOOTER_Handle.Status!=SHOOTER_Status_Err){
        HAL_GPIO_WritePin(GPIOH,GPIO_PIN_5,GPIO_PIN_RESET);
        SHOOTER_Handle.Flag_mag_ena=0;
    }
}

void TS_Shooter_uplimit_trig(void){
    TS_SM_Set_spd(&SM_3_Handle,0);
    SHOOTER_Handle.Status=SHOOTER_Status_Reset;
    SHOOTER_Handle.Flag_send_cmddone=1;

    // TS_Shooter_set_mission(7000);
}

//通常按键抖动在20ms左右，使用20ms以上的采样周期即可
void TS_Shooter_uplimit_trig_poll(void){
    SHOOTER_Handle.Uplimit_trig=(uint8_t)HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_6);
    // printf("uplimit trig:%d\n",SHOOTER_Handle.Uplimit_trig);

    if(SHOOTER_Handle.Uplimit_trig!=SHOOTER_Handle.Last_Uplimit_trig){
        if(SHOOTER_Handle.Uplimit_trig==0){
            TS_Shooter_uplimit_trig();
            printf("trig\n");
        }
    }

    SHOOTER_Handle.Last_Uplimit_trig=SHOOTER_Handle.Uplimit_trig;
}


//蓄力步进电机就位回调，自动发射
void TS_Shooter_inposition_callback(void){
    if(SHOOTER_Handle.Status==SHOOTER_Status_Holding){
        SHOOTER_Handle.Status=SHOOTER_Status_Ready;
        TS_Shooter_disable_mag();
        SHOOTER_Handle.Status=SHOOTER_Status_Released;
        SHOOTER_Handle.Flag_send_cmddone=1;
        SHOOTER_Handle.Status=SHOOTER_Status_Idle;
    }
}

/****************************以下是用户接口*****************************/
void TS_Shooter_Reset(void){
    SHOOTER_Handle.Flag_enable_hold=0;
    printf("try reset\n");
    if(SHOOTER_Handle.Uplimit_trig==0){
        TS_Shooter_uplimit_trig();
        printf("trig \n");
    }
    else{
        if(SHOOTER_Handle.Status==SHOOTER_Status_Idle){
            printf("shooter start reset\n");
            TS_SM_Set_spd(&SM_3_Handle,20);
            SHOOTER_Handle.Status=SHOOTER_Status_Resetting;
        }        
        else{
            printf("shooter is not idle\n");
        }
    }
}

void TS_Shooter_set_mission(int32_t force){
    if(SHOOTER_Handle.Status==SHOOTER_Status_Reset){
        SHOOTER_Handle.Force=force;
        SHOOTER_Handle.Flag_enable_hold=1;
    }
}

void TS_Shooter_mission_poll(void){
    if(SHOOTER_Handle.Status==SHOOTER_Status_Reset && SHOOTER_Handle.Flag_enable_hold==1){
        TS_Shooter_enable_mag();
        TS_delay_ms(500);

        if(TS_SM_Set_pos_inc(&SM_3_Handle,-SHOOTER_Handle.Force,50)==1){
            SM_3_Handle.timeout_to_inpos=7*SHOOTER_Handle.Force/10000;
            SM_3_Handle.pos_aim=SHOOTER_Handle.Force;
            SHOOTER_Handle.Status=SHOOTER_Status_Holding;
        }
        else{
            TS_Shooter_disable_mag();
            SHOOTER_Handle.Status=SHOOTER_Status_Err;
            uint8_t err_code=1;
            TS_send_Response(Main_CMD_ERR,&err_code);  
            printf("shooter err\n");
        }

        SHOOTER_Handle.Flag_enable_hold=0;
    }
}

void TS_Shooter_response_callback(void){
    if(SHOOTER_Handle.Flag_send_cmddone==1){
        SHOOTER_Handle.Flag_send_cmddone=0;
        uint8_t response=CMDDONE_type_shooter;
        TS_send_Response(Main_CMD_CMDDONE,&response);
    }
}
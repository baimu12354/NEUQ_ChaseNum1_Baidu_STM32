#include "TopSpeed_Include.h"

#define ACTION_LIFT_Z_SPD   (100)
#define ACTION_LIFT_X_SPD   (300)

void TS_set_Action(Action_CMD_struct Action,uint8_t action_check_get){
    printf("set act:%X\n",Action.Action_type);
    switch(Action.Action_type){
        case Action_Lift_Z:{
            //确保操作句柄模块处于空闲状态
            // if(SM_1_Handle.SM_status!=SM_NONE){
            //     uint8_t err_code=0;
            //     TS_send_Response(Main_CMD_ERR,&err_code);  
            // }
            // else{
            //     TS_SM_operate(&SM_1_Handle,Action.Action_method,Action.value);
            //     uint8_t response[2]={Main_CMD_SETACTION,action_check_get};
            //     TS_send_Response(Main_CMD_RECVCMD,response); 
            // }

            TS_SM_operate(&SM_1_Handle,Action.Action_method,Action.value);
            uint8_t response[2]={Main_CMD_SETACTION,action_check_get};
            TS_send_Response(Main_CMD_RECVCMD,response); 
            break;
        }
        case Action_Lift_X:{
            //确保操作句柄模块处于空闲状态
            // Action.value=-Action.value;
            // if(SM_2_Handle.SM_status!=SM_NONE){
            //     uint8_t err_code=0;
            //     TS_send_Response(Main_CMD_ERR,&err_code);  
            // }
            // else{
            //     printf("value:%f\n",Action.value);
            //     if(Action.Action_method==Action_method_absolute){
            //         Action.value=Action.value>10000?10000:Action.value;
            //         Action.value=Action.value<-10000?-10000:Action.value;
            //     }
            //     TS_SM_operate(&SM_2_Handle,Action.Action_method,Action.value);
            //     uint8_t response[2]={Main_CMD_SETACTION,action_check_get};
            //     TS_send_Response(Main_CMD_RECVCMD,response); 
            // }
            printf("value:%f\n",Action.value);
            if(Action.Action_method==Action_method_absolute){
                Action.value=Action.value>10000?10000:Action.value;
                Action.value=Action.value<-10000?-10000:Action.value;
            }
            TS_SM_operate(&SM_2_Handle,Action.Action_method,Action.value);
            uint8_t response[2]={Main_CMD_SETACTION,action_check_get};
            TS_send_Response(Main_CMD_RECVCMD,response); 
            break;
        }
        case Action_Gripper_yaw:{
            TS_SERVO_operate(&SERVO_Gripper_yaw_Handle,Action.Action_method,Action.value);
            printf("%f\n",Action.value);
                uint8_t response[2]={Main_CMD_SETACTION,action_check_get};
                TS_send_Response(Main_CMD_RECVCMD,response); 
            break;
        }
        case Action_Gripper_pitch:{
            // printf("%f\n",Action.value);

            if(Action.value>1.0f){
                TS_Change_to_Horizontal();
                // printf("Change_to_Horizontal\n");
            }
            else if(Action.value>0.0f){
                TS_Change_to_Teaching_1();
                // printf("Change_to_Teaching_1\n");
            }
            else if(Action.value>-1.0f){
                TS_Change_to_OverLook();
                // printf("Change_to_OverLook\n");
            }
            else if(Action.value>-2.0f){
                TS_Change_to_Grip();
            }
            else{
                TS_Change_to_Teaching_2();
                // printf("Change_to_Teaching_2\n");
            }
            // printf("%f\n",Action.value);
                uint8_t response[2]={Main_CMD_SETACTION,action_check_get};
                TS_send_Response(Main_CMD_RECVCMD,response); 
            break;
        }
        case Action_Gripper_grip:{
            if(Action.value>0){
                TS_Pump_Start();
            }
            else{
                TS_Pump_Stop();
            }
            uint8_t response[2]={Main_CMD_SETACTION,action_check_get};
            TS_send_Response(Main_CMD_RECVCMD,response); 
            break;
        }
        
        case Action_Shooter_Reset:{
            printf("Action_Shooter_Reset\n");
            TS_Shooter_Reset();
            uint8_t response[2]={Main_CMD_SETACTION,action_check_get};
            TS_send_Response(Main_CMD_RECVCMD,response); 

            break;
        }
        
        case Action_Shooter_shoot:{
            if(SHOOTER_Handle.Status==SHOOTER_Status_Reset){
                if(Action.value>10000){
                    Action.value=10000;
                }
                if(Action.value<0){
                    Action.value=0;
                }
                TS_Shooter_set_mission(Action.value);
                uint8_t response[2]={Main_CMD_SETACTION,action_check_get};
                TS_send_Response(Main_CMD_RECVCMD,response); 
            }
            else{
                TS_send_Response(Main_CMD_ERR,&SYS_STATUS.all);  
            }

            break;
        }
    
        case Action_Shooter_reload:{
            if(Action.value>0){
                TS_Servo_Shooter_reload_ON();
            }
            else{
                TS_Servo_Shooter_reload_OFF();
            }
            TS_Shooter_set_mission(Action.value);
            uint8_t response[2]={Main_CMD_SETACTION,action_check_get};
            break;
        }

        case Action_Flag_Show:{
            // printf("Action_Flag_Show:%f\n",Action.value);
            if(Action.value<-1.0f){
                Color_Now=RGB_COLOR_Blue;
            }
            else if(Action.value<0.0f){
                Color_Now=RGB_COLOR_Green;
            }
            else if(Action.value<1.0f){
                Color_Now=RGB_COLOR_Yellow;
            }
            else if(Action.value<2.0f){
                Color_Now=RGB_COLOR_Orange;
            }
            else{
                Color_Now=RGB_COLOR_NONE;
            }
            uint8_t response[2]={Main_CMD_SETACTION,action_check_get};
            TS_send_Response(Main_CMD_RECVCMD,response); 
            break;
        }
    }
}

void TS_Action_in_position(SM_Handle_struct *H_SM){
    H_SM->SM_status=SM_NONE;
    H_SM->pos_now=H_SM->pos_aim;
    if(H_SM->SM_SELECT==SM_NO1){
        uint8_t response=CMDDONE_type_lift_z;
        TS_send_Response(Main_CMD_CMDDONE,&response);
        H_SM->timeout_to_inpos=-1;
    }

    else if(H_SM->SM_SELECT==SM_NO2){
        uint8_t response=CMDDONE_type_lift_x;
        TS_send_Response(Main_CMD_CMDDONE,&response);
        H_SM->timeout_to_inpos=-1;
    }
    
    else if(H_SM->SM_SELECT==SM_NO3){
        TS_Shooter_inposition_callback();
        H_SM->timeout_to_inpos=-1;
    }
}

void TS_Action_init(void){
    TS_Servo_init();
    TS_SM_init();
    TS_SM_ENA();
    TS_Shooter_init();
    TS_Pump_init();
}

void TS_Action_Store(void){
    TS_TEST_WAIT_TILL_SM1_READY();
    TS_SM_operate(&SM_1_Handle,Action_method_absolute,5000);
    TS_TEST_WAIT_TILL_SM1_READY();

    TS_TEST_WAIT_TILL_SM2_READY();
    TS_SM_operate(&SM_2_Handle,Action_method_absolute,10000);
    TS_TEST_WAIT_TILL_SM2_READY();

    TS_Change_to_OverLook();
    TS_SET_Servo_Gripper_yaw(-90);

    TS_TEST_WAIT_TILL_SM1_READY();
    TS_SM_operate(&SM_1_Handle,Action_method_absolute,2000);
    TS_TEST_WAIT_TILL_SM1_READY();
}

void TS_Action_Reset(void){
    TS_delay_ms(1000);
    TS_SET_Servo_Gripper_yaw(90);
    TS_Servo_Shooter_reload_OFF();
    TS_Pump_Stop();
    TS_Shooter_disable_mag();
    TS_delay_ms(500);
}

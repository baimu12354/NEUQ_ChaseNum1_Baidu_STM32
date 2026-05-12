#include "ChaseNum1_Include.h"

void Menu_init(void){

}

int highlight_now=0;
uint8_t flag_menu_refresh=1;
uint8_t flag_server_started=0;
void Menu_show(uint8_t highlight){
    switch(highlight){
        case 0:{
            tft180_show_string(0,0,"Start Server   <-");
            tft180_show_string(0,20,"Close Server");
            tft180_show_string(0,40,"Start Mission");
            tft180_show_string(0,60,"Suspend Mission");
            break;
        }
        case 1:{
            tft180_show_string(0,0,"Start Server    ");
            tft180_show_string(0,20,"Close Server   <-");
            tft180_show_string(0,40,"Start Mission");
            tft180_show_string(0,60,"Suspend Mission");
            break;
        }
        case 2:{
            tft180_show_string(0,0,"Start Server    ");
            tft180_show_string(0,20,"Close Server    ");
            tft180_show_string(0,40,"Start Mission   <-");
            tft180_show_string(0,60,"Suspend Mission");
            break;
        }
        case 3:{
            tft180_show_string(0,0,"Start Server    ");
            tft180_show_string(0,20,"Close Server    ");
            tft180_show_string(0,40,"Start Mission   ");
            tft180_show_string(0,60,"Suspend Mission <-");
            break;
        }
    }

    if(flag_server_started==1){
        tft180_show_string(0,100,"Server Started");
    }
    else{
        tft180_show_string(0,100,"Server Not Started");
    }
}

void Menu_up_event(void){
    printf("Menu up event\n");
    highlight_now--;
    if(highlight_now<0){
        highlight_now=3;
    }
    flag_menu_refresh=1;
}

void Menu_down_event(void){
    printf("Menu down event\n");
    highlight_now++;
    if(highlight_now>3){
        highlight_now=0;
    }
    flag_menu_refresh=1;
}

void Menu_press_event(void){
    printf("Menu press event\n");
    flag_menu_refresh=1;
    switch(highlight_now){
        case 0:{
            printf("Start Server\n");
            // CN_send_Response(Main_CMD_STARTSERVER,NULL);
            flag_server_started=0 ; //重置服务器启动标志
            // CN_Server_init();
            break;
        }
        case 1:{
            printf("Close Server\n");
            // CN_send_Response(Main_CMD_CLOSESERVER,NULL);
            // flag_server_started=0; //重置服务器启动标志
            // CN_Server_close();
            break;
        }
        case 2:{
            printf("Start Mission\n");
            // CN_send_Response(Main_CMD_STARTMISSION,NULL);
            // CN_Mission_start();
            break;
        }
        case 3:{
            printf("Suspend Mission\n");
            // CN_send_Response(Main_CMD_SUSPENDMISSION,NULL);
            // CN_Mission_suspend();
            break;
        }
    }
}

void Menu_poll(void){
    if(flag_menu_refresh==1){
        flag_menu_refresh=0;
        tft180_clear();
        Menu_show(highlight_now);
    }
}


/****************************按键驱动*****************************/
Knob_struct Knob={0};
Key_struct In_Key={0};
Key_struct Out_Key={0};

void Key_init(void){
    GPIO_InitTypeDef GPIO_Initure={0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_Initure.Pin=GPIO_PIN_5;
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入模式
    GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);

    GPIO_Initure.Pin=GPIO_PIN_1;
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);

    GPIO_Initure.Pin=GPIO_PIN_12;
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);

    GPIO_Initure.Pin=GPIO_PIN_2;
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);

    Knob.up_event=Menu_up_event;
    Knob.down_event=Menu_down_event;
    In_Key.press_event=Menu_press_event;
    Out_Key.press_event=Menu_press_event;

    In_Key.key_last_status=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1);
    Out_Key.key_last_status=HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_12);

}

void Key_poll(void){
    uint8_t Knob_A_level=HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_2);
    uint8_t Knob_B_level=HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_12);

    /*
        当A相为上升沿时，A为高电平，B高电平触发上升事件，低电平触发下降事件
               下降沿时，A为低电平，B高电平触发下降事件，低电平触发上升事件
        其实就是软件轮询模拟编码器，在布尔代数上看就是A相有边沿时输出AB电平的同或值
    */
    if(Knob_A_level!=Knob.Knob_A_Last_status){
        Knob.Knob_A_Last_status=Knob_A_level;
        if(!(Knob_A_level^Knob_B_level)){
            if(Knob.last_turn==1){
                Knob.down_event();
                Knob.last_turn=0;
            }
            else{
                Knob.last_turn=1;
            }
        }
        else{
            if(Knob.last_turn==2){
                Knob.up_event();
                Knob.last_turn=0;
            }
            else{
                Knob.last_turn=2;
            }
        }
    }


    uint8_t Key_in_level=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1);
    uint8_t Key_out_level=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);

    if(Key_in_level==In_Key.key_last_status && Key_in_level==0){
        if(In_Key.output_ena==1){
            In_Key.press_event();
            In_Key.output_ena=0;
        }
    }
    else{
        In_Key.output_ena=1;
        In_Key.key_last_status=Key_in_level;
    }

    if(Key_out_level==Out_Key.key_last_status && Key_out_level==1){
        if(Out_Key.output_ena==1){
            Out_Key.press_event();
            Out_Key.output_ena=0;
        }
    }
    else{
        Out_Key.output_ena=1;
        Out_Key.key_last_status=Key_in_level;
    }

}



/****************************按键驱动*****************************/
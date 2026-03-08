// #include "TopSpeed_Include.h"

// void RC_init(void){
//     GPIO_InitTypeDef GPIO_Initure={0};

//     __HAL_RCC_GPIOI_CLK_ENABLE();
//     __HAL_RCC_SYSCFG_CLK_ENABLE();

//     GPIO_Initure.Pin=GPIO_PIN_6;//|GPIO_PIN_7;
// 	GPIO_Initure.Mode=GPIO_MODE_IT_RISING_FALLING;      //复用模式
// 	GPIO_Initure.Pull=GPIO_NOPULL;              //上拉
// 	GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //高速
// 	HAL_GPIO_Init(GPIOI,&GPIO_Initure);

//     SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI6;
//     SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PI;

//     SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI7;
//     SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PI;

//     HAL_NVIC_SetPriority(EXTI9_5_IRQn,0,0);
//     HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
// }

// void EXTI9_5_IRQHandler(void){
//     if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6)){
//         __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
//         TS_PI6_EXTI_CALLBACK();
//     }
//     if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7)){
//         __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
//         // TS_PI7_EXTI_CALLBACK();
//     }
// }

// void TS_Debug_UART_RX_CALLBACK(uint8_t dat){
//     static int counter=0;
//     static Debug_Command_ENUM Command;
//     static float data=0;

//     switch(counter){
//         case 0:{
//             if(dat==0xFF){
//                 counter++;
//             }
//             else{
//                 counter=0;
//             }
//             break;
//         }

//         case 1:{
//             if(dat==0xF1){
//                 Command=Debug_Command_Set_Spd;
//                 counter++;
//             }
//             else if(dat==0xF2){
//                 Command=Debug_Command_Set_Pos;
//                 counter++;
//             }
//             else{
//                 counter=0;
//             }
//             break;
//         }

//         case 2:
//         case 3:
//         case 4:
//         case 5:{
//             *(((uint8_t*)&data)+(counter-2))=dat;
//             counter++;
//             break;
//         }

//         case 6:{
//             if(dat==0xAA){
//                 counter++;
//             }
//             else{
//                 counter=0;
//             }
//             break;
//         }
//         case 7:{
//             if(dat==0xAA){
//                 if(Command==Debug_Command_Set_Spd){
//                     Vel_aim.M_FL_Vel=data;
//                     // printf("%f\n",Vel_aim.M_FL_Vel);
//                 }
//                 else if(Command==Debug_Command_Set_Pos){
//                     Pos_aim.M_FL_Pos=data;
//                 }
//                 counter=0;
//             }
//             else{
//                 counter=0;
//             }
//             break;
//         }
//     }
// }
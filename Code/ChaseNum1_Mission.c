#include "ChaseNum1_Include.h"

#define CHANNEL1 1000
#define CHANNEL2 1000

#define RC_RUNNING 192
#define RC_MOVING 992
#define RC_BREAKING 1792

void RC_control(void){
     
    uint16_t  ch1_value = (uint16_t)uart_receiver.channel[0];
	 uint16_t  ch2_value = (uint16_t)uart_receiver.channel[1];	
    uint16_t  ch4_value = (uint16_t)uart_receiver.channel[3];

    float velx = 0;
    float vely = 0;
    float turn = 0;

    // 循迹控制
    if  (ch4_value == RC_RUNNING){
       velx = 0;
       vely = (float)(ch2_value - CHANNEL1) / 20;
       turn = (float)(ch1_value - CHANNEL1) / 10;
    }
    // 平移控制
    else if (ch4_value == RC_MOVING){
       turn = 0;
       vely = (float)(ch2_value - CHANNEL1) / 20;
       velx = (float)(ch1_value - CHANNEL1) / 20;
    }
    
    Car_Movement_struct TMovement = {
                       .Car_X_movement = 0,
                       .Car_Y_movement = 0,
                       .Car_angle = 0, 

                       .Car_X_vel = velx,      // 车身的横向速度
                       .Car_Y_vel = vely,      // 车身的纵向速度
                       .Car_angvel = turn      // 车身的角速度
               };
		            
    if (ch4_value == RC_BREAKING){
       CN_set_movement(TMovement,Movement_Breaking); 
    } 
    else {
       CN_set_movement(TMovement,Movement_Running); 
    }
}
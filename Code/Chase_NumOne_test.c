#include "Chase_NumOne_test.h"
#include "TopSpeed_Typedef.h"
#include "TopSpeed_include.h"
#include <stdio.h> 

extern Motor_vel_struct Vel_now;
extern Motor_vel_struct Vel_aim;
extern Motor_duty_struct Duty_now;
extern Motor_pos_struct Pos_now;
extern Motor_pos_struct Pos_aim;
extern float YAW_now;
extern float YAW_aim;

void CNO_Chassis_closedloop_test(ClosedLoop_Type loopType){
   
    switch (loopType)
    {
    case speed_loop:
    {
    Car_Movement_struct my_speed = {0};
    my_speed.Car_speed = 20.0f;  // 设置为 500 (根据你PID参数 PID_OUTMAX=500，这应该是全速)
    //发送指令
    TS_set_movement(my_speed, Movement_Set_speed, 0);
    break;
    }

    case position_loop:
    {
	Car_Movement_struct my_pos = {0};
    my_pos.Car_X_movement = 0.f;
    my_pos.Car_Y_movement = 10000.f; // Y轴前进
    TS_set_movement(my_pos, Movement_Translation, 0);
    break;
    }

    case angle_loop:
    {
	//test：逆时针旋转 90 度
    Car_Movement_struct my_rot = {0};
    my_rot.Car_Z_rotation = 90.0f; // 设定目标角度
    TS_set_movement(my_rot, Movement_Rotation, 0);
    break;
    }
    printf("Test Started! Press Reset to Stop.\n");
    
   
}
}
void CNO_closedloop_show(ClosedLoop_Type loopType){
            if(loopType == speed_loop)
        {
            // 监视速度环：目标速度 vs 当前速度 vs PWM占空比
            // 注意：Duty_now 通常是 float，为了打印方便可以强转 int，或者用 %.0f
            printf("Vel Aim:%.1f | Now:%.1f | Duty:%d\r\n", 
                   Vel_aim.M_FR_Vel,   // 打印右前轮（即 PA9/PC9 那一路）
                   Vel_now.M_FR_Vel, 
                   (int)Duty_now.M_FR_Duty);
        }
        else if(loopType == position_loop)
        {
            // 监视位置环：目标位置 vs 当前位置
            printf("Pos Aim:%.0f | Now:%.0f | Err:%.0f\r\n", 
                   Pos_aim.M_FR_Pos, 
                   Pos_now.M_FR_Pos,
                   Pos_aim.M_FR_Pos - Pos_now.M_FR_Pos);
        }
        else if(loopType == angle_loop)
        {
            // 监视角度环：目标角度 vs 当前角度
            printf("Yaw Aim:%.2f | Now:%.2f | Err:%.2f\r\n", 
                   YAW_aim, 
                   YAW_now,
                   YAW_aim - YAW_now);
        }


}
void CNO_GPIOC9_Low(void){
    
__HAL_RCC_GPIOC_CLK_ENABLE();           
GPIO_InitTypeDef GPIO_Safety = {0};
GPIO_Safety.Pin = GPIO_PIN_9;
GPIO_Safety.Mode = GPIO_MODE_OUTPUT_PP; // 推挽输出强力拉低
GPIO_Safety.Pull = GPIO_PULLDOWN;       
GPIO_Safety.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOC, &GPIO_Safety);         
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

}
		// extern Motor_vel_struct Vel_now;
        // extern Motor_vel_struct Vel_aim;
        // extern Motor_duty_struct Duty_now;
		// printf("T:%.2f,N:%.2f,PWM:%f\n", 
        //        Vel_aim.M_FL_Vel, 
        //        Vel_now.M_FL_Vel, 
        //        Duty_now.M_FL_Duty);

		// extern Motor_pos_struct Pos_now;
        // extern Motor_pos_struct Pos_aim;
        
        // // 打印左前轮的进度
        // printf("Pos T:%.0f, Now:%.0f\n", Pos_aim.M_FL_Pos, Pos_now.M_FL_Pos);

		// extern float YAW_now;
        // extern float YAW_aim;
        // printf("Yaw T:%.2f, Now:%.2f\n", YAW_aim, YAW_now);
        
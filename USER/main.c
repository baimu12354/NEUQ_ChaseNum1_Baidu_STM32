#include "sys.h"
#include "delay.h"
#include "usart.h" 
#include "../Code/ChaseNum1_Include.h"

extern UART_HandleTypeDef UART1_Handle;
extern UART_HandleTypeDef UART4_Handle;
extern UART_HandleTypeDef UART5_Handle;

extern TIM_HandleTypeDef TIM13_PIT_Handle;

extern volatile uint32_t systime_us;

extern uint8_t mission_check;

extern Motor_duty_struct Duty_now;

uint8_t FLAG_MOVECTRL=0;
uint8_t FLAG_CHECK_MOVE_DONE=0;
uint8_t FLAG_CHECK_SM_COM_FAIL=0;


SYS_STATUS_STRUCT SYS_STATUS={.all=0};

int main(void){
	Cache_Enable();                     // 打开L1-Cache
	HAL_Init();				            // 初始化HAL库
	Stm32_Clock_Init(160,5,2,8);        // 设置时钟,系统时钟为400Mhz(特定外设时钟为100Mhz)
	delay_init(400);

	 CN_PIT_3_init_us(10,0);       // PIT3初始化 服务于系统延时（10us）
	 CN_UART1_init();          // debug调试串口
	 Main_COM_init();	      // 上位机通信串口
     printf("Debug Init\n");
	 CN_delay_ms(10);
	 
	 CN_move_init();                 // 底盘运动初始化
	 printf("Move Init\n");
	 CN_delay_ms(10);

	 CN_Motor_init();                // 电机初始化
	 CN_TIM_Motor_PWMOUT_ENA();        // 启用电机PWM输出
	 printf("Motor Init\n");
	 CN_delay_ms(10);

	 CN_3rd_Party_JY61_IMU_init();	 // 初始化陀螺仪
	 printf("Imu Init\n");
	 CN_delay_ms(10);

     CN_TIM2345_ENC_init();		     // 编码器
	 printf("Encoder Init\n");
	 CN_delay_ms(10);

	 uart_receiver_init();
	 printf("RC Init\n");
	 CN_delay_ms(10);

	 CN_PIT_1_init_ms(1,1);          // PIT1初始化 服务于底盘运动控制和运动完成检测（1ms）
	 CN_PIT_2_init_ms(1,2);          // PIT2初始化 服务于机械臂解算等外设

     Car_Movement_struct TMovement = {
                       .Car_X_movement = 0,
                       .Car_Y_movement = 0,
                       .Car_angle = 90, 

                       .Car_X_vel = 0,      // 车身的横向速度
                       .Car_Y_vel = 0,      // 车身的纵向速度
                       .Car_angvel = 10     // 车身的角速度
                }; 

     CN_set_movement(TMovement,Movement_Rotation);


	while(1){

		RC_control();
		//printf("%.5f,%.5f\n",Car_anv_PID_ctrler.err,Car_yaw_PID_ctrler.err);
		printf("%.2f,%.2f,%.2f,%.2f\n",Duty_now.M_FL_Duty,Duty_now.M_FR_Duty,Duty_now.M_BL_Duty,Duty_now.M_BR_Duty);
	    // printf("%d,%d,%d,%d,%d,%d\n",
		// 	(uint16_t)uart_receiver.channel[0],
		// 	(uint16_t)uart_receiver.channel[1],
		// 	(uint16_t)uart_receiver.channel[2],
		// 	(uint16_t)uart_receiver.channel[3],
		// 	(uint16_t)uart_receiver.channel[4],
		// 	(uint16_t)uart_receiver.channel[5]);
		CN_delay_ms(100);	
	}
}

int avg_change=0;
void CN_PIT1_CALLBACK(void){
	CN_Movement_ctrl_CALLBACK(1);
}

void CN_PIT2_CALLBACK(void){
    static int ms_counter=0;	    
    ms_counter++; 
    if(ms_counter > 1000000) ms_counter = 0;    //防止溢出
	
	// 每5ms读取一次编码器增量
	if(ms_counter%5==0){
		CN_read_ENC_INC();
	}
}

void CN_PI6_EXTI_CALLBACK(void){

}

void CN_PI7_EXTI_CALLBACK(void){
	
}
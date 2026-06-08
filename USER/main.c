#include "sys.h"
#include "delay.h"
#include "usart.h" 
#include "../Code/ChaseNum1_Include.h"

uint8_t PrintFlag = 0;

int main(void){

	Cache_Enable();                     // 打开L1-Cache
	HAL_Init();				            // 初始化HAL库
	Stm32_Clock_Init(160,5,2,8);        // 设置时钟,系统时钟为400Mhz(特定外设时钟为100Mhz)
	delay_init(400);
    
	// 电机初始化(避免电机初始化之前疯转)
	CN_Motor_init();
	CN_TIM_Motor_PWMOUT_ENA();     // 启用电机PWM输出
	CN_stop_Motor_Duty();

	// PIT3初始化 服务于系统延时（10us）
	CN_PIT_3_init_us(10,0);
	// Debug调试初始化··			
	Debug_init();
	printf("Motor Init\n");
	CN_delay_ms(10);
	printf("Debug Init\n");
	CN_delay_ms(10);
	// Nano 通信初始化
	Main_COM_init();
	printf("Communication Init\n");
	CN_delay_ms(10);
	// 编码器初始化
    CN_TIM2345_ENC_init();
	printf("Encoder Init\n");
	CN_delay_ms(10);
	// 底盘运动初始化
	CN_move_init();
	printf("Move Init\n");
	CN_delay_ms(10);
	// 机械臂初始化
	Arm_Init();
    printf("Arm Init\n");
	CN_delay_ms(10);
    // 陀螺仪初始化
	CN_3rd_Party_JY61_IMU_init();
	printf("Imu Init\n");
	CN_delay_ms(10);
    //气泵初始化（默认不启动气泵）
	CN_Pump_init();
	// 屏幕初始化	
    Menu_init();
    printf("Screen Init\n");
    CN_delay_ms(10);
	//PIT1 服务于底盘运动控制和 NANO 协议计时，周期 1ms。
	CN_PIT_1_init_ms(1,1);
	CN_PIT_2_init_ms(1,2);
	Nano_COM_Notify_Init_Done();

	while(1) {

		Nano_COM_Task_Loop();

		if (PrintFlag == 1){
			//	CN_UART_write_fifo(CN_UART_2,"hello",5);
			//    printf("%.3f\n",YAW_now);
			//    printf("%d,%d,%d,%d\n",
			//        ENC_data[3],ENC_data[1],ENC_data[2],ENC_data[0]);
            Menu_poll();
			PrintFlag = 0;
		}
	}
}

void CN_PIT1_CALLBACK(void){
   CN_Movement_ctrl_CALLBACK(1);
}

// PIT2 定时器回调 （1ms）
int PIT2_ms_counter=0;
void CN_PIT2_CALLBACK(void){
	PIT2_ms_counter++;
    if(PIT2_ms_counter > 1000000) PIT2_ms_counter = 0;    //防止溢出
	// 每1ms读取一次编码器增量
	if (PIT2_ms_counter%1==0){
		CN_read_ENC_INC();
	}
	// 每20ms打印一次数据
	if (PIT2_ms_counter%100==0){
        PrintFlag = 1;
	}
	Nano_COM_1ms_Tick();
}

void CN_PI6_EXTI_CALLBACK(void){

}

void CN_PI7_EXTI_CALLBACK(void){
	
}

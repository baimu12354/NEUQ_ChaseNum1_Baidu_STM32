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
	Cache_Enable();                             //打开L1-Cache
	HAL_Init();				                    //初始化HAL库
	Stm32_Clock_Init(160,5,2,8);                //设置时钟,系统时钟为400Mhz(特定外设时钟为100Mhz)
	delay_init(400);

   	// TS_Motor_init();
	
	// TS_UART1_init();     // 调试通信uart1
	// Main_COM_init();	 //上位机通信
	// TS_PIT_3_init_us(10,0);

	// TS_move_init();
	// TS_TIM_Motor_PWMOUT_ENA();

	// TS_set_Motor_Duty();

	// TS_3rd_Party_JY61_IMU_init();		//初始化陀螺仪
	// printf("JY61 init\n");

    // TS_TIM2345_ENC_init();		

	// TS_PIT_1_init_ms(1,3);
    // SYS_STATUS.bits.SYS_INITED=1;



	while(1){
		// Main_COM_Response_CALLBACK();//处理通信指令
		// HAL_Delay(500);
	}
}

int avg_change=0;
void TS_PIT1_CALLBACK(void){
	static int ms_counter=0;	    
    ms_counter++; 
    if(ms_counter > 1000000) ms_counter = 0;

	if(ms_counter%5==0){
		TS_read_ENC_INC();
	}

	if(SYS_STATUS.bits.SYS_INITED){
		if(ms_counter%5==0){
			static uint32_t us_time=0;
			TS_Movement_ctrl_CALLBACK((uint32_t)(systime_us-us_time));
			us_time=systime_us;
		}
		if(ms_counter%5==3){
			TS_Movement_done_check_CALLBACK();
		}
	}
}

void TS_PIT2_CALLBACK(void){
	
}

void TS_PI6_EXTI_CALLBACK(void){

}

void TS_PI7_EXTI_CALLBACK(void){
	
}
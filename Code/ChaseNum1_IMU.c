#include "ChaseNum1_Include.h"
#include <math.h>

float YAW_now=0;//单位为deg，记录相对角度
float yaw_bias=0;//上位机指定角度后，需要用这个偏移值来同步逻辑角度和物理角度
float YAW_aim=0;

/*************************************调用wit-JY61接口***********************************/
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;

IMU_Data_struct IMU_data={0};

void CN_3rd_Party_JY61_IMU_UART_write_func(uint8_t *pData, uint32_t len){
    CN_UART_send_data_IT(CN_UART_2,pData,len);
}

void CN_3rd_Party_JY61_IMU_init(void){
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);//选择串口协议
    WitSerialWriteRegister(CN_3rd_Party_JY61_IMU_UART_write_func);//注册串口向传感器写函数
    WitRegisterCallBack(CN_3rd_Party_JY61_IMU_update_CALLBACK);//注册寄存器更新回调
    WitDelayMsRegister(CN_delay_ms);//注册延时函数，必须先调用TS_PIT_3_init_us

    while(1){
        if(CN_auto_scan_JY61()==1){//在所有波特率上搜索传感器
            break;
        }
    }

    WitWriteReg(KEY,0xB588);//打开寄存器写锁定
    CN_delay_ms(50);
    WitWriteReg(RRATE,RRATE_200HZ);//设置输出速率为200hz
    CN_delay_ms(50);
    WitWriteReg(BAUD,WIT_BAUD_115200);//设置传感器接收波特率

    CN_delay_ms(50);
    CN_UART_deinit(CN_UART_2);
    CN_UART2_init(115200);//重新初始化串口，配置为相同的波特率

    // WitWriteReg(RSW,0x6);//只输出加速度、角速度
    WitWriteReg(KEY,0xB588);
    CN_delay_ms(50);
    WitWriteReg(AXIS6,0x01);//关闭磁力计，使用纯惯性解算
    CN_delay_ms(50);
    WitWriteReg(RSW,RSW_ANGLE);//只使用角度输出 | RSW_GYRO
    CN_delay_ms(50);
    WitWriteReg(CALSW,CALANGLEZ);//只使用角度输出
}

uint8_t CN_auto_scan_JY61(void){
    uint32_t bauds[7]={4800,9600,19200,38400,57600,115200,230400};

    for(int i =0;i<7;i++){
        CN_UART_deinit(CN_UART_2);
        CN_UART2_init(bauds[i]);
        WitReadReg(AX, 3);
        CN_delay_ms(100);
        if(s_cDataUpdate == 0){
            printf("baud %d fail\n",bauds[i]);
        }
        else{
            printf("baud %d success\n",bauds[i]);
            return 1;
        }
    }
    return 0;
}
    
void CN_UART2_RX_CALLBACK(uint8_t dat){
    // printf("%c\n",dat);
    WitSerialDataIn(dat);
}

void CN_3rd_Party_JY61_IMU_update_CALLBACK(uint32_t uiReg, uint32_t uiRegNum){
    // printf("callback\n");
    // printf("%d\n",uiReg);
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
            case AX:
                IMU_data.A_X=sReg[AX]*0.0000498246;
                s_cDataUpdate |= ACC_UPDATE;
                break;
            case AY:
                IMU_data.A_Y=sReg[AY]*0.0000498246;
                s_cDataUpdate |= ACC_UPDATE;
                break;
            case AZ:
                IMU_data.A_Z=sReg[AZ]*0.0000498246;
                s_cDataUpdate |= ACC_UPDATE;
                break;
            case GX:
                IMU_data.G_X=sReg[GX]*0.001065264436;
                s_cDataUpdate |= GYRO_UPDATE;

                // printf("%f\n",IMU_data.G_X);
                break;
            case GY:
                IMU_data.G_Y=sReg[GY]*0.001065264436;
                s_cDataUpdate |= GYRO_UPDATE;
                break;
            case GZ:
                IMU_data.G_Z=sReg[GZ]*0.001065264436;   // rad/s
                s_cDataUpdate |= GYRO_UPDATE;
                break;      
                
            case Yaw:
                s_cDataUpdate |= READ_UPDATE;   
                //printf("%f\n",sReg[Yaw]*180.0f/32760);
                YAW_now=-sReg[Yaw]*180.0f/32760;
            default:
				s_cDataUpdate |= READ_UPDATE;
			    break;
        }
		uiReg++;
    }

   // printf("%.3f\n",YAW_now);
}
/*************************************调用wit-JY61接口***********************************/

// 用于上位机设置逻辑角度
void CN_Set_YAW(float yaw){
    while(yaw>180){
        yaw-=360;
    }
    while(yaw<-180){
        yaw+=360;
    }

    yaw_bias=yaw-YAW_now;

    while(yaw_bias>180){
        yaw_bias-=360;
    }
    while(yaw_bias<-180){
        yaw_bias+=360;
    }
}

// 用于上位机获取逻辑角度
float CN_get_logical_yaw(void){
    float logical_yaw=YAW_now+yaw_bias;
    while(logical_yaw>180){
        logical_yaw-=360;
    }
    while(logical_yaw<-180){
        logical_yaw+=360;
    }
    return logical_yaw;
}
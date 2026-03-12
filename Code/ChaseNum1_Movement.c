#include "ChaseNum1_Include.h"
#include <math.h>

uint32_t movement_done_counter=0;

PID_ctrler Car_yaw_PID_ctrler={0};
PID_ctrler Car_yaw_moving_PID_ctrler={0};

float YAW_now=0;//单位为deg，记录相对角度
float yaw_bias=0;//上位机指定角度后，需要用这个偏移值来同步逻辑角度和物理角度
float YAW_aim=0;

static uint8_t cmd_check=0;


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

void CN_3rd_Party_JY61_IMU_update_CALLBACK(uint32_t uiReg, uint32_t uiRegNum);

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
                IMU_data.G_Z=sReg[GZ]*0.001065264436;
                s_cDataUpdate |= GYRO_UPDATE;
                break;      
                
            case Yaw:
                s_cDataUpdate |= READ_UPDATE;   
                // printf("%f\n",sReg[Yaw]*180.0f/32760);
                YAW_now=sReg[Yaw]*180.0f/32760;
            default:
				s_cDataUpdate |= READ_UPDATE;
			    break;
        }
		uiReg++;
    }

    //printf("%f\n",IMU_data.G_X);
}
/*************************************调用wit-JY61接口***********************************/

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

void CN_move_init(void){
    CN_Motor_pos_CL_init();//初始化电机位置环
    CN_CAR_yaw_CL_init();//初始化车身角度环
    CN_CAR_pos_CL_init();//初始化车身位置环
}

#define M_TO_ENC_FACTOR (1.0f)
void CN_set_movement(Car_Movement_struct movement,Movement_type_ENUM movement_type,uint8_t cmd_check_get){
    Motor_pos_struct Motor_pos_to_set={0};
    cmd_check=cmd_check_get;

    if(movement_type==Movement_Translation || movement_type==Movement_Translation_Precise){
        printf("set movement: translation,X:%f,Y:%f\n",movement.Car_X_movement,movement.Car_Y_movement);
        // Motor_pos_to_set.M_FL_Pos=(movement.Car_Y_movement+movement.Car_X_movement)*M_TO_ENC_FACTOR;
        // Motor_pos_to_set.M_FR_Pos=(movement.Car_Y_movement-movement.Car_X_movement)*M_TO_ENC_FACTOR;
        // Motor_pos_to_set.M_BL_Pos=(movement.Car_Y_movement-movement.Car_X_movement)*M_TO_ENC_FACTOR;
        // Motor_pos_to_set.M_BR_Pos=(movement.Car_Y_movement+movement.Car_X_movement)*M_TO_ENC_FACTOR;
        // TS_Motor_pos_CL_set_aim(Motor_pos_to_set);
        //printf("set movement: rotation:%f\n",movement.Car_Z_rotation);
        //printf("set movement: speed:%f\n",movement.Car_speed);
        CN_CAR_pos_CL_set_aim(movement);
        SYS_STATUS.bits.SYS_MOVING=1;
        SYS_STATUS.bits.SYS_INLINE=0;
        SYS_STATUS.bits.SYS_ROTATING=0;
    }
    else if(movement_type==Movement_Rotation || movement_type==Movement_Rotation_Precise){
        printf("set movement: rotation:%f\n",movement.Car_Z_rotation);
        CN_CAR_yaw_CL_set_aim(movement.Car_Z_rotation);
        SYS_STATUS.bits.SYS_MOVING=0;
        SYS_STATUS.bits.SYS_INLINE=0;
        SYS_STATUS.bits.SYS_ROTATING=1;
    }
    else if(movement_type==Movement_Set_speed){
        printf("set movement: speed:%f\n",movement.Car_speed);
        SYS_STATUS.bits.SYS_INLINE=1;
        SYS_STATUS.bits.SYS_MOVING=0;
        SYS_STATUS.bits.SYS_ROTATING=0;
        CN_CAR_vel_CL_set_aim(movement);
    }
    else{
        printf("unknow type:%x\n",movement_type);
    }
}

void CN_Movement_ctrl_CALLBACK(uint32_t us_delta_time){
    if(SYS_STATUS.bits.SYS_INLINE==1){
        CN_Motor_vel_CL_CALLBACK(us_delta_time);
    }
    else{
        if(SYS_STATUS.bits.SYS_MOVING==1){
            CN_CAR_pos_CL_CALLBACK(us_delta_time);
        }
        else if(SYS_STATUS.bits.SYS_ROTATING==1){
            CN_CAR_yaw_CL_CALLBACK(us_delta_time);
        }
    }
}

void CN_Intr_Movement(void){
    uint32_t CH1_8[8]={0};
    CN_TIM_Motor_PWM_duty_set(CH1_8);
    SYS_STATUS.bits.SYS_ACTING=0;
    SYS_STATUS.bits.SYS_INLINE=0;
    SYS_STATUS.bits.SYS_PUMPACTIVE=0;
    SYS_STATUS.bits.SYS_ROTATING=0;
    SYS_STATUS.bits.SYS_MOVING=0;
}

#define TRANSLATION_TOLERANCE_PRECISE   (50)
#define ROTATION_TOLERANCE_PRECISE      (2)
#define CONFIRM_TIME_PRECISE            (50)//*5ms

#define TRANSLATION_TOLERANCE   (150)
#define ROTATION_TOLERANCE      (2)
#define CONFIRM_TIME            (50)//*5ms

float Translation_Tolerance=TRANSLATION_TOLERANCE;
float Rotation_Tolerance=ROTATION_TOLERANCE;
float Confirm_Time=CONFIRM_TIME;

uint8_t Movement_Precise_Mode=0;

void CN_Movement_Change_To_Precise_Mode(void){
    Movement_Precise_Mode=1;
    Translation_Tolerance=TRANSLATION_TOLERANCE_PRECISE;
    Rotation_Tolerance=ROTATION_TOLERANCE_PRECISE;
    Confirm_Time=CONFIRM_TIME_PRECISE;

    printf("change to precise mode\n");
}

void CN_Movement_Change_To_Normal_Mode(void){
    Movement_Precise_Mode=0;
    Translation_Tolerance=TRANSLATION_TOLERANCE;
    Rotation_Tolerance=ROTATION_TOLERANCE;
    Confirm_Time=CONFIRM_TIME;

    printf("change to normal mode\n");
}

#define Pos_Err_LPF_LEN (20)
int M_FL_Pos_Err_LPF[Pos_Err_LPF_LEN]={0};
int M_FR_Pos_Err_LPF[Pos_Err_LPF_LEN]={0};
int M_BL_Pos_Err_LPF[Pos_Err_LPF_LEN]={0};
int M_BR_Pos_Err_LPF[Pos_Err_LPF_LEN]={0};
uint32_t Pos_Err_LPF_Cnt=0;
void CN_Movement_done_check_CALLBACK(void){
    M_FL_Pos_Err_LPF[Pos_Err_LPF_Cnt]=Pos_now.M_FL_Pos-Pos_aim.M_FL_Pos;
    M_FR_Pos_Err_LPF[Pos_Err_LPF_Cnt]=Pos_now.M_FR_Pos-Pos_aim.M_FR_Pos;
    M_BL_Pos_Err_LPF[Pos_Err_LPF_Cnt]=Pos_now.M_BL_Pos-Pos_aim.M_BL_Pos;
    M_BR_Pos_Err_LPF[Pos_Err_LPF_Cnt]=Pos_now.M_BR_Pos-Pos_aim.M_BR_Pos;
    Pos_Err_LPF_Cnt++;
    if(Pos_Err_LPF_Cnt>=Pos_Err_LPF_LEN)Pos_Err_LPF_Cnt=0;
    int M_FL_Pos_Err_LPF_Avg=0;
    int M_FR_Pos_Err_LPF_Avg=0;   
    int M_BL_Pos_Err_LPF_Avg=0;
    int M_BR_Pos_Err_LPF_Avg=0;
    for(int i=0;i<Pos_Err_LPF_LEN;i++){
        M_FL_Pos_Err_LPF_Avg+=M_FL_Pos_Err_LPF[i];
        M_FR_Pos_Err_LPF_Avg+=M_FR_Pos_Err_LPF[i];
        M_BL_Pos_Err_LPF_Avg+=M_BL_Pos_Err_LPF[i];
        M_BR_Pos_Err_LPF_Avg+=M_BR_Pos_Err_LPF[i];
    }
    M_FL_Pos_Err_LPF_Avg/=Pos_Err_LPF_LEN;
    M_FR_Pos_Err_LPF_Avg/=Pos_Err_LPF_LEN;
    M_BL_Pos_Err_LPF_Avg/=Pos_Err_LPF_LEN;
    M_BR_Pos_Err_LPF_Avg/=Pos_Err_LPF_LEN;
    //printf("M_FL_Pos_Err_LPF_Avg:%d,M_FR_Pos_Err_LPF_Avg:%d,M_BL_Pos_Err_LPF_Avg:%d,M_BR_Pos_Err_LPF_Avg:%d\n",
    //       M_FL_Pos_Err_LPF_Avg,M_FR_Pos_Err_LPF_Avg,M_BL_Pos_Err_LPF_Avg,M_BR_Pos_Err_LPF_Avg);
    if(SYS_STATUS.bits.SYS_MOVING==1){
        if(abs(M_FL_Pos_Err_LPF_Avg)<Translation_Tolerance &&
        abs(M_FR_Pos_Err_LPF_Avg)<Translation_Tolerance &&
        abs(M_BL_Pos_Err_LPF_Avg)<Translation_Tolerance &&
        abs(M_BR_Pos_Err_LPF_Avg)<Translation_Tolerance) {
            movement_done_counter++;
        }else{
            movement_done_counter=0;
        }
    }
    else if(SYS_STATUS.bits.SYS_ROTATING==1){
        if(fabsf(YAW_aim-YAW_now)<Rotation_Tolerance){
            movement_done_counter++;
        }
        else{
            movement_done_counter=0;
        }
    }
    else{
        movement_done_counter=0;
    }

    if(movement_done_counter>=Confirm_Time && (SYS_STATUS.bits.SYS_MOVING==1 || SYS_STATUS.bits.SYS_ROTATING==1)){
        CN_Motor_outline();
        if(SYS_STATUS.bits.SYS_MOVING==1){
            SYS_STATUS.bits.SYS_MOVING=0;
            uint8_t response=CMDDONE_type_moving;
            CN_send_Response(Main_CMD_CMDDONE,&response);
        }
        if(SYS_STATUS.bits.SYS_ROTATING==1){
            SYS_STATUS.bits.SYS_ROTATING=0;
            uint8_t response=CMDDONE_type_rotating;
            CN_send_Response(Main_CMD_CMDDONE,&response);
        }
        movement_done_counter=0;
    }
}

#define CAR_YAW_CL_PID_KP (1.0f)
#define CAR_YAW_CL_PID_KI (0)
#define CAR_YAW_CL_PID_KD (0)
#define CAR_YAW_CL_PID_IMAX (0)
#define CAR_YAW_CL_PID_OUTMAX (10)
/****************************机身角度环***************************/
void CN_CAR_yaw_CL_init(void){//初始化电机位置环PID控制器
    init_PID_ctrler(&Car_yaw_PID_ctrler,CAR_YAW_CL_PID_KP,CAR_YAW_CL_PID_KI,CAR_YAW_CL_PID_KD,CAR_YAW_CL_PID_IMAX,CAR_YAW_CL_PID_OUTMAX);
}

void CN_CAR_yaw_CL_CALLBACK(uint32_t us_time){
    float YAW_err=CN_get_relative_angle(YAW_now,YAW_aim);

    YAW_err=fabs(YAW_err)<1?0:YAW_err;

    PID_ctrler_update(&Car_yaw_PID_ctrler,YAW_err);
    //更新位置环输出

    float Rotation_spd=Car_yaw_PID_ctrler.output;

    Vel_aim.M_FL_Vel=Rotation_spd;
    Vel_aim.M_FR_Vel=-Rotation_spd;
    Vel_aim.M_BL_Vel=Rotation_spd;
    Vel_aim.M_BR_Vel=-Rotation_spd;
    //将位置环输出作为转速环输入

    // printf("%.2f,%.2f\n",YAW_now,YAW_aim);

    CN_Motor_vel_CL_CALLBACK(us_time);//调用速度环更新
}

float CN_get_relative_angle(float now,float aim){//获取两个绝对方位角之间的相对方位角
    float difference=aim-now;
    if(difference<-180)return difference+360;
    if(difference>180)return difference-360;
    return difference;
}

float CN_yaw_add(float now,float aim){//获取两个绝对方位角之间的相对方位角
    float res=now+aim;
    if(res<-180)return res+360;
    if(res>180)return res-360;
    return res;
}

void CN_CAR_yaw_CL_set_aim(float yaw){//设置位置环目标(一次性)
    YAW_aim=CN_yaw_add(YAW_now,yaw);//设置目标位置
}
/****************************机身角度环***************************/

Car_Movement_struct C_vel={0};//机身速度环目标
/****************************机身速度环***************************/
void CN_CAR_vel_CL_init(void){//初始化电机位置环PID控制器
    CN_Motor_vel_CL_init();
}

void CN_CAR_vel_CL_CALLBACK(uint32_t us_time){
    Motor_vel_struct M_vel={0};//行进速度分量

    M_vel.M_FL_Vel=C_vel.Car_speed-C_vel.Car_angvel;
    M_vel.M_FR_Vel=C_vel.Car_speed+C_vel.Car_angvel;
    M_vel.M_BL_Vel=C_vel.Car_speed-C_vel.Car_angvel;
    M_vel.M_BR_Vel=C_vel.Car_speed+C_vel.Car_angvel;

    CN_Motor_vel_CL_set_aim(M_vel);

    CN_Motor_vel_CL_CALLBACK(us_time);//调用速度环更新
}


void CN_CAR_vel_CL_set_aim(Car_Movement_struct C_vel_aim){//设置位置环目标(一次性)
    Motor_vel_struct M_vel={0};//行进速度分量

    M_vel.M_FL_Vel=C_vel_aim.Car_speed-C_vel_aim.Car_angvel;
    M_vel.M_FR_Vel=C_vel_aim.Car_speed+C_vel_aim.Car_angvel;
    M_vel.M_BL_Vel=C_vel_aim.Car_speed-C_vel_aim.Car_angvel;
    M_vel.M_BR_Vel=C_vel_aim.Car_speed+C_vel_aim.Car_angvel;

    CN_Motor_vel_CL_set_aim(M_vel);
}

/****************************机身速度环***************************/

#define CAR_YAW_MOVING_CL_PID_KP (0.7f)
#define CAR_YAW_MOVING_CL_PID_KI (0)//*200hz=2
#define CAR_YAW_MOVING_CL_PID_KD (2.0f)
#define CAR_YAW_MOVING_CL_PID_IMAX (2)
#define CAR_YAW_MOVING_CL_PID_OUTMAX (5)

float initial_yaw=0;//记录初始角度
Car_Movement_struct C_pos={0};//机身位置环目标
/****************************机身位置环***************************/
void CN_CAR_pos_CL_init(void){//初始化电机位置环PID控制器
    init_PID_ctrler(&Car_yaw_moving_PID_ctrler,CAR_YAW_MOVING_CL_PID_KP,CAR_YAW_MOVING_CL_PID_KI,CAR_YAW_MOVING_CL_PID_KD,CAR_YAW_MOVING_CL_PID_IMAX,CAR_YAW_MOVING_CL_PID_OUTMAX);
}

void CN_CAR_pos_CL_set_aim(Car_Movement_struct C_pos_aim){//设置位置环目标(一次性)
    initial_yaw=YAW_now;     //记录初始角度
    C_pos=C_pos_aim;         //设置目标位移
    Motor_pos_struct Motor_pos_to_set={0};

    PID_ctrler_reset(&Car_yaw_moving_PID_ctrler);//重置角度环PID控制器

    Motor_pos_to_set.M_FL_Pos=(C_pos_aim.Car_Y_movement+C_pos_aim.Car_X_movement)*M_TO_ENC_FACTOR;
    Motor_pos_to_set.M_FR_Pos=(C_pos_aim.Car_Y_movement-C_pos_aim.Car_X_movement)*M_TO_ENC_FACTOR;
    Motor_pos_to_set.M_BL_Pos=(C_pos_aim.Car_Y_movement-C_pos_aim.Car_X_movement)*M_TO_ENC_FACTOR;
    Motor_pos_to_set.M_BR_Pos=(C_pos_aim.Car_Y_movement+C_pos_aim.Car_X_movement)*M_TO_ENC_FACTOR;
    CN_Motor_pos_CL_set_aim(Motor_pos_to_set);

}

void CN_CAR_pos_CL_CALLBACK(uint32_t us_time){
    float YAW_err=CN_get_relative_angle(YAW_now,initial_yaw);
    PID_ctrler_update(&Car_yaw_moving_PID_ctrler,YAW_err);
    //更新角度环输出
    float Rotation_spd=Car_yaw_moving_PID_ctrler.output;
    Rotation_spd=0;

    Pos_now.M_FL_Pos+=Vel_now.M_FL_Vel;
    Pos_now.M_FR_Pos+=Vel_now.M_FR_Vel;
    Pos_now.M_BL_Pos+=Vel_now.M_BL_Vel;
    Pos_now.M_BR_Pos+=Vel_now.M_BR_Vel;
    //对编码器速度积分得到位置(无损失)

    float FL_Pos_err=Pos_aim.M_FL_Pos-Pos_now.M_FL_Pos;
    float FR_Pos_err=Pos_aim.M_FR_Pos-Pos_now.M_FR_Pos;
    float BL_Pos_err=Pos_aim.M_BL_Pos-Pos_now.M_BL_Pos;
    float BR_Pos_err=Pos_aim.M_BR_Pos-Pos_now.M_BR_Pos;

    FL_Pos_err=fabs(FL_Pos_err)<30?0:FL_Pos_err;
    FR_Pos_err=fabs(FR_Pos_err)<30?0:FR_Pos_err;
    BL_Pos_err=fabs(BL_Pos_err)<30?0:BL_Pos_err;
    BR_Pos_err=fabs(BR_Pos_err)<30?0:BR_Pos_err;

    PID_ctrler_update_with_time(&FL_pos_PID_ctrler,FL_Pos_err,us_time);
    PID_ctrler_update_with_time(&FR_pos_PID_ctrler,FR_Pos_err,us_time);
    PID_ctrler_update_with_time(&BL_pos_PID_ctrler,BL_Pos_err,us_time);
    PID_ctrler_update_with_time(&BR_pos_PID_ctrler,BR_Pos_err,us_time);
    //更新位置环输出

    Vel_aim.M_FL_Vel=FL_pos_PID_ctrler.output+Rotation_spd;
    Vel_aim.M_FR_Vel=FR_pos_PID_ctrler.output-Rotation_spd;
    Vel_aim.M_BL_Vel=BL_pos_PID_ctrler.output+Rotation_spd;
    Vel_aim.M_BR_Vel=BR_pos_PID_ctrler.output-Rotation_spd;
    //将位置环输出作为转速环输入

    // printf("%.2f,%.2f,%.2f,%.2f\n",Pos_now.M_FL_Pos,Pos_now.M_FR_Pos,Pos_now.M_BL_Pos,Pos_now.M_BR_Pos);

    CN_Motor_vel_CL_CALLBACK(us_time);//调用速度环更新
}
/****************************机身位置环***************************/
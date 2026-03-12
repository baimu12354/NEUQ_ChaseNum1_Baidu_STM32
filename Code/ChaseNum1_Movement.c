#include "ChaseNum1_Include.h"
#include <math.h>

uint32_t movement_done_counter=0;

PID_ctrler Car_yaw_PID_ctrler={0};
PID_ctrler Car_yaw_moving_PID_ctrler={0};

static uint8_t cmd_check=0;

/****************************运动功能函数***************************/
// 运动容忍度设置
float Translation_Tolerance=TRANSLATION_TOLERANCE;    
float Rotation_Tolerance=ROTATION_TOLERANCE;
float Confirm_Time=CONFIRM_TIME;

// 运动模式 0：为普通模式 1：为精确模式
uint8_t Movement_Precise_Mode=0;

// 位置误差低通滤波数组
uint32_t Pos_Err_LPF_Cnt=0;
int M_FL_Pos_Err_LPF[Pos_Err_LPF_LEN]={0};
int M_FR_Pos_Err_LPF[Pos_Err_LPF_LEN]={0};
int M_BL_Pos_Err_LPF[Pos_Err_LPF_LEN]={0};
int M_BR_Pos_Err_LPF[Pos_Err_LPF_LEN]={0};

// 运动初始化
void CN_move_init(void){
    CN_Motor_pos_CL_init();//初始化电机位置环
    CN_CAR_yaw_CL_init();//初始化车身角度环
    CN_CAR_pos_CL_init();//初始化车身位置环
}

// 设置运动模式和目标
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
    else if(movement_type==Movement_Set_speed || movement_type==Movement_Set_angvel){
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

// 运动停止
void CN_Intr_Movement(void){
    uint32_t CH1_8[8]={0};
    CN_TIM_Motor_PWM_duty_set(CH1_8);
    SYS_STATUS.bits.SYS_ACTING=0;
    SYS_STATUS.bits.SYS_INLINE=0;
    SYS_STATUS.bits.SYS_PUMPACTIVE=0;
    SYS_STATUS.bits.SYS_ROTATING=0;
    SYS_STATUS.bits.SYS_MOVING=0;
}

// 运动精确模式设置
void CN_Movement_Change_To_Precise_Mode(void){
    Movement_Precise_Mode=1;
    Translation_Tolerance=TRANSLATION_TOLERANCE_PRECISE;
    Rotation_Tolerance=ROTATION_TOLERANCE_PRECISE;
    Confirm_Time=CONFIRM_TIME_PRECISE;

    printf("change to precise mode\n");
}

// 运动普通模式设置
void CN_Movement_Change_To_Normal_Mode(void){
    Movement_Precise_Mode=0;
    Translation_Tolerance=TRANSLATION_TOLERANCE;
    Rotation_Tolerance=ROTATION_TOLERANCE;
    Confirm_Time=CONFIRM_TIME;

    printf("change to normal mode\n");
}

// 运动回调函数
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

// 运动完成检查回调函数（用于容忍度判断）
void CN_Movement_done_check_CALLBACK(void){
    if(SYS_STATUS.bits.SYS_MOVING==1){
        if(distance_to_go < Translation_Tolerance) {
            movement_done_counter++;
        } else {
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
/****************************运动功能函数***************************/


/****************************车身角度环***************************/
#define CAR_YAW_CL_PID_KP (1.0f)
#define CAR_YAW_CL_PID_KI (0)
#define CAR_YAW_CL_PID_KD (0)
#define CAR_YAW_CL_PID_IMAX (0)
#define CAR_YAW_CL_PID_OUTMAX (10)

// 车身角度环初始化
void CN_CAR_yaw_CL_init(void){
    init_PID_ctrler(&Car_yaw_PID_ctrler,CAR_YAW_CL_PID_KP,CAR_YAW_CL_PID_KI,CAR_YAW_CL_PID_KD,CAR_YAW_CL_PID_IMAX,CAR_YAW_CL_PID_OUTMAX);
}

// 车身角度环回调函数
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

// 获取两个绝对方位角之间的相对方位角
float CN_get_relative_angle(float now,float aim){
    float difference=aim-now;
    if(difference<-180)return difference+360;
    if(difference>180)return difference-360;
    return difference;
}

// 获取目标逻辑角度
float CN_yaw_add(float now,float aim){
    float res=now+aim;
    if(res<-180)return res+360;
    if(res>180)return res-360;
    return res;
}

// 设置目标角度
void CN_CAR_yaw_CL_set_aim(float yaw){//设置位置环目标(一次性)
    YAW_aim=CN_yaw_add(YAW_now,yaw);//设置目标位置
}
/****************************车身角度环***************************/


/****************************车身速度环***************************/
// 初始化电机位置环PID控制器
void CN_CAR_vel_CL_init(void){
    CN_Motor_vel_CL_init();  // 电机速度环初始化
    CN_CAR_yaw_CL_init();    // 车身角度环初始化（车身速度需要调用角度环以实现转速分配）
}

// 设置速度环目标(一次性)
void CN_CAR_vel_CL_set_aim(Car_Movement_struct C_vel_aim){
    Motor_vel_struct M_vel={0};//行进速度分量

    M_vel.M_FL_Vel=C_vel_aim.Car_speed-C_vel_aim.Car_angvel;
    M_vel.M_FR_Vel=C_vel_aim.Car_speed+C_vel_aim.Car_angvel;
    M_vel.M_BL_Vel=C_vel_aim.Car_speed-C_vel_aim.Car_angvel;
    M_vel.M_BR_Vel=C_vel_aim.Car_speed+C_vel_aim.Car_angvel;

    CN_Motor_vel_CL_set_aim(M_vel);
}
/****************************车身速度环***************************/

/**************************** 改进版车身位置环 ***************************/
#define CAR_YAW_MOVING_CL_PID_KP (0.7f)
#define CAR_YAW_MOVING_CL_PID_KI (0)//*200hz=2
#define CAR_YAW_MOVING_CL_PID_KD (2.0f)
#define CAR_YAW_MOVING_CL_PID_IMAX (2)
#define CAR_YAW_MOVING_CL_PID_OUTMAX (5)

// 运动参数配置
#define MAX_CAR_SPEED 50.0f     // 最大合速度 (单位/s)
#define MIN_CAR_SPEED 5.0f      // 最小有效速度
#define ACCEL_RATE 2.0f         // 加速度系数

#define TEST_SPEED 10.0f        // 测试速度

typedef struct{
    float x;
    float y;
}Pos_struct;

float initial_yaw = 0;    // 初始角度
float distance_to_go = 0;  // 剩余距离
Pos_struct target_pose = {0}; // 目标位置
Pos_struct start_pose = {0};  // 起始位置

// 初始化电机位置环PID控制器
void CN_CAR_pos_CL_init(void){
    init_PID_ctrler(&Car_yaw_moving_PID_ctrler,CAR_YAW_MOVING_CL_PID_KP,CAR_YAW_MOVING_CL_PID_KI,CAR_YAW_MOVING_CL_PID_KD,CAR_YAW_MOVING_CL_PID_IMAX,CAR_YAW_MOVING_CL_PID_OUTMAX);
}

// 设置位置目标
void CN_CAR_pos_CL_set_aim(Car_Movement_struct C_pos_aim) {
    // 设置角度目标
    initial_yaw=YAW_now;    // 记录初始角度
    PID_ctrler_reset(&Car_yaw_moving_PID_ctrler);
   
    // 设置位置目标
    start_pose.x = 0;  // 以当前位置为原点
    start_pose.y = 0;
    target_pose.x = C_pos_aim.Car_X_movement;
    target_pose.y = C_pos_aim.Car_Y_movement;
    distance_to_go = sqrtf(C_pos_aim.Car_X_movement * C_pos_aim.Car_X_movement + C_pos_aim.Car_Y_movement * C_pos_aim.Car_Y_movement);

    PID_ctrler_reset(&Car_yaw_moving_PID_ctrler);
}

// 位置环核心回调函数
void CN_CAR_pos_CL_CALLBACK(uint32_t us_time) {
    
    // 计算当前位置
    float cur_x, cur_y;
    cur_x = ( (Pos_now.M_FL_Pos - Pos_now.M_FR_Pos - Pos_now.M_BL_Pos + Pos_now.M_BR_Pos) / 4.0f) / M_TO_ENC_FACTOR;
    cur_y = ( (Pos_now.M_FL_Pos + Pos_now.M_FR_Pos + Pos_now.M_BL_Pos + Pos_now.M_BR_Pos) / 4.0f) / M_TO_ENC_FACTOR;

    // 计算残余误差向量
    float err_x = target_pose.x - cur_x;
    float err_y = target_pose.y - cur_y;
    distance_to_go = sqrtf(err_x * err_x + err_y * err_y);

    // 速度规划：计算合速度 V_total
    // 使用P控制器计算期望速度，并限幅
    //float v_target_total = distance_to_go * 5.0f; // 5.0f是位置增益
    float v_target_total = TEST_SPEED; // 固定测试速度
    // if (v_target_total > MAX_CAR_SPEED) v_target_total = MAX_CAR_SPEED;
    // if (v_target_total < MIN_CAR_SPEED && distance_to_go > 0.01f) v_target_total = MIN_CAR_SPEED;
   
    // Vx 和 Vy 的比例始终由目标向量决定
    float move_angle = atan2f(err_y, err_x); 
    float Vx = v_target_total * cosf(move_angle);
    float Vy = v_target_total * sinf(move_angle);

    // 姿态修正 (Yaw)
    float yaw_err = CN_get_relative_angle(YAW_now, initial_yaw);
    yaw_err=fabs(yaw_err)<1?0:yaw_err;    // 到达死区
    PID_ctrler_update(&Car_yaw_moving_PID_ctrler, yaw_err);
    float v_theta = Car_yaw_moving_PID_ctrler.output; 

    // 计算速度
    Vel_aim.M_FL_Vel = (Vy + Vx + v_theta) * M_TO_ENC_FACTOR;
    Vel_aim.M_FR_Vel = (Vy - Vx - v_theta) * M_TO_ENC_FACTOR;
    Vel_aim.M_BL_Vel = (Vy - Vx + v_theta) * M_TO_ENC_FACTOR;
    Vel_aim.M_BR_Vel = (Vy + Vx - v_theta) * M_TO_ENC_FACTOR;

    // 更新位置积分（仅用于内部反馈）
    Pos_now.M_FL_Pos += Vel_now.M_FL_Vel;
    Pos_now.M_FR_Pos += Vel_now.M_FR_Vel;
    Pos_now.M_BL_Pos += Vel_now.M_BL_Vel;
    Pos_now.M_BR_Pos += Vel_now.M_BR_Vel;

    // 调用速度环
    CN_Motor_vel_CL_CALLBACK(us_time);
}



/****************************机身位置环***************************/


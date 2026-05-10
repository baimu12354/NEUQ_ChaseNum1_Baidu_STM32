#include "ChaseNum1_Include.h"
#include <math.h>

Car_Movement_struct Whole_Movement = {0};       // 全局运动信息
Movement_type_ENUM Whole_Movement_Type = 0;       // 全局运动状态

PID_ctrler Car_anv_PID_ctrler={0};
PID_ctrler Car_yaw_PID_ctrler={0};
PID_ctrler Car_yaw_moving_PID_ctrler={0};

/****************************运动功能函数***************************/
// 运动初始化
void CN_move_init(void){
    CN_CAR_vel_CL_init();  // 车身速度环初始化
    CN_CAR_yaw_CL_init();  // 车身角度环初始化
    CN_CAR_anv_CL_init();  // 车身角速度环初始化
    CN_CAR_pos_CL_init();  // 车身位置环初始化
}

// 设置目标和运动模式
void CN_set_movement(Car_Movement_struct movement,Movement_type_ENUM movement_type){
    if (Whole_Movement_Type != movement_type) {
        // PID控制器清空
        PID_ctrler_reset(&Car_yaw_moving_PID_ctrler);
        PID_ctrler_reset(&Car_anv_PID_ctrler);
        PID_ctrler_reset(&Car_yaw_PID_ctrler);
        PID_ctrler_reset(&FL_pos_PID_ctrler);
        PID_ctrler_reset(&FR_pos_PID_ctrler);
        PID_ctrler_reset(&BL_pos_PID_ctrler);
        PID_ctrler_reset(&BR_pos_PID_ctrler);
        Whole_Movement_Type = movement_type;
    }
    switch (movement_type){
        case Movement_Breaking:{
            CN_Intr_Movement();
            break;
        }
        case Movement_Rotation:{
            target_yaw = movement.Car_angle;
            YAW_inital = YAW_now; // 起始角度
            YAW_gain = 0;  // 累计角度
            YAW_pre = YAW_inital;   // 上一次的角度
            CN_CAR_vel_CL_set_aim(0,0);
            if (target_yaw >= 0){
               CN_CAR_anv_CL_set_aim(fabs(movement.Car_angvel));
            }
            else {
               CN_CAR_anv_CL_set_aim(-fabs(movement.Car_angvel)); 
            }
            
            break;
        }
        case Movement_Running:{
            CN_CAR_vel_CL_set_aim(movement.Car_X_vel,movement.Car_Y_vel);
            // CN_CAR_anv_CL_set_aim(movement.Car_angvel);
            CN_CAR_rotation_set_aim(movement.Car_angvel);
            break;
        }
        case Movement_Translation:{
            CN_CAR_pos_CL_set_aim(movement);
            break;
        }
        default:{
            break;
        }
    }

   
}

// 运动停止
void CN_Intr_Movement(void){
    CN_CAR_vel_CL_set_aim(0, 0);
    CN_CAR_anv_CL_set_aim(0); 
    CN_CAR_rotation_set_aim(0);

    Duty_now.M_FL_Duty = 0;
    Duty_now.M_FR_Duty = 0;
    Duty_now.M_BL_Duty = 0;
    Duty_now.M_BR_Duty = 0;
    CN_set_Motor_Duty();
}

// 运动回调函数
void CN_Movement_ctrl_CALLBACK(uint32_t ms){
    static uint32_t time = 0;
    time += ms;
    time = (time > 1000) ? 0 : time;
    switch (Whole_Movement_Type){
        case Movement_Breaking:{
            //CN_Intr_Movement();
            YAW_aim =  YAW_now;
            break;
        }
        case Movement_Running:{
            if (time % 10 == 0){
                YAW_aim = CN_yaw_add(YAW_aim,YAW_acc * 0.01);
            }
            if (time % 10 == 0){
               CN_CAR_yaw_CL_CALLBACK(10000); 
            }
            if (time % 5 == 0){
               CN_CAR_anv_CL_CALLBACK(5000);
            }
            if (time % 1 == 0){
              CN_CAR_vel_CL_CALLBACK(1000);
            }
            break;
        }
        case Movement_Rotation:{
            float YawDelta = YAW_now - YAW_pre;
            if (YawDelta > 180){
                YawDelta -= 360;
            }
            else if (YawDelta < -180){
                YawDelta += 360;
            }
            YAW_gain += YawDelta;

            // 旋转至目标值 切换回运行状态
            if (fabs(YAW_gain) >= fabs(target_yaw)) {
                Car_Movement_struct TempMovement ={
                       .Car_X_movement = 0,
                       .Car_Y_movement = 0,
                       .Car_angle = 0, 

                       .Car_X_vel = 0,      // 车身的横向速度
                       .Car_Y_vel = 0,      // 车身的纵向速度
                       .Car_angvel = 0     // 车身的角速度
                };
                CN_set_movement(TempMovement,Movement_Running);
                break;
            }
            if (fabs(target_yaw - YAW_gain) < 20){
                CN_CAR_anv_CL_set_aim(0.3);
            }

            if (time % 5 == 0){
                CN_CAR_anv_CL_CALLBACK(5000);
            }
            if (time % 1 == 0){
                CN_CAR_vel_CL_CALLBACK(1000);
            }
            YAW_pre = YAW_now;
            YAW_aim =  YAW_now;
            break;
        }
        case Movement_Translation:{
            if (distance_to_go < 100.f){
                Car_Movement_struct TempMovement ={
                       .Car_X_movement = 0,
                       .Car_Y_movement = 0,
                       .Car_angle = 0, 

                       .Car_X_vel = 0,      // 车身的横向速度
                       .Car_Y_vel = 0,      // 车身的纵向速度
                       .Car_angvel = 0     // 车身的角速度
                };
                CN_set_movement(TempMovement,Movement_Running);
                break;
            }
            if (time % 5 == 0){
                CN_CAR_pos_CL_CALLBACK(5000);
            }
            break;
        }
    }
}
/****************************运动功能函数***************************/


/****************************车身角度环***************************/
#define CAR_YAW_CL_PID_KP (0.5f)
#define CAR_YAW_CL_PID_KI (0)
#define CAR_YAW_CL_PID_KD (0)
#define CAR_YAW_CL_PID_IMAX (0)
#define CAR_YAW_CL_PID_OUTMAX (10)


float YAW_acc = 0; // 角度增量

float target_yaw = 0; // 目标角度（任意角度）
float YAW_inital = 0; // 起始角度
float YAW_gain = 0;  // 累计角度
float YAW_pre = 0;   // 上一次的角度

// 设置目标角度增量（连续的、模拟角速度）
void CN_CAR_rotation_set_aim(float oumiga){
    YAW_acc = oumiga;
}

// 车身角度环初始化
void CN_CAR_yaw_CL_init(void){
    init_PID_ctrler(&Car_yaw_PID_ctrler,CAR_YAW_CL_PID_KP,CAR_YAW_CL_PID_KI,CAR_YAW_CL_PID_KD,CAR_YAW_CL_PID_IMAX,CAR_YAW_CL_PID_OUTMAX);
}

// 车身角度环回调函数
void CN_CAR_yaw_CL_CALLBACK(uint32_t us_time){
    float YAW_err=CN_get_relative_angle(YAW_now,YAW_aim);

    // if (YAW_err < -180) {
    //     YAW_err += 360;
    // }
    // else if (YAW_err > 180){
    //     YAW_err -= 360;
    // }
    // 死区设置
    YAW_err=fabs(YAW_err)<1?0:YAW_err;
    // 更新角度环输出
    PID_ctrler_update_with_time(&Car_yaw_PID_ctrler,YAW_err,us_time);
    // 设置目标角速度
    CN_CAR_anv_CL_set_aim(YAW_acc * 0.1+Car_yaw_PID_ctrler.output);
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
void CN_CAR_yaw_CL_set_aim(float yaw){
    YAW_aim=CN_yaw_add(YAW_now,yaw);
}
/****************************车身角度环***************************/


/****************************车身角速度环***************************/

// 角速度设置范围 大致为 -3.0 ~ 3.0

#define CAR_ANV_CL_PID_KP     (0.5f)
#define CAR_ANV_CL_PID_KI     (0.00f)
#define CAR_ANV_CL_PID_KD     (0.0f)
#define CAR_ANV_CL_PID_IMAX    (10.0f)
#define CAR_ANV_CL_PID_OUTMAX   (50.0f)

float target_anv = 0;

// 角速度环初始化
void CN_CAR_anv_CL_init(void){
    init_PID_ctrler(&Car_anv_PID_ctrler,CAR_ANV_CL_PID_KP,CAR_ANV_CL_PID_KI,CAR_ANV_CL_PID_KD,CAR_ANV_CL_PID_IMAX,CAR_ANV_CL_PID_OUTMAX);
}

// 设置目标角速度
void CN_CAR_anv_CL_set_aim(float anv){
    target_anv = anv;
}

// 角速度环控制回调
void CN_CAR_anv_CL_CALLBACK(uint32_t us_time){
    float anv_err = target_anv - IMU_data.G_Z;     // rad/s

    if (fabs(anv_err) < 0.01f) {
        anv_err = 0;
    }
    PID_ctrler_update_with_time(&Car_anv_PID_ctrler, anv_err,us_time);
    
}
/****************************车身角速度环***************************/

/****************************车身速度环***************************/
float target_VelX = 0;
float target_VelY = 0;

// 速度环初始化
void CN_CAR_vel_CL_init(void){
    CN_Motor_vel_CL_init();  // 电机速度环初始化
}

// 设置目标速度
void CN_CAR_vel_CL_set_aim(float VelX, float VelY){
    target_VelX = VelX;
    target_VelY = VelY;
}

// 速度环控制回调
void CN_CAR_vel_CL_CALLBACK(uint32_t us_time){
    // 计算速度 
    Vel_aim.M_FL_Vel = (target_VelY + target_VelX + Car_anv_PID_ctrler.output) * M_TO_ENC_FACTOR;
    Vel_aim.M_FR_Vel = (target_VelY - target_VelX - Car_anv_PID_ctrler.output) * M_TO_ENC_FACTOR;
    Vel_aim.M_BL_Vel = (target_VelY - target_VelX + Car_anv_PID_ctrler.output) * M_TO_ENC_FACTOR;
    Vel_aim.M_BR_Vel = (target_VelY + target_VelX - Car_anv_PID_ctrler.output) * M_TO_ENC_FACTOR;
    CN_Motor_vel_CL_CALLBACK(us_time);
}
/****************************车身速度环***************************/

/**************************** 改进版车身位置环 ***************************/
#define CAR_YAW_MOVING_CL_PID_KP (5.0f)
#define CAR_YAW_MOVING_CL_PID_KI (0.00f)    //*200hz=2
#define CAR_YAW_MOVING_CL_PID_KD (10.0f)
#define CAR_YAW_MOVING_CL_PID_IMAX (2)
#define CAR_YAW_MOVING_CL_PID_OUTMAX (10)

// 运动参数配置
#define MAX_CAR_SPEED 50.0f     // 最大合速度 (单位/s)
#define MIN_CAR_SPEED 5.0f      // 最小有效速度
#define ACCEL_RATE 2.0f         // 加速度系数

#define TEST_SPEED 10.0f        // 测试速度



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

    Pos_now.M_FL_Pos = 0;
    Pos_now.M_FR_Pos = 0;
    Pos_now.M_BL_Pos = 0;
    Pos_now.M_BR_Pos = 0;

    Vel_now.M_FL_Vel = 0;
    Vel_now.M_FR_Vel = 0;
    Vel_now.M_BL_Vel = 0;
    Vel_now.M_BR_Vel = 0;

    ENC_data[0] = 0;
    ENC_data[1] = 0;
    ENC_data[2] = 0;
    ENC_data[3] = 0;
   
    // 设置位置目标
    start_pose.x = 0;  // 以当前位置为原点
    start_pose.y = 0;
    target_pose.x = C_pos_aim.Car_X_movement;
    target_pose.y = C_pos_aim.Car_Y_movement;
    distance_to_go = sqrtf(C_pos_aim.Car_X_movement * C_pos_aim.Car_X_movement + C_pos_aim.Car_Y_movement * C_pos_aim.Car_Y_movement);

    PID_ctrler_reset(&Car_yaw_moving_PID_ctrler);
}

// 计算当前位置
float cur_x, cur_y;

// 位置环核心回调函数
void CN_CAR_pos_CL_CALLBACK(uint32_t us_time) {
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
    yaw_err=fabs(yaw_err)<3.0?0:yaw_err;    // 到达死区
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

    //printf("%.3f\n", Pos_now.M_FL_Pos);
    

    // 调用速度环
    CN_Motor_vel_CL_CALLBACK(us_time);
}



/****************************机身位置环***************************/


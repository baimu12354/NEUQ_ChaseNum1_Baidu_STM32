#include "ChaseNum1_Include.h"
#include <string.h> // for memcpy
#include <stdio.h>

HostCmd_t g_HostCmd; // 引用刚刚定义的结构体

// 放在 uart1 回调中
void ARM_RX_Callback(uint8_t dat) 
{
    static ProtocolState_t state = STATE_WAIT_H1;
    static uint8_t rx_buf[20];
    static uint8_t rx_cnt = 0;
    static uint8_t checksum = 0;

    switch(state) {
        case STATE_WAIT_H1:
            if(dat == 0x5A) {
                state = STATE_WAIT_H2;
                checksum = 0;
            }
            break;
            
        case STATE_WAIT_H2:
            if(dat == 0xA5) state = STATE_WAIT_CMD;
            else state = STATE_WAIT_H1;
            break;
            
        case STATE_WAIT_CMD:
            rx_buf[0] = dat; // 保存指令类型
            checksum += dat;
            rx_cnt = 1;
            if(dat == 0x00) {
                state = STATE_WAIT_CHECK; // 急停无数据载荷，直接等校验和
            } else if (dat == 0x01) {
                state = STATE_RECEIVE_DATA; // 任务指令，准备接收 16 字节浮点数
            } else {
                state = STATE_WAIT_H1; // 未知指令，丢弃
            }
            break;
            
        case STATE_RECEIVE_DATA:
            rx_buf[rx_cnt++] = dat;
            checksum += dat;
            if(rx_cnt >= 17) { // 1字节指令 + 16字节数据
                state = STATE_WAIT_CHECK;
            }
            break;
            
        case STATE_WAIT_CHECK:
            if(dat == checksum) state = STATE_WAIT_T1;
            else state = STATE_WAIT_H1; // 校验和错误，丢弃整帧
            break;
            
        case STATE_WAIT_T1:
            if(dat == 0x0D) state = STATE_WAIT_T2;
            else state = STATE_WAIT_H1;
            break;
            
        case STATE_WAIT_T2:
            if(dat == 0x0A) {
                // 成功接收到完整且校验正确的一帧数据！
                if(rx_buf[0] == 0x00) {
                    g_HostCmd.run_flag = 0; // 触发急停
                    g_HostCmd.new_cmd_flag = 1;
                } 
                else if(rx_buf[0] == 0x01) {
                    g_HostCmd.run_flag = 1;
                    // 使用 memcpy 将字节流直接转回 float (规避内存对齐问题)
                    memcpy(&g_HostCmd.th0, &rx_buf[1],  4);
                    memcpy(&g_HostCmd.x,   &rx_buf[5],  4);
                    memcpy(&g_HostCmd.y,   &rx_buf[9],  4);
                    memcpy(&g_HostCmd.th3, &rx_buf[13], 4);
                    g_HostCmd.new_cmd_flag = 1;
                }
            }
            state = STATE_WAIT_H1; // 恢复初始状态，等待下一帧
            break;
            
        default:
            state = STATE_WAIT_H1;
            break;
    }
}

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


// 机械臂物理结构参数 (单位: cm)
#define ARM_H0      6.5f    // 云台高度
#define ARM_H1      6.5f    // 齿轮相对云台高度
#define ARM_R       3.0f    // 大齿轮投影中心距
#define ARM_L1      20.0f   // 臂1长度
#define ARM_C       2.5f    // 舵机2偏置距离
#define ARM_L2      17.5f    // 臂2长度


// 舵机真实物理标定参数 (虚拟映射与物理硬限幅)
// [舵机 0] 底座旋转 (单圈)
#define S0_ID       0
#define S0_GEAR     1.0f
#define S0_DIR      1.0f    
#define S0_OFFSET   5.0f    
#define S0_MIN      -180.0f 
#define S0_MAX      180.0f  

// [舵机 1] 大齿轮驱动 (多圈)
// 抬升时实际角度往负数走
#define S1_ID       1
#define S1_GEAR     3.0f    
#define S1_DIR      1.0f   
#define S1_OFFSET   -555.0f  
#define S1_MIN      -555.0f 
#define S1_MAX      -15.0f  

// [舵机 2] 肘部关节 (单圈)
// 抬升实际角度减小
#define S2_ID       2
#define S2_GEAR     1.0f
#define S2_DIR      -1.0f   
#define S2_OFFSET   30.0f    
#define S2_MIN      -120.0f 
#define S2_MAX      180.0f  

// [舵机 3] 吸盘关节 (单圈)
// 抬升实际角度减小
#define S3_ID       3
#define S3_GEAR     1.0f
#define S3_DIR      -1.0f   
#define S3_OFFSET   0.0f    
#define S3_MIN      -90.0f  
#define S3_MAX      90.0f   

// 机械臂初始化
void Arm_Init(void) {

    // 调用总线舵机底层初始化 (打开 UART7)
    Servo_Init();

    // 初始姿态
    float t1=0,t2=0;
	Arm_Inverse_Kinematics(5, 20, &t1, &t2);
	Arm_Execute_Move(0, t1,t2, 0);
}

// 机械臂解算
uint8_t Arm_Inverse_Kinematics(float plane_x, float plane_y, float *out_th1, float *out_th2) {
    float L_eff = sqrtf(ARM_L1 * ARM_L1 + ARM_C * ARM_C);
    float phi = atan2f(ARM_C, ARM_L1);
    
    float dx = plane_x - ARM_R;
    float dy = plane_y - (ARM_H0 + ARM_H1);
    float D = sqrtf(dx * dx + dy * dy);
    
    if (D > (L_eff + ARM_L2) || D < fabsf(L_eff - ARM_L2)) {
        return 0; // 越界，连杆无法触达
    }
    
    float cos_beta = (L_eff * L_eff + D * D - ARM_L2 * ARM_L2) / (2.0f * L_eff * D);
    float beta = acosf(fmaxf(-1.0f, fminf(1.0f, cos_beta)));
    float alpha = atan2f(dy, dx);
    
    float theta_eff = alpha + beta;
    float theta1_rad = theta_eff - phi;
    
    float cos_gamma = (L_eff * L_eff + ARM_L2 * ARM_L2 - D * D) / (2.0f * L_eff * ARM_L2);
    float gamma = acosf(fmaxf(-1.0f, fminf(1.0f, cos_gamma)));
    float theta2_rad = gamma - M_PI + phi;
    
    *out_th1 = theta1_rad * 180.0f / M_PI;
    *out_th2 = theta2_rad * 180.0f / M_PI;
    
    return 1;
}

// 物理映射
void Arm_Execute_Move(float v_th0, float v_th1, float v_th2, float v_th3) {
    // 物理角度映射
    float actual_s0 = (v_th0 * S0_GEAR * S0_DIR) + S0_OFFSET;
    float actual_s1 = (v_th1 * S1_GEAR * S1_DIR) + S1_OFFSET;
    float actual_s2 = (v_th2 * S2_GEAR * S2_DIR) + S2_OFFSET;
    float actual_s3 = (v_th3 * S3_GEAR * S3_DIR) + S3_OFFSET;
    
    // 物理硬限幅
    actual_s0 = fmaxf(S0_MIN, fminf(S0_MAX, actual_s0));
    actual_s1 = fmaxf(S1_MIN, fminf(S1_MAX, actual_s1));
    actual_s2 = fmaxf(S2_MIN, fminf(S2_MAX, actual_s2));
    actual_s3 = fmaxf(S3_MIN, fminf(S3_MAX, actual_s3));
    
    // 速度与时间控制
    //uint16_t move_time = 1000;   // 运动总时间 (ms)
    uint16_t move_speed = 100;   // 运动速度 (10°/s) 
    uint16_t accel_time = 200;   // 柔性加速 (ms)
    uint16_t decel_time = 200;   // 柔性减速 (ms)
    uint16_t power = 0;
    
    
    // 舵机驱动
    Servo_Advanced_Single_Turn_Speed(S0_ID, (int16_t)(actual_s0 * 10), move_speed, accel_time, decel_time, power);
    Servo_Advanced_Multi_Turn_Speed( S1_ID, (int32_t)(actual_s1 * 10), (uint16_t)move_speed, accel_time, decel_time, power);
    Servo_Advanced_Single_Turn_Speed(S2_ID, (int16_t)(actual_s2 * 10), move_speed, accel_time, decel_time, power);
    Servo_Advanced_Single_Turn_Speed(S3_ID, (int16_t)(actual_s3 * 10), move_speed, accel_time, decel_time, power);
}

// 机械臂解算测试
void Arm_Standalone_Test(void) {
    float v_th1 = 0.0f, v_th2 = 0.0f;
    
    // 定义一个简单的二维坐标轨迹：高处远点 -> 低处远点 -> 低处近点 -> 高处近点
    // 格式: {th0, plane_x, plane_y, th3}
    float test_points[4][4] = {
        {0.0f,  15.0f, 15.0f, -30.0f}, // 动作 1：正前方，高处伸展
        {0.0f,  15.0f,  5.0f, -60.0f}, // 动作 2：正前方，低处抓取
        {45.0f,  8.0f,  5.0f,   0.0f}, // 动作 3：向左转45度，收回手臂
        {45.0f,  8.0f, 15.0f,   0.0f}  // 动作 4：保持左转，抬高
    };

    printf("\r\n[ARM TEST] Starting standalone routine...\r\n");

    for (int i = 0; i < 4; i++) {
        float target_th0 = test_points[i][0];
        float target_x   = test_points[i][1];
        float target_y   = test_points[i][2];
        float target_th3 = test_points[i][3];
        
        printf("[ARM TEST] Executing Point %d: (th0=%.1f, X=%.1f, Y=%.1f)\r\n", i+1, target_th0, target_x, target_y);
        
        if (Arm_Inverse_Kinematics(target_x, target_y, &v_th1, &v_th2)) {
            // 解算成功，执行物理动作
            Arm_Execute_Move(target_th0, v_th1, v_th2, target_th3);
        } else {
            // 这几个测试点都是在安全范围内的，如果报错说明参数宏定义填错了
            printf("[ARM TEST] Error: Point %d out of reach!\r\n", i+1);
        }
        
        // 等待动作完成 (舵机运动1000ms，这里延时2000ms让它稳定)
        HAL_Delay(2000); 
    }
    
    printf("[ARM TEST] Routine finished.\r\n");
}

// 机械臂指令轮询
void Arm_Task_Loop(void) {
    char tx_buf[128]; // 用于向上位机发送字符串的缓冲区

    // 检查是否有新的上位机指令帧
    if (g_HostCmd.new_cmd_flag == 1) {
        g_HostCmd.new_cmd_flag = 0; 
        
        // 处理急停指令
        if (g_HostCmd.run_flag == 0) {
            Servo_Stop(S0_ID, SERVO_STOP_MODE_HOLD, 0);
            Servo_Stop(S1_ID, SERVO_STOP_MODE_HOLD, 0);
            Servo_Stop(S2_ID, SERVO_STOP_MODE_HOLD, 0);
            Servo_Stop(S3_ID, SERVO_STOP_MODE_HOLD, 0);
            
            // 向上位机报告已急停
            sprintf(tx_buf, "STOPPED: All servos locked.\r\n");
            CN_UART_write_string_FIFO(CN_UART_3, (uint8_t*)tx_buf);
        } 
        // 处理运动指令
        else {
            float v_th1 = 0.0f, v_th2 = 0.0f;
            
            // 实时解算
            if (Arm_Inverse_Kinematics(g_HostCmd.x, g_HostCmd.y, &v_th1, &v_th2)) {
                // 解算成功 执行物理运动
                Arm_Execute_Move(g_HostCmd.th0, v_th1, v_th2, g_HostCmd.th3);
                
                // 向上位机返回成功及四个虚拟角度
                sprintf(tx_buf, "SUCCESS: th0=%.1f, th1=%.1f, th2=%.1f, th3=%.1f\r\n", 
                        g_HostCmd.th0, v_th1, v_th2, g_HostCmd.th3);
                CN_UART_write_string_FIFO(CN_UART_3, (uint8_t*)tx_buf);
            } 
            else {
                // 解算失败 保持原位并报告错误
                sprintf(tx_buf, "ERROR: Target (X:%.1f, Y:%.1f) out of reach!\r\n", 
                        g_HostCmd.x, g_HostCmd.y);
                CN_UART_write_string_FIFO(CN_UART_3, (uint8_t*)tx_buf);
            }
        }
    }
}
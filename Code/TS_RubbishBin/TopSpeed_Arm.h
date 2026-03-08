// #ifndef _TOPSPEED_ARM_H_
// #define _TOPSPEED_ARM_H_

// #include "stm32h7xx_hal.h"
// #include "TopSpeed_Typedef.h"

// #define ARM_L1_LENTH    (0.083f)//m
// #define ARM_L2_LENTH    (0.083f)
// #define ARM_L3_LENTH    (0.074f)

// #define ARM_JOINT_YAW_ANGLE_MAX   (90.0f)
// #define ARM_JOINT_YAW_ANGLE_MIN   (-90.0f)
// #define ARM_JOINT_YAW_CMD_MAX   (3080.0f)
// #define ARM_JOINT_YAW_CMD_MIN   (800.0f)

// #define ARM_JOINT_PITCH_1_ANGLE_MAX   (90.0f)
// #define ARM_JOINT_PITCH_1_ANGLE_MIN   (-90.0f)
// #define ARM_JOINT_PITCH_1_CMD_MAX   (900.0f)
// #define ARM_JOINT_PITCH_1_CMD_MIN   (3020.0f)

// #define ARM_JOINT_PITCH_2_ANGLE_MAX   (90.0f)
// #define ARM_JOINT_PITCH_2_ANGLE_MIN   (-90.0f)
// #define ARM_JOINT_PITCH_2_CMD_MAX   (900.0f)
// #define ARM_JOINT_PITCH_2_CMD_MIN   (3090.0f)

// #define ARM_JOINT_PITCH_3_ANGLE_MAX   (90.0f)
// #define ARM_JOINT_PITCH_3_ANGLE_MIN   (-90.0f)
// #define ARM_JOINT_PITCH_3_CMD_MAX   (880.0f)
// #define ARM_JOINT_PITCH_3_CMD_MIN   (3070.0f)

// #define ARM_JOINT_ROLL_ANGLE_MAX   (180.0f)
// #define ARM_JOINT_ROLL_ANGLE_MIN   (-180.0f)
// #define ARM_JOINT_ROLL_CMD_MAX   (400.0f)
// #define ARM_JOINT_ROLL_CMD_MIN   (2650.0f)

// #define ARM_JOINT_GRIP_ANGLE_MAX   (10000.0f)
// #define ARM_JOINT_GRIP_ANGLE_MIN   (0.0f)
// #define ARM_JOINT_GRIP_CMD_MAX   (3000.0f)
// #define ARM_JOINT_GRIP_CMD_MIN   (1000.0f)

// typedef enum{
//     ARM_Joint_yaw,
//     ARM_Joint_pitch_1,
//     ARM_Joint_pitch_2,
//     ARM_Joint_pitch_3,
//     ARM_Joint_roll,
//     ARM_Joint_grip,
// }ARM_Joint_enum;

// typedef struct{
//     float Joint_yaw_ang;          //车头定义为0度，范围+-90度
//     float Joint_pitch_1_ang;      //与地面垂直定义为0度，范围+-90度
//     float Joint_pitch_2_ang;      //与Joint_pitch_1共线定义为0度，范围+-90度
//     float Joint_pitch_3_ang;      //与Joint_pitch_2共线定义为0度，范围+-90度
//     float Joint_roll_ang;         //夹爪与地面平行定义为0度，范围+-180度
//     float Joint_grip_ang;         //张到最大定义为0，夹到最紧定义为10000

//     uint8_t valid_solve;          //0表示无解，1表示有解
// }ARM_Joint_ang_struct;

// typedef struct{
//     float yaw;          //夹爪的偏航轴，车头定义为0度，范围为+-90度
//     float pitch;        //夹爪的俯仰轴，平行地面定义为0度，上仰为正，下俯为负，范围为+-90度
//     float roll;         //夹爪的滚转轴，平行地面定义为0度，沿夹爪伸出方向看，逆时针为正，顺时针为负，范围为+-180度
//     float end_x;        //俯仰轴末端在俯仰平面上的横坐标
//     float end_y;        //俯仰轴末端在俯仰平面上的纵坐标
// }ARM_Attitude_struct;

// ARM_Joint_ang_struct TS_6DOF_ARM_Inverse_Kinematics_Solve(ARM_Attitude_struct Attitude_aim);
// void TS_6DOF_ARM_set_angle(ARM_Joint_ang_struct Joint_ang);


// #endif // !_TOPSPEED_ACTION_H_
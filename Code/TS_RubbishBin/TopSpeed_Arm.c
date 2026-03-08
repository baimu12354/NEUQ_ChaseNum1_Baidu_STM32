// #include "TopSpeed_Include.h"

// ARM_Joint_ang_struct ARM_Joint_angle_now={0};
// ARM_Attitude_struct ARM_Attitude_now={0};

// void TS_init_ARM(void){
//     TS_Bus_Servo_init();

//     // TS_BusServo_SetID(1,4);
//     // TS_delay_ms(1000);

//     // TS_BusServo_Write(6,2000,1000);

//     // ARM_Joint_ang_struct Joint_ang={0};
//     // Joint_ang.Joint_yaw_ang=0.0f;
//     // Joint_ang.Joint_pitch_1_ang=0.0f;
//     // Joint_ang.Joint_pitch_2_ang=0.0f;
//     // Joint_ang.Joint_pitch_3_ang=0.0f;
//     // Joint_ang.Joint_roll_ang=0.0f;
//     // Joint_ang.Joint_grip_ang=0.0f;
//     // Joint_ang.valid_solve=1;
//     // TS_6DOF_ARM_set_angle(Joint_ang);

//     // ARM_Attitude_now.end_x=-0.12;
//     // ARM_Attitude_now.end_y=0.02f;
//     // ARM_Attitude_now.pitch=-90.0f;
//     // ARM_Attitude_now.roll=0.0f;
//     // ARM_Attitude_now.yaw=0.0f;

//     // ARM_Joint_ang_struct IF_res=TS_6DOF_ARM_Inverse_Kinematics_Solve(ARM_Attitude_now);
//     // IF_res.Joint_grip_ang=0;

//     // TS_6DOF_ARM_set_angle(IF_res);
//     // TS_delay_ms(1000);

//     ARM_Attitude_now.end_x=-0.12;
//     ARM_Attitude_now.end_y=0.0f;
//     ARM_Attitude_now.pitch=-90.0f;
//     ARM_Attitude_now.roll=0.0f;
//     ARM_Attitude_now.yaw=0.0f;

//     ARM_Joint_ang_struct IF_res=TS_6DOF_ARM_Inverse_Kinematics_Solve(ARM_Attitude_now);
//     IF_res.Joint_grip_ang=0;

//     TS_6DOF_ARM_set_angle(IF_res);
//     TS_delay_ms(1000);

//     IF_res.Joint_grip_ang=5000;
//     TS_6DOF_ARM_set_angle(IF_res);

//     TS_delay_ms(1000);


//     ARM_Attitude_now.end_x=0.10;
//     ARM_Attitude_now.end_y=0.3f;
//     ARM_Attitude_now.pitch=-90.0f;
//     ARM_Attitude_now.roll=0.0f;
//     ARM_Attitude_now.yaw=-90.0f;

//     IF_res=TS_6DOF_ARM_Inverse_Kinematics_Solve(ARM_Attitude_now);
//     IF_res.Joint_grip_ang=5000;

//     TS_6DOF_ARM_set_angle(IF_res);

//     TS_delay_ms(1000);

//     ARM_Attitude_now.end_x=0.10;
//     ARM_Attitude_now.end_y=0.0f;
//     ARM_Attitude_now.pitch=-90.0f;
//     ARM_Attitude_now.roll=0.0f;
//     ARM_Attitude_now.yaw=-90.0f;

//     IF_res=TS_6DOF_ARM_Inverse_Kinematics_Solve(ARM_Attitude_now);
//     IF_res.Joint_grip_ang=5000;

//     TS_6DOF_ARM_set_angle(IF_res);

//     TS_delay_ms(1000);

//     IF_res.Joint_grip_ang=0;

//     TS_6DOF_ARM_set_angle(IF_res);
//     TS_delay_ms(1000);

//     ARM_Attitude_now.end_x=0.10;
//     ARM_Attitude_now.end_y=0.5f;
//     ARM_Attitude_now.pitch=0.0f;
//     ARM_Attitude_now.roll=0.0f;
//     ARM_Attitude_now.yaw=-90.0f;

//     IF_res=TS_6DOF_ARM_Inverse_Kinematics_Solve(ARM_Attitude_now);
//     IF_res.Joint_grip_ang=0;

//     TS_6DOF_ARM_set_angle(IF_res);

//     TS_delay_ms(1000);

//     ARM_Joint_ang_struct Joint_ang={0};
//     Joint_ang.Joint_yaw_ang=0.0f;
//     Joint_ang.Joint_pitch_1_ang=0.0f;
//     Joint_ang.Joint_pitch_2_ang=0.0f;
//     Joint_ang.Joint_pitch_3_ang=0.0f;
//     Joint_ang.Joint_roll_ang=0.0f;
//     Joint_ang.Joint_grip_ang=0.0f;
//     Joint_ang.valid_solve=1;
//     TS_6DOF_ARM_set_angle(Joint_ang);

//     // if(IF_res.valid_solve==1){
//     //     printf("arm solved\n");
//     //     printf("yaw:%.2f,pit1:%.2f,pit2:%.2f,pit3:%.2f,roll:%.2f,grip:%.2f\n",IF_res.Joint_yaw_ang,IF_res.Joint_pitch_1_ang,IF_res.Joint_pitch_2_ang,IF_res.Joint_pitch_3_ang,IF_res.Joint_roll_ang,IF_res.Joint_grip_ang);
//     //     TS_6DOF_ARM_set_angle(IF_res);
//     // }
//     // else{
//     //     printf("arm solve fail\n");
//     // }

// }

// ARM_Attitude_struct TS_6DOF_ARM_Forward_Kinematics_Solve(ARM_Joint_ang_struct Joint_ang){
    
// }

// #include <math.h>
// ARM_Joint_ang_struct TS_6DOF_ARM_Inverse_Kinematics_Solve(ARM_Attitude_struct Attitude_aim){
//     ARM_Joint_ang_struct res={0};

//     //处理yaw轴舵机
//     Attitude_aim.yaw=Attitude_aim.yaw>90.0f?90.0f:Attitude_aim.yaw;
//     Attitude_aim.yaw=Attitude_aim.yaw<-90.0f?-90.0f:Attitude_aim.yaw;
//     res.Joint_yaw_ang=Attitude_aim.yaw;

//     //处理roll轴舵机
//     while(Attitude_aim.roll>180.0f){
//         Attitude_aim.roll-=360.0f;
//     }
//     while(Attitude_aim.roll<-180.0f){
//         Attitude_aim.roll+=360.0f;
//     }
//     res.Joint_roll_ang=Attitude_aim.roll;

//     float Cx=Attitude_aim.end_x;
//     float Cy=Attitude_aim.end_y;//pitch轴末端坐标



//     //不允许pitch轴末端离开第一象限
//     if(Cy<-0.05f){
//         res.valid_solve=0;
//         printf("fail:C out\n");
//         return res;
//     }

//     //处理pitch轴
//     Attitude_aim.pitch=Attitude_aim.pitch>90.0f?90.0f:Attitude_aim.pitch;
//     Attitude_aim.pitch=Attitude_aim.pitch<-90.0f?-90.0f:Attitude_aim.pitch;


    
//     printf("end attitude: (%f,%f),%f\n",Cx,Cy,Attitude_aim.pitch);

//     float dir_pitch=Attitude_aim.pitch-90.0f;//将俯仰角转换为方位角
//     if(Cx<0){
//         dir_pitch=-dir_pitch;
//     }    

//     //向前解出上一个关节坐标
//     float Bx=Cx+ARM_L3_LENTH*sinf(degree_to_rad(dir_pitch));
//     float By=Cy-ARM_L3_LENTH*cosf(degree_to_rad(dir_pitch));

//     printf("B: (%f,%f)\n",Bx,By);
    

//     //不允许关节离开第一二象限
//     if(By<-0.05){
//         printf("fail:B out,(%f,%f)\n",Bx,By);
//         res.valid_solve=0;
//         return res;
//     }

//     //计算二连杆第三边长度
//     float OB_len=sqrtf(Bx*Bx+By*By);

//     //无法构成三角形，无解
//     if(OB_len>(ARM_L1_LENTH+ARM_L2_LENTH)){
//         res.valid_solve=0;
//         printf("fail:no triangle L1+L2=%f;OB=%f\n",(ARM_L1_LENTH+ARM_L2_LENTH),OB_len);
//         return res;
//     }

//     //余弦定理
//     float ang_AOB=rad_to_degree(acosf((OB_len*OB_len+ARM_L1_LENTH*ARM_L1_LENTH-ARM_L2_LENTH*ARM_L2_LENTH)/(2*OB_len*ARM_L1_LENTH)));
//     float ang_ABO=rad_to_degree(acosf((OB_len*OB_len+ARM_L2_LENTH*ARM_L2_LENTH-ARM_L1_LENTH*ARM_L1_LENTH)/(2*OB_len*ARM_L2_LENTH)));

//     printf("ang_AOB=%.2f,ang_ABO=%.2f\n",ang_ABO,ang_ABO);

//     float dir_OB=-rad_to_degree(atan2f(Bx, By));
// printf("dir OB=%.2f\n",dir_OB);
//     //计算三关节两种解
//     float theta1_1=dir_OB+ang_AOB;
//     float theta1_2=dir_OB-ang_AOB;

//     float theta2_1=-(ang_AOB+ang_ABO);
//     float theta2_2=(ang_AOB+ang_ABO);

//     float dir_AB_1=theta1_1+theta2_1;
//     float dir_AB_2=theta1_2+theta2_2;

//     float theta3_1=dir_pitch-theta1_1-theta2_1;
//     float theta3_2=dir_pitch-theta1_2-theta2_2;

//     // float theta3_1=-get_relative_angle(dir_AB_1,dir_pitch);
//     // float theta3_2=-get_relative_angle(dir_AB_2,dir_pitch);


//     if(Cx>0){
//         //只要不超范围，优先选择肘部朝上的解
//         if(theta1_1<-90.0f || theta1_1>90.0f){//解1情况下摇臂1离开第一象限，则考虑解2
//             if(theta1_2<-90.0f || theta1_2>90.0f){//若解2摇臂1也离开第一象限，则无解
//                 res.valid_solve=0;
//                 return res;
//             }
//             else{//此时采用解2
//                 res.Joint_pitch_1_ang=theta1_2;
//                 res.Joint_pitch_2_ang=theta2_2;
//                 res.Joint_pitch_3_ang=theta3_2;
//             }
//         }
//         else{//此时优先选用解1
//             res.Joint_pitch_1_ang=theta1_1;
//             res.Joint_pitch_2_ang=theta2_1;
//             res.Joint_pitch_3_ang=theta3_1;
//         }        
//     }
//     else{
//         //只要不超范围，优先选择肘部朝上的解
//         if(theta1_2<-90.0f || theta1_2>90.0f){//解1情况下摇臂1离开第一象限，则考虑解2
//             if(theta1_1<-90.0f || theta1_1>90.0f){//若解2摇臂1也离开第一象限，则无解
//                 res.valid_solve=0;
//                 return res;
//             }
//             else{//此时采用解2
//                 res.Joint_pitch_1_ang=theta1_1;
//                 res.Joint_pitch_2_ang=theta2_1;
//                 res.Joint_pitch_3_ang=theta3_1;
//             }
//         }
//         else{//此时优先选用解1
//             res.Joint_pitch_1_ang=theta1_2;
//             res.Joint_pitch_2_ang=theta2_2;
//             res.Joint_pitch_3_ang=theta3_2;
//         }  
//     }



//     res.valid_solve=1;
//     return res;
// }

// void TS_6DOF_ARM_set_angle(ARM_Joint_ang_struct Joint_ang){
//     // if(Joint_ang.valid_solve){
//     //     TS_BusServo_PreWrite(1,2000,1000);
//     //     TS_delay_ms(100); 
//     //     // TS_BusServo_PreWrite(2,(uint16_t)TS_linear_map_float(Joint_ang.Joint_pitch_1_ang,ARM_JOINT_PITCH_1_ANGLE_MIN,ARM_JOINT_PITCH_1_ANGLE_MAX,ARM_JOINT_PITCH_1_CMD_MIN,ARM_JOINT_PITCH_1_CMD_MAX),1000);
//     //     // TS_delay_ms(100); 
//     //     // TS_BusServo_PreWrite(3,(uint16_t)TS_linear_map_float(Joint_ang.Joint_pitch_2_ang,ARM_JOINT_PITCH_2_ANGLE_MIN,ARM_JOINT_PITCH_2_ANGLE_MAX,ARM_JOINT_PITCH_2_CMD_MIN,ARM_JOINT_PITCH_2_CMD_MAX),1000);
//     //     // TS_delay_ms(100); 
//     //     // TS_BusServo_PreWrite(4,(uint16_t)TS_linear_map_float(Joint_ang.Joint_pitch_3_ang,ARM_JOINT_PITCH_3_ANGLE_MIN,ARM_JOINT_PITCH_3_ANGLE_MAX,ARM_JOINT_PITCH_3_CMD_MIN,ARM_JOINT_PITCH_3_CMD_MAX),1000);
//     //     // TS_delay_ms(100); 
//     //     // TS_BusServo_PreWrite(5,(uint16_t)TS_linear_map_float(Joint_ang.Joint_roll_ang,ARM_JOINT_ROLL_ANGLE_MIN,ARM_JOINT_ROLL_ANGLE_MAX,ARM_JOINT_ROLL_CMD_MIN,ARM_JOINT_ROLL_CMD_MAX),1000);
//     //     // TS_delay_ms(100); 
//     //     // TS_BusServo_PreWrite(6,(uint16_t)TS_linear_map_float(Joint_ang.Joint_grip_ang,ARM_JOINT_GRIP_ANGLE_MIN,ARM_JOINT_GRIP_ANGLE_MAX,ARM_JOINT_GRIP_CMD_MIN,ARM_JOINT_GRIP_CMD_MAX),1000);
//     //     // TS_delay_ms(100); 
//     //     TS_BusServo_move();        
//     //     TS_delay_ms(100); 
//     // }

    
//     TS_BusServo_Write(1,(uint16_t)TS_linear_map_float(Joint_ang.Joint_yaw_ang,ARM_JOINT_YAW_ANGLE_MIN,ARM_JOINT_YAW_ANGLE_MAX,ARM_JOINT_YAW_CMD_MIN,ARM_JOINT_YAW_CMD_MAX),500);
//     TS_delay_ms(100); 
//     TS_BusServo_Write(2,(uint16_t)TS_linear_map_float(Joint_ang.Joint_pitch_1_ang,ARM_JOINT_PITCH_1_ANGLE_MIN,ARM_JOINT_PITCH_1_ANGLE_MAX,ARM_JOINT_PITCH_1_CMD_MIN,ARM_JOINT_PITCH_1_CMD_MAX),500);
//     TS_delay_ms(100); 
//     TS_BusServo_Write(3,(uint16_t)TS_linear_map_float(Joint_ang.Joint_pitch_2_ang,ARM_JOINT_PITCH_2_ANGLE_MIN,ARM_JOINT_PITCH_2_ANGLE_MAX,ARM_JOINT_PITCH_2_CMD_MIN,ARM_JOINT_PITCH_2_CMD_MAX),500);
//     TS_delay_ms(100); 
//     TS_BusServo_Write(4,(uint16_t)TS_linear_map_float(Joint_ang.Joint_pitch_3_ang,ARM_JOINT_PITCH_3_ANGLE_MIN,ARM_JOINT_PITCH_3_ANGLE_MAX,ARM_JOINT_PITCH_3_CMD_MIN,ARM_JOINT_PITCH_3_CMD_MAX),500);
//     TS_delay_ms(100); 
//     TS_BusServo_Write(5,(uint16_t)TS_linear_map_float(Joint_ang.Joint_roll_ang,ARM_JOINT_ROLL_ANGLE_MIN,ARM_JOINT_ROLL_ANGLE_MAX,ARM_JOINT_ROLL_CMD_MIN,ARM_JOINT_ROLL_CMD_MAX),500);
//     TS_delay_ms(100); 
//     TS_BusServo_Write(6,(uint16_t)TS_linear_map_float(Joint_ang.Joint_grip_ang,ARM_JOINT_GRIP_ANGLE_MIN,ARM_JOINT_GRIP_ANGLE_MAX,ARM_JOINT_GRIP_CMD_MIN,ARM_JOINT_GRIP_CMD_MAX),500);
//     TS_delay_ms(100); 
// }

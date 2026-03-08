#ifndef _CHASE_NUM1_TYPEDEF_H_
#define _CHASE_NUM1_TYPEDEF_H_

#include "stm32h7xx_hal.h"

#pragma region SM_TYPEDEF_REGION
typedef enum{
    SM_NONE,
    SM_HELLO,
    SM_DISA,
    SM_ENA,
    SM_SET_ZERO,
    SM_GO_ZERO,
    SM_SET_SPD,
    SM_WAITING_POS,
    SM_SET_POS,
    SM_SET_PARA,
}SM_COM_OPE_ENUM;

typedef enum{
    SM_NO1,
    SM_NO2,
    SM_NO3,
}SM_NO_ENUM;

typedef struct{
    uint16_t SM_BUS_VOL;
    uint16_t SM_BUS_CUR;
    uint16_t SM_ENC_POS;
    int16_t  SM_POS_AIM;
    int16_t  SM_VEL;
    int16_t  SM_POS;
    int16_t  SM_POS_ERR;
    uint8_t  SM_READY;
    uint8_t  SM_STATUS;
}SM_SYSSTATUS_struct;

typedef struct{
    SM_NO_ENUM SM_SELECT;
    SM_COM_OPE_ENUM SM_status;

    float input_min;
    float input_max;

    float output_min;
    float output_max;

    float pos_now;
    float pos_aim;

    int timeout_to_inpos; //单位：秒

    int timeout_counter;
}SM_Handle_struct;
#pragma endregion

#pragma region SERVO_TYPEDEF_REGION
typedef enum{
    Servo_Gripper_yaw,
    Servo_Gripper_pitch1,
    Servo_Gripper_pitch2,
    Servo_Shooter_reload,
}Servo_enum;

typedef struct{
    Servo_enum SERVO_SELECT;

    float input_min;
    float input_max;

    float output_min;
    float output_max;

    float pos_now;
    float pos_aim;

    uint32_t Duty_now;      //当前输出占空比
    uint32_t Duty_aim;      //目标输出占空比
    int32_t Duty_inc;      //在缓输出模式下的占空比增量

    uint8_t Duty_Done;      //是否已到达目标占空比
}SERVO_Handle_struct;
#pragma endregion

#pragma region ACTION_TYPEDEF_REGION
typedef enum{
    Action_Lift_Z           =0x60,      //云台Z轴步进电机，输入0-10000，表示从最低到最高
    Action_Lift_X           =0x61,      //云台X轴步进电机，输入-10000-+10000，表示从最左到最右，中间为0
    Action_Gripper_yaw      =0x62,      //吸盘yaw轴舵机，输入-90-+90，表示以-Y方向为零度的夹爪偏航角度
    Action_Gripper_pitch    =0x63,      //吸盘pitch轴舵机，只有两种状态：俯视和平视，输入正数为平视，负数为俯视，控制此执行器只能使用绝对式控制
    Action_Gripper_grip     =0x64,      //吸盘吸取，只有两种状态：输入正数为吸取，输入负数不吸取，控制此执行器只能使用绝对式控制
    Action_Shooter_Reset    =0x65,      //复位弹弓，弹弓上升到上限位，载荷任意
    Action_Shooter_shoot    =0x66,      //启动弹弓任务，输入0-10000，表示蓄力力度，即射程，控制此执行器只能使用绝对式控制，启动后弹弓会自动蓄力释放
    Action_Shooter_reload   =0x67,      //弹弓装填，正数表示启用装填，负数表示复位装填设备，控制此执行器只能使用绝对式控制
    Action_Flag_Show        =0x68,      //显示旗帜，小于-1表示蓝色；-1~0表示绿色；0~1表示黄色；大于1表示橙色；控制此执行器只能使用绝对式控制
}Action_type_ENUM;

typedef enum{
    Action_method_absolute           =0x70,     //绝对式控制，如果超过限幅会被自动限制到最大值执行
    Action_method_incremental        =0x71,     //增量式控制，如果增量后超过限幅会被自动限制到最大值执行
}Action_method_ENUM;

typedef struct{
    Action_type_ENUM Action_type;
    Action_method_ENUM Action_method;
    float value;
}Action_CMD_struct;
#pragma endregion


#pragma region MOTOR_TYPEDEF_REGION
//定义使得车前进的电机旋转方向为正方向
typedef struct{
    float M_FL_Duty;
    float M_FR_Duty;
    float M_BL_Duty;
    float M_BR_Duty;
}Motor_duty_struct;

typedef struct{
    float M_FL_Vel;
    float M_FR_Vel;
    float M_BL_Vel;
    float M_BR_Vel;
}Motor_vel_struct;

typedef struct{
    float M_FL_Pos;
    float M_FR_Pos;
    float M_BL_Pos;
    float M_BR_Pos;
}Motor_pos_struct;
#pragma endregion

#pragma region MOVEMENT_TYPEDEF_REGION
typedef enum{
    Movement_Translation    =0x50,   
    Movement_Rotation       =0x51, 
    Movement_Set_speed      =0x60,
    Movement_Set_angvel     =0x61,
    Movement_Translation_Precise =0x52, //平移精确模式，要求在Translation_Tolerance_PRECISE范围内到位
    Movement_Rotation_Precise     =0x53, //旋转精确模式，要求在Rotation_Tolerance_PRECISE范围内到位
}Movement_type_ENUM;

typedef struct{
    //定义俯视视角下，上方为车头，记车头方向为Y正方向，右方为X正方向，车正上方为Z正方向
    //用右手螺旋定义Z轴旋转方向，即俯视逆时针为正向旋转，顺时针为逆向旋转
    //单位为标准单位，即m、deg
    float Car_X_movement;
    float Car_Y_movement;
    float Car_Z_rotation;

    float Car_speed;
    float Car_angvel;
}Car_Movement_struct;

typedef struct{
    //单位：rad/s、m/s^2
    float G_X;
    float G_Y;
    float G_Z;

    float A_X;
    float A_Y;
    float A_Z;
}IMU_Data_struct;
#pragma endregion

#pragma region PID_TYPEDEF_REGION
typedef struct{
	uint8_t first_cal;
	float Kp;
	float Ki;
	float Kd;

	float err;
	float err_last;
	float err_sum;

	float i_max;
	float out_max;

	float output;
}PID_ctrler;
#pragma endregion

#pragma region COM_TYPEDEF_REGION

#define COM_USE_PROTOCOL_V1 (0)
#define COM_USE_PROTOCOL_V2 (1)

#if COM_USE_PROTOCOL_V1
/******************************上位机指令*****************************/
#define CMD_REQ   (0xF1)    //上位机要求回复        Request             有载荷
#define CMD_STA   (0xF2)    //上位机询问状态        Status              无载荷
#define CMD_STT   (0xF3)    //上位机要求进入任务    Start               无载荷
#define CMD_RST   (0xF4)    //上位机要求复位任务    Reset               无载荷
#define CMD_MOTD  (0xF5)    //上位机要求紧急停止    Motor Disable       无载荷
#define CMD_INT   (0xF6)    //上位机要求打断动作    Interrupt           无载荷
#define CMD_SET   (0xF7)    //上位机对下位机相关参数设置    Set           有载荷

#define CMD_SETM  (0xFA)    //上位机发送运动指令    Set Movement        有载荷
#define CMD_SETA  (0xFB)    //上位机发送动作指令    Set Action          有载荷
#define CMD_INTA  (0xFC)    //上位机打断动作指令    Action Interrupt    有载荷

/******************************下位机指令*****************************/
#define CMD_REP   (0xE1)    //回复                 Reply               有载荷
#define CMD_SUC   (0xE2)    //确认/成功            Success             无载荷
#define CMD_FAL   (0xE3)    //错误/失败            Fail                无载荷
#define CMD_ASTA  (0xE4)    //下位机返回状态        Answer Status       有载荷
#define CMD_MOV   (0xE5)    //正在执行运动指令      Moving              有载荷
#define CMD_MOVD  (0xE6)    //运动指令完成          Movement Done       无载荷
#define CMD_ACTS  (0xE7)    //动作指令收到          Action Set         有载荷
#define CMD_ACTD  (0xE8)    //动作指令完成          Action Done        有载荷
#define CMD_ACTE  (0xE9)    //动作指令错误          Action Err         有载荷
#define CMD_ERR   (0xD1)    //发生严重错误          Error        有载荷

typedef enum{
    Main_CMD_NONE   =0,
    Main_CMD_REQ    =CMD_REQ,   //指令格式：字节0为请求的数据类型，由REQ_type_enum给出，只有这一个载荷
    Main_CMD_STA    =CMD_STA,
    Main_CMD_STT    =CMD_STT,
    Main_CMD_RST    =CMD_RST,
    Main_CMD_MOTD   =CMD_MOTD,
    Main_CMD_INT    =CMD_INT,
    Main_CMD_SET    =CMD_SET,   //指令格式：字节0为设置的数据类型，由SET_type_enum给出，载荷随后直接从字节1开始

    Main_CMD_SETM   =CMD_SETM,
    Main_CMD_SETA   =CMD_SETA,
    Main_CMD_INTA    =CMD_INTA,

    Main_CMD_REP    =CMD_REP,
    Main_CMD_SUC    =CMD_SUC,
    Main_CMD_FAL    =CMD_FAL,
    Main_CMD_ASTA   =CMD_ASTA,
    Main_CMD_MOV    =CMD_MOV,
    Main_CMD_MOVD   =CMD_MOVD,
    Main_CMD_ACTS   =CMD_ACTS,
    Main_CMD_ACTD   =CMD_ACTD,
    Main_CMD_ACTE   =CMD_ACTE,
    Main_CMD_ERR   =CMD_ERR,
}Main_CMD_ENUM;

typedef enum{
    SYS_INVALID     =0x00,
    SYS_INIT        =0x11,
    SYS_IDLE        =0x22,
    SYS_MISSION     =0x33,
    SYS_TRANSLATING =0x44,
    SYS_ROTATING    =0x55,
    SYS_ROUTING     =0x66,
    SYS_ERR         =0x77,
}SYS_STATUS_ENUM;
#endif

#if COM_USE_PROTOCOL_V2
/******************************上位机指令*****************************/
#define CMD_HELLO       (0xF1)    //上位机要求回复              无载荷
#define CMD_STATUS      (0xF2)    //上位机询问状态              无载荷
#define CMD_RESET       (0xF3)    //上位机要求复位              无载荷
#define CMD_CHECK       (0xF4)    //上位机要求自检              无载荷
#define CMD_RESTART     (0xF5)    //上位机要求下位机重启         无载荷
#define CMD_INLINE      (0xF6)    //上位机要求进入巡线模式       无载荷
#define CMD_OUTLINE     (0xF7)    //上位机要求退出巡线模式       无载荷
#define CMD_SETPARA     (0xF8)    //上位机对下位机相关参数设置    有载荷
#define CMD_GETPARA     (0xF9)    //上位机获取下位机相关参数设置  有载荷
#define CMD_SERVERSTATUS (0xF0)  //上位机响应服务器状态        有载荷

#define CMD_SETMOVEMENT  (0xFA)    //上位机发送运动指令     有载荷
#define CMD_SETACTION    (0xFB)    //上位机发送动作指令     有载荷
#define CMD_INTERRUPT    (0xFC)    //上位机打断动作指令     无载荷

/******************************下位机指令*****************************/
#define CMD_REPLY           (0xE1)    //回复                  无载荷
#define CMD_ASTATUS         (0xE2)    //下位机返回状态         有载荷
#define CMD_APARA           (0xE3)    //下位机响应参数请求     有载荷
#define CMD_RECVCMD         (0xE4)    //下位机已收到指令       有载荷
#define CMD_CMDDONE         (0xE5)    //下位机已完成指令       有载荷
#define CMD_ERR             (0xE6)    //发生错误              有载荷
#define CMD_IFONLINE        (0xE7)    //询问上位机是否在线     无载荷
#define CMD_STARTSERVER     (0xE8)    //要求上位机开启服务器    暂无载荷，后续视是否需要启动参数添加载荷
#define CMD_CLOSESERVER     (0xE9)    //要求上位机关闭服务器    暂无载荷，后续视是否需要启动参数添加载荷
#define CMD_STARTMISSION    (0xEA)    //要求上位机开始任务     暂无载荷，后续视是否需要启动参数添加载荷
#define CMD_SUSPENDMISSION  (0xEB)    //要求上位机暂停任务     暂无载荷，后续视是否需要启动参数添加载荷

typedef enum{
    Main_CMD_NONE           =0,
    Main_CMD_HELLO          =CMD_HELLO,         //无载荷，下位机收到后会回复CMD_REPLY
    Main_CMD_STATUS         =CMD_STATUS,        //无载荷，下位机收到后会回复当前系统状态
    Main_CMD_RESET          =CMD_RESET,         //无载荷，下位机收到后会返回CMD_RECVCMD，然后将退出巡线模式并使机械结构复位
    Main_CMD_CHECK          =CMD_CHECK,         //无载荷，下位机收到后会返回CMD_RECVCMD，然后进行自检，如果成功会返回CMD_CMDDONE，若失败会返回CMD_ERR
    Main_CMD_RESTART        =CMD_RESTART,       //无载荷，下位机收到后会返回CMD_RECVCMD，然后进行软件复位
    Main_CMD_INLINE         =CMD_INLINE,        //无载荷，下位机收到后会返回CMD_RECVCMD，并进入巡线模式等待巡线指令，若系统状态错误无法进入巡线模式，则返回CMD_ERR
    Main_CMD_OUTLINE        =CMD_OUTLINE,       //无载荷，下位机收到后会返回CMD_RECVCMD，并退出巡线模式
    Main_CMD_SETPARA        =CMD_SETPARA,       //指令格式：字节0为设置的数据类型，由SET_type_enum给出，载荷随后直接从字节1开始，设置成功后下位机会返回CMD_RECVCMD
    Main_CMD_GETPARA        =CMD_GETPARA,       //指令格式：字节0为请求的数据类型，由REQ_type_enum给出，只有这一个载荷，下位机收到后会返回CMD_APARA
    Main_CMD_SERVERSTATUS  =CMD_SERVERSTATUS,       //指令格式：字节0为服务器状态，1表示已启动，0表示未启动



    Main_CMD_SETMOVEMENT    =CMD_SETMOVEMENT,   //指令格式：字节0为Movement_type_ENUM，随后紧跟一个Car_Movement_struct，只有在巡线模式下可以设置Movement_Set_speed和Movement_Set_angvel;
                                                    //下位机收到指令后会返回CMD_RECVCMD，巡线模式下不再返回就位命令，平移、旋转指令会在到达窗口内返回CMD_CMDDONE
    Main_CMD_SETACTION      =CMD_SETACTION,     //指令格式：字节0为Action_type_ENUM，字节1为Action_method_ENUM，随后紧跟一个float;
                                                    //下位机收到指令后会返回CMD_RECVCMD，动作完成后会返回CMD_CMDDONE
    Main_CMD_INTERRUPT      =CMD_INTERRUPT,     //无载荷，下位机收到后会返回CMD_RECVCMD，并停止一切动作


    Main_CMD_REPLY          =CMD_REPLY,         //无载荷，仅用于测试通信
    Main_CMD_ASTATUS        =CMD_ASTATUS,       //返回系统状态SYS_STATUS_ENUM
    Main_CMD_APARA          =CMD_APARA,         //返回系统状态SYS_STATUS_ENUM
    Main_CMD_RECVCMD        =CMD_RECVCMD,       //有载荷，表示收到指令，字节0为原始指令的类型，字节1为数据包的校验
    Main_CMD_CMDDONE        =CMD_CMDDONE,       //有载荷，字节0为就位的设备类型，由CMDDONE_type_enum给出
    Main_CMD_ERR            =CMD_ERR,           //有载荷，字节0为原始指令的类型，字节1为错误码
    Main_CMD_IFONLINE       =CMD_IFONLINE,      //无载荷，用于下位机启动后等待上位机就绪，上位机应在完全就绪后再启动对此指令的响应，收到后回复CMD_HELLO
    Main_CMD_STARTSERVER    =CMD_STARTSERVER,  //暂无载荷，后续视是否需要启动参数添加载荷
    Main_CMD_CLOSESERVER    =CMD_CLOSESERVER,  //暂无载荷，后续视是否需要启动参数添加载荷
    Main_CMD_STARTMISSION   =CMD_STARTMISSION,  //暂无载荷，后续视是否需要启动参数添加载荷
    Main_CMD_SUSPENDMISSION   =CMD_SUSPENDMISSION,  //暂无载荷，后续视是否需要启动参数添加载荷
}Main_CMD_ENUM;

typedef enum{
    CMDDONE_type_moving     =0x21,      //位移完成
    CMDDONE_type_rotating   =0x22,      //旋转完成
    CMDDONE_type_lift_z     =0x23,      //云台z轴就位
    CMDDONE_type_lift_x     =0x24,      //云台x轴就位
    CMDDONE_type_shooter    =0x25,      //弹弓发射完成
}CMDDONE_type_enum;

#endif

typedef union {
    struct {
        uint8_t SYS_INITED          : 1;//系统是否初始化完成
        uint8_t SYS_INLINE          : 1;//是否处于巡线状态
        uint8_t SYS_MOVING          : 1;//是否正在位移
        uint8_t SYS_ROTATING        : 1;//是否正在旋转
        uint8_t SYS_ACTING          : 1;//是否正在动作
        uint8_t SYS_PUMPACTIVE      : 1;//气泵是否正在工作
        uint8_t SYS_SHOOTERACTING   : 1;//弹弓是否正在动作
        uint8_t SYS_MAGENABLE       : 1;//电磁铁是否正在工作
    } bits;
    uint8_t all;
} SYS_STATUS_STRUCT;

typedef struct{
    Main_CMD_ENUM CMD;
    uint8_t load[256];
    uint8_t load_len;
    uint8_t check;
    uint8_t done;
}Main_COM_Package_t;

typedef struct{
    uint8_t Movement_type;
    Car_Movement_struct Movement;
}CMD_SETM_load_struct;


#define REQ_YAW     (0x01)
#define REQ_TOF     (0x02)
#define REQ_PUMP    (0x03)
#define REQ_MILE    (0x04)//下位机回复一个float，指示里程计值，单位为脉冲数，非物理单位
typedef enum{
    REQ_type_YAW     =REQ_YAW,//下位机回复一个float，指示航向角
    REQ_type_Tof     =REQ_TOF,//下位机回复一个float，指示距离
    REQ_type_Pump    =REQ_PUMP,//下位机回复一个float，指示吸附程度(0-10000，越大表示吸得越紧)
    REQ_type_Mile    =REQ_MILE,//下位机回复一个float，指示里程计值，单位为脉冲数，非物理单位
}REQ_type_enum;

#define SET_YAW     (0x00)
#define SET_MILE     (0x01)//设置里程计，单位为脉冲数，非物理单位
typedef enum{
    SET_type_YAW     =SET_YAW,//上位机发送一个float，指示需要设定的航向角,下位机会回复CMD_RECVCMD
    SET_type_Mile    =SET_MILE,//上位机发送一个float，指示需要设定的里程计值，单位为脉冲数，非物理单位，下位机会回复CMD_RECVCMD
}SET_type_enum;

#pragma endregion

#pragma region PUMP_TYPEDEF_REGION
typedef struct{
    uint8_t Pump_on;
    uint8_t IfSucked;
    uint32_t SuckStrength;
}Pump_Handle_Struct;
#pragma endregion

#pragma region TOF_TYPEDEF_REGION
typedef enum{
    TOF_Status_InValid,     //无效，模块未初始化或掉线
    TOF_Status_Valid,       //有效
    TOF_Status_OverRange,   //超出量程
    TOF_Status_ERR,         //错误
}TOF_Status_Enum;

typedef struct{
    TOF_Status_Enum Status;
    uint8_t Err_Code;
    uint32_t Distance_mm;
    uint32_t Timeout_Counter;
}TOF_Handle_struct;

#pragma endregion

#pragma region SHOOTER_TYPEDEF_REGION
typedef enum{
    SHOOTER_Status_Err,         //错误状态
    SHOOTER_Status_Idle,        //待命
    SHOOTER_Status_Resetting,   //正在复位
    SHOOTER_Status_Reset,       //复位完毕
    SHOOTER_Status_Holding,     //正在蓄力
    SHOOTER_Status_Ready,       //就绪
    SHOOTER_Status_Released,    //已释放
}SHOOTER_Status_Enum;

typedef struct{
    SHOOTER_Status_Enum Status;
    uint32_t Force;
    uint8_t Uplimit_trig;
    uint8_t Last_Uplimit_trig;
    uint8_t Flag_mag_ena;
    uint8_t Flag_enable_hold; //是否允许蓄力
    uint8_t Flag_send_cmdrecv;
    uint8_t Flag_send_cmddone;
}SHOOTER_Handle_struct;

#pragma endregion


#pragma region TRIGGER_TYPEDEF_REGION
typedef enum{
    Trig_XLeft,     //Trigger1:PB9
    Trig_XRight,    //Trigger2:PB7
    Trig_ZUp,       //Trigger3:PI5
    Trig_ZDown,     //Trigger4:PI4
    Trig_Shooter,   //Trigger5:PI7
}Trigger_Type;

#pragma endregion

typedef enum
{
    RGB565_WHITE    = (0xFFFF),                                                     // 白色
    RGB565_BLACK    = (0x0000),                                                     // 黑色
    RGB565_BLUE     = (0x001F),                                                     // 蓝色
    RGB565_PURPLE   = (0xF81F),                                                     // 紫色
    RGB565_PINK     = (0xFE19),                                                     // 粉色
    RGB565_RED      = (0xF800),                                                     // 红色
    RGB565_MAGENTA  = (0xF81F),                                                     // 品红
    RGB565_GREEN    = (0x07E0),                                                     // 绿色
    RGB565_CYAN     = (0x07FF),                                                     // 青色
    RGB565_YELLOW   = (0xFFE0),                                                     // 黄色
    RGB565_BROWN    = (0xBC40),                                                     // 棕色
    RGB565_GRAY     = (0x8430),                                                     // 灰色

    RGB565_39C5BB   = (0x3616),
    RGB565_66CCFF   = (0x665F),
}rgb565_color_enum;

typedef void (*func_event)(void);
typedef struct{
	uint8_t Knob_A_Last_status;
	uint8_t last_turn;

	func_event up_event;
	func_event down_event;
}Knob_struct;

typedef struct{
	uint8_t key_last_status;
	uint8_t output_ena;
	uint8_t long_press_output_ena;

	func_event press_event;
	uint32_t   long_press_time;
}Key_struct;


#endif // !_CHASE_NUM1_TYPEDEF_H_

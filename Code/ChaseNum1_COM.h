#ifndef _CHASENUM1_COM_H_
#define _CHASENUM1_COM_H_

#include "stm32h7xx_hal.h"
#include "ChaseNum1_Typedef.h"

/*
    通信错误处理：收到上位机的指令，必须有返回，只有三种情况：确认命令已下发、上位机指令错误或无法执行（设备正忙）、设备就位

    对于开环任务比如移动舵机，完成输出后就返回确认指令；
    对于闭环任务比如移动步进电机，如果与驱动通信错误，自动重发包；
    如果持续丢包或驱动无响应，报告通信错误，如果一直没有位移，报告硬件故障；
*/

//动作控制指令：CMD_SETA
/*
    CMD_SETA指令载荷格式：

    第一个字节表示控制的元件类型，由Action_type_ENUM给出：
    ******************************************************************************
    typedef enum{
        Action_Lift_Z           =0x60,      //云台Z轴步进电机，输入0-10000，表示从最低到最高
        Action_Lift_X           =0x61,      //云台X轴步进电机，输入-10000-+10000，表示沿前进方向看从最左到最右，中间为0
        Action_Gripper_yaw      =0x62,      //夹爪yaw轴舵机，输入-90-+90，表示以-Y方向为零度的夹爪偏航角度
        Action_Gripper_grip     =0x63,      //夹爪夹紧舵机，输入0-10000，表示从开口最大到最小，即夹紧程度
    }Action_type_ENUM;
    ******************************************************************************

    第二个字节表示控制方法类型，由Action_method_ENUM给出：
    ******************************************************************************
    typedef enum{
        Action_method_absolute           =0x70,     //绝对式控制，如果超过限幅会被自动限制到最大值执行
        Action_method_incremental        =0x71,     //增量式控制，如果增量后超过限幅会被自动限制到最大值执行
    }Action_method_ENUM;
    ******************************************************************************

    第3-6个字节是一个float，表示传递的值
*/

/*
    动作控制响应：
        下位机收到指令后，如果目标设备空闲，则向目标设备下发指令并向上位机返回CMD_ACTS指令，载荷为模块的Action_type_ENUM
        如果目标设备正忙或错误，则返回CMD_ACTE,载荷为模块的Action_type_ENUM
        对于正忙的设备，可以发送CMD_INTA，载荷为模块的Action_type_ENUM来打断动作

        对于舵机控制设备，只返回CMD_ACTS，不存在设备忙的情况；
        对于步进电机控制的设备，返回CMD_ACTS表示正确下发指令，但是设备不会立即到位，此时设备正忙，不允许再发送指令；

        步进电机控制的设备到位后，会发送CMD_ACTD指令，载荷为就位模块的Action_type_ENUM，此时设备被释放到空闲状态，允许再发送指令；

        发生严重错误时发送CMD_ERR，载荷内容由ERR_ENUM给出//暂未实现
*/


// /******************************上位机指令*****************************/
// #define CMD_REQ   (0xF1)    //上位机要求回复        Request             无载荷
// #define CMD_STA   (0xF2)    //上位机询问状态        Status              无载荷
// #define CMD_STT   (0xF3)    //上位机要求进入任务    Start               无载荷
// #define CMD_RST   (0xF4)    //上位机要求复位任务    Reset               无载荷
// #define CMD_MOTD  (0xF5)    //上位机要求紧急停止    Motor Disable       无载荷
// #define CMD_INT   (0xF6)    //上位机要求打断动作    Interrupt           无载荷

// #define CMD_SETM  (0xFA)    //上位机发送运动指令    Set Movement        有载荷
// #define CMD_SETA  (0xFB)    //上位机发送动作指令    Set Action          有载荷
// #define CMD_INTA  (0xFC)    //上位机打断动作指令    Action Interrupt    有载荷

// /******************************下位机指令*****************************/
// #define CMD_REP   (0xE1)    //回复                 Reply               无载荷
// #define CMD_SUC   (0xE2)    //确认/成功            Success             无载荷
// #define CMD_FAL   (0xE3)    //错误/失败            Fail                无载荷
// #define CMD_ASTA  (0xE4)    //下位机返回状态        Answer Status       有载荷
// #define CMD_MOV   (0xE5)    //正在执行运动指令      Moving              有载荷
// #define CMD_MOVD  (0xE6)    //运动指令完成          Movement Done       无载荷
// #define CMD_ACTS  (0xE7)    //动作指令收到          Action Set         有载荷
// #define CMD_ACTD  (0xE8)    //动作指令完成          Action Done        有载荷
// #define CMD_ACTE  (0xE9)    //动作指令错误          Action Err         有载荷
// #define CMD_ERR   (0xD1)    //发生严重错误          Error        有载荷


void Main_COM_init(void);//通信初始化
void Main_COM_Response_CALLBACK(void);//处理指令
void CN_send_Response(Main_CMD_ENUM CMD,uint8_t* response);//发送应答
void Main_COM_send_pack(Main_CMD_ENUM CMD,uint8_t *load,uint8_t load_len);//发送数据包

/*
    CMD_SETM指令载荷格式：
    第一个字节表示运动类型，由Movement_type_ENUM给出:
        typedef enum{
            Movement_Translation    =0x50,   
            Movement_Rotation       =0x51, 
            Movement_Set_speed      =0x60,
            Movement_Set_angvel     =0x61,
            Movement_Translation_Precise =0x52, //平移精确模式，要求在Translation_Tolerance_PRECISE范围内到位
            Movement_Rotation_Precise     =0x53, //旋转精确模式，要求在Rotation_Tolerance_PRECISE范围内到位
        }Movement_type_ENUM;

    后面是五个浮点数，由Car_Movement_struct给出:
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
*/


typedef enum{
    ERR_SM1=0x01,
    ERR_SM2=0x02,
}ERR_ENUM;

#define IS_RX_MAIN_CMD(CMD) (((CMD) == CMD_HELLO)    || \
                        ((CMD) == CMD_STATUS)      || \
                        ((CMD) == CMD_RESET)      || \
                        ((CMD) == CMD_CHECK)      || \
                        ((CMD) == CMD_RESTART)     || \
                        ((CMD) == CMD_INLINE)      || \
                        ((CMD) == CMD_OUTLINE)     || \
                        ((CMD) == CMD_SETPARA)      || \
                        ((CMD) == CMD_GETPARA)      || \
                        ((CMD) == CMD_SETMOVEMENT)     || \
                        ((CMD) == CMD_SETACTION)      || \
                        ((CMD) == CMD_SERVERSTATUS)      || \
                        ((CMD) == CMD_INTERRUPT))

#define IS_TX_MAIN_CMD(CMD) (((CMD) == CMD_REP)    || \
                        ((CMD) == CMD_SUC)      || \
                        ((CMD) == CMD_FAL)      || \
                        ((CMD) == CMD_ASTA)      || \
                        ((CMD) == CMD_MOV)     || \
                        ((CMD) == CMD_MOVD)   ||\
                        ((CMD) == CMD_ACTS)     || \
                        ((CMD) == CMD_ACTD)     || \
                        ((CMD) == CMD_ACTE)     || \
                        ((CMD) == CMD_ERR))
                                                                            

#define IS_CMD_WITH_LOAD(CMD) (((CMD) == CMD_SETM)    || \
                        ((CMD) == CMD_SETA)     || \
                        ((CMD) == CMD_REQ)     || \
                        ((CMD) == CMD_SET)     || \
                        ((CMD) == CMD_INTA))


/*
    上位机通讯协议：
        上下位机通过UART发送数据包的形式通讯
        数据链路层:UART BAUD:115200，8N1 无流控，无校验

        一个数据包包括六部分：帧头、指令、载荷长度、载荷、校验、帧尾
        帧头：0xAA 0xBB
        指令：由如上宏定义给出
        载荷长度：用一个字节(uint8_t)表示载荷字节数，如果是无载荷的指令则必须为0
        载荷：按照指定的格式存储具体数据
        校验：将载荷中的所有内容按字节(uint8_t)求和，允许溢出，最终保存为一个字节(uint8_t)
        帧尾：0x0F 0x0A
*/
/*要求回复*/
//指令示例: AA BB   F1    00         00      0F 0A
//         帧头    指令   载荷长度    校验     帧尾

/*请求状态*/
//指令示例: AA BB   F2    00         00      0F 0A
//         帧头    指令   载荷长度    校验     帧尾

/*要求进入任务状态*/
//指令示例: AA BB   F3    00         00      0F 0A
//         帧头    指令   载荷长度    校验     帧尾

/*设置运动参数*/
//指令示例: AA BB   FA    15        50           00 00 80 3F     00 00 00 40    00 00 00 00     00 00 00 00   00 00 00 00   4F      0F 0A
//         帧头    指令   载荷长度   运动类型      X轴位移         Y轴位移         Z轴位移         直线速度       旋转速度      校验     帧尾

/*设置运动参数*/
//指令示例: AA BB   FA    15        51           00 00 00 00     00 00 00 00    00 00 34 43     00 00 00 00   00 00 00 00   C8      0F 0A
//         帧头    指令   载荷长度   运动类型      X轴位移         Y轴位移         Z轴位移         直线速度       旋转速度      校验     帧尾
/*
    运行流程：
    上下位机上电，下位机初始化完不启动电机，待命等待上位机信号
    上位机初始化完后，发送CMD_REQ确认下位机在线，下位机收到后回复CMD_REP
    上位机发送CMD_STA确认下位机处于刚初始化完状态，下位机收到后回复CMD_ASTA，
    若状态不对则发送CMD_RST重置状态并再次检查
    上位机等待用户发出发车命令，并发送CMD_STT要求下位机进入任务状态，下位机收到后回复CMD_SUC或CMD_FAL
    上位机发送CMD_SETM要求下位机做出动作，下位机收到后回复CMD_MOV并返回正在执行的动作的数据包校验和，
    如果和上位机校验和不同，则上位机可以发送CMD_INT打断动作，下位机收到后要回复CMD_SUC并中断动作，上位机可以再次发送动作指令
    下位机在执行动作期间必须定时发送CMD_MOV，如果上位机接收指令超时可能会引发询问或复位
    下位机执行完成动作后发送CMD_MOVD并进入待命状态，如果在执行过程中上位机改变指令可以发送CMD_INT打断动作并在收到回复后重新发送指令
    如果上位机在任务中出现无法控制的情况可以发送CMD_RST或CMD_MOTD复位或强制停止，
    在任何时候发送CMD_REQ，下位机收到后都必须回复CMD_REP，否则可能会引发复位
*/

#endif // !_CHASENUM1_COM_H_
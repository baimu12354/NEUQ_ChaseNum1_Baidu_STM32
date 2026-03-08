#include "Emm_V5.h"
#include "../../TopSpeed_StepMotor.h"

/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

/**
  * @brief    将当前位置清零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Reset_CurPos_To_Zero(SM_NO_ENUM SM_SELECT,uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0A;                       // 功能码
  cmd[2] =  0x6D;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
	// HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 4);
  TS_SM_SET_CMD(SM_SELECT, (uint8_t *)cmd, 4);
}

/**
  * @brief    解除堵转保护
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Reset_Clog_Pro(SM_NO_ENUM SM_SELECT,uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0E;                       // 功能码
  cmd[2] =  0x52;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 4);
  TS_SM_SET_CMD(SM_SELECT, (uint8_t *)cmd, 4);
}

/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Read_Sys_Params(SM_NO_ENUM SM_SELECT,uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[i] = addr; ++i;                   // 地址

  switch(s)                             // 功能码
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节
  
  // 发送命令
  // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, i);
  TS_SM_SET_CMD(SM_SELECT, (uint8_t *)cmd, i);
}

/**
  * @brief    修改开环/闭环控制模式
  * @param    addr     ：电机地址
  * @param    svF      ：是否存储标志，false为不存储，true为存储
  * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Modify_Ctrl_Mode(SM_NO_ENUM SM_SELECT,uint8_t addr, uint8_t svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x46;                       // 功能码
  cmd[2] =  0x69;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  ctrl_mode;                  // 控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 6);
  TS_SM_SET_CMD(SM_SELECT, (uint8_t *)cmd, 6);
}

/**
  * @brief    使能信号控制
  * @param    addr  ：电机地址
  * @param    state ：使能状态     ，true为使能电机，false为关闭电机
  * @param    snF   ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_En_Control(SM_NO_ENUM SM_SELECT,uint8_t addr, uint8_t state, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF3;                       // 功能码
  cmd[2] =  0xAB;                       // 辅助码
  cmd[3] =  (uint8_t)state;             // 使能状态
  cmd[4] =  snF;                        // 多机同步运动标志
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 6);
  TS_SM_SET_CMD(SM_SELECT, (uint8_t *)cmd, 6);
}

/**
  * @brief    速度模式
  * @param    addr：电机地址
  * @param    dir ：方向       ，0为CW，其余值为CCW
  * @param    vel ：速度       ，范围0 - 5000RPM
  * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
  * @param    snF ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Vel_Control(SM_NO_ENUM SM_SELECT,uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint8_t snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 功能码
  cmd[2] =  dir;                        // 方向
  cmd[3] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
  cmd[4] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
  cmd[5] =  acc;                        // 加速度，注意：0是直接启动
  cmd[6] =  snF;                        // 多机同步运动标志
  cmd[7] =  0x6B;                       // 校验字节
  
  // 发送命令
  // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 8);
  TS_SM_SET_CMD(SM_SELECT, (uint8_t *)cmd, 8);
}

/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Pos_Control(SM_NO_ENUM SM_SELECT,uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, uint8_t snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节 
  cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  cmd[10] =  raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] =  snF;                       // 多机同步运动标志，false为不启用，true为启用
  cmd[12] =  0x6B;                      // 校验字节
  
  // 发送命令
  // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 13);
  TS_SM_SET_CMD(SM_SELECT, (uint8_t *)cmd, 13);
}

/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Stop_Now(SM_NO_ENUM SM_SELECT,uint8_t addr, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 功能码
  cmd[2] =  0x98;                       // 辅助码
  cmd[3] =  snF;                        // 多机同步运动标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 5);
  TS_SM_SET_CMD(SM_SELECT, (uint8_t *)cmd, 5);
}

/**
  * @brief    多机同步运动
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Synchronous_motion(SM_NO_ENUM SM_SELECT,uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFF;                       // 功能码
  cmd[2] =  0x66;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 4);
  TS_SM_SET_CMD(SM_SELECT, (uint8_t *)cmd,4);
}

/**
  * @brief    设置单圈回零的零点位置
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Set_O(SM_NO_ENUM SM_SELECT,uint8_t addr, uint8_t svF)
{
   uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x93;                       // 功能码
  cmd[2] =  0x88;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 5);
  TS_SM_SET_CMD(SM_SELECT, (uint8_t *)cmd, 5);
}

/**
  * @brief    修改回零参数
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    o_dir  ：回零方向，0为CW，其余值为CCW
  * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
  * @param    o_tm   ：回零超时时间，单位：毫秒
  * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
  * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
  * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
  * @param    potF   ：上电自动触发回零，false为不使能，true为使能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Modify_Params(SM_NO_ENUM SM_SELECT,uint8_t addr, uint8_t svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, uint8_t potF)
{
  uint8_t cmd[32] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x4C;                       // 功能码
  cmd[2] =  0xAE;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[5] =  o_dir;                      // 回零方向
  cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度(RPM)高8位字节
  cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度(RPM)低8位字节 
  cmd[8]  =  (uint8_t)(o_tm >> 24);     // 回零超时时间(bit24 - bit31)
  cmd[9]  =  (uint8_t)(o_tm >> 16);     // 回零超时时间(bit16 - bit23)
  cmd[10] =  (uint8_t)(o_tm >> 8);      // 回零超时时间(bit8  - bit15)
  cmd[11] =  (uint8_t)(o_tm >> 0);      // 回零超时时间(bit0  - bit7 )
  cmd[12] =  (uint8_t)(sl_vel >> 8);    // 无限位碰撞回零检测转速(RPM)高8位字节
  cmd[13] =  (uint8_t)(sl_vel >> 0);    // 无限位碰撞回零检测转速(RPM)低8位字节 
  cmd[14] =  (uint8_t)(sl_ma >> 8);     // 无限位碰撞回零检测电流(Ma)高8位字节
  cmd[15] =  (uint8_t)(sl_ma >> 0);     // 无限位碰撞回零检测电流(Ma)低8位字节 
  cmd[16] =  (uint8_t)(sl_ms >> 8);     // 无限位碰撞回零检测时间(Ms)高8位字节
  cmd[17] =  (uint8_t)(sl_ms >> 0);     // 无限位碰撞回零检测时间(Ms)低8位字节
  cmd[18] =  potF;                      // 上电自动触发回零，false为不使能，true为使能
  cmd[19] =  0x6B;                      // 校验字节
  
  // 发送命令
  // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 20);

  TS_SM_SET_CMD(SM_SELECT, (uint8_t *)cmd, 20);
}

/**
  * @brief    触发回零
  * @param    addr   ：电机地址
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Trigger_Return(SM_NO_ENUM SM_SELECT,uint8_t addr, uint8_t o_mode, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9A;                       // 功能码
  cmd[2] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[3] =  snF;                        // 多机同步运动标志，false为不启用，true为启用
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 5);
  TS_SM_SET_CMD(SM_SELECT, (uint8_t *)cmd, 5);
}

/**
  * @brief    强制中断并退出回零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Interrupt(SM_NO_ENUM SM_SELECT,uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9C;                       // 功能码
  cmd[2] =  0x48;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 5);
  TS_SM_SET_CMD(SM_SELECT, (uint8_t *)cmd, 4);
}


void TS_ZDT_SDK_Set_config(SM_NO_ENUM SM_SELECT,uint8_t addr,uint8_t save){
    uint8_t cmd[33] = {0};

	//帧头
	cmd[0]=0x00;
	cmd[1]=0x48;
	cmd[2]=0xD1;

	//配置内容
	cmd[3]=save;//不保存设置，0x01为保存
	cmd[4]=0x19;//修改电机类型为1.8°电机
	cmd[5]=0x02;//修改脉冲端口控制模式为PUL_FOC，即FOC 矢量闭环模式
	cmd[6]=0x02;//修改通讯端口复用模式为UART_FUN，即串口/RS232/RS485 通讯;
	cmd[7]=0x02;//修改En 引脚的有效电平为Hold，即一直有效
	cmd[8]=0x00;//修改Dir 引脚的有效方向为CW，即顺时针方向
	cmd[9]=0x10;//修改细分为16 细分（注意：256 细分用00 表示）
	cmd[10]=0x01;//修改细分插补功能为Enable，即使能细分插补
	cmd[11]=0x00;//修改自动熄屏功能为Disable，即不使能自动熄屏功能;
	cmd[12]=0x03;
	cmd[13]=0xE8;//修改开环模式工作电流为1000Ma
	cmd[14]=0x0B;
	cmd[15]=0xB8;//修改闭环模式堵转时的最大电流为3000Ma
	cmd[16]=0x1B;
	cmd[17]=0x58;//修改闭环模式最大输出压为7000mV
	cmd[18]=0x05;//修改串口波特率为115200（对应屏幕选项顺序：0x00 为9600，...）;
	cmd[19]=0x07;//修改CAN 通讯速率为500000（对应屏幕选项顺序：0x00 为10000，...）;
	cmd[20]=0x01;//该值原为修改ID 地址为1，现取消，使得可用广播地址批量修改驱动参数;
	cmd[21]=0x00;//修改通讯校验方式为0x6B;
	cmd[22]=0x03;//修改控制命令应答为Both，即回复命令收到和就位;
	cmd[23]=0x01;//修改堵转保护功能为Enable，即使能堵转保护
	cmd[24]=0x00;
	cmd[25]=0x28;//修改堵转保护转速阈值为40RPM（转/每分钟）;
	cmd[26]=0x09;
	cmd[27]=0x60;//修改堵转保护电流阈值为2400Ma;
	cmd[28]=0x0F;
	cmd[29]=0xA0;//修改堵转保护检测时间阈值为4000ms;
	cmd[30]=0x00;
	// cmd[31]=0x14;//修改位置到达窗口为0.5°;
  cmd[31]=0x32;//修改位置到达窗口为5°;
  cmd[31]=0x64;//修改位置到达窗口为10°;
  // cmd[31]=0xC8;//修改位置到达窗口为20°;

	//帧尾
	cmd[32]=0x6B;
  
  TS_SM_SET_CMD(SM_SELECT, (uint8_t *)cmd, 33);
}

void TS_ZDT_SDK_Read_err(SM_NO_ENUM SM_SELECT){
	uint8_t cmd[4] = {0};
  
	// 装载命令
	cmd[0] =  0x00;                       // 地址
	cmd[1] =  0x37;                       // 功能码
	cmd[2] =  0x6B;                       // 辅助码
	
	// 发送命令
	// HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, 5);
	TS_SM_SET_CMD(SM_SELECT, (uint8_t *)cmd, 3);
}

void TS_ZDT_SDK_Set_PID(SM_NO_ENUM SM_SELECT,uint8_t save_flag, uint32_t Kp, uint32_t Ki, uint32_t Kd) {
    uint8_t out_buf[17]={0};

    // 命令固定头部
    out_buf[0] = 0x00;        // 地址
    out_buf[1] = 0x4A;        // 功能码
    out_buf[2] = 0xC3;        // 命令字

    out_buf[3] = save_flag ? 0x01 : 0x00;  // 是否保存

    // 按小端模式写入 Kp（0x0000F230）
    out_buf[4] = (uint8_t)(Kp & 0xFF);
    out_buf[5] = (uint8_t)((Kp >> 8) & 0xFF);
    out_buf[6] = (uint8_t)((Kp >> 16) & 0xFF);
    out_buf[7] = (uint8_t)((Kp >> 24) & 0xFF);

    // Ki
    out_buf[8]  = (uint8_t)(Ki & 0xFF);
    out_buf[9]  = (uint8_t)((Ki >> 8) & 0xFF);
    out_buf[10] = (uint8_t)((Ki >> 16) & 0xFF);
    out_buf[11] = (uint8_t)((Ki >> 24) & 0xFF);

    // Kd
    out_buf[12] = (uint8_t)(Kd & 0xFF);
    out_buf[13] = (uint8_t)((Kd >> 8) & 0xFF);
    out_buf[14] = (uint8_t)((Kd >> 16) & 0xFF);
    out_buf[15] = (uint8_t)((Kd >> 24) & 0xFF);

    // 校验
    out_buf[16] = 0x6B;

    TS_SM_SET_CMD(SM_SELECT, (uint8_t *)out_buf, 17);
}

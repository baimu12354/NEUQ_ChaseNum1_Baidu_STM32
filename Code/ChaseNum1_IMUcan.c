#include "ChaseNum1_Include.h"

/* 全局句柄与数据对象定义 */
FDCAN_HandleTypeDef hfdcan1;
imu_t imu;

//----------------------------- 底层 CAN 硬件配置 ----------------------------- //

// FDCAN1 硬件底层初始化 (由 HAL_FDCAN_Init 自动调用)
// 配置了 PH13 和 PH14 作为 CAN 通信引脚，并指定时钟源
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    if(fdcanHandle->Instance == FDCAN1)
    {
        // 配置 FDCAN 的时钟源为 PLL1Q (根据你的系统配置，这里应该是 100MHz) 
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        {
            // Error_Handler();
        }

        // 开启对应的外设总线时钟 
        __HAL_RCC_FDCAN_CLK_ENABLE();
        __HAL_RCC_GPIOH_CLK_ENABLE(); 

        // 配置引脚复用 (PH13 -> FDCAN1_TX, PH14 -> FDCAN1_RX)
        GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;           // 复用推挽输出
        GPIO_InitStruct.Pull = GPIO_NOPULL;               // 无上下拉
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;     // 高速响应
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;      // PH13/14 对应的 FDCAN1 复用通道为 AF9
        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

        // 配置并开启 FDCAN1 中断 
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 1, 0);      // 设置中断优先级 (视系统情况调整)
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);              // 使能中断线 0
    }
}

// 达秒科技 IMU初始化函数
void IMU_CAN_Init(void)
{
    FDCAN_FilterTypeDef fdcan_filter;

    // FDCAN1 参数配置 (1Mbps 波特率)
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;   // 经典 CAN 格式 (非 CAN-FD)
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;            // 正常通讯模式
    hfdcan1.Init.AutoRetransmission = DISABLE;        // 禁用自动重发 (IMU 通信通常不需要)
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;
    
    // 波特率核心计算
    // PLL1Q = 100MHz。
    // 波特率 = 100MHz / (NominalPrescaler * (1 + NominalTimeSeg1 + NominalTimeSeg2))
    // 1Mbps = 100MHz / (20 * (1 + 2 + 2)) 
    hfdcan1.Init.NominalPrescaler = 20;               
    hfdcan1.Init.NominalSyncJumpWidth = 1;
    hfdcan1.Init.NominalTimeSeg1 = 2;
    hfdcan1.Init.NominalTimeSeg2 = 2;
    
    // 数据段波特率参数 (即使是经典 CAN 模式，H7 也需要随便赋个合法值)
    hfdcan1.Init.DataPrescaler = 1;
    hfdcan1.Init.DataSyncJumpWidth = 1;
    hfdcan1.Init.DataTimeSeg1 = 1;
    hfdcan1.Init.DataTimeSeg2 = 1;
    
    hfdcan1.Init.MessageRAMOffset = 0;                // 只用一个 CAN 时写 0 即可
    hfdcan1.Init.StdFiltersNbr = 1;                   // 1 个标准滤波器
    hfdcan1.Init.ExtFiltersNbr = 0;
    hfdcan1.Init.RxFifo0ElmtsNbr = 32;                // 接收 FIFO0 深度为 32
    hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.TxFifoQueueElmtsNbr = 32;            // 发送 FIFO 深度为 32
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
    
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    {
        // Error_Handler(); // 初始化失败陷入死循环
    }

    // 接收滤波器配置
    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_RANGE;           // 范围滤波模式
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;    // 符合条件的报文放进 FIFO0
    fdcan_filter.FilterID1 = 0x01;                          // 允许接收的最小 ID (包含 IMU 默认的 0x01)
    fdcan_filter.FilterID2 = 0xFF;                          // 允许接收的最大 ID
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter); 
    
    // 拒绝所有未匹配上滤波器的报文 
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, DISABLE, DISABLE);

    // 启动硬件
    HAL_FDCAN_Start(&hfdcan1);

    // 激活 RX FIFO0 收到新数据的中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    // 绑定 IMU 逻辑对象
    imu.can_id = 0x01;
    imu.mst_id = 0x11;
    imu.can_handle = &hfdcan1;
}
//----------------------------- 底层 CAN 硬件配置 ----------------------------- //

//----------------------------- 通信协议 ----------------------------- //
// 声明数据解包相关的内部函数
static void IMU_UpdateAccel(uint8_t* pData);
static void IMU_UpdateGyro(uint8_t* pData);
static void IMU_UpdateEuler(uint8_t* pData);
static void IMU_UpdateQuaternion(uint8_t* pData);

// FIFO0 接收中断回调函数 (由 FDCAN1_IT0_IRQHandler 触发)
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if(hfdcan->Instance == FDCAN1)
        {
            FDCAN_RxHeaderTypeDef rx_header;
            uint8_t rx_data[8] = {0};
            
            // 把数据从硬件 FIFO 里面提取到数组里 
            if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
            {
                // 判断数据帧类型 (Byte[0] 代表类型) 并进入相应的解包函数
                switch(rx_data[0])
                {
                    case 1: IMU_UpdateAccel(rx_data); break;      // 加速度
                    case 2: IMU_UpdateGyro(rx_data); break;       // 角速度
                    case 3: IMU_UpdateEuler(rx_data); break;      // 欧拉角 (俯仰、横滚、偏航)
                    case 4: IMU_UpdateQuaternion(rx_data); break; // 四元数
                }
            }
        }
    }
}

// 无符号整数转浮点数 (用于解析接收到的压缩数据)
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

// 发送指令给陀螺仪的底层函数
static void imu_send_cmd(uint8_t reg_id, uint8_t ac, uint32_t data)
{
    if(imu.can_handle == NULL) return;
    
    FDCAN_TxHeaderTypeDef tx_header;
    
    // 达妙指令格式：头(CC), 寄存器, 读写标志, 尾(DD), 数据(4Byte小端)
    uint8_t buf[8] = {0xCC, reg_id, ac, 0xDD, 0, 0, 0, 0};
    memcpy(buf + 4, &data, 4);
    
    tx_header.Identifier = imu.mst_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;                                      
    tx_header.MessageMarker = 0x00;                   

    // 检查发送邮箱是否有空位，有则压入数据
    if(HAL_FDCAN_GetTxFifoFreeLevel(imu.can_handle) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(imu.can_handle, &tx_header, buf);
    }
}

// 外部接口（可以放在中断或者循环中）
void imu_request_accel(void) { imu_send_cmd(ACCEL_DATA, CMD_READ, 0); }
void imu_request_gyro(void)  { imu_send_cmd(GYRO_DATA, CMD_READ, 0); }
void imu_request_euler(void) { imu_send_cmd(EULER_DATA, CMD_READ, 0); }
void imu_request_quat(void)  { imu_send_cmd(QUAT_DATA, CMD_READ, 0); }


// 针对不同数据的解包函数 (位操作解析)
static void IMU_UpdateAccel(uint8_t* pData)
{
    uint16_t accel[3];
    accel[0] = pData[3]<<8 | pData[2];
    accel[1] = pData[5]<<8 | pData[4];
    accel[2] = pData[7]<<8 | pData[6];
    
    imu.accel[0] = uint_to_float(accel[0], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    imu.accel[1] = uint_to_float(accel[1], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    imu.accel[2] = uint_to_float(accel[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
}

static void IMU_UpdateGyro(uint8_t* pData)
{
    uint16_t gyro[3];
    gyro[0] = pData[3]<<8 | pData[2];
    gyro[1] = pData[5]<<8 | pData[4];
    gyro[2] = pData[7]<<8 | pData[6];
    
    imu.gyro[0] = uint_to_float(gyro[0], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    imu.gyro[1] = uint_to_float(gyro[1], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    imu.gyro[2] = uint_to_float(gyro[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
}

static void IMU_UpdateEuler(uint8_t* pData)
{
    int euler[3];
    euler[0] = pData[3]<<8 | pData[2];
    euler[1] = pData[5]<<8 | pData[4];
    euler[2] = pData[7]<<8 | pData[6];
    
    imu.pitch = uint_to_float(euler[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
    imu.yaw   = uint_to_float(euler[1], YAW_CAN_MIN, YAW_CAN_MAX, 16);
    imu.roll  = uint_to_float(euler[2], ROLL_CAN_MIN, ROLL_CAN_MAX, 16);
}

static void IMU_UpdateQuaternion(uint8_t* pData)
{
    int w = pData[1]<<6 | ((pData[2]&0xF8)>>2);
    int x = (pData[2]&0x03)<<12 | (pData[3]<<4) | ((pData[4]&0xF0)>>4);
    int y = (pData[4]&0x0F)<<10 | (pData[5]<<2) | ((pData[6]&0xC0)>>6);
    int z = (pData[6]&0x3F)<<8  | pData[7];
    
    imu.q[0] = uint_to_float(w, Quaternion_MIN, Quaternion_MAX, 14);
    imu.q[1] = uint_to_float(x, Quaternion_MIN, Quaternion_MAX, 14);
    imu.q[2] = uint_to_float(y, Quaternion_MIN, Quaternion_MAX, 14);
    imu.q[3] = uint_to_float(z, Quaternion_MIN, Quaternion_MAX, 14);
}
//----------------------------- 通信协议 ----------------------------- //
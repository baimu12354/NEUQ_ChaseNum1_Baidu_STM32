#include "ChaseNum1_Include.h"
#include <math.h>
#include <string.h>

// 协议帧头第 1 字节。
#define NANO_FRAME_HEAD1                 0xAAU
// 协议帧头第 2 字节。
#define NANO_FRAME_HEAD2                 0x55U
// 协议帧尾第 1 字节。
#define NANO_FRAME_TAIL1                 0x0DU
// 协议帧尾第 2 字节。
#define NANO_FRAME_TAIL2                 0x0AU
// 当前协议版本。
#define NANO_FRAME_VERSION               0x01U
// 命令接收载荷最大长度，当前最长命令为 4 个 float。
#define NANO_RX_PAYLOAD_MAX              32U
// 回包发送载荷最大长度，用于状态包和机械臂位姿包。
#define NANO_TX_PAYLOAD_MAX              64U
// NANO 协议接收软件环形缓冲区大小，必须为 2 的整数次幂，协议和外部接口保持不变。
#define NANO_RX_RING_SIZE                512U
// 每次主循环最多解析的 NANO 协议字节数，避免一次处理过久影响其它任务。
#define NANO_RX_PARSE_BUDGET             64U
#if ((NANO_RX_RING_SIZE & (NANO_RX_RING_SIZE - 1U)) != 0U)
#error "NANO_RX_RING_SIZE must be power of two"
#endif
// 底盘速度命令超时时间，单位 ms。
#define NANO_CHASSIS_RUN_TIMEOUT_MS      500U
// 气泵放下等待时间，单位 ms，后续根据实测释放时间调整。
#define NANO_PUMP_RELEASE_WAIT_MS        500U

// 上位机 running 横纵速度输入范围，单位为上位机百分量，允许带正负方向。
#define NANO_UP_RUN_SPEED_MAX            100.0f
// 下位机 running 横纵速度内部最小非零值。
#define NANO_DN_RUN_SPEED_MIN            1.0f
// 下位机 running 横纵速度内部最大值。
#define NANO_DN_RUN_SPEED_MAX            20.0f

// 上位机 running 角速度输入范围，单位为上位机百分量，允许带正负方向。
#define NANO_UP_RUN_WZ_MAX               100.0f
// 下位机 running 角速度内部最大值，RUN 模式角速度直接进入底盘解算。
#define NANO_DN_RUN_WZ_MAX               200.0f
// 上位机定角旋转最大角速度输入最小值。
#define NANO_UP_ROT_WZ_MIN               1.0f
// 上位机定角旋转最大角速度输入最大值。
#define NANO_UP_ROT_WZ_MAX               100.0f
//+下位机定角旋转最大角速度内部最小值。
#define NANO_DN_ROT_WZ_MIN               1.0f
//+下位机定角旋转最大角速度内部最大值。
#define NANO_DN_ROT_WZ_MAX               10.0f
//+上位机定向平移距离输入最大值，单位 cm，允许带正负方向。
#define NANO_UP_TRANS_DIST_MAX_CM        1000.0f
//+下位机定向平移距离内部最大值。
#define NANO_DN_TRANS_DIST_MAX           50000.0f
//+上位机定向平移速度输入最小值。
#define NANO_UP_TRANS_SPEED_MIN          1.0f
//+上位机定向平移速度输入最大值。
#define NANO_UP_TRANS_SPEED_MAX          100.0f
//+下位机定向平移速度内部最小值。
#define NANO_DN_TRANS_SPEED_MIN          1.0f
//+下位机定向平移速度内部最大值。
#define NANO_DN_TRANS_SPEED_MAX          20.0f

//+模块 ID：系统。
#define NANO_MODULE_SYSTEM               0x01U
//+模块 ID：底盘。
#define NANO_MODULE_CHASSIS              0x02U
//+模块 ID：机械臂。
#define NANO_MODULE_ARM                  0x03U
//+模块 ID：气泵。
#define NANO_MODULE_PUMP                 0x04U

//+系统命令：握手。
#define NANO_CMD_SYS_HELLO               0x01U
//+系统命令：复位外设。
#define NANO_CMD_SYS_RESET               0x02U
//+系统命令：急停锁止。
#define NANO_CMD_SYS_ESTOP               0x03U
//+系统命令：读取系统状态。
#define NANO_CMD_SYS_GET_STATUS          0x04U

//+底盘命令：周期速度控制，载荷为 vx、vy、wz 三个 float。
#define NANO_CMD_CHASSIS_RUN             0x10U
//+底盘命令：普通停止，零速度姿态控制。
#define NANO_CMD_CHASSIS_STOP            0x11U
//+底盘命令：刹车。
#define NANO_CMD_CHASSIS_BRAKE           0x12U
//+底盘命令：相对当前角度旋转，载荷为 angle、max_wz 两个 float。
#define NANO_CMD_CHASSIS_ROTATE_TO       0x13U
//+底盘命令：相对当前位置平移，载荷为 dx、dy、speed 三个 float。
#define NANO_CMD_CHASSIS_TRANSLATE_REL   0x14U
//+底盘命令：读取底盘状态。
#define NANO_CMD_CHASSIS_GET_STATE       0x15U

//+机械臂命令：吸盘末端目标，载荷为 x、y、z、dir 四个 float。
#define NANO_CMD_ARM_TARGET_XYZ_DIR      0x20U
//+机械臂命令：回默认休息姿态。
#define NANO_CMD_ARM_FOLD_HOME           0x21U
//+机械臂命令：停止并锁持。
#define NANO_CMD_ARM_STOP_HOLD           0x22U
//+机械臂命令：读取逻辑位姿。
#define NANO_CMD_ARM_GET_POSE            0x23U

//+气泵命令：开始吸附。
#define NANO_CMD_PUMP_SUCK               0x30U
//+气泵命令：放下工件。
#define NANO_CMD_PUMP_RELEASE            0x31U
//+气泵命令：读取气泵状态。
#define NANO_CMD_PUMP_GET_STATE          0x32U

//+通用响应：命令已接收。
#define NANO_RESP_ACK                    0x80U
//+通用响应：命令被拒绝。
#define NANO_RESP_NACK                   0x81U
//+通用响应：系统状态。
#define NANO_RESP_STATUS                 0x82U
//+通用响应：异步事件。
#define NANO_RESP_EVENT                  0x83U
//+通用响应：机械臂逻辑位姿。
#define NANO_RESP_ARM_POSE               0x84U
//+通用响应：底盘状态。
#define NANO_RESP_CHASSIS_STATE          0x85U
//+通用响应：气泵状态。
#define NANO_RESP_PUMP_STATE             0x86U

//+错误码：无错误。
#define NANO_ERR_OK                      0x00U
//+错误码：未知命令。
#define NANO_ERR_BAD_CMD                 0x01U
//+错误码：载荷长度错误。
#define NANO_ERR_BAD_LEN                 0x02U
//+错误码：校验错误。
#define NANO_ERR_BAD_CHECK               0x03U
//+错误码：系统或模块状态不允许。
#define NANO_ERR_BAD_STATE               0x04U
//+错误码：模块忙。
#define NANO_ERR_BUSY                    0x05U
//+错误码：目标超出范围。
#define NANO_ERR_RANGE_LIMIT             0x06U
//+错误码：目标落入禁止区域。
#define NANO_ERR_FORBIDDEN_ZONE          0x07U
//+错误码：命令超时。
#define NANO_ERR_TIMEOUT                 0x08U
//+错误码：设备异常。
#define NANO_ERR_DEVICE                  0x09U

//+系统事件：上电初始化完成。
#define NANO_EVENT_SYS_READY             0x01U
//+系统事件：复位完成。
#define NANO_EVENT_SYS_RESET_DONE        0x02U
//+系统事件：已经进入急停。
#define NANO_EVENT_SYS_ESTOP             0x03U
//+底盘事件：旋转到位。
#define NANO_EVENT_CHASSIS_ROTATE_DONE   0x10U
//+底盘事件：平移到位。
#define NANO_EVENT_CHASSIS_TRANS_DONE    0x11U
//+底盘事件：速度命令超时后普通停止。
#define NANO_EVENT_CHASSIS_TIMEOUT_STOP  0x12U
//+机械臂事件：到达目标。
#define NANO_EVENT_ARM_DONE              0x20U
//+机械臂事件：默认休息姿态到位。
#define NANO_EVENT_ARM_FOLD_DONE         0x21U
//+机械臂事件：命令被拒绝。
#define NANO_EVENT_ARM_REJECTED          0x22U
//+气泵事件：吸附完成。
#define NANO_EVENT_PUMP_SUCK_DONE        0x30U
//+气泵事件：放下完成。
#define NANO_EVENT_PUMP_RELEASE_DONE     0x31U
//+气泵事件：超过最长工作时间后自动停泵。
#define NANO_EVENT_PUMP_TIMEOUT_STOP     0x32U

//+系统状态：正在初始化。
#define NANO_SYS_INIT                    0U
//+系统状态：正在复位外设。
#define NANO_SYS_RET                     1U
//+系统状态：空闲。
#define NANO_SYS_IDLE                    2U
//+系统状态：有模块正在动作。
#define NANO_SYS_ACTIVE                  3U
//+系统状态：急停锁止。
#define NANO_SYS_ESTOP                   4U
//+系统状态：故障预留。
#define NANO_SYS_FAULT                   5U

//+底盘状态：空闲或零速度保持。
#define NANO_CHASSIS_IDLE                0U
//+底盘状态：周期速度运行。
#define NANO_CHASSIS_RUN                 1U
//+底盘状态：旋转中。
#define NANO_CHASSIS_ROTATE              2U
//+底盘状态：平移中。
#define NANO_CHASSIS_TRANSLATE           3U
//+底盘状态：普通停止。
#define NANO_CHASSIS_STOP                4U
//+底盘状态：刹车中。
#define NANO_CHASSIS_BRAKE               5U
//+底盘状态：故障预留。
#define NANO_CHASSIS_FAULT               6U

//+机械臂状态：空闲。
#define NANO_ARM_IDLE                    0U
//+机械臂状态：正在到目标点。
#define NANO_ARM_MOVING                  1U
//+机械臂状态：正在收敛。
#define NANO_ARM_FOLDING                 2U
//+机械臂状态：停止锁持。
#define NANO_ARM_HOLD                    3U
//+机械臂状态：命令被拒绝。
#define NANO_ARM_REJECTED                4U
//+机械臂状态：故障预留。
#define NANO_ARM_FAULT                   5U

//+气泵状态：空闲。
#define NANO_PUMP_IDLE                   0U
//+气泵状态：吸附中。
#define NANO_PUMP_SUCKING                1U
//+气泵状态：已经吸附。
#define NANO_PUMP_SUCKED                 2U
//+气泵状态：正在放下。
#define NANO_PUMP_RELEASING              3U
//+气泵状态：已经放下。
#define NANO_PUMP_RELEASED               4U
//+气泵状态：故障预留。
#define NANO_PUMP_FAULT                  5U

typedef struct {
    uint8_t seq;
    uint8_t module;
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[NANO_RX_PAYLOAD_MAX];
} NanoPackage_t;

typedef enum {
    NANO_RX_HEAD1 = 0,
    NANO_RX_HEAD2,
    NANO_RX_VER,
    NANO_RX_SEQ,
    NANO_RX_MODULE,
    NANO_RX_CMD,
    NANO_RX_LEN,
    NANO_RX_PAYLOAD,
    NANO_RX_CHECK,
    NANO_RX_TAIL1,
    NANO_RX_TAIL2
} NanoRxState_t;

static volatile uint8_t g_nano_package_ready = 0U;
static NanoPackage_t g_nano_rx_package = {0};
static NanoPackage_t g_nano_ready_package = {0};
static NanoRxState_t g_nano_rx_state = NANO_RX_HEAD1;
static uint8_t g_nano_rx_index = 0U;
static uint8_t g_nano_rx_check = 0U;
static volatile uint8_t g_nano_rx_ring[NANO_RX_RING_SIZE] = {0U};
static volatile uint16_t g_nano_rx_ring_head = 0U;
static volatile uint16_t g_nano_rx_ring_tail = 0U;
static volatile uint32_t g_nano_rx_ring_drop_count = 0U;
static volatile uint32_t g_nano_rx_byte_count = 0U;
static uint32_t g_nano_last_loop_ms = 0U;
static uint32_t g_nano_task_loop_count = 0U;
static uint32_t g_nano_rx_frame_count = 0U;
static uint32_t g_nano_rx_process_count = 0U;
static uint32_t g_nano_rx_busy_drop_count = 0U;
static uint32_t g_nano_rx_bad_check_count = 0U;
static uint32_t g_nano_rx_bad_tail_count = 0U;
static uint32_t g_nano_rx_bad_len_count = 0U;
static uint32_t g_nano_tx_frame_count = 0U;

static uint32_t g_nano_ms = 0U;

static uint8_t g_system_state = NANO_SYS_INIT;
static uint8_t g_chassis_state = NANO_CHASSIS_IDLE;
static uint8_t g_arm_state = NANO_ARM_IDLE;
static uint8_t g_pump_state = NANO_PUMP_IDLE;

static uint8_t g_chassis_run_active = 0U;
static uint32_t g_chassis_last_run_ms = 0U;
static uint8_t g_chassis_action_seq = 0U;

static uint8_t g_arm_action_seq = 0U;
static uint8_t g_pump_action_seq = 0U;
static uint8_t g_reset_active = 0U;
static uint8_t g_reset_arm_wait = 0U;
static uint8_t g_reset_pump_wait = 0U;
static uint32_t g_pump_release_start_ms = 0U;

static uint8_t Nano_COM_Calc_Check(uint8_t ver, uint8_t seq, uint8_t module,
                                   uint8_t cmd, uint8_t len, const uint8_t *payload);
static void Nano_COM_Send_Frame(uint8_t module, uint8_t cmd, uint8_t seq,
                                const uint8_t *payload, uint8_t len);
static void Nano_COM_Send_Ack(uint8_t seq, uint8_t module, uint8_t cmd, uint8_t state);
static void Nano_COM_Send_Nack(uint8_t seq, uint8_t module, uint8_t cmd, uint8_t err, uint8_t state);
static void Nano_COM_Send_Event(uint8_t seq, uint8_t module, uint8_t event, uint8_t state, uint8_t result);
static void Nano_COM_Send_Status(uint8_t seq);
static void Nano_COM_Send_Chassis_State(uint8_t seq);
static void Nano_COM_Send_Arm_Pose(uint8_t seq);
static void Nano_COM_Send_Pump_State(uint8_t seq);
static void Nano_COM_Process_Package(const NanoPackage_t *package);
static uint8_t Nano_COM_System_Allows_Command(uint8_t module, uint8_t cmd);
static void Nano_COM_Start_Reset(uint8_t seq);
static void Nano_COM_Update_System_State(void);
static void Nano_COM_Update_Chassis(void);
static void Nano_COM_Update_Arm(void);
static void Nano_COM_Update_Pump(void);
static void Nano_COM_Reset_Rx_Parser(void);
static uint8_t Nano_COM_Rx_Ring_Push(uint8_t dat);
static uint8_t Nano_COM_Rx_Ring_Pop(uint8_t *dat);
static void Nano_COM_Parse_Rx_Bytes(void);
static void Nano_COM_Parse_Byte(uint8_t dat);
static float Nano_COM_Get_Float_LE(const uint8_t *src);
static void Nano_COM_Put_Float_LE(uint8_t *dst, float value);
static void Nano_COM_Put_U32_LE(uint8_t *dst, uint32_t value);
static float Nano_COM_Limit_Abs_Float(float value, float max_abs);
static float Nano_COM_Map_Signed_Linear(float value, float up_max_abs, float dn_max_abs);
static float Nano_COM_Map_Signed_Zero_Min(float value, float up_max_abs,
                                          float dn_min_abs, float dn_max_abs);
static float Nano_COM_Map_Unsigned_Min(float value, float up_min, float up_max,
                                       float dn_min, float dn_max);
static float Nano_COM_Unmap_Signed_Linear(float value, float dn_max_abs, float up_max_abs);
static float Nano_COM_Unmap_Signed_Zero_Min(float value, float dn_min_abs,
                                            float dn_max_abs, float up_max_abs);

// 协议初始化，清空解析状态并根据 NANO_PROTOCOL_UART_SELECT 选择通信串口。
void Nano_COM_Init(void)
{ 
#if NANO_PROTOCOL_UART_SELECT == NANO_PROTOCOL_UART2
    CN_UART2_init();
#elif NANO_PROTOCOL_UART_SELECT == NANO_PROTOCOL_UART7
    CN_UART7_init();
#endif
    g_system_state = NANO_SYS_INIT;
    g_chassis_state = NANO_CHASSIS_IDLE;
    g_arm_state = NANO_ARM_IDLE;
    g_pump_state = NANO_PUMP_IDLE;
    g_nano_package_ready = 0U;
    Nano_COM_Reset_Rx_Parser();
    __disable_irq();
    g_nano_rx_ring_head = 0U;
    g_nano_rx_ring_tail = 0U;
    g_nano_rx_ring_drop_count = 0U;
    g_nano_rx_byte_count = 0U;
    __enable_irq();
    g_nano_last_loop_ms = 0U;
    g_nano_task_loop_count = 0U;
    g_nano_rx_frame_count = 0U;
    g_nano_rx_process_count = 0U;
    g_nano_rx_busy_drop_count = 0U;
    g_nano_rx_bad_check_count = 0U;
    g_nano_rx_bad_tail_count = 0U;
    g_nano_rx_bad_len_count = 0U;
    g_nano_tx_frame_count = 0U;
}

// 主流程确认所有外设初始化完成后调用。
void Nano_COM_Notify_Init_Done(void)
{
    g_system_state = NANO_SYS_IDLE;
    Nano_COM_Send_Event(0U, NANO_MODULE_SYSTEM, NANO_EVENT_SYS_READY, g_system_state, NANO_ERR_OK);
    Nano_COM_Send_Status(0U);
}

// 1ms 定时任务，只做轻量计数，避免在中断里处理协议业务。
void Nano_COM_1ms_Tick(void)
{
    g_nano_ms++;
}

// 读取协议诊断快照，供屏幕状态页显示。
void Nano_COM_Get_Diag(Nano_COM_Diag_t *diag)
{
    uint16_t head = 0U;
    uint16_t tail = 0U;

    if (diag == NULL) return;
    memset(diag, 0, sizeof(Nano_COM_Diag_t));

    __disable_irq();
    head = g_nano_rx_ring_head;
    tail = g_nano_rx_ring_tail;
    diag->rx_drop_count = g_nano_rx_ring_drop_count;
    diag->rx_byte_count = g_nano_rx_byte_count;
    __enable_irq();

    diag->ms_tick = g_nano_ms;
    diag->last_loop_ms = g_nano_last_loop_ms;
    diag->task_loop_count = g_nano_task_loop_count;
    diag->rx_frame_count = g_nano_rx_frame_count;
    diag->rx_process_count = g_nano_rx_process_count;
    diag->rx_busy_drop_count = g_nano_rx_busy_drop_count;
    diag->rx_bad_check_count = g_nano_rx_bad_check_count;
    diag->rx_bad_tail_count = g_nano_rx_bad_tail_count;
    diag->rx_bad_len_count = g_nano_rx_bad_len_count;
    diag->tx_frame_count = g_nano_tx_frame_count;
    diag->rx_ring_occupied = (head >= tail) ?
        (uint16_t)(head - tail) : (uint16_t)(NANO_RX_RING_SIZE - (tail - head));
    diag->package_ready = g_nano_package_ready;
    diag->rx_state = (uint8_t)g_nano_rx_state;
    diag->system_state = g_system_state;
    diag->chassis_state = g_chassis_state;
    diag->arm_state = g_arm_state;
    diag->pump_state = g_pump_state;
}

// 主循环任务，处理收包和各模块状态变化。
void Nano_COM_Task_Loop(void)
{
    NanoPackage_t package;

    g_nano_task_loop_count++;
    g_nano_last_loop_ms = HAL_GetTick();
    Nano_COM_Parse_Rx_Bytes();

    if (g_nano_package_ready != 0U) {
        __disable_irq();
        memcpy(&package, &g_nano_ready_package, sizeof(package));
        g_nano_package_ready = 0U;
        __enable_irq();

        Nano_COM_Process_Package(&package);
        g_nano_rx_process_count++;
    }

    Arm_Task_Loop();
    CN_Pump_Task_Loop();

    Nano_COM_Update_Chassis();
    Nano_COM_Update_Arm();
    Nano_COM_Update_Pump();
    Nano_COM_Update_System_State();
}

// 清空 NANO 协议解析状态机。
static void Nano_COM_Reset_Rx_Parser(void)
{
    g_nano_rx_state = NANO_RX_HEAD1;
    g_nano_rx_index = 0U;
    g_nano_rx_check = 0U;
    memset(&g_nano_rx_package, 0, sizeof(g_nano_rx_package));
}

// NANO 协议接收中断写入软件环形缓冲区，满时丢弃最旧字节，保留最新数据流。
static uint8_t Nano_COM_Rx_Ring_Push(uint8_t dat)
{
    uint16_t next_head = (uint16_t)((g_nano_rx_ring_head + 1U) & (NANO_RX_RING_SIZE - 1U));

    if (next_head == g_nano_rx_ring_tail) {
        g_nano_rx_ring_tail = (uint16_t)((g_nano_rx_ring_tail + 1U) & (NANO_RX_RING_SIZE - 1U));
        g_nano_rx_ring_drop_count++;
    }

    g_nano_rx_ring[g_nano_rx_ring_head] = dat;
    g_nano_rx_ring_head = next_head;
    return 1U;
}

// 从 NANO 协议软件环形缓冲区取出一个字节。
static uint8_t Nano_COM_Rx_Ring_Pop(uint8_t *dat)
{
    if (dat == NULL) return 0U;

    __disable_irq();
    if (g_nano_rx_ring_head == g_nano_rx_ring_tail) {
        __enable_irq();
        return 0U;
    }

    *dat = g_nano_rx_ring[g_nano_rx_ring_tail];
    g_nano_rx_ring_tail = (uint16_t)((g_nano_rx_ring_tail + 1U) & (NANO_RX_RING_SIZE - 1U));
    __enable_irq();
    return 1U;
}

// 主循环按预算消费 NANO 协议软件缓冲区中的字节。
static void Nano_COM_Parse_Rx_Bytes(void)
{
    uint8_t dat = 0U;
    uint8_t count = 0U;

    while ((count < NANO_RX_PARSE_BUDGET) && (g_nano_package_ready == 0U) &&
           (Nano_COM_Rx_Ring_Pop(&dat) != 0U)) {
        Nano_COM_Parse_Byte(dat);
        count++;
    }
}

// NANO 协议接收回调：中断内只入队，不在中断内解析协议业务。
void Nano_COM_RX_Callback(uint8_t dat)
{
    g_nano_rx_byte_count++;
    (void)Nano_COM_Rx_Ring_Push(dat);
}

// NANO 协议字节接收解析状态机。
static void Nano_COM_Parse_Byte(uint8_t dat)
{
    switch (g_nano_rx_state) {
        case NANO_RX_HEAD1: {
            if (dat == NANO_FRAME_HEAD1) {
                g_nano_rx_state = NANO_RX_HEAD2;
            }
            break;
        }

        case NANO_RX_HEAD2: {
            g_nano_rx_state = (dat == NANO_FRAME_HEAD2) ? NANO_RX_VER : NANO_RX_HEAD1;
            break;
        }

        case NANO_RX_VER: {
            if (dat == NANO_FRAME_VERSION) {
                g_nano_rx_check = dat;
                g_nano_rx_state = NANO_RX_SEQ;
            } else {
                g_nano_rx_state = NANO_RX_HEAD1;
            }
            break;
        }

        case NANO_RX_SEQ: {
            g_nano_rx_package.seq = dat;
            g_nano_rx_check += dat;
            g_nano_rx_state = NANO_RX_MODULE;
            break;
        }

        case NANO_RX_MODULE: {
            g_nano_rx_package.module = dat;
            g_nano_rx_check += dat;
            g_nano_rx_state = NANO_RX_CMD;
            break;
        }

        case NANO_RX_CMD: {
            g_nano_rx_package.cmd = dat;
            g_nano_rx_check += dat;
            g_nano_rx_state = NANO_RX_LEN;
            break;
        }

        case NANO_RX_LEN: {
            g_nano_rx_package.len = dat;
            g_nano_rx_index = 0U;
            g_nano_rx_check += dat;
            if (dat > NANO_RX_PAYLOAD_MAX) {
                g_nano_rx_bad_len_count++;
                g_nano_rx_state = NANO_RX_HEAD1;
            } else if (dat == 0U) {
                g_nano_rx_state = NANO_RX_CHECK;
            } else {
                g_nano_rx_state = NANO_RX_PAYLOAD;
            }
            break;
        }

        case NANO_RX_PAYLOAD: {
            g_nano_rx_package.payload[g_nano_rx_index++] = dat;
            g_nano_rx_check += dat;
            if (g_nano_rx_index >= g_nano_rx_package.len) {
                g_nano_rx_state = NANO_RX_CHECK;
            }
            break;
        }

        case NANO_RX_CHECK: {
            if (dat == g_nano_rx_check) {
                g_nano_rx_state = NANO_RX_TAIL1;
            } else {
                g_nano_rx_bad_check_count++;
                g_nano_rx_state = NANO_RX_HEAD1;
            }
            break;
        }

        case NANO_RX_TAIL1: {
            if (dat == NANO_FRAME_TAIL1) {
                g_nano_rx_state = NANO_RX_TAIL2;
            } else {
                g_nano_rx_bad_tail_count++;
                g_nano_rx_state = NANO_RX_HEAD1;
            }
            break;
        }

        case NANO_RX_TAIL2: {
            if (dat == NANO_FRAME_TAIL2) {
                if (g_nano_package_ready == 0U) {
                    memcpy(&g_nano_ready_package, &g_nano_rx_package, sizeof(g_nano_ready_package));
                    g_nano_package_ready = 1U;
                    g_nano_rx_frame_count++;
                } else {
                    g_nano_rx_busy_drop_count++;
                }
            } else {
                g_nano_rx_bad_tail_count++;
            }
            g_nano_rx_state = NANO_RX_HEAD1;
            break;
        }

        default: {
            g_nano_rx_state = NANO_RX_HEAD1;
            break;
        }
    }
}

// 计算一字节累加校验。
static uint8_t Nano_COM_Calc_Check(uint8_t ver, uint8_t seq, uint8_t module,
                                   uint8_t cmd, uint8_t len, const uint8_t *payload)
{
    uint8_t check = ver + seq + module + cmd + len;
    uint8_t i = 0U;

    for (i = 0U; i < len; i++) {
        check += payload[i];
    }
    return check;
}

// 发送一帧 NANO 协议数据。
static void Nano_COM_Send_Frame(uint8_t module, uint8_t cmd, uint8_t seq,
                                const uint8_t *payload, uint8_t len)
{
    uint8_t frame[NANO_TX_PAYLOAD_MAX + 10U] = {0U};
    uint8_t index = 0U;

    if (len > NANO_TX_PAYLOAD_MAX) return;

    frame[index++] = NANO_FRAME_HEAD1;
    frame[index++] = NANO_FRAME_HEAD2;
    frame[index++] = NANO_FRAME_VERSION;
    frame[index++] = seq;
    frame[index++] = module;
    frame[index++] = cmd;
    frame[index++] = len;

    if ((payload != NULL) && (len > 0U)) {
        memcpy(&frame[index], payload, len);
        index += len;
    }

    frame[index++] = Nano_COM_Calc_Check(NANO_FRAME_VERSION, seq, module, cmd, len, payload);
    frame[index++] = NANO_FRAME_TAIL1;
    frame[index++] = NANO_FRAME_TAIL2;
    g_nano_tx_frame_count++;
    CN_UART_write_fifo(NANO_PROTOCOL_UART_PORT, frame, index);
}

// 发送 ACK，表示命令已经接收。
static void Nano_COM_Send_Ack(uint8_t seq, uint8_t module, uint8_t cmd, uint8_t state)
{
    uint8_t payload[5] = {seq, module, cmd, NANO_ERR_OK, state};
    Nano_COM_Send_Frame(module, NANO_RESP_ACK, seq, payload, sizeof(payload));
}

// 发送 NACK，表示命令被拒绝。
static void Nano_COM_Send_Nack(uint8_t seq, uint8_t module, uint8_t cmd, uint8_t err, uint8_t state)
{
    uint8_t payload[5] = {seq, module, cmd, err, state};
    Nano_COM_Send_Frame(module, NANO_RESP_NACK, seq, payload, sizeof(payload));
}

// 发送异步事件。
static void Nano_COM_Send_Event(uint8_t seq, uint8_t module, uint8_t event, uint8_t state, uint8_t result)
{
    uint8_t payload[5] = {seq, module, event, state, result};
    Nano_COM_Send_Frame(module, NANO_RESP_EVENT, seq, payload, sizeof(payload));
}

// 发送系统状态快照。
static void Nano_COM_Send_Status(uint8_t seq)
{
    uint8_t payload[16] = {0U};
    uint8_t index = 0U;

    payload[index++] = g_system_state;
    payload[index++] = g_chassis_state;
    payload[index++] = g_arm_state;
    payload[index++] = g_pump_state;
    payload[index++] = (uint8_t)CN_Chassis_Get_Movement_Type();
    payload[index++] = Pump_Handle.Pump_on;
    payload[index++] = Pump_Handle.IfSucked;
    payload[index++] = Arm_Is_Planner_Active();
    Nano_COM_Put_U32_LE(&payload[index], g_nano_ms);
    index += 4U;
    Nano_COM_Put_Float_LE(&payload[index], YAW_now);

    Nano_COM_Send_Frame(NANO_MODULE_SYSTEM, NANO_RESP_STATUS, seq, payload, sizeof(payload));
}

// 发送底盘状态。
static void Nano_COM_Send_Chassis_State(uint8_t seq)
{
    uint8_t payload[22] = {0U};
    uint8_t index = 0U;
    float wz_feedback = 0.0f;

    payload[index++] = g_chassis_state;
    payload[index++] = (uint8_t)CN_Chassis_Get_Movement_Type();
    Nano_COM_Put_Float_LE(&payload[index], YAW_now);
    index += 4U;
    Nano_COM_Put_Float_LE(&payload[index],
                          Nano_COM_Unmap_Signed_Zero_Min(target_VelX,
                                                         NANO_DN_RUN_SPEED_MIN,
                                                         NANO_DN_RUN_SPEED_MAX,
                                                         NANO_UP_RUN_SPEED_MAX));
    index += 4U;
    Nano_COM_Put_Float_LE(&payload[index],
                          Nano_COM_Unmap_Signed_Zero_Min(target_VelY,
                                                         NANO_DN_RUN_SPEED_MIN,
                                                         NANO_DN_RUN_SPEED_MAX,
                                                         NANO_UP_RUN_SPEED_MAX));
    index += 4U;
    if (g_chassis_state == NANO_CHASSIS_ROTATE) {
        wz_feedback = Nano_COM_Unmap_Signed_Linear(target_anv,
                                                   NANO_DN_ROT_WZ_MAX,
                                                   NANO_UP_ROT_WZ_MAX);
    } else {
        wz_feedback = Nano_COM_Unmap_Signed_Linear(target_anv,
                                                   NANO_DN_RUN_WZ_MAX,
                                                   NANO_UP_RUN_WZ_MAX);
    }
    Nano_COM_Put_Float_LE(&payload[index], wz_feedback);
    index += 4U;
    Nano_COM_Put_Float_LE(&payload[index],
                          Nano_COM_Unmap_Signed_Linear(distance_to_go,
                                                       NANO_DN_TRANS_DIST_MAX,
                                                       NANO_UP_TRANS_DIST_MAX_CM));

    Nano_COM_Send_Frame(NANO_MODULE_CHASSIS, NANO_RESP_CHASSIS_STATE, seq, payload, sizeof(payload));
}

// 发送机械臂当前逻辑位姿。
static void Nano_COM_Send_Arm_Pose(uint8_t seq)
{
    uint8_t payload[36] = {0U};
    uint8_t index = 0U;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float dir = 0.0f;
    float th0 = 0.0f;
    float th1 = 0.0f;
    float th2 = 0.0f;
    float th3 = 0.0f;

    Arm_Get_Logical_Pose(&x, &y, &z, &dir, &th0, &th1, &th2, &th3);
    payload[index++] = g_arm_state;
    payload[index++] = 0U;
    payload[index++] = Arm_Is_Planner_Active();
    payload[index++] = 0U;
    Nano_COM_Put_Float_LE(&payload[index], x);
    index += 4U;
    Nano_COM_Put_Float_LE(&payload[index], y);
    index += 4U;
    Nano_COM_Put_Float_LE(&payload[index], z);
    index += 4U;
    Nano_COM_Put_Float_LE(&payload[index], dir);
    index += 4U;
    Nano_COM_Put_Float_LE(&payload[index], th0);
    index += 4U;
    Nano_COM_Put_Float_LE(&payload[index], th1);
    index += 4U;
    Nano_COM_Put_Float_LE(&payload[index], th2);
    index += 4U;
    Nano_COM_Put_Float_LE(&payload[index], th3);

    Nano_COM_Send_Frame(NANO_MODULE_ARM, NANO_RESP_ARM_POSE, seq, payload, sizeof(payload));
}

// 发送气泵状态。
static void Nano_COM_Send_Pump_State(uint8_t seq)
{
    uint8_t payload[10] = {0U};
    uint8_t index = 0U;

    payload[index++] = g_pump_state;
    payload[index++] = Pump_Handle.Pump_on;
    payload[index++] = Pump_Handle.IfSucked;
    payload[index++] = 0U;
    Nano_COM_Put_Float_LE(&payload[index], CN_Pump_Get_Feedback_Freq_Hz());
    index += 4U;
    payload[index++] = (uint8_t)Pump_Handle.SuckStrength;
    payload[index++] = 0U;

    Nano_COM_Send_Frame(NANO_MODULE_PUMP, NANO_RESP_PUMP_STATE, seq, payload, sizeof(payload));
}

// 判断当前系统状态是否允许执行命令。
static uint8_t Nano_COM_System_Allows_Command(uint8_t module, uint8_t cmd)
{
    if (g_system_state == NANO_SYS_INIT) {
        return (module == NANO_MODULE_SYSTEM &&
               (cmd == NANO_CMD_SYS_HELLO || cmd == NANO_CMD_SYS_GET_STATUS)) ? 1U : 0U;
    }

    if (g_system_state == NANO_SYS_ESTOP || g_system_state == NANO_SYS_FAULT) {
        return (module == NANO_MODULE_SYSTEM &&
               (cmd == NANO_CMD_SYS_HELLO || cmd == NANO_CMD_SYS_GET_STATUS ||
                cmd == NANO_CMD_SYS_RESET)) ? 1U : 0U;
    }

    if (g_system_state == NANO_SYS_RET) {
        return (module == NANO_MODULE_SYSTEM &&
               (cmd == NANO_CMD_SYS_HELLO || cmd == NANO_CMD_SYS_GET_STATUS)) ? 1U : 0U;
    }

    return 1U;
}

// 处理一包完整命令。
static void Nano_COM_Process_Package(const NanoPackage_t *package)
{
    Car_Movement_struct movement = {0};

    if (package == NULL) return;

    if (!Nano_COM_System_Allows_Command(package->module, package->cmd)) {
        Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_STATE, g_system_state);
        return;
    }

    switch (package->module) {
        case NANO_MODULE_SYSTEM: {
            if (package->cmd == NANO_CMD_SYS_HELLO) {
                if (package->len != 0U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_LEN, g_system_state);
                    break;
                }
                Nano_COM_Send_Ack(package->seq, package->module, package->cmd, g_system_state);
                Nano_COM_Send_Status(package->seq);
            } else if (package->cmd == NANO_CMD_SYS_GET_STATUS) {
                if (package->len != 0U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_LEN, g_system_state);
                    break;
                }
                Nano_COM_Send_Status(package->seq);
            } else if (package->cmd == NANO_CMD_SYS_RESET) {
                if (package->len != 0U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_LEN, g_system_state);
                    break;
                }
                Nano_COM_Send_Ack(package->seq, package->module, package->cmd, g_system_state);
                Nano_COM_Start_Reset(package->seq);
            } else if (package->cmd == NANO_CMD_SYS_ESTOP) {
                if (package->len != 0U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_LEN, g_system_state);
                    break;
                }
                CN_set_movement(movement, Movement_Breaking);
                g_HostCmd.cmd_type = ARM_HOST_CMD_STOP;
                g_HostCmd.run_flag = 0U;
                g_HostCmd.new_cmd_flag = 1U;
                CN_Pump_Stop();
                g_chassis_state = NANO_CHASSIS_BRAKE;
                g_arm_state = NANO_ARM_HOLD;
                g_pump_state = NANO_PUMP_IDLE;
                g_chassis_run_active = 0U;
                g_system_state = NANO_SYS_ESTOP;
                Nano_COM_Send_Ack(package->seq, package->module, package->cmd, g_system_state);
                Nano_COM_Send_Event(package->seq, NANO_MODULE_SYSTEM, NANO_EVENT_SYS_ESTOP,
                                    g_system_state, NANO_ERR_OK);
            } else {
                Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_CMD, g_system_state);
            }
            break;
        }

        case NANO_MODULE_CHASSIS: {
            if (package->cmd == NANO_CMD_CHASSIS_RUN) {
                if (package->len != 12U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_LEN, g_chassis_state);
                    break;
                }
                movement.Car_X_vel = Nano_COM_Map_Signed_Zero_Min(
                    Nano_COM_Get_Float_LE(&package->payload[0]),
                    NANO_UP_RUN_SPEED_MAX, NANO_DN_RUN_SPEED_MIN, NANO_DN_RUN_SPEED_MAX);
                movement.Car_Y_vel = Nano_COM_Map_Signed_Zero_Min(
                    Nano_COM_Get_Float_LE(&package->payload[4]),
                    NANO_UP_RUN_SPEED_MAX, NANO_DN_RUN_SPEED_MIN, NANO_DN_RUN_SPEED_MAX);
                movement.Car_angvel = Nano_COM_Map_Signed_Linear(
                    Nano_COM_Get_Float_LE(&package->payload[8]),
                    NANO_UP_RUN_WZ_MAX, NANO_DN_RUN_WZ_MAX);
                CN_set_movement(movement, Movement_Running);
                g_chassis_state = NANO_CHASSIS_RUN;
                g_chassis_run_active = 1U;
                g_chassis_last_run_ms = g_nano_ms;
                g_chassis_action_seq = package->seq;
                Nano_COM_Send_Ack(package->seq, package->module, package->cmd, g_chassis_state);
            } else if (package->cmd == NANO_CMD_CHASSIS_STOP) {
                if (package->len != 0U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_LEN, g_chassis_state);
                    break;
                }
                CN_set_movement(movement, Movement_Running);
                g_chassis_state = NANO_CHASSIS_STOP;
                g_chassis_run_active = 0U;
                Nano_COM_Send_Ack(package->seq, package->module, package->cmd, g_chassis_state);
            } else if (package->cmd == NANO_CMD_CHASSIS_BRAKE) {
                if (package->len != 0U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_LEN, g_chassis_state);
                    break;
                }
                CN_set_movement(movement, Movement_Breaking);
                g_chassis_state = NANO_CHASSIS_BRAKE;
                g_chassis_run_active = 0U;
                Nano_COM_Send_Ack(package->seq, package->module, package->cmd, g_chassis_state);
            } else if (package->cmd == NANO_CMD_CHASSIS_ROTATE_TO) {
                if (package->len != 8U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_LEN, g_chassis_state);
                    break;
                }
                if (g_chassis_state == NANO_CHASSIS_ROTATE || g_chassis_state == NANO_CHASSIS_TRANSLATE) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BUSY, g_chassis_state);
                    break;
                }
                movement.Car_angle = Nano_COM_Get_Float_LE(&package->payload[0]);
                movement.Car_angvel = Nano_COM_Map_Unsigned_Min(
                    Nano_COM_Get_Float_LE(&package->payload[4]),
                    NANO_UP_ROT_WZ_MIN, NANO_UP_ROT_WZ_MAX,
                    NANO_DN_ROT_WZ_MIN, NANO_DN_ROT_WZ_MAX);
                CN_set_movement(movement, Movement_Rotation);
                g_chassis_state = NANO_CHASSIS_ROTATE;
                g_chassis_action_seq = package->seq;
                g_chassis_run_active = 0U;
                Nano_COM_Send_Ack(package->seq, package->module, package->cmd, g_chassis_state);
            } else if (package->cmd == NANO_CMD_CHASSIS_TRANSLATE_REL) {
                if (package->len != 12U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_LEN, g_chassis_state);
                    break;
                }
                if (g_chassis_state == NANO_CHASSIS_ROTATE || g_chassis_state == NANO_CHASSIS_TRANSLATE) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BUSY, g_chassis_state);
                    break;
                }
                movement.Car_X_movement = Nano_COM_Map_Signed_Linear(
                    Nano_COM_Get_Float_LE(&package->payload[0]),
                    NANO_UP_TRANS_DIST_MAX_CM, NANO_DN_TRANS_DIST_MAX);
                movement.Car_Y_movement = Nano_COM_Map_Signed_Linear(
                    Nano_COM_Get_Float_LE(&package->payload[4]),
                    NANO_UP_TRANS_DIST_MAX_CM, NANO_DN_TRANS_DIST_MAX);
                movement.Car_translation_vel = Nano_COM_Map_Unsigned_Min(
                    Nano_COM_Get_Float_LE(&package->payload[8]),
                    NANO_UP_TRANS_SPEED_MIN, NANO_UP_TRANS_SPEED_MAX,
                    NANO_DN_TRANS_SPEED_MIN, NANO_DN_TRANS_SPEED_MAX);
                CN_set_movement(movement, Movement_Translation);
                g_chassis_state = NANO_CHASSIS_TRANSLATE;
                g_chassis_action_seq = package->seq;
                g_chassis_run_active = 0U;
                Nano_COM_Send_Ack(package->seq, package->module, package->cmd, g_chassis_state);
            } else if (package->cmd == NANO_CMD_CHASSIS_GET_STATE) {
                if (package->len != 0U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_LEN, g_chassis_state);
                    break;
                }
                Nano_COM_Send_Chassis_State(package->seq);
            } else {
                Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_CMD, g_chassis_state);
            }
            break;
        }

        case NANO_MODULE_ARM: {
            if (package->cmd == NANO_CMD_ARM_TARGET_XYZ_DIR) {
                if (package->len != 16U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_LEN, g_arm_state);
                    break;
                }
                if (g_arm_state == NANO_ARM_MOVING || g_arm_state == NANO_ARM_FOLDING ||
                    Arm_Is_Planner_Active() != 0U || g_HostCmd.new_cmd_flag != 0U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BUSY, g_arm_state);
                    break;
                }
                g_HostCmd.world_x = Nano_COM_Get_Float_LE(&package->payload[0]);
                g_HostCmd.world_y = Nano_COM_Get_Float_LE(&package->payload[4]);
                g_HostCmd.world_z = Nano_COM_Get_Float_LE(&package->payload[8]);
                g_HostCmd.suc_dir_deg = Nano_COM_Get_Float_LE(&package->payload[12]);
                g_HostCmd.cmd_type = ARM_HOST_CMD_MOVE;
                g_HostCmd.run_flag = 1U;
                g_HostCmd.new_cmd_flag = 1U;
                g_arm_state = NANO_ARM_MOVING;
                g_arm_action_seq = package->seq;
                Nano_COM_Send_Ack(package->seq, package->module, package->cmd, g_arm_state);
            } else if (package->cmd == NANO_CMD_ARM_FOLD_HOME) {
                if (package->len != 0U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_LEN, g_arm_state);
                    break;
                }
                if (g_arm_state == NANO_ARM_MOVING || g_arm_state == NANO_ARM_FOLDING ||
                    Arm_Is_Planner_Active() != 0U || g_HostCmd.new_cmd_flag != 0U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BUSY, g_arm_state);
                    break;
                }
                g_HostCmd.cmd_type = ARM_HOST_CMD_FOLD;
                g_HostCmd.run_flag = 1U;
                g_HostCmd.new_cmd_flag = 1U;
                g_arm_state = NANO_ARM_FOLDING;
                g_arm_action_seq = package->seq;
                Nano_COM_Send_Ack(package->seq, package->module, package->cmd, g_arm_state);
            } else if (package->cmd == NANO_CMD_ARM_STOP_HOLD) {
                if (package->len != 0U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_LEN, g_arm_state);
                    break;
                }
                g_HostCmd.cmd_type = ARM_HOST_CMD_STOP;
                g_HostCmd.run_flag = 0U;
                g_HostCmd.new_cmd_flag = 1U;
                g_arm_state = NANO_ARM_HOLD;
                Nano_COM_Send_Ack(package->seq, package->module, package->cmd, g_arm_state);
            } else if (package->cmd == NANO_CMD_ARM_GET_POSE) {
                if (package->len != 0U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_LEN, g_arm_state);
                    break;
                }
                Nano_COM_Send_Arm_Pose(package->seq);
            } else {
                Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_CMD, g_arm_state);
            }
            break;
        }

        case NANO_MODULE_PUMP: {
            if (package->cmd == NANO_CMD_PUMP_SUCK) {
                if (package->len != 0U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_LEN, g_pump_state);
                    break;
                }
                if (g_pump_state == NANO_PUMP_SUCKING || g_pump_state == NANO_PUMP_SUCKED ||
                    g_pump_state == NANO_PUMP_RELEASING) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BUSY, g_pump_state);
                    break;
                }
                CN_Pump_Start();
                g_pump_state = NANO_PUMP_SUCKING;
                g_pump_action_seq = package->seq;
                Nano_COM_Send_Ack(package->seq, package->module, package->cmd, g_pump_state);
            } else if (package->cmd == NANO_CMD_PUMP_RELEASE) {
                if (package->len != 0U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_LEN, g_pump_state);
                    break;
                }
                CN_Pump_Stop();
                Pump_Handle.IfSucked = 0U;
                g_pump_state = NANO_PUMP_RELEASING;
                g_pump_release_start_ms = g_nano_ms;
                g_pump_action_seq = package->seq;
                Nano_COM_Send_Ack(package->seq, package->module, package->cmd, g_pump_state);
            } else if (package->cmd == NANO_CMD_PUMP_GET_STATE) {
                if (package->len != 0U) {
                    Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_LEN, g_pump_state);
                    break;
                }
                Nano_COM_Send_Pump_State(package->seq);
            } else {
                Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_CMD, g_pump_state);
            }
            break;
        }

        default: {
            Nano_COM_Send_Nack(package->seq, package->module, package->cmd, NANO_ERR_BAD_CMD, g_system_state);
            break;
        }
    }
}

// 启动系统复位流程。
static void Nano_COM_Start_Reset(uint8_t seq)
{
    g_system_state = NANO_SYS_RET;
    g_reset_active = 1U;
    g_reset_arm_wait = 1U;
    g_reset_pump_wait = 1U;

    CN_Intr_Movement();
    g_chassis_state = NANO_CHASSIS_IDLE;
    g_chassis_run_active = 0U;

    g_HostCmd.cmd_type = ARM_HOST_CMD_FOLD;
    g_HostCmd.run_flag = 1U;
    g_HostCmd.new_cmd_flag = 1U;
    g_arm_state = NANO_ARM_FOLDING;
    g_arm_action_seq = seq;

    CN_Pump_Stop();
    Pump_Handle.IfSucked = 0U;
    g_pump_state = NANO_PUMP_RELEASING;
    g_pump_release_start_ms = g_nano_ms;
    g_pump_action_seq = seq;

}

// 根据模块忙闲刷新系统总状态。
static void Nano_COM_Update_System_State(void)
{
    if (g_system_state == NANO_SYS_ESTOP || g_system_state == NANO_SYS_FAULT ||
        g_system_state == NANO_SYS_INIT || g_system_state == NANO_SYS_RET) {
        return;
    }

    if (g_chassis_state == NANO_CHASSIS_RUN || g_chassis_state == NANO_CHASSIS_ROTATE ||
        g_chassis_state == NANO_CHASSIS_TRANSLATE || g_arm_state == NANO_ARM_MOVING ||
        g_arm_state == NANO_ARM_FOLDING || g_pump_state == NANO_PUMP_SUCKING ||
        g_pump_state == NANO_PUMP_RELEASING) {
        g_system_state = NANO_SYS_ACTIVE;
    } else {
        g_system_state = NANO_SYS_IDLE;
    }
}

// 刷新底盘动作完成和速度命令超时。
static void Nano_COM_Update_Chassis(void)
{
    Car_Movement_struct stop_movement = {0};

    if ((g_chassis_run_active != 0U) &&
        ((uint32_t)(g_nano_ms - g_chassis_last_run_ms) > NANO_CHASSIS_RUN_TIMEOUT_MS)) {
        CN_set_movement(stop_movement, Movement_Running);
        g_chassis_run_active = 0U;
        g_chassis_state = NANO_CHASSIS_STOP;
        Nano_COM_Send_Event(g_chassis_action_seq, NANO_MODULE_CHASSIS,
                            NANO_EVENT_CHASSIS_TIMEOUT_STOP, g_chassis_state, NANO_ERR_TIMEOUT);
    }

    if (g_chassis_state == NANO_CHASSIS_ROTATE &&
        CN_Chassis_Get_Movement_Type() == Movement_Running) {
        g_chassis_state = NANO_CHASSIS_IDLE;
        Nano_COM_Send_Event(g_chassis_action_seq, NANO_MODULE_CHASSIS,
                            NANO_EVENT_CHASSIS_ROTATE_DONE, g_chassis_state, NANO_ERR_OK);
    }

    if (g_chassis_state == NANO_CHASSIS_TRANSLATE &&
        CN_Chassis_Get_Movement_Type() == Movement_Running) {
        g_chassis_state = NANO_CHASSIS_IDLE;
        Nano_COM_Send_Event(g_chassis_action_seq, NANO_MODULE_CHASSIS,
                            NANO_EVENT_CHASSIS_TRANS_DONE, g_chassis_state, NANO_ERR_OK);
    }
}

// 刷新机械臂内部事件。
static void Nano_COM_Update_Arm(void)
{
    uint8_t event = ARM_STATUS_EVENT_NONE;

    if (Arm_Fetch_Status_Event(&event) == 0U) return;

    if (event == ARM_STATUS_EVENT_DONE) {
        g_arm_state = NANO_ARM_IDLE;
        Nano_COM_Send_Event(g_arm_action_seq, NANO_MODULE_ARM, NANO_EVENT_ARM_DONE,
                            g_arm_state, NANO_ERR_OK);
    } else if (event == ARM_STATUS_EVENT_FOLD_DONE) {
        g_arm_state = NANO_ARM_IDLE;
        Nano_COM_Send_Event(g_arm_action_seq, NANO_MODULE_ARM, NANO_EVENT_ARM_FOLD_DONE,
                            g_arm_state, NANO_ERR_OK);
        g_reset_arm_wait = 0U;
    } else if (event == ARM_STATUS_EVENT_REJECTED) {
        g_arm_state = NANO_ARM_REJECTED;
        Nano_COM_Send_Event(g_arm_action_seq, NANO_MODULE_ARM, NANO_EVENT_ARM_REJECTED,
                            g_arm_state, NANO_ERR_RANGE_LIMIT);
        g_reset_arm_wait = 0U;
    } else if (event == ARM_STATUS_EVENT_STOPPED) {
        g_arm_state = NANO_ARM_HOLD;
        g_reset_arm_wait = 0U;
    }
}

// 刷新气泵吸附完成、超时停泵、释放完成和系统复位完成。
static void Nano_COM_Update_Pump(void)
{
    if ((g_pump_state == NANO_PUMP_SUCKING || g_pump_state == NANO_PUMP_SUCKED) &&
        (Pump_Handle.Pump_on == 0U)) {
        Pump_Handle.IfSucked = 0U;
        Pump_Handle.SuckStrength = 0U;
        g_pump_state = NANO_PUMP_IDLE;
        Nano_COM_Send_Event(g_pump_action_seq, NANO_MODULE_PUMP,
                            NANO_EVENT_PUMP_TIMEOUT_STOP, g_pump_state, NANO_ERR_TIMEOUT);
        return;
    }

    if (g_pump_state == NANO_PUMP_SUCKING) {
        if (Pump_Handle.IfSucked != 0U) {
            g_pump_state = NANO_PUMP_SUCKED;
            Nano_COM_Send_Event(g_pump_action_seq, NANO_MODULE_PUMP,
                                NANO_EVENT_PUMP_SUCK_DONE, g_pump_state, NANO_ERR_OK);
        }
    }

    if (g_pump_state == NANO_PUMP_RELEASING) {
        if ((uint32_t)(g_nano_ms - g_pump_release_start_ms) >= NANO_PUMP_RELEASE_WAIT_MS) {
            g_pump_state = NANO_PUMP_RELEASED;
            g_reset_pump_wait = 0U;
            Nano_COM_Send_Event(g_pump_action_seq, NANO_MODULE_PUMP,
                                NANO_EVENT_PUMP_RELEASE_DONE, g_pump_state, NANO_ERR_OK);
        }
    }

    if ((g_reset_active != 0U) && (g_reset_arm_wait == 0U) && (g_reset_pump_wait == 0U)) {
        g_reset_active = 0U;
        g_system_state = NANO_SYS_IDLE;
        g_pump_state = NANO_PUMP_IDLE;
        Nano_COM_Send_Event(g_pump_action_seq, NANO_MODULE_SYSTEM,
                            NANO_EVENT_SYS_RESET_DONE, g_system_state, NANO_ERR_OK);
        Nano_COM_Send_Status(g_pump_action_seq);
    }
}

// 对称限幅，保留输入符号。
static float Nano_COM_Limit_Abs_Float(float value, float max_abs)
{
    if (max_abs < 0.0f) {
        max_abs = -max_abs;
    }
    if (value > max_abs) return max_abs;
    if (value < -max_abs) return -max_abs;
    return value;
}

// 线性映射带符号量：上位机 [-up_max_abs,+up_max_abs] 映射到下位机 [-dn_max_abs,+dn_max_abs]。
static float Nano_COM_Map_Signed_Linear(float value, float up_max_abs, float dn_max_abs)
{
    float limited = Nano_COM_Limit_Abs_Float(value, up_max_abs);

    if (up_max_abs <= 0.0f) return 0.0f;
    return limited * (dn_max_abs / up_max_abs);
}

// 带最小非零输出的线性映射，用于克服底盘低速死区；输入为 0 时仍输出 0。
static float Nano_COM_Map_Signed_Zero_Min(float value, float up_max_abs,
                                          float dn_min_abs, float dn_max_abs)
{
    float sign = 1.0f;
    float mag = value;
    float ratio = 0.0f;
    float mapped = 0.0f;

    if (mag < 0.0f) {
        sign = -1.0f;
        mag = -mag;
    }
    if (mag <= 0.0f) return 0.0f;
    if (mag > up_max_abs) mag = up_max_abs;
    if (up_max_abs <= 1.0f) return sign * dn_max_abs;

    if (mag < 1.0f) mag = 1.0f;
    ratio = (mag - 1.0f) / (up_max_abs - 1.0f);
    mapped = dn_min_abs + ratio * (dn_max_abs - dn_min_abs);
    return sign * mapped;
}

// 无符号速度映射：上位机 [up_min,up_max] 映射到下位机 [dn_min,dn_max]，越界时按边界夹紧。
static float Nano_COM_Map_Unsigned_Min(float value, float up_min, float up_max,
                                       float dn_min, float dn_max)
{
    float ratio = 0.0f;

    if (value < 0.0f) value = -value;
    if (value < up_min) value = up_min;
    if (value > up_max) value = up_max;
    if (up_max <= up_min) return dn_min;

    ratio = (value - up_min) / (up_max - up_min);
    return dn_min + ratio * (dn_max - dn_min);
}

// 线性反向映射，用于状态回包保持上位机量程。
static float Nano_COM_Unmap_Signed_Linear(float value, float dn_max_abs, float up_max_abs)
{
    float limited = Nano_COM_Limit_Abs_Float(value, dn_max_abs);

    if (dn_max_abs <= 0.0f) return 0.0f;
    return limited * (up_max_abs / dn_max_abs);
}

// 带最小非零输出映射的反向换算，输入为 0 时仍回报 0。
static float Nano_COM_Unmap_Signed_Zero_Min(float value, float dn_min_abs,
                                            float dn_max_abs, float up_max_abs)
{
    float sign = 1.0f;
    float mag = value;
    float ratio = 0.0f;
    float mapped = 0.0f;

    if (mag < 0.0f) {
        sign = -1.0f;
        mag = -mag;
    }
    if (mag <= 0.0f) return 0.0f;
    if (mag > dn_max_abs) mag = dn_max_abs;
    if (dn_max_abs <= dn_min_abs) return sign * up_max_abs;

    if (mag < dn_min_abs) mag = dn_min_abs;
    ratio = (mag - dn_min_abs) / (dn_max_abs - dn_min_abs);
    mapped = 1.0f + ratio * (up_max_abs - 1.0f);
    return sign * mapped;
}

static float Nano_COM_Get_Float_LE(const uint8_t *src)
{
    union {
        float f;
        uint8_t b[4];
    } data;

    data.b[0] = src[0];
    data.b[1] = src[1];
    data.b[2] = src[2];
    data.b[3] = src[3];
    return data.f;
}

// 以小端格式写入 float。
static void Nano_COM_Put_Float_LE(uint8_t *dst, float value)
{
    union {
        float f;
        uint8_t b[4];
    } data;

    data.f = value;
    dst[0] = data.b[0];
    dst[1] = data.b[1];
    dst[2] = data.b[2];
    dst[3] = data.b[3];
}

// 以小端格式写入 uint32。
static void Nano_COM_Put_U32_LE(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

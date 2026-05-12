#include "stm32h7xx_hal.h"
#include "ChaseNum1_Include.h"
#include <string.h>
#include <stdio.h>

// 串口拼出的完整命令包
Main_COM_Package_t Package_get = {0};
uint8_t mission_check = 0;

// 下位机发送帧时使用的递增序号
static uint8_t Main_COM_seq = 0;
// 判断当前是否处于上位机运动控制状态
static uint8_t Main_COM_motion_active = 0;
// 最近一次收到有效 RUN 命令的时间
static uint32_t Main_COM_last_motion_ms = 0;
// 最近一次周期状态反馈时间
static uint32_t Main_COM_last_feedback_ms = 0;
// 状态反馈周期，0 表示关闭
static uint16_t Main_COM_feedback_interval_ms = MAIN_COM_FEEDBACK_DEFAULT_MS;
// 当前底盘模式：Movement_Breaking 或 Movement_Running
static Movement_type_ENUM Main_COM_current_mode = Movement_Breaking;
// 串口中断与主循环之间的完整包就绪标志
static volatile uint8_t Main_COM_package_ready = 0;

// 解包状态变量
typedef enum {
    Main_COM_RX_HEAD1 = 0,
    Main_COM_RX_HEAD2,
    Main_COM_RX_VERSION,
    Main_COM_RX_SEQ,
    Main_COM_RX_CMD,
    Main_COM_RX_LEN,
    Main_COM_RX_PAYLOAD,
    Main_COM_RX_CHECK,
    Main_COM_RX_TAIL1,
    Main_COM_RX_TAIL2,
} Main_COM_RX_State_t;
static Main_COM_RX_State_t Main_COM_rx_state = Main_COM_RX_HEAD1;
static Main_COM_Package_t Main_COM_rx_package = {0};
static uint8_t Main_COM_rx_payload_index = 0;
static uint8_t Main_COM_rx_seq = 0;
static uint8_t Main_COM_rx_check = 0;

// 为状态反馈时间戳、运动命令超时和周期反馈判断提供判断
static uint32_t Main_COM_now_ms(void)
{
    return systime_us / 1000u;
}

// 清空当前接收机状态 重新等待下一帧
static void Main_COM_reset_rx(void)
{
    Main_COM_rx_state = Main_COM_RX_HEAD1;
    Main_COM_rx_payload_index = 0;
    Main_COM_rx_check = 0;
    memset(&Main_COM_rx_package, 0, sizeof(Main_COM_rx_package));
}

// 接收出错时重启接收状态机（同时找到新的帧头）
static void Main_COM_restart_rx(uint8_t dat)
{
    Main_COM_reset_rx();
    if (dat == MAIN_COM_FRAME_HEAD1) {
        Main_COM_rx_state = Main_COM_RX_HEAD2;
    }
}

// 计算协议校验和
static uint8_t Main_COM_calc_check(uint8_t version, uint8_t seq, uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
    uint8_t check = 0;
    check += version;
    check += seq;
    check += cmd;
    check += payload_len;
    for (uint8_t i = 0; i < payload_len; i++) {
        check += payload[i];
    }
    return check;
}

// 从 little-endian 字节数组中读取 uint16_t
static uint16_t Main_COM_get_u16_le(const uint8_t *src)
{
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

// 把 uint32_t 按 little-endian 写入 payload
static void Main_COM_put_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8) & 0xFFu);
    dst[2] = (uint8_t)((value >> 16) & 0xFFu);
    dst[3] = (uint8_t)((value >> 24) & 0xFFu);
}

// 从 little-endian IEEE-754 float32 字节中读取 float
static float Main_COM_get_float_le(const uint8_t *src)
{
    union {
        float f;
        uint8_t b[4];
    } value;

    value.b[0] = src[0];
    value.b[1] = src[1];
    value.b[2] = src[2];
    value.b[3] = src[3];
    return value.f;
}

// 把 float 按 little-endian IEEE-754 float32 写入 payload
static void Main_COM_put_float_le(uint8_t *dst, float value)
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

// 对上位机输入速度进行限幅，并过滤 NaN
static float Main_COM_limit_float(float value, float max_abs)
{
    if (value != value) {
        return 0.0f;
    }
    if (value > max_abs) {
        return max_abs;
    }
    if (value < -max_abs) {
        return -max_abs;
    }
    return value;
}

// 发送 ACK 响应，表示命令已被下位机接收并执行
static void Main_COM_send_ack(uint8_t cmd)
{
    uint8_t payload[2] = {cmd, MAIN_COM_ERR_NONE};
    Main_COM_send_frame(MAIN_COM_RESP_ACK, payload, sizeof(payload));
}

// 发送错误响应，表示命令错误、payload 长度错误或控制超时
static void Main_COM_send_error(uint8_t cmd, uint8_t err)
{
    uint8_t payload[2] = {cmd, err};
    Main_COM_send_frame(MAIN_COM_RESP_ERR, payload, sizeof(payload));
}

// 调用底盘刹车模式
static void Main_COM_apply_brake(void)
{
    Car_Movement_struct movement = {0};

    CN_set_movement(movement, Movement_Breaking);
    Main_COM_current_mode = Movement_Breaking;
    Main_COM_motion_active = 0;
}

// 调用底盘运行函数
static void Main_COM_apply_run(float vx, float vy, float wz_deg_s)
{
    Car_Movement_struct movement = {0};

    movement.Car_X_vel = Main_COM_limit_float(vx, MAIN_COM_MAX_VX);
    movement.Car_Y_vel = Main_COM_limit_float(vy, MAIN_COM_MAX_VY);
    movement.Car_angvel = Main_COM_limit_float(wz_deg_s, MAIN_COM_MAX_WZ_DEG_S);

    CN_set_movement(movement, Movement_Running);
    Main_COM_current_mode = Movement_Running;
    Main_COM_motion_active = 1;
    Main_COM_last_motion_ms = Main_COM_now_ms();
}

// 初始化下位机通信模块
void Main_COM_init(void)
{
    CN_UART3_init();
    Main_COM_apply_brake();
    Main_COM_reset_rx();
}

// 下位机通用发送帧函数
void Main_COM_send_frame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
    uint8_t frame[MAIN_COM_MAX_PAYLOAD_LEN + MAIN_COM_FRAME_OVERHEAD] = {0};
    uint8_t index = 0;

    if (payload_len > MAIN_COM_MAX_PAYLOAD_LEN) {
        return;
    }

    frame[index++] = MAIN_COM_FRAME_HEAD1;
    frame[index++] = MAIN_COM_FRAME_HEAD2;
    frame[index++] = MAIN_COM_FRAME_VERSION;
    frame[index++] = Main_COM_seq++;
    frame[index++] = cmd;
    frame[index++] = payload_len;

    if ((payload != NULL) && (payload_len > 0u)) {
        memcpy(&frame[index], payload, payload_len);
        index += payload_len;
    }

    frame[index] = Main_COM_calc_check(frame[2], frame[3], frame[4], &frame[6], payload_len);
    index++;
    frame[index++] = MAIN_COM_FRAME_TAIL1;
    frame[index++] = MAIN_COM_FRAME_TAIL2;

    CN_UART_write_fifo(CN_UART_3, frame, index);
}

// 发送当前底盘状态和航向角反馈
void Main_COM_send_status(void)
{
    uint8_t payload[MAIN_COM_STATUS_PAYLOAD_LEN] = {0};
    uint8_t index = 0;

    payload[index++] = (uint8_t)Main_COM_current_mode;
    Main_COM_put_float_le(&payload[index], CN_get_logical_yaw());
    index += 4;
    Main_COM_put_float_le(&payload[index], target_VelX);
    index += 4;
    Main_COM_put_float_le(&payload[index], target_VelY);
    index += 4;
    Main_COM_put_float_le(&payload[index], YAW_acc);
    index += 4;
    Main_COM_put_u32_le(&payload[index], Main_COM_now_ms());

    Main_COM_send_frame(MAIN_COM_RESP_STATUS, payload, sizeof(payload));
}

// 下位机命令调度函数
static void Main_COM_process_package(const Main_COM_Package_t *package)
{
    uint8_t cmd = (uint8_t)package->CMD;

    switch (cmd) {
        case MAIN_COM_CMD_HELLO: {
            if (package->load_len != 0u) {
                Main_COM_send_error(cmd, MAIN_COM_ERR_BAD_LEN);
                break;
            }
            Main_COM_send_ack(cmd);
            Main_COM_send_status();
            break;
        }

        case MAIN_COM_CMD_BRAKE: {
            if (package->load_len != 0u) {
                Main_COM_send_error(cmd, MAIN_COM_ERR_BAD_LEN);
                break;
            }
            Main_COM_apply_brake();
            Main_COM_send_ack(cmd);
            Main_COM_send_status();
            break;
        }

        case MAIN_COM_CMD_RUN: {
            if (package->load_len != MAIN_COM_RUN_PAYLOAD_LEN) {
                Main_COM_send_error(cmd, MAIN_COM_ERR_BAD_LEN);
                break;
            }
            Main_COM_apply_run(Main_COM_get_float_le(&package->load[0]),
                               Main_COM_get_float_le(&package->load[4]),
                               Main_COM_get_float_le(&package->load[8]));
            Main_COM_send_ack(cmd);
            Main_COM_send_status();
            break;
        }

        case MAIN_COM_CMD_GET_STATUS: {
            uint8_t data = 1;
            CN_UART_write_fifo(CN_UART_3,&data,1);
            if (package->load_len != 0u) {
                Main_COM_send_error(cmd, MAIN_COM_ERR_BAD_LEN);
                break;
            }
            Main_COM_send_status();
            break;
        }

        // 功能暂时未启用
        case MAIN_COM_CMD_SET_YAW: {
            if (package->load_len != MAIN_COM_SET_YAW_PAYLOAD_LEN) {
                Main_COM_send_error(cmd, MAIN_COM_ERR_BAD_LEN);
                break;
            }
            // CN_Set_YAW(Main_COM_get_float_le(&package->load[0]));
            Main_COM_send_ack(cmd);
            Main_COM_send_status();
            break;
        }

        case MAIN_COM_CMD_SET_FEEDBACK: {
            uint16_t interval_ms;

            if (package->load_len != MAIN_COM_SET_FEEDBACK_PAYLOAD_LEN) {
                Main_COM_send_error(cmd, MAIN_COM_ERR_BAD_LEN);
                break;
            }

            interval_ms = Main_COM_get_u16_le(&package->load[0]);
            if (interval_ms == 0u) {
                Main_COM_feedback_interval_ms = 0;
            } else if (interval_ms < MAIN_COM_FEEDBACK_MIN_MS) {
                Main_COM_feedback_interval_ms = MAIN_COM_FEEDBACK_MIN_MS;
            } else if (interval_ms > MAIN_COM_FEEDBACK_MAX_MS) {
                Main_COM_feedback_interval_ms = MAIN_COM_FEEDBACK_MAX_MS;
            } else {
                Main_COM_feedback_interval_ms = interval_ms;
            }

            Main_COM_send_ack(cmd);
            Main_COM_send_status();
            break;
        }

        default: {
            Main_COM_send_error(cmd, MAIN_COM_ERR_BAD_CMD);
            break;
        }
    }
}

// 下位机通信回调函数
void Main_COM_Response_CALLBACK(void)
{
    static Main_COM_Package_t package;
    uint32_t now_ms = Main_COM_now_ms();

    if (Main_COM_package_ready != 0u) {
        __disable_irq();
        memcpy(&package, &Package_get, sizeof(package));
        Package_get.done = 0;
        Main_COM_package_ready = 0;
        __enable_irq();

        Main_COM_process_package(&package);
        now_ms = Main_COM_now_ms();
    }

    if ((Main_COM_motion_active != 0u) &&
        ((uint32_t)(now_ms - Main_COM_last_motion_ms) > MAIN_COM_CMD_TIMEOUT_MS)) {
        Main_COM_apply_brake();
        Main_COM_send_error(MAIN_COM_CMD_RUN, MAIN_COM_ERR_TIMEOUT);
        Main_COM_send_status();
    }

    if ((Main_COM_feedback_interval_ms > 0u) &&
        ((uint32_t)(now_ms - Main_COM_last_feedback_ms) >= Main_COM_feedback_interval_ms)) {
        Main_COM_last_feedback_ms = now_ms;
        Main_COM_send_status();
    }
}

// Nano 上位机解包函数
void Main_COM_RX_CALLBACK(uint8_t dat)
{
    switch (Main_COM_rx_state) {
        case Main_COM_RX_HEAD1: {
            if (dat == MAIN_COM_FRAME_HEAD1) {
                Main_COM_rx_state = Main_COM_RX_HEAD2;
            }
            break;
        }

        case Main_COM_RX_HEAD2: {
            if (dat == MAIN_COM_FRAME_HEAD2) {
                Main_COM_rx_state = Main_COM_RX_VERSION;
            } else {
                Main_COM_restart_rx(dat);
            }
            break;
        }

        case Main_COM_RX_VERSION: {
            if (dat != MAIN_COM_FRAME_VERSION) {
                Main_COM_restart_rx(dat);
                break;
            }
            Main_COM_rx_check = dat;
            Main_COM_rx_state = Main_COM_RX_SEQ;
            break;
        }

        case Main_COM_RX_SEQ: {
            Main_COM_rx_seq = dat;
            Main_COM_rx_check += dat;
            Main_COM_rx_state = Main_COM_RX_CMD;
            break;
        }

        case Main_COM_RX_CMD: {
            Main_COM_rx_package.CMD = (Main_CMD_ENUM)dat;
            Main_COM_rx_check += dat;
            Main_COM_rx_state = Main_COM_RX_LEN;
            break;
        }

        case Main_COM_RX_LEN: {
            if (dat > MAIN_COM_MAX_PAYLOAD_LEN) {
                Main_COM_restart_rx(dat);
                break;
            }
            Main_COM_rx_package.load_len = dat;
            Main_COM_rx_payload_index = 0;
            Main_COM_rx_check += dat;
            if (dat == 0u) {
                Main_COM_rx_state = Main_COM_RX_CHECK;
            } else {
                Main_COM_rx_state = Main_COM_RX_PAYLOAD;
            }
            break;
        }

        case Main_COM_RX_PAYLOAD: {
            Main_COM_rx_package.load[Main_COM_rx_payload_index++] = dat;
            Main_COM_rx_check += dat;
            if (Main_COM_rx_payload_index >= Main_COM_rx_package.load_len) {
                Main_COM_rx_state = Main_COM_RX_CHECK;
            }
            break;
        }

        case Main_COM_RX_CHECK: {
            if (dat != Main_COM_rx_check) {
                Main_COM_restart_rx(dat);
                break;
            }
            Main_COM_rx_package.check = dat;
            Main_COM_rx_state = Main_COM_RX_TAIL1;
            break;
        }

        case Main_COM_RX_TAIL1: {
            if (dat == MAIN_COM_FRAME_TAIL1) {
                Main_COM_rx_state = Main_COM_RX_TAIL2;
            } else {
                Main_COM_restart_rx(dat);
            }
            break;
        }

        case Main_COM_RX_TAIL2: {
            if (dat == MAIN_COM_FRAME_TAIL2) {
                if (Main_COM_package_ready == 0u) {
                    Main_COM_rx_package.done = 1;
                    Package_get = Main_COM_rx_package;
                    Main_COM_package_ready = 1;
                }
                (void)Main_COM_rx_seq;
                Main_COM_reset_rx();
            } else {
                Main_COM_restart_rx(dat);
            }
            break;
        }

        default: {
            Main_COM_reset_rx();
            break;
        }
    }
}

// uart3 接收回调
void CN_UART3_RX_CALLBACK(uint8_t dat)
{
    Main_COM_RX_CALLBACK(dat);
}


//----------------------------------------vofa+ 上位机调试----------------------------------------//
volatile Motor_Tuning_Params_t motor_tune = {0};
volatile bool param_updated_flag = 0;
uint8_t motor_tun_flag = 0;

static uint8_t vofa_receive_buffer[9];
static uint8_t vofa_receive_count = 0;

// uart1 接收回调
void CN_UART1_RX_CALLBACK(uint8_t dat)
{
    VOFA_Single_Byte_Process(dat);
}

// VOFA+ 上位机解包函数
void VOFA_Single_Byte_Process(uint8_t rx_byte)
{
    if (vofa_receive_count == 0u) {
        if (rx_byte == 0xAAu) {
            vofa_receive_buffer[0] = rx_byte;
            vofa_receive_count = 1;
        }
        return;
    }

    if (vofa_receive_count == 1u) {
        if (rx_byte == 0x55u) {
            vofa_receive_buffer[1] = rx_byte;
            vofa_receive_count = 2;
        } else if (rx_byte == 0xAAu) {
            vofa_receive_count = 1;
        } else {
            vofa_receive_count = 0;
        }
        return;
    }

    if ((vofa_receive_count >= 2u) && (vofa_receive_count < 9u)) {
        vofa_receive_buffer[vofa_receive_count++] = rx_byte;
    }

    if (vofa_receive_count == 9u) {
        if ((vofa_receive_buffer[7] == 0x5Au) && (vofa_receive_buffer[8] == 0xA5u)) {
            uint8_t rx_id = vofa_receive_buffer[2];
            VOFA_Float_Union rx_float;

            rx_float.bytes[0] = vofa_receive_buffer[3];
            rx_float.bytes[1] = vofa_receive_buffer[4];
            rx_float.bytes[2] = vofa_receive_buffer[5];
            rx_float.bytes[3] = vofa_receive_buffer[6];

            switch (rx_id) {
                case 0: {
                    motor_tune.target_angle = rx_float.f_data;
                    motor_tun_flag = 1;
                    break;
                }
                case 1:
                    motor_tune.Kp_angle = rx_float.f_data;
                    break;
                case 2:
                    motor_tune.Ki_angle = rx_float.f_data;
                    break;
                case 3:
                    motor_tune.Kd_angle = rx_float.f_data;
                    break;
                case 4:
                    motor_tune.target_vel = rx_float.f_data;
                    break;
                case 5:
                    motor_tune.Kp_vel = rx_float.f_data;
                    break;
                case 6:
                    motor_tune.Ki_vel = rx_float.f_data;
                    break;
                case 7:
                    motor_tune.Kd_vel = rx_float.f_data;
                    break;
                default:
                    break;
            }

            param_updated_flag = true;
        }

        vofa_receive_count = 0;
    }
}
//----------------------------------------vofa+ 上位机调试----------------------------------------//

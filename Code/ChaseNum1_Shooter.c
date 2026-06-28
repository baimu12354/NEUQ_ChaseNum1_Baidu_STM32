#include "ChaseNum1_Include.h"
#include <string.h>

//射击舵机协议帧头第 1 字节。
#define SHOOTER_SERVO_HEAD1                0x12U
//射击舵机协议帧头第 2 字节。
#define SHOOTER_SERVO_HEAD2                0x4CU
//射击舵机高级单圈速度位置控制命令。
#define SHOOTER_SERVO_CMD_ADV_SINGLE_SPEED 0x0CU

//射击模块状态机阶段结构。
typedef struct {
    uint8_t state;
    uint8_t friction_on;
    uint8_t pending_event;
    uint16_t pwm_duty;
    int16_t logic_servo_pos;
    uint32_t state_start_ms;
    uint32_t action_start_ms;
    uint32_t wait_ms;
} Shooter_Handle_t;

//TIM16 专用于射击摩擦轮 PWM。
static TIM_HandleTypeDef TIM16_Shooter_Handle = {0};
//射击模块运行句柄。
static Shooter_Handle_t Shooter_Handle = {0};

static void Shooter_PWM_GPIO_Init(void);
static void Shooter_PWM_TIM_Init(void);
static void Shooter_PWM_Set_Duty(uint16_t duty);
static void Shooter_Friction_Start(void);
static void Shooter_Friction_Stop(void);
static void Shooter_DIR_Init(void);
static void Shooter_DIR_Set_Forward(void);
static void Shooter_Servo_Send_Packet(uint8_t cmd_id, uint8_t *content, uint8_t length);
static void Shooter_Servo_Single_Turn_Speed(int16_t position);
static uint32_t Shooter_Estimate_Servo_Time_MS(int16_t from_pos, int16_t to_pos, uint32_t extra_ms);
static void Shooter_Set_Event(uint8_t event);
static void Shooter_Enter_State(uint8_t state, uint32_t wait_ms);

//射击模块初始化，默认关闭摩擦轮并让舵机回初始位置。
void Shooter_Init(void)
{
    memset(&Shooter_Handle, 0, sizeof(Shooter_Handle));
    Shooter_Handle.state = SHOOTER_STATE_IDLE;
    Shooter_Handle.logic_servo_pos = SHOOTER_SERVO_HOME_POS;
    Shooter_Handle.pwm_duty = 0U;

    Shooter_DIR_Init();
    Shooter_PWM_GPIO_Init();
    Shooter_PWM_TIM_Init();
    CN_UART5_init();

    // Shooter_PWM_Set_Duty(1000U);

    Shooter_Friction_Stop();
    Shooter_Servo_Single_Turn_Speed(SHOOTER_SERVO_HOME_POS);
}

//启动一次射击动作，忙时返回 0。
uint8_t Shooter_Start(void)
{
    if (Shooter_Is_Busy() != 0U) {
        return 0U;
    }

    Shooter_Handle.pending_event = SHOOTER_EVENT_NONE;
    Shooter_Handle.action_start_ms = HAL_GetTick();
    Shooter_DIR_Set_Forward();
    Shooter_Friction_Start();
    Shooter_Enter_State(SHOOTER_STATE_SPINUP, SHOOTER_SPINUP_MS);
    return 1U;
}

//立即关闭摩擦轮并让舵机回初始位置。
void Shooter_Reset(void)
{
    uint32_t wait_ms = 0U;
    int16_t estimate_from_pos = Shooter_Handle.logic_servo_pos;

    Shooter_Friction_Stop();
    Shooter_Handle.pending_event = SHOOTER_EVENT_NONE;
    Shooter_Servo_Single_Turn_Speed(SHOOTER_SERVO_HOME_POS);

    if (Shooter_Handle.state == SHOOTER_STATE_RETURNING ||
        Shooter_Handle.state == SHOOTER_STATE_RESETTING) {
        estimate_from_pos = SHOOTER_SERVO_PUSH_POS;
    }

    wait_ms = Shooter_Estimate_Servo_Time_MS(estimate_from_pos,
                                             SHOOTER_SERVO_HOME_POS,
                                             SHOOTER_RESET_EXTRA_WAIT_MS);
    Shooter_Handle.logic_servo_pos = SHOOTER_SERVO_HOME_POS;
    Shooter_Handle.action_start_ms = HAL_GetTick();
    Shooter_Enter_State(SHOOTER_STATE_RESETTING, wait_ms);
}

//射击模块非阻塞任务循环，需要在主循环中周期调用。
void Shooter_Task_Loop(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (Shooter_Handle.state == SHOOTER_STATE_IDLE ||
        Shooter_Handle.state == SHOOTER_STATE_FAULT) {
        return;
    }

    if ((uint32_t)(now_ms - Shooter_Handle.action_start_ms) > SHOOTER_ACTION_TIMEOUT_MS) {
        Shooter_Friction_Stop();
        Shooter_Handle.state = SHOOTER_STATE_IDLE;
        Shooter_Set_Event(SHOOTER_EVENT_TIMEOUT);
        return;
    }

    if ((uint32_t)(now_ms - Shooter_Handle.state_start_ms) < Shooter_Handle.wait_ms) {
        return;
    }

    if (Shooter_Handle.state == SHOOTER_STATE_SPINUP) {
        Shooter_Servo_Single_Turn_Speed(SHOOTER_SERVO_PUSH_POS);
        Shooter_Handle.wait_ms = Shooter_Estimate_Servo_Time_MS(Shooter_Handle.logic_servo_pos,
                                                                SHOOTER_SERVO_PUSH_POS,
                                                                SHOOTER_SERVO_EXTRA_WAIT_MS);
        Shooter_Handle.logic_servo_pos = SHOOTER_SERVO_PUSH_POS;
        Shooter_Enter_State(SHOOTER_STATE_PUSHING, Shooter_Handle.wait_ms);
    } else if (Shooter_Handle.state == SHOOTER_STATE_PUSHING) {
        Shooter_Friction_Stop();
        Shooter_Servo_Single_Turn_Speed(SHOOTER_SERVO_HOME_POS);
        Shooter_Handle.wait_ms = Shooter_Estimate_Servo_Time_MS(Shooter_Handle.logic_servo_pos,
                                                                SHOOTER_SERVO_HOME_POS,
                                                                SHOOTER_SERVO_EXTRA_WAIT_MS);
        Shooter_Handle.logic_servo_pos = SHOOTER_SERVO_HOME_POS;
        Shooter_Enter_State(SHOOTER_STATE_RETURNING, Shooter_Handle.wait_ms);
    } else if (Shooter_Handle.state == SHOOTER_STATE_RETURNING) {
        Shooter_Handle.state = SHOOTER_STATE_IDLE;
        Shooter_Set_Event(SHOOTER_EVENT_DONE);
    } else if (Shooter_Handle.state == SHOOTER_STATE_RESETTING) {
        Shooter_Handle.state = SHOOTER_STATE_IDLE;
        Shooter_Set_Event(SHOOTER_EVENT_RESET_DONE);
    }
}

//读取并清除射击模块事件，读到事件返回 1。
uint8_t Shooter_Fetch_Event(uint8_t *event)
{
    if (event == NULL) {
        return 0U;
    }

    if (Shooter_Handle.pending_event == SHOOTER_EVENT_NONE) {
        return 0U;
    }

    *event = Shooter_Handle.pending_event;
    Shooter_Handle.pending_event = SHOOTER_EVENT_NONE;
    return 1U;
}

//读取射击模块当前状态。
uint8_t Shooter_Get_State(void)
{
    return Shooter_Handle.state;
}

//读取射击模块是否忙碌。
uint8_t Shooter_Is_Busy(void)
{
    return (Shooter_Handle.state != SHOOTER_STATE_IDLE &&
            Shooter_Handle.state != SHOOTER_STATE_FAULT) ? 1U : 0U;
}

//读取摩擦轮是否正在输出 PWM。
uint8_t Shooter_Is_Friction_On(void)
{
    return Shooter_Handle.friction_on;
}

//读取当前摩擦轮 PWM 占空比计数值。
uint16_t Shooter_Get_PWM_Duty(void)
{
    return Shooter_Handle.pwm_duty;
}

//读取当前逻辑舵机位置，单位 0.1 度。
int16_t Shooter_Get_Logic_Servo_Pos(void)
{
    return Shooter_Handle.logic_servo_pos;
}

//初始化 PB8 为 TIM16_CH1 PWM 输出。
static void Shooter_PWM_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_Initure.Pin = SHOOTER_PWM_GPIO_PIN;
    GPIO_Initure.Mode = GPIO_MODE_AF_PP;
    GPIO_Initure.Pull = GPIO_NOPULL;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_Initure.Alternate = SHOOTER_PWM_GPIO_AF;
    HAL_GPIO_Init(SHOOTER_PWM_GPIO_PORT, &GPIO_Initure);
}

//初始化 TIM16，给 PB8 输出摩擦轮 PWM。
static void Shooter_PWM_TIM_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    __HAL_RCC_TIM16_CLK_ENABLE();
    TIM16_Shooter_Handle.Instance = TIM16;
    TIM16_Shooter_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM16_Shooter_Handle.Init.Prescaler = SHOOTER_PWM_PRESCALER_COUNT - 1U;
    TIM16_Shooter_Handle.Init.Period = SHOOTER_PWM_PERIOD_COUNT - 1U;
    TIM16_Shooter_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM16_Shooter_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&TIM16_Shooter_Handle);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0U;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&TIM16_Shooter_Handle, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TIM16_Shooter_Handle, TIM_CHANNEL_1);
    Shooter_PWM_Set_Duty(0U);
}

//限制并写入摩擦轮 PWM 占空比。
static void Shooter_PWM_Set_Duty(uint16_t duty)
{
    if (duty > SHOOTER_PWM_PERIOD_COUNT) {
        duty = SHOOTER_PWM_PERIOD_COUNT;
    }

    Shooter_Handle.pwm_duty = duty;
    __HAL_TIM_SET_COMPARE(&TIM16_Shooter_Handle, TIM_CHANNEL_1, duty);
}

//开启摩擦轮。
static void Shooter_Friction_Start(void)
{
    Shooter_Handle.friction_on = 1U;
    Shooter_PWM_Set_Duty(SHOOTER_PWM_DEFAULT_DUTY);
}

//关闭摩擦轮。
static void Shooter_Friction_Stop(void)
{
    Shooter_Handle.friction_on = 0U;
    Shooter_PWM_Set_Duty(0U);
}

//初始化 PB7 为 DRV8701 方向输出。
static void Shooter_DIR_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_Initure.Pin = SHOOTER_DIR_GPIO_PIN;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull = GPIO_NOPULL;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SHOOTER_DIR_GPIO_PORT, &GPIO_Initure);
    Shooter_DIR_Set_Forward();
}

//设置 DRV8701 为射击正转方向。
static void Shooter_DIR_Set_Forward(void)
{
    HAL_GPIO_WritePin(SHOOTER_DIR_GPIO_PORT, SHOOTER_DIR_GPIO_PIN, SHOOTER_DIR_FORWARD_LEVEL);
}

//向 UART5 上的射击舵机发送一帧最小总线舵机协议。
static void Shooter_Servo_Send_Packet(uint8_t cmd_id, uint8_t *content, uint8_t length)
{
    uint8_t buffer[32] = {0U};
    uint32_t sum = 0U;
    uint8_t i = 0U;

    if (length > 24U) {
        return;
    }

    buffer[0] = SHOOTER_SERVO_HEAD1;
    buffer[1] = SHOOTER_SERVO_HEAD2;
    buffer[2] = cmd_id;
    buffer[3] = length;

    if ((content != NULL) && (length > 0U)) {
        memcpy(&buffer[4], content, length);
    }

    for (i = 0U; i < (uint8_t)(4U + length); i++) {
        sum += buffer[i];
    }
    buffer[4U + length] = (uint8_t)(sum & 0xFFU);

    CN_UART_write_fifo(CN_UART_5, buffer, (uint32_t)(4U + length + 1U));
}

//发送高级单圈速度位置控制，只用于射击舵机。
static void Shooter_Servo_Single_Turn_Speed(int16_t position)
{
    uint8_t content[11] = {0U};

    content[0] = SHOOTER_SERVO_ID;
    content[1] = (uint8_t)(position & 0xFF);
    content[2] = (uint8_t)((position >> 8) & 0xFF);
    content[3] = (uint8_t)(SHOOTER_SERVO_SPEED & 0xFFU);
    content[4] = (uint8_t)((SHOOTER_SERVO_SPEED >> 8) & 0xFFU);
    content[5] = (uint8_t)(SHOOTER_SERVO_ACCEL & 0xFFU);
    content[6] = (uint8_t)((SHOOTER_SERVO_ACCEL >> 8) & 0xFFU);
    content[7] = (uint8_t)(SHOOTER_SERVO_DECEL & 0xFFU);
    content[8] = (uint8_t)((SHOOTER_SERVO_DECEL >> 8) & 0xFFU);
    content[9] = (uint8_t)(SHOOTER_SERVO_POWER & 0xFFU);
    content[10] = (uint8_t)((SHOOTER_SERVO_POWER >> 8) & 0xFFU);

    Shooter_Servo_Send_Packet(SHOOTER_SERVO_CMD_ADV_SINGLE_SPEED, content, sizeof(content));
}

//根据逻辑角度差和速度估算舵机等待时间。
static uint32_t Shooter_Estimate_Servo_Time_MS(int16_t from_pos, int16_t to_pos, uint32_t extra_ms)
{
    int32_t diff = (int32_t)to_pos - (int32_t)from_pos;
    uint32_t abs_diff = 0U;
    uint32_t move_ms = 0U;

    if (diff < 0) {
        diff = -diff;
    }
    abs_diff = (uint32_t)diff;

    if (SHOOTER_SERVO_SPEED == 0U) {
        return extra_ms;
    }

    move_ms = (abs_diff * 1000U + SHOOTER_SERVO_SPEED - 1U) / SHOOTER_SERVO_SPEED;
    return move_ms + extra_ms;
}

//记录待上报事件，保留最近一次事件。
static void Shooter_Set_Event(uint8_t event)
{
    Shooter_Handle.pending_event = event;
}

//切换状态并记录该阶段开始时间和等待时间。
static void Shooter_Enter_State(uint8_t state, uint32_t wait_ms)
{
    Shooter_Handle.state = state;
    Shooter_Handle.state_start_ms = HAL_GetTick();
    Shooter_Handle.wait_ms = wait_ms;
}

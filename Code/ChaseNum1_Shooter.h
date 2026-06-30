#ifndef __CHASE_NUM1_SHOOTER_H__
#define __CHASE_NUM1_SHOOTER_H__

#include "stm32h7xx_hal.h"
#include <stdint.h>

//射击摩擦轮 PWM 使用 PB8/TIM16_CH1。
#define SHOOTER_PWM_GPIO_PORT              GPIOB
//射击摩擦轮 PWM 引脚。
#define SHOOTER_PWM_GPIO_PIN               GPIO_PIN_8
//射击摩擦轮 PWM 引脚复用功能。
#define SHOOTER_PWM_GPIO_AF                GPIO_AF1_TIM16

//射击摩擦轮方向使用 PB7，输出到 DRV8701 的 DIR 引脚。
#define SHOOTER_DIR_GPIO_PORT              GPIOB
//射击摩擦轮方向引脚。
#define SHOOTER_DIR_GPIO_PIN               GPIO_PIN_7
//摩擦轮正转时 DIR 引脚电平，方向反了就改这里。
#define SHOOTER_DIR_FORWARD_LEVEL          GPIO_PIN_SET

//TIM16 输入时钟估算值，当前工程 APB2 定时器时钟约 200MHz。
#define SHOOTER_PWM_BASE_CLOCK_HZ          200000000U
//TIM16 预分频计数值，对应 Prescaler = SHOOTER_PWM_PRESCALER_COUNT - 1。
#define SHOOTER_PWM_PRESCALER_COUNT        10U
//TIM16 自动重装载计数值，占空比范围为 0~SHOOTER_PWM_PERIOD_COUNT。
#define SHOOTER_PWM_PERIOD_COUNT           10000U
//摩擦轮默认占空比，范围 0~SHOOTER_PWM_PERIOD_COUNT。
#define SHOOTER_PWM_DEFAULT_DUTY           3000U

//射击舵机总线 ID。
#define SHOOTER_SERVO_ID                   4U
//射击舵机初始位置，单位 0.1 度。
#define SHOOTER_SERVO_HOME_POS             1650
//射击舵机推弹位置，单位 0.1 度。
#define SHOOTER_SERVO_PUSH_POS             -40
//射击舵机运动速度，单位 0.1 度/秒。
#define SHOOTER_SERVO_SPEED                3000U
//射击舵机加速参数，沿用舵机协议定义。
#define SHOOTER_SERVO_ACCEL                100U
//射击舵机减速参数，沿用舵机协议定义。
#define SHOOTER_SERVO_DECEL                100U
//射击舵机功率限制，0 表示使用舵机默认限制。
#define SHOOTER_SERVO_POWER                0U

//摩擦轮预热时间，收到射击命令后先等待该时间再推弹。
#define SHOOTER_SPINUP_MS                  000U
//舵机运动估算后额外增加的等待裕量。
#define SHOOTER_SERVO_EXTRA_WAIT_MS        500U
//复位回位时额外增加的等待裕量。
#define SHOOTER_RESET_EXTRA_WAIT_MS        500U
//一次射击动作最大允许时间，超时后强制关摩擦轮并上报超时。
#define SHOOTER_ACTION_TIMEOUT_MS          8000U

//射击模块内部状态：空闲。
#define SHOOTER_STATE_IDLE                 0U
//射击模块内部状态：摩擦轮预热中。
#define SHOOTER_STATE_SPINUP               1U
//射击模块内部状态：舵机正在推弹。
#define SHOOTER_STATE_PUSHING              2U
//射击模块内部状态：舵机正在回初始位置。
#define SHOOTER_STATE_RETURNING            3U
//射击模块内部状态：正在复位。
#define SHOOTER_STATE_RESETTING            4U
//射击模块内部状态：故障预留。
#define SHOOTER_STATE_FAULT                5U

//射击模块事件：无事件。
#define SHOOTER_EVENT_NONE                 0U
//射击模块事件：一次射击动作完成。
#define SHOOTER_EVENT_DONE                 1U
//射击模块事件：复位动作完成。
#define SHOOTER_EVENT_RESET_DONE           2U
//射击模块事件：动作超时。
#define SHOOTER_EVENT_TIMEOUT              3U

//射击模块初始化，默认关闭摩擦轮并让舵机回初始位置。
void Shooter_Init(void);
//射击模块非阻塞任务循环，需要在主循环中周期调用。
void Shooter_Task_Loop(void);
//启动一次完整射击动作，忙时返回 0。
uint8_t Shooter_Start(void);
//立即关闭摩擦轮并让舵机回初始位置。
void Shooter_Reset(void);
//读取并清除射击模块事件，读到事件返回 1。
uint8_t Shooter_Fetch_Event(uint8_t *event);
//读取射击模块当前状态。
uint8_t Shooter_Get_State(void);
//读取射击模块是否忙碌。
uint8_t Shooter_Is_Busy(void);
//读取摩擦轮是否正在输出 PWM。
uint8_t Shooter_Is_Friction_On(void);
//读取当前摩擦轮 PWM 占空比计数值。
uint16_t Shooter_Get_PWM_Duty(void);
//读取当前逻辑舵机位置，单位 0.1 度。
int16_t Shooter_Get_Logic_Servo_Pos(void);

#endif

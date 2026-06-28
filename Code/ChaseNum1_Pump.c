#include "ChaseNum1_Include.h"

// 气泵 PWM 定时器自动重装载值，对应 10000 级占空比。
#define PUMP_PWM_PERIOD_COUNT               10000U
// 气泵默认占空比，范围 0 到 PUMP_PWM_PERIOD_COUNT。
#define PUMP_DEFAULT_DUTY                   10000U
// 气泵最长连续工作时间，超时后自动停泵；0 表示不启用超时保护。
#define PUMP_AUTO_STOP_TIMEOUT_MS           0
// 置 1 后，开启吸附频率检测；检测到吸附成功后只置位状态，不自动关闭气泵。
#define PUMP_SUCK_DETECT_ENABLE             1U
//+吸附检测方向枚举：反馈频率小于等于阈值时认为吸附。
#define PUMP_SUCK_FREQ_LOW_IS_SUCKED        0U
//+吸附检测方向枚举：反馈频率大于等于阈值时认为吸附。
#define PUMP_SUCK_FREQ_HIGH_IS_SUCKED       1U
//+吸附检测方向选择：LOW 表示频率小于等于阈值吸附，HIGH 表示频率大于等于阈值吸附。
#define PUMP_SUCK_FREQ_COMPARE_MODE         PUMP_SUCK_FREQ_LOW_IS_SUCKED 
//+气压反馈频率阈值，单位 Hz；具体比较方向由 PUMP_SUCK_FREQ_COMPARE_MODE 决定。
#define PUMP_SUCK_FREQ_THRESHOLD_HZ         41.5f           //  需要根据实际情况修改
// 吸附判断采样间隔，单位 ms；每隔该时间把当前频率写入一次判断缓冲区。
#define PUMP_SUCK_CHECK_INTERVAL_MS         20U             
// 吸附判断缓冲区长度；数值越大，判断越稳，但响应越慢。
#define PUMP_SUCK_FREQ_FILTER_SIZE          100U             //  需要根据实际情况修改
// 缓冲区中达到吸附频率条件的次数大于等于该值时，认为吸附成功。
#define PUMP_SUCK_FREQ_CONFIRM_COUNT        20U             //  需要根据实际情况修改
// 吸附兜底判定时间，单位 ms；超过该时间仍未检测到吸附时，直接判定为吸附成功，0 表示关闭该功能。
#define PUMP_SUCK_FORCE_DONE_TIMEOUT_MS     5000U           //  需要根据实际情况修改
// TIM15 基准时钟，当前工程 APB2=100MHz 且定时器倍频后约为 200MHz。
#define PUMP_TIM_BASE_CLOCK_HZ              200000000U
// TIM15 预分频系数，对应 Prescaler = PUMP_TIM_PRESCALER_COUNT - 1。
#define PUMP_TIM_PRESCALER_COUNT            10U
// TIM15 计数频率，PE6 输入捕获频率计算使用该值。
#define PUMP_TIM_TICK_HZ                    (PUMP_TIM_BASE_CLOCK_HZ / PUMP_TIM_PRESCALER_COUNT)
// PE6 反馈信号输入捕获滤波等级，0 表示不滤波，信号抖动明显时可适当增大。
#define PUMP_FEEDBACK_IC_FILTER             0U
// 超过该时间没有新的上升沿，则读取频率时返回 0，单位 ms。
#define PUMP_FEEDBACK_TIMEOUT_MS            200U
// 反馈信号超时对应的 TIM15 计数 tick。
#define PUMP_FEEDBACK_TIMEOUT_TICKS         ((PUMP_TIM_TICK_HZ / 1000U) * PUMP_FEEDBACK_TIMEOUT_MS)
// 频率滤波使用的周期样本数量；数量越大越平稳，但响应越慢。
#define PUMP_FREQ_PERIOD_FILTER_SIZE        10U
// 频率 IIR 低通滤波系数，范围 0~1；越小越平稳，越大响应越快。
#define PUMP_FREQ_IIR_ALPHA                 0.2f

#if (PUMP_SUCK_FREQ_COMPARE_MODE != PUMP_SUCK_FREQ_LOW_IS_SUCKED) && \
    (PUMP_SUCK_FREQ_COMPARE_MODE != PUMP_SUCK_FREQ_HIGH_IS_SUCKED)
#error "PUMP_SUCK_FREQ_COMPARE_MODE must be PUMP_SUCK_FREQ_LOW_IS_SUCKED or PUMP_SUCK_FREQ_HIGH_IS_SUCKED"
#endif

// 气泵全局句柄
Pump_Handle_Struct Pump_Handle={0};

// TIM15 同时用于 PE5 PWM 输出和 PE6 输入捕获。
TIM_HandleTypeDef TIM15_Motor_Handle={0};

// 当前 PWM 占空比，范围 0 到 PUMP_PWM_PERIOD_COUNT。
static uint16_t g_pump_pwm_duty = PUMP_DEFAULT_DUTY;
#if (PUMP_AUTO_STOP_TIMEOUT_MS > 0U) || (PUMP_SUCK_FORCE_DONE_TIMEOUT_MS > 0U)
// 气泵启动时刻，用于自动停泵保护和吸附兜底判定。
static uint32_t g_pump_start_ms = 0U;
#endif
// TIM15 溢出次数，用于把单个自动重装载周期扩展成更长的测量时间基准。
static volatile uint32_t g_pump_tim_overflow_count = 0U;
// 上一次 PE6 上升沿捕获到的扩展计数值。
static volatile uint32_t g_pump_last_capture_total_ticks = 0U;
// 是否已经捕获到第一帧有效上升沿。
static volatile uint8_t g_pump_feedback_capture_valid = 0U;
// 最近一次反馈信号周期，单位为 TIM15 计数 tick。
static volatile uint32_t g_pump_feedback_period_ticks = 0U;
// 最近一次滤波后的反馈频率，单位 Hz。
static volatile float g_pump_feedback_freq_hz = 0.0f;
// 最近一次捕获到上升沿的系统时间，用于判断反馈信号是否已经停止。
static volatile uint32_t g_pump_last_capture_ms = 0U;
// 频率滤波周期缓冲区，保存最近若干次输入捕获得到的周期 tick。
static volatile uint32_t g_pump_period_filter_buffer[PUMP_FREQ_PERIOD_FILTER_SIZE] = {0U};
// 频率滤波周期缓冲区当前写入位置。
static volatile uint8_t g_pump_period_filter_index = 0U;
// 频率滤波周期缓冲区有效样本数量。
static volatile uint8_t g_pump_period_filter_count = 0U;
// 输入捕获每写入一次新周期，该版本号加 1，用于保证 IIR 每个新样本只更新一次。
static volatile uint32_t g_pump_period_filter_version = 0U;
// 主循环已经处理到的周期样本版本号。
static uint32_t g_pump_period_filter_last_version = 0U;
// 滤波频率是否已经有过有效初值。
static uint8_t g_pump_freq_filter_valid = 0U;
// 吸附判断缓冲区，1 表示该次频率大于等于阈值，0 表示未达到吸附条件。
static uint8_t g_pump_suck_freq_buffer[PUMP_SUCK_FREQ_FILTER_SIZE] = {0U};
// 吸附判断缓冲区当前写入位置。
static uint8_t g_pump_suck_freq_index = 0U;
// 吸附判断缓冲区是否已经写满过一轮。
static uint8_t g_pump_suck_freq_buffer_full = 0U;
// 最近一次执行吸附判断采样的时间。
static uint32_t g_pump_last_suck_check_ms = 0U;

static void CN_Pump_Feedback_Reset(void);
static void CN_Pump_Feedback_Start(void);
static void CN_Pump_Feedback_Stop(void);
static void CN_Pump_Freq_Filter_Reset(void);
static void CN_Pump_Freq_Filter_Push(uint32_t period_ticks);
static float CN_Pump_Freq_Filter_Update(void);
static void CN_Pump_Suck_Detect_Reset(void);
static uint8_t CN_Pump_Is_Sucked_Freq(float freq_hz);
static void CN_Pump_Suck_Detect_Update(float freq_hz);

// 气泵 - 将占空比限制在定时器允许范围内
static uint16_t CN_Pump_Clamp_Duty(uint16_t duty)
{
    return (duty > PUMP_PWM_PERIOD_COUNT) ? PUMP_PWM_PERIOD_COUNT : duty;
}

// 气泵 - 将占空比写入 TIM15_CH1
static void CN_Pump_Apply_Duty(uint16_t duty)
{
    g_pump_pwm_duty = CN_Pump_Clamp_Duty(duty);
    __HAL_TIM_SET_COMPARE(&TIM15_Motor_Handle, TIM_CHANNEL_1, g_pump_pwm_duty);
}

// 气泵 - 清空频率滤波缓冲区和滤波输出
static void CN_Pump_Freq_Filter_Reset(void)
{
    uint8_t i = 0U;

    for (i = 0U; i < PUMP_FREQ_PERIOD_FILTER_SIZE; i++) {
        g_pump_period_filter_buffer[i] = 0U;
    }

    g_pump_period_filter_index = 0U;
    g_pump_period_filter_count = 0U;
    g_pump_period_filter_version = 0U;
    g_pump_period_filter_last_version = 0U;
    g_pump_freq_filter_valid = 0U;
    g_pump_feedback_freq_hz = 0.0f;
}

// 气泵 - 把一次输入捕获得到的周期写入滤波缓冲区
static void CN_Pump_Freq_Filter_Push(uint32_t period_ticks)
{
    if (period_ticks == 0U) {
        return;
    }

    g_pump_period_filter_buffer[g_pump_period_filter_index] = period_ticks;
    g_pump_period_filter_index++;
    if (g_pump_period_filter_index >= PUMP_FREQ_PERIOD_FILTER_SIZE) {
        g_pump_period_filter_index = 0U;
    }

    if (g_pump_period_filter_count < PUMP_FREQ_PERIOD_FILTER_SIZE) {
        g_pump_period_filter_count++;
    }

    g_pump_period_filter_version++;
}

// 气泵 - 根据周期缓冲区计算滤波后的反馈频率，单位 Hz
static float CN_Pump_Freq_Filter_Update(void)
{
    uint32_t samples[PUMP_FREQ_PERIOD_FILTER_SIZE] = {0U};
    uint32_t sample_version = 0U;
    uint32_t sum_ticks = 0U;
    uint32_t min_ticks = 0U;
    uint32_t max_ticks = 0U;
    uint8_t sample_count = 0U;
    uint8_t used_count = 0U;
    uint8_t i = 0U;
    float avg_period_ticks = 0.0f;
    float new_freq_hz = 0.0f;

    __disable_irq();
    sample_version = g_pump_period_filter_version;
    if (sample_version != g_pump_period_filter_last_version) {
        sample_count = g_pump_period_filter_count;
        for (i = 0U; i < sample_count; i++) {
            samples[i] = g_pump_period_filter_buffer[i];
        }
    }
    __enable_irq();

    if (sample_version == g_pump_period_filter_last_version) {
        return g_pump_feedback_freq_hz;
    }

    g_pump_period_filter_last_version = sample_version;
    if (sample_count == 0U) {
        return g_pump_feedback_freq_hz;
    }

    min_ticks = samples[0];
    max_ticks = samples[0];
    for (i = 0U; i < sample_count; i++) {
        sum_ticks += samples[i];
        if (samples[i] < min_ticks) {
            min_ticks = samples[i];
        }
        if (samples[i] > max_ticks) {
            max_ticks = samples[i];
        }
    }

    used_count = sample_count;
    if (sample_count >= 4U) {
        sum_ticks -= min_ticks;
        sum_ticks -= max_ticks;
        used_count = sample_count - 2U;
    }

    if (used_count == 0U || sum_ticks == 0U) {
        return g_pump_feedback_freq_hz;
    }

    avg_period_ticks = (float)sum_ticks / (float)used_count;
    new_freq_hz = (float)PUMP_TIM_TICK_HZ / avg_period_ticks;

    if (g_pump_freq_filter_valid == 0U) {
        g_pump_feedback_freq_hz = new_freq_hz;
        g_pump_freq_filter_valid = 1U;
    } else {
        g_pump_feedback_freq_hz += (new_freq_hz - g_pump_feedback_freq_hz) * PUMP_FREQ_IIR_ALPHA;
    }

    Pump_Handle.FeedbackFreqHz = g_pump_feedback_freq_hz;
    return g_pump_feedback_freq_hz;
}

// 气泵 - 清空频率测量状态
static void CN_Pump_Feedback_Reset(void)
{
    g_pump_tim_overflow_count = 0U;
    g_pump_last_capture_total_ticks = 0U;
    g_pump_feedback_capture_valid = 0U;
    g_pump_feedback_period_ticks = 0U;
    g_pump_feedback_freq_hz = 0.0f;
    g_pump_last_capture_ms = HAL_GetTick();
    CN_Pump_Freq_Filter_Reset();
    Pump_Handle.FeedbackFreqHz = 0.0f;
    Pump_Handle.FeedbackPeriodTicks = 0U;
    __HAL_TIM_SET_COUNTER(&TIM15_Motor_Handle, 0U);
}

// 气泵 - 开启 PE6 反馈频率测量
static void CN_Pump_Feedback_Start(void)
{
    CN_Pump_Feedback_Reset();
    __HAL_TIM_CLEAR_FLAG(&TIM15_Motor_Handle, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&TIM15_Motor_Handle, TIM_FLAG_CC2);
    HAL_TIM_Base_Start_IT(&TIM15_Motor_Handle);
    HAL_TIM_IC_Start_IT(&TIM15_Motor_Handle, TIM_CHANNEL_2);
}

// 气泵 - 关闭 PE6 反馈频率测量
static void CN_Pump_Feedback_Stop(void)
{
    HAL_TIM_IC_Stop_IT(&TIM15_Motor_Handle, TIM_CHANNEL_2);
    HAL_TIM_Base_Stop_IT(&TIM15_Motor_Handle);
    CN_Pump_Feedback_Reset();
}

// 气泵 - 清空吸附判断缓冲区
static void CN_Pump_Suck_Detect_Reset(void)
{
    uint8_t i = 0U;

    for (i = 0U; i < PUMP_SUCK_FREQ_FILTER_SIZE; i++) {
        g_pump_suck_freq_buffer[i] = 0U;
    }

    g_pump_suck_freq_index = 0U;
    g_pump_suck_freq_buffer_full = 0U;
    g_pump_last_suck_check_ms = HAL_GetTick();
    Pump_Handle.IfSucked = 0U;
    Pump_Handle.SuckStrength = 0U;
}

//+气泵 - 根据宏定义选择吸附频率判定方向。
static uint8_t CN_Pump_Is_Sucked_Freq(float freq_hz)
{
#if PUMP_SUCK_FREQ_COMPARE_MODE == PUMP_SUCK_FREQ_LOW_IS_SUCKED
    return (freq_hz <= PUMP_SUCK_FREQ_THRESHOLD_HZ) ? 1U : 0U;
#else
    return (freq_hz >= PUMP_SUCK_FREQ_THRESHOLD_HZ) ? 1U : 0U;
#endif
}

// 气泵 - 根据当前频率更新吸附判断结果
static void CN_Pump_Suck_Detect_Update(float freq_hz)
{
#if PUMP_SUCK_DETECT_ENABLE
    uint8_t i = 0U;
    uint8_t valid_count = 0U;
    uint8_t sucked_sample_count = 0U;

    g_pump_suck_freq_buffer[g_pump_suck_freq_index] = CN_Pump_Is_Sucked_Freq(freq_hz);

    g_pump_suck_freq_index++;
    if (g_pump_suck_freq_index >= PUMP_SUCK_FREQ_FILTER_SIZE) {
        g_pump_suck_freq_index = 0U;
        g_pump_suck_freq_buffer_full = 1U;
    }

    valid_count = (g_pump_suck_freq_buffer_full != 0U) ?
        PUMP_SUCK_FREQ_FILTER_SIZE : g_pump_suck_freq_index;

    for (i = 0U; i < valid_count; i++) {
        if (g_pump_suck_freq_buffer[i] != 0U) {
            sucked_sample_count++;
        }
    }

    Pump_Handle.SuckStrength = (uint32_t)freq_hz;

    if (Pump_Handle.IfSucked != 0U) {
        return;
    }

    if (sucked_sample_count >= PUMP_SUCK_FREQ_CONFIRM_COUNT) {
        Pump_Handle.IfSucked = 1U;
    } else {
        Pump_Handle.IfSucked = 0U;
    }
#else
    (void)freq_hz;
#endif
}

// 气泵 - 初始化
void CN_Pump_init(void){
    CN_Pump_TIM_init();
    Pump_Handle.Pump_on = 0U;
    CN_Pump_Feedback_Reset();
    CN_Pump_Suck_Detect_Reset();
}

// 气泵 - 定时器驱动初始化
void CN_Pump_TIM_init(void){
    GPIO_InitTypeDef GPIO_Initure={0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_Initure.Pin=GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用模式
	GPIO_Initure.Pull=GPIO_NOPULL;              //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //高速
    GPIO_Initure.Alternate=GPIO_AF4_TIM15;
	HAL_GPIO_Init(GPIOE,&GPIO_Initure);

    __HAL_RCC_TIM15_CLK_ENABLE();
    TIM15_Motor_Handle.Instance=TIM15;
    TIM15_Motor_Handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//200mhz
    TIM15_Motor_Handle.Init.Prescaler=PUMP_TIM_PRESCALER_COUNT-1U;//20mhz
    TIM15_Motor_Handle.Init.Period=PUMP_PWM_PERIOD_COUNT-1U;//2khz
    TIM15_Motor_Handle.Init.CounterMode=TIM_COUNTERMODE_UP;
    TIM15_Motor_Handle.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&TIM15_Motor_Handle);

    TIM_OC_InitTypeDef sConfigOC={0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;  // PWM 模式 1
    sConfigOC.Pulse = PUMP_DEFAULT_DUTY;  // 设置默认占空比
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;  // 输出极性
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;  // 禁用快速模式

    HAL_TIM_PWM_ConfigChannel(&TIM15_Motor_Handle, &sConfigOC, TIM_CHANNEL_1);

    TIM_IC_InitTypeDef sConfigIC={0};
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;  // 捕获 PE6 反馈信号的上升沿
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;        // TIM15_CH2 直接连接到 TI2
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;                  // 每个上升沿都捕获
    sConfigIC.ICFilter = PUMP_FEEDBACK_IC_FILTER;            // 输入滤波，默认不滤波
    HAL_TIM_IC_ConfigChannel(&TIM15_Motor_Handle, &sConfigIC, TIM_CHANNEL_2);

    HAL_NVIC_SetPriority(TIM15_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM15_IRQn);
    CN_Pump_Feedback_Reset();
    CN_Pump_Apply_Duty(0U);
}

// 气泵 - 获取 PE6 反馈信号频率，单位 Hz；长时间没有新上升沿时返回 0。
float CN_Pump_Get_Feedback_Freq_Hz(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (Pump_Handle.Pump_on == 0U) {
        return 0.0f;
    }

    if ((uint32_t)(now_ms - g_pump_last_capture_ms) > PUMP_FEEDBACK_TIMEOUT_MS) {
        g_pump_feedback_freq_hz = 0.0f;
        g_pump_feedback_period_ticks = 0U;
        CN_Pump_Freq_Filter_Reset();
        Pump_Handle.FeedbackFreqHz = 0.0f;
        Pump_Handle.FeedbackPeriodTicks = 0U;
        return 0.0f;
    }

    return CN_Pump_Freq_Filter_Update();
}

// 气泵 - 获取最近一次 PE6 反馈信号周期，单位为 TIM15 计数 tick。
uint32_t CN_Pump_Get_Feedback_Period_Ticks(void)
{
    return g_pump_feedback_period_ticks;
}

// 气泵 - 开启
void CN_Pump_Start(void){
    CN_Pump_Apply_Duty(PUMP_DEFAULT_DUTY);
    Pump_Handle.Pump_on = 1U;
    CN_Pump_Suck_Detect_Reset();
    CN_Pump_Feedback_Start();
#if (PUMP_AUTO_STOP_TIMEOUT_MS > 0U) || (PUMP_SUCK_FORCE_DONE_TIMEOUT_MS > 0U)
    g_pump_start_ms = HAL_GetTick();
#endif
    HAL_TIM_PWM_Start(&TIM15_Motor_Handle, TIM_CHANNEL_1);
}

// 气泵 - 关闭
void CN_Pump_Stop(void){
    CN_Pump_Apply_Duty(0U);
    Pump_Handle.Pump_on = 0U;
    HAL_TIM_PWM_Stop(&TIM15_Motor_Handle, TIM_CHANNEL_1);
    CN_Pump_Feedback_Stop();
}

// 气泵非阻塞任务：刷新反馈频率和吸附状态；如开启超时保护，则超时后自动停泵。
void CN_Pump_Task_Loop(void)
{
    uint32_t now_ms = HAL_GetTick();
    float freq_hz = 0.0f;

    if (Pump_Handle.Pump_on == 0U) return;

    freq_hz = CN_Pump_Get_Feedback_Freq_Hz();

#if PUMP_SUCK_DETECT_ENABLE
    if ((uint32_t)(now_ms - g_pump_last_suck_check_ms) >= PUMP_SUCK_CHECK_INTERVAL_MS) {
        g_pump_last_suck_check_ms = now_ms;
        CN_Pump_Suck_Detect_Update(freq_hz);
    }
#endif

#if PUMP_SUCK_FORCE_DONE_TIMEOUT_MS > 0U
    if ((Pump_Handle.IfSucked == 0U) &&
        ((uint32_t)(now_ms - g_pump_start_ms) >= PUMP_SUCK_FORCE_DONE_TIMEOUT_MS)) {
        Pump_Handle.IfSucked = 1U;
        Pump_Handle.SuckStrength = (uint32_t)freq_hz;
    }
#endif

#if PUMP_AUTO_STOP_TIMEOUT_MS > 0U
    if ((uint32_t)(now_ms - g_pump_start_ms) >= PUMP_AUTO_STOP_TIMEOUT_MS) {
        CN_Pump_Stop();
    }
#endif
}

// 气泵 - TIM15 溢出处理，由全局 HAL_TIM_PeriodElapsedCallback 转发调用。
void CN_Pump_TIM15_PeriodElapsedCallback(void)
{
    if (Pump_Handle.Pump_on == 0U) {
        return;
    }

    g_pump_tim_overflow_count++;

    if (g_pump_feedback_capture_valid != 0U) {
        uint32_t now_total_ticks = g_pump_tim_overflow_count * PUMP_PWM_PERIOD_COUNT;

        if ((uint32_t)(now_total_ticks - g_pump_last_capture_total_ticks) > PUMP_FEEDBACK_TIMEOUT_TICKS) {
            g_pump_feedback_capture_valid = 0U;
            g_pump_feedback_period_ticks = 0U;
            g_pump_feedback_freq_hz = 0.0f;
            CN_Pump_Freq_Filter_Reset();
            Pump_Handle.FeedbackPeriodTicks = 0U;
            Pump_Handle.FeedbackFreqHz = 0.0f;
        }
    }
}

// 气泵 - PE6 输入捕获处理，使用相邻两个上升沿的时间差记录原始周期。
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM15) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)) {
        if (Pump_Handle.Pump_on == 0U) {
            return;
        }

        uint32_t capture_ticks = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        uint32_t overflow_count = g_pump_tim_overflow_count;

        // HAL 会先处理捕获事件再处理更新事件；如果捕获时更新标志已经置位，说明计数器刚溢出，需要临时补上这一轮溢出。
        if ((__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) && (capture_ticks < (PUMP_PWM_PERIOD_COUNT / 2U))) {
            overflow_count++;
        }

        uint32_t total_ticks = overflow_count * PUMP_PWM_PERIOD_COUNT + capture_ticks;

        if (g_pump_feedback_capture_valid != 0U) {
            uint32_t period_ticks = total_ticks - g_pump_last_capture_total_ticks;

            if (period_ticks != 0U) {
                g_pump_feedback_period_ticks = period_ticks;
                CN_Pump_Freq_Filter_Push(period_ticks);
                Pump_Handle.FeedbackPeriodTicks = period_ticks;
            }
        } else {
            g_pump_feedback_capture_valid = 1U;
        }

        g_pump_last_capture_total_ticks = total_ticks;
        g_pump_last_capture_ms = HAL_GetTick();
    }
}

// 硬件中断自动跳转
void TIM15_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM15_Motor_Handle);  // 通知 HAL 处理 TIM15 更新和输入捕获中断
}

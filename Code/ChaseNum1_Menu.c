#include "ChaseNum1_Include.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// 屏幕刷新周期，单位 ms；状态页不用高速刷新，避免占用主循环时间。
#define MENU_REFRESH_MS                 300U
// 当前 6x8 字体的单个字符宽度，按列刷新数值时用于换算像素坐标。
#define MENU_FONT_WIDTH                 6U
// 当前使用 6x8 字体，横屏 160x128 最多显示 16 行。
#define MENU_LINE_HEIGHT                8U
// 每行最多显示字符数，按 160 像素宽度和 6x8 字体估算后留一点余量。
#define MENU_LINE_CHARS                 26U
// 数值缓存行数，与屏幕可显示行数一致；只有数值变化时才重新写屏幕。
#define MENU_VALUE_CACHE_LINES          16U
// 屏幕底色，黑底比白底更容易看清状态变化。
#define MENU_COLOR_BG                   RGB565_BLACK
// 标题颜色。
#define MENU_COLOR_TITLE                RGB565_CYAN
// 正常信息颜色。
#define MENU_COLOR_TEXT                 RGB565_WHITE
// 重点状态颜色。
#define MENU_COLOR_OK                   RGB565_GREEN
// 错误提示颜色。
#define MENU_COLOR_WARN                 RGB565_YELLOW

static uint8_t g_menu_inited = 0U;
static uint32_t g_menu_last_refresh_ms = 0U;
static uint32_t g_menu_last_loop_count = 0U;
static uint32_t g_menu_last_rx_frame_count = 0U;
static uint32_t g_menu_last_tx_frame_count = 0U;
static uint8_t g_menu_alive_mark = 0U;
static char g_menu_value_cache[MENU_VALUE_CACHE_LINES][MENU_LINE_CHARS];
static uint16_t g_menu_color_cache[MENU_VALUE_CACHE_LINES];

// 系统状态名称，屏幕上直接显示可读单词。
static const char *Menu_System_State_Name(uint8_t state)
{
    switch (state) {
        case 0U: return "Init";
        case 1U: return "Reset";
        case 2U: return "Idle";
        case 3U: return "Active";
        case 4U: return "EStop";
        case 5U: return "Fault";
        default: return "Unknown";
    }
}

// 底盘状态名称，屏幕上直接显示可读单词。
static const char *Menu_Chassis_State_Name(uint8_t state)
{
    switch (state) {
        case 0U: return "Idle";
        case 1U: return "Run";
        case 2U: return "Rotate";
        case 3U: return "Translate";
        case 4U: return "Stop";
        case 5U: return "Brake";
        case 6U: return "Fault";
        default: return "Unknown";
    }
}

// 机械臂状态名称，屏幕上直接显示可读单词。
static const char *Menu_Arm_State_Name(uint8_t state)
{
    switch (state) {
        case 0U: return "Idle";
        case 1U: return "Moving";
        case 2U: return "Folding";
        case 3U: return "Hold";
        case 4U: return "Rejected";
        case 5U: return "Fault";
        default: return "Unknown";
    }
}

// 气泵状态名称，屏幕上直接显示可读单词。
static const char *Menu_Pump_State_Name(uint8_t state)
{
    switch (state) {
        case 0U: return "Idle";
        case 1U: return "Sucking";
        case 2U: return "Sucked";
        case 3U: return "Release";
        case 4U: return "Released";
        case 5U: return "Fault";
        default: return "Unknown";
    }
}

// 按字符列显示文本，用于初始化时写固定标题和标签。
static void Menu_Show_Text(uint8_t line, uint8_t col, uint16_t color, const char *text)
{
    tft180_set_color(color, MENU_COLOR_BG);
    tft180_show_string((uint16_t)(col * MENU_FONT_WIDTH), (uint16_t)(line * MENU_LINE_HEIGHT), text);
}

// 更新指定行的动态数值区；缓存内容和颜色都不变时不写屏幕，减少闪烁。
static void Menu_Update_Value(uint8_t line, uint8_t col, uint16_t color, const char *fmt, ...)
{
    char value[MENU_LINE_CHARS];
    char padded[MENU_LINE_CHARS];
    uint8_t max_chars = 0U;
    uint8_t len = 0U;
    uint8_t i = 0U;
    va_list args;

    if (line >= MENU_VALUE_CACHE_LINES) return;
    if (col >= MENU_LINE_CHARS) return;

    max_chars = (uint8_t)(MENU_LINE_CHARS - col);
    if (max_chars >= MENU_LINE_CHARS) {
        max_chars = (uint8_t)(MENU_LINE_CHARS - 1U);
    }

    memset(value, 0, sizeof(value));
    va_start(args, fmt);
    vsnprintf(value, (size_t)max_chars + 1U, fmt, args);
    va_end(args);

    memset(padded, 0, sizeof(padded));
    len = (uint8_t)strlen(value);
    if (len > max_chars) {
        len = max_chars;
    }

    for (i = 0U; i < max_chars; i++) {
        padded[i] = (i < len) ? value[i] : ' ';
    }
    padded[max_chars] = '\0';

    if ((strcmp(g_menu_value_cache[line], padded) == 0) &&
        (g_menu_color_cache[line] == color)) {
        return;
    }

    strcpy(g_menu_value_cache[line], padded);
    g_menu_color_cache[line] = color;
    Menu_Show_Text(line, col, color, padded);
}

// 屏幕状态页初始化，只显示必要运行状态，不参与任何控制逻辑。
void Menu_init(void)
{
    tft180_set_dir(TFT180_CROSSWISE);
    TS_TFT180_init();
    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_color(MENU_COLOR_TEXT, MENU_COLOR_BG);
    tft180_clear();

    memset(g_menu_value_cache, 0, sizeof(g_menu_value_cache));
    memset(g_menu_color_cache, 0, sizeof(g_menu_color_cache));

    Menu_Show_Text(0U, 0U, MENU_COLOR_TITLE, "ChaseNum1 Status");
    Menu_Show_Text(1U, 0U, MENU_COLOR_TEXT,  "Alive :");
    Menu_Show_Text(3U, 0U, MENU_COLOR_TEXT,  "System :");
    Menu_Show_Text(4U, 0U, MENU_COLOR_TEXT,  "Chassis:");
    Menu_Show_Text(5U, 0U, MENU_COLOR_TEXT,  "Arm    :");
    Menu_Show_Text(6U, 0U, MENU_COLOR_TEXT,  "Pump   :");
    Menu_Show_Text(8U, 0U, MENU_COLOR_TEXT,  "UART2 RX:");
    Menu_Show_Text(9U, 0U, MENU_COLOR_TEXT,  "Frames :");
    Menu_Show_Text(10U, 0U, MENU_COLOR_TEXT, "Buffer :");
    Menu_Show_Text(12U, 0U, MENU_COLOR_TEXT, "Link   :");
    Menu_Show_Text(13U, 0U, MENU_COLOR_TEXT, "Errors :");
    Menu_Show_Text(15U, 0U, MENU_COLOR_TEXT, "Frame  :");

    g_menu_inited = 1U;
    g_menu_last_refresh_ms = 0U;
    g_menu_last_loop_count = 0U;
    g_menu_last_rx_frame_count = 0U;
    g_menu_last_tx_frame_count = 0U;
    g_menu_alive_mark = 0U;
}

// 屏幕状态页轮询刷新，需要在主循环中周期调用。
void Menu_poll(void)
{
    Nano_COM_Diag_t nano_diag;
    CN_UART_Diag_t uart2_diag;
    uint32_t now_ms = 0U;
    uint32_t rx_total_error = 0U;
    uint32_t uart_total_error = 0U;
    uint16_t alive_color = MENU_COLOR_WARN;

    if (g_menu_inited == 0U) return;

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - g_menu_last_refresh_ms) < MENU_REFRESH_MS) return;
    g_menu_last_refresh_ms = now_ms;

    Nano_COM_Get_Diag(&nano_diag);
    CN_UART_Get_Diag(CN_UART_2, &uart2_diag);

    if (nano_diag.task_loop_count != g_menu_last_loop_count) {
        g_menu_alive_mark ^= 1U;
        g_menu_last_loop_count = nano_diag.task_loop_count;
    }

    alive_color = (g_menu_alive_mark != 0U) ? MENU_COLOR_OK : MENU_COLOR_TEXT;
    rx_total_error = nano_diag.rx_drop_count + nano_diag.rx_busy_drop_count +
                     nano_diag.rx_bad_check_count + nano_diag.rx_bad_tail_count +
                     nano_diag.rx_bad_len_count;
    uart_total_error = uart2_diag.uart_error_count + uart2_diag.uart_overrun_count +
                       uart2_diag.uart_frame_error_count + uart2_diag.uart_noise_error_count +
                       uart2_diag.tx_fifo_write_fail_count;

    Menu_Update_Value(1U, 8U, alive_color, "%lu  Mark:%lu",
                      (unsigned long)nano_diag.task_loop_count, (unsigned long)g_menu_alive_mark);

    Menu_Update_Value(3U, 9U, MENU_COLOR_OK, "%s", Menu_System_State_Name(nano_diag.system_state));
    Menu_Update_Value(4U, 9U, MENU_COLOR_TEXT, "%s", Menu_Chassis_State_Name(nano_diag.chassis_state));
    Menu_Update_Value(5U, 9U, MENU_COLOR_TEXT, "%s", Menu_Arm_State_Name(nano_diag.arm_state));
    Menu_Update_Value(6U, 9U, MENU_COLOR_TEXT, "%s", Menu_Pump_State_Name(nano_diag.pump_state));

    Menu_Update_Value(8U, 10U, MENU_COLOR_TEXT, "%lu", (unsigned long)nano_diag.rx_byte_count);
    Menu_Update_Value(9U, 9U, MENU_COLOR_TEXT, "R:%lu T:%lu",
                      (unsigned long)nano_diag.rx_frame_count, (unsigned long)nano_diag.tx_frame_count);
    Menu_Update_Value(10U, 9U, MENU_COLOR_TEXT, "Ring:%lu Ready:%lu",
                      (unsigned long)nano_diag.rx_ring_occupied, (unsigned long)nano_diag.package_ready);

    if (rx_total_error == 0U && uart_total_error == 0U) {
        Menu_Update_Value(12U, 9U, MENU_COLOR_OK, "OK");
        Menu_Update_Value(13U, 9U, MENU_COLOR_TEXT, "None");
    } else {
        Menu_Update_Value(12U, 9U, MENU_COLOR_WARN, "Proto:%lu", (unsigned long)rx_total_error);
        Menu_Update_Value(13U, 9U, MENU_COLOR_WARN, "Uart:%lu", (unsigned long)uart_total_error);
    }

    if (g_menu_last_rx_frame_count != nano_diag.rx_frame_count ||
        g_menu_last_tx_frame_count != nano_diag.tx_frame_count) {
        Menu_Update_Value(15U, 9U, MENU_COLOR_OK, "Updated");
    } else {
        Menu_Update_Value(15U, 9U, MENU_COLOR_TEXT, "No new frame");
    }

    g_menu_last_rx_frame_count = nano_diag.rx_frame_count;
    g_menu_last_tx_frame_count = nano_diag.tx_frame_count;
}

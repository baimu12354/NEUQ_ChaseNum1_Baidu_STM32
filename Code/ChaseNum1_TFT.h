#ifndef _CHASE_NUM1_TFT_H_
#define _CHASE_NUM1_TFT_H_

#include "stm32h7xx_hal.h"
#include "ChaseNum1_Typedef.h"

/*
    屏幕驱动引脚定义：
        SCLK: GPIOI_PIN_1
        MOSI: GPIOI_PIN_3
        RES: GPIOG_PIN_10
        DC: GPIOD_PIN_4
        CS: GPIOD_PIN_2
        BL: GPIOD_PIN_3
*/

extern SPI_HandleTypeDef Hard_SPI_2_Handle;


void TS_TFT180_init(void);
void TS_Hard_SPI_init(void);

#define TFT180_DEFAULT_DISPLAY_DIR      (TFT180_CROSSWISE)                        // 默认的显示方向
#define TFT180_DEFAULT_PENCOLOR         (RGB565_RED)                            // 默认的画笔颜色
#define TFT180_DEFAULT_BGCOLOR          (RGB565_WHITE)                          // 默认的背景颜色
#define TFT180_DEFAULT_DISPLAY_FONT     (TFT180_8X16_FONT)                      // 默认的字体模式

#define TFT180_DC(x)                    ((x) ? (HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET))  : (HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET)))
#define TFT180_RST(x)                   ((x) ? (HAL_GPIO_WritePin(GPIOG,GPIO_PIN_10,GPIO_PIN_SET))  : (HAL_GPIO_WritePin(GPIOG,GPIO_PIN_10,GPIO_PIN_RESET)))
#define TFT180_CS(x)                    ((x) ? (HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET))  : (HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET)))
#define TFT180_BLK(x)                   ((x) ? (HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET))  : (HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET)))
//=================================================定义 TFT180 基本配置================================================


//=================================================定义 TFT180 参数结构体===============================================
typedef enum
{
    TFT180_PORTAIT                      = 0,                                    // 竖屏模式
    TFT180_PORTAIT_180                  = 1,                                    // 竖屏模式  旋转180
    TFT180_CROSSWISE                    = 2,                                    // 横屏模式
    TFT180_CROSSWISE_180                = 3,                                    // 横屏模式  旋转180
}tft180_dir_enum;

typedef enum
{
    TFT180_6X8_FONT                     = 0,                                    // 6x8      字体
    TFT180_8X16_FONT                    = 1,                                    // 8x16     字体
    TFT180_16X16_FONT                   = 2,                                    // 16x16    字体 目前不支持
}tft180_font_size_enum;

extern  uint16_t  tft180_width_max ;
extern  uint16_t  tft180_height_max;
//=================================================定义 TFT180 参数结构体===============================================


//=================================================声明 TFT180 基础函数================================================
void    tft180_clear                    (void);                                                                               // TFT180 清屏函数
void    tft180_full                     (const uint16_t color);                                                                 // TFT180 屏幕填充函数
void    tft180_set_dir                  (tft180_dir_enum dir);                                                                // TFT180 设置显示方向
void    tft180_set_font                 (tft180_font_size_enum font);                                                         // TFT180 设置显示字体
void    tft180_set_color                (const uint16_t pen, const  uint16_t bgcolor);                                            // TFT180 设置显示颜色
void    tft180_draw_point               (uint16_t x, uint16_t y, const uint16_t color);                                             // TFT180 画点函数
void    tft180_draw_line                (uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, const uint16_t color);     // TFT180 画线函数

void    tft180_show_char                (uint16_t x, uint16_t y, const char dat);                                                 // TFT180 显示字符
void    tft180_show_string              (uint16_t x, uint16_t y, const char dat[]);                                               // TFT180 显示字符串
void    tft180_show_int                 (uint16_t x,uint16_t y, const int32_t dat, uint8_t num);                                      // TFT180 显示32位有符号 (去除整数部分无效的0)
void    tft180_show_uint                (uint16_t x,uint16_t y, const uint32_t dat, uint8_t num);                                     // TFT180 显示32位无符号 (去除整数部分无效的0)
void    tft180_show_float               (uint16_t x, uint16_t y, const double dat, uint8_t num, uint8_t pointnum);

void    tft180_show_binary_image        (uint16_t x, uint16_t y, const uint8_t *image, uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height);                    // TFT180 显示二值图像 数据每八个点组成一个字节数据
void    tft180_show_gray_image          (uint16_t x, uint16_t y, const uint8_t *image, uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height, uint8_t threshold);   // TFT180 显示 8bit 灰度图像 带二值化阈值
void    tft180_show_rgb565_image        (uint16_t x, uint16_t y, const uint16_t *image, uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height, uint8_t color_mode); // TFT180 显示 RGB565 彩色图像

void    tft180_show_wave                (uint16_t x, uint16_t y, const uint16_t *wave, uint16_t width, uint16_t value_max, uint16_t dis_width, uint16_t dis_value_max);              // TFT180 显示波形
void    tft180_show_chinese             (uint16_t x, uint16_t y, uint8_t size, const uint8_t *chinese_buffer, uint8_t number, const uint16_t color);                               // TFT180 汉字显示
                                                                                                                              // 1.8寸TFT屏幕初始化
void    tft180_init                     (void);
//=================================================声明 TFT180 基础函数================================================


#endif // !_CHASE_NUM1_TFT_H_
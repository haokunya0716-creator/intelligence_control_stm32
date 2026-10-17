#ifndef __OLED_HARDWARE_I2C_H
#define __OLED_HARDWARE_I2C_H

#include "main.h" // 包含 STM32 HAL 库头文件

#define OLED_CMD  0	// 写命令
#define OLED_DATA 1	// 写数据

// OLED 默认的 I2C 设备地址 (0x3C 左移一位 = 0x78)
#define OLED_I2C_ADDRESS 0x78 

// 配置像素分辨率，不是屏幕对角线的英寸数。当前支持 128x64 和 128x32。
#define OLED_WIDTH  128U
#define OLED_HEIGHT 64U
#define OLED_PAGE_COUNT (OLED_HEIGHT / 8U)

// 列地址偏移：标准 SSD1306 通常为 0；SH1106 模块常见为 2。
// 当前设为 2，用于验证屏幕边缘亮线是否由列映射不匹配引起。
#define OLED_COLUMN_OFFSET 2U

#if OLED_COLUMN_OFFSET != 0U && OLED_COLUMN_OFFSET != 2U
#error "OLED_COLUMN_OFFSET must be 0 (SSD1306) or 2 (common SH1106 modules)"
#endif

#if OLED_WIDTH != 128U || (OLED_HEIGHT != 64U && OLED_HEIGHT != 32U)
#error "Supported SSD1306 resolutions: 128x64 and 128x32"
#endif

// 与 Core/Src/i2c.c 中的 I2C1 引脚配置保持一致：PB6=SCL，PB7=SDA。
#define OLED_I2C_GPIO_PORT GPIOB
#define OLED_I2C_SCL_PIN GPIO_PIN_6
#define OLED_I2C_SDA_PIN GPIO_PIN_7

// ================= OLED 控制用函数 =================
void delay_ms(uint32_t ms);
void OLED_ColorTurn(uint8_t i);
void OLED_DisplayTurn(uint8_t i);
void OLED_WR_Byte(uint8_t dat, uint8_t cmd);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey);
uint32_t oled_pow(uint8_t m, uint8_t n);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t sizey);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t sizey);
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t no, uint8_t sizey);
void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[]);
void OLED_Init(void);
void oled_i2c_sda_unlock(void);

#endif /* #ifndef __OLED_HARDWARE_I2C_H */

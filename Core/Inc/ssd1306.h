/* ============================================================
 * ssd1306.h —— SSD1306 OLED 接口（I2C2：PB10=SCL / PB11=SDA）
 *
 * 用法：
 *   ssd1306_init();                 // 上电初始化
 *   ssd1306_clear();                // 清显存
 *   ssd1306_draw_string(0,0,"xxx"); // 画字符串（坐标 x,y 为像素）
 *   ssd1306_refresh();              // 整屏上屏
 * ============================================================ */
#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>

void ssd1306_init(void);
void ssd1306_clear(void);
void ssd1306_draw_string(uint8_t x, uint8_t y, const char* str);
void ssd1306_refresh(void);

#endif

/* ============================================================
 * button.h —— PC13 按键接口（EXTI 下降沿 + 200ms 消抖）
 *
 * 用法：主循环里
 *   if (button_get_flag()) { button_clear_flag(); ... }  // 切模式
 * ============================================================ */
#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>

void    button_init(void);
uint8_t button_get_flag(void);   /* 是否有待处理的按键 */
void    button_clear_flag(void); /* 清除按键标志 */

#endif

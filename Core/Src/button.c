/* ============================================================
 * button.c —— PC13 板上 User Key 按键（EXTI 下降沿中断）
 *
 * 流程：按下按键（PC13 接 GND）→ EXTI15_10 中断 → 本文件里的
 *       HAL_GPIO_EXTI_Callback 置一个标志位 → 主循环轮询标志位
 *       切换手感模式。用标志位而不是直接在中断里处理，是让
 *       中断尽量短，不干扰 1kHz 控制环。
 * ============================================================ */
#include "button.h"
#include "main.h"        /* KEY_BUTTON_Pin / KEY_BUTTON_GPIO_Port（CubeMX 生成在 main.h） */

static volatile uint32_t last_press = 0;  /* 上次按键时刻，用于软件消抖 */
static volatile uint8_t  mode_flag  = 0;  /* "按过键"标志，主循环读取 */

/* 初始化：标志清零 */
void button_init(void)
{
    last_press = 0;
    mode_flag = 0;
}

/* ---------------------------------------------------------------
 * 按键中断回调（强定义，覆盖 stm32h7xx_it.c / HAL 的 __weak 版本）
 * 200ms 消抖：机械按键按下瞬间会抖动，快速连续抖动会被过滤掉，
 * 只在真正稳定的按击（间隔 >200ms）时置标志。
 * 注意：在中断上下文里执行，函数要短，只置标志不干活。
 * --------------------------------------------------------------- */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_BUTTON_Pin)
    {
        uint32_t now = HAL_GetTick();
        if (now - last_press > 200)   /* 200ms 消抖窗口 */
        {
            last_press = now;
            mode_flag = 1;
        }
    }
}

/* 查询是否有待处理的按键（主循环调用） */
uint8_t button_get_flag(void) { return mode_flag; }

/* 清除按键标志（主循环处理完调用） */
void    button_clear_flag(void) { mode_flag = 0; }

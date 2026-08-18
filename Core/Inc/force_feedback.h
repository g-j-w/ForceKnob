/* ============================================================
 * force_feedback.h —— 力反馈（手感）接口
 *
 * 模式编号（main.c 的 g_mode）：
 *   0 = DETENT  段落感    1 = DAMPING 阻尼    2 = SPRING 弹簧
 * ============================================================ */
#ifndef FORCE_FEEDBACK_H
#define FORCE_FEEDBACK_H

void  force_feedback_init(void);
void  force_feedback_set_zero(float angle);   /* SPRING 模式记录回中零点 */
float force_feedback_compute(float angle, float velocity, int mode);
      /* 输入机械角[rad]、角速度[rad/s]、模式；返回限幅后的 Vq[V] */
const char* force_feedback_get_mode_name(int mode);   /* 模式名（OLED/串口用） */

#endif

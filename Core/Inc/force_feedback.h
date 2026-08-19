/* ============================================================
 * force_feedback.h —— 手感接口
 *
 * 模式编号（main.c 的 g_mode）：
 *   0 = SPIN   自转（开路旋转，main.c 实现）
 *   1 = DETENT 步进（段落感）
 *   2 = DAMPING 阻尼
 * ============================================================ */
#ifndef FORCE_FEEDBACK_H
#define FORCE_FEEDBACK_H

void  force_feedback_init(void);
float force_feedback_compute(float angle, float velocity, int mode);
      /* 输入机械角[rad]、角速度[rad/s]、模式(1或2)；返回限幅后的 Vq[V] */
const char* force_feedback_get_mode_name(int mode);   /* 模式名（串口显示用） */

#endif

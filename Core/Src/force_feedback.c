/* ============================================================
 * force_feedback.c —— 手感算法
 *
 * 三档模式（0=自转 1=步进 2=阻尼）：
 *   0. SPIN   自转：开路旋转（main.c 中断里实现）
 *   1. DETENT 步进：比例弹簧 + 死区（借鉴 smartKnob 实现）
 *   2. DAMPING 阻尼：速度比例阻尼
 *
 * 为什么段落用"比例弹簧 + 死区"而不是 sin 力：
 *   （依据 smartKnob 开源力反馈旋钮 + espp bldc_haptics 实现）
 *   1. sin 力在档位中心附近斜率最陡、最"硬"，电机被来回弹 → 振荡/震动
 *   2. 比例弹簧线性、稳定；档位中心加 ±死区，让电机在档位上安静休息
 *   3. 速度用重滤波（encoder.c α=0.02），阻尼平滑、不把噪声注入进力矩
 * ============================================================ */
#include "force_feedback.h"
#include "foc_math.h"     /* 需要 _2PI / clamp() */
#include <math.h>

/* ============ 手感参数 ============ */
#define NUM_DETENTS    12                               /* 段落数：一圈 12 段 */
#define DETENT_STEP    (_2PI / NUM_DETENTS)             /* 每档角度 [rad] */
#define K_DETENT_P     14.0f                            /* 段落比例刚度 [V/rad]（越大越难拧） */
#define DEAD_ZONE      (2.0f * _PI / 180.0f)            /* 档位中心死区 ±2°（关键！防振荡） */
#define B_DETENT       0.05f                            /* 段落基础阻尼 [V/(rad/s)] */
#define B_DETENT_CENTER 0.25f                           /* 靠近档位中心额外阻尼（压残余振荡） */
/* 边界 ENDSTOP —— 最简单鲁棒的反馈：转到 ±END_MAX_ANGLE 硬停被推回。
 * 中间完全自由（只留轻阻尼），只有边界才出力，不容易受齿槽/振荡影响。 */
#define END_MAX_ANGLE  (_PI)                            /* 边界 ±180°（转半圈到顶） */
#define K_END         12.0f                             /* 边界推力 [V/rad]（挡得死） */
#define B_END         0.10f                             /* 阻尼 [V/(rad/s)] */
#define VQ_LIMIT       4.5f                             /* Vq 限幅（安全上限，满力会发热） */
#define VEL_CUTOFF     60.0f                            /* 速度超过此值不出力，防失控 */

static const char* mode_names[3] = {"SPIN", "DETENT", "ENDSTOP"};

/* 初始化 */
void force_feedback_init(void)
{
}

/* ---------------------------------------------------------------
 * 力反馈主函数：输入角度/速度/模式(1或2)，输出限幅后的 Vq 电压
 * 调用频率：1kHz（在 TIM2 中断里；模式 0=SPIN 由 main.c 单独处理）
 * --------------------------------------------------------------- */
float force_feedback_compute(float angle, float velocity, int mode)
{
    /* 防失控：电机转太快就撤力（避免正反馈），smartKnob 同样做法 */
    if (fabsf(velocity) > VEL_CUTOFF) return 0.0f;

    float vq = 0.0f;

    switch (mode)
    {
        case 1: /* ---- DETENT 步进：比例弹簧 + 死区 ----
                 * 1. a2d = 距最近档位的角度（±step/2）
                 * 2. 死区：|a2d| < DEAD_ZONE 内输入为 0 → 档位上不较劲、不振荡
                 * 3. 线性弹簧 Vq = Kp·input，越过档位中点时 a2d 跳变 → "咔哒"
                 * 4. 靠近档位中心时阻尼加强（espp bldc_haptics 的
                 *    derivative 增益渐变思路），专门压住中心残余振荡 */
        {
            float a2d = angle - roundf(angle / DETENT_STEP) * DETENT_STEP;
            float dz  = clamp(a2d, -DEAD_ZONE, DEAD_ZONE);   /* 死区内的角度 */
            float input = -(a2d - dz);                        /* 中心平坦，两边线性 */

            float proximity = 1.0f - fabsf(a2d) / (DETENT_STEP * 0.5f);  /* 1=中心 0=边缘 */
            float b_eff = B_DETENT + B_DETENT_CENTER * clamp(proximity, 0.0f, 1.0f);

            vq = K_DETENT_P * input - b_eff * velocity;
        }
        break;

        case 2: /* ---- ENDSTOP 边界挡块 ----
                 * 转到 ±END_MAX_ANGLE 被硬挡（强推回），中间自由。
                 * 只有边界才出力，不受段落振荡/齿槽影响。 */
        {
            if (angle < -END_MAX_ANGLE)
                vq =  K_END * (-END_MAX_ANGLE - angle) - B_END * velocity;
            else if (angle > END_MAX_ANGLE)
                vq = -K_END * (angle - END_MAX_ANGLE) - B_END * velocity;
            else
                vq = -B_END * velocity;   /* 中间：轻阻尼 */
        }
        break;

        default:
            vq = 0.0f;
            break;
    }

    return clamp(vq, -VQ_LIMIT, VQ_LIMIT);
}

/* 模式名（串口显示用） */
const char* force_feedback_get_mode_name(int mode)
{
    if (mode >= 0 && mode < 3) return mode_names[mode];
    return "UNKNOWN";
}

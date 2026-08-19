/* ============================================================
 * force_feedback.c —— 手感算法
 *
 * 三档模式（0=自转 1=步进 2=阻尼）：
 *
 *   0. SPIN   自转：电机持续匀速旋转（开路旋转，在 main.c 中断里实现）
 *   1. DETENT 步进：正弦势阱，一圈 12 格咔哒
 *   2. DAMPING 阻尼：像在油里转，越转越黏
 *
 * 本文件负责 1/2 档（依赖编码器角度的反馈模式）：
 *   1. DETENT  力矩 = -K_DETENT·sin(12·θ) - B_DAMPING·ω
 *   2. DAMPING 力矩 = -B_DAMPING·ω
 * 0 档 SPIN 是开路旋转，不依赖编码器，直接在 main.c 的 ISR 里实现。
 * ============================================================ */
#include "force_feedback.h"
#include "foc_math.h"     /* 需要 clamp() */
#include <math.h>

/* ============ 手感参数（按需调整，一次只动一个） ============
 * ⚠️ 实际出力上限由 VQ_LIMIT 决定：K/TORQUE_GAIN 再大也会被它卡住。 */
#define NUM_DETENTS  12          /* 段落数：一圈 12 段 */
#define K_DETENT     0.4f        /* 段落刚度：越大越难拧过去，越小越不容易振 */
#define B_DAMPING    0.20f       /* 阻尼系数：越大越粘（0.20 可压住段落振荡） */
#define TORQUE_GAIN  6.0f        /* 力矩 → Vq 电压的整体放大倍数 */
#define VQ_LIMIT     3.5f        /* Vq 限幅 ±3.5V（安全上限！2804 相阻 2.3Ω，3.5V≈1.5A 峰值） */

static const char* mode_names[3] = {"SPIN", "DETENT", "DAMPING"};

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
    float torque = 0.0f;

    switch (mode)
    {
        case 1: /* ---- DETENT 步进（段落感）----
                 * 力矩 = -K·sin(12·θ) - B·ω
                 * 12·θ 让一圈出现 12 个稳定点，每个都是"咔哒"一下。 */
            torque = -K_DETENT * sinf(NUM_DETENTS * angle)
                     - B_DAMPING * velocity;
            break;

        case 2: /* ---- DAMPING 纯阻尼 ----
                 * 力矩 = -B·ω：速度越快阻力越大，静止时无力。 */
            torque = -B_DAMPING * velocity;
            break;

        default:
            torque = 0.0f;
            break;
    }

    /* 力矩 → Vq 电压，并做安全限幅 */
    float vq = torque * TORQUE_GAIN;
    return clamp(vq, -VQ_LIMIT, VQ_LIMIT);
}

/* 模式名（串口显示用） */
const char* force_feedback_get_mode_name(int mode)
{
    if (mode >= 0 && mode < 3) return mode_names[mode];
    return "UNKNOWN";
}

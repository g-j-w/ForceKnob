/* ============================================================
 * force_feedback.c —— 力反馈（手感）算法
 *
 * 三种模式，最后都换算成一个"目标力矩"→ 再乘增益变成 Vq 电压：
 *
 *   0. DETENT  段落感：正弦势阱，像旋钮一格一格咔哒
 *   1. DAMPING 阻尼：像在油里转，越转越黏
 *   2. SPRING  弹簧：转到哪松手弹回按按键的位置
 *
 * 每个模式都叠加一层阻尼项 -B_DAMPING·ω，用来压住共振/回弹。
 * 所有参数都集中在文件顶部，按教程第 8 节边试边调（一次只动一个）。
 * ============================================================ */
#include "force_feedback.h"
#include "foc_math.h"     /* 需要 clamp() */
#include <math.h>

/* ============ 手感参数（按教程第 8 节调整） ============
 * ⚠️ 注意：K_DETENT/TORQUE_GAIN 算出的 Vq 会远超 VQ_LIMIT，
 *    所以实际出力被 VQ_LIMIT 卡住——想加力，改 VQ_LIMIT 才有效！
 *    （VQ_LIMIT = 4.5V 对应电流 ≈ 4.5/2.3 ≈ 2A 峰值，DRV8313 允许，
 *     但长时间满力保持会发烫，注意散热）
 */
#define NUM_DETENTS  12          /* 段落数：一圈 12 段（2804 是 7 对极，机械一圈对应 12 个电气周期齿槽感） */
#define K_DETENT     0.5f        /* 段落刚度：越小越不容易振（0.5 可减少段落振荡） */
#define K_SPRING     0.5f        /* 弹簧刚度：越大回中越快/越硬 */
#define B_DAMPING    0.20f       /* 阻尼系数：越大越粘（0.20 压制段落共振） */
#define TORQUE_GAIN  6.0f        /* 力矩 → Vq 电压的整体放大倍数 */
#define VQ_LIMIT     3.5f        /* Vq 限幅 ±3.5V（安全上限！2804 相阻 2.3Ω，3.5V≈1.5A 峰值） */

static const char* mode_names[3] = {"DETENT", "DAMPING", "SPRING"};
static float spring_zero = 0.0f;  /* SPRING 模式的回中零点 [rad] */

/* 初始化：弹簧零点清零 */
void force_feedback_init(void)
{
    spring_zero = 0.0f;
}

/* 设置弹簧回中零点（切到 SPRING 模式时，把当前角度记下来） */
void force_feedback_set_zero(float angle)
{
    spring_zero = angle;
}

/* ---------------------------------------------------------------
 * 力反馈主函数：输入角度/速度/模式，输出限幅后的 Vq 电压
 * 调用频率：1kHz（在 TIM2 中断里）
 * --------------------------------------------------------------- */
float force_feedback_compute(float angle, float velocity, int mode)
{
    float torque = 0.0f;

    switch (mode)
    {
        case 0: /* ---- DETENT 段落感 ----
                 * 力矩 = -K·sin(12·θ) - B·ω
                 * sin 每 2π 一个齿槽，12·θ 让一圈出现 12 个稳定点，
                 * 每个稳定点都是"咔哒"一下。负号让力矩总是指向最近的槽。 */
            torque = -K_DETENT * sinf(NUM_DETENTS * angle)
                     - B_DAMPING * velocity;
            break;

        case 1: /* ---- DAMPING 纯阻尼 ----
                 * 力矩 = -B·ω：速度越快阻力越大，静止时无力。 */
            torque = -B_DAMPING * velocity;
            break;

        case 2: /* ---- SPRING 弹簧 ----
                 * 力矩 = -K·(θ-θ0) - B·ω：偏离零点越远拉力越大。 */
            torque = -K_SPRING * (angle - spring_zero)
                     - B_DAMPING * velocity;
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

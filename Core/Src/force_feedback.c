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
/* 回中 SPRING —— 照搬 smartKnob motor_task.cpp「自动回中」+ mad2804.h 参数
 *  - SPRING_P = endstop_strength(0.6) × 4 = 2.4  （smartKnob 自动回中比例增益）
 *  - SPRING_D = 0.148                            （mad2804.h FOC_PID_D，阻尼）
 *  - 死区 1° = smartKnob DEAD_ZONE_RAD           （中心内不出力，稳定） */
#define SPRING_P       2.4f                            /* 回中比例增益 [V/rad]（照搬 smartKnob） */
#define SPRING_D       0.148f                          /* 回中阻尼 [V/(rad/s)]（照搬 mad2804.h） */
#define SPRING_DEAD_ZONE (1.0f * _PI / 180.0f)        /* 回中死区 ±1°（照搬 smartKnob） */
#define VQ_LIMIT       4.5f                            /* Vq 限幅：留足力翻过电机齿槽（smartKnob 低齿槽电机用 3V，我们可能需更高） */
#define VEL_CUTOFF     60.0f                            /* 速度超过此值不出力，防失控 */

static const char* mode_names[3] = {"SPIN", "DETENT", "SPRING"};

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

        case 2: /* ---- SPRING 回中弹簧（照搬 smartKnob）----
                 * 固定回中到 0°（上电对齐位置）。
                 * input = -(距中心角度) + 死区；vq = P·input - D·速度
                 * 死区内输入为 0（中心平坦），死区外线性弹簧。 */
        {
            float dead_zone_adjustment = clamp(angle, -SPRING_DEAD_ZONE, SPRING_DEAD_ZONE);
            float input = -(angle - dead_zone_adjustment);
            vq = SPRING_P * input - SPRING_D * velocity;
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

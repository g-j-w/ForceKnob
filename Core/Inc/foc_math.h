/* ============================================================
 * foc_math.h —— FOC 数学工具（纯头文件，全 inline）
 *
 * 控制环里不需要电流采样，所以是"电压模式 FOC"：
 *   力矩(Vq) → 反 Park → 反 Clarke → 中心对齐 → 占空比
 *
 * 变换解释（对照三相无刷电机）：
 *   - 反 Park：把转子坐标系里给的 Vd/Vq，逆时针转到静止 αβ 坐标系
 *   - 反 Clarke：把静止 αβ 两个正交量，展开成三相 U/V/W 的相电压
 *   - 中心对齐：给三相同时加一个零序电压，充分利用母线电压
 *   - 占空比：相电压 / 母线电压 + 0.5，就是 PWM 的占空比
 *
 * 本文件还集中定义了：
 *   - 角度常数 _PI/_2PI
 *   - 极对数 POLE_PAIRS（DFRobot 2804 = 7 对极）
 *   - 通用工具 clamp()、normalize_angle()
 * ============================================================ */
#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <math.h>       /* sinf/cosf/fmaxf/fminf */

/* ---------- 常用常数 ---------- */
#define _PI   3.14159265358979f
#define _2PI  6.28318530717959f

/* ---------- 极对数 ----------
 * ⚠️ DFRobot 2804（SKU FIT1034）官方规格 = 7 对极。
 *    换别的电机后要重测：把这里改成 1，DAMPING 模式匀速转一整圈，
 *    数"卡顿"的次数就是极对数，再填回来。 */
#define POLE_PAIRS  7

/* ---------- 限幅：把 x 限制在 [min, max] ---------- */
static inline float clamp(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

/* ---------- 角度归一化到 -PI ~ +PI ----------
 * 有些算法里角度会越界（比如弹簧力算 2 圈后的差值），
 * 先归一化避免 sin/cos 的大角度精度问题和比较歧义。 */
static inline float normalize_angle(float a)
{
    while (a >  _PI) a -= _2PI;
    while (a < -_PI) a += _2PI;
    return a;
}

/* ---------- Park 变换：静止 Ia/Ib → 转子 Id/Iq ----------
 * 电流模式才用得到；本工程是电压模式，这个函数保留备用。 */
static inline void park_transform(float ia, float ib, float theta,
                                  float* id, float* iq)
{
    float c = cosf(theta);
    float s = sinf(theta);
    *id =  ia * c + ib * s;
    *iq = -ia * s + ib * c;
}

/* ---------- 反 Park 变换：转子 Vd/Vq → 静止 Vα/Vβ ----------
 * theta = 电气角（机械角 × 极对数）。
 * 力矩模式下 Vd=0，实际只有 Vq 在起作用。
 * |Vα|   | cosθ  -sinθ | |Vd|
 * |Vβ| = | sinθ   cosθ | |Vq| */
static inline void inv_park_transform(float vd, float vq, float theta,
                                      float* valpha, float* vbeta)
{
    float c = cosf(theta);
    float s = sinf(theta);
    *valpha = vd * c - vq * s;
    *vbeta  = vd * s + vq * c;
}

/* ---------- 反 Clarke 变换：静止 Vα/Vβ → 三相相电压 Va/Vb/Vc ----------
 * 三相相差 120°：
 *   Va = Vα
 *   Vb = -0.5·Vα + (√3/2)·Vβ
 *   Vc = -0.5·Vα - (√3/2)·Vβ        （√3/2 ≈ 0.866025404） */
static inline void inv_clarke(float valpha, float vbeta,
                              float* va, float* vb, float* vc)
{
    *va = valpha;
    *vb = -0.5f * valpha + 0.866025404f * vbeta;
    *vc = -0.5f * valpha - 0.866025404f * vbeta;
}

/* ---------- 中心对齐（零序注入）+ 限幅 ----------
 * 三相相电压最大只能到 ±母线/2，否则 PWM 会过调制削顶。
 * 做法：算出三相最大/最小，给三相同时加 -(max+min)/2 的零序电压，
 * 让波形整体向下压，等效于把母线电压利用率提高 ~15%。 */
static inline void center_align(float* va, float* vb, float* vc, float vdc)
{
    float max = fmaxf(*va, fmaxf(*vb, *vc));
    float min = fminf(*va, fminf(*vb, *vc));
    float offset = -0.5f * (max + min);

    *va += offset;
    *vb += offset;
    *vc += offset;

    float limit = vdc * 0.5f;
    *va = clamp(*va, -limit, limit);
    *vb = clamp(*vb, -limit, limit);
    *vc = clamp(*vc, -limit, limit);
}

/* ---------- 相电压 → 占空比 (0~1) ----------
 * 三相半桥：占空比 d = v/VDC + 0.5
 *   v = +VDC/2 → d = 1.0（上管常通）
 *   v = 0      → d = 0.5（上下各半）
 *   v = -VDC/2 → d = 0.0（下管常通）
 * 占空比再乘 ARR 就是 CCR 写入值。 */
static inline void voltage_to_duty(float va, float vb, float vc, float vdc,
                                   float* da, float* db, float* dc)
{
    *da = va / vdc + 0.5f;
    *db = vb / vdc + 0.5f;
    *dc = vc / vdc + 0.5f;
}

#endif

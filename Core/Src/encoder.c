/* ============================================================
 * encoder.c —— AS5600 磁编码器驱动（I2C1：PB8=SCL / PB9=SDA）
 *
 * 功能：
 *   1. 读 AS5600 的 12 位原始角度（0~4095，一圈）
 *   2. 跨圈跟踪：角度越过 0 点时自动加减圈数，得到"多圈总角度"
 *   3. 速度：1ms 差分 + EMA 低通滤波，去掉编码器噪声毛刺
 *   4. 上电对齐后调用 encoder_reset_zero() 把当前位置记为 0
 *
 * 注意：本文件在 1kHz 控制环中断里被调用，函数要尽量快。
 *       I2C1 读一次约 550µs（100kHz）或 140µs（400kHz），
 *       I2C1 总线只挂编码器，不要在上面接别的器件抢总线。
 * ============================================================ */
#include "encoder.h"
#include "foc_math.h"     /* 需要 _2PI */
#include "i2c.h"          /* 需要 hi2c1 */
#include <stdint.h>
#include <math.h>

#define AS5600_ADDR         0x36      /* AS5600 7 位地址 */
#define AS5600_RAW_ANGLE_H  0x0C      /* 角度高字节寄存器 */
#define AS5600_RAW_ANGLE_L  0x0D      /* 角度低字节寄存器 */
#define ENCODER_CPR         4096.0f   /* 12 位分辨率 = 4096 份/圈 */
#define DT                  0.001f    /* 控制周期 1ms（和 TIM2 中断对齐！） */

static uint16_t raw_value = 0;      /* 最近一次读到的原始角度 [0,4095] */
static uint16_t last_raw  = 0;      /* 上一帧原始角度，用于算增量 */
static uint8_t  first     = 1;      /* 第一帧标志：只采样不算速度 */
static int32_t  multi_turn  = 0;    /* 累计圈数（跨 0 点用） */
static float    total_angle = 0.0f; /* 多圈总机械角 [rad] */
static float    velocity    = 0.0f; /* 角速度 [rad/s]（已滤波） */

/* ---------------------------------------------------------------
 * 读一次 AS5600 原始角度（阻塞，约几百 µs）
 * 失败（总线上没器件/线没接好）就沿用上次的值，不让角度跳变。
 * --------------------------------------------------------------- */
static void read_raw(void)
{
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR << 1, AS5600_RAW_ANGLE_H,
                         I2C_MEMADD_SIZE_8BIT, buf, 2, 2) == HAL_OK)
    {
        /* 高 4 位是状态位，只取低 12 位 */
        raw_value = (uint16_t)((((uint16_t)buf[0] << 8) | buf[1]) & 0x0FFF);
    }
}

/* ---------------------------------------------------------------
 * 编码器初始化：所有状态清零
 * --------------------------------------------------------------- */
void encoder_init(void)
{
    first = 1;
    last_raw = 0;
    multi_turn = 0;
    total_angle = 0.0f;
    velocity = 0.0f;
}

/* ---------------------------------------------------------------
 * 编码器更新：每 1ms 调用一次（TIM2 中断里）
 *
 * 单圈角度 → 多圈角度：
 *   把当前 raw 减上一帧 raw 的差，看有没有"跳半圈"来判断是否
 *   跨过 0 点（4095→0）。差值超过 ±2048（半圈）就是跨圈了。
 *   multi_turn 记录圈数，总角度 = 圈数×2π + 当前单圈角。
 *
 * 速度：
 *   直接用 (本次角-上次角)/1ms 差分 → 会把 ±1 LSB 噪声放大成尖峰。
 *   所以套一层一阶低通 EMA：v_new = 0.7·v_old + 0.3·v_raw
 * --------------------------------------------------------------- */
void encoder_update(void)
{
    read_raw();

    if (first)   /* 第一帧：只采样，不算速度（避免上电巨大速度尖峰） */
    {
        first = 0;
        last_raw = raw_value;
        total_angle = (float)raw_value / ENCODER_CPR * _2PI;
        velocity = 0.0f;
        return;
    }

    /* 跨圈检测：用 int16_t 差值，-2048~+2047 内是正常转动 */
    int16_t diff = (int16_t)raw_value - (int16_t)last_raw;
    if (diff > 2048)       multi_turn--;   /* 反向跨过 0 点，圈数减一 */
    else if (diff < -2048) multi_turn++;   /* 正向跨过 0 点，圈数加一 */
    last_raw = raw_value;

    float single_turn = (float)raw_value / ENCODER_CPR * _2PI;   /* 单圈角 [rad] */
    float new_total   = (float)multi_turn * _2PI + single_turn;  /* 多圈角 [rad] */

    float raw_vel = (new_total - total_angle) / DT;   /* 原始速度 [rad/s] */
    velocity = 0.7f * velocity + 0.3f * raw_vel;      /* EMA 低通滤波 */

    total_angle = new_total;
}

/* ---------------------------------------------------------------
 * 电机对齐完成后调用：把当前位置记为机械角 0
 * （必须在电机转到位、还在通电时立即调用，然后才松开 PWM）
 * --------------------------------------------------------------- */
void encoder_reset_zero(void)
{
    read_raw();
    last_raw = raw_value;
    multi_turn = 0;
    total_angle = 0.0f;
    velocity = 0.0f;
    first = 0;
}

/* 读总机械角 [rad]（volatile：中断写、主循环读，用 getter 拿） */
float encoder_get_angle(void)    { return total_angle; }

/* 读角速度 [rad/s]（已滤波） */
float encoder_get_velocity(void) { return velocity; }

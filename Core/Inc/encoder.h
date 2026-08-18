/* ============================================================
 * encoder.h —— AS5600 磁编码器接口（I2C1：PB8=SCL / PB9=SDA）
 *
 * 上电时序：先 motor_align()（main.c），转子吸到电气零度后
 *           调 encoder_reset_zero()，把当前位置记为机械角 0。
 * ============================================================ */
#ifndef ENCODER_H
#define ENCODER_H

void  encoder_init(void);
void  encoder_update(void);          /* 1kHz 中断里调用（每 1ms 一次） */
void  encoder_reset_zero(void);      /* 电机对齐后清零机械角 */
float encoder_get_angle(void);       /* 总机械角度 [rad]（多圈） */
float encoder_get_velocity(void);    /* 角速度 [rad/s]（已 EMA 滤波） */

#endif

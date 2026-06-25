#ifndef __MECANUM_H__
#define __MECANUM_H__

#include "motor.h"

/* ============================================================
 * 麦伦小车几何参数（单位：mm）
 * ============================================================ */

/* 麦伦轮直径 */
#define WHEEL_DIAMETER   60.0f
#define WHEEL_RADIUS     30.0f

/* 前后轮距 */
#define WHEEL_BASE       120.0f

/* 左右轮距 */
#define TRACK_WIDTH      196.0f

/* 半轴距 + 半轮距，用于旋转分量计算 */
#define HALF_BASE        (WHEEL_BASE / 2.0f)    /* Ly = 60mm */
#define HALF_TRACK       (TRACK_WIDTH / 2.0f)   /* Lx = 98mm */
#define MOTION_L         (HALF_BASE + HALF_TRACK) /* L = 158mm */

/* ============================================================
 * 麦伦轮 X字布置映射
 * ============================================================
 *  电机A → 右前轮 (RF)
 *  电机B → 右后轮 (RR)
 *  电机C → 左后轮 (LR)
 *  电机D → 左前轮 (LF)
 *
 *  坐标系：+X 右, +Y 前 (小车前向为Y轴), +ω 逆时针
 *
 *  X字布置逆运动学公式（V_linear = ω_wheel * R）：
 *    V_A = Vy - Vx - ω·L    (RF)
 *    V_B = Vy + Vx - ω·L    (RR)
 *    V_C = Vy - Vx + ω·L    (LR)
 *    V_D = Vy + Vx + ω·L    (LF)
 *  其中 L = Lx + Ly = 158mm, R = 30mm
 * ============================================================ */

/* ============================================================
 * API 函数声明
 * ============================================================ */

/**
 * @brief  麦伦小车运动控制
 * @param  Vx  横向速度（mm/s），右为正
 * @param  Vy  纵向速度（mm/s），前为正
 * @param  w   角速度（rad/s），逆时针为正
 * @note   调用 motor_X_pid() 控制四路电机转速，
 *         需在 TIM1 中断回调中周期调用（~50Hz）
 */
void Motion_Control(float Vx, float Vy, float w);

#endif /* __MECANUM_H__ */

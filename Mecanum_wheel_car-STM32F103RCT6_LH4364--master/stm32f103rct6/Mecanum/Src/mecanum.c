#include "mecanum.h"

#define PI 3.14159265f

/* ============================================================
 * 麦伦小车运动控制
 * ============================================================ */

/**
 * @brief  麦伦小车运动控制（X字布置逆运动学）
 * @param  Vx  横向速度（mm/s），右为正
 * @param  Vy  纵向速度（mm/s），前为正
 * @param  w   角速度（rad/s），逆时针为正
 * @note   将期望底盘运动 (Vx, Vy, w) 解算为四轮转速（RPM），
 *         通过 motor_X_pid() 输出到电机。
 *
 *         X字布置逆运动学：
 *           V_A = Vy - Vx - w * L    (RF, 右前)
 *           V_B = Vy + Vx - w * L    (RR, 右后)
 *           V_C = Vy - Vx + w * L    (LR, 左后)
 *           V_D = Vy + Vx + w * L    (LF, 左前)
 *
 *         转速转换：
 *           ω_wheel = V_linear / R   (rad/s)
 *           RPM = ω_wheel * 60 / (2π) = V_linear * 30 / (π * R)
 *
 *         代入 R = 30mm：
 *           RPM = V_linear / π
 */
void Motion_Control(float Vx, float Vy, float w)
{
    float v_A, v_B, v_C, v_D;
    float rpm_A, rpm_B, rpm_C, rpm_D;

    /* 逆运动学：底盘速度 → 四轮线速度 (mm/s) */
    v_A = Vy - Vx - w * MOTION_L;
    v_B = Vy + Vx - w * MOTION_L;
    v_C = Vy - Vx + w * MOTION_L;
    v_D = Vy + Vx + w * MOTION_L;

    /* 线速度 → RPM：RPM = V_linear * 60 / (2π * R) = V_linear / π (R=30) */
    rpm_A = v_A / PI;
    rpm_B = v_B / PI;
    rpm_C = v_C / PI;
    rpm_D = v_D / PI;

    /* 驱动四路电机 PID 调速 */
    motor_A_pid(rpm_A);
    motor_B_pid(rpm_B);
    motor_C_pid(rpm_C);
    motor_D_pid(rpm_D);
}

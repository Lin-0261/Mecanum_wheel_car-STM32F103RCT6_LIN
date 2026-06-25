#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include "tim.h"

/* ============================================================
 * 电机驱动参数宏定义
 * ============================================================ */

/* PWM最大占空比，对应 htim3.Init.Period = 1000 */
#define MOTOR_PWM_MAX 1000

/* 电机每转一圈产生的编码器脉冲数（含x2倍频后的总数） */
#define ENCODER_PULSES_PER_REV  780

/* 速度采样周期（ms），与TIM1更新中断周期一致（~10ms, 100Hz） */
#define SPEED_SAMPLE_TIME_MS 10

/* ============================================================
 * 编码器引脚与定时器映射关系
 * ============================================================
 *  电机A → E1A(PA15), E1B(PB3) → TIM2 编码器模式
 *  电机B → E2A(PB6),  E2B(PB7) → TIM4 编码器模式
 *  电机C → E3A(PC6),  E3B(PC7) → TIM8 编码器模式
 *  电机D → E4A(PA1),  E4B(PA2) → EXTI1/EXTI2 软件编码器（上升沿触发，2倍频）
 *  PWM   → TIM3 CH1~CH4 → 电机A~D
 *  采样定时器 → TIM1 更新中断，周期约20ms
 * ============================================================ */

/* ============================================================
 * 全局变量声明
 * ============================================================ */

/* 各电机实时转速（单位：RPM，转/分钟），由TIM1中断每20ms更新一次 */
extern int16_t motorA_speed_rpm;
extern int16_t motorB_speed_rpm;
extern int16_t motorC_speed_rpm;
extern int16_t motorD_speed_rpm;

/* 各电机最近一个采样周期内的编码器计数增量（原始值，用于调试） */
extern int16_t motorA_encoder_delta;
extern int16_t motorB_encoder_delta;
extern int16_t motorC_encoder_delta;
extern int16_t motorD_encoder_delta;

/* ============================================================
 * 函数声明
 * ============================================================ */

/**
 * @brief  电机模块初始化
 * @note   启动TIM1中断模式，使能20ms周期采样
 *         需在main()中MX_TIM1_Init()之后调用
 */
void motor_init(void);

/**
 * @brief  速度更新函数
 * @note   由TIM1更新中断回调 HAL_TIM_PeriodElapsedCallback() 调用
 *         读取四个编码器计数值，计算增量并转换为转速（RPM）
 *           RPM = delta * 60000 / (15000 * 20ms) = delta / 5
 */
void motor_speed_update(void);

/**
 * @brief  电机A PWM控制
 * @param  pwm_value  正值正转，负值反转，0停止，范围[-1000, 1000]
 */
void motorA_pwm(int pwm_value);

/**
 * @brief  电机B PWM控制
 * @param  pwm_value  正值正转，负值反转，0停止，范围[-1000, 1000]
 */
void motorB_pwm(int pwm_value);

/**
 * @brief  电机C PWM控制
 * @param  pwm_value  正值正转，负值反转，0停止，范围[-1000, 1000]
 */
void motorC_pwm(int pwm_value);

/**
 * @brief  电机D PWM控制
 * @param  pwm_value  正值正转，负值反转，0停止，范围[-1000, 1000]
 */
void motorD_pwm(int pwm_value);

/* ============================================================
 * PID 控制参数宏定义（四路电机共用）
 * ============================================================ */
#define PID_KP       3.0f  /* 比例系数：100RPM误差→1000PWM满输出 */
#define PID_KI       2.0f   /* 积分系数：快速累积消除静差 */
#define PID_KD       0.0f   /* 微分系数（暂关闭，避免抑制响应） */
#define PID_INTEGRAL_MAX  200.0f   /* 积分限幅（Ki*200=1000，覆盖全PWM范围） */
#define PID_OUTPUT_MAX    1000.0f  /* 输出限幅，匹配MOTOR_PWM_MAX */

/* ============================================================
 * PID 控制函数声明
 * ============================================================ */

void motor_A_pid(float target_rpm);
void motor_B_pid(float target_rpm);
void motor_C_pid(float target_rpm);
void motor_D_pid(float target_rpm);

#endif /* __MOTOR_H__ */

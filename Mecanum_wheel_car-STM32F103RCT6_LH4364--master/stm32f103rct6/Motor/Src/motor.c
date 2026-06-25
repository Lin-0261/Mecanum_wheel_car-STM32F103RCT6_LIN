#include "motor.h"

/* ============================================================
 * 全局变量定义
 * ============================================================ */

/* 各电机实时转速（RPM），正值正转，负值反转 */
int16_t motorA_speed_rpm = 0;
int16_t motorB_speed_rpm = 0;
int16_t motorC_speed_rpm = 0;
int16_t motorD_speed_rpm = 0;

/* 各电机当前采样周期的编码器计数增量（调试用） */
int16_t motorA_encoder_delta = 0;
int16_t motorB_encoder_delta = 0;
int16_t motorC_encoder_delta = 0;
int16_t motorD_encoder_delta = 0;

/* ============================================================
 * 静态变量
 * ============================================================ */

/* PWM启动标志，确保四个通道只启动一次 */
static uint8_t motor_pwm_started = 0;

/* 电机D软件编码器脉冲计数（EXTI中断累加，motor_speed_update中读取并清零） */
static volatile int16_t motorD_encoder_count = 0;

/* ============================================================
 * 初始化函数
 * ============================================================ */

/**
 * @brief  电机模块初始化
 * @note   启动TIM1基本定时器中断模式，使能约10ms周期的速度采样中断
 *         TIM1配置：PSC=7199, ARR=99, APB2=72MHz
 *         中断频率 = 72MHz / 7200 / 100 = 100Hz = 10ms/次
 */
void motor_init(void)
{
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
}

/* ============================================================
 * EXTI中断回调（电机D软件编码器）
 * ============================================================ */

/**
 * @brief  EXTI中断回调，处理电机D编码器脉冲计数
 * @note   E4A(PA1)→EXTI1, E4B(PA2)→EXTI2, 均为上升沿触发
 *         通过检测另一相信号电平判断旋转方向：
 *         - A上升沿时B为低 → 正转计数+
 *         - A上升沿时B为高 → 反转计数-
 *         - B上升沿时A为高 → 正转计数+
 *         - B上升沿时A为低 → 反转计数-
 *         每个编码器周期产生2个计数（2倍频），与ENCODER_PULSES_PER_REV匹配
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == E4A_Pin) {
        if (HAL_GPIO_ReadPin(E4B_GPIO_Port, E4B_Pin) == GPIO_PIN_RESET)
            motorD_encoder_count--;
        else
            motorD_encoder_count++;
    } else if (GPIO_Pin == E4B_Pin) {
        if (HAL_GPIO_ReadPin(E4A_GPIO_Port, E4A_Pin) == GPIO_PIN_SET)
            motorD_encoder_count--;
        else
            motorD_encoder_count++;
    }
}

/* ============================================================
 * 速度计算函数
 * ============================================================ */

/**
 * @brief  速度更新函数
 * @note   在TIM1更新中断回调中调用（约20ms一次）
 *         使用 __HAL_TIM_GET_COUNTER 读取编码器当前计数值，
 *         读完后立即归零，防止16位计数器溢出回绕。
 *         由于每次读后归零，当前读数即为此周期的增量，
 *         利用 int16_t 转换自动识别正反转方向。
 *         电机每转一圈产生 ENCODER_PULSES_PER_REV(15000) 个脉冲：
 *           RPM = delta * 60000 / (15000 * 20ms) = delta / 5
 */
void motor_speed_update(void)
{
    int16_t delta;
    int32_t rpm;

    /* ---- 电机A：TIM2 编码器 (E1A/PA15, E1B/PB3) ---- */
    delta = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    motorA_encoder_delta = delta;
    rpm = (int32_t)delta * 60000 / (ENCODER_PULSES_PER_REV * SPEED_SAMPLE_TIME_MS);
    motorA_speed_rpm = -1*(int16_t)rpm;

    /* ---- 电机B：TIM4 编码器 (E2A/PB6, E2B/PB7) ---- */
    delta = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    motorB_encoder_delta = delta;
    rpm = (int32_t)delta * 60000 / (ENCODER_PULSES_PER_REV * SPEED_SAMPLE_TIME_MS);
    motorB_speed_rpm = -1*(int16_t)rpm;

    /* ---- 电机C：TIM8 编码器 (E3A/PC6, E3B/PC7) ---- */
    delta = (int16_t)__HAL_TIM_GET_COUNTER(&htim8);
    __HAL_TIM_SET_COUNTER(&htim8, 0);
    motorC_encoder_delta = delta;
    rpm = (int32_t)delta * 60000 / (ENCODER_PULSES_PER_REV * SPEED_SAMPLE_TIME_MS);
    motorC_speed_rpm = (int16_t)rpm;

    /* ---- 电机D：EXTI1/EXTI2 软件编码器 (E4A/PA1, E4B/PA2) ---- */
    delta = motorD_encoder_count;
    motorD_encoder_count = 0;
    motorD_encoder_delta = delta;
    rpm = (int32_t)delta * 60000 / (ENCODER_PULSES_PER_REV * SPEED_SAMPLE_TIME_MS);
    motorD_speed_rpm = (int16_t)rpm;
}

/* ============================================================
 * PWM辅助函数
 * ============================================================ */

/**
 * @brief  首次调用时启动TIM3全部四个PWM通道
 */
static void motor_pwm_start(void)
{
    if (motor_pwm_started)
        return;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    motor_pwm_started = 1;
}

/**
 * @brief  PWM值限幅，取绝对值并截断到[0, MOTOR_PWM_MAX]
 */
static uint16_t clamp_pwm(int value)
{
    if (value < 0) value = -value;
    if (value > MOTOR_PWM_MAX) value = MOTOR_PWM_MAX;
    return (uint16_t)value;
}

/* ============================================================
 * 电机PWM控制函数
 * ============================================================ */

/**
 * @brief  电机A控制
 * @note   IN1=PA11, IN2=PA12, PWM=PA6(TIM3_CH1)
 *         pwm>0: IN1高/IN2低 → 正转
 *         pwm<0: IN1低/IN2高 → 反转
 *         pwm=0: 两路均低 → 停止（刹车）
 */
void motorA_pwm(int pwm_value)
{
    motor_pwm_start();
    if (pwm_value > 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    } else if (pwm_value < 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, clamp_pwm(pwm_value));
}

/**
 * @brief  电机B控制
 * @note   IN1=PC10, IN2=PC11, PWM=PA7(TIM3_CH2)
 */
void motorB_pwm(int pwm_value)
{
    motor_pwm_start();
    if (pwm_value > 0) {
        // HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    } else if (pwm_value < 0) {
        // HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        // HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, clamp_pwm(pwm_value));
}

/**
 * @brief  电机C控制
 * @note   IN1=PC12, IN2=PD2, PWM=PB0(TIM3_CH3)
 */
void motorC_pwm(int pwm_value)
{
    motor_pwm_start();
    if (pwm_value > 0) {
        // HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_SET);
    } else if (pwm_value < 0) {
        // HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_RESET);
        // HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, clamp_pwm(pwm_value));
}

/**
 * @brief  电机D控制
 * @note   IN1=PC8, IN2=PC9, PWM=PB1(TIM3_CH4)
 */
void motorD_pwm(int pwm_value)
{
    motor_pwm_start();
    if (pwm_value > 0) {
        HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_RESET);
    } else if (pwm_value < 0) {
        HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, clamp_pwm(pwm_value));
}

/* ============================================================
 * PID 控制模块
 * ============================================================ */

/* 每个电机独立的PID状态 */
typedef struct {
    float integral;     /* 积分累加器 */
    float prev_error;   /* 上一次误差，用于微分计算 */
} pid_state_t;

static pid_state_t pidA = {0};
static pid_state_t pidB = {0};
static pid_state_t pidC = {0};
static pid_state_t pidD = {0};

/**
 * @brief  通用PID计算
 * @param  pid     当前电机的PID状态指针
 * @param  target  目标转速（RPM）
 * @param  current 当前实测转速（RPM）
 * @return PWM输出值 [-PID_OUTPUT_MAX, PID_OUTPUT_MAX]
 * @note   dt = SPEED_SAMPLE_TIME_MS / 1000 = 20ms = 0.02s
 *         应在与 motor_speed_update() 相同的20ms周期内调用
 */
static float pid_compute(pid_state_t *pid, float target, float current)
{
    const float dt = SPEED_SAMPLE_TIME_MS / 1000.0f;
    float error = target - current;
    float derivative;

    /* 积分累加并限幅，防止积分饱和 */
    pid->integral += error * dt;
    if (pid->integral > PID_INTEGRAL_MAX)
        pid->integral = PID_INTEGRAL_MAX;
    else if (pid->integral < -PID_INTEGRAL_MAX)
        pid->integral = -PID_INTEGRAL_MAX;

    /* 微分项 */
    derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    /* PID输出 = Kp*e + Ki*∫e + Kd*de/dt */
    float output = PID_KP * error
                 + PID_KI * pid->integral
                 + PID_KD * derivative;

    /* 输出限幅 */
    if (output > PID_OUTPUT_MAX)
        output = PID_OUTPUT_MAX;
    else if (output < -PID_OUTPUT_MAX)
        output = -PID_OUTPUT_MAX;

    return output;
}

/**
 * @brief  电机A PID转速控制
 * @param  target_rpm  目标转速（RPM），正值正转，负值反转
 * @note   读取 motorA_speed_rpm 作为反馈，PID运算后调用 motorA_pwm() 输出
 */
void motor_A_pid(float target_rpm)
{
    if (target_rpm == 0.0f) {
        pidA.integral = 0.0f;
        pidA.prev_error = 0.0f;
        motorA_pwm(0);
        return;
    }
    float output = pid_compute(&pidA, target_rpm, (float)motorA_speed_rpm);
    motorA_pwm((int)output);
}

/**
 * @brief  电机B PID转速控制
 */
void motor_B_pid(float target_rpm)
{
    if (target_rpm == 0.0f) {
        pidB.integral = 0.0f;
        pidB.prev_error = 0.0f;
        motorB_pwm(0);
        return;
    }
    float output = pid_compute(&pidB, target_rpm, (float)motorB_speed_rpm);
    motorB_pwm((int)output);
}

/**
 * @brief  电机C PID转速控制
 */
void motor_C_pid(float target_rpm)
{
    if (target_rpm == 0.0f) {
        pidC.integral = 0.0f;
        pidC.prev_error = 0.0f;
        motorC_pwm(0);
        return;
    }
    float output = pid_compute(&pidC, target_rpm, (float)motorC_speed_rpm);
    motorC_pwm((int)output);
}

/**
 * @brief  电机D PID转速控制
 */
void motor_D_pid(float target_rpm)
{
    if (target_rpm == 0.0f) {
        pidD.integral = 0.0f;
        pidD.prev_error = 0.0f;
        motorD_pwm(0);
        return;
    }
    float output = pid_compute(&pidD, target_rpm, (float)motorD_speed_rpm);
    motorD_pwm((int)output);
}

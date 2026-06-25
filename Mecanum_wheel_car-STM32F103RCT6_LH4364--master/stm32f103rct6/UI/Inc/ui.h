#ifndef __UI_H__
#define __UI_H__

#include "main.h"

/* 按键引脚定义 — 硬件上拉，按下为低电平 */
#define BTN_UP_PORT      GPIOC
#define BTN_UP_PIN       GPIO_PIN_4
#define BTN_RIGHT_PORT   GPIOC
#define BTN_RIGHT_PIN    GPIO_PIN_5
#define BTN_DOWN_PORT    GPIOA
#define BTN_DOWN_PIN     GPIO_PIN_5
#define BTN_LEFT_PORT    GPIOA
#define BTN_LEFT_PIN     GPIO_PIN_4
#define BTN_CENTER_PORT  GPIOB
#define BTN_CENTER_PIN   GPIO_PIN_2

/* 运动参数 */
typedef struct {
    float Vx;          /* 横向速度 mm/s，右为正 */
    float Vy;          /* 纵向速度 mm/s，前为正 */
    float w;           /* 角速度 rad/s，逆时针为正 */
    uint32_t time_ms;  /* 运动持续时间 ms */
} UI_Params;

extern UI_Params ui_params;
extern uint8_t ui_motion_active;

void UI_Init(void);
void UI_Update(void);

#endif /* __UI_H__ */

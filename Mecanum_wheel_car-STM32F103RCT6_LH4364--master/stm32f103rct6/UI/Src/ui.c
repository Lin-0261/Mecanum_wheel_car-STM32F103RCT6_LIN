#include "ui.h"
#include "oled.h"
#include "mecanum.h"
#include "motor.h"
#include <stdio.h>

/* ============================================================
 * 常量定义
 * ============================================================ */

#define DEBOUNCE_FRAMES  2    /* 消抖帧数，50ms/帧 → 100ms消抖 */
#define HOLD_REPEAT_START 15  /* 长按开始连发帧数 (~750ms) */
#define HOLD_REPEAT_RATE  3   /* 连发间隔帧数 (~150ms) */

/* 参数范围与步进 */
#define VX_MIN    -1000.0f
#define VX_MAX     1000.0f
#define VX_STEP    50.0f
#define VX_HOLD_STEP 50.0f
#define VY_MIN    -1000.0f
#define VY_MAX     1000.0f
#define VY_STEP    50.0f
#define VY_HOLD_STEP 50.0f
#define W_MIN      -5.0f
#define W_MAX       5.0f
#define W_STEP      0.1f
#define W_HOLD_STEP 0.1f
#define TIME_MIN   100U
#define TIME_MAX   10000U
#define TIME_STEP  500U
#define TIME_HOLD_STEP 100U

/* OLED 行坐标 (afont12x6, 12px高 + 1px间距) */
#define LINE0_Y    0
#define LINE1_Y    13
#define LINE2_Y    26
#define LINE3_Y    39
#define LINE4_Y    52

/* ============================================================
 * 类型定义
 * ============================================================ */

typedef enum {
    UI_STATE_NAVIGATE,
    UI_STATE_EDIT,
    UI_STATE_RUNNING
} UI_State;

typedef enum {
    MENU_VX = 0,
    MENU_VY,
    MENU_W,
    MENU_TIME,
    MENU_RUN,
    MENU_COUNT
} MenuItem;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t raw_prev;     /* 上一帧原始电平: 1=高/释放, 0=低/按下 */
    uint16_t held_frames; /* 持续按下的帧数 */
    uint8_t pressed;      /* 本帧检测到下降沿 */
} Button;

/* ============================================================
 * 全局变量
 * ============================================================ */

UI_Params ui_params;
uint8_t ui_motion_active;

/* ============================================================
 * 静态变量
 * ============================================================ */

static UI_State  ui_state;
static MenuItem  ui_cursor;
static uint32_t  motion_start_tick;
static uint32_t  motion_elapsed;
static Button    btns[5];
static uint8_t   frame_parity;  /* 用于EDIT闪烁 */

/* ============================================================
 * 按键驱动
 * ============================================================ */

static void Btn_Init(void)
{
    btns[0] = (Button){BTN_UP_PORT,    BTN_UP_PIN,    1, 0, 0};
    btns[1] = (Button){BTN_RIGHT_PORT, BTN_RIGHT_PIN, 1, 0, 0};
    btns[2] = (Button){BTN_DOWN_PORT,  BTN_DOWN_PIN,  1, 0, 0};
    btns[3] = (Button){BTN_LEFT_PORT,  BTN_LEFT_PIN,  1, 0, 0};
    btns[4] = (Button){BTN_CENTER_PORT,BTN_CENTER_PIN,1, 0, 0};
}

static void Btn_Scan(void)
{
    for (int i = 0; i < 5; i++) {
        uint8_t raw = (HAL_GPIO_ReadPin(btns[i].port, btns[i].pin) == GPIO_PIN_SET) ? 1 : 0;

        btns[i].pressed = 0;

        if (raw == 0) {
            if (btns[i].held_frames < 0xFFFF) btns[i].held_frames++;
            if (btns[i].held_frames == DEBOUNCE_FRAMES) {
                btns[i].pressed = 1;
            }
        } else {
            btns[i].held_frames = 0;
        }
    }
}

/* 按键边沿检测宏 */
#define BTN_UP      (btns[0].pressed)
#define BTN_RIGHT   (btns[1].pressed)
#define BTN_DOWN    (btns[2].pressed)
#define BTN_LEFT    (btns[3].pressed)
#define BTN_CENTER  (btns[4].pressed)

/* 长按连发: 第一帧或超过延迟后每RATE帧触发一次 */
#define BTN_RIGHT_REPEAT  (btns[1].held_frames == DEBOUNCE_FRAMES \
                           || (btns[1].held_frames >= HOLD_REPEAT_START \
                               && (btns[1].held_frames - HOLD_REPEAT_START) % HOLD_REPEAT_RATE == 0))
#define BTN_LEFT_REPEAT   (btns[3].held_frames == DEBOUNCE_FRAMES \
                           || (btns[3].held_frames >= HOLD_REPEAT_START \
                               && (btns[3].held_frames - HOLD_REPEAT_START) % HOLD_REPEAT_RATE == 0))
#define BTN_UP_REPEAT     (btns[0].held_frames == DEBOUNCE_FRAMES \
                           || (btns[0].held_frames >= HOLD_REPEAT_START \
                               && (btns[0].held_frames - HOLD_REPEAT_START) % HOLD_REPEAT_RATE == 0))
#define BTN_DOWN_REPEAT   (btns[2].held_frames == DEBOUNCE_FRAMES \
                           || (btns[2].held_frames >= HOLD_REPEAT_START \
                               && (btns[2].held_frames - HOLD_REPEAT_START) % HOLD_REPEAT_RATE == 0))

/* 任意按键按下（用于中止运动） */
#define BTN_ANY  (BTN_UP || BTN_DOWN || BTN_LEFT || BTN_RIGHT || BTN_CENTER)

/* ============================================================
 * 参数调整辅助
 * ============================================================ */

static float Clamp(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static void AdjustValue(float *val, float step, float hold_step,
                        float lo, float hi, int dir)
{
    float s = (BTN_RIGHT_REPEAT || BTN_LEFT_REPEAT)
              ? hold_step : step;
    *val = Clamp(*val + dir * s, lo, hi);
}

static void AdjustUint32(uint32_t *val, uint32_t step, uint32_t hold_step,
                         uint32_t lo, uint32_t hi, int dir)
{
    uint32_t s = (BTN_RIGHT_REPEAT || BTN_LEFT_REPEAT)
                 ? hold_step : step;
    int32_t v = (int32_t)*val + dir * (int32_t)s;
    if (v < (int32_t)lo) v = lo;
    if (v > (int32_t)hi) v = hi;
    *val = (uint32_t)v;
}

/* ============================================================
 * 运动启停
 * ============================================================ */

static void Motion_Start(void)
{
    motion_start_tick = HAL_GetTick();
    motion_elapsed = 0;
    ui_motion_active = 1;
    ui_state = UI_STATE_RUNNING;
}

static void Motion_Stop(void)
{
    ui_motion_active = 0;
    motorA_pwm(0);
    motorB_pwm(0);
    motorC_pwm(0);
    motorD_pwm(0);
    ui_state = UI_STATE_NAVIGATE;
}

/* ============================================================
 * 界面绘制
 * ============================================================ */

static void DrawTitle(void)
{
    OLED_DrawFilledRectangle(0, 0, 128, 12, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(20, 0, "Mecanum Car", &afont12x6, OLED_COLOR_REVERSED);
}

static void DrawParamLine(uint8_t y, MenuItem item,
                          const char *label, const char *value_str,
                          const char *unit)
{
    char buf[22];
    uint8_t selected = (ui_cursor == item);
    uint8_t editing  = (ui_state == UI_STATE_EDIT && selected);
    uint8_t blink_on  = (frame_parity < 20);  /* 约1Hz闪烁 */

    /* 光标符 */
    char cursor = selected ? '>' : ' ';
    snprintf(buf, sizeof(buf), "%c%s", cursor, label);
    OLED_PrintASCIIString(0, y, buf, &afont12x6, OLED_COLOR_NORMAL);

    /* 值域起始x坐标 */
    uint8_t vx = strlen(buf) * 6 + 2;

    if (editing && blink_on) {
        /* 编辑闪烁: 在值下方画矩形高亮，文字反色 */
        uint8_t bw = strlen(value_str) * 6;
        OLED_DrawFilledRectangle(vx, y, bw + 4, 12, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(vx + 2, y, (char *)value_str, &afont12x6, OLED_COLOR_REVERSED);
    } else {
        OLED_PrintASCIIString(vx, y, (char *)value_str, &afont12x6, OLED_COLOR_NORMAL);
    }

    /* 单位 */
    vx += strlen(value_str) * 6 + 4;
    OLED_PrintASCIIString(vx, y, (char *)unit, &afont12x6, OLED_COLOR_NORMAL);
}

static void DrawTimeLine(uint8_t y)
{
    char buf[22];
    uint8_t time_sel = (ui_cursor == MENU_TIME);
    uint8_t run_sel  = (ui_cursor == MENU_RUN);
    uint8_t any_sel  = time_sel || run_sel;

    char cursor = any_sel ? '>' : ' ';
    snprintf(buf, sizeof(buf), "%cT :", cursor);
    OLED_PrintASCIIString(0, y, buf, &afont12x6, OLED_COLOR_NORMAL);

    uint8_t vx = strlen(buf) * 6 + 2;

    /* 时间值: 整数格式化避免 %f 依赖 newlib-nano 浮点支持 */
    uint32_t ms = ui_params.time_ms;
    snprintf(buf, sizeof(buf), "%lu.%02lus",
             (unsigned long)(ms / 1000), (unsigned long)((ms % 1000) / 10));

    uint8_t editing = (ui_state == UI_STATE_EDIT && time_sel);
    uint8_t blink_on = (frame_parity < 20);

    if (editing && blink_on) {
        uint8_t bw = strlen(buf) * 6;
        OLED_DrawFilledRectangle(vx, y, bw + 4, 12, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(vx + 2, y, buf, &afont12x6, OLED_COLOR_REVERSED);
    } else {
        OLED_PrintASCIIString(vx, y, buf, &afont12x6, OLED_COLOR_NORMAL);
    }

    vx += strlen(buf) * 6 + 6;

    /* RUN按钮: 仅当RUN选中时绘制外框 */
    if (run_sel) {
        OLED_DrawRectangle(vx - 2, y - 1, 34, 14, OLED_COLOR_NORMAL);
    }
    OLED_PrintASCIIString(vx, y, "RUN", &afont12x6, OLED_COLOR_NORMAL);
}

static void DrawNavigateMenu(void)
{
    char vbuf[16];

    DrawTitle();

    snprintf(vbuf, sizeof(vbuf), "%+04d", (int)ui_params.Vx);
    DrawParamLine(LINE1_Y, MENU_VX, "Vx:", vbuf, "mm/s");

    snprintf(vbuf, sizeof(vbuf), "%+04d", (int)ui_params.Vy);
    DrawParamLine(LINE2_Y, MENU_VY, "Vy:", vbuf, "mm/s");

    /* w 值格式化: 转整数避免 %f */
    {
        int wi = (int)(ui_params.w * 100.0f);
        int wa = (wi < 0) ? -wi : wi;
        snprintf(vbuf, sizeof(vbuf), "%c%d.%02d",
                 (wi < 0) ? '-' : '+', wa / 100, wa % 100);
    }
    DrawParamLine(LINE3_Y, MENU_W, "w :", vbuf, "rad/s");

    DrawTimeLine(LINE4_Y);
}

static void DrawRunningScreen(void)
{
    motion_elapsed = HAL_GetTick() - motion_start_tick;
    uint32_t remain = (motion_elapsed < ui_params.time_ms)
                      ? (ui_params.time_ms - motion_elapsed) : 0;

    char buf[32];

    OLED_DrawFilledRectangle(0, 0, 128, 12, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(32, 0, "RUNNING", &afont12x6, OLED_COLOR_REVERSED);

    snprintf(buf, sizeof(buf), "Vx:%+04d Vy:%+04d", (int)ui_params.Vx, (int)ui_params.Vy);
    OLED_PrintASCIIString(0, LINE1_Y, buf, &afont12x6, OLED_COLOR_NORMAL);

    {
        int wi = (int)(ui_params.w * 100.0f);
        int wa = (wi < 0) ? -wi : wi;
        snprintf(buf, sizeof(buf), "w:%c%d.%02d T:%lu.%02lus",
                 (wi < 0) ? '-' : '+', wa / 100, wa % 100,
                 (unsigned long)(ui_params.time_ms / 1000),
                 (unsigned long)((ui_params.time_ms % 1000) / 10));
    }
    OLED_PrintASCIIString(0, LINE2_Y, buf, &afont12x6, OLED_COLOR_NORMAL);

    uint32_t remain_sec  = remain / 1000;
    uint32_t remain_cs   = (remain % 1000) / 10;
    snprintf(buf, sizeof(buf), " Remain: %lu.%02lus",
             (unsigned long)remain_sec, (unsigned long)remain_cs);
    OLED_PrintASCIIString(0, LINE3_Y, buf, &afont12x6, OLED_COLOR_NORMAL);

    /* 进度条 */
    uint8_t bar_y = 44;
    uint8_t bar_w = 120;
    uint8_t bar_h = 6;
    uint8_t bar_x = 4;
    OLED_DrawRectangle(bar_x, bar_y, bar_w, bar_h, OLED_COLOR_NORMAL);
    if (ui_params.time_ms > 0) {
        uint8_t fill = (uint8_t)((uint32_t)bar_w * motion_elapsed / ui_params.time_ms);
        if (fill > bar_w) fill = bar_w;
        if (fill > 0) {
            OLED_DrawFilledRectangle(bar_x + 1, bar_y + 1, fill - 1, bar_h - 2, OLED_COLOR_NORMAL);
        }
    }

    OLED_PrintASCIIString(24, LINE4_Y, "Press to STOP", &afont12x6, OLED_COLOR_NORMAL);
}

/* ============================================================
 * 状态机处理
 * ============================================================ */

static void HandleNavigate(void)
{
    if (BTN_UP) {
        if (ui_cursor > 0) ui_cursor--;
    }
    if (BTN_DOWN) {
        if (ui_cursor < MENU_RUN) ui_cursor++;
    }
    if (BTN_CENTER) {
        if (ui_cursor == MENU_RUN) {
            Motion_Start();
        } else if (ui_cursor == MENU_TIME) {
            /* TIME和RUN在同一行，CENTER进入TIME编辑 */
            ui_state = UI_STATE_EDIT;
        } else {
            ui_state = UI_STATE_EDIT;
        }
    }
}

static void HandleEdit(void)
{
    /* CENTER确认，回到导航 */
    if (BTN_CENTER) {
        ui_state = UI_STATE_NAVIGATE;
        return;
    }

    /* LEFT/RIGHT 调整值 */
    switch (ui_cursor) {
    case MENU_VX:
        if (BTN_LEFT_REPEAT)
            AdjustValue(&ui_params.Vx, VX_STEP, VX_HOLD_STEP, VX_MIN, VX_MAX, -1);
        if (BTN_RIGHT_REPEAT)
            AdjustValue(&ui_params.Vx, VX_STEP, VX_HOLD_STEP, VX_MIN, VX_MAX, +1);
        break;
    case MENU_VY:
        if (BTN_LEFT_REPEAT)
            AdjustValue(&ui_params.Vy, VY_STEP, VY_HOLD_STEP, VY_MIN, VY_MAX, -1);
        if (BTN_RIGHT_REPEAT)
            AdjustValue(&ui_params.Vy, VY_STEP, VY_HOLD_STEP, VY_MIN, VY_MAX, +1);
        break;
    case MENU_W:
        if (BTN_LEFT_REPEAT)
            AdjustValue(&ui_params.w, W_STEP, W_HOLD_STEP, W_MIN, W_MAX, -1);
        if (BTN_RIGHT_REPEAT)
            AdjustValue(&ui_params.w, W_STEP, W_HOLD_STEP, W_MIN, W_MAX, +1);
        break;
    case MENU_TIME:
        if (BTN_LEFT_REPEAT)
            AdjustUint32(&ui_params.time_ms, TIME_STEP, TIME_HOLD_STEP, TIME_MIN, TIME_MAX, -1);
        if (BTN_RIGHT_REPEAT)
            AdjustUint32(&ui_params.time_ms, TIME_STEP, TIME_HOLD_STEP, TIME_MIN, TIME_MAX, +1);
        break;
    default:
        break;
    }

    /* UP/DOWN 切换编辑项 */
    if (BTN_UP && ui_cursor > MENU_VX) ui_cursor--;
    if (BTN_DOWN && ui_cursor < MENU_TIME) ui_cursor++;
}

static void HandleRunning(void)
{
    motion_elapsed = HAL_GetTick() - motion_start_tick;

    /* 超时停止 */
    if (motion_elapsed >= ui_params.time_ms) {
        Motion_Stop();
        return;
    }

    /* 任意按键中止 */
    if (BTN_ANY) {
        Motion_Stop();
        return;
    }
}

/* ============================================================
 * 公开API
 * ============================================================ */

void UI_Init(void)
{
    ui_params.Vx = 100.0f;
    ui_params.Vy = 100.0f;
    ui_params.w = 0.0f;
    ui_params.time_ms = 2000;

    ui_motion_active = 0;
    ui_state = UI_STATE_NAVIGATE;
    ui_cursor = MENU_VX;
    motion_start_tick = 0;
    motion_elapsed = 0;
    frame_parity = 0;

    Btn_Init();
}

void UI_Update(void)
{
    Btn_Scan();
    frame_parity = (frame_parity + 1) % 40;  /* 40帧=2秒循环 */

    OLED_NewFrame();

    switch (ui_state) {
    case UI_STATE_NAVIGATE:
        HandleNavigate();
        DrawNavigateMenu();
        break;
    case UI_STATE_EDIT:
        HandleEdit();
        DrawNavigateMenu();
        break;
    case UI_STATE_RUNNING:
        HandleRunning();
        DrawRunningScreen();
        break;
    }

    OLED_ShowFrame();
}

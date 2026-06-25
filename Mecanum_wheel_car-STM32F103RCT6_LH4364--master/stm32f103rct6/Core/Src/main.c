/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "oled.h"
#include <stdio.h>
#include <string.h>
#include "mecanum.h"
#include "ui.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---- UART3 帧接收状态机 ---- */
/* 帧格式: 0x55 0x54 Vx_H Vx_L Vy_H Vy_L w_H w_L checksum (9字节) */
/* Vx, Vy: int16 (mm/s), w: int16 (milli-rad/s = rad/s * 1000) */
/* checksum: XOR of bytes 2~7 (6个数据字节) */
/* 注: 头2字节 + 数据6字节 + 校验1字节 = 共9字节 */

#define UART_FRAME_HEADER1  0x55
#define UART_FRAME_HEADER2  0x54
#define UART_FRAME_DATA_LEN  6   /* 数据字节数 (不含校验) */
#define UART_FRAME_BUF_LEN    7   /* 缓冲区大小 (数据+校验) */

typedef enum {
    UART_STATE_WAIT_H1 = 0,
    UART_STATE_WAIT_H2,
    UART_STATE_RECV_DATA
} uart_rx_state_t;

static uart_rx_state_t uart_state = UART_STATE_WAIT_H1;
static uint8_t uart_data_buf[UART_FRAME_BUF_LEN];  /* 6数据 + 1校验 */
static uint8_t uart_data_idx = 0;
static uint8_t uart_rx_byte = 0;  /* 单字节接收缓冲 */
static uint32_t uart_frame_count = 0;  /* 有效帧计数 */
static uint32_t uart_byte_count = 0;   /* 接收字节总数 (调试用) */
static uint32_t last_frame_tick = 0;   /* 上次收到有效帧的 systick */
#define UART_FRAME_TIMEOUT_MS  500     /* 超时阈值: 超过则清空状态机并归零速度 */

/**
  * @brief  USART3 接收完成回调 (每收到1字节触发)
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;

    uint8_t b = uart_rx_byte;
    uart_byte_count++;  /* 计数每个到达的字节 */

    switch (uart_state) {
    case UART_STATE_WAIT_H1:
        if (b == UART_FRAME_HEADER1) {
            uart_state = UART_STATE_WAIT_H2;
        }
        break;

    case UART_STATE_WAIT_H2:
        if (b == UART_FRAME_HEADER2) {
            uart_state = UART_STATE_RECV_DATA;
            uart_data_idx = 0;
        } else if (b != UART_FRAME_HEADER1) {
            uart_state = UART_STATE_WAIT_H1;
        }
        /* 如果是 0x55 则保持在 WAIT_H2 等待 0x54 */
        break;

    case UART_STATE_RECV_DATA:
        uart_data_buf[uart_data_idx++] = b;
        /* 收齐 6字节数据 + 1字节校验 = 共7字节 */
        if (uart_data_idx >= UART_FRAME_BUF_LEN) {
            /* 计算校验和: XOR of buf[0..5] (6个数据字节) */
            uint8_t ck = 0;
            for (int i = 0; i < UART_FRAME_DATA_LEN; i++) {
                ck ^= uart_data_buf[i];
            }

            /* buf[6] 是接收到的校验字节 */
            if (ck == uart_data_buf[UART_FRAME_DATA_LEN]) {
                /* 解析 Vx, Vy (int16, mm/s) */
                int16_t vx_raw = ((int16_t)uart_data_buf[0] << 8) | uart_data_buf[1];
                int16_t vy_raw = ((int16_t)uart_data_buf[2] << 8) | uart_data_buf[3];
                /* 解析 w (int16, milli-rad/s → float rad/s) */
                int16_t w_raw  = ((int16_t)uart_data_buf[4] << 8) | uart_data_buf[5];

                ui_params.Vx = (float)vx_raw;
                ui_params.Vy = (float)vy_raw;
                ui_params.w  = (float)w_raw / 1000.0f;

                /* 记录收帧时刻，用于超时检测 */
                last_frame_tick = HAL_GetTick();

                /* 非零速度时激活运动 */
                if (vx_raw != 0 || vy_raw != 0 || w_raw != 0) {
                    ui_motion_active = 1;
                } else {
                    ui_motion_active = 0;
                }
                uart_frame_count++;
            }
            /* 校验失败则丢弃此帧 */
            uart_state = UART_STATE_WAIT_H1;
        }
        break;
    }

    /* 重新启动单字节接收 */
    HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);
}

/**
  * @brief  TIM1 更新中断回调函数
  * @note   由 HAL_TIM_IRQHandler(&htim1) 链式调用
  *         TIM1 配置：PSC=7199, ARR=200, 约20ms触发一次
  *         在此读取四个编码器计数值并更新电机转速
  */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {

        motor_speed_update();

        if (ui_motion_active) {
            Motion_Control(ui_params.Vx, ui_params.Vy, ui_params.w);
        } else {
            Motion_Control(0.0f, 0.0f, 0.0f);
        }

    }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  motor_init();
  HAL_Delay(20);
  OLED_Init();

  /* 初始化运动参数 (由 UART 远程控制更新) */
  ui_params.Vx = 0.0f;
  ui_params.Vy = 0.0f;
  ui_params.w = 0.0f;
  ui_params.time_ms = 0;
  ui_motion_active = 0;

  /* 启动 USART3 中断接收 (单字节, 用于帧解析) */
  HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    char buf[22];

    /* ---- 串口超时检测: 超过 500ms 没收有效帧则清空状态机并归零速度 ---- */
    if (last_frame_tick != 0 &&
        (HAL_GetTick() - last_frame_tick) > UART_FRAME_TIMEOUT_MS)
    {
        /* 中止当前 UART 接收, 清空硬件缓冲 */
        HAL_UART_AbortReceive_IT(&huart3);
        /* 重置软件状态机 */
        uart_state = UART_STATE_WAIT_H1;
        uart_data_idx = 0;
        /* 速度归零 */
        ui_params.Vx = 0.0f;
        ui_params.Vy = 0.0f;
        ui_params.w  = 0.0f;
        ui_motion_active = 0;
        /* 重新启动中断接收 */
        HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);
        last_frame_tick = 0;  /* 归零避免重复进入 */
    }

    OLED_NewFrame();

    /* 标题栏 */
    OLED_DrawFilledRectangle(0, 0, 128, 12, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(26, 0, "UART Control", &afont12x6, OLED_COLOR_REVERSED);

    /* Vx */
    snprintf(buf, sizeof(buf), "Vx: %+04d mm/s", (int)ui_params.Vx);
    OLED_PrintASCIIString(0, 14, buf, &afont12x6, OLED_COLOR_NORMAL);

    /* Vy */
    snprintf(buf, sizeof(buf), "Vy: %+04d mm/s", (int)ui_params.Vy);
    OLED_PrintASCIIString(0, 27, buf, &afont12x6, OLED_COLOR_NORMAL);

    /* w */
    {
        int wi = (int)(ui_params.w * 100.0f);
        int wa = (wi < 0) ? -wi : wi;
        snprintf(buf, sizeof(buf), "w : %c%d.%02d rad/s",
                 (wi < 0) ? '-' : '+', wa / 100, wa % 100);
    }
    OLED_PrintASCIIString(0, 40, buf, &afont12x6, OLED_COLOR_NORMAL);

    /* 状态行: 运动状态 + 帧计数 / 字节计数 */
    if (ui_motion_active) {
        snprintf(buf, sizeof(buf), "ON  F:%lu B:%lu",
                 (unsigned long)uart_frame_count,
                 (unsigned long)uart_byte_count);
    } else {
        snprintf(buf, sizeof(buf), "OFF F:%lu B:%lu",
                 (unsigned long)uart_frame_count,
                 (unsigned long)uart_byte_count);
    }
    OLED_PrintASCIIString(0, 53, buf, &afont12x6, OLED_COLOR_NORMAL);

    OLED_ShowFrame();
    HAL_Delay(50);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

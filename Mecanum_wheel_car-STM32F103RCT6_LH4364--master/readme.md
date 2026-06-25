# 麦克纳姆轮小车运动控制系统

## 项目概述

本项目为"汽车数字设计"课程设计——基于 STM32F103RCT6 的麦克纳姆轮小车运动控制系统，集成 K230 AI 视觉模块与 PC 上位机，形成**三端协同**的智能机器人平台。

| 层级 | 硬件 | 功能 |
|------|------|------|
| 上位机 | PC (PyQt5) | 远程遥控、摇杆操控、电机转速曲线监控 |
| 视觉层 | K230 (Micropython) | AI 视觉、WiFi 图传、TCP 文字收发 |
| 控制层 | STM32F103RCT6 | 运动解算、PID 速度环、编码器读取、OLED 交互 |

```
┌──────────────────┐     WiFi / TCP      ┌──────────────────┐
│   PC 上位机       │ ◄──────────────────► │   K230 视觉模块    │
│  (PyQt5 遥控)     │   图传 + 文字指令    │  (AI + 摄像头)    │
└────────┬─────────┘                     └────────┬─────────┘
                                                  │ UART
                    ┌──────────────────┐          │
                     │  STM32F103RCT6   │◄────────┘
                     │  (运动控制器)     │
                     └────────┬─────────┘
                              │ PWM + 编码器
                     ┌────────┴─────────┐
                     │  四路电机 + 编码器│
                     └──────────────────┘
```

## 硬件平台

### STM32 控制器

| 组件 | 型号/规格 |
|------|----------|
| 主控 | 魔女开发板 STM32F103RCT6 (72 MHz, 256KB Flash, 48KB RAM) |
| 电机 | MG513P30 12V 直流减速电机（四台，含霍尔编码器） |
| 电机驱动 | 轮趣四路 TB6612FNG 电机驱动板 |
| 电池 | 12V 航模锂电池（3S LiPo） |
| 显示屏 | SSD1306 0.96 寸 OLED（128×64, I2C, 地址 0x78） |
| 按键 | 五向按键（上/下/左/右/中，硬件上拉，按下低电平） |
| 循迹 | 4 路红外循迹传感器（下拉输入） |
| 车轮 | 60mm 直径麦克纳姆轮，X 型布置 |
| 底盘参数 | 轴距 120mm, 轮距 196mm |

### K230 视觉模块

| 组件 | 说明 |
|------|------|
| 主控 | Canaan K230 (RISC-V 双核, 1.6GHz + 800MHz) |
| 摄像头 | MIPI CSI 接口 |
| 显示 | LCD 800×480 |
| 连接 | WiFi STA 模式，TCP 图传与文字收发 |
| 固件 | Micropython + CanMV |

## 引脚分配

### 电机 PWM（TIM3, 1kHz）

| 通道 | 引脚 | 电机 |
|------|------|------|
| TIM3 CH1 | PA6 | 电机A（右前） |
| TIM3 CH2 | PA7 | 电机B（右后） |
| TIM3 CH3 | PB0 | 电机C（左后） |
| TIM3 CH4 | PB1 | 电机D（左前） |

### 编码器

| 电机 | 编码器 A | 编码器 B | 定时器 | 方式 |
|------|---------|---------|--------|------|
| 电机A | PA15 | PB3 | TIM2 | 硬件编码器 |
| 电机B | PB6 | PB7 | TIM4 | 硬件编码器 |
| 电机C | PC6 | PC7 | TIM8 | 硬件编码器 |
| 电机D | PA1 | PA2 | EXTI1/EXTI2 | 软件编码器 |

### 方向控制

| 电机 | IN1 | IN2 |
|------|-----|-----|
| 电机A | PA11 | PA12 |
| 电机B | PC10 | PC11 |
| 电机C | PC12 | PD2 |
| 电机D | PC8 | PC9 |

### 按键与显示

| 功能 | GPIO | 说明 |
|------|------|------|
| 按键上 | PC4 | 硬件上拉，按下低电平 |
| 按键右 | PC5 | 硬件上拉，按下低电平 |
| 按键下 | PA5 | 硬件上拉，按下低电平 |
| 按键左 | PA4 | 硬件上拉，按下低电平 |
| 按键中 | PB2 | 硬件上拉，按下低电平 |
| OLED SCL | PB8 | I2C1 重映射, 400kHz |
| OLED SDA | PB9 | I2C1 重映射, 400kHz |

### 循迹传感器

| 序号 | GPIO | 说明 |
|------|------|------|
| S1 | PB12 | 下拉输入 |
| S2 | PB13 | 下拉输入 |
| S3 | PB14 | 下拉输入 |
| S4 | PB15 | 下拉输入 |

## 软件架构

```
Mecanum wheel robot/
├── stm32f103rct6/           # STM32 固件（运动控制）
│   ├── Core/                # HAL 层代码（CubeMX 生成）
│   │   ├── Inc/             # main.h, gpio.h, i2c.h, tim.h, usart.h
│   │   └── Src/             # main.c, gpio.c, i2c.c, tim.c, usart.c
│   ├── Motor/               # 电机驱动与 PID 控制
│   │   ├── Inc/motor.h
│   │   └── Src/motor.c
│   ├── Mecanum/             # 麦克纳姆轮运动学解算
│   │   ├── Inc/mecanum.h
│   │   └── Src/mecanum.c
│   ├── OLED/                # SSD1306 OLED 显示驱动
│   │   ├── Inc/oled.h, font.h
│   │   └── Src/oled.c, font.c
│   ├── UI/                  # 按键交互与 OLED 菜单界面
│   │   ├── Inc/ui.h
│   │   └── Src/ui.c
│   ├── cmake/               # CMake 工具链配置
│   ├── CMakeLists.txt
│   ├── CMakePresets.json
│   └── car_project.ioc      # STM32CubeMX 项目文件
│
├── k230/                    # K230 AI 视觉模块
│   ├── k230_video_stream.py     # WiFi 图传服务端（双通道）
│   ├── k230_video_client.py     # PC 端 OpenCV 视频接收
│   ├── k230_text_receiver.py    # TCP 文字接收 + OSD 叠加
│   ├── send_text_to_k230.py     # PC 端文字指令发送
│   ├── wifi_camera_lcd.py       # WiFi 摄像头 LCD 显示
│   ├── examples/                # CanMV 官方示例（405+ 文件）
│   │   ├── 05-AI-Demo/          # AI 推理（人脸/手势/OCR/YOLO）
│   │   ├── 20-YOLO-Module/      # YOLOv5/v8/v11 检测与分割
│   │   ├── 07-April-Tags/       # AprilTag 姿态估计
│   │   └── ...
│   └── libs/                    # 工具库（AI2D, YOLO, PipeLine 等）
│
├── Host_computer/           # PC 上位机
│   └── robot_control_upper.py   # PyQt5 远程遥控程序
│
├── PCB拓展版/               # PCB 扩展板设计
│   ├── stm32f103rct6拓展版.pdf       # 原理图
│   ├── SCH_Schematic1_2026-06-02.pdf # 最新原理图
│   └── ProPrj_stm32f103rct6拓展版_2026-05-28.epro  # 工程文件
│
├── 小车底盘.SLDPRT           # 底盘 SolidWorks 3D 模型
└── readme.md
```

### STM32 控制流程

```
TIM1 中断 (20ms)
  ├── motor_speed_update()     # 读取编码器，计算四路 RPM
  └── Motion_Control(Vx,Vy,w)  # 运动学解算 → PID → PWM 输出
       └── if target==0: 直接刹车，清零 PID 状态

主循环 (50ms)
  └── UI_Update()
       ├── Btn_Scan()           # 按键扫描（100ms 消抖）
       ├── 状态机处理           # NAVIGATE → EDIT → RUNNING
       └── OLED 渲染           # 菜单界面 / 运行界面

UART 通信
  ├── USART1: 预留外部通信
  └── USART3: 预留 K230 / 上位机指令接收
```

### K230 视觉系统

| 脚本 | 功能 |
|------|------|
| `k230_video_stream.py` | 双通道图传：YUV→LCD 硬件直连 + RGB→JPEG→TCP 无线图传 |
| `k230_video_client.py` | PC 端 OpenCV 接收 JPEG 流并显示 |
| `k230_text_receiver.py` | TCP 监听文字指令，叠加 OSD 到 LCD 画面 |
| `send_text_to_k230.py` | PC 端通过 TCP 发送文字到 K230 的 LCD 显示 |

### PC 上位机

`robot_control_upper.py` — 基于 PyQt5 的深色科技风遥控界面，功能包括：

- **视频预览区**：预留 K230 图传画面嵌入区域
- **电机监控面板**：2×2 四路电机曲线图（目标/实际 RPM 实时对比）
- **方向摇杆**：鼠标拖拽 + WASD 键盘双模操控
- **转向摇杆**：独立转向/自转控制
- **速度面板**：± 按钮速度档位调节
- **急停按钮**：一键紧急停止
- **100ms 周期**：读取摇杆输入 → 麦克纳姆解算 → 发送目标转速

## 电机参数

| 参数 | 值 |
|------|-----|
| 编码器脉冲/转 | 780（含 2 倍频） |
| PWM 频率 | 1kHz（TIM3, PSC=71, ARR=999） |
| 控制周期 | 20ms（TIM1, PSC=7199, ARR=200） |
| PID 参数 | Kp=3.0, Ki=2.0, Kd=0 |
| 积分限幅 | ±200 |
| 输出限幅 | ±1000 |

### PID 零目标优化

当目标转速为 0 时，跳过 PID 运算，直接输出 PWM=0 并清零积分/微分状态，确保快速刹停、避免积分残留导致电机慢转。

## UI 操作指南

### 按键功能

| 按键 | GPIO | 方向 |
|------|------|------|
| PC4 | BTN_UP | 上 |
| PC5 | BTN_RIGHT | 右 |
| PA5 | BTN_DOWN | 下 |
| PA4 | BTN_LEFT | 左 |
| PB2 | BTN_CENTER | 中/确认 |

### 三种运行状态

| 状态 | 上/下 | 左/右 | 中键 |
|------|-------|-------|------|
| **NAVIGATE** 导航 | 移动光标 | — | 进入编辑 / 启动运行 |
| **EDIT** 编辑 | 切换编辑项 | 调整数值（长按连发） | 确认并返回 |
| **RUNNING** 运行 | — | — | 任意键中止 |

### 可调参数

| 参数 | 范围 | 步进 |
|------|------|------|
| Vx 横向速度 | -1000 ~ +1000 mm/s | 50 mm/s |
| Vy 纵向速度 | -1000 ~ +1000 mm/s | 50 mm/s |
| w 角速度 | -5.00 ~ +5.00 rad/s | 0.1 rad/s |
| T 运动时间 | 0.1 ~ 10.0 s | 0.1 s |

### 典型操作流程

1. 上电后 OLED 显示参数设置菜单，默认选中 Vx
2. 按 **上/下** 移动 `>` 光标到需要调节的参数
3. 按 **中键** 进入编辑模式（数值闪烁）
4. 按 **左/右** 调整数值，长按可连续调节
5. 按 **中键** 确认，返回导航模式
6. 光标移到 `RUN`，按 **中键** 启动小车
7. 运行期间显示倒计时与进度条，**任意键**可紧急停止

### 运行界面说明

```
┌──────────────────┐
│     RUNNING      │  ← 反色标题栏
│ Vx:+0400 Vy:+0400│  ← 当前运动参数
│ w:+0.00 T:2.00s  │  ← 角速度与设定时间
│ Remain: 1.85s    │  ← 剩余时间
│ ████████░░░░░░░░ │  ← 进度条
│  Press to STOP   │  ← 操作提示
└──────────────────┘
```

## 构建方法

### STM32 固件

- **工具链**: GCC ARM None EABI (`arm-none-eabi-gcc`)
- **构建系统**: CMake 3.22+ + Ninja
- **代码生成**: STM32CubeMX（仅用于外设初始化，非构建必需）

```bash
cd stm32f103rct6
cmake --preset Debug
cmake --build build/Debug
```

当前固件大小：Flash ~29KB / 256KB, RAM ~3.8KB / 48KB。

### K230 部署

将 `k230/` 目录下的脚本通过 CanMV IDE 或 MicroSD 烧录到 K230 模块，上电后自动运行。

```bash
# 启动图传服务端（K230 端）
import k230_video_stream

# PC 端接收视频
python k230_video_client.py
```

### PC 上位机

```bash
cd Host_computer
pip install pyqt5 opencv-python
python robot_control_upper.py
```

## 开发日志

### 2026-06-02 — 上位机与 K230 联调

- PC 上位机 PyQt5 界面开发完成：摇杆操控、电机曲线监控、急停按钮
- K230 WiFi 图传双通道方案（YUV→LCD + RGB→JPEG→TCP）
- K230 TCP 文字收发 + OSD 叠加显示
- 三端通信架构搭建（PC ↔ K230 ↔ STM32）

### 2026-05-30 — UI 交互系统

- 五向按键驱动：100ms 消抖，长按连发（750ms 触发，150ms 间隔）
- OLED 菜单界面：参数设置（Vx/Vy/w/T）+ RUN 执行
- 三状态状态机：NAVIGATE → EDIT → RUNNING
- 运行倒计时与进度条显示
- 任意键紧急停止
- 电机 PID 零目标优化：target=0 时直接刹车

### 2026-05-28 — 基础框架搭建

- STM32CubeMX 外设初始化（时钟、GPIO、TIM、I2C、USART）
- 四路电机 PWM 驱动与方向控制
- 四路编码器读取（3 硬件 + 1 软件 EXTI）
- 20ms 周期速度采样与 RPM 计算
- 四路独立 PID 速度环
- 麦克纳姆轮 X 型布置逆运动学解算
- SSD1306 OLED 驱动（I2C，帧缓冲，绘图 API，ASCII/中文渲染）
- PCB 扩展板设计完成

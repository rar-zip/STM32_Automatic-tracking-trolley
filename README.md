# 智能循迹避障小车

基于 STM32G431 + OpenMV H7 双板架构的智能循迹小车，参加嵌入式竞赛项目。

## 硬件架构

| 模块 | 型号                 | 功能                      |
| ---- | -------------------- | ------------------------- |
| 主控 | STM32F103C8T6        | 电机控制、传感器融合      |
| 视觉 | OpenMV Cam H7        | 黑线识别、位置偏差计算    |
| 电机 | 4路 PWM (TIM4 CH1-4) | 差速驱动                  |
| 通信 | UART (115200 8N1)    | OpenMV → STM32 二进制协议 |
| 循迹 | 红外对管             | 辅助循迹（待集成）        |
| 避障 | 超声波 HC-SR04       | 前方障碍检测（待集成）    |

### 引脚连接

**OpenMV H7 → STM32**

- OpenMV P4 (TX) → STM32 PA10 (RX)
- OpenMV P5 (RX) → STM32 PA9 (TX)
- GND ↔ GND（必须共地）

**STM32 电机驱动**

- PB6 → TIM4_CH1 (左电机 A)
- PB7 → TIM4_CH2 (左电机 B)
- PB8 → TIM4_CH3 (右电机 A)
- PB9 → TIM4_CH4 (右电机 B)

## 通信协议

OpenMV 与 STM32 之间采用二进制帧协议，每帧 9 字节：

```
| 0x55 | 0xAA | x_err (int16 LE) | y_err (int16 LE) | status | checksum | 0x0D |
|------|------|------------------|------------------|--------|----------|------|
|  0   |  1   |    2     3       |    4     5       |   6    |    7     |  8   |
```

- **x_err**: 水平偏差（像素），正值 = 目标在右侧
- **y_err**: 垂直偏差（像素），正值 = 目标在下方
- **status**: `0x00` = 追踪中，`0x01` = 目标丢失
- **checksum**: 前 7 字节（0-6）累加和的低 8 位

> STM32 使用 DMA + IDLE 中断接收，无需轮询。

## 软件架构

```
final/
├── stm32_car/           # STM32F103 固件 (Keil MDK)
│   ├── User/main.c      # 主程序：DMA 接收 + P 控制器循迹
│   ├── Hardware/        # 外设驱动：PWM、电机、按键、LED、OLED
│   ├── Library/         # STM32F10x 标准外设库
│   ├── Start/           # CMSIS 启动文件
│   └── System/          # 延时函数
├── openmv_h7/           # OpenMV H7 固件
│   └── track_binary.py  # 黑线追踪 + 二进制帧发送
└── frontend/            # Web 展示页面
    ├── index.html
    └── style.css
```

### 控制算法

STM32 主循环采用 P（比例）控制器：

- **死区**：|x_err| < 10px 时直行，避免抖动
- **比例控制**：`turn = KP * x_err`，KP=0.4
  - 左轮速度 = BASE_SPEED - turn
  - 右轮速度 = BASE_SPEED + turn
- **丢失保护**：超过 500ms 未收到数据或收到 LOST 状态 → 刹车

## 快速开始

### 1. STM32 固件烧录

用 Keil MDK 打开 `final/stm32_car/Project.uvprojx`，编译后通过 ST-Link / 串口烧录。

### 2. OpenMV 固件部署

用 OpenMV IDE 打开 `final/openmv_h7/track_binary.py`，保存到 OpenMV Cam H7 并运行。

### 3. 调试

- 上电后 OpenMV 绿灯闪烁 = 追踪中，红灯闪烁 = 搜索中
- OpenMV IDE 串行终端可看到 `X_err: xx, Y_err: yy` 或 `LOST`
- STM32 可通过 OLED（待集成）或串口查看接收数据

## 目录说明

```
嵌赛/
├── README.md            # 本文件
├── .gitignore           # 编译产物忽略规则
├── docs/                # 项目文档
├── final/               # 最终参赛代码
└── test/                # 开发测试代码（存档）
    ├── stm32_car_test/  # STM32 通信测试工程
    └── openmv_h7_test/  # OpenMV 追踪测试脚本
```

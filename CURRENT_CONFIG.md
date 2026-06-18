# smart_car 完整技术参考手册

> 基于 STM32F407VET6 的智能巡检小车
> FreeRTOS + CMSIS-RTOS V2 + HAL + CMake | ARM GCC
> WiFi 遥控 · RFID 定位 · 循迹避障 · 温度分级响应

---

## 目录

1. [平台规格](#1-平台规格)
2. [文件清单与职责](#2-文件清单与职责)
3. [RTOS 任务](#3-rtos-任务)
4. [FreeRTOS 资源](#4-freertos-资源)
5. [硬件引脚映射](#5-硬件引脚映射)
6. [电机控制](#6-电机控制)
7. [传感器子系统](#7-传感器子系统)
8. [PID 控制器](#8-pid-控制器)
9. [循迹控制](#9-循迹控制)
10. [避障控制](#10-避障控制)
11. [温度控制](#11-温度控制)
12. [RFID-RC522 子系统](#12-rfid-rc522-子系统)
13. [WiFi 通信协议](#13-wifi-通信协议)
14. [状态机](#14-状态机)
15. [OLED 显示](#15-oled-显示)
16. [调试串口命令](#16-调试串口命令)
17. [上电自检](#17-上电自检)
18. [模块实现状态](#18-模块实现状态)
19. [构建](#19-构建)

---

## 1. 平台规格

| 项目 | 值 |
|---|---|
| 主控 | STM32F407VET6 (Cortex-M4 FPU) |
| 系统主频 | 168 MHz |
| 外部晶振 | 25 MHz |
| Flash | 512 KB |
| RAM | 128 KB + 64 KB CCMRAM |
| HAL 时基 | TIM8 |
| RTOS | FreeRTOS + CMSIS-RTOS V2 |
| 工具链 | ARM GCC (cmake --preset Debug) |
| 输出文件 | build/Debug/smart_car.elf |

---

## 2. 文件清单与职责

### 2.1 应用层任务文件 (`Core/App/Tasks/`)

| 文件 | 功能 |
|---|---|
| **CtrlTask.c** / **Ctrl.h** | **核心控制任务** — 系统状态机主循环。每 30ms 运行一次：读取全部传感器 → 处理 RFID 事件 → 确定状态 → 调度各控制器执行 → 发送电机命令。还负责 RFID 停车测量 (place_N 3s)、终点掉头 (end_stop 1.5s)、WiFi 遥测定时发送 (3s)。`Ctrl.h` 定义了 `SystemState` 枚举、`SensorData_t` 结构体、`MotorCmd_t` 结构体、温度阈值、障碍物阈值等全局类型和常量。 |
| **DriverTask.c** | **电机驱动任务** — 阻塞等待 `MotorActionHandle` 队列，收到 `MotorCmd_t` 后调用 `motor.c` 驱动 TB6612FNG，执行前进/后退/转向/旋转/停止等动作。 |
| **SensorTask.c** | **传感器采集任务** — 每 80ms 采集循迹传感器值并发送到 `TrackHandle` 队列；每 500ms 读取 MLX90614（环境温度 + 物体温度）、AHT20（温湿度）、MQ-8（ADC 值 + 数字量）。通过 `I2CMutexHandle` 互斥保护 I2C 总线。 |
| **HCSR04.c** | **超声波驱动** — 独立任务，每 80ms 触发 5 次测距采样，去除最高最低后取裁剪平均值，输出到全局 `g_distance`。脉冲超时 30000us (~5m)。 |
| **TrackCtrl.c** / **TrackCtrl.h** | **循迹控制** — PID 循迹算法 + 模式状态机（FOLLOW / CROSS / SHARP_LEFT / SHARP_RIGHT / SEARCH_LEFT / SEARCH_RIGHT）。4 路红外传感器查表计算偏差 (-3~+3)，速度 PID + 转向 PID 双环控制。 |
| **ObstacleCtrl.c** / **ObstacleCtrl.h** | **避障控制** — 11 状态避障状态机。检测到 ≤30cm 障碍物时：刹车 → 左/右扫描采样 → 选择较宽侧绕行 → 弧线前进 → 找回轨道。总超时 420 周期自动退出。 |
| **ThermalCtrl.c** / **ThermalCtrl.h** | **温度控制** — Alert 状态直接放行循迹命令；Warning 状态将 PWM 降为 50%；Emergency 状态执行 4 阶段紧急返回（原地 180° 掉头 → 找线 → 循迹回撤 → 完成）。 |
| **RfidTask.c** | **RFID 读取任务** — RC522 底层驱动。轮询 200ms + EXTI 中断唤醒。完整通信序列：Request(REQIDL) → Anticollision → UID 校验。连续 3 次读取失败判定标签离开。输出 `[RFID] NEW TAG` / `[RFID] TAG LOST` 调试信息。 |
| **RfidReader.c** / **RfidReader.h** | **RFID 数据抽象层** — UID 压缩 (CRC8 类算法，4 字节→1 字节)、标签 ID→位置名称映射表 (8 个预定义位置 + unknown)、标签状态查询接口。 |
| **WifiComm.c** / **WifiComm.h** | **WiFi 通信协议** — ESP8266 AP 模式初始化、TCP Server、JSON 遥测构建、命令解析与 Ack 回复、RFID 位置缓存。 |
| **WifiTask.c** | **WiFi 任务入口** — 每 20ms 调用 `Wifi_TaskStep()`，处理 ESP8266 AT 初始化、TCP 数据收发、链路存活保活。 |
| **UartTask.c** | **调试串口任务** — 通过 USART2 接收命令（start/stop/pause/evacuate 等）+ ESP8266 桥接模式切换（esp/esp_off/+++）。 |
| **OledTask.c** | **OLED 显示任务** — 双页面循环显示，每 2s 切换。页面 0：状态 + 距离 + MQ8 + 温湿度；页面 1：AHT20 + 循迹二进制。 |
| **LedTask.c** | **LED 心跳任务** — 每 500ms 切换状态，计数发送到 `LEDFlashHandle` 队列。无实际 LED 硬件 (`BOARD_HAS_STATUS_LED=0`)。 |
| **BatteryCtrl.c** / **BatteryCtrl.h** | **电池管理** — 当前为桩代码，固定返回 100% / 12V。`BatteryCtrl_Return()` 返回空命令（未实现低电量返航逻辑）。 |
| **Encoder.c** / **Encoder.h** | **编码器** — 桩代码 (`BOARD_HAS_ENCODER=0`)，固定返回 0。 |
| **motor.c** / **motor.h** | **电机底层驱动** — TB6612FNG 双 H 桥控制。`Motor_SetSpeed(pwm)` 设置 PWM 占空比 (0-1000)，`Motor_SetDirection(fwd/bwd)` 设置方向 GPIO。PWM 最大值 1000，刹车模式停止。 |
| **pid.c** / **pid.h** | **增量式 PID** — 微分作用在测量值上（避免设定值突变冲击），一阶低通滤波微分项，积分分离抗饱和。 |
| **Aht20.c** / **Aht20.h** | **AHT20 温湿度驱动** — I2C2 读取温湿度传感器。 |
| **SelfTest.c** / **SelfTest.h** | **上电自检** — 8 项测试：UART / OLED / MLX90614 / MQ8 / 循迹 / HCSR04 / 电机 / 蜂鸣器。 |
| **board_compat.h** | **板级兼容层** — 统一外设宏映射（UART、I2C、PWM 定时器）、蜂鸣器控制、电机待机、MQ8 读取。 |

### 2.2 CubeMX 生成文件 (`Core/Src/`, `Core/Inc/`)

| 文件 | 功能 |
|---|---|
| **freertos.c** | FreeRTOS 初始化：创建 9 个任务 + 6 个队列 + 1 个互斥锁 |
| **main.c** | 系统入口，HAL 初始化，调用 `MX_FREERTOS_Init()` |
| **gpio.c** | GPIO 初始化（所有引脚模式/上下拉/速度） |
| **spi.c** | SPI1 初始化（PA5/PA6/PA7，Master，8bit，Mode 0） |
| **usart.c** | USART2 + USART3 初始化（115200 8N1） |
| **stm32f4xx_it.c** | 中断向量表 |
| **stm32f4xx_hal_timebase_tim.c** | HAL 时基 (TIM8) |

---

## 3. RTOS 任务

### 3.1 任务表

| 任务函数 | 名称 | 优先级 | 栈 (字×4) | 运行间隔 | run_count[] 索引 |
|---|---|---|---|---|---|
| `StartLedTask` | LedTask | osPriorityLow (-2) | 1024 B | 500ms | `[0]` |
| `StartUartTask` | UartTask | osPriorityLow (-2) | 1024 B | 50ms | `[1]` |
| `StartOledTask` | OledTask | osPriorityLow (-2) | 1024 B | 300ms | `[2]` |
| `StartHCSR04Task` | HCSR04Task | osPriorityRealtime (12) | 1024 B | 80ms | `[3]` |
| `StartSensorTask` | SensorTask | osPriorityBelowNormal1 (-1) | 1024 B | 80ms | `[4]` |
| `StartDriverTask` | DriverTask | osPriorityAboveNormal (1) | 1024 B | 阻塞 (queue, 100 ticks) | `[5]` |
| `StartCtrlTask` | CtrlTask | osPriorityNormal (0) | 2048 B | 30ms (`osDelay(30)`) | `[6]` |
| `StartRfidTask` | myRfidTask | osPriorityLow (-2) | 1024 B | 200ms 轮询 + EXTI IRQ 唤醒 | `[7]` |
| `StartWifiTask` | WifiTask | osPriorityLow (-2) | 2048 B | 20ms | `[8]` |

### 3.2 任务间通信

```
SensorTask ──(TrackHandle queue: uint8_t)──→ CtrlTask (读取循迹数据)
CtrlTask   ──(MotorActionHandle queue: MotorCmd_t)──→ DriverTask (执行电机动作)
LedTask    ──(LEDFlashHandle queue: uint32_t)──→ (未使用)
```

I2C 总线保护通过 `I2CMutexHandle` 互斥锁。

### 3.3 CtrlTask 主循环 (每 30ms)

```
1. Ctrl_ReadAllSensors()         — 读取全局传感器变量到 SensorData_t
2. Ctrl_HandleRfidEvent(&data)  — RFID 上升沿/下降沿检测
3. Encoder_Reset()              — 重置编码器计数
4. Ctrl_DetermineState(&data)   — 状态判定（含 5s 温度防抖）
5. switch(current_state)        — 调度 TrackCtrl / ThermalCtrl / ObstacleCtrl
6. if (rfid_measure_active)      — 强制停车 3s 测量，到时发 WiFi
7. if (rfid_return_home_active)  — 强制右旋 1.5s 掉头
8. Ctrl_HandleAlarm(state)      — 蜂鸣器/LED 控制
9. Ctrl_HandleWifiReport(&data) — 每 3s 发一次 WiFi 遥测
10. osMessageQueuePut(MotorActionHandle)  — 发送电机命令
11. osDelay(30)
```

---

## 4. FreeRTOS 资源

### 4.1 消息队列

| 句柄 | 类型 | 深度 | 元素大小 | 发送者 | 接收者 |
|---|---|---|---|---|---|
| `TrackHandle` | uint8_t | 16 | 1 B | SensorTask | CtrlTask |
| `MotorActionHandle` | MotorCmd_t | 16 | 12 B | CtrlTask | DriverTask |
| `LEDFlashHandle` | uint32_t | 16 | 4 B | LedTask | (未使用) |
| `DistanceHandle` | float | 16 | 4 B | (未使用) | (未使用) |
| `DriverPWMHandle` | uint32_t | 16 | 4 B | (未使用) | (未使用) |

### 4.2 互斥锁

| 句柄 | 用途 |
|---|---|
| `I2CMutexHandle` | 保护 MLX90614 (I2C1) 和 AHT20 (I2C2) 的 I2C 总线访问 |

### 4.3 事件标志

| 标志 | 用途 |
|---|---|
| `RFID_FLAG_IRQ (0x00000001)` | RC522 IRQ 引脚 (PD7) EXTI 上升沿触发，唤醒 RfidTask |

---

## 5. 硬件引脚映射

### 5.1 串口

| 外设 | TX | RX | 波特率 | 用途 |
|---|---|---|---|---|
| USART2 | PA2 | PA3 | 115200 8N1 | 调试控制台 (`BOARD_DEBUG_UART`) |
| USART3 | PC10 | PC11 | 115200 8N1 | ESP-01S WiFi 模块 (`BOARD_WIFI_UART`) |

### 5.2 I2C

| 外设 | SCL | SDA | 设备 | 地址 |
|---|---|---|---|---|
| I2C1 | PB6 | PB7 | MLX90614 红外测温 | `0x5A` (7-bit) / `0xB4` (8-bit) |
| I2C2 | PB10 | PB11 | AHT20 温湿度 | AHT20 内部地址 |
| I2C3 | PA8 | PC9 | OLED 0.96" 128x64 | `0x3C` / `0x78` (取决于驱动) |

### 5.3 SPI

| 外设 | SCK | MOSI | MISO | CS (软件) | 设备 |
|---|---|---|---|---|---|
| SPI1 | PA5 | PA7 | PA6 | PD3 | RC522 RFID |

SPI 配置：
- Mode 0 (CPOL=0, CPHA=0)
- CubeMX 初始化波特率预分频 = 2（84 MHz）
- RFID 初始化时重配为 16（~10.5 MHz）
- 软件 NSS 控制（PD3 GPIO 拉低选中）

### 5.4 ADC

| 外设 | 引脚 | 通道 | 用途 | 采样方式 |
|---|---|---|---|---|
| ADC1 | PC1 | ADC_CHANNEL_11 | MQ-8 气体传感器模拟量 | 单次转换，轮询等待 |

### 5.5 定时器

| 定时器 | 用途 | 配置 |
|---|---|---|
| TIM1 | 超声波微秒计时 | 预分频 167 → 1 MHz，计数 0~65535 |
| TIM3 | 右电机 PWM | — |
| TIM4 | 左电机 PWM CH2 + 右电机 PWM CH3 | — |
| TIM8 | HAL 时基 | — |

### 5.6 GPIO 完整映射

| 功能 | 引脚 | 方向 | 备注 |
|---|---|---|---|
| **电机驱动 (TB6612FNG)** | | | |
| AIN1_L | PB0 | 输出 | 左电机方向 1 |
| AIN2_L | PC8 | 输出 | 左电机方向 2 |
| BIN1_L | PD0 | 输出 | 左电机方向 3 |
| BIN2_L | PE15 | 输出 | 左电机方向 4 |
| AIN1_R | PB2 | 输出 | 右电机方向 1 |
| AIN2_R | PC12 | 输出 | 右电机方向 2 |
| BIN1_R | PB1 | 输出 | 右电机方向 3 |
| BIN2_R | PA12 | 输出 | 右电机方向 4 |
| STBY_L | PE9 | 输出 | 左电机待机（使能） |
| STBY_R | PA11 | 输出 | 右电机待机（使能） |
| 左 PWM | PD13 (TIM4 CH2) | 输出 | 左电机速度 PWM |
| 右 PWM | PD14 (TIM4 CH3) | 输出 | 右电机速度 PWM |
| **RFID** | | | |
| RC522_SDA (CS) | PD3 | 输出 | 片选，低电平选中 |
| RFID_IRQ (EXTI) | PD7 | 输入 | 上升沿中断 |
| RFID_IRQ (备用) | PD5 | 输入 | 当前未使用 |
| **传感器** | | | |
| X1 (S1, 最右循迹) | PA0 | 输入 | 检测黑线 = SET |
| X2 (S2) | PA1 | 输入 | 检测黑线 = SET |
| X3 (S3) | PD1 | 输入 | 检测黑线 = SET |
| X4 (S4, 最左循迹) | PD2 | 输入 | 检测黑线 = SET |
| Trig (超声触发) | PB8 | 输出 | 15us 高电平触发 |
| Echo (超声回波) | PB9 | 输入 | 正脉宽 = 距离 |
| MQ8_DO | PC13 | 输入 | 数字输出 |
| MG8_AO (MQ8 模拟) | PC1 | 模拟 | ADC1 输入 |
| **其他** | | | |
| BUZZER_IN | PE5 | 输出 | 低电平 = 蜂鸣器开 |
| DS18B20_DQ | PA4 | — | 预留，当前未使用 |

---

## 6. 电机控制

### 6.1 方向控制逻辑 (TB6612FNG)

| 方向 | 左电机 | 右电机 |
|---|---|---|
| 前进 | AIN1=H, AIN2=L, BIN1=H, BIN2=L | BIN1=H, BIN2=L, AIN1=H, AIN2=L |
| 后退 | AIN1=L, AIN2=H, BIN1=L, BIN2=H | BIN1=L, BIN2=H, AIN1=L, AIN2=H |
| 停止 (刹车) | 所有 IN = H, PWM = 0 | 所有 IN = H, PWM = 0 |

`Board_MotorStandbySet(1)` 释放待机（使能电机驱动）。

### 6.2 命令 → 电机动作 (DriverTask)

| MotorCmdType | 左电机 | 右电机 | 左 PWM | 右 PWM |
|---|---|---|---|---|
| `MOTOR_CMD_STOP` | 刹车 | 刹车 | 0 | 0 |
| `MOTOR_CMD_FORWARD` | 前进 | 前进 | `cmd.pwm_left` | `cmd.pwm_right` |
| `MOTOR_CMD_TURN_LEFT` | 前进 | 前进 | `cmd.pwm / 3` | `cmd.pwm` |
| `MOTOR_CMD_TURN_RIGHT` | 前进 | 前进 | `cmd.pwm` | `cmd.pwm / 3` |
| `MOTOR_CMD_SPIN_LEFT` | 后退 | 前进 | `cmd.pwm` | `cmd.pwm` |
| `MOTOR_CMD_SPIN_RIGHT` | 前进 | 后退 | `cmd.pwm` | `cmd.pwm` |

PWM 范围：0 ~ 1000 (`PWM_MAX_VALUE = 1000U`)

---

## 7. 传感器子系统

### 7.1 HC-SR04 超声波

**全局变量：** `volatile float g_distance`

**测量流程：**
1. Trig 输出 15us 高电平
2. 等待 Echo 上升沿，启动 TIM1 计时
3. 等待 Echo 下降沿或 30000us 超时
4. 距离 = 脉宽(us) × 0.017 (声速 340m/s 换算)
5. 每次执行 5 次采样（间隔 80ms），取裁剪平均值

**参数：**

| 参数 | 值 | 说明 |
|---|---|---|
| 采样数 | 5 | 每次任务运行 |
| 采样间隔 | 80ms | 避免回波干扰 |
| 最小有效脉宽 | 117 us | 对应 2cm |
| 最大有效脉宽 | 23529 us | 对应 400cm |
| 超时 | 30000 us | 约 5m |
| 输出方式 | 裁剪平均 | 去除最高最低后算术平均 |

### 7.2 循迹传感器 (反射式红外)

**硬件：** 4 路 TCRT5000 或同类反射式红外传感器。

**引脚组合：**
```
track_value = (X4 << 3) | (X3 << 2) | (X2 << 1) | X1
```

**传感器布局：**
```
     小车前方
  ┌──────────────┐
  │ S1  S2  S3  S4│   ← S1 = X1 (最右), S4 = X4 (最左)
  │ ← ← 车头 ← ← │
  └──────────────┘
```

**输出：** `g_track_status` (0~15)，通过 `TrackHandle` 队列每 80ms 发送到 CtrlTask。

### 7.3 MLX90614 红外测温 (I2C1)

**地址：** `0x5A` (7-bit) / `0xB4` (8-bit)

**寄存器：**

| 寄存器 | 地址 | 说明 |
|---|---|---|
| `MLX90614_REG_TA` | 0x06 | 环境温度 (传感器自身温度) |
| `MLX90614_REG_TOBJ1` | 0x07 | 物体温度 (红外测量目标) |

**读取方式：**
```c
HAL_I2C_Mem_Read(&hi2c1, 0xB4, reg, I2C_MEMADD_SIZE_8BIT, buf, 3, 100);
uint16_t raw = (buf[1] << 8) | buf[0];  // big-endian 16-bit
float temp = raw * 0.02f - 273.15f;     // Kelvin → Celsius
```

**保护：** 使用 `I2CMutexHandle` 互斥锁，I2C 读取失败重试 3 次（间隔 10ms）。

**全局变量：**
- `g_mlx90614_ambient` — 环境温度
- `g_mlx90614_object` — 物体温度

### 7.4 AHT20 温湿度 (I2C2)

**全局变量：**
- `g_aht20_temp` — 温度 (°C)
- `g_aht20_humidity` — 湿度 (%RH)

**更新间隔：** 500ms（与 MLX90614 和 MQ8 同时更新）

### 7.5 MQ-8 气体传感器

**模拟输出：** PC1 → ADC1_CH11 → `g_mq8_adc_raw` (0~4095)
**数字输出：** PC13 → `g_mq8_do` (0/1)

ADC 读取方式：单次转换，`HAL_ADC_PollForConversion(&hadc1, 20)`。

### 7.6 SensorTask 运行时间线

```
T=0ms:     读取循迹 → 发送队列
T=80ms:    读取循迹 → 发送队列
T=160ms:   读取循迹 → 发送队列
T=240ms:   读取循迹 → 发送队列
T=320ms:   读取循迹 → 发送队列
T=400ms:   读取循迹 → 发送队列
T=480ms:   读取循迹 → 发送队列
T=500ms:   读取循迹 → 发送队列 + 读取 MLX90614 + AHT20 + MQ8
T=560ms:   读取循迹 → 发送队列
...循环...
```

---

## 8. PID 控制器

### 8.1 算法描述

增量式 PID，微分作用在测量值上（避免设定值突变冲击），一阶低通滤波微分项，积分分离抗饱和。

### 8.2 计算公式

```
error = setpoint - measurement

P = Kp × error

D_raw = -(measurement - prev_measurement) / Ts          // 微分在测量值
D_state += alpha × (D_raw - D_state)                    // 一阶低通滤波
D = Kd × D_state

I_candidate = integral + Ki × error × Ts                // 积分候选
I_candidate = clamp(I_candidate, I_min, I_max)

output = P + I_candidate + D

// 积分分离：输出饱和且误差同向时，不更新积分
if (!((output > out_max && error > 0) || (output < out_min && error < 0))) {
    integral = I_candidate                               // 提交积分
}

最终输出 = clamp(P + integral + D, out_min, out_max)
```

### 8.3 默认参数

| 参数 | 默认值 |
|---|---|
| 采样时间 (Ts) | 0.03 s |
| 微分滤波 α | 0.35 |
| 输出限幅 | ±1000 |
| 积分限幅 | ±1000 |

---

## 9. 循迹控制

### 9.1 误差查表

从 `Sensor_GetTrackStatus()` 获取 4-bit 循迹值，映射为偏差值：

| 位模式 (X4..X1) | 数值 | 偏差 | 含义 |
|---|---|---|---|
| `0110` | 0x06 | 0 | 居中 (S2+S3 在黑线上) |
| `1001` | 0x09 | 0 | 居中 (S1+S4 在黑线上) |
| `0101` | 0x05 | 0 | 居中 (S1+S3) |
| `1010` | 0x0A | 0 | 居中 (S2+S4) |
| `0010` | 0x02 | -1 | 偏左 (S2 检测到) |
| `0001` | 0x01 | -2 | 显著偏左 (S1 检测到) |
| `0011` | 0x03 | -2 | 显著偏左 (S1+S2) |
| `1101` | 0x0D | -3 | 严重偏左 |
| `1110` | 0x0E | -3 | 严重偏左 |
| `0100` | 0x04 | +1 | 偏右 (S3 检测到) |
| `1000` | 0x08 | +2 | 显著偏右 (S4 检测到) |
| `1100` | 0x0C | +2 | 显著偏右 (S3+S4) |
| `1011` | 0x0B | +3 | 严重偏右 |
| `0000` | 0x00 | last×1.8 | 丢线（增益放大） |
| `1111` | 0x0F | 0 | 十字路口（直行） |

**注意：** 偏差为负 = 向左偏（需向右转修正），偏差为正 = 向右偏（需向左转修正）。

### 9.2 循迹模式状态机

```
                ┌──────────┐
                │  FOLLOW  │ ◄──────── default
                └────┬─────┘
             ┌───────┼──────────┐
             ▼       ▼          ▼
         ┌──────┐ ┌────────┐ ┌──────────┐
         │CROSS │ │SHARP_L │ │SHARP_R   │
         │(0x0F)│ │left    │ │right     │
         └──┬───┘ └───┬────┘ └─────┬────┘
            └──┬──────┴──────┬─────┘
               ▼             ▼
         ┌──────────┐ ┌──────────┐
         │SEARCH_L  │ │SEARCH_R  │
         │(left)    │ │(right)   │
         └──────────┘ └──────────┘
```

| 模式 | 进入条件 | 行为 | PWM | 退出条件 |
|---|---|---|---|---|
| FOLLOW | 默认 | PID 转向 + 速度控制 | PID 输出 | track==0x0F / 急弯 / 丢线 |
| CROSS | track == 0x0F | 直行 8 周期 (230) | 230 | 8 周期完成 → FOLLOW |
| SHARP_LEFT | 偏差 < -1 且模式为 01/03/0D/0E | 左转 18 周期 | 520 | 18 周期 → FOLLOW 或 SEARCH |
| SHARP_RIGHT | 偏差 > 1 且模式为 08/0B/0C | 右转 18 周期 | 520 | 18 周期 → FOLLOW 或 SEARCH |
| SEARCH_LEFT | 急弯后仍丢线 | 原地左旋 50 周期 | 430 | 找到线 → FOLLOW |
| SEARCH_RIGHT | 急弯后仍丢线 | 原地右旋 50 周期 | 430 | 找到线 → FOLLOW |

### 9.3 PID 参数

#### 速度 PID (外环)

| 参数 | 值 |
|---|---|
| 目标速度 (setpoint) | 260.0 |
| Kp | 2.0 |
| Ki | 0.4 |
| Kd | 0.0 |
| 输出限幅 | 0.0 ~ 600.0 |
| 积分限幅 | -120.0 ~ 120.0 |

速度 PID 输出为直行 PWM 基准值，再叠加转向 PID 的修正。

#### 转向 PID (内环)

| 参数 | 值 |
|---|---|
| 目标偏差 (setpoint) | 0.0 |
| Kp | 95.0 |
| Ki | 8.0 |
| Kd | 20.0 |
| 输出限幅 | -320.0 ~ 320.0 |
| 积分限幅 | -80.0 ~ 80.0 |
| 微分滤波 α | 0.28 |

转向 PID 输出修正量，叠加到左右 PWM：
```
pwm_left  = speed_pid_out - turn_pid_out
pwm_right = speed_pid_out + turn_pid_out
```

### 9.4 PWM 限幅常量

| 常量 | 值 | 说明 |
|---|---|---|
| `TRACK_PWM_MIN` | 60 | 最小 PWM |
| `TRACK_PWM_MAX` | 620 | 最大 PWM |
| `TRACK_LOST_GAIN` | 1.8 | 丢线时偏差增益 |
| `TRACK_CROSS_SPEED` | 230 | 十字路口直行速度 |
| `TRACK_SHARP_TURN_PWM` | 520 | 急弯旋转 PWM |
| `TRACK_SEARCH_PWM` | 430 | 搜索模式旋转 PWM |

---

## 10. 避障控制

### 10.1 触发条件

障碍物距离 ≤ 30.0 cm (`OBS_DETECT_DIST = 30.0f`)

### 10.2 11 状态避障状态机

```
                  ┌─────────┐
                  │  IDLE   │ ◄── 默认 / 完成后回到此状态
                  └────┬────┘
                       │ 距离 ≤ 30cm
                       ▼
                  ┌─────────┐
           ┌─────►│  BRAKE  │ 刹车 8 周期
           │      └────┬────┘
           │           │
           │      ┌────▼────────┐
           │      │ SCAN_LEFT   │ 左转 18 周期 (620)
           │      │ _TURN       │
           │      └────┬────────┘
           │           │
           │      ┌────▼──────────┐
           │      │ SCAN_LEFT     │ 停车 14 周期，记录左距离
           │      │ _SAMPLE       │
           │      └────┬──────────┘
           │           │
           │      ┌────▼──────────┐
           │      │ SCAN_LEFT     │ 右转回位 18 周期 (620)
           │      │ _RETURN       │
           │      └────┬──────────┘
           │           │
           │      ┌────▼─────────┐
           │      │ SCAN_RIGHT   │ 右转 18 周期 (620)
           │      │ _TURN        │
           │      └────┬─────────┘
           │           │
           │      ┌────▼───────────┐
           │      │ SCAN_RIGHT     │ 停车 14 周期，记录右距离
           │      │ _SAMPLE        │
           │      └────┬───────────┘
           │           │
           │      ┌────▼───────────┐
           │      │ SCAN_RIGHT     │ 左转回位 18 周期 (620)
           │      │ _RETURN        │
           │      └────┬───────────┘
           │           │
           │      ┌────▼───────────┐
           │      │  TURN_CHOICE   │ 选较宽侧转 90° (27 周期, 620)
           │      └────┬───────────┘
           │           │
           │      ┌────▼───────────┐
           │      │   ADVANCE      │ 弧线前进 18~70 周期
           │      └────┬───────────┘
           │           │
           │      ┌────▼───────────┐
           │      │  FIND_LINE     │ 反向旋转找线 50 周期超时
           │      └────┬───────────┘
           │           │
           └───────────┘
```

### 10.3 绕行方向选择

```
obs_bypass_dir = (左距离 >= 右距离) ? -1 (向左绕) : 1 (向右绕)
```

### 10.4 弧线前进 PWM

| 绕行方向 | pwm_left | pwm_right |
|---|---|---|
| 向左绕 (-1) | 196 (280×70%) | 280 (100%) |
| 向右绕 (+1) | 280 (100%) | 196 (280×70%) |

### 10.5 时序常量

| 常量 | 值 | 说明 |
|---|---|---|
| `OBS_SCAN_PWM` | 620 | 扫描旋转速度 |
| `OBS_FIND_PWM` | 420 | 找线旋转速度 |
| `OBS_FORWARD_PWM` | 280 | 绕行前进速度 |
| `OBS_BRAKE_CYCLES` | 8 | 刹车周期数 |
| `OBS_SCAN_ANGLE_CYCLES` | 18 | 扫描 90° 旋转周期 |
| `OBS_SAMPLE_CYCLES` | 14 | 距离采样周期 |
| `OBS_TURN_90_CYCLES` | 27 | 90° 转弯周期 |
| `OBS_ADVANCE_MIN_CYCLES` | 18 | 绕行前进最少 |
| `OBS_ADVANCE_MAX_CYCLES` | 70 | 绕行前进最多 |
| `OBS_FIND_LINE_TIMEOUT` | 50 | 找线超时 |
| `OBS_TOTAL_TIMEOUT` | 420 | 总超时 |

---

## 11. 温度控制

### 11.1 温度阈值

| 阈值宏 | 值 | 对应状态 | 行为 |
|---|---|---|---|
| `THERMAL_ALERT_THRESHOLD` | ≥ 29.0°C | STATE_THERMAL_ALERT | 继续巡逻，蜂鸣器关 |
| `THERMAL_WARNING_THRESHOLD` | ≥ 30.0°C | STATE_THERMAL_WARNING | 速度 50%，蜂鸣器开 |
| `THERMAL_EMERGENCY_THRESHOLD` | ≥ 31.0°C | STATE_EMERGENCY | 紧急返回，蜂鸣器 + LED |

### 11.2 各状态控制行为

**STATE_THERMAL_ALERT (`ThermalCtrl_Alert`)：**
- 直接返回循迹命令，不做任何修改
- 蜂鸣器关闭

**STATE_THERMAL_WARNING (`ThermalCtrl_Warning`)：**
- 将左右 PWM 降至 50%（最小不低于 50）
- 蜂鸣器打开

**STATE_EMERGENCY (`ThermalCtrl_Emergency`)：**
- 执行紧急返回状态机（覆盖循迹命令）

### 11.3 紧急返回状态机

```
RETURN_IDLE(0) → RETURN_SPIN_180(1) → RETURN_FIND_LINE(2)
              → RETURN_FOLLOW_LINE(3) → RETURN_DONE(4)
```

| 状态 | 动作 | PWM | 退出条件 |
|---|---|---|---|
| RETURN_IDLE | 初始化，切到 SPIN_180 | — | 立即 |
| RETURN_SPIN_180 | 原地右旋 180° | 700 | 53 周期 (~1.6s) |
| RETURN_FIND_LINE | 原地右旋找线 | 420 | 检测到轨道 或 60 周期超时 |
| RETURN_FOLLOW_LINE | 循迹返回（遇障碍停） | TrackCtrl 输出 | `ObstacleCtrl_IsDone()` |
| RETURN_DONE | 停车 | STOP | — |

---

## 12. RFID-RC522 子系统

### 12.1 引脚连接

| RC522 引脚 | STM32 引脚 | 说明 |
|---|---|---|
| SCK | PA5 | SPI1 时钟 (~10.5MHz) |
| MOSI | PA7 | SPI1 主机输出 |
| MISO | PA6 | SPI1 主机输入 |
| SDA (NSS) | PD3 | 片选，低电平选中 |
| IRQ | PD7 | EXTI 上升沿中断 |
| RST | — | 通过软复位控制 |
| 3.3V | 3.3V | 供电 |
| GND | GND | 地 |

### 12.2 RC522 初始化寄存器值

```
T_MODE      = 0x8D   // 定时器模式：自动 RF 场开启
T_PRESCALER = 0x3E   // 定时器预分频
T_RELOAD_L  = 30     // 定时器重装值低字节
T_RELOAD_H  = 0     // 定时器重装值高字节
TX_ASK      = 0x40   // TX ASK 调制
MODE        = 0x3D   // 模式寄存器
RFCFG       = 0x70   // RF 配置
TX_CONTROL  = 0x83   // 天线开启 (bit[1:0] = 11)
```

### 12.3 通信序列

每次读取标签分两步：

```
Step 1: Request (REQIDL = 0x26)
  ─→ 发送 7-bit REQIDL 命令
  ←─ 期望 16-bit 回复 (tag type)

Step 2: Anticollision (ANTICOLL = 0x93)
  ─→ 发送 0x93, 0x20
  ←─ 期望 40-bit 回复 (4-byte UID + 1-byte XOR checksum)
  校验: uid[4] == uid[0] ^ uid[1] ^ uid[2] ^ uid[3]
```

Rfid_ToCard 底层函数：
1. 使能中断 (COMMIEN = 0x77)
2. 清中断标志，清 FIFO
3. 写入发送数据到 FIFO
4. 写入 TRANSCEIVE 命令
5. 置位 BIT_FRAMING 启动发送
6. 轮询 COMMIRQ 寄存器等待完成 (200 次循环)
7. 检查错误寄存器 (0x1B mask)
8. 从 FIFO 读取回传数据

### 12.4 UID 压缩 (CRC8 类)

```c
uint8_t Rfid_CompressUid(const uint8_t *uid, uint8_t uid_size) {
    uint8_t result = 0;
    for (i = 0; i < uid_size; i++) {
        result = (result << 1) | (result >> 7);  // 循环左移 1 位
        result ^= uid[i];                          // 异或当前字节
    }
    if (result == 0) result = uid[0];
    if (result == 0) result = 1;
    return result;
}
```

**目的：** 将 4 字节 UID（如 `90:7C:A6:02`）压缩为 1 字节 ID（如 `58`），用于稳定标识标签位置。

### 12.5 标签位置映射

| 压缩 ID | 位置名 | 动作 |
|---|---|---|
| 58 | `start` | 调用 `Ctrl_Start()` → 开始巡逻 |
| 53 | `place_1` | 停车 3s → 测量距离+温度 → 发送 WiFi 遥测 → 继续 |
| 78 | `place_2` | 同上 |
| 199 | `place_3` | 同上 |
| 95 | `place_4` | 同上 |
| 86 | `place_5` | 同上 |
| 111 | `place_6` | 同上 |
| 228 | `end_stop` | 原地右旋 1.5s (PWM=1400) → 继续巡逻（返航回桩） |
| 其他 | `unknown` | 忽略，不触发任何动作 |

### 12.6 标签存在检测

- **上升沿：** 连续读到 UID（`Rfid_ReadCardUid == OK`）且与上次 UID 不同 → `Rfid_UpdateUid()` 更新 + 串口输出 `[RFID] NEW TAG`
- **保持：** 连续读到相同 UID → 静默（只重置 `miss_count = 0`）
- **下降沿：** 连续 3 次 (`RFID_MISS_LIMIT = 3`) 读不到标签 → 串口输出 `[RFID] TAG LOST` + `Rfid_ClearTag()`
- **CtrlTask 侧：** 只检测 `Rfid_IsTagPresent()` 的上升沿（`prev_rfid_present → 1`），下降沿不触发动作，但位置已被 WiFi 缓存

### 12.7 RfidTask 完整循环

```
for (;;) {
    // 阻塞等待 200ms 或 EXTI IRQ 唤醒
    osThreadFlagsWait(RFID_FLAG_IRQ, osFlagsWaitAny, 200);

    if (Rfid_ReadCardUid(uid, &uid_size) == OK) {
        if (UID 与上次不同) {
            更新 RFID 状态
            打印: [RFID] NEW TAG  UID=xx:xx:xx:xx  id=XX loc=XXXX
            miss_count = 0
        } else {
            miss_count = 0  // 相同标签继续存在
        }
    } else {
        miss_count++
        if (miss_count >= 3) {
            打印: [RFID] TAG LOST  id=XX
            Rfid_ClearTag()
        }
    }
}
```

### 12.8 RFID 位置缓存机制 (WiFi 联动)

```
时间线示例：
T=0s:    WiFi 遥测发送 → rfid_loc = "start" (缓存)
T=1.2s:  小车经过 place_1 标签
         RfidTask: [RFID] NEW TAG UID=... id=53 loc=place_1
         CtrlTask: 检测到上升沿
                   → Ctrl_Printf("[CTRL] RFID id=53 loc=place_1")
                   → Wifi_UpdateRfidLocation("place_1")  ← 立即推送给 WiFi
                   → rfid_measure_active = 1 (停车 3s)
T=1.5s:  小车离开 place_1 范围 → Rfid_ReadTag() = 0
T=3.0s:  WiFi 遥测发送 ← 从缓存读取 rfid_loc = "place_1" ✅
```

关键：`Wifi_UpdateRfidLocation()` 在 CtrlTask 检测到新标签时立即被调用，将位置名写入 `g_wifi_last_rfid_loc`。WiFi 遥测构建时直接读此缓存，不依赖实时 `Rfid_ReadTag()`（可能为 0）。

### 12.9 串口调试输出示例

```
[RFID] Task started, waiting for tags...
[RFID] NEW TAG  UID=2A:3B:4C:5D  id=58 loc=start
[CTRL] RFID id=58 loc=start
[CTRL] -> START patrolling

[RFID] NEW TAG  UID=90:7C:A6:02  id=53 loc=place_1
[CTRL] RFID id=53 loc=place_1
[CTRL] -> MEASURE 3s at place_1
[CTRL] Measuring 1/3s  dist=12.3 temp=25.4
[CTRL] Measuring 2/3s  dist=12.3 temp=25.4
[CTRL] Measure done, send data via WiFi

[RFID] TAG LOST  id=53
[RFID] NEW TAG  UID=60:80:A6:02  id=228 loc=end_stop
[CTRL] RFID id=228 loc=end_stop
[CTRL] -> RETURN HOME spin 180
[CTRL] Return home done, resume patrol
```

---

## 13. WiFi 通信协议

### 13.1 硬件与连接

| 参数 | 值 |
|---|---|
| 模块 | ESP8266 (ESP-01S) |
| USART | USART3 (PC10=TX, PC11=RX) |
| 波特率 | 115200 8N1 |
| 模式 | Soft AP |
| SSID | `SmartCar_F407` |
| 密码 | `12345678` |
| IP | `192.168.4.1` |
| TCP 端口 | 8080 |
| 连接方式 | 手机 TCP Client 连接小车 TCP Server |

### 13.2 ESP8266 初始化流程

逐步发送 AT 指令，每步最多重试 2 次，间隔 350ms，响应超时 2500ms：

| 步骤 | 命令 | 说明 |
|---|---|---|
| 1 | `AT` | 测试连接 |
| 2 | `ATE0` | 关闭回显 |
| 3 | `AT+CWMODE=2` | 设为 Soft AP 模式 |
| 4 | `AT+CWSAP="SmartCar_F407","12345678",5,3` | 配置 AP（信道 5，加密 WPA2） |
| 5 | `AT+CIPMUX=1` | 启用多连接 |
| 6 | `AT+CIPSERVER=1,8080` | 启动 TCP Server，端口 8080 |
| 7 | `AT+CIPSTO=0` | 超时设为 0（永不超时断开） |
| 8 | `AT+CIFSR` | 获取 IP 地址 |

成功后输出：`[WIFI] AP ready: SSID=SmartCar_F407 IP=192.168.4.1 PORT=8080`

### 13.3 小车 → 手机 (遥测)

**发送方式：** 每 3 秒自动发送一次（通过 `Wifi_TaskStep` 的链路存活保活机制）。

**触发条件：** TCP 客户端已连接 + 距离上次存活发送 ≥ 3s + 距离上次 TCP 发送 ≥ 800ms

**遥测 JSON 格式：**
```json
{"type":"telemetry","MLX_obj":36.5,"MQ8":1230.5,"AHT_temp":25.4,"AHT_hum":55.2,"dist":12.3,"track":5,"track_bin":"0101","rfid_loc":"place_1","state":"start_patrol"}
```

**字段说明：**

| 字段 | 类型 | 示例 | 说明 |
|---|---|---|---|---|
| `type` | string | `"telemetry"` | 固定标识 |
| `MLX_obj` | float(1位小数) | `36.5` | MLX90614 物体温度 °C（红外测温） |
| `MQ8` | float(1位小数) | `1230.5` | MQ-8 ADC 值 ×10 再格式化 |
| `AHT_temp` | float(1位小数) | `25.4` | AHT20 温度 °C |
| `AHT_hum` | float(1位小数) | `55.2` | AHT20 湿度 %RH |
| `dist` | float(1位小数) | `12.3` | 超声波距离 cm |
| `track` | int(0-15) | `5` | 循迹 4-bit 值 |
| `track_bin` | string | `"0101"` | 循迹二进制 |X4 X3 X2 X1| |
| `rfid_loc` | string | `"place_1"` | RFID 最新标签位置（缓存） |
| `state` | string | `"start_patrol"` | 系统状态名 |

**特殊：** `Ctrl_HandleRfidEvent` 中 place_N 测量结束后也会调用 `Wifi_SendTelemetry(&data)` 立即发送一次遥测。

### 13.4 手机 → 小车 (命令 & Ack)

手机发送命令字符串（TCP payload），小车识别后回复 Ack JSON。

**命令解析方式：** 使用 `strstr()` 子串匹配（按顺序检查）。

**支持的命令列表：**

| 命令字符串 | 小车动作 | Ack JSON `result` |
|---|---|---|
| `"emergency_stop"` | `Ctrl_Stop()` | `"evacuate"` |
| `"start_patrol"` | `Ctrl_Start()` | `"start_patrol"` |
| `"return_home"` | `Ctrl_Start()` | `"return_home"` |
| `"temp_warning"` | 无动作（仅回应） | `"temp_warning"` |
| `"temp_alarm"` | 无动作（仅回应） | `"temp_alarm"` |
| `"idle"` | `Ctrl_Stop()` | `"idle"` |
| `"evacuate"` | `Ctrl_RequestEmergency()` | `"evacuate"` |
| `"pause"` | `Ctrl_Stop()` | `"idle"` |
| `"manual_reset"` | 无动作 | `"ignored"` |
| `"start"` | `Ctrl_Start()` | `"start_patrol"` |
| `"stop"` | `Ctrl_Stop()` | `"idle"` |
| `"ping"` 或 `"hello"` | 无动作 | `"pong"` |
| `"status"` | 查询当前状态 | `"<当前状态名>"` |
| 其他 | 无动作 | `"ignored"` |

**注意：** `strstr()` 子串匹配，所以发送 `"start_patrol"` 也会匹配，因为 `"start"` 是它的子串。但 `"start_patrol"` 在 `"start"` 之前检查，所以优先匹配 `"start_patrol"` → 正确。

**Ack 格式：**
```json
{"type":"ack","cmd":"emergency_stop","result":"evacuate"}
```

### 13.5 JSON 格式总览

**遥测 (telemetry)：**
```json
{"type":"telemetry","MLX_obj":<float>,"MQ8":<float>,"AHT_temp":<float>,"AHT_hum":<float>,"dist":<float>,"track":<int>,"track_bin":"<str>","rfid_loc":"<str>","state":"<str>"}
```

**应答 (ack)：**
```json
{"type":"ack","cmd":"<str>","result":"<str>"}
```

*(仅 telemetry 和 ack 两种 JSON 类型在主动使用)*

### 13.6 Wifi_TaskStep 完整流程

```
Wifi_TaskStep() 每 20ms 调用一次:

1. Wifi_ServiceAtInit()
   — 如果 AP 未就绪，逐步发送初始化 AT 指令
   — 每步等待 350ms，超时 2500ms，最多重试 2 次
   — 全部完成后 g_wifi_ap_ready = 1

2. Wifi_ServiceStatusPoll()
   — 如果 AP 就绪但无客户端连接，每 1s 发 AT+CIPSTATUS 轮询

3. 如果桥接模式 → 跳过后续

4. Wifi_ProcessRxText()
   — 解析 +IPD,<link>,<len>:<payload> 提取 TCP 命令
   — 或者直接匹配关键字 (start/stop/ping 等)
   — 处理后调用 Wifi_ProcessCommand()

5. 链路存活检测 (3s 一次)
   — 客户端已连接 + 距上次存活 ≥ 3s + 距上次发送 ≥ 800ms
   — → 调用 Wifi_SendLiveTelemetry() 发送遥测

6. 队列发送 (800ms 间隔)
   — 从 g_wifi_tx_queue 取出消息
   — 根据 kind 构建 JSON（telemetry / alert / rfid / line）
   — 调用 Wifi_SendTcpPayload() 发送
```

### 13.7 TCP 发送函数

```c
Wifi_SendTcpPayload("json_data\n")
  → 发送 "AT+CIPSEND=<link>,<len>\r\n"
  → osDelay(250ms)
  → UART 发送 JSON 数据
  → 之后 800ms 内不再发送
```

### 13.8 桥接模式

通过 USART2（调试串口）发送 `esp` / `esp_on` / `at` 进入桥接模式，将 USART2 数据直接透传到 USART3（ESP8266 AT 指令接口）。

```
PC ←→ USART2 ←→ USART3 ←→ ESP-01S
```

退出方式：
- 发送 `+++`（三个加号）
- 发送 `esp_off`

---

## 14. 状态机

### 14.1 状态枚举

```c
typedef enum {
    STATE_STANDBY = 0,         // 待机
    STATE_PATROL,              // 巡逻
    STATE_THERMAL_ALERT,       // 温度预警
    STATE_THERMAL_WARNING,     // 温度报警
    STATE_EMERGENCY,           // 紧急撤离
    STATE_LOW_BATTERY          // 低电量返回
} SystemState;
```

### 14.2 状态名映射

| 枚举 | 整数值 | WiFi state | OLED 显示 | 说明 |
|---|---|---|---|---|
| `STATE_STANDBY` | 0 | `"idle"` | `IDLE` | 未启动/手动停止 |
| `STATE_PATROL` | 1 | `"start_patrol"` | `PATROL` | 正常巡逻 |
| `STATE_THERMAL_ALERT` | 2 | `"temp_warning"` | `T_WARN` | 温度 ≥ 29°C |
| `STATE_THERMAL_WARNING` | 3 | `"temp_alarm"` | `T_ALARM` | 温度 ≥ 30°C |
| `STATE_EMERGENCY` | 4 | `"evacuate"` | `EVACUATE` | 温度 ≥ 31°C |
| `STATE_LOW_BATTERY` | 5 | `"return_home"` | `RET_HOME` | 终点 RFID 触发 |

### 14.3 状态判定优先级 (Ctrl_DetermineState)

从高到低：
```
1. rfid_return_home_active == true     → STATE_LOW_BATTERY
2. manual_override_enabled == true     → manual_override_state
3. !system_started                     → STATE_STANDBY
4. temperature ≥ 31°C                  → STATE_EMERGENCY
5. temperature ≥ 30°C                  → STATE_THERMAL_WARNING
6. temperature ≥ 29°C                  → STATE_THERMAL_ALERT
7. else                                → STATE_PATROL
```

### 14.4 温度状态防抖 (5000ms)

从任意温度状态回退到 `STATE_PATROL` 时，强制保持原状态 5 秒：

```
温度升至 31°C → STATE_EMERGENCY      (立即)
温度降至 28°C → STATE_EMERGENCY      (开始 5s 计时)
              → STATE_EMERGENCY      (第 3 秒)
              → STATE_PATROL         (5 秒后)

温度升降跨越预警/报警时无延迟：
  29°C → STATE_THERMAL_ALERT    (立即)
  30°C → STATE_THERMAL_WARNING  (立即)
  28°C → STATE_THERMAL_ALERT    (立即降级，无延迟)
  24°C → start 5s 计时          (5s 后回 PATROL)
```

### 14.5 状态切换动作

| 状态 | 电机行为 | 蜂鸣器 | LED | 避障 |
|---|---|---|---|---|
| IDLE | STOP | OFF | — | — |
| PATROL | TrackCtrl + ObstacleCtrl | OFF | — | 启用 |
| T_WARN | TrackCtrl + ObstacleCtrl (正常) | OFF | — | 启用 |
| T_ALARM | TrackCtrl + ObstacleCtrl (50% PWM) | ON | — | 启用 |
| EVACUATE | ThermalCtrl 紧急返回 | ON | 闪烁 | 内部处理 |
| RET_HOME | BatteryCtrl_Return | — | — | — |

### 14.6 状态转换图

```
               ┌──────────────────────────────────────┐
               │                                      │
               ▼                                      │
          ┌─────────┐    RFID start    ┌──────────┐   │
          │  IDLE   │ ────────────────► │  PATROL  │   │
          │ (STANDBY)│ ◄────── stop ────│          │   │
          └─────────┘                  └─────┬─────┘   │
               ▲                             │         │
               │                    ┌────────┼────┐    │
               │                    │        │    │    │
               │              ┌─────▼──┐ ┌──▼────┐│    │
               │              │T_WARN  │ │T_ALARM││    │
               │              │(≥29°C) │ │(≥30°C)││    │
               │              └────┬───┘ └──┬────┘│    │
               │                   │        │     │    │
               │              ┌────▼────────▼──┐  │    │
               │              │   EVACUATE     │  │    │
               │              │   (≥31°C)      │  │    │
               │              └───────┬────────┘  │    │
               │                      │           │    │
               │              ┌───────▼────────┐  │    │
               │              │   RET_HOME     │  │    │
               │              │ (end_stop RFID)│  │    │
               │              └───────┬────────┘  │    │
               │                      │           │    │
               └──────────────────────┘───────────┘    │
                                  (5s 防抖后回 PATROL) ──┘
```

---

## 15. OLED 显示

### 15.1 页面切换

每 2 秒切换一次页面（`page ^= 1`）。

### 15.2 页面 0 — 状态 + 主要传感器

```
ST:<状态名>             ← IDLE/PATROL/T_WARN/T_ALARM/EVACUATE/RET_HOME
DIS:<距离>cm            ← 超声波距离
MQ8:<ADC值> D:<0/1>    ← MQ-8 模拟值 + 数字值
T:<温度> H:<湿度>       ← 温度 + 湿度
```

### 15.3 页面 1 — 环境 + 循迹

```
AHT:<温度> H:<湿度>     ← AHT20 温度 + 湿度
TRACK:<bit3><bit2><bit1><bit0>  ← 循迹传感器二进制
(空)
(空)
```

### 15.4 显示限幅

| 显示值 | 范围 |
|---|---|
| 距离 (×10) | 0 ~ 9999 |
| MLX90614 温度 (×10) | -999 ~ 999 |
| AHT20 温度 (×10) | -999 ~ 999 |
| 湿度 (×10) | 0 ~ 999 |

---

## 16. 调试串口命令

通过 USART2 (115200 8N1) 发送，使用 `strcmp()` 精确匹配。

| 命令 | 动作 |
|---|---|
| `start` | `Ctrl_Start()` → 开始巡逻 |
| `stop` | `Ctrl_Stop()` → 刹车停止 |
| `pause` | `Ctrl_Stop()` → 刹车停止 |
| `evacuate` | `Ctrl_RequestEmergency()` → 紧急撤离 |
| `return_home` | `Ctrl_Start()` → 开始巡逻 |
| `emergency_stop` | `Ctrl_Stop()` → 刹车停止 |
| `manual_reset` | 忽略 |
| `esp` / `esp_on` / `at` | 进入 ESP8266 桥接模式 |
| `esp_off` | 退出 ESP8266 桥接模式 |
| `+++` | 退出 ESP8266 桥接模式 |
| 其他 | `[CMD] Unknown: <命令>` |

### 调试串口输出格式

```
// 上电自检
[POST] UART online
[POST] OLED: PASS
[POST] MLX90614: PASS
[POST] MQ8 AO raw: 1234
[POST] MQ8 DO: 0
[POST] Track bits: 06
[POST] HCSR04: PASS
[POST] Motor DRV: PASS
[POST] Buzzer: PASS
Result: 8/8 PASS
System Ready!

// WiFi 初始化
[WIFI] AP ready: SSID=SmartCar_F407 IP=192.168.4.1 PORT=8080

// 心跳 (CTRLTASK_VERBOSE_LOG=1 时)
[HB] led=123 uart=456 oled=789 hcsr04=101 sensor=112 driver=131 ctrl=415 rfid=161 wifi=718 drop=0 dist=12.3

// RFID
[RFID] Task started, waiting for tags...
[RFID] NEW TAG  UID=2A:3B:4C:5D  id=58 loc=start
[CTRL] RFID id=58 loc=start
[CTRL] -> START patrolling

// RFID 测量
[CTRL] RFID id=53 loc=place_1
[CTRL] -> MEASURE 3s at place_1
[CTRL] Measuring 1/3s  dist=12.3 temp=25.4
[CTRL] Measure done, send data via WiFi

// RFID 终点掉头
[CTRL] RFID id=228 loc=end_stop
[CTRL] -> RETURN HOME spin 180
[CTRL] Return home done, resume patrol

// 调试串口命令反馈
[CMD] start
[CMD] stop
[CMD] Unknown: xyz
```

---

## 17. 上电自检

上电后在 USART2 输出自检结果。

| # | 测试 | 方法 | 通过标准 |
|---|---|---|---|
| 1 | UART | 发送探测字符串 | `HAL_UART_Transmit` 返回 `HAL_OK` |
| 2 | OLED | I2C 设备就绪检查，地址 0x78 | `HAL_I2C_IsDeviceReady` 返回 `HAL_OK` |
| 3 | MLX90614 | I2C 设备就绪检查，地址 0xB4 | `HAL_I2C_IsDeviceReady` 返回 `HAL_OK` |
| 4 | MQ8 | ADC1 单次转换 | `hadc1.Instance != NULL` |
| 5 | 循迹 | 读取 X1~X4 GPIO | 始终 PASS |
| 6 | HCSR04 | TIM1 实例检查 + 触发脉冲 | `htim1.Instance != NULL` |
| 7 | 电机 | TIM3 + TIM4 实例检查 + 开启 PWM | `htim3/htim4.Instance != NULL` |
| 8 | 蜂鸣器 | 打开 40ms | 始终 PASS |

结果输出：
```
Result: 8/8 PASS
System Ready!
```
或：
```
Result: 5/8 PASS
Warning: Some modules failed!
```

---

## 18. 模块实现状态

| 模块 | 状态 | 详细说明 |
|---|---|---|
| **CtrlTask** | ✅ | 主状态机、RFID 事件处理、WiFi 遥测调度、测量/掉头覆盖 |
| **DriverTask** | ✅ | 队列驱动电机，支持 6 种命令 |
| **SensorTask** | ✅ | 循迹 80ms、I2C 传感器 500ms、MQ8 ADC |
| **TrackCtrl** | ✅ | PID 双环循迹 + 6 模式状态机 |
| **ObstacleCtrl** | ✅ | 11 状态超声波避障 |
| **ThermalCtrl** | ✅ | 4 阶段紧急返回、降速、放行 |
| **HCSR04** | ✅ | 5 次采样裁剪平均、超时检测 |
| **RfidTask** | ✅ | RC522 SPI 驱动、IRQ + 轮询、3 次丢失判定 |
| **RfidReader** | ✅ | UID CRC8 压缩、8 位置映射、缓存接口 |
| **WifiComm** | ✅ | ESP8266 AP 模式、TCP Server、JSON 遥测/命令/Ack |
| **WifiTask** | ✅ | 20ms 循环、AT 初始化、链路保活 |
| **OledTask** | ✅ | I2C3 驱动、双页面 2s 切换 |
| **UartTask** | ✅ | 命令解析、ESP 桥接模式 |
| **LedTask** | ✅ | 500ms 心跳（无实际 LED 硬件） |
| **SelfTest** | ✅ | 8 项上电自检 |
| **motor** | ✅ | TB6612FNG 驱动、PWM 0~1000 |
| **pid** | ✅ | 增量式 PID、微分滤波、积分分离 |
| **BatteryCtrl** | 🟡 | 桩代码 — 固定 100%/12V，低电量返航未实现 |
| **Encoder** | 🟡 | 桩代码 — `BOARD_HAS_ENCODER=0` |
| **AHT20** | 🟡 | I2C2 地址已映射，传感器硬件未验证 |
| **DS18B20** | ❌ | 引脚 PA4 已预留，驱动未实现 |

---

## 19. 构建

### 19.1 编译命令

```powershell
cd D:\develop\Workplace\VscodeWorkplace\stm32\smart_car
cmake --build --preset Debug
```

### 19.2 输出

`build/Debug/smart_car.elf`

### 19.3 当前资源占用

| 区域 | 使用 | 总大小 | 占用比 |
|---|---|---|---|
| RAM | ~24,768 B | 128 KB | 18.9% |
| CCMRAM | 0 B | 64 KB | 0% |
| FLASH | ~79,144 B | 512 KB | 15.1% |

### 19.4 WiFi 连接步骤

```
1. 手机连接 WiFi: SmartCar_F407 / 12345678
2. 打开 TCP Client App，连接 192.168.4.1:8080
3. 小车将自动每 3s 发送遥测 JSON
4. 手机可发送命令（如 "start_patrol"）控制小车
```

---

*文档版本: 2026-06-17 | 对应源码 git HEAD*

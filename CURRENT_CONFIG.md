# smart_car 技术参考手册

> 基于 STM32F407VET6 的智能小车，FreeRTOS + HAL + CMake 构建。
> 本文件为完整技术参考，涵盖引脚映射、任务结构、状态机、通信协议等。

---

## 1. 平台规格

| 项目 | 值 |
|---|---|
| 主控 | STM32F407VET6 (Cortex-M4) |
| 系统主频 | 168 MHz |
| 外部晶振 | 25 MHz |
| HAL 时基 | TIM8 |
| RTOS | FreeRTOS + CMSIS-RTOS V2 |
| 工具链 | ARM GCC (cmake --preset Debug) |
| 输出文件 | build/Debug/smart_car.elf |

---

## 2. RTOS 任务

| 任务函数 | 名称 | 优先级 | 栈 (字×4) | 轮询/延时 | run_count 索引 |
|---|---|---|---|---|---|
| StartLedTask | `LedTask` | osPriorityLow | 1024 | 500ms | `[0]` |
| StartUartTask | `UartTask` | osPriorityLow | 1024 | 50ms | `[1]` |
| StartOledTask | `OledTask` | osPriorityLow | 1024 | 300ms | `[2]` |
| StartHCSR04Task | `HCSR04Task` | osPriorityRealtime | 1024 | 80ms | `[3]` |
| StartSensorTask | `SensorTask` | osPriorityBelowNormal1 | 1024 | 80ms | `[4]` |
| StartDriverTask | `DriverTask` | osPriorityAboveNormal | 1024 | 阻塞(queue, 100ticks) | `[5]` |
| StartCtrlTask | `CtrlTask` | osPriorityNormal | 2048 | 30ms | `[6]` |
| StartRfidTask | `myRfidTask` | osPriorityLow | 1024 | 200ms (带 IRQ 唤醒) | `[7]` |
| StartWifiTask | `WifiTask` | osPriorityLow | 2048 | 20ms | `[8]` |

---

## 3. FreeRTOS 资源

### 3.1 消息队列

| 队列句柄 | 消息类型 | 大小 | 深度 | 发送者 | 接收者 |
|---|---|---|---|---|---|
| `TrackHandle` | uint8_t | 16 | — | SensorTask | — |
| `MotorActionHandle` | MotorCmd_t (12 字节) | 16 | — | CtrlTask | DriverTask |
| `LEDFlashHandle` | uint32_t | 16 | — | LedTask | — |
| `DistanceHandle` | float | 16 | — | — | — |
| `DriverPWMHandle` | uint32_t | 16 | — | — | — |

### 3.2 互斥锁

| 句柄 | 用途 |
|---|---|
| `I2CMutexHandle` | 保护 I2C1/2 总线访问（MLX90614 + AHT20） |

### 3.3 事件标志

| 标志 | 用途 |
|---|---|
| `RFID_FLAG_IRQ (0x01)` | RFID EXTI 中断唤醒 RfidTask |

---

## 4. 硬件引脚映射

### 4.1 串口

| 外设 | TX | RX | 波特率 | 用途 |
|---|---|---|---|---|
| USART2 | PA2 | PA3 | 115200 8N1 | 调试串口、命令输入 |
| USART3 | PC10 | PC11 | 115200 8N1 | ESP8266 WiFi 模块 |

### 4.2 I2C

| 外设 | SCL | SDA | 速率 | 设备 |
|---|---|---|---|---|
| I2C1 | PB6 | PB7 | — | MLX90614 (0xB4) |
| I2C2 | PB10 | PB11 | — | AHT20 |
| I2C3 | PA8 | PC9 | Fast Mode | OLED (0x78) |

### 4.3 SPI

| 外设 | SCK | MOSI | MISO | CS | 用途 |
|---|---|---|---|---|---|
| SPI1 | PA5 | PA7 | PA6 | PD3 (软件 NSS) | RC522 RFID |

SPI 模式：Master, 8bit, CPOL=0/CPHA=0 (Mode 0)
波特率：CubeMX 初始化 2 (84MHz)，RFID 初始化时重配为 16 (~10.5MHz)

### 4.4 ADC

| 外设 | 引脚 | 通道 | 用途 |
|---|---|---|---|
| ADC1 | PC1 | ADC_CHANNEL_11 | MQ-8 气体传感器模拟量 |

### 4.5 定时器

| 定时器 | 用途 | 配置 |
|---|---|---|
| TIM1 | 超声波微秒计时基准 | 预分频 167 → 1MHz |
| TIM3 | 右电机 PWM | — |
| TIM4 | 左电机 PWM (CH2=左, CH3=右) | — |
| TIM8 | HAL 时基 | — |

### 4.6 GPIO

#### 电机驱动 (TB6612FNG)

| 信号 | 引脚 | 方向 |
|---|---|---|
| AIN1_L | — | 左电机方向 1 |
| AIN2_L | — | 左电机方向 2 |
| BIN1_L | — | 左电机方向 3 |
| BIN2_L | — | 左电机方向 4 |
| AIN1_R | — | 右电机方向 1 |
| AIN2_R | — | 右电机方向 2 |
| BIN1_R | — | 右电机方向 3 |
| BIN2_R | — | 右电机方向 4 |
| STBY_L | — | 左电机待机(使能) |
| STBY_R | — | 右电机待机(使能) |
| PWM_L | TIM4 CH2 (PD13) | 左电机 PWM |
| PWM_R | TIM4 CH3 (PD14) | 右电机 PWM |

PWM 最大值：1000

#### 其他 GPIO

| 功能 | 引脚 | 类型 | 备注 |
|---|---|---|---|
| RFID CS (SDA) | PD3 | 输出 | RC522 片选，低电平有效 |
| RFID IRQ | PD7 | 输入(EXTI 上升沿) | RC522 中断请求 |
| 蜂鸣器 | PE5 | 输出 | 低电平有效 |
| 超声波 Trig | PB8 | 输出 | HC-SR04 触发脉冲 |
| 超声波 Echo | PB9 | 输入 | HC-SR04 回波脉冲 |
| MQ-8 DO | PC13 | 输入 | MQ-8 数字输出 |
| DS18B20 DQ | PA4 | — | 预留 |

#### 循迹传感器 (反射式红外)

| 传感器 | 引脚 | 逻辑 |
|---|---|---|
| X1 (S1, 最右) | PA0 | HAL_GPIO_PIN_SET = 检测到黑线 |
| X2 (S2) | PA1 | 同上 |
| X3 (S3) | PD1 | 同上 |
| X4 (S4, 最左) | PD2 | 同上 |

组合值：`(X4<<3) | (X3<<2) | (X2<<1) | X1` → 4-bit 值 0-15

---

## 5. 电机控制

### 5.1 方向逻辑

| 方向 | 左电机 (MOTOR_LEFT) | 右电机 (MOTOR_RIGHT) |
|---|---|---|
| 前进 (MOTOR_FORWARD) | AIN1=H, AIN2=L, BIN1=H, BIN2=L | BIN1=H, BIN2=L, AIN1=H, AIN2=L |
| 后退 (MOTOR_BACKWARD) | AIN1=L, AIN2=H, BIN1=L, BIN2=H | BIN1=L, BIN2=H, AIN1=L, AIN2=H |
| 停止 | 刹车模式(AII=H, PWM=0) | 刹车模式(AII=H, PWM=0) |

### 5.2 命令 → 电机动作 (DriverTask)

| 命令 | 左方向 | 右方向 | 左速度 | 右速度 |
|---|---|---|---|---|
| MOTOR_CMD_STOP | stop | stop | 0 | 0 |
| MOTOR_CMD_FORWARD | forward | forward | pwm_left | pwm_right |
| MOTOR_CMD_TURN_LEFT | forward | forward | pwm / 3 | pwm |
| MOTOR_CMD_TURN_RIGHT | forward | forward | pwm | pwm / 3 |
| MOTOR_CMD_SPIN_LEFT | backward | forward | pwm | pwm |
| MOTOR_CMD_SPIN_RIGHT | forward | backward | pwm | pwm |

---

## 6. 传感器子系统

### 6.1 HC-SR04 超声波

- 每次触发 5 次采样，间隔 80ms
- 触发脉冲：15us 高电平
- 有效脉冲范围：117us (2cm) ~ 23529us (400cm)
- 距离计算：`pulse_us * 0.017f`
- 超时阈值：30000us
- 输出：裁剪平均值（去除最高/最低后取平均）
- 全局变量：`g_distance`

### 6.2 循迹传感器

- 4 路反射式红外传感器
- 通过 `TrackHandle` 队列由 SensorTask 发送
- 传感器布局：`S1(X1) S2(X2) S3(X3) S4(X4)` — 从右到左
- 详见第 9 章循迹控制

### 6.3 MLX90614 (I2C1)

- 地址：`0xB4` (7-bit: 0x5A)
- 寄存器：`0x06` = 环境温度, `0x07` = 物体温度
- 温度转换：`temp_c = (raw * 0.02f) - 273.15f`
- 重试：3 次，间隔 10ms
- 使用 `I2CMutexHandle` 互斥保护

### 6.4 AHT20 (I2C2)

- 全局变量：`g_aht20_temp`, `g_aht20_humidity`
- 更新间隔：500ms

### 6.5 MQ-8 气体传感器

- 模拟量 (ADC1): 全局 `g_mq8_adc_raw`
- 数字量 (PC13): 全局 `g_mq8_do`

### 6.6 温度阈值

| 阈值 | 值 | 对应状态 |
|---|---|---|
| THERMAL_ALERT_THRESHOLD | ≥ 29.0°C | STATE_THERMAL_ALERT (temp_warning) |
| THERMAL_WARNING_THRESHOLD | ≥ 30.0°C | STATE_THERMAL_WARNING (temp_alarm) |
| THERMAL_EMERGENCY_THRESHOLD | ≥ 31.0°C | STATE_EMERGENCY (evacuate) |
| OBS_DETECT_DIST | ≤ 30.0 cm | 触发避障 |

---

## 7. RFID-RC522 子系统

### 7.1 引脚连接

| RC522 | STM32 | 功能 |
|---|---|---|
| SCK | PA5 | SPI1_SCK (~10.5MHz) |
| MOSI | PA7 | SPI1_MOSI |
| MISO | PA6 | SPI1_MISO |
| SDA (CS) | PD3 | 片选，低电平有效 (软件 NSS) |
| IRQ | PD7 | EXTI 上升沿中断 |
| 3.3V | 3.3V | 供电 |

### 7.2 标签 ID 映射

| 压缩 ID | 位置名 | 行为 |
|---|---|---|
| 58 | `start` | 调用 `Ctrl_Start()` 开始巡逻 |
| 53 | `place_1` | 停车测量 3 秒 → 发送遥测 → 继续巡逻 |
| 78 | `place_2` | 同上 |
| 199 | `place_3` | 同上 |
| 95 | `place_4` | 同上 |
| 86 | `place_5` | 同上 |
| 111 | `place_6` | 同上 |
| 228 | `end_stop` | 原地右旋 1.5 秒 → 继续（返航回桩） |
| 其他 | `unknown` | 忽略 |

### 7.3 UID 压缩

使用 CRC8 类算法将 4 字节 UID 压缩为 1 字节 ID：
- 循环移位 + 逐字节异或
- 如果结果为零，回退到 `uid[0]`（若仍为零则置 1）

### 7.4 读取机制

- RfidTask 轮询（200ms）+ EXTI 中断（PD7 上升沿）
- 连续 3 次读取不到标签 → 视为离开
- 串口调试输出（USART2）：

```
[RFID] NEW TAG  UID=90:7C:A6:02  id=58 loc=start
[CTRL] RFID id=58 loc=start
[CTRL] -> START patrolling
```

### 7.5 RFID 位置缓存

检测到新标签时，`Ctrl_HandleRfidEvent` 立即调用 `Wifi_UpdateRfidLocation(loc)` 更新缓存。WiFi 遥测时直接读取缓存，不受限于当前标签是否仍在读取范围内。

---

## 8. WiFi 通信协议

### 8.1 连接参数

| 参数 | 值 |
|---|---|
| 模块 | ESP8266 (ESP-01S) |
| 模式 | AP (Soft AP) |
| SSID | `SmartCar_F407` |
| 密码 | `12345678` |
| TCP 端口 | 8080 |
| IP 地址 | 192.168.4.1 |
| 物理 UART | USART3 (PC10/PC11) |

### 8.2 小车 → 手机 (遥测)

**自动发送间隔**：3 秒（WiFi 链路存活检测触发 `Wifi_SendLiveTelemetry`）

格式：

```json
{"type":"telemetry","MQ8":<xx.x>,"AHT_temp":<xx.x>,"AHT_hum":<xx.x>,"dist":<xx.x>,"track":<0-15>,"track_bin":"<4位二进制>","rfid_loc":"<位置名>","state":"<状态名>"}
```

字段：

| 字段 | 类型 | 说明 |
|---|---|---|
| `type` | string | 固定 `"telemetry"` |
| `MQ8` | float | MQ-8 ADC 值 (1 位小数) |
| `AHT_temp` | float | 温度 °C (1 位小数) |
| `AHT_hum` | float | 湿度 % (1 位小数) |
| `dist` | float | 超声波距离 cm (1 位小数) |
| `track` | int | 循迹 4-bit 值 0-15 |
| `track_bin` | string | 循迹二进制字符串，如 `"1010"` |
| `rfid_loc` | string | RFID 位置名，缓存最近非 `unknown` 的值 |
| `state` | string | 系统状态名 |

`rfid_loc` 缓存机制：当 RFID 检测到新标签时，`CtrlTask` 通过 `Wifi_UpdateRfidLocation()` 立即推送位置名到 WiFi 模块。无论 WiFi 何时发送遥测，`rfid_loc` 始终是最新检测到的标签位置，不会因标签已移出读取范围而丢失。

### 8.3 手机 → 小车 (命令 & Ack)

手机发送命令字符串（TCP payload），小车回复 Ack JSON：

| 命令 | 小车动作 | Ack result |
|---|---|---|
| `idle` | `Ctrl_Stop()` | `"idle"` |
| `start_patrol` | `Ctrl_Start()` | `"start_patrol"` |
| `temp_warning` | 仅回应 | `"temp_warning"` |
| `temp_alarm` | 仅回应 | `"temp_alarm"` |
| `evacuate` | `Ctrl_RequestEmergency()` | `"evacuate"` |
| `return_home` | `Ctrl_Start()` | `"return_home"` |
| `emergency_stop` | `Ctrl_Stop()` | `"evacuate"` |
| `pause` | `Ctrl_Stop()` | `"idle"` |
| `manual_reset` | 忽略 | `"ignored"` |
| `start` | `Ctrl_Start()` (兼容旧名) | `"start_patrol"` |
| `stop` | `Ctrl_Stop()` (兼容旧名) | `"idle"` |
| `ping` / `hello` | 心跳回应 | `"pong"` |
| `status` | 查询当前状态 | `"<当前状态名>"` |

Ack 格式：

```json
{"type":"ack","cmd":"<收到的命令>","result":"<结果>"}
```

未识别的命令：`{"type":"ack","cmd":"unknown","result":"ignored"}`

### 8.4 ESP8266 初始化流程

1. `AT` — 测试连接
2. `ATE0` — 关闭回显
3. `AT+CWMODE=2` — Soft AP 模式
4. `AT+CWSAP="SmartCar_F407","12345678",5,3` — 配置 AP
5. `AT+CIPMUX=1` — 多连接
6. `AT+CIPSERVER=1,8080` — 启动 TCP Server
7. `AT+CIPSTO=0` — 超时设为 0
8. `AT+CIFSR` — 获取 IP

### 8.5 桥接模式

通过调试串口 (USART2) 发送 `esp` / `esp_on` / `at` 进入桥接模式，直接将 USART2 数据透传到 USART3 (ESP8266 AT 指令)。发送 `esp_off` 或 `+++` 退出。

---

## 9. 状态机

### 9.1 状态定义

```
STATE_STANDBY (0)          → "idle"
STATE_PATROL (1)           → "start_patrol"
STATE_THERMAL_ALERT (2)    → "temp_warning"   (≥29°C)
STATE_THERMAL_WARNING (3)  → "temp_alarm"     (≥30°C)
STATE_EMERGENCY (4)        → "evacuate"       (≥31°C)
STATE_LOW_BATTERY (5)      → "return_home"    (终点 RFID 触发)
```

### 9.2 优先级

`Ctrl_DetermineState` 优先级（高 → 低）：

1. `STATE_LOW_BATTERY` — 终点 RFID 掉头（最高优先级）
2. `STATE_STANDBY` — 手动覆盖 / 未启动
3. `STATE_EMERGENCY` — 紧急撤离
4. `STATE_THERMAL_WARNING` — 温度报警
5. `STATE_THERMAL_ALERT` — 温度预警
6. `STATE_PATROL` — 正常巡逻（最低）

### 9.3 温度状态防抖

从任意温度状态回到 `STATE_PATROL` 时，强制保持当前状态 5 秒：

```
温度 ≥ 30°C → STATE_THERMAL_WARNING
温度 ≥ 31°C → STATE_EMERGENCY (立即升级)
温度 < 29°C → 仍保持原状态 5 秒 → 回到 STATE_PATROL
```

预警/报警之间升级无延迟。

### 9.4 OLED 显示映射

```
STATE_STANDBY          → "IDLE"
STATE_PATROL           → "PATROL"
STATE_THERMAL_ALERT    → "T_WARN"
STATE_THERMAL_WARNING  → "T_ALARM"
STATE_EMERGENCY        → "EVACUATE"
STATE_LOW_BATTERY      → "RET_HOME"
```

### 9.5 状态触发与动作

| 触发条件 | 状态 | 动作 |
|---|---|---|
| RFID start tag | → PATROL | `Ctrl_Start()` |
| RFID place_N tag | → 停车 3s 测量 | `MOTOR_CMD_STOP` → 发送遥测 |
| RFID end_stop tag | → 掉头 1.5s | `MOTOR_CMD_SPIN_RIGHT`, PWM=1400 |
| 温度 < 29°C | → PATROL | 正常循迹 + 避障 |
| 温度 ≥ 29°C | → THERMAL_ALERT | 循迹 + 避障 (蜂鸣器关) |
| 温度 ≥ 30°C | → THERMAL_WARNING | 循迹 + 避障 (蜂鸣器开, 速度 50%) |
| 温度 ≥ 31°C | → EMERGENCY | 紧急返回状态机 (蜂鸣器开, LED 闪烁) |
| `stop` 命令 | → STANDBY | 刹车停止 |

### 9.6 ThermalCtrl 紧急返回状态机

```
RETURN_IDLE(0) → RETURN_SPIN_180(1) → RETURN_FIND_LINE(2) → RETURN_FOLLOW_LINE(3) → RETURN_DONE(4)
```

| 状态 | PWM | 退出条件 |
|---|---|---|
| RETURN_SPIN_180 | SPIN_RIGHT, 700 | 53 个控制周期 (~1.6s) |
| RETURN_FIND_LINE | SPIN_RIGHT, 420 | 检测到轨道线 或 60 周期超时 |
| RETURN_FOLLOW_LINE | TrackCtrl_Run | 障碍物或完成 |
| RETURN_DONE | STOP | — |

---

## 10. 循迹控制 (PID)

### 10.1 传感器误差查表

| 传感器位模式 (S4..S1) | 误差 | 含义 |
|---|---|---|
| `0110` (0x06) | 0 | 居中：S2, S3 |
| `1001` (0x09) | 0 | 居中：S1, S4 |
| `0101` (0x05) | 0 | 居中：S1, S3 |
| `1010` (0x0A) | 0 | 居中：S2, S4 |
| `0010` (0x02) | -1 | 偏左：S2 |
| `0001` (0x01) | -2 | 显著偏左：S1 |
| `0011` (0x03) | -2 | 显著偏左：S1, S2 |
| `1101` (0x0D) | -3 | 严重偏左 |
| `1110` (0x0E) | -3 | 严重偏左 |
| `0100` (0x04) | 1 | 偏右：S3 |
| `1000` (0x08) | 2 | 显著偏右：S4 |
| `1100` (0x0C) | 2 | 显著偏右：S3, S4 |
| `1011` (0x0B) | 3 | 严重偏右 |
| `0000` (0x00) | last_error × 1.8 | 丢线：增益放大误差 |
| `1111` (0x0F) | 0 | 十字路口：直行 |

### 10.2 循迹模式状态机

```
FOLLOW(0) ↔ CROSS(1) ↔ SHARP_LEFT(2) / SHARP_RIGHT(3) ↔ SEARCH_LEFT(4) / SEARCH_RIGHT(5)
```

| 模式 | 进入条件 | 行为 | 退出条件 |
|---|---|---|---|
| FOLLOW | 默认 | PID 转向 | 十字路口 / 急弯 / 丢线 |
| CROSS | track == 0x0F | 直行 8 个周期 | 切回 FOLLOW |
| SHARP_LEFT | 误差 < -1 且模式为 01/03/0D/0E | 左转 18 个周期，PWM=520 | 切回 FOLLOW 或 SEARCH |
| SHARP_RIGHT | 误差 > 1 且模式为 08/0B/0C | 右转 18 个周期，PWM=520 | 切回 FOLLOW 或 SEARCH |
| SEARCH_LEFT | 急弯左后仍丢线 | 原地左旋，PWM=430，50 周期超时 | 找到线 → FOLLOW |
| SEARCH_RIGHT | 急弯右后仍丢线 | 原地右旋，PWM=430，50 周期超时 | 找到线 → FOLLOW |

### 10.3 PID 参数

#### 速度 PID

| 参数 | 值 |
|---|---|
| 目标值 | 260.0 |
| Kp | 2.0 |
| Ki | 0.4 |
| Kd | 0.0 |
| 输出限幅 | 0.0 ~ 600.0 |
| 积分限幅 | -120.0 ~ 120.0 |

#### 转向 PID

| 参数 | 值 |
|---|---|
| 目标值 | 0.0 |
| Kp | 95.0 |
| Ki | 8.0 |
| Kd | 20.0 |
| 输出限幅 | -320.0 ~ 320.0 |
| 积分限幅 | -80.0 ~ 80.0 |
| 微分滤波 alpha | 0.28 |

### 10.4 循迹 PWM 限幅

| 常量 | 值 |
|---|---|
| TRACK_PWM_MIN | 60 |
| TRACK_PWM_MAX | 620 |
| TRACK_LOST_GAIN | 1.8 |
| TRACK_CROSS_SPEED | 230 |
| TRACK_SHARP_TURN_PWM | 520 |
| TRACK_SEARCH_PWM | 430 |

---

## 11. 避障控制

### 11.1 触发条件

障碍物距离 ≤ 30.0 cm (OBS_DETECT_DIST)

### 11.2 状态机 (11 状态)

```
IDLE(0) → BRAKE(1) → SCAN_LEFT_TURN(2) → SCAN_LEFT_SAMPLE(3) → SCAN_LEFT_RETURN(4) → SCAN_RIGHT_TURN(5) → SCAN_RIGHT_SAMPLE(6) → SCAN_RIGHT_RETURN(7) → TURN_CHOICE(8) → ADVANCE(9) → FIND_LINE(10)
```

### 11.3 关键时序

| 阶段 | PWM | 周期数 | 说明 |
|---|---|---|---|
| BRAKE | STOP | 8 | 刹车 |
| 左/右扫描旋转 | SPIN, 620 | 18 | 旋转 90° |
| 左/右采样 | STOP | 14 | 测量距离 |
| 左/右回位 | SPIN_R/L, 620 | 18 | 回到原方向 |
| TURN_CHOICE | 选较远侧旋转 | 27 | 转 90° 到选定方向 |
| ADVANCE | 弧线前进 280/196 | 18~70 | 绕过障碍 |
| FIND_LINE | 反向旋转 420 | 50 超时 | 找回轨道 |

### 11.4 绕行方向

```
obs_bypass_dir = (左距离 >= 右距离) ? -1 : 1
```

弧线前进 PWM：
- 向左绕：`pwm_left = 196, pwm_right = 280`
- 向右绕：`pwm_left = 280, pwm_right = 196`

总超时：420 控制周期

---

## 12. OLED 显示

### 12.1 页面切换

每 2 秒切换一次页面。

### 12.2 页面 0

```
ST:<状态>
DIS:<距离>cm
MQ8:<ADC> D:<0/1>
T:<温度> H:<湿度>
```

### 12.3 页面 1

```
AHT:<温度> H:<湿度>
TRACK:<bit3><bit2><bit1><bit0>
(空)
(空)
```

---

## 13. 模块实现状态

| 模块 | 状态 | 说明 |
|---|---|---|
| CtrlTask | ✅ | 控制系统状态机、RFID 事件、WiFi 遥测 |
| DriverTask | ✅ | 电机命令队列执行 |
| SensorTask | ✅ | 传感器采集 (ADC, I2C, 循迹) |
| TrackCtrl | ✅ | PID 循迹控制 |
| ObstacleCtrl | ✅ | 11 状态避障 |
| ThermalCtrl | ✅ | 温度状态机 + 紧急返回 |
| HCSR04 | ✅ | 超声波测距 (TIM1 + GPIO) |
| RfidTask + RfidReader | ✅ | RC522 驱动 + 标签映射 |
| WifiComm + WifiTask | ✅ | ESP8266, 遥测 3s, 命令 + Ack |
| OledTask | ✅ | 双页面循环显示 |
| UartTask | ✅ | 命令解析 + ESP 桥接 |
| LedTask | ✅ | 500ms 心跳 (无实际 LED) |
| SelfTest | ✅ | 8 项上电自检 |
| motor | ✅ | TB6612FNG 驱动 |
| pid | ✅ | 增量式 PID |
| BatteryCtrl | 🟡 | 固定返回值，等待硬件接入 |
| Encoder | 🟡 | `BOARD_HAS_ENCODER=0`，桩代码 |
| AHT20 | 🟡 | I2C2 已映射，传感器未验证 |

---

## 14. 构建

```powershell
cmake --build --preset Debug
```

产物：`build/Debug/smart_car.elf`

当前资源占用：

| 区域 | 使用 | 总大小 | 占用比 |
|---|---|---|---|
| RAM | ~24.8 KB | 128 KB | 18.9% |
| CCMRAM | 0 B | 64 KB | 0% |
| FLASH | ~79.1 KB | 512 KB | 15.1% |

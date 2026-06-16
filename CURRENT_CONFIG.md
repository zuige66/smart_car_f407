# smart_car 当前配置说明

## 1. 当前状态

`clion_car` 到 `smart_car` 的核心迁移已经完成，当前工程可以正常交叉编译通过。

已验证命令：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

已生成：

- `build/Debug/smart_car.elf`

## 2. 芯片与时钟

- 主控：`STM32F407VET6`
- 内核：`Cortex-M4`
- 系统主频：`168 MHz`
- 外部晶振：`25 MHz`
- HAL 时基：`TIM8`
- 系统框架：`FreeRTOS + CMSIS-RTOS V2`

## 3. 当前 CubeMX 外设配置

### 3.1 串口

- `USART2`
  - `PA2`：TX
  - `PA3`：RX
  - 当前用于调试串口、命令输入

- `USART3`
  - `PC10`：TX
  - `PC11`：RX
  - 当前预留给 WiFi

### 3.2 I2C

- `I2C1`
  - `PB6`：SCL
  - `PB7`：SDA
  - 当前用于 `MLX90614`

- `I2C2`
  - `PB10`：SCL
  - `PB11`：SDA
  - 当前预留给 `AHT20`

- `I2C3`
  - `PA8`：SCL
  - `PC9`：SDA
  - 当前用于 `OLED`

### 3.3 ADC

- `ADC1`
  - `PC1`：`MG8_AO`
  - 通道：`ADC_CHANNEL_11`
  - 当前已在 `SensorTask` 中接入读取

### 3.4 定时器

- `TIM4`
  - `PD13`：PWM CH2
  - `PD14`：PWM CH3
  - 当前用于电机 PWM 输出

- `TIM1`
  - 内部时钟
  - 预分频：`167`
  - 当前计数频率：`1 MHz`
  - 当前在软件中作为超声测距的微秒计时基准

说明：

- `TIM1_CC_IRQn` 目前在 CubeMX 中已经打开。
- 但由于当前 `Echo` 在 `PB9`，并没有直接作为 `TIM1` 输入捕获引脚接入，所以当前超声实现采用的是：
  - `GPIO 轮询`
  - `TIM1 微秒计时`
- 不是旧工程里的输入捕获方案。

## 4. 当前软件使用的引脚映射

### 4.1 传感器

- 循迹
  - `X1 = PA0`
  - `X2 = PA1`
  - `X3 = PD1`
  - `X4 = PD2`

- 超声
  - `Trig = PB8`
  - `Echo = PB9`

- MQ8
  - `DO = PC13`
  - `AO = PC1`

### 4.2 其他输出

- `BUZZER_IN = PE5`
- `RC522_SDA = PD3`
- `DS18B20_DQ = PA4`

## 5. 已经迁移并接入的模块

以下模块已经接入到 `smart_car`：

- `CtrlTask`
- `DriverTask`
- `SensorTask`
- `TrackCtrl`
- `ObstacleCtrl`
- `ThermalCtrl`
- `HCSR04`
- `UartTask`
- `OledTask`
- `LedTask`
- `motor`
- `pid`
- `oled/font`

以下生成文件也已按迁移结果同步修改：

- `Core/Src/freertos.c`
- `Core/Src/stm32f4xx_it.c`
- `Core/Inc/stm32f4xx_it.h`
- `Core/Src/stm32f4xx_hal_timebase_tim.c`
- `Core/Src/gpio.c`
- `CMakeLists.txt`
- `smart_car.ioc`

## 6. 当前为占位实现的模块

- `BatteryCtrl`
  - 当前返回固定电量、电压
  - 低电量返航逻辑未实现

- `Encoder`
  - 当前仍为桩实现
  - `BOARD_HAS_ENCODER = 0`
  - 目前循迹控制没有真实速度闭环反馈

- `LedTask`
  - 当前 `BOARD_HAS_STATUS_LED = 0`
  - 软件任务保留，但没有映射实际状态灯

## 7. 状态机

### 7.1 状态列表

| 代码 enum | WiFi JSON state | OLED 显示 | 触发条件 |
|---|---|---|---|
| `STATE_STANDBY` | `idle` | `IDLE` | 未启动 / `stop` 命令 |
| `STATE_PATROL` | `start_patrol` | `PATROL` | 已启动，温度 < 29°C |
| `STATE_THERMAL_ALERT` | `temp_warning` | `T_WARN` | 温度 ≥ 29°C |
| `STATE_THERMAL_WARNING` | `temp_alarm` | `T_ALARM` | 温度 ≥ 30°C |
| `STATE_EMERGENCY` | `evacuate` | `EVACUATE` | 温度 ≥ 31°C |
| `STATE_LOW_BATTERY` | `return_home` | `RET_HOME` | 终点 RFID 触发（掉头 1.5s） |

### 7.2 温度阈值防抖

任意预警状态（T_WARN / T_ALARM / EVACUATE）回到 `start_patrol` 前，强制保持 **5 秒**：

```
温度 30°C → state = "temp_alarm"
温度 24°C → state = "temp_alarm"  (保持5秒)
          → state = "start_patrol" (5秒后)
```

预警之间升级/降级无延迟（如 T_WARN → T_ALARM 立即切换）。

## 8. WiFi 通信协议（USART3 → ESP8266）

### 8.1 连接信息

- SSID: `SmartCar_F407`
- 密码: `12345678`
- TCP 端口: `8080`
- IP: `192.168.4.1`

### 8.2 小车 → 上位机（遥测）

每 3 秒自动发送一次，格式：

```json
{"type":"telemetry","MQ8":<xx.x>,"AHT_temp":<xx.x>,"AHT_hum":<xx.x>,"dist":<xx.x>,"track":<0-15>,"track_bin":"<4位二进制>","rfid_loc":"<位置名>","state":"<状态名>"}
```

字段说明：

| 字段 | 说明 |
|---|---|
| `MQ8` | 气体传感器 ADC 值 |
| `AHT_temp` | 温度 |
| `AHT_hum` | 湿度 |
| `dist` | 超声波距离 cm |
| `track` | 循迹传感器值 0-15 |
| `track_bin` | 循迹二进制串，如 `"1010"` |
| `rfid_loc` | RFID 当前位置名 |
| `state` | 小车当前状态 |

`rfid_loc` 会缓存最近一次非 `unknown` 的值，防止采样间隙漏掉点位。

### 8.3 上位机 → 小车（命令 & Ack）

上位机发送命令字符串，小车回复 Ack JSON：

| 发送命令 | 小车动作 | Ack JSON `result` |
|---|---|---|
| `idle` | 停车 | `"idle"` |
| `start_patrol` | 开始巡检 | `"start_patrol"` |
| `pause` | 停车 | `"idle"` |
| `stop` | 停车（兼容旧名） | `"idle"` |
| `start` | 开始巡检（兼容旧名） | `"start_patrol"` |
| `emergency_stop` | 紧急停车 | `"evacuate"` |
| `evacuate` | 紧急撤离 | `"evacuate"` |
| `return_home` | 开始巡检 | `"return_home"` |
| `temp_warning` | 仅回应（无动作） | `"temp_warning"` |
| `temp_alarm` | 仅回应（无动作） | `"temp_alarm"` |
| `manual_reset` | 仅回应 | `"ignored"` |
| `ping` / `hello` | 心跳 | `"pong"` |
| `status` | 查询当前状态 | `"<当前状态名>"` |

Ack 格式：

```json
{"type":"ack","cmd":"<收到的命令>","result":"<结果>"}
```

未识别的命令返回 `{"type":"ack","cmd":"unknown","result":"ignored"}`。

## 9. RFID-RC522

### 9.1 引脚连接

| RC522 | STM32 | 功能 |
|---|---|---|
| SCK | PA5 | SPI1_SCK |
| MOSI | PA7 | SPI1_MOSI |
| MISO | PA6 | SPI1_MISO |
| SDA (CS) | PD3 | 片选（低电平有效） |
| IRQ | PD7 | EXTI 上升沿中断 |
| 3.3V | 3.3V | 供电 |
| GND | GND | 地 |

### 9.2 SPI 配置

- SPI1, Master, 8bit, CPOL=0/CPHA=0 (Mode 0)
- 波特率预分频：初始化 2（84MHz），RFID 初始化时重配为 16（~10.5MHz）
- 软件 NSS（PD3 GPIO 控制片选）

### 9.3 标签 ID 映射

| 压缩 ID | 位置名 | 行为 |
|---|---|---|
| 58 | `start` | 开始巡检（`Ctrl_Start()`） |
| 53 | `place_1` | 停车测数据 3 秒 → 发送遥测 → 继续 |
| 78 | `place_2` | 同上 |
| 199 | `place_3` | 同上 |
| 95 | `place_4` | 同上 |
| 86 | `place_5` | 同上 |
| 111 | `place_6` | 同上 |
| 228 | `end_stop` | 掉头 1.5s → 继续巡检（返航回桩） |

### 9.4 串口调试输出

```
[RFID] NEW TAG  UID=90:7C:A6:02  id=58 loc=start
[CTRL] RFID id=58 loc=start
[CTRL] -> START patrolling

[RFID] NEW TAG  UID=70:7E:A6:02  id=53 loc=place_1
[CTRL] RFID id=53 loc=place_1
[CTRL] -> MEASURE 3s at place_1
[CTRL] Measuring 1/3s  dist=12.3 temp=28.5
[CTRL] Measure done, send data via WiFi

[RFID] NEW TAG  UID=60:80:A6:02  id=228 loc=end_stop
[CTRL] RFID id=228 loc=end_stop
[CTRL] -> RETURN HOME spin 180
[CTRL] Return home done, resume patrol
```

### 9.5 读取机制

- FreeRTOS 任务 `myRfidTask`（优先级 Low，栈 1024B）
- 轮询间隔 200ms + EXTI 中断（PD7 上升沿）触发快速响应
- 连续 3 次读不到标签 → 视为标签已离开

## 7. 最后一轮检查结论

### 7.1 已确认完成

- 应用层代码已全部位于 `smart_car/Core/App/Tasks`
- `OLED` 已从旧工程风格适配到当前 `I2C3`
- `MG8_AO` 已真实接入 `ADC1`
- `MG8_DO` 已在源码和 `.ioc` 中改为输入
- `TIM8` 时基中断配置已修正
- ARM GCC 交叉编译已通过

### 7.2 仍需你上板确认的点

最需要确认的是电机方向引脚映射。

当前软件采用：

- 左电机方向：`AIN1_L + AIN2_L`
- 右电机方向：`AIN1_R + AIN2_R`
- 使能：`STBY_L + STBY_R`
- PWM：`TIM4 CH2 + TIM4 CH3`

但你当前工程中还存在这些命名：

- `BIN1_L`
- `BIN2_L`
- `BIN2_R`

所以从软件角度看，当前最可能出现的实际上板问题是：

- 能编译
- 主流程能跑
- 但电机方向或左右轮行为不符合真实接线

如果上板后电机行为异常，优先检查：

- [motor.c](/D:/develop/Workplace/VscodeWorkplace/stm32/smart_car/Core/App/Tasks/motor.c)

### 7.3 当前工程的实际意义

现在这个版本已经可以认为是“迁移后的第一版可运行框架”：

- 主任务框架已经齐全
- 编译链路已经打通
- 串口、OLED、循迹、MQ8、超声主链路已经接好
- 剩余问题主要是个别板级细节和未完成功能模块

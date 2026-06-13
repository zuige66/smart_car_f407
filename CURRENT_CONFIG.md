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

## 6. 当前仍为占位实现的模块

下面这些模块目前能编译，但还不是完整硬件实现：

- `BatteryCtrl`
  - 当前返回固定电量、电压
  - 低电量返航逻辑未实现

- `WifiComm`
  - WiFi 初始化、上报、告警、命令接收未实现

- `RfidReader`
  - RFID 检测与读卡未实现

- `Encoder`
  - 当前仍为桩实现
  - `BOARD_HAS_ENCODER = 0`
  - 目前循迹控制没有真实速度闭环反馈

- `LedTask`
  - 当前 `BOARD_HAS_STATUS_LED = 0`
  - 软件任务保留，但没有映射实际状态灯

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

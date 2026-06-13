# smart_car_f407

基于 `STM32F407VET6` 的智能小车控制工程，使用 `STM32CubeMX + HAL + FreeRTOS + CMake` 开发。  
本项目由旧工程 `clion_car (STM32F103)` 迁移到新工程 `smart_car (STM32F407)`，并按当前硬件重新适配了引脚、外设和任务结构。

## 1. 项目概况

- 主控：`STM32F407VET6`
- 外部晶振：`8 MHz`
- 系统主频：`168 MHz`
- 开发方式：`STM32CubeMX + CMake`
- RTOS：`FreeRTOS CMSIS V2`

## 2. 当前功能

- OLED 状态显示
- 串口调试通信 `USART2`
- WiFi 通信任务 `USART3`
- RFID 识别任务 `SPI1 + EXTI7`
- 超声波测距 `TIM1`
- 循迹传感器采集
- MLX90614 红外测温
- MQ8 / MG8 气体传感器采集
- 电机驱动与控制任务

## 3. 主要外设分配

- `USART2`
  - 调试串口
  - `PA2 -> TX`
  - `PA3 -> RX`
- `USART3`
  - WiFi 模块串口
  - `PC10 -> TX`
  - `PC11 -> RX`
- `I2C1`
  - `MLX90614`
- `I2C2`
  - 预留 / AHT20
- `I2C3`
  - OLED
  - 当前配置为 `Fast Mode`
- `SPI1`
  - RFID / RC522
  - `PA5 -> SCK`
  - `PA6 -> MISO`
  - `PA7 -> MOSI`
- `PD3`
  - RFID 片选 `SDA/CS`
- `PD7`
  - RFID 中断 `EXTI7`
- `PC1`
  - `MG8_AO / ADC1_IN11`

## 4. 任务结构

- `UartTask`
  - 调试串口接收与命令解析
- `OledTask`
  - OLED 周期刷新显示
- `SensorTask`
  - 传感器采集
- `HCSR04Task`
  - 超声波测距
- `DriverTask`
  - 电机动作输出
- `CtrlTask`
  - 状态机与控制逻辑
- `RfidTask`
  - RFID 轮询 / 中断处理
- `WifiTask`
  - WiFi 收发与协议处理

## 5. 启动调试信息

当前工程上电后会先输出最小启动探针：

```text
BOOT USART2
BOOT USART3
```

随后进入任务初始化，`USART2` 会输出调试任务启动日志，便于快速判断：

- 系统是否启动
- 串口是否连通
- 当前观察的是 `USART2` 还是 `USART3`

## 6. 编译方式

在项目根目录执行：

```powershell
cmake --build --preset Debug
```

生成产物位于：

```text
build/Debug/
```

## 7. 调试说明

- 推荐使用 `USART2` 作为主调试串口
- 串口参数：
  - `115200`
  - `8N1`
- 如果修改了 CubeMX 配置，务必复查以下文件是否被回退：
  - `Core/Src/stm32f4xx_hal_timebase_tim.c`
  - `Core/Src/stm32f4xx_it.c`
  - `Core/Inc/stm32f4xx_it.h`
  - `Core/Src/system_stm32f4xx.c`

## 8. 相关文档

- `CURRENT_CONFIG.md`
  - 当前工程配置说明
- `PORTING_DIFF.md`
  - 从 `STM32F103` 迁移到 `STM32F407` 后的差异说明


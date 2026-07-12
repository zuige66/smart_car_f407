# AGENTS.md - smart_car

> 本文件是进入 `smart_car` 仓库后，AI agent 和协作者应优先阅读的项目说明。
> 目标：减少 CubeMX 覆盖、减少误改、保持 STM32F407 小车工程可持续维护。

## 0. 项目事实

- 项目：`smart_car`
- MCU：STM32F407
- 工程类型：STM32CubeMX + HAL + FreeRTOS + CMake
- 主要构建方式：`cmake --build --preset Debug`
- 串口调试口：`USART2`
- WiFi 模块：`USART3 + ESP-01S`
- 代码主目录：`Core/`
- CubeMX 工程文件：`smart_car.ioc`

## 1. 目录约定

- `Core/Src/`：CubeMX 生成代码为主。除非确实必须，不要把核心业务逻辑长期放在这里。
- `Core/Inc/`：CubeMX 生成头文件为主。
- `Core/App/Tasks/`：FreeRTOS 任务入口和任务级业务逻辑。
- `Core/App/Module/`：模块化功能逻辑，例如控制、WiFi、AI、避障、循迹。
- `Core/App/Driver/`：硬件驱动适配层，例如电机、AHT20、RFID、PID。
- `Core/App/Platform/`：对 CubeMX 生成代码的非侵入式替代/补丁逻辑。
- `Core/AI/`：NanoEdge AI 相关库和头文件。
- `docs/`：项目文档。

## 2. 代码放置规则

1. 优先把业务逻辑放在 `Core/App/Tasks/`、`Core/App/Module/`、`Core/App/Driver/`。
2. 只有在硬件初始化、RTOS 任务创建、ISR 接入等场景下，才修改 `Core/Src/` 的生成文件。
3. 如果某个问题可以通过 `Core/App/Platform/` 覆盖或旁路修复，优先用这种方式，减少 CubeMX 重新生成后的冲突。
4. 不要把大段自定义逻辑直接塞进 CubeMX 生成函数中，除非它明确位于 `USER CODE BEGIN/END` 区域并且适合长期维护。

## 3. CubeMX 注意事项

1. `Core/Src/freertos.c`、`gpio.c`、`i2c.c`、`spi.c`、`tim.c`、`usart.c` 等文件可能被重新生成覆盖。
2. 修改 CubeMX 相关文件前，先判断这部分改动是不是应该转移到 `Core/App/...`。
3. 若必须依赖 CubeMX 生成内容，尽量只依赖：
   - 外设句柄
   - 任务创建入口声明
   - GPIO / TIM / I2C / UART 初始化结果
4. 重新生成后，优先检查：
   - `freertos.c` 任务创建是否还指向正确的 `Start...Task`
   - 引脚命名是否变化
   - 定时器通道、串口实例、I2C 速度是否被改回默认
   - 中断使能是否仍符合当前项目预期

## 4. 当前项目约束

1. 串口调试信息默认走 `USART2`。
2. WiFi 模块默认走 `USART3`，ESP-01S 使用 AT 模式驱动。
3. OLED 使用 I2C3。
4. AHT20 使用 I2C2。
5. MLX90614 使用 I2C1。
6. 当前 AI 功能位于 `AITask.c` + `AIAnomalyDetect.c`，状态共享位于 `AIStatus.c`。
7. 当前控制状态机位于 `CtrlTask.c`，外部命令映射位于 `WifiComm.c`。

## 5. 构建与验证

- 构建命令：`cmake --build --preset Debug`
- 若修改了 CMake、AI 库、任务文件、模块文件，默认要重新编译。
- 若修改了串口、WiFi、AI、传感器逻辑，最终应给出至少一种可观察验证方式：
  - 串口日志
  - OLED 显示
  - WiFi JSON 数据

## 6. 修改建议

1. 先读相关文件再改，不要凭文件名猜逻辑。
2. 小步修改，优先保证可编译。
3. 若用户正在频繁用 CubeMX 重新生成，优先采用“不易被覆盖”的实现方式。
4. 若发现功能异常，先确认：
   - 数据源是否真的更新
   - 任务是否真的在运行
   - 日志是否能证明问题位置
5. 若是 AI / 传感器问题，优先增加低频调试输出，不要直接刷爆串口。

## 7. 禁止事项

1. 不要无理由大改 `Core/Src/` 生成代码。
2. 不要擅自删除用户已有调试日志、任务、外设配置。
3. 不要在未确认引脚定义的情况下修改电机/串口/I2C 映射。
4. 不要把学习模式下的 AI 分数直接用于最终控制逻辑而不加保护。
5. 不要使用破坏性 git 命令清理用户工作区。

## 8. 推荐工作流

1. 先确认当前需求影响的模块。
2. 读取相关任务、模块、驱动文件。
3. 判断改动应放在 `App` 层还是 CubeMX 生成层。
4. 修改后先编译。
5. 再给出用户可直接观察的验证点。

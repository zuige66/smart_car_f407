# NanoEdge AI 集成指南

## 概述

本项目集成了 NanoEdge AI 库，使用**预训练异常检测模型**对 MLX90614 红外温度传感器数据进行实时异常检测。

## 文件结构

```
smart_car/
├── Core/
│   ├── AI/
│   │   ├── NanoEdgeAI.h      # AI库头文件 (API定义)
│   │   └── libneai.a         # AI静态库 (预编译，含预训练模型)
│   └── App/
│       ├── Tasks/
│       │   └── AITask.c       # AI FreeRTOS任务 (主循环)
│       └── Module/
│           ├── AIAnomalyDetect.h    # 异常检测封装层
│           ├── AIAnomalyDetect.c
│           ├── AIStatus.h           # AI状态跨任务读写
│           └── AIStatus.c
└── docs/
    └── AI_INTEGRATION.md     # 本文档
```

## 工作原理

NanoEdge AI 异常检测的核心思路：

1. **预训练模型**：NanoEdge AI Studio 根据正常数据训练好模型，嵌入 `libneai.a` 中
2. **输入**：32 个 float 组成的时间序列信号
3. **输出**：相似度 (0-100)，100 = 完全正常，0 = 完全异常
4. **判定**：低于阈值认为异常

## 当前配置

| 项目 | 值 |
|---|---|
| 数据源 | MLX90614 物体温度 (红外测温) |
| 采样方式 | 连续读取 32 次，间隔 3ms |
| 信号长度 | 32 个 float (`NEAI_INPUT_SIGNAL_LENGTH`) |
| 轴数 | 1 (`NEAI_INPUT_AXIS_NUMBER`) |
| 模式 | 预训练 (`use_pretrained = true`) |
| 任务周期 | 200ms |
| 采样耗时 | ~96ms (32 × 3ms) |

## 运行流程

```
开机
  ↓
AI_AnomalyDetect_SetUsePretrained(1U)
  ↓
AI_AnomalyDetect_Init()  →  加载预训练模型，立即就绪
  ↓
osDelay(2000)  →  等待传感器稳定
  ↓
┌─ 每 200ms 循环 ──────────────────────────────┐
│                                                │
│  连续读取 MLX90614 × 32 次 (间隔 3ms)          │
│           ↓                                    │
│  neai_anomalydetection_detect(buffer, &score)  │
│           ↓                                    │
│  AI_StatusSet(ready=1, score, valid=1)         │
│           ↓                                    │
│  → OLED 显示 AI:xxx                            │
│  → WiFi 遥测 ai_score / ai_ready               │
│  → CtrlTask 状态机决策                          │
└────────────────────────────────────────────────┘
```

## 传感器数据准备

从 MLX90614 红外温度传感器连续快速采样 32 次，形成时间序列：

```c
static void AI_PrepareSensorData(float *buffer)
{
    for (uint8_t i = 0; i < NEAI_INPUT_SIGNAL_LENGTH; i++) {
        buffer[i] = AI_ReadMlx90614Object();  // I2C读取，带互斥锁+重试
        osDelay(3U);
    }
}
```

每次读取通过 `HAL_I2C_Mem_Read` 直接访问 MLX90614 的 `TOBJ1` 寄存器 (0x07)，使用 `I2CMutexHandle` 互斥锁保护 I2C 总线。

## AI 状态如何影响系统状态机

CtrlTask 的 `Ctrl_DetermineState()` 根据 AI 相似度分数决定系统状态：

| 相似度 | 系统状态 | 行为 |
|---|---|---|
| ≥ 70 | STATE_PATROL | 正常巡逻 |
| 50 ~ 69 | STATE_THERMAL_ALERT | 预警，蜂鸣器关 |
| 30 ~ 49 | STATE_THERMAL_WARNING | 报警，速度 50%，蜂鸣器开 |
| < 30 | STATE_EMERGENCY | 紧急撤离 |

阈值定义 (`AIAnomalyDetect.h`)：
```c
#define AI_SCORE_NORMAL_MIN     70U
#define AI_SCORE_WARNING_MIN    50U
#define AI_SCORE_ALARM_MIN      30U
```

**防抖机制**：从高警报状态退回 PATROL 时，需保持 5 秒确认。

## 串口调试输出

每 1 秒输出一次：
```
[AI] ready=1 valid=1 score=87 mlx_obj=36.5
```

## WiFi 遥测

遥测 JSON 中包含 AI 字段：
```json
{"type":"telemetry","MLX_obj":36.5,"ai_score":87,"ai_ready":1}
```

Alert JSON 同样包含：
```json
{"type":"alert","state":"temp_warning","ai_score":45,"ai_ready":1}
```

## OLED 显示

页面 1 第 3 行显示 AI 状态：
- `AI:87` — 相似度分数
- `AI:--` — 未就绪或检测失败

## API 参考

### AI_AnomalyDetect_Init()
```c
bool AI_AnomalyDetect_Init(void);
```
初始化 AI 模块。预训练模式下立即就绪。

### AI_AnomalyDetect_Check()
```c
AI_DetectResult_t AI_AnomalyDetect_Check(const float *data, uint8_t *similarity_out);
```
检测异常，输入 32 个 float，输出相似度百分比。

### AI_AnomalyDetect_IsAnomaly()
```c
bool AI_AnomalyDetect_IsAnomaly(uint8_t similarity, uint8_t threshold);
```
判断是否为异常，低于阈值认为异常。

### AI_StatusSet() / AI_StatusGet()
```c
void AI_StatusSet(uint8_t ready, uint8_t similarity, uint8_t score_valid, uint32_t learn_count);
AIStatus_t AI_StatusGet(void);
```
跨任务安全的 AI 状态读写（通过 volatile 变量）。

## 注意事项

1. **模型是预训练的**：`libneai.a` 中已嵌入 NanoEdge AI Studio 生成的模型，无需上电后学习
2. **数据格式**：必须是 32 个连续的同类型传感器采样，不能混合不同传感器
3. **I2C 互斥**：AITask 和 SensorTask 共享 I2C 总线，通过 `I2CMutexHandle` 互斥锁保护
4. **阈值调整**：根据实际场景调整 `AI_SCORE_*` 宏定义

## 参考资料

- NanoEdge AI Studio: https://www.st.com/en/development-tools/nanoedgeaistudio.html
- MLX90614 数据手册: https://www.melexis.com/en/product/MLX90614/Digital-Plug-Play-Infrared-Thermometer-TO-Can

# NanoEdge AI 集成指南

## 概述

本项目集成了 NanoEdge AI 库，用于传感器数据的异常检测。该库专为 STM32F4 系列 MCU 优化，支持 Cortex-M4 硬件浮点运算。

## 文件结构

```
smart_car/
├── Core/
│   ├── AI/
│   │   ├── NanoEdgeAI.h      # AI库头文件
│   │   └── libneai.a         # AI静态库
│   └── App/
│       └── Module/
│           ├── AIAnomalyDetect.h    # 异常检测模块头文件
│           ├── AIAnomalyDetect.c    # 异常检测模块实现
│           └── AIIntegration_Example.c  # 集成示例
└── docs/
    └── AI_INTEGRATION.md     # 本文档
```

## 技术规格

- **MCU**: STM32F407 (Cortex-M4)
- **输入信号长度**: 32个float
- **输入轴数**: 1
- **最小学习样本数**: 298
- **内存占用**: RAM ~224字节, Flash ~48字节
- **算法**: ZSM (Z-Score Method) 异常检测
- **性能**: 准确率 99.12%, 主KPI 99.96%

## 使用步骤

### 1. 初始化

```c
#include "AIAnomalyDetect.h"

// 在系统启动后初始化
if (!AI_AnomalyDetect_Init()) {
    // 初始化失败处理
}
```

### 2. 学习阶段

学习阶段需要收集正常状态下的传感器数据，至少298个样本：

```c
float sensor_data[32];

// 准备传感器数据 (32个float)
// 例如: 温度、湿度、加速度等
Prepare_Sensor_Data(sensor_data);

AI_LearnStatus_t status = AI_AnomalyDetect_Learn(sensor_data);

switch (status) {
    case AI_LEARN_DONE:
        // 学习完成
        break;
    case AI_LEARN_IN_PROGRESS:
        // 继续学习
        break;
    case AI_LEARN_ERROR:
        // 错误处理
        break;
}
```

### 3. 检测阶段

学习完成后，可以进行异常检测：

```c
float sensor_data[32];
uint8_t similarity;

Prepare_Sensor_Data(sensor_data);

AI_DetectResult_t result = AI_AnomalyDetect_Check(sensor_data, &similarity);

if (result == AI_DETECT_OK) {
    // similarity: 0-100, 100表示完全正常
    if (AI_AnomalyDetect_IsAnomaly(similarity, 80)) {
        // 检测到异常!
        // 触发报警、停止电机等
    }
}
```

## 数据准备指南

输入缓冲区需要32个float，可以根据应用场景灵活分配：

### 方案1: 单传感器 (32个采样点)
```c
// 适合: 振动检测、声音检测
buffer[0] = sensor_sample_0;
buffer[1] = sensor_sample_1;
// ... 共32个采样点
```

### 方案2: 多传感器组合
```c
// 适合: 综合环境监测
// [0-7]: 温度 (8个采样点)
// [8-15]: 湿度 (8个采样点)
// [16-23]: 气体浓度 (8个采样点)
// [24-31]: 加速度 (8个采样点)
```

### 方案3: 当前小车配置
```c
// [0-7]:   MLX90614物体温度
// [8-15]:  MLX90614环境温度
// [16-23]: AHT20温度
// [24-31]: AHT20湿度
```

## 应用场景

### 1. 电机异常检测
- 监测电机电流波形
- 检测轴承磨损、堵转等异常

### 2. 电池健康监测
- 监测电压、电流曲线
- 检测电池老化、过放等异常

### 3. 环境异常检测
- 温度异常波动
- 气体泄漏检测

### 4. 运动状态监测
- 加速度异常
- 碰撞检测

## API 参考

### AI_AnomalyDetect_Init()
```c
bool AI_AnomalyDetect_Init(void);
```
初始化AI模块，使用预训练模型。

### AI_AnomalyDetect_Learn()
```c
AI_LearnStatus_t AI_AnomalyDetect_Learn(const float *data);
```
学习正常模式，输入32个float数据。

### AI_AnomalyDetect_Check()
```c
AI_DetectResult_t AI_AnomalyDetect_Check(const float *data, uint8_t *similarity_out);
```
检测异常，返回相似度百分比。

### AI_AnomalyDetect_IsAnomaly()
```c
bool AI_AnomalyDetect_IsAnomaly(uint8_t similarity, uint8_t threshold);
```
判断是否为异常，低于阈值认为异常。

### AI_AnomalyDetect_IsReady()
```c
bool AI_AnomalyDetect_IsReady(void);
```
检查是否已完成学习并就绪。

## 注意事项

1. **学习数据质量**: 学习阶段必须使用正常状态的数据，异常数据会影响检测精度
2. **数据预处理**: 建议对传感器数据进行归一化处理
3. **阈值选择**: 根据应用场景调整异常阈值 (建议80-90)
4. **实时性**: 检测函数执行时间很短，适合实时应用
5. **内存占用**: 库本身占用RAM约224字节，注意总内存预算

## 故障排除

### 初始化失败
- 检查库文件是否正确链接
- 确认MCU型号匹配 (STM32F4系列)

### 检测不准确
- 增加学习样本数量
- 检查学习数据是否为正常状态
- 调整异常阈值

### 内存不足
- 优化其他模块的内存占用
- 考虑使用外部存储

## 参考资料

- NanoEdge AI Studio: https://www.st.com/en/development-tools/nanoedgeaistudio.html
- STM32F407参考手册: https://www.st.com/resource/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf

# 🎯 小车控制架构详解

## 整体架构图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           控制层 (决策者)                                │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐               │
│    │  CtrlTask   │    │  WiFi命令   │    │  RFID事件   │               │
│    │  (主控)     │◄───│  (远程)     │    │  (触发)     │               │
│    └──────┬──────┘    └─────────────┘    └─────────────┘               │
│           │                                                             │
│           ▼                                                             │
│    ┌─────────────────────────────────────────────────────┐             │
│    │              状态机 (State Machine)                  │             │
│    ├─────────────────────────────────────────────────────┤             │
│    │ STATE_STANDBY      → 停止                           │             │
│    │ STATE_PATROL       → 巡逻 (循迹+避障)               │             │
│    │ STATE_THERMAL_ALERT  → 温度预警 (减速)              │             │
│    │ STATE_THERMAL_WARNING → 温度报警 (慢速)             │             │
│    │ STATE_EMERGENCY    → 紧急 (停止/逃离)               │             │
│    │ STATE_LOW_BATTERY  → 低电量 (返回)                  │             │
│    └─────────────────────────────────────────────────────┘             │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
                                      │
                                      │ 根据状态调用不同模块
                                      ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                           算法层 (执行者)                                │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│    ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐ │
│    │ TrackCtrl   │  │ObstacleCtrl │  │ ThermalCtrl │  │ BatteryCtrl │ │
│    │ 循迹控制    │  │ 避障控制    │  │ 温度控制    │  │ 电池控制    │ │
│    ├─────────────┤  ├─────────────┤  ├─────────────┤  ├─────────────┤ │
│    │ 输入:       │  │ 输入:       │  │ 输入:       │  │ 输入:       │ │
│    │ - 循迹传感器│  │ - 超声波    │  │ - 温度      │  │ - 电池电量  │ │
│    │ 输出:       │  │ 输出:       │  │ 输出:       │  │ 输出:       │ │
│    │ - 电机命令  │  │ - 电机命令  │  │ - 电机命令  │  │ - 电机命令  │ │
│    └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘ │
│           │                │                │                │         │
│           └────────────────┴────────────────┴────────────────┘         │
│                                    │                                    │
│                                    ▼                                    │
│                           MotorCmd_t 命令                               │
│                    ┌──────────────────────────┐                        │
│                    │ cmd: 命令类型            │                        │
│                    │ pwm: 基础速度            │                        │
│                    │ pwm_left: 左轮速度       │                        │
│                    │ pwm_right: 右轮速度      │                        │
│                    └──────────────────────────┘                        │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
                                      │
                                      │ 通过消息队列传递
                                      ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                           执行层 (驱动者)                                │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│    ┌─────────────────────────────────────────────────────────────────┐ │
│    │                      DriverTask                                 │ │
│    ├─────────────────────────────────────────────────────────────────┤ │
│    │                                                                 │ │
│    │  接收 MotorAction 队列                                          │ │
│    │           │                                                     │ │
│    │           ▼                                                     │ │
│    │  switch (cmd.cmd) {                                             │ │
│    │    case MOTOR_CMD_FORWARD:                                      │ │
│    │      Motor_SetDirection(LEFT, FORWARD);                         │ │
│    │      Motor_SetDirection(RIGHT, FORWARD);                        │ │
│    │      Motor_SetSpeed(LEFT, cmd.pwm_left);                        │ │
│    │      Motor_SetSpeed(RIGHT, cmd.pwm_right);                      │ │
│    │      break;                                                     │ │
│    │    case MOTOR_CMD_TURN_LEFT:                                    │ │
│    │      Motor_SetSpeed(LEFT, cmd.pwm_left);    // 慢               │ │
│    │      Motor_SetSpeed(RIGHT, cmd.pwm_right);  // 快               │ │
│    │      break;                                                     │ │
│    │    ...                                                          │ │
│    │  }                                                              │ │
│    │           │                                                     │ │
│    │           ▼                                                     │ │
│    │  Motor_SetSpeed() → PWM输出 → TB6612 → 电机                    │ │
│    │                                                                 │ │
│    └─────────────────────────────────────────────────────────────────┘ │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                           硬件层                                        │
├─────────────────────────────────────────────────────────────────────────┤
│    ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                  │
│    │   TIM4      │  │   TIM3      │  │  TB6612FNG  │                  │
│    │  (左电机)   │  │  (右电机)   │  │  (H桥驱动)  │                  │
│    └──────┬──────┘  └──────┬──────┘  └──────┬──────┘                  │
│           │                │                │                          │
│           └────────────────┴────────────────┘                          │
│                                    │                                    │
│                                    ▼                                    │
│                           ┌─────────────┐                              │
│                           │  左电机 右电机│                              │
│                           │   (M)   (M)  │                              │
│                           └─────────────┘                              │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 📋 详细说明

### 1. 状态逻辑控制（谁决定小车做什么？）

**主控：CtrlTask.c**

```c
// CtrlTask.c - 主控制任务
void StartCtrlTask(void *argument)
{
    for (;;) {
        // 1. 读取所有传感器
        data = Ctrl_ReadAllSensors();
        
        // 2. 处理RFID事件
        Ctrl_HandleRfidEvent(&data);
        
        // 3. 确定系统状态
        new_state = Ctrl_DetermineState(&data);
        
        // 4. 根据状态调用不同模块
        switch (current_state) {
            case STATE_STANDBY:
                cmd.cmd = MOTOR_CMD_STOP;  // 停止
                break;
                
            case STATE_PATROL:
                cmd = TrackCtrl_Run(&data);      // 循迹控制
                obs_cmd = ObstacleCtrl_Run(&data); // 避障控制
                if (避障检测到障碍) {
                    cmd = obs_cmd;  // 避障优先
                }
                break;
                
            case STATE_THERMAL_ALERT:
                cmd = ThermalCtrl_Alert(&data, TrackCtrl_Run(&data));
                break;
                
            case STATE_EMERGENCY:
                cmd = ThermalCtrl_Emergency(&data);
                break;
                
            case STATE_LOW_BATTERY:
                cmd = BatteryCtrl_Return(&data);
                break;
        }
        
        // 5. 发送命令到电机任务
        osMessageQueuePut(MotorActionHandle, &cmd);
        
        osDelay(30);  // 30ms周期
    }
}
```

**状态转换触发条件：**

| 触发源 | 条件 | 目标状态 |
|--------|------|---------|
| RFID | 扫描到"start"标签 | STATE_PATROL |
| RFID | 扫描到"end_stop"标签 | STATE_LOW_BATTERY |
| AI检测 | similarity < 80 | STATE_THERMAL_ALERT |
| AI检测 | similarity < 60 | STATE_THERMAL_WARNING |
| AI检测 | similarity < 40 | STATE_EMERGENCY |
| WiFi命令 | 用户发送停止 | STATE_STANDBY |

---

### 2. 运动逻辑控制（谁决定电机怎么转？）

**算法模块：**

| 模块 | 文件 | 输入 | 输出 | 作用 |
|------|------|------|------|------|
| **TrackCtrl** | TrackCtrl.c | 循迹传感器 | MotorCmd_t | 循迹巡逻 |
| **ObstacleCtrl** | ObstacleCtrl.c | 超声波距离 | MotorCmd_t | 避障 |
| **ThermalCtrl** | ThermalCtrl.c | 温度+循迹 | MotorCmd_t | 温度报警时的行为 |
| **BatteryCtrl** | BatteryCtrl.c | 电池+循迹 | MotorCmd_t | 低电量返回 |

**TrackCtrl 示例：**

```c
// TrackCtrl.c - 循迹控制
MotorCmd_t TrackCtrl_Run(SensorData_t *data)
{
    uint8_t track_data = data->track;
    int8_t track_error = CalculateError(track_data);
    
    // 状态机
    switch (g_track_mode) {
        case TRACK_MODE_FOLLOW:
            // 正常跟随，PID调速
            base_pwm = TARGET_SPEED;  // 260
            turn_output = PID_Compute(&turn_pid, track_error);
            left_pwm = base_pwm + turn_output;
            right_pwm = base_pwm - turn_output;
            return MakeForward(left_pwm, right_pwm);
            
        case TRACK_MODE_SHARP_LEFT:
            // 急左转
            return MakeTurn(-1, 180);
            
        case TRACK_MODE_SEARCH:
            // 搜索轨道
            return MakeSearch(direction);
    }
}
```

---

### 3. 电机执行（谁真正控制电机？）

**执行层：DriverTask.c**

```c
// DriverTask.c - 电机驱动任务
void StartDriverTask(void *argument)
{
    MotorDriver_Init();
    
    for (;;) {
        // 1. 从队列接收命令
        if (osMessageQueueGet(MotorActionHandle, &cmd, NULL, 100) == osOK) {
            
            // 2. 根据命令类型执行
            switch (cmd.cmd) {
                case MOTOR_CMD_STOP:
                    Motor_Stop(MOTOR_LEFT);
                    Motor_Stop(MOTOR_RIGHT);
                    break;
                    
                case MOTOR_CMD_FORWARD:
                    Motor_SetDirection(MOTOR_LEFT, MOTOR_FORWARD);
                    Motor_SetDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                    Motor_SetSpeed(MOTOR_LEFT, cmd.pwm_left);
                    Motor_SetSpeed(MOTOR_RIGHT, cmd.pwm_right);
                    break;
                    
                case MOTOR_CMD_TURN_LEFT:
                    // 左轮慢，右轮快
                    Motor_SetSpeed(MOTOR_LEFT, cmd.pwm_left);   // 90
                    Motor_SetSpeed(MOTOR_RIGHT, cmd.pwm_right); // 180
                    break;
                    
                case MOTOR_CMD_SPIN_LEFT:
                    // 原地左旋
                    Motor_SetDirection(MOTOR_LEFT, MOTOR_BACKWARD);
                    Motor_SetDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                    Motor_SetSpeed(MOTOR_LEFT, cmd.pwm);
                    Motor_SetSpeed(MOTOR_RIGHT, cmd.pwm);
                    break;
            }
        }
    }
}
```

---

## 🔄 完整控制流程示例

### 场景：小车循迹巡逻

```
时间线:
─────────────────────────────────────────────────────────────────────

t=0ms   SensorTask: 读取循迹传感器 → track=0x06 (居中)
        │
t=30ms  CtrlTask: 
        │  ├─ 状态: STATE_PATROL
        │  ├─ 调用 TrackCtrl_Run()
        │  │   ├─ track_error = 0 (居中)
        │  │   ├─ turn_output = 0
        │  │   └─ 返回: {FORWARD, left=260, right=260}
        │  └─ 发送到 MotorAction 队列
        │
t=35ms  DriverTask:
        │  ├─ 接收命令
        │  ├─ Motor_SetSpeed(LEFT, 260)
        │  └─ Motor_SetSpeed(RIGHT, 260)
        │
        │  电机转动，小车直行
        │
t=60ms  SensorTask: 读取循迹传感器 → track=0x02 (偏左)
        │
t=90ms  CtrlTask:
        │  ├─ 调用 TrackCtrl_Run()
        │  │   ├─ track_error = -1
        │  │   ├─ turn_output = -40 (PID计算)
        │  │   └─ 返回: {FORWARD, left=220, right=300}
        │  └─ 发送到队列
        │
t=95ms  DriverTask:
        │  ├─ Motor_SetSpeed(LEFT, 220)   // 左轮慢
        │  └─ Motor_SetSpeed(RIGHT, 300)  // 右轮快
        │
        │  小车向右修正，回到轨道
        │
t=120ms SensorTask: track=0x06 (回到居中)
        │
        ... 循环继续
```

---

## 📊 控制层级总结

| 层级 | 模块 | 职责 | 周期 |
|------|------|------|------|
| **状态层** | CtrlTask | 决定小车做什么 | 30ms |
| **算法层** | TrackCtrl等 | 决定电机怎么转 | 被CtrlTask调用 |
| **执行层** | DriverTask | 执行电机命令 | 100ms |
| **硬件层** | Motor/TIM | PWM输出 | 硬件自动 |

---

## 🎮 谁可以控制小车？

### 1. 自动控制（代码逻辑）

| 控制源 | 触发方式 | 控制内容 |
|--------|---------|---------|
| **循迹传感器** | 黑线检测 | 转向、速度 |
| **超声波** | 距离检测 | 避障、停车 |
| **AI检测** | 相似度 | 报警、紧急 |
| **RFID** | 标签扫描 | 启动、停止、测量 |
| **温度传感器** | 温度值 | 报警级别 |
| **电池电量** | 电压值 | 低电量返回 |

### 2. 手动控制（WiFi命令）

```c
// WiFi可以发送的命令
Wifi_SendCommand("start");     // 启动巡逻
Wifi_SendCommand("stop");      // 停止
Wifi_SendCommand("forward");   // 前进
Wifi_SendCommand("left");      // 左转
Wifi_SendCommand("right");     // 右转
Wifi_SendCommand("emergency"); // 紧急停止
```

---

## 🔧 修改控制逻辑的位置

| 想修改什么 | 修改哪个文件 |
|-----------|-------------|
| 状态转换逻辑 | CtrlTask.c → Ctrl_DetermineState() |
| 循迹行为 | TrackCtrl.c → TrackCtrl_Run() |
| 避障行为 | ObstacleCtrl.c → ObstacleCtrl_Run() |
| 温度报警行为 | ThermalCtrl.c |
| 低电量行为 | BatteryCtrl.c |
| 电机执行方式 | DriverTask.c |
| 电机底层驱动 | motor.c |

---

## 📁 相关文件索引

| 文件 | 作用 |
|------|------|
| Core/App/Tasks/CtrlTask.c | 主控制任务，状态机 |
| Core/App/Tasks/DriverTask.c | 电机驱动任务 |
| Core/App/Tasks/SensorTask.c | 传感器数据采集 |
| Core/App/Module/TrackCtrl.c | 循迹控制算法 |
| Core/App/Module/ObstacleCtrl.c | 避障控制算法 |
| Core/App/Module/ThermalCtrl.c | 温度控制算法 |
| Core/App/Module/BatteryCtrl.c | 电池控制算法 |
| Core/App/Driver/motor.c | 电机底层驱动 |
| Core/App/Driver/pid.c | PID控制算法 |

---

## 2026-07-12 控制逻辑更新

- 状态判定优先级增加红外温度硬触发：`MLX90614 object_temp >= 60°C` 时进入 `STATE_EMERGENCY`。
- AI 分数仅负责正常/预警/报警分级：`>=80` 正常，`70~79` 预警，`<70` 报警。
- RFID `end_stop` 返航和高温紧急撤离保持同类动作：右旋 180°。
- 当前 180° 实车校准值：`RETURN_SPIN_CYCLES=45`。
- 当前循迹纠偏参数：`TRACK_TURN_GAIN=250`，`TRACK_TURN_PWM_DIFF_MAX=350`，`TRACK_PWM_MAX=450`。

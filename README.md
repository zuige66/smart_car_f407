# smart_car — STM32F407VET6 智能巡检小车

STM32F407 + FreeRTOS + ESP8266 WiFi 遥控智能小车。

## 快速开始

```powershell
cmake --build --preset Debug
```

产物：`build/Debug/smart_car.elf`

## WiFi 连接

| 参数 | 值 |
|---|---|
| SSID | `SmartCar_F407` |
| 密码 | `12345678` |
| TCP | `192.168.4.1:8080` |

小车自动每 3 秒发送 JSON 遥测，手机可发送命令（`start_patrol` / `idle` / `evacuate` 等）。

## 功能列表

- 4 路红外循迹 + PID 控制
- 11 状态超声波避障
- 温度分级响应：29°C 预警 / 30°C 降速 / 31°C 紧急返航
- RFID-RC522 8 个位置点识别 + 停车测量 + 终点掉头
- ESP8266 WiFi 遥测上报 + TCP 命令控制
- OLED 双页面状态显示
- 上电 8 项自检

## 文档

| 文档 | 说明 |
|---|---|
| [CURRENT_CONFIG.md](CURRENT_CONFIG.md) | **完整技术参考手册** — 引脚映射、任务表、传感器、PID、RFID、WiFi 协议、状态机、避障、循迹、自检等所有细节 |
| [PORTING_DIFF.md](PORTING_DIFF.md) | STM32F103 → STM32F407 迁移差异说明 |

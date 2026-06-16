# smart_car — STM32F407 智能巡检小车

STM32F407VET6 + FreeRTOS + ESP8266 WiFi 遥控智能小车。  
由旧工程 `clion_car (STM32F103)` 迁移而来，按实际硬件重新适配。

## 功能

- **循迹** — 4 路红外 + PID 转向 (Kp=95, Ki=8, Kd=20)
- **避障** — 11 状态超声波避障，自动绕行较宽侧
- **温度分级响应** — 29°C 预警 / 30°C 降速 / 31°C 紧急返航
- **RFID 定位** — RC522 识别 8 个点位 (start / place_1~6 / end_stop)，遇点位停车测量 3s
- **WiFi 遥测** — 每 3s 上报 JSON（含温度、湿度、距离、循迹、RFID 位置、状态）
- **手机遥控** — TCP 命令：idle / start_patrol / evacuate / return_home 等
- **OLED 显示** — 双页面循环：状态 + 传感器数据
- **ESP 桥接** — 调试串口透传 ESP8266 AT 指令

## 快速开始

```powershell
cmake --build --preset Debug
```

产物：`build/Debug/smart_car.elf`

## WiFi 连接

AP: `SmartCar_F407` / `12345678` → TCP `192.168.4.1:8080`

## 文档

- **[CURRENT_CONFIG.md](CURRENT_CONFIG.md)** — 完整技术参考（引脚映射、任务表、状态机、协议等）
- **[PORTING_DIFF.md](PORTING_DIFF.md)** — 从 STM32F103 迁移到 STM32F407 的差异说明

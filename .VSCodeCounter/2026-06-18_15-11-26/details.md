# Details

Date : 2026-06-18 15:11:26

Directory d:\\develop\\Workplace\\VscodeWorkplace\\stm32\\smart_car\\Core

Total : 66 files,  6698 codes, 3358 comments, 1677 blanks, all 11733 lines

[Summary](results.md) / Details / [Diff Summary](diff.md) / [Diff Details](diff-details.md)

## Files
| filename | language | code | comment | blank | total |
| :--- | :--- | ---: | ---: | ---: | ---: |
| [Core/App/Driver/Aht20.c](/Core/App/Driver/Aht20.c) | C | 111 | 44 | 28 | 183 |
| [Core/App/Driver/Aht20.h](/Core/App/Driver/Aht20.h) | C++ | 6 | 16 | 6 | 28 |
| [Core/App/Driver/RfidReader.c](/Core/App/Driver/RfidReader.c) | C | 69 | 51 | 15 | 135 |
| [Core/App/Driver/RfidReader.h](/Core/App/Driver/RfidReader.h) | C++ | 10 | 31 | 10 | 51 |
| [Core/App/Driver/board\_compat.h](/Core/App/Driver/board_compat.h) | C++ | 46 | 8 | 14 | 68 |
| [Core/App/Driver/motor.c](/Core/App/Driver/motor.c) | C | 82 | 38 | 11 | 131 |
| [Core/App/Driver/motor.h](/Core/App/Driver/motor.h) | C++ | 17 | 30 | 11 | 58 |
| [Core/App/Driver/pid.c](/Core/App/Driver/pid.c) | C | 94 | 69 | 20 | 183 |
| [Core/App/Driver/pid.h](/Core/App/Driver/pid.h) | C++ | 29 | 54 | 13 | 96 |
| [Core/App/Module/BatteryCtrl.c](/Core/App/Module/BatteryCtrl.c) | C | 22 | 33 | 7 | 62 |
| [Core/App/Module/BatteryCtrl.h](/Core/App/Module/BatteryCtrl.h) | C++ | 9 | 27 | 9 | 45 |
| [Core/App/Module/Encoder.c](/Core/App/Module/Encoder.c) | C | 11 | 18 | 4 | 33 |
| [Core/App/Module/Encoder.h](/Core/App/Module/Encoder.h) | C++ | 7 | 15 | 6 | 28 |
| [Core/App/Module/ObstacleCtrl.c](/Core/App/Module/ObstacleCtrl.c) | C | 185 | 62 | 28 | 275 |
| [Core/App/Module/ObstacleCtrl.h](/Core/App/Module/ObstacleCtrl.h) | C++ | 8 | 22 | 8 | 38 |
| [Core/App/Module/SelfTest.c](/Core/App/Module/SelfTest.c) | C | 178 | 72 | 50 | 300 |
| [Core/App/Module/SelfTest.h](/Core/App/Module/SelfTest.h) | C++ | 6 | 14 | 6 | 26 |
| [Core/App/Module/ThermalCtrl.c](/Core/App/Module/ThermalCtrl.c) | C | 108 | 54 | 20 | 182 |
| [Core/App/Module/ThermalCtrl.h](/Core/App/Module/ThermalCtrl.h) | C++ | 10 | 36 | 10 | 56 |
| [Core/App/Module/TrackCtrl.c](/Core/App/Module/TrackCtrl.c) | C | 269 | 101 | 36 | 406 |
| [Core/App/Module/TrackCtrl.h](/Core/App/Module/TrackCtrl.h) | C++ | 10 | 32 | 10 | 52 |
| [Core/App/Module/WifiComm.c](/Core/App/Module/WifiComm.c) | C | 676 | 9 | 128 | 813 |
| [Core/App/Module/WifiComm.h](/Core/App/Module/WifiComm.h) | C++ | 18 | 7 | 5 | 30 |
| [Core/App/Platform/runtime\_diag.c](/Core/App/Platform/runtime_diag.c) | C | 105 | 11 | 21 | 137 |
| [Core/App/Platform/runtime\_diag.h](/Core/App/Platform/runtime_diag.h) | C++ | 30 | 22 | 8 | 60 |
| [Core/App/Platform/stm32f4xx\_hal\_timebase\_tim\_user.c](/Core/App/Platform/stm32f4xx_hal_timebase_tim_user.c) | C | 43 | 12 | 11 | 66 |
| [Core/App/Platform/stm32f4xx\_it\_user.c](/Core/App/Platform/stm32f4xx_it_user.c) | C | 69 | 10 | 15 | 94 |
| [Core/App/Tasks/Ctrl.h](/Core/App/Tasks/Ctrl.h) | C++ | 55 | 44 | 16 | 115 |
| [Core/App/Tasks/CtrlTask.c](/Core/App/Tasks/CtrlTask.c) | C | 335 | 70 | 54 | 459 |
| [Core/App/Tasks/DriverTask.c](/Core/App/Tasks/DriverTask.c) | C | 138 | 25 | 14 | 177 |
| [Core/App/Tasks/HCSR04.c](/Core/App/Tasks/HCSR04.c) | C | 149 | 36 | 33 | 218 |
| [Core/App/Tasks/LedTask.c](/Core/App/Tasks/LedTask.c) | C | 46 | 13 | 12 | 71 |
| [Core/App/Tasks/OledTask.c](/Core/App/Tasks/OledTask.c) | C | 121 | 46 | 20 | 187 |
| [Core/App/Tasks/RfidTask.c](/Core/App/Tasks/RfidTask.c) | C | 395 | 121 | 79 | 595 |
| [Core/App/Tasks/SensorTask.c](/Core/App/Tasks/SensorTask.c) | C | 167 | 49 | 34 | 250 |
| [Core/App/Tasks/UartTask.c](/Core/App/Tasks/UartTask.c) | C | 194 | 25 | 33 | 252 |
| [Core/App/Tasks/WifiTask.c](/Core/App/Tasks/WifiTask.c) | C | 23 | 13 | 8 | 44 |
| [Core/Inc/FreeRTOSConfig.h](/Core/Inc/FreeRTOSConfig.h) | C++ | 72 | 79 | 22 | 173 |
| [Core/Inc/adc.h](/Core/Inc/adc.h) | C++ | 12 | 27 | 14 | 53 |
| [Core/Inc/font.h](/Core/Inc/font.h) | C++ | 28 | 9 | 7 | 44 |
| [Core/Inc/gpio.h](/Core/Inc/gpio.h) | C++ | 11 | 27 | 12 | 50 |
| [Core/Inc/i2c.h](/Core/Inc/i2c.h) | C++ | 16 | 27 | 16 | 59 |
| [Core/Inc/main.h](/Core/Inc/main.h) | C++ | 73 | 39 | 20 | 132 |
| [Core/Inc/oled.h](/Core/Inc/oled.h) | C++ | 28 | 0 | 7 | 35 |
| [Core/Inc/runtime\_diag.h](/Core/Inc/runtime_diag.h) | C++ | 4 | 0 | 3 | 7 |
| [Core/Inc/spi.h](/Core/Inc/spi.h) | C++ | 12 | 27 | 14 | 53 |
| [Core/Inc/stm32f4xx\_hal\_conf.h](/Core/Inc/stm32f4xx_hal_conf.h) | C++ | 278 | 132 | 86 | 496 |
| [Core/Inc/stm32f4xx\_it.h](/Core/Inc/stm32f4xx_it.h) | C++ | 17 | 34 | 15 | 66 |
| [Core/Inc/tim.h](/Core/Inc/tim.h) | C++ | 17 | 27 | 17 | 61 |
| [Core/Inc/usart.h](/Core/Inc/usart.h) | C++ | 14 | 27 | 15 | 56 |
| [Core/Src/adc.c](/Core/Src/adc.c) | C | 50 | 51 | 29 | 130 |
| [Core/Src/font.c](/Core/Src/font.c) | C | 419 | 12 | 12 | 443 |
| [Core/Src/freertos.c](/Core/Src/freertos.c) | C | 128 | 113 | 44 | 285 |
| [Core/Src/gpio.c](/Core/Src/gpio.c) | C | 73 | 57 | 32 | 162 |
| [Core/Src/i2c.c](/Core/Src/i2c.c) | C | 117 | 99 | 58 | 274 |
| [Core/Src/main.c](/Core/Src/main.c) | C | 84 | 104 | 43 | 231 |
| [Core/Src/oled.c](/Core/Src/oled.c) | C | 474 | 264 | 59 | 797 |
| [Core/Src/spi.c](/Core/Src/spi.c) | C | 44 | 51 | 26 | 121 |
| [Core/Src/stm32f4xx\_hal\_msp.c](/Core/Src/stm32f4xx_hal_msp.c) | C | 7 | 53 | 24 | 84 |
| [Core/Src/stm32f4xx\_hal\_timebase\_tim.c](/Core/Src/stm32f4xx_hal_timebase_tim.c) | C | 48 | 64 | 18 | 130 |
| [Core/Src/stm32f4xx\_it.c](/Core/Src/stm32f4xx_it.c) | C | 47 | 110 | 39 | 196 |
| [Core/Src/syscalls.c](/Core/Src/syscalls.c) | C | 162 | 47 | 36 | 245 |
| [Core/Src/sysmem.c](/Core/Src/sysmem.c) | C | 28 | 51 | 9 | 88 |
| [Core/Src/system\_stm32f4xx.c](/Core/Src/system_stm32f4xx.c) | C | 347 | 318 | 83 | 748 |
| [Core/Src/tim.c](/Core/Src/tim.c) | C | 165 | 95 | 62 | 322 |
| [Core/Src/usart.c](/Core/Src/usart.c) | C | 72 | 74 | 42 | 188 |

[Summary](results.md) / Details / [Diff Summary](diff.md) / [Diff Details](diff-details.md)
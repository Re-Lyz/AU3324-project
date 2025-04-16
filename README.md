﻿# AU3324-project

主要电气硬件构成：**DC 伺服电机、ESP32、TTL 到 RS485 模块、MPU6050 传感器、OLED 显示屏**

主要连接方式：

ESP32为主板，主要连接TTL模块、MPU6050 传感器和OLED 显示屏，然后TTL模块接上RS485，485接伺服电机，供电伺服电机是独立供电的，详细连接示意：

**电源部分**

- 将 24V 电源接到伺服电机。
- ESP32、MPU6050 和 OLED 显示屏则由 ESP32 的供电或独立稳定电源（确保共地）提供适合的电压（通常为 3.3V 或 5V）。

**ESP32 与 TTL 到 RS485 模块**

- ESP32 的 UART TX、RX 分别接到 TTL 模块的 TX、RX。
- 同时，选定的数字引脚（如 GPIO4）连接到 TTL 模块的 DE/RE 控制引脚，用于模式切换。

**TTL 到 RS485 模块与伺服电机**

- TTL 模块的 RS485 侧的 A/B 接口直接连接到伺服电机的 RS485 通信口（A 对 A，B 对 B）。

**ESP32 与 I2C 设备（MPU6050 与 OLED）**

- ESP32 的 SDA、SCL 分别连接到 MPU6050 和 OLED 显示屏的 SDA、SCL。
- 这两个设备共用同一组 I2C 总线，但需确保它们的 I2C 地址不冲突。


## 基于蓝牙连接的云台控制台（Console）

将设备连接到ESP32后启动服务器，支持云台数据监控与姿态控制







------

### 要点记录：

1. 关于TTL2RS485模块与esp32之间的连接与引脚定义的关系，传入和传出的端口与程序中的定义是相反的
2. 选择合适速度曲线参数，考虑到伺服电机自身的加速度和速度精度，这里梯形曲线的加速度和s曲线的平均加速度为10 0.1rps/s，最大速度为30rpm。
3. 运动模式的选择，经过测试，位置模式无法实时调整伺服电机的相关参数，参数的更改必须发生在停机的时候，这造成速度曲线的振动十分明显。最后采用速度模式，能够随时调整电机运动时的一些参数，还需要进一步测试。
4. 维持平衡的问题采用的位置模式，因为这里会有停机的时间段来调整参数，使用位置模式比较稳定
5. 关于单双环pid的反馈，双环的参数更难设置调整，目前没有观察到双环的稳定性比单环好，还需要进一步调整参数

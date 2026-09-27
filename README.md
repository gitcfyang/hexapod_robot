# Hexapod Robot

六足机器人，基于 Raspberry Pi Pico (RP2040)。18 路数字舵机（2× PCA9685）、
BNO055 IMU 自动调平，支持 ELRS CRSF / PS2 手柄 / USB 三种输入。

## 特性

- 控制循环（默认 100Hz，运行期可调 50~200Hz）+ 实时 IK 解算；四种步态
  （三角 6/8、波纹、波浪）+ 仿生连续变速
- 平衡模式（机身姿态直接控制）+ IMU 姿态补偿自动调平（可选）
- CRSF / PS2 双驱动常编译，`!MODE` 运行时切换，无需焊接插拔；两个驱动共用同一套
  通道与参数（手柄的摇杆/按键在固件里换算到同一套通道号），遥控页 5Hz 实时显示
  协议原值（CRSF 16 通道 172~1811 / PS2 四摇杆 0~255 + 16 按键）
- Betaflight 式「模式」页：解锁/平衡/步态/站立姿态挂在哪个通道、落在哪几档
  （模式矩阵），以及手柄的一次性动作用哪组按键（组合键），全部是运行期参数，
  `!CFGW` 掉电保持；解锁是电平语义，开关拨走即上锁
- 电池保护（过压/低压/过放自动断舵机供电）、硬件看门狗、I2C 总线自恢复
- 非易失存储：校准参数（18 路 horn_offset + PWM 周期）存 flash 掉电保持，
  事件日志环形记录（BOOT/电池/校准），`!SAVE` / `!LOG` / `!LOGC`
- 运行时参数：65 项（步态行程/阈值/开关/方向取反/IMU 手感/通道映射/模式矩阵与
  组合键/控制周期/外设所有权与波特率）可在网页或串口在线调整，`!CFG` / `!CFGR` /
  `!CFGW`，改完即生效、`!CFGW` 存 flash 掉电保持
- 板载预留接口在线可控：直流电机 ×2（仅调速，无方向脚）/ 蜂鸣器 / 双 LED /
  外部 UART0（GP0/GP1）/ 外部 I2C 或 ADC（GP26/27）/ PCA9685 空闲 14 路 PWM，
  串口命令 + 网页「外设」页（外设尚未接线，详见 [STATUS.md](STATUS.md)）
- TCP/IP 远程访问：串口控制台内置 TCP 桥接（多客户端广播 + 命令反向下发），
  可 headless 运行，配 `tools/tcp_monitor.py` 远程操作
- PCB 设计、Gerber、全项目 BOM、机械 STEP 全部开源（见 [hardware/](hardware/)）

## 快速开始

```bash
cd pico/build && make -j$(nproc)
```

编译产物 `hexapod_pico.uf2` 拖入 Pico 的 USB 盘即烧录；Linux 下可免按键：
`picotool load -f -x build/hexapod_pico.uf2`（运行中的固件会被软复位进 BOOTSEL）。
调试控制台：`python3 tools/serial_console.py`（自动连接、断线重连）。
网页配置台：`python3 tools/webconfig/server.py` → 浏览器开 `127.0.0.1:8080`
（打开先是**开始页**：只看硬件是否在线、机器人跑的是什么固件，并可一键烧录新固件；
检测到设备后点「连接」才进调试页 —— 状态: 实时电压/I2C/IMU/舵机角度 · 参数按功能
分页（步态/姿态/遥控/模式/电源/平衡/控制/外设）: 滑条改参数并写 flash · 遥控: 输入源切换 +
通道实时显示 · 模式: 解锁/步态/站姿的模式矩阵 + 手柄组合键 + 实时状态徽标 ·
控制: 舵机供电开关 + 行走/转向/步态/抬腿按钮 + 两块 PCA9685 的
PWM 周期滑条 · 外设: 电机/LED/蜂鸣器/外部 UART/外部 I2C+ADC/空闲 PWM 控制卡 ·
校准: 走查/保存/总线扫描 · 日志: 串口日志 + 命令输入；拔 USB、桥接断开或点
「断开连接」都会回到开始页，细节见 [STATUS.md](STATUS.md)）。
连接前可勾「调试时允许遥控器控制」—— 不勾（默认）则调试期间遥控器只上报通道值，
机器人只听网页命令；机器人自己会在失去心跳 6s 后放回遥控器（网页挂了不会锁死）。
远程访问：控制台默认同时开 TCP 桥接（`127.0.0.1:7100`），
另开终端 `python3 tools/tcp_monitor.py` 即可远程收发命令（可多开）。

## 硬件概览

- MCU: Pico (RP2040) · 舵机: 18× 双轴数字舵机, 2× PCA9685 · IMU: BNO055 (I²C 0x29)
- 电池: 2S 18650, GP28 ADC2 分压 47/377 (330k+47k)，低压/过压/过放自动断舵机供电（双路AO4407A实现）
  （`batt_check` 参数默认 0，待新板实测分压后启用）
- 足端微动开关 ×6 · 无源蜂鸣器 · 双 LED · 直流电机 ×2（调速）
- 预留未接线：外部 UART0（GP0/GP1）· 外部 I2C 或 ADC（GP26/27）·
  PCA9685 空闲 14 路 PWM（编号与舵机通道不相交，可与舵机同批输出）
- 完整 GPIO 分布、电池保护分级行为: 见 [STATUS.md](STATUS.md) 硬件章节

## 机架

本项目机架使用 18× **双轴舵机**（STEP 模型见 [hardware/mechanical/](hardware/mechanical/)）。
低成本替代可用 3D 打印方案 [MakeYourPet/hexapod](https://github.com/MakeYourPet/hexapod)
(MIT，MG996R 单轴舵机)：移植本固件只需修改 [pico/Inc/hexapod_config.h](pico/Inc/hexapod_config.h)
的腿节长度、舵机零位、安装角等参数（详见 STATUS.md），IK 与步态算法本身无需修改（同为 3 关节腿）。

## 控制

| 模式 | CH1 | CH2 | CH3 | CH4 | CH5~CH8 |
|---|---|---|---|---|---|
| 正常 | 左右平移 | 前进/后退 | 机身高度 | 原地转向 | 解锁 / 步态 / 站立姿态 / 平衡模式 |
| 平衡 | 机身横滚 | 机身俯仰 | 机身高度 | 机身偏航 | 同上 |

表中 CH1~CH8 是**默认**映射，全部可在网页上改（「遥控」页改摇杆通道、「模式」页改
开关）：摇杆通道填 `0~19`（`!CFG ch_fwd <0-19>`），开关是「通道 + 低/中/高档位」的
模式矩阵。**遥控器与手柄共用这一套** —— 手柄的摇杆与按键先换算成同一套通道号，
再走同一段映射，改完立即生效、`!CFGW` 掉电保持。
出厂默认方向取反全 0；若某个方向与手感相反，翻「遥控」页的「方向取反」
（如 `!CFG inv_str 1`），不要改通道号。
输入源由参数 `input_mode` 决定（`!MODE crsf|ps2` 或网页按钮切换，`!CFGW` 掉电保持），
`INPUT_CONTROL_MODE` 只是出厂默认值。PS2 完整按键映射见 STATUS.md。

## 结构

```
hexapod_robot/
├── pico/        # RP2040 固件 (HAL 结构: Inc/Src)
├── tools/       # 串口控制台 serial_console.py (+ TCP 桥接) · IK 仿真 ik_gait_debug.py
│              # webconfig/ 网页配置台 · tcp_monitor.py 远程客户端 · 无硬件测试
├── hardware/    # PCB 设计/机械/BOM (见 hardware/README.md)
├── README.md
└── STATUS.md    # 固件细节: GPIO/参数/步态/调试/电路保护
```

## 状态

> ⚠️ **项目仍在持续开发中，功能和接口可能随时变动。**

## License

固件: GNU GPLv3（见 [LICENSE](LICENSE)）。硬件文件授权另行约定（见 [hardware/README.md](hardware/README.md)）。

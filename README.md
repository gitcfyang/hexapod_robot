# Hexapod Robot

六足机器人，基于 Raspberry Pi Pico (RP2040)。18 路数字舵机（2× PCA9685）、
BNO055 IMU 自动调平，支持 ELRS CRSF / PS2 手柄 / USB 三种输入。

## 特性

- 100Hz 控制循环 + 实时 IK 解算；四种步态（三角 6/8、波纹、波浪）+ 仿生连续变速
- 平衡模式（机身姿态直接控制）+ IMU 姿态补偿自动调平（可选）
- CRSF / PS2 双驱动常编译，`!MODE` 运行时切换，无需焊接插拔
- 电池保护（过压/低压/过放自动断舵机供电）、硬件看门狗、I2C 总线自恢复
- PCB 设计、Gerber、全项目 BOM、机械 STEP 全部开源（见 [hardware/](hardware/)）

## 快速开始

```bash
cd pico/build && make -j$(nproc)
```

编译产物 `hexapod_pico.uf2` 拖入 Pico 的 USB 盘即烧录。
调试控制台：`python3 tools/serial_console.py`（自动连接、断线重连）。

## 硬件概览

- MCU: Pico (RP2040) · 舵机: 18× 双轴数字舵机, 2× PCA9685 · IMU: BNO055 (I²C 0x29)
- 电池: 2S 18650, GP28 ADC2 分压 47/377 (330k+47k)，低压/过压/过放自动断舵机供电
  （当前 `BATTERY_CHECK_ENABLED=0`，待新板实测分压后启用）
- 足端微动开关 ×6 · 无源蜂鸣器 · 双 LED · 直流电机 ×2
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

输入源由 `INPUT_CONTROL_MODE` 编译期选择（0=CRSF, 1=PS2, 2=USB），
无线电模式下可用 `!MODE crsf|ps2` 运行时切换。PS2 完整按键映射见 STATUS.md。

## 结构

```
hexapod_robot/
├── pico/        # RP2040 固件 (HAL 结构: Inc/Src)
├── tools/       # 串口控制台 serial_console.py · IK 仿真 ik_gait_debug.py
├── hardware/    # PCB 源工程/Gerber/BOM/机械 STEP (规范见 hardware/README.md)
├── README.md
└── STATUS.md    # 固件细节: GPIO/参数/步态/调试/电路保护
```

## 状态

> ⚠️ **项目仍在持续开发中，功能和接口可能随时变动。**

## License

固件: GNU GPLv3（见 [LICENSE](LICENSE)）。硬件文件授权另行约定（见 [hardware/README.md](hardware/README.md)）。

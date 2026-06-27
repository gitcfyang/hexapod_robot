# Hexapod Robot

六足机器人固件，基于 Raspberry Pi Pico (RP2040)。

## 硬件

- **MCU**: Raspberry Pi Pico (RP2040)
- **舵机**: 18 路数字舵机，2× PCA9685 驱动
- **接收器**: ELRS CRSF (UART)
- **IMU**: BNO055 9轴姿态传感器 (I²C 0x28, 可选)
- **I²C**: I2C1, GP14 (SDA) / GP15 (SCL) @400kHz
  - ⚠️ **PCA9685 必须用 3.3V 供电！** 5V 供电会导致 I2C 电平倒灌烧毁 RP2040 GPIO
  - GP14/GP15 在 PCB 上靠近 PCA9685 布局，减少走线干扰
- **足端传感器**: 6× 微动开关，GP16~GP21 (输入上拉，落地→GND)
- **蜂鸣器**: GP13 (有源蜂鸣器，高电平驱动)
- **腿节**: Coxa 45mm / Femur 75mm / Tibia 120mm

## 结构

```
hexapod_robot/
├── README.md
├── STATUS.md                         # 详细固件状态与参数说明
├── pico/                             # RP2040 固件
│   ├── CMakeLists.txt                # CMake 构建配置
│   ├── hexapod_pico.c                # 入口：主循环、调试输出
│   ├── Inc/                          # 头文件
│   │   ├── hexapod_config.h          #   机械参数、舵机映射、CRSF 通道
│   │   ├── hexapod_types.h           #   数据结构定义
│   │   ├── hexapod_core.h            #   核心控制与机器人实例
│   │   ├── hexapod_ik.h              #   逆运动学 (IK)
│   │   ├── hexapod_gait.h            #   步态引擎
│   │   ├── hexapod_math.h            #   三角函数查表
│   │   ├── hexapod_crsf.h            #   CRSF 协议解析
│   │   ├── hexapod_hal.h             #   硬件抽象层接口
│   │   ├── hexapod_i2c_protocol.h    #   PCA9685 I²C 协议
│   │   └── bno055.h                  #   BNO055 IMU 寄存器映射
│   └── Src/                          # 实现文件
│       ├── hexapod_core.c            #   控制循环、IK 集成、转向步态
│       ├── hexapod_ik.c              #   单腿 IK + 机身旋转矩阵
│       ├── hexapod_gait.c            #   波纹/三角/波浪步态序列
│       ├── hexapod_math.c            #   sin/cos/acos/atan4 查表实现
│       ├── hexapod_crsf.c            #   ELRS 通道解析、摇杆映射
│       ├── hexapod_hal_pico.c        #   Pico 硬件驱动 (I²C/UART/舵机/IMU/足端开关)
│       ├── hexapod_i2c_protocol.c    #   PWM 校准、批量写入
│       └── bno055.c                  #   BNO055 I²C 驱动 (NDOF 融合)
└── tools/
    ├── ik_gait_debug.py              # IK + 步态 Python 仿真可视化
    └── serial_console.py             # USB 串口交互控制台 (舵机微调/步态/姿态切换)
```

## 控制

| 模式 | CH1 (Roll) | CH2 (Pitch) | CH3 (Throttle) | CH4 (Yaw) |
|---|---|---|---|---|
| 正常 | 左右平移 | 前进/后退 | 机身高度 | 原地转向 |
| 平衡 | 机身横滚 | 机身俯仰 | 机身高度 | 机身偏航 |

- CH5: 解锁/上电
- CH6: 步态切换 (低=三角6 / 中=三角8 / 高=波浪24)
- CH7: 站立姿态 (低=窄80% / 中=正常100% / 高=宽120%)
- CH8: 平衡模式开关

## IMU 姿态补偿 (可选)

启用 `IMU_ENABLED=1` 后，BNO055 实时测量机身倾斜角度，通过 `body_rot_offset` 自动叠加到 IK 解算，实现机身自动调平：

```
BNO055 (I²C 0x28) → 欧拉角读取 → body_rot_offset (取反)
                                               ↓
CRSF 摇杆 → body_rot ─────────────→ [ + ] → IK 旋转矩阵 → 舵机
```

- Roll/Pitch 补偿生效，Yaw 不补偿（避免与转向冲突）
- 两种模式（正常/平衡）均受益
- 传感器未检测到时自动降级，不影响基本功能

## 编译

```bash
cd pico/build && make -j$(nproc)
```

编译产物 `hexapod_pico.uf2` 拖入 Pico 的 USB 盘即可烧录。

## 调试

USB CDC 串口命令：

| 命令 | 功能 |
|---|---|
| `!C` | 交互式舵机校准 |
| `!V` | 切换调试等级 (0-3) |
| `!P<id> <ang>` | 单舵机角度控制 |
| `!O` / `!S` | 解锁 / 停止 |
| `!F` / `!B` / `!L` / `!R` | 方向移动 |
| `!G<n>` | 步态切换 (0-4) |
| `!M<n>` | 站立姿态 (-1=窄, 0=正常, 1=宽) |
| `!A` | 打印全部 18 路舵机角度 |

快捷用法: `python3 tools/serial_console.py` 可直接输入 `<id> <angle>` (自动加 `!P` 前缀)

详见 [STATUS.md](STATUS.md)。

## 状态

> ⚠️ **项目仍在持续开发中，功能和接口可能随时变动。**

## License
This project is licensed under the GNU General Public License v3.0 — see [LICENSE](LICENSE) for details.

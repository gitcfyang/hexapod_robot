# Hexapod Robot

六足机器人固件，基于 Raspberry Pi Pico (RP2040)。

## 硬件

- **MCU**: Raspberry Pi Pico (RP2040)
- **舵机**: 18 路数字舵机，2× PCA9685 驱动
- **接收器**: ELRS CRSF (UART)
- **腿节**: Coxa 45mm / Femur 75mm / Tibia 120mm

## 控制

| 模式 | CH1 (Roll) | CH2 (Pitch) | CH3 (Throttle) | CH4 (Yaw) |
|---|---|---|---|---|
| 正常 | 左右平移 | 前进/后退 | 机身高度 | 原地转向 |
| 平衡 | 机身横滚 | 机身俯仰 | 机身高度 | 机身偏航 |

- CH5: 解锁/上电
- CH6: 步态切换（波纹 / 三角 / 波浪）
- CH8: 平衡模式开关

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

详见 [STATUS.md](STATUS.md)。

## 状态

> ⚠️ **项目仍在持续开发中，功能和接口可能随时变动。**

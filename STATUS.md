# 六足机器人 固件状态

## 硬件
- MCU: Raspberry Pi Pico (RP2040)
- 舵机: 18路, 2× PCA9685 (仅用1块, 地址0x40, 右腿9路 ID 0-8)
- PCA9685: I²C1 GP2/GP3 @400kHz, 内部上拉
- PWM: 100Hz 数字舵机, 两片 PCA9685 独立校准
- 接收器: ELRS CRSF, UART1 GP4/GP5 @420000bps
- IMU: BNO055 9轴姿态传感器, I²C1 GP2/GP3 (共享), 地址 0x28 (可选)
- 腿: coxa=45mm, femur=75mm, tibia=120mm
- 舵机零位: FEMUR_SERVO_ZERO=450, TIBIA_SERVO_ZERO=900
- 底板: coxa基座高出底板25mm

## IK 公式 (hexapod_ik.c)
```
S_c = ±(atan4(z,x) - COXA_ANGLE) + horn
d   = √(x²+z²) - Lc
a   = √(d²+y²)
S_t = ±(TIBIA_ZERO - acos((Lf²+Lt²-a²)/(2·Lf·Lt))) + horn
S_f = ±(acos((Lf²+a²-Lt²)/(2·Lf·a)) - atan4(y,d) - FEMUR_ZERO) + horn
     右腿 invert(取反), 左腿不变
```

## 关键参数 (hexapod_config.h)
- 站立: INIT_Y=50mm
- COXA_ANGLE: RR=1350, RM=900, RF=450 (左镜像)
- FOOT_DX: RR=-78, RM=0, RF=78 (左镜像)
- FOOT_DZ: RR=78, RM=110, RF=78 (左镜像)
- 步长: TRAVEL_MAX_FORWARD=140mm, STRAFE=60mm, TURN=60mm
- 抬腿: LIFT_HEIGHT 5-60mm
- 机身高度调节: BODY_HEIGHT_RANGE_MM=±20mm

## 仿生连续变速
- 摇杆幅度连续映射到步态周期 (替代原设计 CH7 三段开关)
- GAIT_PERIOD_MAX=180ms → MIN=45ms
- 低摇杆 = 短步长 + 低频率, 高摇杆 = 大步长 + 高频率
- CH7 保留为可选频率范围倍率 (SPEED_SWITCH_ENABLED=0)

## 控制链路
```
正常模式:
  CRSF CH1-8 → 两级死区 → 仿生连续变速 → 子步态插值(1200微步)
  → [IMU body_rot_offset 取反叠加] → IK → servo batch → I²C (PCA9685) → 100Hz PWM

平衡模式:
  CRSF CH1-8 → body_rot + body_pos.y 直接映射 (全部取反, 方向已校准)
  → [IMU body_rot_offset 取反叠加] → IK (standing) → servo batch → I²C → 100Hz PWM
```

## 平衡模式 body_rot 坐标
- X=前 Y=上 Z=右 (右手定则)
- body_rot.x = Roll 横滚 (Rx, 绕 X 前进轴, 摇杆 CH1)
- body_rot.y = Yaw 偏航 (Ry, 绕 Y 垂直轴, 摇杆 CH4)
- body_rot.z = Pitch 俯仰 (Rz, 绕 Z 左右轴, 摇杆 CH2)
- 三轴均取反，使摇杆方向与机身倾斜方向直觉一致
- 旋转顺序: Ry(Yaw) → Rx(Roll) → Rz(Pitch)

## IMU 姿态补偿
- BNO055 9轴 IMU, 与 PCA9685 共享 I²C1 (GP2/GP3), 地址 0x28
- 工作模式: NDOF (9-DOF 融合), 100Hz 欧拉角输出
- 补偿通道: body_rot_offset (Roll/Pitch 取反, Yaw=0 不补偿)
- 叠加点: compute_leg_ik() 中 effective_body_rot = body_rot + body_rot_offset
- 启用: IMU_ENABLED=1 (hexapod_config.h)
- 增益: IMU_COMPENSATION_GAIN (×10, 默认 10=1:1 直接补偿)
- 驱动: bno055.h/c, 通过 hal_imu_init()/hal_imu_read() 封装
- 容错: 传感器未检测到时打印警告, 固件正常运行 (无补偿)
- 坐标映射: BNO055 Roll(Y轴) → Robot Roll(X轴), BNO055 Pitch(X轴) → Robot Pitch(Z轴)
-           (具体正负号取决于 IMU 安装方向, 在 hal_imu_read() 中调整)

## PWM 校准
- 用 PWM_PERIOD_US (每板实测周期) 直接计算脉宽计数值
- 不依赖 PCA9685 振荡器精度
- `pulse_to_count = pulse_us × 4096 / pwm_period_us`
- 当前: 200Hz → PWM_PERIOD_US_BOARD0/BOARD1 (示波器实测后填入)
- 两块 PCA9685 使用独立的校准值，消除器件间振荡器差异
- I²C 写入失败检测 + 自动重试 + 连续 3 次失败自动总线恢复 (I2C spec 3.1.16)

## 控制频率
- 控制循环: 100Hz (CONTROL_LOOP_PERIOD_MS=10ms)
- PCA9685 PWM: 100Hz
- IK 解算: 100Hz (原 50Hz)，每次更新每帧 PWM 都有新角度
- I2C 批量写入: ~1.5ms/板，占空比 ~30% (200Hz 下 5ms 周期)
- 第二块 PCA9685 接入后 I2C 占空比 ~60%，仍有 2ms IK 计算余量（200hz下）

## 输入控制模式
- 编译期开关: `INPUT_CONTROL_MODE` (hexapod_config.h)
  - `0` = CRSF 接收器 (ELRS, UART1 @420000 baud) — 默认
  - `1` = USB CDC 串口命令 (USB 虚拟串口, 无需 UART1)
- 两种模式互斥，无运行时冲突
- USB 串口命令列表: `!F` `!B` `!L` `!R` `!Q` `!E` `!S` (运动), `!O` (解锁), `!G<n>` (步态), `!T` (平衡), `!U/!D` (抬腿), `!V` (调试), `!C` (校准), `!P` `!W` `!Z` `!A` (舵机直控)

## 鲁棒性保护 (watchdog-i2c-recovery)

### 硬件看门狗
- RP2040 硬件 Watchdog，超时 1500ms
- 主循环每迭代喂狗
- 任何死锁 (I2C 卡死 / ISR 风暴) 在 1.5s 内自动复位
- 调试器挂接时自动暂停 (pause_on_debug=1)

### I2C 总线恢复
- 连续 3 次 I2C 写入失败 → 自动执行总线恢复 (`pca9685_i2c_recover`)
- 恢复流程: 检测 SDA 是否被拉低 → 发送 9 个 SCL 脉冲释放 → STOP 条件 → 重新初始化 I2C
- 标准 I2C 规范 3.1.16 流程，耗时约 1-2ms
- 成功后连续失败计数归零，正常继续

### 电路保护建议 (防止 ESD 击穿)
- **已发生事件**: 2026-06 手触碰板子 → ESD 静电击穿外部 QSPI Flash 芯片 → Pico 无法启动 (仅能进 BOOTSEL 大容量模式)
- **最小保护方案** (成本 < ¥5):
  - I2C SDA/SCL (GP2/GP3) 各串联 100Ω 限流电阻
  - 3.3V 对地并联 TVS 管 (SMAJ3.3A / PESD3V3)
  - 板子装盒或贴绝缘胶带，裸露 GPIO 是 ESD 直接入口
- **强烈建议**:
  - 舵机独立供电 (2S LiPo → 7.4V → 舵机电源轨)，Pico 单独供电
  - PCA9685 端 SDA/SCL 各上拉 2.2kΩ 到 PCA9685 VCC，限制反灌电流
  - 3.3V 输出对地加 100μF 电解 + 0.1μF 陶瓷，抑制舵机 EMI
- **进阶**: SRV05-4 ESD 阵列 / I2C 光耦隔离 (ADUM1250) / 共模扼流圈

## 步态
- RIPPLE_12 (默认): 4抬腿+8支撑=12步, 占空比67%
- TRIPOD_6: 2抬腿+4支撑=6步, 交替三脚
- WAVE_24: 3抬腿+20支撑=24步, 极慢波浪
- TRIPOD_8 / TRIPOD_4: 备选
- 抬腿高度: 0→峰值→0 二次曲线 (无跳变)
- 地面支撑: 位移取反 (与抬腿方向相反, 已修复)
- 子步态插值: 1 gait step = 100 sub-units, 连续轨迹

## 调试工具 （已修复）
- USB CDC 串口命令 (CRSF 模式下也可用)
- `!C`: 交互式舵机校准模式
- `!V`: 切换调试等级 (0-3)
- `!P<id> <ang>` / `!W<id>` / `!Z` / `!A`: 舵机直控
- `!O`: 解锁, `!F/B/L/R`: 运动, `!S`: 停止
- tools/ik_gait_debug.py: Python 仿真可视化

## 待完善
- [ ] 用 !C 实测每条腿的 horn_offset
- [ ] 用 !W 实测舵机机械极限, 收紧 SERVO_xxx_MIN/MAX
- [ ] 电池 ADC 分压器 + BATTERY_CHECK_ENABLED
- [ ] Core1 双核卸载 IK 计算
- [x] 加入 IMU (BNO055 I²C 驱动 + body_rot_offset 姿态补偿) — 待实测验证
- [x] 硬件看门狗 + I2C 总线自动恢复
- [ ] Pico 换新 → 加电路保护 (I2C 串联电阻 + TVS + 绝缘)
- [ ] 加入 足端传感器，自动进行步态调整

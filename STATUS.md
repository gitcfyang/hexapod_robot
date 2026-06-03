# 六足机器人 固件状态

## 硬件
- MCU: Raspberry Pi Pico (RP2040)
- 舵机: 18路, 2× PCA9685 (仅用1块, 地址0x40, 右腿9路 ID 0-8)
- PCA9685: I²C1 GP2/GP3 @400kHz, 内部上拉
- PWM: 100Hz 数字舵机, PWM_PERIOD_US=8475 (实测校准)
- 接收器: ELRS CRSF, UART1 GP4/GP5 @420000bps
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
- 站立: INIT_Y=80mm
- COXA_ANGLE: RR=1350, RM=900, RF=450 (左镜像)
- FOOT_DX: RR=-78, RM=0, RF=78 (左镜像)
- FOOT_DZ: RR=78, RM=110, RF=78 (左镜像)
- 步长: TRAVEL_MAX_FORWARD=140mm, STRAFE=60mm, TURN=60mm
- 抬腿: LIFT_HEIGHT 10-60mm
- 机身高度调节: BODY_HEIGHT_RANGE_MM=±20mm

## 仿生连续变速
- 摇杆幅度连续映射到步态周期 (替代 CH7 三段开关)
- GAIT_PERIOD_MAX=180ms → MIN=70ms
- 低摇杆 = 短步长 + 低频率, 高摇杆 = 大步长 + 高频率
- CH7 保留为可选频率范围倍率 (SPEED_SWITCH_ENABLED=0)

## 控制链路
```
正常模式:
  CRSF CH1-8 → 两级死区 → 仿生连续变速 → 子步态插值(1200微步)
  → IK → servo batch → I²C (PCA9685) → 100Hz PWM

平衡模式:
  CRSF CH1-8 → body_rot(pitch/roll/yaw) + body_pos.y 直接映射
  → IK (standing) → servo batch → I²C → 100Hz PWM
```

## PWM 校准
- 用 PWM_PERIOD_US (实测周期) 直接计算脉宽计数值
- 不依赖 PCA9685 振荡器精度
- `pulse_to_count = pulse_us × 4096 / PWM_PERIOD_US`
- 当前: 100Hz → PWM_PERIOD_US=8475
- I²C 写入失败检测 + 自动重试 + 告警

## 步态
- RIPPLE_12 (默认): 4抬腿+8支撑=12步, 占空比67%
- TRIPOD_6: 2抬腿+4支撑=6步, 交替三脚
- WAVE_24: 3抬腿+20支撑=24步, 极慢波浪
- TRIPOD_8 / TRIPOD_4: 备选
- 抬腿高度: 0→峰值→0 二次曲线 (无跳变)
- 地面支撑: 位移取反 (与抬腿方向相反, 已修复)
- 子步态插值: 1 gait step = 100 sub-units, 连续轨迹

## 调试工具
- USB CDC 串口命令 (CRSF 模式下也可用)
- `!C`: 交互式舵机校准模式
- `!V`: 切换调试等级 (0-3)
- `!P<id> <ang>` / `!W<id>` / `!Z` / `!A`: 舵机直控
- `!O`: 解锁, `!F/B/L/R`: 运动, `!S`: 停止
- tools/ik_gait_debug.py: Python 仿真可视化

## 待完善
- [ ] 用 !C 实测每条腿的 horn_offset
- [ ] 用 !W 实测舵机机械极限, 收紧 SERVO_xxx_MIN/MAX
- [ ] 第二块 PCA9685 (左腿9路 ID 9-17)
- [ ] 电池 ADC 分压器 + BATTERY_CHECK_ENABLED
- [ ] Core1 双核卸载 IK 计算
- [ ] 外部 I²C 上拉电阻 (当前仅内部~50kΩ, 400kHz 临界)

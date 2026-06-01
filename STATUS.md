# 六足机器人 固件状态

## 硬件
- MCU: Raspberry Pi Pico (RP2040)
- 舵机: 18路, 2× PCA9685 (仅用1块, 地址0x40)
- 接收器: ELRS CRSF, UART1 GP4/GP5 @420000bps
- 腿: coxa=45mm, femur=75mm, tibia=120mm
- 舵机零位: femur离垂直45°(向上), tibia⊥femur90°
- 底板: coxa基座高出底板25mm, 舵机全0°时底板离地≈7mm

## IK 公式 (hexapod_ik.c:43-111)
```
S_c = ±(atan4(z,x) - COXA_ANGLE) + horn
d   = √(x²+z²) - Lc
a   = √(d²+y²)
S_t = ±(T0 - acos((Lf²+Lt²-a²)/(2·Lf·Lt))) + horn
S_f = ±(acos((Lf²+a²-Lt²)/(2·Lf·a)) - atan4(y,d) - F0) + horn
     右腿 invert(取反), 左腿不变
```

## 关键参数 (hexapod_config.h)
- FEMUR_SERVO_ZERO=450, TIBIA_SERVO_ZERO=900
- 站立: FOOT_REACH=110mm, INIT_Y=80mm
- 休息: FOOT_REACH_REST=182mm, INIT_Y_REST=31mm
- COXA_ANGLE: RR=1350, RM=900, RF=450 (左镜像)
- 步态: GAIT_STEP_PERIOD_MS=60 (默认), 舵机刷新20ms

## 控制链路
CRSF CH1-8 → crsf_to_control() → travel_length → smooth减速 → gait步进 → IK → 舵机

## 待完善
- [ ] 用 !P 实测每条腿的 horn_offset
- [ ] 用 !W 实测舵机机械极限, 收紧 SERVO_xxx_MIN/MAX
- [ ] 第二块 PCA9685 (左腿9路)
- [ ] 电池 ADC 分压器 + BATTERY_CHECK_ENABLED
- [ ] 行走中姿态微调 (body_rot 平衡)
- [ ] Core1 双核卸载 IK 计算

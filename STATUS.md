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
正常:  CRSF CH1-8 → 两级死区 → linear/smooth → gait步进 → IK → 舵机
平衡:  CRSF CH1-8 → 两级死区 → body_rot + body_pos.y直接映射 → IK → 舵机 (不行走)

## 待完善
- [x] 舵机horn_offset校准工具 (!C 交互式校准模式)
- [x] 接收机输入两级死区处理 (hexapod_config.h:122-128)
- [ ] 用 !P 实测每条腿的 horn_offset
- [ ] 用 !W 实测舵机机械极限, 收紧 SERVO_xxx_MIN/MAX
- [ ] 第二块 PCA9685 (左腿9路)
- [ ] 电池 ADC 分压器 + BATTERY_CHECK_ENABLED
- [x] 接收机输入两级死区处理 (hexapod_config.h:122-128)
- [x] 平衡模式机身姿态控制 (body_rot pitch/roll/yaw + 线性高度)
- [x] 正常模式下机身高度改为线性直接映射 (替代积分器)
- [x] 修复 zero_debounce 循环重触发导致大腿持续摆动 (force_gait_step_cnt 无限重入)
- [x] 修复站立时步态序列保持抬腿相位导致 femur 持续偏离 (compute_leg_ik stand_still)
- [x] 优化步态参数: 抬腿 80→40mm, 步长 100→160mm, 平移 80→120mm, 旋转 60→90mm
- [ ] Core1 双核卸载 IK 计算

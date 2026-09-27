/**
 * @file hexapod_config.h
 * @brief 六足机器人配置文件
 * @note 用户应根据实际机器人参数修改此文件
 *
 * ★ 两类参数, 改法不同:
 *
 *   1. 硬件/机械常量 (引脚、腿节长度、舵机零位、安装角、舵机 ID 映射)
 *      —— 直接改下面的数字。改了要重新编译烧录。
 *
 *   2. 运行时可调参数 (电池阈值、步长、死区、方向取反、功能开关…)
 *      —— 形如 `#define X (g_params.x)`, 实际值存在 RAM 里, 可用
 *          !CFG / !CFGR / !CFGW 串口改, 或在网页 Configurator 里拖滑块。
 *          它们各自的默认值写在同一行的 `X_DEFAULT` 宏里 ——
 *          换板/换机架时改那个值即可, 复位后即为新默认。
 *          详见 hexapod_params.h。
 *
 *   两类都以 `_DEFAULT` 结尾的宏 = 上面第 2 类的"出厂默认值"。
 */

#ifndef HEXAPOD_CONFIG_H
#define HEXAPOD_CONFIG_H

#include "hexapod_types.h"
#include "hexapod_params.h"   /* 运行时参数实体 g_params (见文件头说明) */

/* ==================== 固件标识 ==================== */

/* 固件版本 (横幅 + !VER + 网页开始页显示), 取自构建时生成的 hexapod_version.h
 * (git describe: 打 tag 处 = "v0.4.0", tag 之后 = "v0.3.0-16-g8f5b04f", 工作区
 * 有未提交改动再加 "-dirty")。版本历史只存在于 git tag, 不手工维护第二个号。
 * 不在 CMake 构建里编译时 (无生成头) 退化为 unknown。 */
#if __has_include("hexapod_version.h")
#include "hexapod_version.h"
#else
#define HEXAPOD_FW_VERSION "unknown"
#endif

/* ==================== 机械参数配置 ==================== */

/*
 * 坐标系统 (右手定则)：
 *
 *          Z+ (右)
 *          │
 *     LF   │   RF      机身俯视图，机头朝 X+
 *      ╲   │   ╱
 *    LM ───○─── RM      ○ = 机身几何中心 (原点)
 *      ╱   │   ╲
 *     LR   │   RR
 *          │
 *          └────── X+ (前)
 *
 *   Y+ = 垂直向上 (机身抬升方向)
 *
 * 每条腿 3 个关节，从机身向外依次为：
 *   Coxa (基节) → 水平旋转，绕 Y 轴
 *   Femur (股节) → 垂直旋转，绕 Z 轴
 *   Tibia (胫节) → 垂直旋转，绕 Z 轴
 */

/* ---- 腿节长度 (mm) ----
 * 从关节转轴中心到下一个关节转轴中心的直线距离。
 * 用量具实测，精确到 mm。
 *
 *   机身 ──[Coxa: 水平]──[Femur: 竖直]──[Tibia: 竖直]── 足端
 *          ←─ L_coxa ─→←── L_femur ──→←── L_tibia ──→
 */
#define LEG_COXA_LENGTH     45
#define LEG_FEMUR_LENGTH    75
#define LEG_TIBIA_LENGTH    120

/* ---- 舵机零位参考 (0.1度单位) ----
 *
 * 这两个参数将 IK 几何角度映射为舵机输出。
 * 设定为使「所有舵机 0° = 站立姿态(足端刚好触地)」。
 *
 * 公式: femure_servo  = (angle_a1 + angle_a2) - FEMUR_SERVO_ZERO
 *       tibia_servo   = TIBIA_SERVO_ZERO - acos(膝角余弦)
 *
 * 你的硬件站立几何:
 *   ik_feet=137mm, pos_y=31mm, ik_a≈140.5mm
 *   femur_total = α_2 - α_1 (股节在足端线上方)
 *               = acos((75²+140.5²-120²)/(2×75×140.5)) - atan4(31,137)
 *               ≈ 58.7° - 12.7° = 46.0°
 *   → FEMUR_SERVO_ZERO = 450 (舵机0°→股节45°, 差1°在horn_offset补)
 *   膝角 = acos((75²+120²-140.5²)/(2×75×120)) ≈ 89.1° → TIBIA_SERVO_ZERO = 900
 */
#define FEMUR_SERVO_ZERO    450     /* 舵机0° → 股节离垂直45° = 离水平45° */
#define TIBIA_SERVO_ZERO    900     /* 舵机0° → 膝角90° */

/* ==================== 安全配置 ==================== */

/* 电池电压保护总开关：设为 1 启用，设为 0 完全禁用
 * 此开关统一控制三处电压保护:
 *   1. 上电启动检测 (hexapod_pico.c) — 电压异常拒绝舵机供电
 *   2. 运行电压监测 (hexapod_pico.c) — 每 1s 分级报警/断电
 *   3. 核心循环检查 (hexapod_core.c) — 紧急停止
 * 设为 0 时: 跳过 ADC 读取, 舵机供电无条件开启, 无任何电压报警。
 * ⚠️ 新 PCB (2026-08) 已将 ADC 改至 GP28 (绕开旧板损坏的 GP26) +
 *   VBAT 入口 TVS + ADC 输入齐纳钳位。
 * ★ 运行时可改: !CFG batt_check 1 /  网页「参数 → 功能开关」 */
#define BATTERY_CHECK_ENABLED_DEFAULT   0   /* ★ 待新 PCB 实测分压电路后启用 */
#define BATTERY_CHECK_ENABLED           (g_params.batt_check)

/* ==================== 电池检测配置 (GP28 ADC2, 2S 18650) ==================== */

/* 分压电阻: R1=330kΩ (电池+), R2=47kΩ (地), ADC 抽头在 R2
 *   (新 PCB 2026-08 由 100k+15k 改来: 串阻提高到 330k, 电压尖峰时
 *    流入 ADC 的电流被进一步限制, 与齐纳钳位配合更安全)
 * ADC 电压 = 电池电压 × R2/(R1+R2) = 电池 × 47/377
 * → 电池电压 = ADC 电压 × 377/47 ≈ 8.02×
 * 电池 8.4V (满) → ADC 1.05V;  6.6V (空) → ADC 0.82V  (安全范围)
 * 新 PCB (2026-08): 电池 ADC 从 GP26 改至 GP28 (ADC2),
 *   顺带绕开旧板被电压尖峰损坏的 GP26 ADC 通道 (见 STATUS 事件 3) */
#define BATTERY_ADC_PIN         28
#define BATTERY_ADC_INPUT       2
#define ADC_REF_VOLTAGE         3300
#define ADC_RESOLUTION          4095

/* 分压比: 理论值 377/47 = 8.021, 实测标定为 8.134
 *   单点标定 (2026-09-27, !BATT): 万用表 8.37V vs 引脚 1029mV
 *   → 理论引脚应为 1043mV, 实测低 -1.4%。误差源: 330k||47k ≈ 41k 的
 *   源阻抗令 ADC 采样保持电容充电不足 (RP2040 ADC 建议源阻抗 10k 量级)
 *   + 电阻容差 + 3.3V 参考(即 3.3V 轨)自身容差。
 *   标定把这个固定比例误差折算进比值 —— 注意它同时把 3.3V 轨的偏差
 *   一并折进来了, 换板/换稳压方案后需重新标定。
 *   标定后读取偏低的方向对应"截止偏早", 属安全侧。
 * ★ 运行时可改: !CFG batt_ratio_milli 8134 (×1000, 现场二次标定用) */
#define BATTERY_DIVIDER_RATIO_DEFAULT   8.134f    /* 标定值 (理论 377/47 = 8.021) */
#define BATTERY_DIVIDER_RATIO           (0.001f * (float)g_params.batt_ratio_milli)

/* 2S 18650 电压阈值 (mV)
 *   8.4V = 充满 (4.2V/节)
 *   8.8V = 过压保护阈值 — 断开舵机供电 + 红灯蜂鸣报警
 *          (高于满电 8.4V; 可捕获误接 3S 电池 9V+ 或电源故障)
 *   7.4V = 标称
 *   7.0V = 低电压警告 (3.5V/节) — 红灯闪烁 + 蜂鸣提示, 建议尽快回充
 *   6.6V = 保护截止 (3.3V/节) — 断开舵机供电, 防止过放损坏电池
 *
 * ★ 四个阈值运行时可改 (!CFG batt_ov_mv / batt_warn_mv / batt_cutoff_mv /
 *   batt_recov_mv), 网页「参数 → 电池」页也有滑块。 */
#define BATTERY_OVERVOLTAGE_MV_DEFAULT  8800
#define BATTERY_WARNING_MV_DEFAULT      7000
#define BATTERY_CUTOFF_MV_DEFAULT       6600
#define BATTERY_RECOVERY_MV_DEFAULT     7300    /* 回升到此值以上才重新接通舵机供电 */
#define BATTERY_OVERVOLTAGE_MV          (g_params.batt_ov_mv)
#define BATTERY_WARNING_MV              (g_params.batt_warn_mv)
#define BATTERY_CUTOFF_MV               (g_params.batt_cutoff_mv)
#define BATTERY_RECOVERY_MV             (g_params.batt_recov_mv)

/* 电压监测间隔 (ms) — 运行时 !CFG batt_interval_ms */
#define BATTERY_CHECK_INTERVAL_MS_DEFAULT   1000
#define BATTERY_CHECK_INTERVAL_MS           (g_params.batt_interval_ms)

/* 未接电池判定门限 (mV) —— 低于此值视为"分压抽头悬空", 即 USB 供电, 非故障。
 *
 * 为什么不能用截止电压代替: 未接电池时抽头被 R2 (47k) 拉到 ~0mV, 会被
 * `v < cutoff` 判成过放 → 假报警 + 停机。而 USB 供电调试是最常用的台面场景。
 * 2S 18650 即使过放到保护板切断也不会低于 ~5V, 所以 3V 这条线两边都很宽裕。
 *
 * ★ 运行时可改: !CFG batt_absent_mv 3000 (现场按实测调整)
 *   若新板 USB 供电时读数偏高 (分压前级漏电流), 调高此值即可。 */
#define BATTERY_ABSENT_MV_DEFAULT           3000
#define BATTERY_ABSENT_MV                   (g_params.batt_absent_mv)

/* 判定"电压已落定"的门限 (mV) —— 趋势窗口内净跌幅超过它, 说明电压还在往下走,
 * 此时不判故障, 再看一轮。
 *
 * 为什么需要: 拔电池/拔电源后, 电池+ 节点上的储能电容经分压电阻缓慢放电,
 * 电压从 8.4V 一路衰减到 0 —— 中途必然穿过 6.6V 截止区。若只看"是否低于截止"
 * 就会在衰减途中误触发停机。而真过放的电池在负载下会"掉到某个值然后停住",
 * 所以判据是**是否还在下降**而不是降得多快: 快衰减/慢衰减一视同仁。
 *
 * 窗口取 4 次采样 (见 hal_battery_state 的 BATT_TREND_LEN), 所以这里 100mV 意为
 * "约 25mV/s 以上的净衰减"。分压 377kΩ 下储能电容要大到 700µF (τ≈264s) 才会
 * 慢到判不出来, 而到那时电压在截止区里能赖上几分钟, 早已不构成"拔了电池还在跑"
 * 的场景。采样离散度实测 ~12mV, 远在门限之下, 停住的电池不会被误判成"还在下降"。 */
#define BATTERY_SETTLE_MV                   100

/* ==================== IMU 姿态传感器配置 ==================== */

/* IMU 启用：1 = 启用 BNO055 姿态补偿，0 = 禁用
 * 启用后 I2C 总线上必须有 BNO055。
 * 若传感器未检测到，固件会打印警告并继续运行 (无补偿)。
 * ★ 运行时可改: !CFG imu_enabled 1 / 网页「参数 → 功能开关」。
 *   置 0 后不再读 IMU, 但 I2C 上仍会初始化一次 (失败不阻塞启动)。 */
#define IMU_ENABLED_DEFAULT     0
#define IMU_ENABLED             (g_params.imu_enabled)

/* BNO055 I2C 地址 (7-bit)
 *   COM3 接 GND → 0x28 (默认)
 *   COM3 接 VCC → 0x29
 *   PCB 实测: 本板 BNO055 应答在 0x29 (!I2C 检测确认) */
#define BNO055_I2C_ADDR         0x29

/* 安装方向: 正常安装 (芯片朝上, 2026-08 新 PCB 已修正倒扣失误)
 *   安装几何: Xc=rX(前), Yc=rZ(右), Zc=rY(上)
 *   (旧板倒扣为 Zc=-rY(下), 平放 chip pitch≈180° 需要 -1800 修正,
 *    新板正常安装后平放 chip pitch≈0, 修正项已删除)
 *   轴映射 (hal_imu_read):
 *     robot_roll  = IMU_ROLL_SIGN  × chip_pitch
 *     robot_pitch = IMU_PITCH_SIGN × chip_roll
 *   轴对应关系与旧板相同 (Xc=rX, Yc=rZ 未变), 仅去掉翻转偏移
 *   实测验证: 机器人平放 → !IMU 的 roll/pitch 应 ≈0;
 *   若补偿加剧倾斜 (正反馈) → 取反对应符号
 * ★ 两个符号运行时也能改 (!CFG imu_roll_sign / imu_pitch_sign),
 *   方便在真机上一次性试出正确极性, 不用反复烧录。 */
#define IMU_ROLL_SIGN_DEFAULT   -1
#define IMU_PITCH_SIGN_DEFAULT  +1
#define IMU_ROLL_SIGN           (g_params.imu_roll_sign)
#define IMU_PITCH_SIGN          (g_params.imu_pitch_sign)

/* ==================== IMU BOOT/INT 引脚 ==================== */

/* BOOT 引脚 (GP22, 新 PCB 2026-08 由 GP27 改来): 上电保持高电平
 * → BNO055 进入正常应用模式 (拉低则进入 bootloader 模式)。
 * 启用后: 固件启动时驱动 BOOT=高; I2C 初始化失败时自动循环
 * BOOT 引脚 (拉低 50ms → 释放 → 等待 POR) 并重试, 实现自动恢复 */
#define IMU_BOOT_GPIO_ENABLED   1
#define IMU_BOOT_PIN            22
#define IMU_INIT_RETRY_MAX      3    /* 初始化失败自动恢复重试次数 */

/* IMU 补偿增益 (×10, 10 = 1:1 直接补偿)
 * 增益 < 10 → 欠补偿 (响应平缓, 适合高速运动)
 * 增益 > 10 → 过补偿 (可能振荡, 需要调参)
 * ★ 运行时可改: !CFG imu_gain 8 */
#define IMU_COMPENSATION_GAIN_DEFAULT   10
#define IMU_COMPENSATION_GAIN           (g_params.imu_gain)

/* ==================== 调试配置 ==================== */

/* 调试输出等级 (通过 USB CDC 串口输出)：
 *   0 = 静默，仅输出启动信息和每 2s 状态摘要
 *   1 = 输入调试：打印 CRSF 链接状态 + 8 通道原始值
 *   2 = 舵机调试：打印 18 路舵机角度/脉宽
 *   3 = 全量调试：打印 IK 解算中间值（会产生大量输出，可能影响控制周期） */
#define DEBUG_LEVEL             0   /* ★ 生产模式：关闭调试输出 */

/* 调试输出间隔（毫秒），避免 USB 输出阻塞控制循环 */
#define DEBUG_PRINT_INTERVAL_MS 1000

/* 遥控通道遥测推送间隔 (毫秒)。网页遥控页的实时通道显示用, 与 DEBUG_LEVEL
 * 无关 —— 它是网页的数据源而不是调试输出。5Hz 在 115200 的 CDC 上无压力
 * (~40B × 5), 且被网页解析后不进日志面板。 */
#define CH_TELEM_INTERVAL_MS    200

/* 输入控制模式 (编译期默认输入源, 运行时可 !MODE 切换):
 *   0 = CRSF 接收器 (ELRS, UART1 @420000 baud) — PS2 驱动同步编译, 可 !MODE ps2 切换
 *   1 = PS2 手柄 (bit-bang SPI GP6~GP9)        — CRSF 同步编译, 可 !MODE crsf 切换
 *   2 = USB CDC 串口命令 (无遥控器, 无需额外硬件)
 * 一块板子同时接上 ELRS 和 PS2 接收器后, 无需焊接/插拔即可用 !MODE 切换输入源 */
#define INPUT_CONTROL_MODE      1   /* ★ 0=CRSF, 1=PS2, 2=USB */

/* 上电默认输入源 (运行时参数 input_mode, 可被 flash 里的记录覆盖)。
 * !MODE crsf|ps2 会同时写这个参数, 所以 !CFGW 后重启跟随上次的选择。
 * USB 构建 (INPUT_CONTROL_MODE==2) 没有遥控器, 参数保留但初始化时不读。 */
#if INPUT_CONTROL_MODE == 1
#define INPUT_MODE_DEFAULT      1   /* 双模构建默认 PS2, 与改动前一致 */
#else
#define INPUT_MODE_DEFAULT      0   /* CRSF/USB 构建 */
#endif
#define INPUT_MODE_RUNTIME      (g_params.input_mode)

/* 遥控门 (!RC) 自解锁超时 (毫秒): 锁定由网页 server 心跳 (!RC 0, ~2s) 维持,
 * 超过这个时间没再收到 !RC 就自动解锁 —— 上位机崩掉后遥控器必须能兜底操控,
 * 宁可解锁不可锁死。 */
#define RC_LOCK_TIMEOUT_MS      6000

/* 无舵机调试模式：PRODUCTION=0 要求舵机硬件就绪才启动 */
#define HEADLESS_MODE           0   /* ★ PS2 测试: 舵机/I2C 已死仍进入主循环 (测完改回 0) */

/* PCA9685 舵机板数量 (1 或 2)
 *   1 = 仅一块板 (地址 0x40)，控制左半身 9 路舵机 (ID 9-17)
 *   2 = 两块板 (地址 0x40 + 0x41)，控制全部 18 路舵机 (ID 0-17) */
#define PCA9685_BOARD_COUNT     2   /* 接好第二块板后改为 2 */

/* PS2 无线手柄支持 (自动派生, 通常无需手改):
 *   无线电模式 (INPUT_CONTROL_MODE 0/1) 恒为 1 — 双驱动常编译, 支持 !MODE 运行时切换;
 *   仅 USB 模式 (2) 为 0 (不编译 PS2 驱动, 零开销) */
#if INPUT_CONTROL_MODE == 2
#define PS2_ENABLED             0
#else
#define PS2_ENABLED             1
#endif

/* USB CDC 调试控制台开关 (与 USB 数据传输模式二选一):
 *   1 = USB CDC 作为调试控制台 (printf 输出 + !I2C/!P 等调试命令) — 无线电模式默认
 *   0 = USB CDC 专用于上位机数据传输 (INPUT_CONTROL_MODE==2): 调试输出全部静默,
 *       通道保持干净, 仅承载上位机协议数据 (输入命令解析不受影响)
 * 自动派生: 仅 USB 数据模式 (2) 下为 0, 其余模式恒为 1 */
#if INPUT_CONTROL_MODE == 2
#define USB_DEBUG_ENABLED       0
#else
#define USB_DEBUG_ENABLED       1
#endif

/* ==================== CRSF 通道映射 ====================
 *
 * 默认映射基于 ELRS 标准通道顺序。
 * 查看调试输出 [DBG1] 行的原始值来确定每个通道对应什么功能。
 *
 * CH1~CH4 在正常模式和平衡模式下功能不同：
 *
 *   正常模式 (CH8=低位):
 *     CH1 (Aileron/Roll):    左右平移 (Strafe)
 *     CH2 (Elevator/Pitch):  前进/后退 (Forward)
 *     CH3 (Throttle):        机身高度 (线性直接映射，无弹簧)
 *     CH4 (Rudder/Yaw):      原地旋转 (Turn)
 *
 *   平衡模式 (CH8=高位):
 *     CH1 (Aileron/Roll):    机身横滚 Roll
 *     CH2 (Elevator/Pitch):  机身俯仰 Pitch
 *     CH3 (Throttle):        机身高度 (线性直接映射)
 *     CH4 (Rudder/Yaw):      机身偏航 Yaw
 *     机器人原地不动，不做平移/旋转行走
 *
 *   CH5~CH8 开关在两种模式下功能相同:
 *
 * ★ 八个通道号运行时可改 (!CFG crsf_ch_fwd / crsf_ch_str / ...),
 *   网页「遥控 → 通道映射」填 0~15 (对应遥控器 CH1~CH16)。
 *   换遥控器/改 ELRS 通道顺序后不用重烧, 改完 !CFGW 即可掉电保持。
 *   取反仍是按功能的 (dir 组), 不随通道号走 —— 换通道后若方向反了,
 *   翻转对应的 inv_* 而不是调这里的通道号。 */
#define CRSF_CHANNEL_FORWARD_DEFAULT      1   // CH2: 正常=前进/后退, 平衡=俯仰
#define CRSF_CHANNEL_STRAFE_DEFAULT       0   // CH1: 正常=左右平移, 平衡=横滚
#define CRSF_CHANNEL_TURN_DEFAULT         3   // CH4: 正常=原地旋转, 平衡=偏航
#define CRSF_CHANNEL_HEIGHT_DEFAULT       2   // CH3: 正常=机身高度, 平衡=机身高度
#define CRSF_CHANNEL_ARM_DEFAULT          4   // CH5: 解锁 (二段开关)
#define CRSF_CHANNEL_GAIT_DEFAULT         5   // CH6: 步态 (三段开关)
#define CRSF_CHANNEL_SPEED_DEFAULT        6   // CH7: 站立姿态 (三段: -1=窄80%, 0=正常100%, +1=宽120%)
#define CRSF_CHANNEL_BALANCE_DEFAULT      7   // CH8: 平衡模式 (二段开关)

#define CRSF_CHANNEL_FORWARD      (g_params.crsf_ch_fwd)
#define CRSF_CHANNEL_STRAFE       (g_params.crsf_ch_str)
#define CRSF_CHANNEL_TURN         (g_params.crsf_ch_turn)
#define CRSF_CHANNEL_HEIGHT       (g_params.crsf_ch_hgt)
#define CRSF_CHANNEL_ARM          (g_params.crsf_ch_arm)
#define CRSF_CHANNEL_GAIT         (g_params.crsf_ch_gait)
#define CRSF_CHANNEL_SPEED        (g_params.crsf_ch_stance)
#define CRSF_CHANNEL_BALANCE      (g_params.crsf_ch_bal)

/* ---- 站立姿态缩放 (CH7) ----
 * ★ 三个缩放比 + 过渡速度/步长均运行时可改
 *   (!CFG stance_narrow / stance_normal / stance_wide / stance_speed /
 *    stance_step_mm), 网页「参数 → 站立姿态」。 */
#define STANCE_DEFAULT_MODE       -1   /* 上电默认: -1=窄, 0=正常, +1=宽 */
#define STANCE_SCALE_NARROW_DEFAULT      80   /* 窄姿态: 80% (足端靠近机身) */
#define STANCE_SCALE_NORMAL_DEFAULT     100   /* 正常姿态: 100% */
#define STANCE_SCALE_WIDE_DEFAULT       120   /* 宽姿态: 120% (足端远离机身) */
#define STANCE_SCALE_NARROW      (g_params.stance_narrow)
#define STANCE_SCALE_NORMAL      (g_params.stance_normal)
#define STANCE_SCALE_WIDE        (g_params.stance_wide)

/* 姿态切换过渡速度 (×100 单位/控制周期, 10ms)
 * 值越小越平滑。30×100/10ms → 极值切换需 ~1.3s，
 * 确保每条腿在步态抬腿期间逐步挪到新位置，避免擦地。 */
#define STANCE_TRANSITION_SPEED_DEFAULT  30
#define STANCE_TRANSITION_SPEED          (g_params.stance_speed)

/* 单腿每周期最大位移 (mm)，防止着地后首次抬起时骤跳。
 * 设太小 → 切换慢；设太大 → 空中骤跳。
 * 2mm/周期 = 200mm/s，足够跟踪 STANCE_TRANSITION_SPEED=30 的节奏。 */
#define STANCE_MAX_STEP_MM_DEFAULT        2
#define STANCE_MAX_STEP_MM                (g_params.stance_step_mm)

/* ---- CRSF 死区参数 ----
 *
 * 两级死区设计：
 *
 *   第 1 级：原始通道死区 (CRSF raw units)
 *     CRSF_CH_VALUE_DEADBAND 定义了摇杆中位附近的死区宽度。
 *     channel ∈ [MID-DEADBAND, MID+DEADBAND] → 输出强制为 0。
 *     CRSF 通道范围 172~1811 (跨度 ~1639)，默认死区 ±40 ≈ ±2.4%。
 *
 *   第 2 级：控制量死区 (映射后的 -500~+500 范围)
 *     CONTROL_DEADBAND 定义了摇杆映射后的死区阈值。
 *     mapped ∈ [-DEADBAND, +DEADBAND] → 输出强制为 0。
 *     默认 ±15/500 = ±3%，过滤映射后的微小残余。
 *
 *   两级串联效果：摇杆需偏离中位足够远才会产生运动，
 *   消除摇杆抖动、中位漂移和机械虚位引起的误动作。
 *
 * ★ 两个死区运行时可改 (!CFG crsf_deadband / deadband) */
#define CRSF_CH_VALUE_DEADBAND_DEFAULT    40   /* 原始通道死区 (CRSF units)，±40 约 ±2.4% */
#define CONTROL_DEADBAND_DEFAULT          5    /* 控制量死区 (-500~+500)，±15 约 ±3% */
#define CRSF_CH_VALUE_DEADBAND    (g_params.crsf_deadband)
#define CONTROL_DEADBAND          (g_params.deadband)

/* 高度控制阈值：油门杆须偏离中位超过此值才开始改变抬腿高度。
 * 因为高度是积分控制（每周期累积），阈值需比运动通道的死区更大。
 * 默认 = CONTROL_DEADBAND × 2 ≈ 30，即摇杆偏离约 6% 才响应。 */
#define HEIGHT_CONTROL_THRESHOLD  (CONTROL_DEADBAND * 2)

/* 机身高度线性控制范围 (mm)
 * 摇杆满量程 (±500) 映射到的机身高度偏移。
 * body_pos.y = stick * BODY_HEIGHT_RANGE_MM / 500
 * 正值抬升机身 (腿向下伸展), 负值降低机身 (腿向上收缩) */
#define BODY_HEIGHT_RANGE_MM_DEFAULT      90
#define BODY_HEIGHT_RANGE_MM      (g_params.body_h_range_mm)

/* 机身姿态旋转范围 (0.1° 单位)
 * 摇杆满量程 (±500) 映射到的机身旋转角。
 * body_rot = stick * BODY_ROTATION_MAX / 500
 * 400 = 40.0°, 即摇杆推到底时机身倾斜 40° */
#define BODY_ROTATION_MAX_DEFAULT         400
#define BODY_ROTATION_MAX         (g_params.body_rot_max)

/* ---- CRSF 摇杆→控制量 缩放参数 ----
 * 摇杆范围 -500~+500, 映射到实际运动参数
 * ★ 全部运行时可改, 网页「参数 → 运动」页有滑块。
 *   调步长时注意: 满杆位移超过腿的可达范围会让 IK 解算失败 (腿伸不直),
 *   表现为该腿抖动或停在原地。改大后先在悬空状态试。 */
#define TRAVEL_MAX_FORWARD_MM_DEFAULT   150   /* 满杆步长 (mm)，约体长1/3 */
#define TRAVEL_MAX_STRAFE_MM_DEFAULT    110   /* 满杆平移步长 (mm) */
#define TRAVEL_MAX_TURN_MM_DEFAULT       70   /* 满杆旋转步长 (mm) */
#define LIFT_HEIGHT_MIN_MM_DEFAULT        5   /* 最低抬腿高度 (mm) */
#define LIFT_HEIGHT_MAX_MM_DEFAULT       60   /* 最高抬腿高度 (mm) */
#define TRAVEL_MAX_FORWARD_MM    (g_params.travel_fwd_mm)
#define TRAVEL_MAX_STRAFE_MM     (g_params.travel_str_mm)
#define TRAVEL_MAX_TURN_MM       (g_params.travel_turn_mm)
#define LIFT_HEIGHT_MIN_MM       (g_params.lift_min_mm)
#define LIFT_HEIGHT_MAX_MM       (g_params.lift_max_mm)

/* ---- 仿生连续变速 ----
 *
 * 六足虫行走时靠改变迈腿频率调速，而非改变步长。
 * 摇杆推得越多 → 步态周期越短（频率越高）。
 *
 *   period = PERIOD_MAX - (MAX-MIN) × stick_magnitude / 500
 *
 *   摇杆微动 (~10%):  period ≈ 175ms → 12×175=2100ms/周期 ≈ 0.48 Hz (慢走)
 *   摇杆半量 (~50%):  period ≈ 125ms → 12×125=1500ms/周期 ≈ 0.67 Hz (常步)
 *   摇杆满量 (100%):  period ≈  70ms → 12×70= 840ms/周期 ≈ 1.19 Hz (快走)
 *
 * 注意: 步长 (travel_length) 也随摇杆线性变化。
 *       低摇杆 = 短步长 + 低频率 → 精细缓动
 *       高摇杆 = 大步长 + 高频率 → 快速行进
 *       两者叠加产生自然的加速度曲线。
 *
 * ★ 运行时 !CFG gait_min_ms / gait_max_ms */
#define GAIT_PERIOD_MAX_MS_DEFAULT      190    /* 微动: 最慢步频 */
#define GAIT_PERIOD_MIN_MS_DEFAULT       50    /* 满杆: 最快步频 */
#define GAIT_PERIOD_MAX_MS       (g_params.gait_max_ms)
#define GAIT_PERIOD_MIN_MS       (g_params.gait_min_ms)

/* CH7 通道保留 (CRSF_CHANNEL_SPEED)，暂不参与控制。
 * 连续变速由摇杆幅度自动映射，无需开关干预。 */

/* ---- 腿基座在机身上的安装位置 ----
 *
 * 即 Coxa 舵机转轴中心在机身坐标系中的坐标 (单位: mm)
 *
 *   offset_x: 前后偏移。所有腿通常为负值 (在重心后方)。
 *             RR/RF = -43, RM = -63 (中腿更靠后)
 *
 *   offset_z: 左右偏移。右腿为正 (机身右侧)，左腿为负 (左侧)。
 *             RR/RF = ±82, RM/LM = 0 (中腿在正中线上)
 *
 * 测量方法：从机身几何中心 (原点) 量到每个 Coxa 舵机轴的垂直投影点。
 *
 *         Z+ (机身右侧)
 *         │
 *    LF   │@offset_z=+82   RF      @ = Coxa 转轴位置
 *     ╲   │   ╱                   所有腿 offset_x 均为负值
 *  LM ───○─── RM                  所以腿装在重心偏后方
 *     ╱   │   ╲
 *    LR   │@offset_z=-82   RR
 *         │
 *         └────── X+
 */
#define BODY_OFFSET_RR_X    -104
#define BODY_OFFSET_RR_Z    63     /* 右后: Z+ */
#define BODY_OFFSET_RM_X    0
#define BODY_OFFSET_RM_Z    79     /* 右中: Z+ (中腿最宽) */
#define BODY_OFFSET_RF_X    104
#define BODY_OFFSET_RF_Z    63     /* 右前: Z+ */

#define BODY_OFFSET_LR_X    -104
#define BODY_OFFSET_LR_Z    -63    /* 左后: Z- (镜像) */
#define BODY_OFFSET_LM_X    0
#define BODY_OFFSET_LM_Z    -79    /* 左中: Z- (中腿最宽) */
#define BODY_OFFSET_LF_X    104
#define BODY_OFFSET_LF_Z    -63    /* 左前: Z- (镜像) */

/* ---- Coxa 舵机安装偏角 (0.1度单位, 900 = 90°) ----
 *
 * 公式: servo = atan4(foot_z, foot_x) - COXA_ANGLE
 *
 * COXA_ANGLE 的含义：当足端在 atan4=0 (正前方) 时，舵机需要的角度。
 * 换言之，COXA_ANGLE 是「舵机 0° 时，腿指向的方向」在机身坐标系中的角度。
 *
 *   0° = 正前方 (X+)
 *   900 (90°) = 正右方 (Z+)
 *   ±1800 (±180°) = 正后方 (X-)
 *   -900 (-90°) = 正左方 (Z-)
 *
 * 取值方法：解锁后看 Level 3 中 Coxa 的角度输出，
 * 调整 COXA_ANGLE 使 Coxa 接近 0 (舵机中位)。
 *
 *   Coxa_IK > 0 太多 → 增大 COXA_ANGLE
 *   Coxa_IK < 0 太多 → 减小 COXA_ANGLE
 */
/* 足端方向 = 舵机0°时 coxa 的指向:
 *   RR=135°(后右) RM=90°(正右) RF=45°(前右)
 *   LR=-135°(后左) LM=-90°(正左) LF=-45°(前左)
 *
 * 这些角度由硬件机械结构决定，不可随意修改。
 * COXA_ANGLE 使 atan4(init_foot) - COXA_ANGLE = 0，即站立时 coxa=0°。 */
#define COXA_ANGLE_RR       1350    /* 右后: 135° 后方偏右 */
#define COXA_ANGLE_RM       900     /* 右中:  90° 正右方 */
#define COXA_ANGLE_RF       450     /* 右前:  45° 前方偏右 */
#define COXA_ANGLE_LR       -1350   /* 左后: -135° 后方偏左 */
#define COXA_ANGLE_LM       -900    /* 左中:  -90° 正左方 */
#define COXA_ANGLE_LF       -450    /* 左前:  -45° 前方偏左 */

/* 前进方向取反开关 (CH2)
 * 如果推摇杆前进时机体后退，设为 1 翻转前进/后退方向。
 * 原因：某些遥控器的 CH2 (Pitch) 输出极性相反 (拉杆=高位, 推杆=低位)。
 * 不要通过翻转 coxa_invert 来修正方向——那会同时破坏转动方向。
 * ★ 四个取反开关运行时可改 (!CFG inv_fwd / inv_str / inv_h / inv_turn),
 *   网页「参数 → 方向取反」页做成开关, 遥控器换机/换固件后不用重烧。 */
#define FORWARD_DIRECTION_INVERT_DEFAULT   0
#define STRAFE_DIRECTION_INVERT_DEFAULT    1
#define HEIGHT_DIRECTION_INVERT_DEFAULT    0
#define TURN_DIRECTION_INVERT_DEFAULT      0
#define FORWARD_DIRECTION_INVERT   (g_params.inv_fwd)

/* 平移方向取反开关 (CH1)
 * 摇杆左推→右平移 / 右推→左平移 时，设为 1 翻转。 */
#define STRAFE_DIRECTION_INVERT    (g_params.inv_str)

/* 高度方向取反开关 (CH3)
 * 摇杆推高→机身下降 / 拉低→机身抬升 时，设为 1 翻转。 */
#define HEIGHT_DIRECTION_INVERT    (g_params.inv_h)

/* 旋转方向取反开关 (CH4)
 * 摇杆左推→顺时针转 / 右推→逆时针转 时，设为 1 翻转。 */
#define TURN_DIRECTION_INVERT      (g_params.inv_turn)

/* ==================== PS2 手柄方向取反开关 ====================
 * 独立于 CRSF (两者摇杆极性约定不同, 校准互不影响)。
 * 默认值按 PS2 标准直觉: 推上=前进, 推左=左移, 推右=右转, 推上=机身升高。
 * 实测哪个轴方向反了就翻转对应宏 (0↔1)。
 * ★ 四个开关运行时可改 (!CFG ps2_inv_fwd / ps2_inv_str / ps2_inv_h /
 *   ps2_inv_turn), 网页「参数 → 方向取反」页做成开关。 */
#define PS2_FORWARD_DIRECTION_INVERT_DEFAULT   1
#define PS2_STRAFE_DIRECTION_INVERT_DEFAULT    1
#define PS2_HEIGHT_DIRECTION_INVERT_DEFAULT    1
#define PS2_TURN_DIRECTION_INVERT_DEFAULT      0
#define PS2_FORWARD_DIRECTION_INVERT   (g_params.ps2_inv_fwd)
#define PS2_STRAFE_DIRECTION_INVERT    (g_params.ps2_inv_str)
#define PS2_HEIGHT_DIRECTION_INVERT    (g_params.ps2_inv_h)
#define PS2_TURN_DIRECTION_INVERT      (g_params.ps2_inv_turn)

/* PS2 高度轴 (LY) 专用死区 (±25/500 = ±5%):
 * 高度为积分控制 (LY 弹簧摇杆 = 速率输入, 回中=高度保持),
 * 死区用于抑制中心抖动造成的积分漂移, 无需太大
 * ★ 运行时 !CFG ps2_h_deadzone */
#define PS2_HEIGHT_DEADZONE_DEFAULT    25
#define PS2_HEIGHT_DEADZONE            (g_params.ps2_h_deadzone)

/* PS2 摇杆指数曲线 (expo) 混合比例 0~100:
 * 0 = 纯线性 (禁用); 100 = 纯三次方曲线 (初段最钝)
 * PS2 电位器摇杆仅 8 位分辨率 (一格≈0.79% 指令) 且带机械旷量与噪声,
 * 中位附近难以精细控制 — expo 压缩初段灵敏度、放大末段,
 * 满杆输出恒为 ±500, 最大行程不受影响。
 * 仅作用于 PS2 路径 (ps2_expo), 与 CRSF/USB 控制解耦
 * ★ 运行时 !CFG ps2_expo, 网页「参数 → 手感」页拖滑块即时感受 */
#define PS2_STICK_EXPO_DEFAULT        50
#define PS2_STICK_EXPO                (g_params.ps2_expo)

/* ---- 初始足端位置 (站立时足端在腿基座坐标系中的坐标) ----
 *
 * 你的硬件几何 (侧视图，沿 coxa 指向方向看):
 *
 *     Coxa基座 ●
 *              ╲  coxa=45mm (水平)
 *               ╲
 *    股节根部    ●────╲              femur=75mm
 *    (舵机0°=     ╲   ╲ 向上53mm    (向上45°=离垂直45°)
 *     离垂直45°)   ╲    ╲ 向外53mm
 *                   ● 膝关节
 *                    ╲
 *          tibia      ╲ 向下84mm    tibia=120mm
 *          =120mm      ╲ 向外84mm   (与femur成90°→斜向下45°)
 *                        ● 足端
 *
 *  ┌──────────── 站立状态 (ARM 后, 舵机≠0°) ────────┐
 *  │ INIT_Y=50, 足端在coxa下方50mm                   │
 *  │ FOOT_DZ=110 (RM), FOOT_DX=±78 (RR/RF)           │
 *  │ 膝角≈58°, 股节≈32°                              │
 *  │ coxa≈0° (足端方向不变)                           │
 *  └────────────────────────────────────────────────┘
 */

/* ---- 休息状态足端 (舵机全0°, 底板贴地) ---- */
/* 站立时足端在 coxa 下方的基准深度 (mm)。
 * 机身高度调节**不在此处**叠加; body_pos.y 由 IK 层施加:
 *   init_pos_y = INIT_Y
 *   ik.c: relative_pos.y = target_foot.y + body_pos.y
 * BODY_HEIGHT_RANGE_MM 决定油门杆能调多远 (±90mm)。 */
#define INIT_Y               50

/* 足端在 coxa 基座坐标系中的站立位置
 *
 * 由硬件出射角 (RR=135°, RM=90°, RF=45°) 和腿长计算。
 * 这些值决定 atan4 零点，与 COXA_ANGLE 配套。 */
#define FOOT_DX_RR     -113     /* RR: 110×cos135° */
#define FOOT_DZ_RR      113     /* RR: 110×sin135° */

#define FOOT_DX_RM       0     /* RM: 110×cos90° */
#define FOOT_DZ_RM     160     /* RM: 110×sin90° */

#define FOOT_DX_RF      113     /* RF: 110×cos45° */
#define FOOT_DZ_RF      113     /* RF: 110×sin45° */

#define FOOT_DX_LR     -113     /* LR: 镜像 */
#define FOOT_DZ_LR     -113  

#define FOOT_DX_LM       0     /* LM: 镜像 */
#define FOOT_DZ_LM    -160

#define FOOT_DX_LF      113     /* LF: 镜像 */
#define FOOT_DZ_LF     -113

/* ==================== 舵机参数配置 ==================== */

/*
 * 舵机角度约定：
 *   单位: 0.1 度 (900 = 90°)
 *   零位: 角度 = 0 → 脉宽 = 1500μs → 舵机机械中位
 *   正值: 顺时针 (从舵机输出轴顶部看) → 脉宽 > 1500μs
 *   负值: 逆时针 → 脉宽 < 1500μs
 *
 *   软件保护: IK 输出角度超过 [min, max] 时会被钳位到边界值，
 *   并设置 solution_warning 标志。不会输出越界脉宽。
 *
 * 调参方法 (用 !W 命令扫摆测试):
 *   1. !W<id>  观察舵机从 30° 扫到 150°
 *   2. 如果 30° 时已碰到机械限位 → 把 min 改大 (例: -260 → -100)
 *   3. 如果 150° 时已碰到机械限位 → 把 max 改小 (例: 740 → 500)
 *   4. 安全余量: 在机械极限内侧留 5~10° 余量
 *
 * 注意: 左腿和右腿的 min/max 符号相反，因为 invert 方向不同。
 *   右腿 (invert=true): min 为逆时针极限 (负值), max 为顺时针极限 (正值)
 *   左腿 (invert=false): min 为逆时针极限 (负值), max 为顺时针极限 (正值)
 *   左腿的绝对值可能不同，因为机械结构镜像后活动范围可能不对称。
 */

/* ---- 右后腿 (RR) 舵机限位 (0.1度) ----
 * Coxa 暂时放宽到 ±90°，用 !W 找到实际机械极限后再收紧 */
#define SERVO_COXA_MIN_RR   -900    /* Coxa 逆时针极限 (暂定) */
#define SERVO_COXA_MAX_RR   900     /* Coxa 顺时针极限 (暂定) */
#define SERVO_FEMUR_MIN_RR  -590    /* Femur 逆时针极限 (暂定) */
#define SERVO_FEMUR_MAX_RR  900     /* Femur 顺时针极限 (暂定) */
#define SERVO_TIBIA_MIN_RR  -695    /* Tibia 逆时针极限 (暂定) */
#define SERVO_TIBIA_MAX_RR  900     /* Tibia 顺时针极限 (暂定) */

/* ---- 右中腿 (RM) 舵机限位 ---- */
#define SERVO_COXA_MIN_RM   -900
#define SERVO_COXA_MAX_RM   900
#define SERVO_FEMUR_MIN_RM  -600
#define SERVO_FEMUR_MAX_RM  900
#define SERVO_TIBIA_MIN_RM  -687
#define SERVO_TIBIA_MAX_RM  900

/* ---- 右前腿 (RF) 舵机限位 ---- */
#define SERVO_COXA_MIN_RF   -900
#define SERVO_COXA_MAX_RF   900
#define SERVO_FEMUR_MIN_RF  -610
#define SERVO_FEMUR_MAX_RF  900
#define SERVO_TIBIA_MIN_RF  -627
#define SERVO_TIBIA_MAX_RF  900

/* ---- 左后腿 (LR) 舵机限位 ---- */
#define SERVO_COXA_MIN_LR   -900
#define SERVO_COXA_MAX_LR   900
#define SERVO_FEMUR_MIN_LR  -900
#define SERVO_FEMUR_MAX_LR  660
#define SERVO_TIBIA_MIN_LR  -900
#define SERVO_TIBIA_MAX_LR  695

/* ---- 左中腿 (LM) 舵机限位 ---- */
#define SERVO_COXA_MIN_LM   -900
#define SERVO_COXA_MAX_LM   900
#define SERVO_FEMUR_MIN_LM  -900
#define SERVO_FEMUR_MAX_LM  665
#define SERVO_TIBIA_MIN_LM  -900
#define SERVO_TIBIA_MAX_LM  697

/* ---- 左前腿 (LF) 舵机限位 ---- */
#define SERVO_COXA_MIN_LF   -900
#define SERVO_COXA_MAX_LF   900
#define SERVO_FEMUR_MIN_LF  -900
#define SERVO_FEMUR_MAX_LF  650
#define SERVO_TIBIA_MIN_LF  -900
#define SERVO_TIBIA_MAX_LF  735

/* ==================== 舵机ID映射 ==================== */

/*
 * 每个舵机的全局 ID (0~17)，对应 hal_servo_set_angle 的 servo_id 参数。
 *
 * PCA9685 分配 (PCB 定版):
 *   板 0x40 (ADDR=GND): 舵机 ID 9~17 (左半身 LR+LM+LF, 3腿 × 3关节 = 9 路)
 *   板 0x41 (ADDR=VCC): 舵机 ID 0~8  (右半身 RR+RM+RF, 3腿 × 3关节 = 9 路)
 *
 * 每腿 3 关节顺序: Coxa(0) → Femur(1) → Tibia(2)
 *
 * 物理通道映射 (PCB 定版, 见 pca9685_servo_to_channel):
 *   左侧板 0x40 (ID 9~17): 从前至后 LED7~LED15
 *     LF: LED7,8,9  → LM: LED10,11,12 → LR: LED13,14,15
 *   右侧板 0x41 (ID 0~8): 从前至后 LED8~LED0
 *     RF: LED8,7,6  → RM: LED5,4,3    → RR: LED2,1,0
 */
/* 仅 Coxa 需要具名 (被 !PER 校准的 6 路中位列表引用)。
 * Femur / Tibia 的 ID 由 hal_get_servo_id() 按 leg*3+1 / leg*3+2 推出, 不另设宏。 */
#define SERVO_RR_COXA       0     /* 右后 Coxa  */
#define SERVO_RM_COXA       3     /* 右中 Coxa  */
#define SERVO_RF_COXA       6     /* 右前 Coxa  */
#define SERVO_LR_COXA       9     /* 左后 Coxa  */
#define SERVO_LM_COXA       12    /* 左中 Coxa  */
#define SERVO_LF_COXA       15    /* 左前 Coxa  */

/* ==================== 预留外设 (PCB 已布线, 暂未接线) ====================
 *
 * 这些器件都在 PCB 上留了接口但没有实际接负载, 所以固件默认全部关闭
 * (led_* 除外, 那两个是既有的状态灯)。开关都是运行时参数, 网页「外设」页
 * 直接可调, 无需重烧。
 *
 * ⚠️ 所有权: led_heartbeat / led_alarm 置 0 之前, 主循环每 1~2s 会覆盖写
 *    对应的 LED —— 想用 !LED g 1 手动控制, 必须先把那一位关掉, 否则手动
 *    设置会在下一个状态块被冲掉。
 */

/* ---- 蜂鸣器 (GP13, 无源, PWM 方波驱动) ----
 * 无源蜂鸣器靠改变 PWM 频率发声, 所以频率下限由 16 位 wrap 决定:
 *   时钟 125MHz/16 = 7.8125MHz, 最低频率 200Hz → wrap = 39062 (仍在 16 位内)
 * !BUZZ 命令和网页外设页都用这个下限夹取值, 不要只改一处。 */
#define BUZZER_PIN                      13      /* GP13 (PWM6B) */
#define BUZZER_PWM_CLKDIV               16.0f   /* 125MHz / 16 = 7.8125MHz */
#define BUZZER_MIN_FREQ_HZ              200     /* 最低频率 (16-bit wrap 上限约束) */
#define BUZZER_MAX_FREQ_HZ              10000   /* 上限: 再高只是刺耳, 且 wrap 只剩个位数 */
#define BUZZER_MAX_MS                   2000    /* 单次蜂鸣时长上限 (阻塞命令) */

/* ---- 状态灯所有权 (0 = 交还给手动控制) ---- */
#define LED_HEARTBEAT_ENABLED_DEFAULT   1   /* 绿灯心跳 (2s 状态块) */
#define LED_ALARM_ENABLED_DEFAULT       1   /* 红灯报警 (1s 电池块) */
#define LED_HEARTBEAT_ENABLED           (g_params.led_heartbeat)
#define LED_ALARM_ENABLED               (g_params.led_alarm)

/* ---- 外部 UART0 (GP0=TX, GP1=RX) ----
 * 与 CRSF 用的 UART1 (GP4/GP5) 相互独立, 可同时工作。典型用途: 接一个
 * 串口模块 (GPS / 上位机 / 第二个接收机)。收到的一行原样转发到 USB 调试口
 * ([U0] 前缀), 发出去用 !UART0T <text>。
 * 波特率改动由主循环 20ms 轮询检测并重新 uart_init。 */
#define EXT_UART0_TX_PIN                0
#define EXT_UART0_RX_PIN                1
#define UART0_ENABLED_DEFAULT           0
#define UART0_BAUD_DEFAULT              115200
#define UART0_ENABLED                   (g_params.uart0_en)
#define UART0_BAUD                      (g_params.uart0_baud)

/* ---- 外部 I2C / ADC 排针 (GP26, GP27) ----
 * ⚠️ RP2040 的 GPIO 功能表: GP26/27 只映射到 I2C1, 而 i2c1 已经被
 *    GP14/15 上的 PCA9685 + BNO055 占用 (I2C0 只能到 GP28/29, 其中
 *    GP28 是电池 ADC)。所以这里**不能用硬件 I2C 控制器** —— 用 i2c1
 *    驱动 GP26/27 会让外部排针与内部舵机总线电性相连, 悬空的外部引脚
 *    会把整条内部总线拖死。
 *
 *    改用软件位翻转 (bit-bang) 实现, 是真正独立的第二路总线:
 *    速率低 (约 50kHz), 扫描/读写寄存器够用。
 *
 * 模式 (ext_i2c_mode):
 *   0 = 关闭        (引脚保持上次状态, 不驱动)
 *   1 = 软件 I2C    (GP26=SDA, GP27=SCL, 开漏 + 内部上拉)
 *   2 = ADC         (GP26=ADC0, GP27=ADC1, 12 位单端)
 *
 * ⚠️ 内部上拉约 50kΩ, 只能应付短线低速。接实际器件时排针上仍应有
 *    4.7kΩ 外部上拉 —— !I2C2 扫不到设备时先查这个, 别怀疑固件。
 *
 * ⚠️ 旧板 (2026-08 之前) GP26 的 ADC 通道曾被打坏 (见 STATUS 事件 3),
 *    现板电池 ADC 已改到 GP28 绕开它。!ADC2 读数异常时先怀疑硬件。 */
#define EXT_I2C_SDA_PIN                 26
#define EXT_I2C_SCL_PIN                 27
#define EXT_ADC0_PIN                    26
#define EXT_ADC1_PIN                    27
#define EXT_ADC0_INPUT                  0
#define EXT_ADC1_INPUT                  1
#define EXT_I2C_BIT_DELAY_US            5    /* 半周期延时: 5µs → ~100kHz 方波, SCL ≈ 50kHz */
#define EXT_I2C_MODE_OFF                0
#define EXT_I2C_MODE_I2C                1
#define EXT_I2C_MODE_ADC                2
#define EXT_I2C_MODE_DEFAULT            EXT_I2C_MODE_OFF
/* 当前模式 (参数是唯一真源)。故意不叫 EXT_I2C_MODE —— 与取值 EXT_I2C_MODE_I2C
 * 只差一个后缀, 写成 if (EXT_I2C_MODE_I2C) 恒真而编译器一声不吭。 */
#define EXT_PIN_MODE                    (g_params.ext_i2c_mode)

/* ---- 空闲 PWM 通道 (2× PCA9685 共 32 路, 舵机占 18 路) ----
 * 剩下 14 路做通用 PWM 输出 (测试工具语义: 值只在 RAM, 断电不保持)。
 *
 * 编号约定 (!PWM <idx>, 网页外设页同):
 *   idx 0~6  = 左板 0x40 的 LED0~LED6
 *   idx 7~13 = 右板 0x41 的 LED9~LED15
 * 这两段正好是各自板子上没被舵机占用的通道 (见 pca9685_servo_to_channel)。
 *
 * ⚠️ 必须合并进 hal_servo_flush 的同一批 16 通道整写里: 那批每 ~20ms
 *    发一次, 只写自己那几路的话空闲通道会被一起清零。 */
#define FREE_PWM_COUNT                  14
#define FREE_PWM_LEFT_FIRST             0
#define FREE_PWM_LEFT_LAST              6
#define FREE_PWM_RIGHT_FIRST            7
#define FREE_PWM_RIGHT_LAST             13
#define FREE_PWM_LEFT_CH_BASE           0    /* 左板空闲段起始物理通道 */
#define FREE_PWM_RIGHT_CH_BASE          9    /* 右板空闲段起始物理通道 */

/* ==================== 配置数据结构 ==================== */

/**
 * @brief 获取默认腿部配置
 * @param configs 输出配置数组（至少6个元素）
 */
static inline void hexapod_get_default_config(leg_config_t *configs)
{
    /*
     * 每条腿的配置字段含义:
     *
     *   coxa/femur/tibia_length  : 腿节长度 (mm)，从对应的宏复制
     *   offset_x, offset_z       : 腿基座在机身上的安装坐标 (mm)
     *   coxa_angle               : Coxa 舵机的安装偏角 (0.1°)
     *   init_pos_x, _y, _z       : 站立时足端在机身坐标系中的目标坐标 (mm)
     *   coxa/femur/tibia_min/max : 舵机软件限位 (0.1°)
     *   coxa/femur/tibia_invert  : 舵机方向反转 (true=反转)
     *   coxa/femur/tibia_horn_offset : 舵盘安装偏移 (0.1°)，校准站立姿态的核心参数
     *
     * invert 规则:
     *   右腿 (RR/RM/RF): 全部 true  — 因为右腿舵机在机身右侧，转动方向与左腿镜像
     *   左腿 (LR/LM/LF): 全部 false — 左腿方向与数学模型一致
     *
     *   调参: 用 !P<id> 500 发送正角度。
     *         如果腿往「预期反方向」转 → 切换对应的 invert 值。
     *
     * horn_offset 校准方法 (推荐):
     *   1. 解锁机器人，观察 Level 3 输出的 IK 角度
     *      例: RR: 650 -249 -611
     *   2. 用 !P<id> <angle> 手动找到该关节的最佳站立角度
     *      例: !P0 400  (发现 Coxa 在 400 时腿的姿态最好)
     *   3. horn_offset = 最佳角度 - IK 输出角度
     *      例: coxa_horn_offset = 400 - 650 = -250
     *   4. 填入配置、重新编译、验证
     *   5. 重复直到所有腿站立姿态正确
     *
     *   这比反复猜测 init_pos 直观得多——你直接告诉舵机"站在这儿"。
     */

    /* 右后腿 (RR):
     *   offset = (-104, 63), coxa_angle = -45°
     *   脚在基座后方 30mm、外侧 40mm
     *   → init_pos = (-104-30, INIT_Y, 63+40) = (-134, INIT_Y, 103) */
    configs[LEG_RR].coxa_length = LEG_COXA_LENGTH;
    configs[LEG_RR].femur_length = LEG_FEMUR_LENGTH;
    configs[LEG_RR].tibia_length = LEG_TIBIA_LENGTH;
    configs[LEG_RR].offset_x = BODY_OFFSET_RR_X;
    configs[LEG_RR].offset_z = BODY_OFFSET_RR_Z;
    configs[LEG_RR].coxa_angle = COXA_ANGLE_RR;
    configs[LEG_RR].init_pos_x = BODY_OFFSET_RR_X + FOOT_DX_RR;
    configs[LEG_RR].init_pos_y = INIT_Y;
    configs[LEG_RR].init_pos_z = BODY_OFFSET_RR_Z + FOOT_DZ_RR;
    configs[LEG_RR].coxa_min = SERVO_COXA_MIN_RR;
    configs[LEG_RR].coxa_max = SERVO_COXA_MAX_RR;
    configs[LEG_RR].femur_min = SERVO_FEMUR_MIN_RR;
    configs[LEG_RR].femur_max = SERVO_FEMUR_MAX_RR;
    configs[LEG_RR].tibia_min = SERVO_TIBIA_MIN_RR;
    configs[LEG_RR].tibia_max = SERVO_TIBIA_MAX_RR;
    configs[LEG_RR].coxa_invert = false;
    configs[LEG_RR].femur_invert = true;
    configs[LEG_RR].tibia_invert = true;
    configs[LEG_RR].coxa_horn_offset = -10;
    configs[LEG_RR].femur_horn_offset = 0;
    configs[LEG_RR].tibia_horn_offset = 0;

    /* 右中腿 (RM):
     *   offset = (0, 79), coxa_angle = 0°
     *   脚在基座前方 30mm、外侧 30mm
     *   → init_pos = (0+30, INIT_Y, 79+30) = (30, INIT_Y, 109) */
    configs[LEG_RM].coxa_length = LEG_COXA_LENGTH;
    configs[LEG_RM].femur_length = LEG_FEMUR_LENGTH;
    configs[LEG_RM].tibia_length = LEG_TIBIA_LENGTH;
    configs[LEG_RM].offset_x = BODY_OFFSET_RM_X;
    configs[LEG_RM].offset_z = BODY_OFFSET_RM_Z;
    configs[LEG_RM].coxa_angle = COXA_ANGLE_RM;
    configs[LEG_RM].init_pos_x = BODY_OFFSET_RM_X + FOOT_DX_RM;
    configs[LEG_RM].init_pos_y = INIT_Y;
    configs[LEG_RM].init_pos_z = BODY_OFFSET_RM_Z + FOOT_DZ_RM;
    configs[LEG_RM].coxa_min = SERVO_COXA_MIN_RM;
    configs[LEG_RM].coxa_max = SERVO_COXA_MAX_RM;
    configs[LEG_RM].femur_min = SERVO_FEMUR_MIN_RM;
    configs[LEG_RM].femur_max = SERVO_FEMUR_MAX_RM;
    configs[LEG_RM].tibia_min = SERVO_TIBIA_MIN_RM;
    configs[LEG_RM].tibia_max = SERVO_TIBIA_MAX_RM;
    configs[LEG_RM].coxa_invert = false;
    configs[LEG_RM].femur_invert = true;
    configs[LEG_RM].tibia_invert = true;
    configs[LEG_RM].coxa_horn_offset = -10;
    configs[LEG_RM].femur_horn_offset = -20;
    configs[LEG_RM].tibia_horn_offset = 8;

    /* 右前腿 (RF):
     *   offset = (104, 63), coxa_angle = +45°
     *   脚在基座前方 30mm、外侧 40mm
     *   → init_pos = (104+30, INIT_Y, 63+40) = (134, INIT_Y, 103) */
    configs[LEG_RF].coxa_length = LEG_COXA_LENGTH;
    configs[LEG_RF].femur_length = LEG_FEMUR_LENGTH;
    configs[LEG_RF].tibia_length = LEG_TIBIA_LENGTH;
    configs[LEG_RF].offset_x = BODY_OFFSET_RF_X;
    configs[LEG_RF].offset_z = BODY_OFFSET_RF_Z;
    configs[LEG_RF].coxa_angle = COXA_ANGLE_RF;
    configs[LEG_RF].init_pos_x = BODY_OFFSET_RF_X + FOOT_DX_RF;
    configs[LEG_RF].init_pos_y = INIT_Y;
    configs[LEG_RF].init_pos_z = BODY_OFFSET_RF_Z + FOOT_DZ_RF;
    configs[LEG_RF].coxa_min = SERVO_COXA_MIN_RF;
    configs[LEG_RF].coxa_max = SERVO_COXA_MAX_RF;
    configs[LEG_RF].femur_min = SERVO_FEMUR_MIN_RF;
    configs[LEG_RF].femur_max = SERVO_FEMUR_MAX_RF;
    configs[LEG_RF].tibia_min = SERVO_TIBIA_MIN_RF;
    configs[LEG_RF].tibia_max = SERVO_TIBIA_MAX_RF;
    configs[LEG_RF].coxa_invert = false;
    configs[LEG_RF].femur_invert = true;
    configs[LEG_RF].tibia_invert = true;
    configs[LEG_RF].coxa_horn_offset = -10;
    configs[LEG_RF].femur_horn_offset = -10;
    configs[LEG_RF].tibia_horn_offset = 68;

    /* 左后腿 (LR) — 与 RR 镜像:
     *   offset = (-104, -63), coxa_angle = -45°
     *   脚在基座后方 30mm、外侧 (更左) 40mm (Z 负方向)
     *   → init_pos = (-104-30, INIT_Y, -63-40) = (-134, INIT_Y, -103) */
    configs[LEG_LR].coxa_length = LEG_COXA_LENGTH;
    configs[LEG_LR].femur_length = LEG_FEMUR_LENGTH;
    configs[LEG_LR].tibia_length = LEG_TIBIA_LENGTH;
    configs[LEG_LR].offset_x = BODY_OFFSET_LR_X;
    configs[LEG_LR].offset_z = BODY_OFFSET_LR_Z;
    configs[LEG_LR].coxa_angle = COXA_ANGLE_LR;
    configs[LEG_LR].init_pos_x = BODY_OFFSET_LR_X + FOOT_DX_LR;
    configs[LEG_LR].init_pos_y = INIT_Y;
    configs[LEG_LR].init_pos_z = BODY_OFFSET_LR_Z + FOOT_DZ_LR;
    configs[LEG_LR].coxa_min = SERVO_COXA_MIN_LR;
    configs[LEG_LR].coxa_max = SERVO_COXA_MAX_LR;
    configs[LEG_LR].femur_min = SERVO_FEMUR_MIN_LR;
    configs[LEG_LR].femur_max = SERVO_FEMUR_MAX_LR;
    configs[LEG_LR].tibia_min = SERVO_TIBIA_MIN_LR;
    configs[LEG_LR].tibia_max = SERVO_TIBIA_MAX_LR;
    configs[LEG_LR].coxa_invert = false;
    configs[LEG_LR].femur_invert = false;
    configs[LEG_LR].tibia_invert = false;
    configs[LEG_LR].coxa_horn_offset = -15;
    configs[LEG_LR].femur_horn_offset = 55;
    configs[LEG_LR].tibia_horn_offset = 0;

    /* 左中腿 (LM) — 与 RM 镜像:
     *   offset = (0, -79), coxa_angle = 0°
     *   → init_pos = (0+30, INIT_Y, -79-30) = (30, INIT_Y, -109) */
    configs[LEG_LM].coxa_length = LEG_COXA_LENGTH;
    configs[LEG_LM].femur_length = LEG_FEMUR_LENGTH;
    configs[LEG_LM].tibia_length = LEG_TIBIA_LENGTH;
    configs[LEG_LM].offset_x = BODY_OFFSET_LM_X;
    configs[LEG_LM].offset_z = BODY_OFFSET_LM_Z;
    configs[LEG_LM].coxa_angle = COXA_ANGLE_LM;
    configs[LEG_LM].init_pos_x = BODY_OFFSET_LM_X + FOOT_DX_LM;
    configs[LEG_LM].init_pos_y = INIT_Y;
    configs[LEG_LM].init_pos_z = BODY_OFFSET_LM_Z + FOOT_DZ_LM;
    configs[LEG_LM].coxa_min = SERVO_COXA_MIN_LM;
    configs[LEG_LM].coxa_max = SERVO_COXA_MAX_LM;
    configs[LEG_LM].femur_min = SERVO_FEMUR_MIN_LM;
    configs[LEG_LM].femur_max = SERVO_FEMUR_MAX_LM;
    configs[LEG_LM].tibia_min = SERVO_TIBIA_MIN_LM;
    configs[LEG_LM].tibia_max = SERVO_TIBIA_MAX_LM;
    configs[LEG_LM].coxa_invert = false;
    configs[LEG_LM].femur_invert = false;
    configs[LEG_LM].tibia_invert = false;
    configs[LEG_LM].coxa_horn_offset = 10;
    configs[LEG_LM].femur_horn_offset = 85;
    configs[LEG_LM].tibia_horn_offset = 2;

    /* 左前腿 (LF) — 与 RF 镜像:
     *   offset = (104, -63), coxa_angle = +45°
     *   → init_pos = (104+30, INIT_Y, -63-40) = (134, INIT_Y, -103) */
    configs[LEG_LF].coxa_length = LEG_COXA_LENGTH;
    configs[LEG_LF].femur_length = LEG_FEMUR_LENGTH;
    configs[LEG_LF].tibia_length = LEG_TIBIA_LENGTH;
    configs[LEG_LF].offset_x = BODY_OFFSET_LF_X;
    configs[LEG_LF].offset_z = BODY_OFFSET_LF_Z;
    configs[LEG_LF].coxa_angle = COXA_ANGLE_LF;
    configs[LEG_LF].init_pos_x = BODY_OFFSET_LF_X + FOOT_DX_LF;
    configs[LEG_LF].init_pos_y = INIT_Y;
    configs[LEG_LF].init_pos_z = BODY_OFFSET_LF_Z + FOOT_DZ_LF;
    configs[LEG_LF].coxa_min = SERVO_COXA_MIN_LF;
    configs[LEG_LF].coxa_max = SERVO_COXA_MAX_LF;
    configs[LEG_LF].femur_min = SERVO_FEMUR_MIN_LF;
    configs[LEG_LF].femur_max = SERVO_FEMUR_MAX_LF;
    configs[LEG_LF].tibia_min = SERVO_TIBIA_MIN_LF;
    configs[LEG_LF].tibia_max = SERVO_TIBIA_MAX_LF;
    configs[LEG_LF].coxa_invert = false;
    configs[LEG_LF].femur_invert = false;
    configs[LEG_LF].tibia_invert = false;
    configs[LEG_LF].coxa_horn_offset = -10;
    configs[LEG_LF].femur_horn_offset = 60;
    configs[LEG_LF].tibia_horn_offset = 40;
}

#endif /* HEXAPOD_CONFIG_H */



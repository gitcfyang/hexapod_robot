/**
 * @file hexapod_config.h
 * @brief 六足机器人配置文件
 * @note 用户应根据实际机器人参数修改此文件
 */

#ifndef HEXAPOD_CONFIG_H
#define HEXAPOD_CONFIG_H

#include "hexapod_types.h"

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

/* 电池低压保护：设为 1 启用，设为 0 禁用
 * 启用后，当电池电压低于 MIN_VOLTAGE_MV (定义在 hexapod_core.h) 时自动停止 */
#define BATTERY_CHECK_ENABLED   0   /* 电池接好 ADC 后改为 1 */

/* ==================== 调试配置 ==================== */

/* 调试输出等级 (通过 USB CDC 串口输出)：
 *   0 = 静默，仅输出启动信息和每 2s 状态摘要
 *   1 = 输入调试：打印 CRSF 链接状态 + 8 通道原始值
 *   2 = 舵机调试：打印 18 路舵机角度/脉宽
 *   3 = 全量调试：打印 IK 解算中间值（会产生大量输出，可能影响控制周期） */
#define DEBUG_LEVEL             3   /* ★ 生产模式：关闭调试输出 */

/* 调试输出间隔（毫秒），避免 USB 输出阻塞控制循环 */
#define DEBUG_PRINT_INTERVAL_MS 1000

/* 无舵机调试模式：PRODUCTION=0 要求舵机硬件就绪才启动 */
#define HEADLESS_MODE           0   /* ★ 生产模式：舵机必须正常 */

/* PCA9685 舵机板数量 (1 或 2)
 *   1 = 仅一块板 (地址 0x40)，控制右半身 9 路舵机 (ID 0-8)
 *   2 = 两块板 (地址 0x40 + 0x41)，控制全部 18 路舵机 (ID 0-17) */
#define PCA9685_BOARD_COUNT     1   /* 接好第二块板后改为 2 */

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
 */
#define CRSF_CHANNEL_FORWARD      1   // CH2: 正常=前进/后退, 平衡=俯仰
#define CRSF_CHANNEL_STRAFE       0   // CH1: 正常=左右平移, 平衡=横滚
#define CRSF_CHANNEL_TURN         3   // CH4: 正常=原地旋转, 平衡=偏航
#define CRSF_CHANNEL_HEIGHT       2   // CH3: 正常=机身高度, 平衡=机身高度
#define CRSF_CHANNEL_ARM          4   // CH5: 解锁 (二段开关)
#define CRSF_CHANNEL_GAIT         5   // CH6: 步态 (三段开关)
#define CRSF_CHANNEL_SPEED        6   // CH7: 速度 (三段开关)
#define CRSF_CHANNEL_BALANCE      7   // CH8: 平衡模式 (二段开关)

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
 *   消除摇杆抖动、中位漂移和机械虚位引起的误动作。 */
#define CRSF_CH_VALUE_DEADBAND    40     /* 原始通道死区 (CRSF units)，±40 约 ±2.4% */
#define CONTROL_DEADBAND          15     /* 控制量死区 (-500~+500)，±15 约 ±3% */

/* 高度控制阈值：油门杆须偏离中位超过此值才开始改变抬腿高度。
 * 因为高度是积分控制（每周期累积），阈值需比运动通道的死区更大。
 * 默认 = CONTROL_DEADBAND × 2 ≈ 30，即摇杆偏离约 6% 才响应。 */
#define HEIGHT_CONTROL_THRESHOLD  (CONTROL_DEADBAND * 2)

/* 机身高度线性控制范围 (mm)
 * 摇杆满量程 (±500) 映射到的机身高度偏移。
 * body_pos.y = stick * BODY_HEIGHT_RANGE_MM / 500
 * 正值抬升机身 (腿向下伸展), 负值降低机身 (腿向上收缩) */
#define BODY_HEIGHT_RANGE_MM      40

/* 机身姿态旋转范围 (0.1° 单位)
 * 摇杆满量程 (±500) 映射到的机身旋转角。
 * body_rot = stick * BODY_ROTATION_MAX / 500
 * 200 = 20.0°, 即摇杆推到底时机身倾斜 20° */
#define BODY_ROTATION_MAX         200

/* ---- CRSF 摇杆→控制量 缩放参数 ----
 * 摇杆范围 -500~+500, 映射到实际运动参数 */
#define TRAVEL_MAX_FORWARD_MM   100     /* 满杆前进/后退步长 (mm) */
#define TRAVEL_MAX_STRAFE_MM    80      /* 满杆平移步长 (mm) */
#define TRAVEL_MAX_TURN_MM      60      /* 满杆旋转步长 (mm) */
#define LIFT_SPEED_MM_PER_TICK  3       /* 升降速度 (mm/周期), 油门杆用 */
#define LIFT_HEIGHT_MIN_MM      20      /* 最低抬腿高度 (mm) */
#define LIFT_HEIGHT_MAX_MM      150     /* 最高抬腿高度 (mm) */

/* ---- 速度档位 (CH7 三段开关) ----
 * 通过改变步态推进间隔实现变速 */
#define SPEED_SLOW_MS           90      /* 低速: 步态推进间隔 ms */
#define SPEED_NORMAL_MS         60      /* 中速: 步态推进间隔 ms */
#define SPEED_FAST_MS           35      /* 高速: 步态推进间隔 ms */

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
 *   LR=-135°(后左) LM=-90°(正左) LF=-45°(前左) */
#define COXA_ANGLE_RR       1350    /* 右后: 135° 后方偏右 */
#define COXA_ANGLE_RM       900     /* 右中:  90° 正右方 */
#define COXA_ANGLE_RF       450     /* 右前:  45° 前方偏右 */
#define COXA_ANGLE_LR       -1350   /* 左后: -135° 后方偏左 */
#define COXA_ANGLE_LM       -900    /* 左中:  -90° 正左方 */
#define COXA_ANGLE_LF       -450    /* 左前:  -45° 前方偏左 */

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
 *  ┌──────────── 休息状态 (所有舵机=0°) ────────────┐
 *  │ 股节向上53mm+胫节向下84mm → 足端在基座下方 31mm │
 *  │ coxa45 + 股节水平53 + 胫节水平84 = 水平182mm    │
 *  │ 足端距底板: 31-25(基座高出底板)=6mm ≈ 贴地      │
 *  │ 膝角≈90°, 股节≈46° → 舵机全0° → 底板离地≈7mm  │
 *  │ FOOT_REACH=182, INIT_Y=31                       │
 *  └────────────────────────────────────────────────┘
 *
 *  ┌──────────── 站立状态 (ARM 后, 舵机≠0°) ────────┐
 *  │ 腿弯曲使机身抬高+收脚, 获得稳定支撑和运动空间     │
 *  │ 站立 INIY_Y 更大 (机身更高)                     │
 *  │ 站立 FOOT_REACH 更小 (脚更靠近机身)              │
 *  │ FOOT_REACH=110, INIT_Y=80                       │
 *  │                                                  │
 *  │ IK验证:                                          │
 *  │   d = 110-45 = 65mm                             │
 *  │   a = √(65²+80²) ≈ 103mm                        │
 *  │   膝角 = acos((20025-10625)/18000) ≈ 58°        │
 *  │   tibia_servo = 900-585 = 315 → invert → -32°   │
 *  │   股节 = 83° - 51° = 32°,  servo = 320-450      │
 *  │        = -130 → invert → 13°                     │
 *  │   Coxa舵机 ≈ 0° (足端方向不变)                   │
 *  └────────────────────────────────────────────────┘
 */

/* ---- 休息状态足端 (舵机全0°, 底板贴地) ---- */
#define FOOT_REACH_REST     182     /* 休息: coxa基座→足端水平距离 */
#define INIT_Y_REST          31     /* 休息: coxa基座→足端垂直距离 */

/* ---- 站立状态足端 (ARM后, 机身抬高+收脚) ---- */
#define FOOT_REACH           110     /* 站立: coxa基座→足端水平距离 */
#define INIT_Y               80      /* 站立: coxa基座→足端下深度 */

/* 足端方向角度 (舵机0°时 coxa 指向, 休息和站立共用) */
#define FOOT_DX_RR     -78     /* RR: 110×cos135° */
#define FOOT_DZ_RR      78     /* RR: 110×sin135° */

#define FOOT_DX_RM       0     /* RM: 110×cos90° */
#define FOOT_DZ_RM     110     /* RM: 110×sin90° */

#define FOOT_DX_RF      78     /* RF: 110×cos45° */
#define FOOT_DZ_RF      78     /* RF: 110×sin45° */

#define FOOT_DX_LR     -78     /* LR: 镜像 */
#define FOOT_DZ_LR     -78

#define FOOT_DX_LM       0     /* LM: 镜像 */
#define FOOT_DZ_LM    -110

#define FOOT_DX_LF      78     /* LF: 镜像 */
#define FOOT_DZ_LF     -78

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
#define SERVO_FEMUR_MIN_RR  -900    /* Femur 逆时针极限 (暂定) */
#define SERVO_FEMUR_MAX_RR  900     /* Femur 顺时针极限 (暂定) */
#define SERVO_TIBIA_MIN_RR  -900    /* Tibia 逆时针极限 (暂定) */
#define SERVO_TIBIA_MAX_RR  900     /* Tibia 顺时针极限 (暂定) */

/* ---- 右中腿 (RM) 舵机限位 ---- */
#define SERVO_COXA_MIN_RM   -900
#define SERVO_COXA_MAX_RM   900
#define SERVO_FEMUR_MIN_RM  -900
#define SERVO_FEMUR_MAX_RM  900
#define SERVO_TIBIA_MIN_RM  -900
#define SERVO_TIBIA_MAX_RM  900

/* ---- 右前腿 (RF) 舵机限位 ---- */
#define SERVO_COXA_MIN_RF   -900
#define SERVO_COXA_MAX_RF   900
#define SERVO_FEMUR_MIN_RF  -900
#define SERVO_FEMUR_MAX_RF  900
#define SERVO_TIBIA_MIN_RF  -900
#define SERVO_TIBIA_MAX_RF  900

/* ---- 左后腿 (LR) 舵机限位 ---- */
#define SERVO_COXA_MIN_LR   -900
#define SERVO_COXA_MAX_LR   900
#define SERVO_FEMUR_MIN_LR  -900
#define SERVO_FEMUR_MAX_LR  900
#define SERVO_TIBIA_MIN_LR  -900
#define SERVO_TIBIA_MAX_LR  900

/* ---- 左中腿 (LM) 舵机限位 ---- */
#define SERVO_COXA_MIN_LM   -900
#define SERVO_COXA_MAX_LM   900
#define SERVO_FEMUR_MIN_LM  -900
#define SERVO_FEMUR_MAX_LM  900
#define SERVO_TIBIA_MIN_LM  -900
#define SERVO_TIBIA_MAX_LM  900

/* ---- 左前腿 (LF) 舵机限位 ---- */
#define SERVO_COXA_MIN_LF   -900
#define SERVO_COXA_MAX_LF   900
#define SERVO_FEMUR_MIN_LF  -900
#define SERVO_FEMUR_MAX_LF  900
#define SERVO_TIBIA_MIN_LF  -900
#define SERVO_TIBIA_MAX_LF  900

/* ==================== 舵机ID映射 ==================== */

/*
 * 每个舵机的全局 ID (0~17)，对应 hal_servo_set_angle 的 servo_id 参数。
 *
 * PCA9685 分配:
 *   板 0 (0x40): 舵机 ID 0~8  (右半身 RR+RM+RF, 3腿 × 3关节 = 9 路)
 *   板 1 (0x41): 舵机 ID 9~17 (左半身 LR+LM+LF, 3腿 × 3关节 = 9 路)
 *
 * 每腿 3 关节顺序: Coxa(0) → Femur(1) → Tibia(2)
 *
 * 接线对应关系:
 *   PCA9685 #1 CH0 → 舵机 0 (RR Coxa)
 *   PCA9685 #1 CH1 → 舵机 1 (RR Femur)
 *   PCA9685 #1 CH2 → 舵机 2 (RR Tibia)
 *   ...以此类推...
 *   PCA9685 #2 CH0 → 舵机 9 (LR Coxa)
 */
#define SERVO_RR_COXA       0     /* 右后 Coxa  */
#define SERVO_RR_FEMUR      1     /* 右后 Femur */
#define SERVO_RR_TIBIA      2     /* 右后 Tibia */

#define SERVO_RM_COXA       3     /* 右中 Coxa  */
#define SERVO_RM_FEMUR      4     /* 右中 Femur */
#define SERVO_RM_TIBIA      5     /* 右中 Tibia */

#define SERVO_RF_COXA       6     /* 右前 Coxa  */
#define SERVO_RF_FEMUR      7     /* 右前 Femur */
#define SERVO_RF_TIBIA      8     /* 右前 Tibia */

#define SERVO_LR_COXA       9     /* 左后 Coxa  */
#define SERVO_LR_FEMUR      10    /* 左后 Femur */
#define SERVO_LR_TIBIA      11    /* 左后 Tibia */

#define SERVO_LM_COXA       12    /* 左中 Coxa  */
#define SERVO_LM_FEMUR      13    /* 左中 Femur */
#define SERVO_LM_TIBIA      14    /* 左中 Tibia */

#define SERVO_LF_COXA       15    /* 左前 Coxa  */
#define SERVO_LF_FEMUR      16    /* 左前 Femur */
#define SERVO_LF_TIBIA      17    /* 左前 Tibia */

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
    configs[LEG_RR].coxa_invert = true;
    configs[LEG_RR].femur_invert = true;
    configs[LEG_RR].tibia_invert = true;
    configs[LEG_RR].coxa_horn_offset = 0;
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
    configs[LEG_RM].coxa_invert = true;
    configs[LEG_RM].femur_invert = true;
    configs[LEG_RM].tibia_invert = true;
    configs[LEG_RM].coxa_horn_offset = 0;
    configs[LEG_RM].femur_horn_offset = 0;
    configs[LEG_RM].tibia_horn_offset = 0;

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
    configs[LEG_RF].coxa_invert = true;
    configs[LEG_RF].femur_invert = true;
    configs[LEG_RF].tibia_invert = true;
    configs[LEG_RF].coxa_horn_offset = 0;
    configs[LEG_RF].femur_horn_offset = 0;
    configs[LEG_RF].tibia_horn_offset = 0;

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
    configs[LEG_LR].coxa_horn_offset = 0;
    configs[LEG_LR].femur_horn_offset = 0;
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
    configs[LEG_LM].coxa_horn_offset = 0;
    configs[LEG_LM].femur_horn_offset = 0;
    configs[LEG_LM].tibia_horn_offset = 0;

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
    configs[LEG_LF].coxa_horn_offset = 0;
    configs[LEG_LF].femur_horn_offset = 0;
    configs[LEG_LF].tibia_horn_offset = 0;
}

#endif /* HEXAPOD_CONFIG_H */



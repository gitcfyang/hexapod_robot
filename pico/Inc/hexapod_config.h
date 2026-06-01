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
 * 根据遥控器实际的通道顺序修改以下宏。
 * 查看调试输出 [DBG1] 行的原始值来确定每个通道对应什么功能。
 *
 * 常见预设：
 *   Mode 2 (AETR):  CH1=副翼(转) CH2=升降 CH3=油门(前后) CH4=方向(平移)
 *   Mode 2 (TAER):  CH1=油门(前后) CH2=副翼(转) CH3=升降 CH4=方向(平移)
 *
 * 当前预设：AETR (EdgeTX/OpenTX 默认) */
#define CRSF_CHANNEL_FORWARD      2   // CH3: 油门 (左摇杆Y，无弹簧)
#define CRSF_CHANNEL_STRAFE       3   // CH4: 方向 (左摇杆X)
#define CRSF_CHANNEL_TURN         0   // CH1: 副翼 (右摇杆X)
#define CRSF_CHANNEL_HEIGHT       1   // CH2: 升降 (右摇杆Y)
#define CRSF_CHANNEL_ARM          4   // CH5: 解锁 (二段/三段开关)
#define CRSF_CHANNEL_GAIT         5   // CH6: 步态 (三段开关)
#define CRSF_CHANNEL_SPEED        6   // CH7: 速度
#define CRSF_CHANNEL_BALANCE      7   // CH8: 平衡模式

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
 * 定义：Coxa 舵机 0° 输出时，腿的「正前方」方向。
 * 如果舵机安装在机身上有旋转偏移，用此参数补偿。
 *
 *   正值 = 腿自然指向前方 (前腿)
 *   零   = 腿指向正侧方 (中腿)
 *   负值 = 腿自然指向后方 (后腿)
 *
 *              机头 X+
 *             /   ↑   \
 *     LF=+60°│ 前  │ RF=+60°   (前腿朝前 60°)
 *            │     │
 *     LM= 0° ├──○──┤ RM= 0°    (中腿朝正侧方)
 *            │     │
 *     LR=-60°│ 后  │ RR=-60°   (后腿朝后 60°)
 *             \   │   /
 *
 * 调参方法：
 *   1. !P<id> 0  让舵机输出中位
 *   2. 观察腿的指向
 *   3. 如果 RR 腿指向正侧方而非后方 → 减小 COXA_ANGLE_RR (更负)
 *   4. 目标：COXA_ANGLE_RR 使腿在零位时指向正后方
 */
#define COXA_ANGLE_RR       -450    /* 右后: 朝后 45° */
#define COXA_ANGLE_RM       0       /* 右中: 朝正侧方 */
#define COXA_ANGLE_RF       450     /* 右前: 朝前 45° */
#define COXA_ANGLE_LR       -450    /* 左后: 朝后 45° */
#define COXA_ANGLE_LM       0       /* 左中: 朝正侧方 */
#define COXA_ANGLE_LF       450     /* 左前: 朝前 45° */

/* ---- 初始足端位置 (站立时脚在机身坐标系中的坐标, 单位: mm) ----
 *
 * 每条腿独立定义，因为腿基座不在同一个圆上。
 *
 * 计算公式：
 *   init_pos_x = offset_x + FOOT_DX    (脚在腿基座前方多远)
 *   init_pos_z = offset_z + FOOT_DZ    (脚在腿基座外侧多远)
 *   init_pos_y = INIT_Y               (脚在机身下方多深，所有腿相同)
 *
 * 其中 FOOT_DX/DZ 是「脚相对于 Coxa 转轴」的 XZ 偏移量，
 * 应使脚落在 Coxa 的舒适可达范围内 (通常 30~80mm 的 XZ 距离)。
 *
 * 站立姿态示意 (俯视)：
 *
 *           Z (右) →
 *           │
 *     LF ●  │  ● RF      ● = 足端位置
 *           │
 *     LM ●  │  ● RM      所有脚在机身下方 (init_y > 0)
 *           │
 *     LR ●  │  ● RR
 *           │
 *     ─────○───── X (前) →
 *           │
 *
 *   init_pos_y = 站立时机身离地高度补偿。
 *     值越大 → 腿伸得越直 (机身越高)。
 *     值越小 → 腿弯得越多 (机身越低，重心更稳)。
 *     腿总长 = coxa + femur + tibia = 45+75+120 = 240mm，
 *     典型站立高度约 140~180mm (离地)，对应 init_y ≈ 60~100mm
 *     (具体取决于腿的姿态和重量下压)。
 */

/* 脚相对腿基座的偏移量 (mm)，逐腿可调 */
#define FOOT_DX_RR     -30    /* RR 脚在基座后方 30mm */
#define FOOT_DZ_RR      40    /* RR 脚在基座外侧 (更右) 40mm */

#define FOOT_DX_RM      30    /* RM 脚在基座前方 30mm */
#define FOOT_DZ_RM      30    /* RM 脚在基座外侧 30mm */

#define FOOT_DX_RF      30    /* RF 脚在基座前方 30mm */
#define FOOT_DZ_RF      40    /* RF 脚在基座外侧 40mm */

#define FOOT_DX_LR     -30    /* LR 脚在基座后方 30mm (与 RR 对称) */
#define FOOT_DZ_LR     -40    /* LR 脚在基座外侧 (更左) 40mm, Z 负方向 */

#define FOOT_DX_LM      30    /* LM 脚在基座前方 30mm (与 RM 对称) */
#define FOOT_DZ_LM     -30    /* LM 脚在基座外侧 30mm, Z 负方向 */

#define FOOT_DX_LF      30    /* LF 脚在基座前方 30mm (与 RF 对称) */
#define FOOT_DZ_LF     -40    /* LF 脚在基座外侧 40mm, Z 负方向 */

#define INIT_Y          25    /* 足端在机身下方深度 (所有腿共用) */

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

/* ---- 右后腿 (RR) 舵机限位 (0.1度) ---- */
#define SERVO_COXA_MIN_RR   -260    /* Coxa 逆时针极限 */
#define SERVO_COXA_MAX_RR   740     /* Coxa 顺时针极限 */
#define SERVO_FEMUR_MIN_RR  -1010   /* Femur 逆时针极限 */
#define SERVO_FEMUR_MAX_RR  950     /* Femur 顺时针极限 */
#define SERVO_TIBIA_MIN_RR  -1060   /* Tibia 逆时针极限 */
#define SERVO_TIBIA_MAX_RR  770     /* Tibia 顺时针极限 */

/* ---- 右中腿 (RM) 舵机限位 ---- */
#define SERVO_COXA_MIN_RM   -530
#define SERVO_COXA_MAX_RM   530
#define SERVO_FEMUR_MIN_RM  -1010
#define SERVO_FEMUR_MAX_RM  950
#define SERVO_TIBIA_MIN_RM  -1060
#define SERVO_TIBIA_MAX_RM  770

/* ---- 右前腿 (RF) 舵机限位 ---- */
#define SERVO_COXA_MIN_RF   -580
#define SERVO_COXA_MAX_RF   740
#define SERVO_FEMUR_MIN_RF  -1010
#define SERVO_FEMUR_MAX_RF  950
#define SERVO_TIBIA_MIN_RF  -1060
#define SERVO_TIBIA_MAX_RF  770

/* ---- 左后腿 (LR) 舵机限位 (与 RR 镜像，min/max 符号和范围可能不同) ---- */
#define SERVO_COXA_MIN_LR   -740
#define SERVO_COXA_MAX_LR   260
#define SERVO_FEMUR_MIN_LR  -950
#define SERVO_FEMUR_MAX_LR  1010
#define SERVO_TIBIA_MIN_LR  -770
#define SERVO_TIBIA_MAX_LR  1060

/* ---- 左中腿 (LM) 舵机限位 ---- */
#define SERVO_COXA_MIN_LM   -530
#define SERVO_COXA_MAX_LM   530
#define SERVO_FEMUR_MIN_LM  -950
#define SERVO_FEMUR_MAX_LM  1010
#define SERVO_TIBIA_MIN_LM  -770
#define SERVO_TIBIA_MAX_LM  1060

/* ---- 左前腿 (LF) 舵机限位 ---- */
#define SERVO_COXA_MIN_LF   -740
#define SERVO_COXA_MAX_LF   580
#define SERVO_FEMUR_MIN_LF  -950
#define SERVO_FEMUR_MAX_LF  1010
#define SERVO_TIBIA_MIN_LF  -770
#define SERVO_TIBIA_MAX_LF  1060

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
     *   femur/tibia_horn_offset  : 舵盘安装偏移 (0.1°)，用于修正机械装配误差
     *
     * invert 规则:
     *   右腿 (RR/RM/RF): 全部 true  — 因为右腿舵机在机身右侧，转动方向与左腿镜像
     *   左腿 (LR/LM/LF): 全部 false — 左腿方向与数学模型一致
     *
     *   调参: 用 !P<id> 500 发送正角度。
     *         如果腿往「预期反方向」转 → 切换对应的 invert 值。
     *
     * horn_offset 规则:
     *   如果舵盘 (servo horn) 安装时有角度偏差，用此字段补偿。
     *   例如舵盘偏了 +5° → horn_offset = +50
     *   调参: !P<id> 0 发送中位，观察腿是否在预期的物理零位。
     *         偏多少度就填多少 (单位 0.1°)。
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
    configs[LEG_LF].femur_horn_offset = 0;
    configs[LEG_LF].tibia_horn_offset = 0;
}

#endif /* HEXAPOD_CONFIG_H */



/**
 * @file hexapod_params.h
 * @brief 运行时可调参数表 (串口 !CFG 命令 / 网页 Configurator)
 *
 * 动机:
 *   原本所有可调量都是 hexapod_config.h 里的编译期 #define —— 改一个步长
 *   或电压阈值都要重编译 + 重新烧录 + 重新上电。本模块把它们搬进 RAM:
 *   可串口改、可存 flash 掉电保持、可由网页表单编辑。
 *
 * 与 hexapod_config.h 的关系:
 *   config.h 保留同名宏, 但重定义为 (g_params.<字段>) —— 于是所有既有
 *   调用点一行都不用改, 编译期常量悄悄变成运行时变量。
 *   默认值以 <同名>_DEFAULT 的形式仍写在 config.h, 换板/换机架时改那一处。
 *
 *   config.h  ──#define TRAVEL_MAX_FORWARD_MM (g_params.travel_fwd_mm)
 *   params.c  ──表项 { "travel_fwd_mm", &g_params.travel_fwd_mm, 20, 250,
 *                      TRAVEL_MAX_FORWARD_MM_DEFAULT, "mm", "motion" }
 *
 * 约束:
 *   - 参数一律 int32_t。需要小数的量以定点存 (字段名后缀说明标度,
 *     如 batt_ratio_milli = 分压比 × 1000);
 *   - 表里的 min/max 是"固件侧防呆", 不是安全边界。网页限位与这里一致,
 *     但直接发 !CFG 也会被夹住 —— 越界值可能让 IK/步态解算失败;
 *   - 改参数不校验运动学自洽性 (如 lift_max > 腿长)。抬腿高度这类量
 *     请先把机器人架空虚转再试;
 *   - Flash 记录按"名字 → 值"存, 不按索引, 所以增删表项不会错位;
 *     改名等同于丢弃旧值 (回落默认值), 这是刻意的。
 */
#ifndef HEXAPOD_PARAMS_H
#define HEXAPOD_PARAMS_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== 运行时参数存储 ==================== */

/**
 * @brief 全部运行时参数 (RAM 实体, 定义在 hexapod_params.c)
 *
 * 字段名 = 参数表里的名字, 与 hexapod_config.h 的宏一一对应。
 * 十组 (见 param_t.group), 网页每页显示一到多组 (modes 组在「模式」页,
 * 不是滑条而是矩阵界面)。
 */
typedef struct {
    /* ---- 电池 (batt) ---- */
    int32_t batt_check;          /* 电池保护总开关, 0/1 */
    int32_t batt_ov_mv;          /* 过压阈值 mV */
    int32_t batt_warn_mv;        /* 低压警告 mV */
    int32_t batt_cutoff_mv;      /* 过放截止 mV */
    int32_t batt_recov_mv;       /* 电压恢复 mV */
    int32_t batt_ratio_milli;    /* 分压比 ×1000 (8134 = 8.134) */
    int32_t batt_interval_ms;    /* 电压监测间隔 ms */
    int32_t batt_absent_mv;      /* 未接电池 (USB 供电) 判定门限 mV */

    /* ---- 运动 (motion) ---- */
    int32_t travel_fwd_mm;       /* 满杆前进步长 mm */
    int32_t travel_str_mm;       /* 满杆平移步长 mm */
    int32_t travel_turn_mm;      /* 满杆旋转步长 mm */
    int32_t body_rot_max;        /* 机身姿态最大角 0.1° */
    int32_t body_h_range_mm;     /* 机身高度调节范围 mm */
    int32_t height_integrate;    /* 高度通道: 1=积分(速率) 0=线性(绝对) */
    int32_t lift_min_mm;         /* 最低抬腿高度 mm */
    int32_t lift_max_mm;         /* 最高抬腿高度 mm */
    int32_t gait_min_ms;         /* 满杆步态周期 ms (最快) */
    int32_t gait_max_ms;         /* 微动步态周期 ms (最慢) */

    /* ---- 站立姿态 (stance) ---- */
    int32_t stance_narrow;       /* 窄姿态缩放 % */
    int32_t stance_normal;       /* 正常姿态缩放 % */
    int32_t stance_wide;         /* 宽姿态缩放 % */
    int32_t stance_speed;        /* 姿态过渡速度 ×100/周期 */
    int32_t stance_step_mm;      /* 单腿每周期最大位移 mm */

    /* ---- 手感 / 死区 (tune, 两协议共用) ---- */
    int32_t deadband;            /* 控制量死区 (-500~+500 域) */
    int32_t ch_deadband;         /* 原始通道死区 (raw 172~1811 域) */
    int32_t expo;                /* 摇杆指数曲线 0~100 (%) */
    int32_t h_deadzone;          /* 高度轴死区 (仅积分模式生效) */

    /* ---- 方向取反 (dir, 两协议共用) ---- */
    int32_t inv_fwd;             /* 前进取反 */
    int32_t inv_str;             /* 平移取反 */
    int32_t inv_h;               /* 高度取反 */
    int32_t inv_turn;            /* 旋转取反 */

    /* ---- IMU (imu) ---- */
    int32_t imu_enabled;         /* IMU 姿态补偿开关, 0/1 */
    int32_t imu_gain;            /* 补偿增益 ×10 (10 = 1:1) */
    int32_t imu_roll_sign;       /* Roll 轴符号 ±1 */
    int32_t imu_pitch_sign;      /* Pitch 轴符号 ±1 */

    /* ---- 通道映射 (chan) ----
     * 值为统一通道 id 0~19 (见 hexapod_input.h): CRSF 上 = CH1~CH16,
     * PS2 上 0~3 = 摇杆、4~19 = 按键。 */
    int32_t ch_fwd;              /* 前进/俯仰 */
    int32_t ch_str;              /* 平移/横滚 */
    int32_t ch_turn;             /* 旋转/偏航 */
    int32_t ch_hgt;              /* 机身高度 */
    int32_t input_mode;          /* 默认输入源 0=CRSF 1=PS2 */

    /* ---- 模式矩阵 (modes) ----
     * 值 = ch<<3 | bits (ch: 统一通道 id, 31=未分配; bits: 低1/中2/高4),
     * 打包/解码见 hexapod_input.c, 界面在网页「模式」页。 */
    int32_t mode_arm;            /* 解锁 (电平语义) */
    int32_t mode_bal;            /* 平衡模式 */
    int32_t mode_g0;             /* 步态: 波纹12步 */
    int32_t mode_g1;             /* 步态: 三角6步 */
    int32_t mode_g2;             /* 步态: 三角8步 */
    int32_t mode_g3;             /* 步态: 波浪24步 */
    int32_t mode_g4;             /* 步态: 快速三角4步 */
    int32_t mode_sn;             /* 站立姿态: 窄 */
    int32_t mode_sp;             /* 站立姿态: 正常 */
    int32_t mode_sw;             /* 站立姿态: 宽 */

    /* ---- PS2 组合键 (modes 组) ----
     * 值 = btn_a | btn_b<<5 (16 = 无); 只对 PS2 帧生效, 详见 config.h。 */
    int32_t combo_arm;           /* 解锁/上锁 */
    int32_t combo_bal;           /* 平衡模式 */
    int32_t combo_gnext;         /* 步态: 下一个 */
    int32_t combo_gprev;         /* 步态: 上一个 */
    int32_t combo_snext;         /* 站立姿态: 下一档 */
    int32_t combo_sprev;         /* 站立姿态: 上一档 */
    int32_t combo_lup;           /* 抬腿高度 +5 */
    int32_t combo_ldn;           /* 抬腿高度 -5 */
    int32_t combo_estop;         /* 急停 */

    /* ---- 系统 (sys) ---- */
    int32_t loop_ms;             /* 控制循环周期 (ms); 见 config.h 的缩放说明 */

    /* ---- 预留外设 (per) ----
     * 这些器件都还没接线, 参数控制的是"固件要不要去驱动那几个引脚"。 */
    int32_t led_heartbeat;       /* 主循环绿灯心跳使能 0/1 */
    int32_t led_alarm;           /* 主循环红灯报警使能 0/1 */
    int32_t uart0_en;            /* 外部 UART0 (GP0/GP1) 使能 0/1 */
    int32_t uart0_baud;          /* 外部 UART0 波特率 */
    int32_t ext_i2c_mode;        /* GP26/27 用途 0=关 1=软件 I2C 2=ADC */
} hexapod_params_t;

/** 全局参数实体。config.h 的宏全部指向这里, 因此本符号必须尽早存在。 */
extern hexapod_params_t g_params;

/* ==================== 参数表 ==================== */

/** 表项描述。name/unit/group 均为静态字符串, 不释放。 */
typedef struct {
    const char *name;        /* 唯一名字 (≤ 11 字符, 用于 !CFG 与 flash 记录) */
    int32_t    *ptr;         /* 指向 g_params 中的字段 */
    int32_t     min;         /* 下限 (含) */
    int32_t     max;         /* 上限 (含) */
    int32_t     def;         /* 出厂默认 (= config.h 的 *_DEFAULT) */
    const char *unit;        /* 显示单位, 可为空串 */
    const char *group;       /* 分组: batt/motion/stance/tune/dir/imu/chan/modes/sys/per */
} param_t;

/** @brief 参数表首地址与项数 (只读, 供遍历/网页生成表单用) */
const param_t *params_table(void);
int            params_count(void);

/* ==================== 操作 ==================== */

/** @brief 全部恢复默认值 (不动 flash, 仅 RAM) */
void params_init(void);

/**
 * @brief 按名字查找
 * @return 表项指针, 未找到为 NULL
 */
const param_t *params_find(const char *name);

/**
 * @brief 设置一个参数 (按名字), 超范围自动夹到 min/max
 * @param out_actual 可选, 回填实际生效值 (被夹时与入参不同)
 * @return true = 名字存在且已设置
 */
bool params_set(const char *name, int32_t value, int32_t *out_actual);

/** @brief 单个参数恢复默认 */
bool params_reset(const char *name);

/** @brief 全部参数恢复默认 */
void params_reset_all(void);

/** @brief 是否有参数被改成非默认值 (供 !CFG 提示"未保存") */
int params_changed_count(void);

/* ==================== 输出 (!CFG 系列) ==================== */

/**
 * 一行一个参数。格式刻意做成 key=value, 网页 server.py 用正则抓:
 *   [P] name=travel_fwd_mm val=150 min=20 max=250 def=150 unit=mm grp=motion
 * 改这一行必须同步改 tools/webconfig/server.py 与 test_webconfig.py。
 */
void params_print_all(void);
void params_print_one(const param_t *p);
void params_print_changed(void);

/* ==================== 命令入口 (hal_pico.c 的 !CFG 家族) ==================== */

/** @brief !CFG [name] [value] — 列出/查询/设置, 参数为命令名之后的文本 */
void params_cmd_config(const char *args);

/** @brief !CFGR [name] — 恢复默认 (无参 = 全部) */
void params_cmd_reset(const char *args);

/** @brief !CFGW — 保存到 flash (非安全上下文会拒绝) */
bool params_cmd_save(bool safe_context);

/* ==================== 持久化 ==================== */

/**
 * @brief 从 flash 载入并应用 (无记录时保持默认值)
 * @return 实际应用的参数条数, 0 = 无记录或校验失败
 */
int params_load(void);

/**
 * @brief 保存当前参数到 flash (仅存与默认值不同的项)
 * @param safe_context true = 机器人静止, 允许整扇区擦除
 * @return true = 写入并校验成功
 */
bool params_save(bool safe_context);

#endif /* HEXAPOD_PARAMS_H */

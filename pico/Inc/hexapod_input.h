/**
 * @file hexapod_input.h
 * @brief 统一输入层: CRSF / PS2 共用同一套通道与参数
 *
 * 动机:
 *   CRSF 与 PS2 各有自己的摇杆量程、死区、取反和开关解析, 参数表里因此
 *   出现两套几乎重复的项 (inv_* / ps2_inv_*), 网页遥控页也要分两摊维护。
 *   本模块把「协议 → 控制状态」这段收敛到一处: 两个驱动各自把原始数据
 *   翻译成**统一通道 id (0~19) 的 CRSF 量程值**, 之后的行为完全一致。
 *
 * 统一通道 id (两协议同一套 ch_* 参数、同一套模式矩阵):
 *   0 = 平移/横滚   1 = 前进/俯仰   2 = 机身高度   3 = 转向/偏航
 *   4~19 = 开关/按钮
 *     - CRSF: 0~15 = CH1~CH16 原值 (172~1811), 16~19 恒为中位 992;
 *     - PS2:  0~3 = RX/RY/LY/LX 摇杆 (0~255 → CRSF 量程, 带中位校准),
 *            4~19 = 16 个按键 (按下 1811 / 松开 172), 位序同 PSB_* 宏。
 *
 *   `[CH]` 遥测与网页通道面板仍按**协议原值**显示 (CRSF 992 / PS2 128 之类),
 *   归一化只发生在固件内部, 所以页面不需要知道这里做了什么。
 *
 * 每帧求值顺序 (与文件内实现一一对应):
 *   ① 模式矩阵 (mode_* 参数): 每个模式 = 通道 + 激活区间 + 触发方式
 *      (按住生效 / 单击触发);
 *   ② 摇杆: 映射 → 死区 → expo → 取反;
 *   ③ 高度: 积分 (height_integrate=1) 或线性 (=0);
 *   ④ 行程/姿态共享段 (平衡模式与正常模式各一套);
 *   ⑤ 组合键 (两协议, 单击 = 边沿触发 / 按住 = 期间生效, 和弦优先);
 *   ⑥ 抬腿高度钳位。
 *
 * 注意:
 *   - 本模块不依赖 hexapod_ps2.h / hexapod_crsf.h, USB 构建也能编译;
 *   - 激活的模式每帧重新写 gait_type / stance_mode, 所以 !G / !M 这类
 *     一次性命令会被下一帧覆盖 —— 与 CH6 开关在改动前的行为一致;
 *   - 输入源切换时调 hexapod_input_reset(): 清高度积分器与按键沿状态,
 *     否则切回来第一次按键会被当成"一直按着"而误触发。
 */
#ifndef HEXAPOD_INPUT_H
#define HEXAPOD_INPUT_H

#include <stdint.h>
#include <stdbool.h>
#include "hexapod_types.h"

/* ==================== 通道 id ==================== */

#define HEXINP_CH_COUNT     20   /* 统一通道数 (CRSF 16 通道 + 4 个伪通道) */

/* 前 4 个槽位对 PS2 是固定的摇杆顺序 (硬件约定, 不可改配):
 * 通道号可以改 (ch_* 参数指的就是这些 id), 但"槽位 2 是高度"这类语义固定。
 * 默认 ch_* 参数 (str=0 fwd=1 hgt=2 turn=3) 正是这套顺序。 */
#define HEXINP_CH_STR       0    /* 平移 / 横滚 */
#define HEXINP_CH_FWD       1    /* 前进 / 俯仰 */
#define HEXINP_CH_HGT       2    /* 机身高度 */
#define HEXINP_CH_TURN      3    /* 转向 / 偏航 */
#define HEXINP_CH_BTN_BASE  4    /* 4~19: 16 个按键 (位序同 PSB_*) */

/* ==================== 统一量程 (CRSF 原始标度) ====================
 *
 * 全固件唯一的输入标度: 172(最低) ~ 992(中位) ~ 1811(最高), 跨度 819 每侧。
 * CRSF 通道本来就是这套值; PS2 摇杆与按键由驱动换算过来 (按键按下 = 1811)。
 * 模式矩阵的档位判定也按这套值: <792 低 / >1192 高 / 其余中。 */
#define CRSF_CH_VALUE_MIN   172
#define CRSF_CH_VALUE_MID   992
#define CRSF_CH_VALUE_MAX   1811

/* ==================== 协议标识 ==================== */

#define HEXINP_PROTO_CRSF   0    /* 无线电 (ELRS) 帧 */
#define HEXINP_PROTO_PS2    1    /* PS2 手柄帧; 组合键在这条路径上用真按键求值 */

/* ==================== 模式参数打包 (mode_*) ====================
 *
 * mode_* 参数值 = ch<<3 | bits | click<<8, 范围 0~511:
 *   ch    = 统一通道 id 0~19; 31 = 未分配 (该模式永不激活)
 *   bits  = 允许的档位掩码: LOW=1, MID=2, HIGH=4 (可多选)
 *   click = 1 单击触发 (进入激活区间的沿动作一次)
 *           0 按住生效 (停在激活区间内期间保持, 默认)
 *
 * 档位按原始值判定: raw < 792 → LOW, raw > 1192 → HIGH, 其余 MID。
 * 于是同一个模式既能接 CRSF 二段开关 (只勾 HIGH), 也能接 PS2 按键
 * (按下 = HIGH, 松开 = LOW)。
 *
 * 「单击」效果按目标: 解锁/平衡 = 进入沿翻转一次 (复刻更早的边沿语义),
 * 步态/站位 = 进入沿选择一次 (锁存, 离开再进才再触发)。多行目标
 * (步态/站位) 若同一帧里既有电平行激活又有单击行进入沿, **电平优先**:
 * 只要有任一行按住型激活, 本帧就不处理单击行。
 *
 * 解码时必须**先掩低 8 位**再取 ch (bit8 属于 click, 直接整体右移会渗进 ch)。 */
#define HEXINP_MODE_CH_UNASSIGNED  31
#define HEXINP_MODE_LOW            1
#define HEXINP_MODE_MID            2
#define HEXINP_MODE_HIGH           4

/* ==================== 组合键参数打包 (combo_*) ====================
 *
 * combo_* 参数值 = btn_a | btn_b<<5 | hold<<10, 范围 0~2047:
 *   btn_x = 按键下标 0~15 (同 PSB_* 位序); 16 = 无
 *   btn_b = 16  → 单键: 点位 btn_a (或按住, 见 hold)
 *   btn_b ≠ 16  → 和弦: 按住 btn_a, 再操作 btn_b
 *   hold  = 1 按住生效: 解锁/平衡按住期间保持; 一次性动作 (切步态/站位、
 *             抬腿、急停) 按住每 400ms 重复触发一次
 *           0 单击触发: 上升沿动作一次 (默认, 复刻旧行为)
 *
 * 按键来源: PS2 = 真按键; CRSF = 通道 CH5~CH16 (值 > 1192 算按下,
 * 即开关拨到高位), 和弦因此也能用两个开关表达。
 *
 * 和弦优先: 和弦触发后, 本帧 btn_a / btn_b 的沿即被消耗,
 * 单键组合键不会再跟着触发 (复刻旧代码「× 按住时 D-Pad 让位给抬腿」)。 */
#define HEXINP_COMBO_NONE          16

/** 按住生效的一次性动作用它做连发间隔 (与主循环周期解耦, 不随 loop_ms 变) */
#define HEXINP_COMBO_HOLD_REPEAT_MS  400u

/* ==================== 模式目标 (矩阵行与组合键的归属) ====================
 *
 * 两协议各有让位规则, 避免同一个开关既走矩阵又走组合键:
 *   - PS2 帧: 若某目标**已配置组合键**, 该目标的矩阵行被跳过 ——
 *     否则默认模式下按 SELECT (组合键 = 平衡) 会同时命中矩阵行里的
 *     解锁通道, 解锁就被"按住即生效"的电平语义锁死了;
 *   - CRSF 帧: 反过来, 某目标的矩阵行**已分配**时该目标的组合键让位 ——
 *     默认 CH5~CH8 都走矩阵, 组合键 (若有配) 不抢。
 * 于是默认配置下两协议的行为与参数化之前逐字节相同。 */
typedef enum {
    HEXINP_TGT_ARM = 0,     /* 解锁/锁定 */
    HEXINP_TGT_BAL,         /* 平衡模式 */
    HEXINP_TGT_GAIT,        /* 步态 (g0~g4 绝对选择) */
    HEXINP_TGT_STANCE,      /* 站立姿态 (窄/正常/宽) */
    HEXINP_TGT_COUNT
} hexinp_target_t;

/* ==================== API ==================== */

/**
 * @brief 把一帧统一通道数据应用到控制状态
 * @param proto       HEXINP_PROTO_CRSF / HEXINP_PROTO_PS2
 * @param channels    统一通道数组 (长度 HEXINP_CH_COUNT), CRSF 量程 172~1811
 * @param buttons     16 位按键掩码, **置位 = 按下**。PS2 帧由驱动取反
 *                    (原始帧"按下=0"); CRSF 帧由驱动从通道 CH5~CH16 派生
 *                    (>1192 算按下; 16~19 恒中位 → 高 4 位恒 0)
 * @param ctrl_state  机器人控制状态 (原地更新)
 */
void hexapod_input_apply(uint8_t proto, const uint16_t *channels,
                         uint16_t buttons, control_state_t *ctrl_state);

/**
 * @brief 清空跨帧状态 (高度积分器 + 按键沿 + 组合键按住计时 + 模式跳变检测)
 * @note  输入源切换 / 解析器重置时调用, 避免把上一路的残留当成这一路的输入
 */
void hexapod_input_reset(void);

#endif /* HEXAPOD_INPUT_H */

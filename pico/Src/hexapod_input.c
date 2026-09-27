/**
 * @file hexapod_input.c
 * @brief 统一输入层实现 (设计说明见 hexapod_input.h)
 *
 * 本文件里的映射原语 (map_channel_to_control / apply_control_deadband /
 * expo_curve) 原在 hexapod_crsf.c 与 hexapod_ps2.c 各有一份, 现集中到这里;
 * 两个驱动只剩「把原始数据翻译成统一通道」的薄包装。
 */

#include "hexapod_input.h"

#include "hexapod_config.h"
#include "hexapod_gait.h"
#include "hexapod_hal.h"

/* ==================== 映射原语 ==================== */

/**
 * @brief 原始通道值 (172~1811) → 控制量 (-500~+500)
 *
 * 中位两侧各留一段死区 (ch_deadband), 剩下的行程线性映射到 0~500,
 * 于是摇杆手感不随死区大小改变 (死区只是把有效行程往后挪)。
 */
static int16_t map_channel_to_control(uint16_t ch_value)
{
    int16_t result;

    if (ch_value < CRSF_CH_VALUE_MID - CH_VALUE_DEADBAND) {
        /* 负方向 */
        int16_t diff = CRSF_CH_VALUE_MID - CH_VALUE_DEADBAND - ch_value;
        int32_t range = CRSF_CH_VALUE_MID - CH_VALUE_DEADBAND - CRSF_CH_VALUE_MIN;
        if (range > 0) {
            result = (int16_t)(-500 * (int32_t)diff / range);
        } else {
            result = 0;
        }
        if (result < -500) result = -500;
    } else if (ch_value > CRSF_CH_VALUE_MID + CH_VALUE_DEADBAND) {
        /* 正方向 */
        int16_t diff = ch_value - CRSF_CH_VALUE_MID - CH_VALUE_DEADBAND;
        int32_t range = CRSF_CH_VALUE_MAX - (CRSF_CH_VALUE_MID + CH_VALUE_DEADBAND);
        if (range > 0) {
            result = (int16_t)(500 * (int32_t)diff / range);
        } else {
            result = 0;
        }
        if (result > 500) result = 500;
    } else {
        /* 死区内，输出 0 */
        result = 0;
    }

    return result;
}

/**
 * @brief 第 2 级死区: 把映射后的控制量中小于阈值的值归零
 *
 * 与第 1 级 (map_channel_to_control 里的 ch_deadband) 串联,
 * 确保摇杆偏离中位足够远时才产生运动。
 */
static int16_t apply_control_deadband(int16_t value)
{
    if (value > -CONTROL_DEADBAND && value < CONTROL_DEADBAND) {
        return 0;
    }
    return value;
}

/**
 * @brief 指数曲线 (expo): 压缩中位附近斜率, 满杆仍为满量程
 * @param x   输入控制量 (-500 ~ +500)
 * @param mix 混合比例 0~100: 0 = 纯线性 (禁用), 100 = 纯三次方
 *
 * out = (linear*(100-mix) + cubic*mix) / 100, cubic = x³/500²
 * 纯三次方参考: 25% 杆量→1.56% 输出, 50%→12.5%, 100%→100%
 * 默认 0 (线性) —— 两协议共用, 需要钝化中心区时再调大。
 */
static int16_t expo_curve(int16_t x, int32_t mix)
{
    if (mix <= 0 || x == 0) return x;
    if (mix > 100) mix = 100;
    int32_t cubic = (int32_t)x * x * x / (500 * 500);
    return (int16_t)(((int32_t)x * (100 - mix) + cubic * mix) / 100);
}

/* ==================== 模式 / 组合键解码 ==================== */

/** 解包后的模式项 */
typedef struct {
    uint8_t ch;      /* 统一通道 id; HEXINP_MODE_CH_UNASSIGNED = 未分配 */
    uint8_t bits;    /* 激活档位掩码 (LOW|MID|HIGH); 0 = 未分配 */
    bool    click;   /* true = 单击触发 (进档沿一次); false = 按住生效 */
} mode_t;

/** 解包后的组合键 */
typedef struct {
    uint8_t a;       /* 主键下标 0~15; HEXINP_COMBO_NONE = 无 */
    uint8_t b;       /* 副键下标 0~15; HEXINP_COMBO_NONE = 单键 */
    bool    hold;    /* true = 按住生效 (一次性动作连发); false = 单击触发 */
} combo_t;

/* 参数表里的值可能被 !CFG 直接写成任意整数, 越界一律当"未分配"处理,
 * 不让一个手滑的数字变成"每次都激活"或越界访问通道数组。 */
static mode_t mode_unpack(int32_t v)
{
    mode_t m;
    /* click 在 bit8, 必须先掩低 8 位再取 ch —— 否则它渗进 ch 的高位 */
    int32_t lo = v & 0xFF;
    m.ch    = (uint8_t)((lo >> 3) & 0x1F);
    m.bits  = (uint8_t)(lo & 0x7);
    m.click = ((v >> 8) & 1) != 0;
    if (m.ch >= HEXINP_CH_COUNT) {
        m.ch    = HEXINP_MODE_CH_UNASSIGNED;
        m.bits  = 0;
        m.click = false;
    }
    return m;
}

static combo_t combo_unpack(int32_t v)
{
    combo_t c;
    c.a    = (uint8_t)(v & 0x1F);
    c.b    = (uint8_t)((v >> 5) & 0x1F);
    c.hold = ((v >> 10) & 1) != 0;
    if (c.a > 15) c.a = HEXINP_COMBO_NONE;
    if (c.b > 15) c.b = HEXINP_COMBO_NONE;
    return c;
}

/** 该模式项是否已分配 (通道在范围内且至少勾了一个档位) */
static bool mode_assigned(mode_t m)
{
    return (m.ch != HEXINP_MODE_CH_UNASSIGNED) && (m.bits != 0);
}

/** 模式项在当前通道值下是否激活 */
static bool mode_active(mode_t m, const uint16_t *channels)
{
    if (!mode_assigned(m)) return false;

    uint16_t raw = channels[m.ch];
    uint8_t level;
    if (raw < CRSF_CH_VALUE_MID - 200)      level = HEXINP_MODE_LOW;
    else if (raw > CRSF_CH_VALUE_MID + 200) level = HEXINP_MODE_HIGH;
    else                                    level = HEXINP_MODE_MID;

    return (m.bits & level) != 0;
}

/* ==================== 跨帧状态 ==================== */

/* 组合键参数表的固定顺序 (与 mode 目标的对应关系写死在下面的效果段) */
enum {
    C_ARM = 0, C_BAL, C_GNEXT, C_GPREV, C_SNEXT, C_SPREV, C_LUP, C_LDN, C_ESTOP,
    C_COUNT
};

/* 高度积分器: 定点 1/64 —— 满行程 (±500) 每帧累积, 16ms 帧下满量程约 2 秒;
 * 小幅度 = 缓慢变化, 天然滤除手持微抖。实际控制量 = 积分值 / 64。 */
#define HEIGHT_INTEGRAL_SCALE  64
static int32_t s_height_integral   = 0;
static int8_t  s_prev_integrate    = -1;   /* -1 = 尚未求值过 (首次调用不算跳变) */
static int16_t s_last_height_ctrl  = 0;    /* 上一帧实际使用的高度控制量 */

/* 按键沿状态: 单击组合键靠它算上升沿 (按住型不用, 但统一维护) */
static uint16_t s_prev_buttons = 0;

/* 模式行"上一帧是否激活", 顺序 = s_mode_all (arm, bal, g0..g4, sn, sp, sw);
 * 单击行靠它算进入沿。不放在组合键跳过门里 —— 否则改配置后残留假沿。 */
static uint8_t s_mode_active_prev[10] = { 0 };

/* 组合键按住计时: 只对 hold=1 生效。激活沿起表 (s_combo_held=1),
 * 条件消失清零 —— 于是中途改配置不会残留"按住"状态。 */
static uint32_t s_combo_last_ms[C_COUNT] = { 0 };
static uint8_t  s_combo_held[C_COUNT]    = { 0 };

void hexapod_input_reset(void)
{
    s_height_integral  = 0;
    s_prev_integrate   = -1;
    s_last_height_ctrl = 0;
    s_prev_buttons     = 0;
    for (int i = 0; i < 10; i++) s_mode_active_prev[i] = 0;
    for (int i = 0; i < C_COUNT; i++) {
        s_combo_last_ms[i] = 0;
        s_combo_held[i]    = 0;
    }
}

/* ==================== 组合键 ==================== */

static const int32_t *const s_combo[C_COUNT] = {
    &COMBO_ARM,   &COMBO_BAL,
    &COMBO_GNEXT, &COMBO_GPREV,
    &COMBO_SNEXT, &COMBO_SPREV,
    &COMBO_LUP,   &COMBO_LDN,
    &COMBO_ESTOP,
};

/* 步态矩阵行 (索引 = gait_type_t 的值) 与站立姿态矩阵行 (值 -1/0/+1) */
static const int32_t *const s_mode_gait[5] = {
    &MODE_G0, &MODE_G1, &MODE_G2, &MODE_G3, &MODE_G4,
};
static const int32_t *const s_mode_stance[3] = {
    &MODE_SN, &MODE_SP, &MODE_SW,
};
static const int8_t s_stance_value[3] = { -1, 0, 1 };

/* 全部模式行, 顺序与 s_mode_active_prev 一致 —— 每帧末尾无条件整表刷新沿状态 */
static const int32_t *const s_mode_all[10] = {
    &MODE_ARM, &MODE_BAL,
    &MODE_G0, &MODE_G1, &MODE_G2, &MODE_G3, &MODE_G4,
    &MODE_SN, &MODE_SP, &MODE_SW,
};

/** 进入平衡模式时蜂鸣一声 (单击翻转与按住上升沿共用) */
static void balance_beep(void)
{
    static const uint16_t tone[]  = {1500};
    static const uint16_t tdur[]  = {80};
    hal_play_sound(1, tone, tdur);
}

/** 在"已分配的步态"之间循环 (dir=+1 下一个 / -1 上一个) */
static void gait_cycle(control_state_t *cs, int8_t dir)
{
    int8_t list[5];
    int n = 0;
    for (int i = 0; i < 5; i++) {
        if (mode_assigned(mode_unpack(*s_mode_gait[i]))) list[n++] = (int8_t)i;
    }
    if (n == 0) return;   /* 一个步态都没分配 → 组合键不做事 */

    int cur = -1;
    for (int i = 0; i < n; i++) {
        if (list[i] == (int8_t)cs->gait_type) { cur = i; break; }
    }
    int next;
    if (cur < 0) {
        next = (dir > 0) ? 0 : n - 1;   /* 当前步态不在循环内 → 从循环起点进 */
    } else {
        next = (cur + dir) % n;
        if (next < 0) next += n;
    }

    uint8_t g = (uint8_t)list[next];
    if (cs->gait_type != g) {
        cs->gait_type = g;
        hexapod_gait_select((gait_type_t)g, cs);
    }
}

/** 站立姿态循环: -1 窄 → 0 正常 → +1 宽 (回绕) */
static void stance_step(control_state_t *cs, int8_t dir)
{
    int8_t s = cs->stance_mode;
    if (dir > 0) s = (s >= 1)  ? -1 : (int8_t)(s + 1);
    else         s = (s <= -1) ?  1 : (int8_t)(s - 1);
    cs->stance_mode = s;
}

/** 组合键的"按住条件": 单键 = 该键按下; 和弦 = 两键都按下 */
static bool combo_level(uint16_t buttons, uint16_t mask_a, uint16_t mask_b)
{
    if (!(buttons & mask_a)) return false;
    return (mask_b == 0) || ((buttons & mask_b) != 0);
}

/**
 * @brief 求值全部组合键 (两协议: PS2 真按键 / CRSF 从通道派生的掩码)
 * @param skip_mask  位 i = 1 → 跳过组合键 i (该目标在 CRSF 上由矩阵行接管)
 *
 * 两遍扫描: 先和弦 (btn_b ≠ 无), 后单键。和弦触发后消耗两个键的沿,
 * 单键不会再跟着触发 —— 否则 CROSS+UP 会同时命中「抬腿 +」和「步态切换」。
 *
 * hold=0 (单击, 默认): 上升沿触发一次, 与旧行为一致。
 * hold=1 (按住): 解锁/平衡按住期间保持; 一次性动作按下即触发, 继续按住
 * 每 HEXINP_COMBO_HOLD_REPEAT_MS 重复一次 (用 hal 毫秒时间戳计时, 与
 * loop_ms / 帧率解耦)。s_combo_held 由激活沿置位、条件消失清零, 于是
 * 中途改配置不会残留"按住"状态。
 */
static void combos_apply(uint16_t buttons, control_state_t *cs, uint32_t skip_mask)
{
    const uint32_t now = hal_get_tick_ms();
    uint16_t edges = (uint16_t)(buttons & ~s_prev_buttons);   /* 本帧上升沿 */
    uint16_t consumed = 0;
    bool fired[C_COUNT] = { false };

    combo_t  combos[C_COUNT];
    uint16_t maska[C_COUNT];
    uint16_t maskb[C_COUNT];
    uint8_t  prev_held[C_COUNT];
    for (int i = 0; i < C_COUNT; i++) {
        combos[i]  = combo_unpack(*s_combo[i]);
        maska[i]   = (combos[i].a != HEXINP_COMBO_NONE) ? (uint16_t)(1u << combos[i].a) : 0;
        maskb[i]   = (combos[i].b != HEXINP_COMBO_NONE) ? (uint16_t)(1u << combos[i].b) : 0;
        prev_held[i] = s_combo_held[i];
    }

    for (int i = 0; i < C_COUNT; i++) {          /* 第一遍: 和弦 */
        if (maska[i] == 0 || maskb[i] == 0) continue;
        if (skip_mask & (1u << i)) { s_combo_held[i] = 0; continue; }
        if ((buttons & maska[i]) && (edges & maskb[i])) {   /* 按住 A, 点 B */
            fired[i] = true;
            consumed |= (uint16_t)(maska[i] | maskb[i]);
            s_combo_last_ms[i] = now;
            s_combo_held[i] = 1;
        } else if (combos[i].hold && (buttons & maska[i]) && (buttons & maskb[i])) {
            if (s_combo_held[i] &&
                (uint32_t)(now - s_combo_last_ms[i]) >= HEXINP_COMBO_HOLD_REPEAT_MS) {
                fired[i] = true;
                s_combo_last_ms[i] = now;
            }
        } else {
            s_combo_held[i] = 0;
        }
    }

    for (int i = 0; i < C_COUNT; i++) {          /* 第二遍: 单键 */
        if (maska[i] == 0 || maskb[i] != 0) continue;
        if (skip_mask & (1u << i)) { s_combo_held[i] = 0; continue; }
        if (edges & maska[i] & ~consumed) {
            fired[i] = true;
            s_combo_last_ms[i] = now;
            s_combo_held[i] = 1;
        } else if (combos[i].hold && (buttons & maska[i])) {
            if (s_combo_held[i] &&
                (uint32_t)(now - s_combo_last_ms[i]) >= HEXINP_COMBO_HOLD_REPEAT_MS) {
                fired[i] = true;
                s_combo_last_ms[i] = now;
            }
        } else {
            s_combo_held[i] = 0;
        }
    }

    /* ---- 效果 ----
     * hold 组合键被跳过时连电平都不写 —— 该目标这一帧由矩阵行负责。 */
    if (maska[C_ARM]) {
        if (combos[C_ARM].hold) {
            if (!(skip_mask & (1u << C_ARM)))
                cs->robot_on = combo_level(buttons, maska[C_ARM], maskb[C_ARM]);
        } else if (fired[C_ARM]) {
            cs->robot_on = !cs->robot_on;
        }
    }

    if (maska[C_BAL]) {
        if (combos[C_BAL].hold) {
            if (!(skip_mask & (1u << C_BAL))) {
                bool on = combo_level(buttons, maska[C_BAL], maskb[C_BAL]);
                cs->balance_mode = on;
                if (on && !prev_held[C_BAL]) balance_beep();   /* 进入平衡: 蜂鸣一声 */
            }
        } else if (fired[C_BAL]) {
            cs->balance_mode = !cs->balance_mode;
            if (cs->balance_mode) balance_beep();
        }
    }

    if (fired[C_GNEXT]) gait_cycle(cs, +1);
    if (fired[C_GPREV]) gait_cycle(cs, -1);
    if (fired[C_SNEXT]) stance_step(cs, +1);
    if (fired[C_SPREV]) stance_step(cs, -1);

    if (fired[C_LUP]) {
        cs->leg_lift_height += 5;
        if (cs->leg_lift_height > LIFT_HEIGHT_MAX_MM) cs->leg_lift_height = LIFT_HEIGHT_MAX_MM;
    }
    if (fired[C_LDN]) {
        cs->leg_lift_height -= 5;
        if (cs->leg_lift_height < LIFT_HEIGHT_MIN_MM) cs->leg_lift_height = LIFT_HEIGHT_MIN_MM;
    }

    /* 急停放在行程/姿态求值之后执行, 否则清掉的量会被本帧的摇杆映射写回来 */
    if (maska[C_ESTOP]) {
        bool estop = fired[C_ESTOP];
        if (combos[C_ESTOP].hold && !(skip_mask & (1u << C_ESTOP)))
            estop = combo_level(buttons, maska[C_ESTOP], maskb[C_ESTOP]);
        if (estop) {
            cs->robot_on = false;
            cs->travel_length.x = 0;
            cs->travel_length.y = 0;
            cs->travel_length.z = 0;
            cs->body_rot.x = 0;
            cs->body_rot.y = 0;
            cs->body_rot.z = 0;
        }
    }

    s_prev_buttons = buttons;
}

/* ==================== 主入口 ==================== */

void hexapod_input_apply(uint8_t proto, const uint16_t *channels,
                         uint16_t buttons, control_state_t *ctrl_state)
{
    if (!channels || !ctrl_state) return;

    const bool is_ps2 = (proto == HEXINP_PROTO_PS2);
    const bool combo_arm_assigned    = is_ps2 && (combo_unpack(COMBO_ARM).a    != HEXINP_COMBO_NONE);
    const bool combo_bal_assigned    = is_ps2 && (combo_unpack(COMBO_BAL).a    != HEXINP_COMBO_NONE);
    const bool combo_gait_assigned   = is_ps2 && ((combo_unpack(COMBO_GNEXT).a != HEXINP_COMBO_NONE) ||
                                                  (combo_unpack(COMBO_GPREV).a != HEXINP_COMBO_NONE));
    const bool combo_stance_assigned = is_ps2 && ((combo_unpack(COMBO_SNEXT).a != HEXINP_COMBO_NONE) ||
                                                  (combo_unpack(COMBO_SPREV).a != HEXINP_COMBO_NONE));

    /* CRSF 帧的组合键让位表: 某目标矩阵行已分配 → 该目标的组合键跳过
     * (矩阵优先)。默认 CH5~CH8 都走矩阵, 组合键不抢; 把矩阵行设成
     * 「未分配」即把该目标交给组合键。PS2 帧方向相反, 由上面的
     * combo_*_assigned 在矩阵一侧处理。 */
    uint32_t combo_skip = 0;
    if (!is_ps2) {
        if (mode_assigned(mode_unpack(MODE_ARM))) combo_skip |= 1u << C_ARM;
        if (mode_assigned(mode_unpack(MODE_BAL))) combo_skip |= 1u << C_BAL;
        for (int i = 0; i < 5; i++) {
            if (mode_assigned(mode_unpack(*s_mode_gait[i]))) {
                combo_skip |= (1u << C_GNEXT) | (1u << C_GPREV);
                break;
            }
        }
        for (int i = 0; i < 3; i++) {
            if (mode_assigned(mode_unpack(*s_mode_stance[i]))) {
                combo_skip |= (1u << C_SNEXT) | (1u << C_SPREV);
                break;
            }
        }
    }

    /* ---- ① 模式矩阵 ----
     * 目标已配组合键 (仅 PS2) → 该目标的矩阵行让位, 否则默认表下
     * PS2 按键会顺带命中矩阵行里的 CRSF 通道号。
     * 行内 click=1 为单击触发: 只在进入激活区间的沿动作一次;
     * 多行目标 (步态/站位) 电平优先 —— 有任一行按住型激活就不理单击行。 */
    if (!combo_arm_assigned) {
        mode_t m = mode_unpack(MODE_ARM);
        if (mode_assigned(m)) {
            if (m.click) {
                if (mode_active(m, channels) && !s_mode_active_prev[0])
                    ctrl_state->robot_on = !ctrl_state->robot_on;
            } else {
                ctrl_state->robot_on = mode_active(m, channels);
            }
        }
    }

    if (!combo_bal_assigned) {
        mode_t m = mode_unpack(MODE_BAL);
        if (mode_assigned(m)) {
            if (m.click) {
                if (mode_active(m, channels) && !s_mode_active_prev[1]) {
                    ctrl_state->balance_mode = !ctrl_state->balance_mode;
                    if (ctrl_state->balance_mode) balance_beep();
                }
            } else {
                ctrl_state->balance_mode = mode_active(m, channels);
            }
        }
    }

    if (!combo_gait_assigned) {
        int8_t want = -1;           /* 表序迭代, 最后一个激活的行生效 */
        bool level_active = false;
        for (int i = 0; i < 5; i++) {
            mode_t m = mode_unpack(*s_mode_gait[i]);
            if (!m.click && mode_active(m, channels)) {
                want = (int8_t)i;
                level_active = true;
            }
        }
        if (!level_active) {        /* 无电平行激活时才看单击行 (进档沿选一次) */
            for (int i = 0; i < 5; i++) {
                mode_t m = mode_unpack(*s_mode_gait[i]);
                if (m.click && mode_active(m, channels) && !s_mode_active_prev[2 + i]) {
                    want = (int8_t)i;
                }
            }
        }
        if (want >= 0 && ctrl_state->gait_type != (uint8_t)want) {
            ctrl_state->gait_type = (uint8_t)want;
            hexapod_gait_select((gait_type_t)want, ctrl_state);
        }
    }

    if (!combo_stance_assigned) {
        int8_t want = 0;
        bool found = false;
        bool level_active = false;
        for (int i = 0; i < 3; i++) {
            mode_t m = mode_unpack(*s_mode_stance[i]);
            if (!m.click && mode_active(m, channels)) {
                want = s_stance_value[i];
                found = true;
                level_active = true;
            }
        }
        if (!level_active) {
            for (int i = 0; i < 3; i++) {
                mode_t m = mode_unpack(*s_mode_stance[i]);
                if (m.click && mode_active(m, channels) && !s_mode_active_prev[7 + i]) {
                    want = s_stance_value[i];
                    found = true;
                }
            }
        }
        if (found && ctrl_state->stance_mode != want) {
            ctrl_state->stance_mode = want;   /* 主循环随后调 hexapod_apply_stance() */
        }
    }

    /* 沿状态整表刷新: 无条件执行 (不经组合键跳过门) —— 否则改配置后
     * 矩阵行恢复求值的那一帧会拿旧值算出假沿。 */
    for (int i = 0; i < 10; i++) {
        s_mode_active_prev[i] = mode_active(mode_unpack(*s_mode_all[i]), channels) ? 1 : 0;
    }

    /* ---- ② 摇杆: 映射 → 死区 → expo → 取反 ---- */
    int16_t forward = apply_control_deadband(map_channel_to_control(channels[INPUT_CH_FWD]));
    int16_t strafe  = apply_control_deadband(map_channel_to_control(channels[INPUT_CH_STR]));
    int16_t turn    = apply_control_deadband(map_channel_to_control(channels[INPUT_CH_TURN]));

    forward = expo_curve(forward, STICK_EXPO);
    strafe  = expo_curve(strafe,  STICK_EXPO);
    turn    = expo_curve(turn,    STICK_EXPO);

    if (FORWARD_DIRECTION_INVERT) forward = -forward;
    if (STRAFE_DIRECTION_INVERT)  strafe  = -strafe;
    if (TURN_DIRECTION_INVERT)    turn    = -turn;

    /* ---- ③ 高度: 积分 (速率输入) 或线性 (绝对位置) ---- */
    int16_t height_ctrl;
    int16_t height_raw = map_channel_to_control(channels[INPUT_CH_HGT]);
    const bool integrate = (HEIGHT_INTEGRATE != 0);

    if (integrate && s_prev_integrate == 0) {
        /* 线性 → 积分: 用上一帧的线性值给积分器续上, 高度不跳变 */
        s_height_integral = (int32_t)s_last_height_ctrl * HEIGHT_INTEGRAL_SCALE;
    }
    s_prev_integrate = integrate ? 1 : 0;

    if (integrate) {
        /* 中心死区: 抑制摇杆回中抖动造成的积分漂移 */
        if (height_raw > -HEIGHT_DEADZONE && height_raw < HEIGHT_DEADZONE) {
            height_raw = 0;
        }
        if (HEIGHT_DIRECTION_INVERT) height_raw = -height_raw;
        height_raw = expo_curve(height_raw, STICK_EXPO);

        s_height_integral += height_raw;
        const int32_t lim = 500 * HEIGHT_INTEGRAL_SCALE;
        if (s_height_integral >  lim) s_height_integral =  lim;
        if (s_height_integral < -lim) s_height_integral = -lim;
        height_ctrl = (int16_t)(s_height_integral / HEIGHT_INTEGRAL_SCALE);
    } else {
        height_ctrl = apply_control_deadband(height_raw);
        if (HEIGHT_DIRECTION_INVERT) height_ctrl = -height_ctrl;
        height_ctrl = expo_curve(height_ctrl, STICK_EXPO);
    }
    s_last_height_ctrl = height_ctrl;

    /* ---- ④ 行程 / 姿态 (两协议共用同一段数学) ---- */
    if (ctrl_state->balance_mode) {
        /* 平衡模式: 摇杆直接驱动机身姿态, 原地不动 */
        ctrl_state->body_rot.x = -(strafe  * BODY_ROTATION_MAX) / 500;  /* 横滚 Roll */
        ctrl_state->body_rot.y = -(turn    * BODY_ROTATION_MAX) / 500;  /* 偏航 Yaw */
        ctrl_state->body_rot.z = -(forward * BODY_ROTATION_MAX) / 500;  /* 俯仰 Pitch */
        ctrl_state->body_pos.y = (height_ctrl * BODY_HEIGHT_RANGE_MM) / 500;

        ctrl_state->travel_length.x = 0;
        ctrl_state->travel_length.y = 0;
        ctrl_state->travel_length.z = 0;
    } else {
        /* 正常模式: 摇杆 → 步长 */
        ctrl_state->travel_length.x =  (forward * TRAVEL_MAX_FORWARD_MM) / 500;
        ctrl_state->travel_length.z = -(strafe  * TRAVEL_MAX_STRAFE_MM)  / 500;  /* 右推 → 右平移 */
        ctrl_state->travel_length.y =  (turn    * TRAVEL_MAX_TURN_MM)    / 500;
        ctrl_state->body_pos.y = (height_ctrl * BODY_HEIGHT_RANGE_MM) / 500;

        ctrl_state->body_rot.x = 0;
        ctrl_state->body_rot.y = 0;
        ctrl_state->body_rot.z = 0;

        /* 仿生连续变速: 三轴摇杆的最大幅度 → 步态周期 */
        const int16_t period_max = GAIT_PERIOD_MAX_MS;
        const int32_t range = (int32_t)period_max - (int32_t)GAIT_PERIOD_MIN_MS;

        int16_t stick_mag = (forward >= 0) ? forward : -forward;
        int16_t tmp = (strafe >= 0) ? strafe : -strafe;
        if (tmp > stick_mag) stick_mag = tmp;
        tmp = (turn >= 0) ? turn : -turn;
        if (tmp > stick_mag) stick_mag = tmp;

        if (stick_mag <= CONTROL_DEADBAND) {
            ctrl_state->speed_control = period_max;
        } else {
            ctrl_state->speed_control = period_max
                - (int16_t)(range * (int32_t)stick_mag / 500);
        }
    }

    /* ---- ⑤ 组合键 (两协议; CRSF 上该目标有矩阵行时已由 combo_skip 让位) ---- */
    combos_apply(buttons, ctrl_state, combo_skip);

    /* ---- ⑥ 抬腿高度边界钳位 ---- */
    if (ctrl_state->leg_lift_height > LIFT_HEIGHT_MAX_MM)
        ctrl_state->leg_lift_height = LIFT_HEIGHT_MAX_MM;
    if (ctrl_state->leg_lift_height < LIFT_HEIGHT_MIN_MM)
        ctrl_state->leg_lift_height = LIFT_HEIGHT_MIN_MM;
}

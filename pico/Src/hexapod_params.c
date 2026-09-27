/**
 * @file hexapod_params.c
 * @brief 运行时可调参数: 表 + 读写 + flash 持久化
 *
 * 设计要点见 hexapod_params.h。
 *
 * 本文件是 hexapod_config.h 的"另一半": config.h 把宏重定向到
 * g_params 的字段, 这里给出字段的默认值 (从 config.h 的 *_DEFAULT 取)
 * 和取值范围。两者必须成对维护 —— 加一个参数 = 三处改动:
 *   1. hexapod_params.h   加字段
 *   2. hexapod_config.h   #define X (g_params.x) + #define X_DEFAULT <值>
 *   3. 本文件             s_table[] 加一行
 * 漏了第 3 步的后果是"这个量改不了但也不报错", 所以 params_init() 结束
 * 时会校验字段数与表项数一致。
 */

#include "hexapod_params.h"

#include <string.h>

#include "hexapod_config.h"
#include "hexapod_hal.h"
#include "hexapod_store.h"

/* ==================== 参数实体 ==================== */

hexapod_params_t g_params;

/* ==================== 参数表 ==================== */

/*
 * 默认值一律取自 config.h 的 <宏名>_DEFAULT —— 换板/换机架只改那一处。
 * min/max 按"机械/电气上不会立刻出事"的经验边界给定, 并把常见的
 * 自相矛盾组合挡在外面 (如 lift_max 不得超过腿总长 240mm 的一半)。
 */
static const param_t s_table[] = {
    /* ---- 电池 ---- */
    { "batt_check",     &g_params.batt_check,      0,     1, BATTERY_CHECK_ENABLED_DEFAULT, "",   "batt" },
    { "batt_ov_mv",     &g_params.batt_ov_mv,      8000,  9500, BATTERY_OVERVOLTAGE_MV_DEFAULT, "mV", "batt" },
    { "batt_warn_mv",   &g_params.batt_warn_mv,    6200,  8000, BATTERY_WARNING_MV_DEFAULT,     "mV", "batt" },
    { "batt_cutoff_mv", &g_params.batt_cutoff_mv,  6000,  7200, BATTERY_CUTOFF_MV_DEFAULT,      "mV", "batt" },
    { "batt_recov_mv",  &g_params.batt_recov_mv,   6200,  8200, BATTERY_RECOVERY_MV_DEFAULT,    "mV", "batt" },
    { "batt_ratio_milli", &g_params.batt_ratio_milli, 6500, 10000,
      (int32_t)(BATTERY_DIVIDER_RATIO_DEFAULT * 1000.0f + 0.5f), "x1000", "batt" },
    { "batt_interval_ms", &g_params.batt_interval_ms, 100, 10000,
      BATTERY_CHECK_INTERVAL_MS_DEFAULT, "ms", "batt" },
    { "batt_absent_mv", &g_params.batt_absent_mv, 1000, 5000,
      BATTERY_ABSENT_MV_DEFAULT, "mV", "batt" },

    /* ---- 运动 ---- */
    { "travel_fwd_mm",  &g_params.travel_fwd_mm,   20, 250, TRAVEL_MAX_FORWARD_MM_DEFAULT, "mm", "motion" },
    { "travel_str_mm",  &g_params.travel_str_mm,   20, 250, TRAVEL_MAX_STRAFE_MM_DEFAULT,  "mm", "motion" },
    { "travel_turn_mm", &g_params.travel_turn_mm,  10, 150, TRAVEL_MAX_TURN_MM_DEFAULT,    "mm", "motion" },
    { "body_rot_max",   &g_params.body_rot_max,    50, 600, BODY_ROTATION_MAX_DEFAULT,  "0.1deg", "motion" },
    { "body_h_range_mm", &g_params.body_h_range_mm, 20, 120, BODY_HEIGHT_RANGE_MM_DEFAULT, "mm", "motion" },
    { "lift_min_mm",    &g_params.lift_min_mm,      1,  30, LIFT_HEIGHT_MIN_MM_DEFAULT, "mm", "motion" },
    { "lift_max_mm",    &g_params.lift_max_mm,     20, 100, LIFT_HEIGHT_MAX_MM_DEFAULT, "mm", "motion" },
    { "gait_min_ms",    &g_params.gait_min_ms,     20, 200, GAIT_PERIOD_MIN_MS_DEFAULT, "ms", "motion" },
    { "gait_max_ms",    &g_params.gait_max_ms,     60, 400, GAIT_PERIOD_MAX_MS_DEFAULT, "ms", "motion" },

    /* ---- 站立姿态 ---- */
    { "stance_narrow",  &g_params.stance_narrow,   50, 100, STANCE_SCALE_NARROW_DEFAULT, "%", "stance" },
    { "stance_normal",  &g_params.stance_normal,   50, 150, STANCE_SCALE_NORMAL_DEFAULT, "%", "stance" },
    { "stance_wide",    &g_params.stance_wide,     80, 180, STANCE_SCALE_WIDE_DEFAULT,   "%", "stance" },
    { "stance_speed",   &g_params.stance_speed,     5, 100, STANCE_TRANSITION_SPEED_DEFAULT, "x100/周期", "stance" },
    { "stance_step_mm", &g_params.stance_step_mm,   1,  10, STANCE_MAX_STEP_MM_DEFAULT, "mm", "stance" },

    /* ---- 手感 / 死区 ---- */
    { "deadband",       &g_params.deadband,         0,  50, CONTROL_DEADBAND_DEFAULT,      "",     "tune" },
    { "crsf_deadband",  &g_params.crsf_deadband,    0, 200, CRSF_CH_VALUE_DEADBAND_DEFAULT, "",    "tune" },
    { "ps2_expo",       &g_params.ps2_expo,         0, 100, PS2_STICK_EXPO_DEFAULT,        "%",    "tune" },
    { "ps2_h_deadzone", &g_params.ps2_h_deadzone,   0, 100, PS2_HEIGHT_DEADZONE_DEFAULT,   "",     "tune" },

    /* ---- 方向取反 (0/1) ---- */
    { "inv_fwd",        &g_params.inv_fwd,       0, 1, FORWARD_DIRECTION_INVERT_DEFAULT,    "", "dir" },
    { "inv_str",        &g_params.inv_str,       0, 1, STRAFE_DIRECTION_INVERT_DEFAULT,     "", "dir" },
    { "inv_h",          &g_params.inv_h,         0, 1, HEIGHT_DIRECTION_INVERT_DEFAULT,     "", "dir" },
    { "inv_turn",       &g_params.inv_turn,      0, 1, TURN_DIRECTION_INVERT_DEFAULT,       "", "dir" },
    { "ps2_inv_fwd",    &g_params.ps2_inv_fwd,   0, 1, PS2_FORWARD_DIRECTION_INVERT_DEFAULT, "", "dir" },
    { "ps2_inv_str",    &g_params.ps2_inv_str,   0, 1, PS2_STRAFE_DIRECTION_INVERT_DEFAULT,  "", "dir" },
    { "ps2_inv_h",      &g_params.ps2_inv_h,     0, 1, PS2_HEIGHT_DIRECTION_INVERT_DEFAULT,  "", "dir" },
    { "ps2_inv_turn",   &g_params.ps2_inv_turn,  0, 1, PS2_TURN_DIRECTION_INVERT_DEFAULT,    "", "dir" },

    /* ---- IMU ---- */
    { "imu_enabled",    &g_params.imu_enabled,     0,     1, IMU_ENABLED_DEFAULT,            "",   "imu" },
    { "imu_gain",       &g_params.imu_gain,       0,  30, IMU_COMPENSATION_GAIN_DEFAULT, "x10", "imu" },
    { "imu_roll_sign",  &g_params.imu_roll_sign, -1,   1, IMU_ROLL_SIGN_DEFAULT,        "",    "imu" },
    { "imu_pitch_sign", &g_params.imu_pitch_sign, -1,  1, IMU_PITCH_SIGN_DEFAULT,       "",    "imu" },

    /* ---- 通道映射 (值 = CRSF 通道下标 0~15) ---- */
    { "crsf_ch_fwd",    &g_params.crsf_ch_fwd,      0, 15, CRSF_CHANNEL_FORWARD_DEFAULT, "", "chan" },
    { "crsf_ch_str",    &g_params.crsf_ch_str,      0, 15, CRSF_CHANNEL_STRAFE_DEFAULT,  "", "chan" },
    { "crsf_ch_turn",   &g_params.crsf_ch_turn,     0, 15, CRSF_CHANNEL_TURN_DEFAULT,    "", "chan" },
    { "crsf_ch_hgt",    &g_params.crsf_ch_hgt,      0, 15, CRSF_CHANNEL_HEIGHT_DEFAULT,  "", "chan" },
    { "crsf_ch_arm",    &g_params.crsf_ch_arm,      0, 15, CRSF_CHANNEL_ARM_DEFAULT,     "", "chan" },
    { "crsf_ch_gait",   &g_params.crsf_ch_gait,     0, 15, CRSF_CHANNEL_GAIT_DEFAULT,    "", "chan" },
    { "crsf_ch_stance", &g_params.crsf_ch_stance,   0, 15, CRSF_CHANNEL_SPEED_DEFAULT,   "", "chan" },
    { "crsf_ch_bal",    &g_params.crsf_ch_bal,      0, 15, CRSF_CHANNEL_BALANCE_DEFAULT, "", "chan" },
    { "input_mode",     &g_params.input_mode,       0,  1, INPUT_MODE_DEFAULT,           "", "chan" },

    /* ---- 系统 ---- */
    { "loop_ms",        &g_params.loop_ms,          5,     20, CONTROL_LOOP_PERIOD_MS_DEFAULT, "ms", "sys" },

    /* ---- 预留外设 ---- */
    { "led_heartbeat",  &g_params.led_heartbeat,    0,      1, LED_HEARTBEAT_ENABLED_DEFAULT, "", "per" },
    { "led_alarm",      &g_params.led_alarm,        0,      1, LED_ALARM_ENABLED_DEFAULT,     "", "per" },
    { "uart0_en",       &g_params.uart0_en,         0,      1, UART0_ENABLED_DEFAULT,         "", "per" },
    { "uart0_baud",     &g_params.uart0_baud,    2400, 1000000, UART0_BAUD_DEFAULT,        "baud", "per" },
    { "ext_i2c_mode",   &g_params.ext_i2c_mode,     0,      2, EXT_I2C_MODE_DEFAULT,          "", "per" },
};

const param_t *params_table(void) { return s_table; }

int params_count(void)
{
    return (int)(sizeof(s_table) / sizeof(s_table[0]));
}

/* ==================== 小工具 ==================== */

static const char *skip_space(const char *p)
{
    while (*p == ' ' || *p == '\t') p++;
    return p;
}

/** @brief 解析十进制整数 (支持前导 -), 成功返回 true 并前移指针 */
static bool parse_i32(const char **pp, int32_t *out)
{
    const char *p = skip_space(*pp);
    bool neg = false;
    if (*p == '-') { neg = true; p++; }
    else if (*p == '+') p++;

    if (*p < '0' || *p > '9') return false;

    int32_t v = 0;
    while (*p >= '0' && *p <= '9') {
        v = v * 10 + (*p - '0');
        if (v > 100000000) v = 100000000;   /* 防溢出, 后续会被夹到 max */
        p++;
    }
    *out = neg ? -v : v;
    *pp = p;
    return true;
}

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ==================== 操作 ==================== */

void params_init(void)
{
    int n = params_count();
    for (int i = 0; i < n; i++) {
        *s_table[i].ptr = s_table[i].def;
    }

    /* 表项数必须等于结构体字段数 —— 漏加表项的参数是"静默不可调"的,
     * 这类 bug 在网页上表现为"改了没反应", 很难查, 所以在这里挡住。 */
    int fields = (int)(sizeof(hexapod_params_t) / sizeof(int32_t));
    if (fields != n) {
        hal_debug_printf("[PARAM] WARNING: struct has %d fields but table has %d rows\r\n",
                         fields, n);
    }

    /* 名字放不进 STORE_PARAM_NAME_LEN 的后果是静默的: !CFG 查不到, 存 flash
     * 也被截断成另一个名字。启动时报出来, 别等到"存了却读不回"才发现。 */
    for (int i = 0; i < n; i++) {
        if (strlen(s_table[i].name) >= STORE_PARAM_NAME_LEN) {
            hal_debug_printf("[PARAM] WARNING: name '%s' too long (max %u)\r\n",
                             s_table[i].name, (unsigned)(STORE_PARAM_NAME_LEN - 1));
        }
    }
}

const param_t *params_find(const char *name)
{
    if (!name) return NULL;
    int n = params_count();
    for (int i = 0; i < n; i++) {
        if (strcmp(s_table[i].name, name) == 0) return &s_table[i];
    }
    return NULL;
}

bool params_set(const char *name, int32_t value, int32_t *out_actual)
{
    const param_t *p = params_find(name);
    if (!p) return false;

    int32_t v = clamp_i32(value, p->min, p->max);
    *p->ptr = v;
    if (out_actual) *out_actual = v;
    return true;
}

bool params_reset(const char *name)
{
    const param_t *p = params_find(name);
    if (!p) return false;
    *p->ptr = p->def;
    return true;
}

void params_reset_all(void)
{
    params_init();
}

int params_changed_count(void)
{
    int n = params_count(), c = 0;
    for (int i = 0; i < n; i++) {
        if (*s_table[i].ptr != s_table[i].def) c++;
    }
    return c;
}

/* ==================== 输出 ==================== */

void params_print_one(const param_t *p)
{
    if (!p) return;
    /* 格式契约: server.py 的 RE_PARAM 依赖 key=value 全为无空格 token。
     * 改这里必须同步改 tools/webconfig/server.py + test_webconfig.py。 */
    hal_debug_printf("[P] name=%s val=%d min=%d max=%d def=%d unit=%s grp=%s chg=%d\r\n",
                     p->name, (int)*p->ptr, (int)p->min, (int)p->max, (int)p->def,
                     p->unit, p->group, (*p->ptr != p->def) ? 1 : 0);
}

void params_print_all(void)
{
    int n = params_count();
    hal_debug_printf("[P] === %d params (%d changed) ===\r\n", n, params_changed_count());
    for (int i = 0; i < n; i++) {
        params_print_one(&s_table[i]);
    }
    hal_debug_printf("[P] === end ===\r\n");
}

void params_print_changed(void)
{
    int n = params_count(), c = 0;
    for (int i = 0; i < n; i++) {
        if (*s_table[i].ptr != s_table[i].def) {
            params_print_one(&s_table[i]);
            c++;
        }
    }
    if (c == 0) {
        hal_debug_printf("[P] 全部参数为默认值 (未存过 flash 或已重置)\r\n");
    }
}

/* ==================== 命令处理 (供 hal_pico.c 调用) ==================== */

/**
 * @brief !CFG [name] [value]
 *
 * 无参 → 列出全部; 只有名字 → 查一个; 名字+值 → 设置 (并回显实际生效值,
 * 被 min/max 夹住时前端能立刻看到)。
 */
void params_cmd_config(const char *args)
{
    const char *p = skip_space(args);
    if (*p == '\0') {
        params_print_all();
        return;
    }

    char name[STORE_PARAM_NAME_LEN];
    uint8_t n = 0;
    while (*p && *p != ' ' && *p != '\t' && n < sizeof(name) - 1) {
        name[n++] = *p++;
    }
    name[n] = '\0';

    const param_t *t = params_find(name);
    if (!t) {
        hal_debug_printf("[P] FAIL unknown param '%s' (试 !CFG 列出全部)\r\n", name);
        return;
    }

    p = skip_space(p);
    if (*p == '\0') {          /* 只有名字 → 查询 */
        params_print_one(t);
        return;
    }

    int32_t want = 0;
    if (!parse_i32(&p, &want)) {
        hal_debug_printf("[P] FAIL bad value for '%s' (需要整数)\r\n", name);
        return;
    }

    int32_t got = 0;
    params_set(name, want, &got);
    if (got != want) {
        hal_debug_printf("[P] WARN %s clamped %d -> %d (范围 %d~%d)\r\n",
                         name, (int)want, (int)got, (int)t->min, (int)t->max);
    }
    params_print_one(t);       /* 回显实际生效值 */
}

/** @brief !CFGR [name] — 恢复默认 (无参 = 全部) */
void params_cmd_reset(const char *args)
{
    const char *p = skip_space(args);
    if (*p == '\0') {
        params_reset_all();
        hal_debug_printf("[P] 全部参数已恢复默认 (尚未写 flash, 需 !CFGW 才持久)\r\n");
        params_print_all();
        return;
    }

    char name[STORE_PARAM_NAME_LEN];
    uint8_t n = 0;
    while (*p && *p != ' ' && *p != '\t' && n < sizeof(name) - 1) {
        name[n++] = *p++;
    }
    name[n] = '\0';

    if (!params_reset(name)) {
        hal_debug_printf("[P] FAIL unknown param '%s'\r\n", name);
        return;
    }
    params_print_one(params_find(name));
}

/** @brief !CFGW — 保存到 flash (仅安全上下文) */
bool params_cmd_save(bool safe_context)
{
    if (!safe_context) {
        hal_debug_printf("[STORE] Refuse: robot is armed, disarm first (!S)\r\n");
        return false;
    }
    return params_save(safe_context);
}

/* ==================== 持久化 ==================== */

int params_load(void)
{
    static store_param_entry_t entries[STORE_PARAMS_MAX];
    uint16_t n = store_load_params(entries, STORE_PARAMS_MAX);
    if (n == 0) return 0;

    int applied = 0, unknown = 0;
    for (uint16_t i = 0; i < n; i++) {
        const param_t *t = params_find(entries[i].name);
        if (!t) {
            /* 表里已删掉的参数 (固件降级/改名): 跳过, 不影响其余参数 */
            unknown++;
            continue;
        }
        *t->ptr = clamp_i32(entries[i].value, t->min, t->max);
        applied++;
    }

    hal_debug_printf("[PARAM] Loaded %d params from flash%s\r\n",
                     applied, unknown ? " (部分名字已失效, 已跳过)" : "");
    return applied;
}

bool params_save(bool safe_context)
{
    static store_param_entry_t entries[STORE_PARAMS_MAX];
    int n = params_count();
    if (n > (int)STORE_PARAMS_MAX) n = (int)STORE_PARAMS_MAX;

    /* 只存非默认值 —— 换板改 config.h 默认值后, 旧记录里"等于旧默认"的项
     * 不会把新默认压回去。 */
    uint16_t cnt = 0;
    for (int i = 0; i < n; i++) {
        if (*s_table[i].ptr == s_table[i].def) continue;
        strncpy(entries[cnt].name, s_table[i].name, STORE_PARAM_NAME_LEN - 1);
        entries[cnt].name[STORE_PARAM_NAME_LEN - 1] = '\0';
        entries[cnt].value = *s_table[i].ptr;
        cnt++;
    }

    return store_save_params(entries, cnt, safe_context);
}

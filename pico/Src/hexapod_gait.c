/**
 * @file hexapod_gait.c
 * @brief 六足机器人步态控制实现
 */

#include "hexapod_gait.h"
#include <string.h>

/* 预定义步态表
 *
 * 关于 tl_div_factor 的设计说明：
 *   tl_div_factor 是地面支撑阶段位移量的缩放除数（denominator）。
 *   它与 steps_in_gait - nr_lifted_pos（实际地面步数）可能相差 1，
 *   因为 gait 周期中有一个过渡步（transition step），在此期间腿同时
 *   完成落地/离地动作。差值 1 是有意的设计，用于平滑腿切换时的速度曲线。
 *   例如 RIPPLE_12: 12 步 = 3 抬腿 + 8 地面 + 1 过渡步。
 */
/*
 * 统一单腿运动参数:
 *   所有步态使用相同的 4 步抬腿 + 8 步支撑 = 每腿相同轨迹。
 *   区别仅在于各腿的相位编排 (gait_leg_nr) 和周期长度 (steps_in_gait)。
 *
 *   抬腿: 4 步二次抛物线, 足端平滑起落
 *   支撑: 8 步地面扫掠, 占空比 67%
 */
static const gait_t gait_table[GAIT_MAX] = {
    /* GAIT_RIPPLE_12 - 波纹步态 (12步)
     *
     * 6 腿依次抬腿, 间隔 2 步 → 形成后→前的波浪。
     * 任意时刻约 2 条腿在空中, 4 条腿支撑。
     *
     *   步 1-4: RR 抬腿    步 3-6: RM 抬腿 (与 RR 重叠 2 步)
     *   步 5-8: RF 抬腿    步 7-10: LR 抬腿
     *   步 9-0: LM 抬腿    步 11-2: LF 抬腿         */
    {
        .nom_gait_speed = 100,
        .steps_in_gait = 12,
        .nr_lifted_pos = 4,
        .front_down_pos = 2,
        .lift_div_factor = 2,
        .tl_div_factor = 8,
        .half_lift_height = 3,
        .gait_leg_nr = {1, 3, 5, 7, 9, 11}  // RR, RM, RF, LR, LM, LF
    },

    /* GAIT_TRIPOD_6 - 交替三角步态 (12步)
     *
     * 对角线编组, 3 腿同时抬/落, 交替三角支撑。
     * 三角A: RR + RF + LM    三角B: RM + LR + LF
     *
     * 单腿运动与 RIPPLE 完全相同 (4 抬 + 8 撑),
     * 区别是三角组的腿相位一致 (同起同落)。
     *
     *   步 0-3: 三角A抬起, 三角B支撑
     *   步 4-5: 全部支撑 (过渡)
     *   步 6-9: 三角B抬起, 三角A支撑
     *   步 10-11: 全部支撑 (过渡)                       */
    {
        .nom_gait_speed = 100,
        .steps_in_gait = 12,
        .nr_lifted_pos = 4,
        .front_down_pos = 2,
        .lift_div_factor = 2,
        .tl_div_factor = 8,
        .half_lift_height = 3,
        .gait_leg_nr = {0, 6, 0, 6, 0, 6}  // RR,RF,LM=三角A; RM,LR,LF=三角B
    },

    /* GAIT_TRIPOD_8 - 交替三角步态 (16步, 更从容版)
     *
     * 与上相同的对角线编组, 但周期拉长到 16 步,
     * 两组抬腿之间有更长的全支撑过渡 (4 步)。
     *   步 0-3: 三角A抬起    步 4-7: 全部支撑
     *   步 8-11: 三角B抬起   步 12-15: 全部支撑        */
    {
        .nom_gait_speed = 100,
        .steps_in_gait = 16,
        .nr_lifted_pos = 4,
        .front_down_pos = 2,
        .lift_div_factor = 2,
        .tl_div_factor = 12,
        .half_lift_height = 3,
        .gait_leg_nr = {0, 8, 0, 8, 0, 8}  // RR,RF,LM=三角A; RM,LR,LF=三角B
    },

    /* GAIT_WAVE_24 - 波浪步态 (24步, 每次仅 1 腿抬起)
     *
     * 6 腿严格依次抬腿, 间隔 4 步, 各腿抬腿不重叠。
     * 任意时刻仅 1 腿在空中, 5 腿支撑 → 极稳极慢。
     * 单腿仍为 4 步抬腿, 轨迹与 RIPPLE 完全一致。
     *
     *   步 0-3: RR    步 4-7: RM    步 8-11: RF
     *   步 12-15: LR  步 16-19: LM  步 20-23: LF       */
    {
        .nom_gait_speed = 100,
        .steps_in_gait = 24,
        .nr_lifted_pos = 4,
        .front_down_pos = 2,
        .lift_div_factor = 2,
        .tl_div_factor = 20,
        .half_lift_height = 3,
        .gait_leg_nr = {0, 4, 8, 12, 16, 20}  // 严格依次, 无重叠
    },

    /* GAIT_TRIPOD_4 - 快速交替三角步态 (12步, 快节奏版)
     *
     * 与 GAIT_TRIPOD_6 参数完全相同, 保留用于未来
     * 可能的速度区分 (更高 base speed 或不同占空比)。     */
    {
        .nom_gait_speed = 150,
        .steps_in_gait = 12,
        .nr_lifted_pos = 4,
        .front_down_pos = 2,
        .lift_div_factor = 2,
        .tl_div_factor = 8,
        .half_lift_height = 3,
        .gait_leg_nr = {0, 6, 0, 6, 0, 6}  // RR,RF,LM=三角A; RM,LR,LF=三角B
    }
};

/**
 * @brief 初始化步态系统
 * @note 步态表 gait_table 已静态定义在 ROM 中，无需运行时初始化。
 *       此函数保留用于未来可能的动态步态加载扩展。
 */
void hexapod_gait_init(void)
{
    /* 当前实现为空：所有步态在编译期定义。
     * 调用 hexapod_gait_select() 即可加载指定步态。 */
}

/**
 * @brief 获取预定义步态
 */
const gait_t* hexapod_gait_get(gait_type_t gait_type)
{
    if (gait_type >= GAIT_MAX) {
        return &gait_table[GAIT_RIPPLE_12];  // 默认步态
    }
    return &gait_table[gait_type];
}

/**
 * @brief 选择步态
 */
void hexapod_gait_select(gait_type_t gait_type, control_state_t *ctrl_state)
{
    if (!ctrl_state) return;
    
    const gait_t *new_gait = hexapod_gait_get(gait_type);
    memcpy(&ctrl_state->gait_cur, new_gait, sizeof(gait_t));
    ctrl_state->gait_type = gait_type;
    ctrl_state->gait_step = 0;  // 重置步态步数
}

/**
 * @brief 步态步进
 */
void hexapod_gait_step(control_state_t *ctrl_state)
{
    if (!ctrl_state) return;
    
    ctrl_state->gait_step++;
    if (ctrl_state->gait_step >= ctrl_state->gait_cur.steps_in_gait) {
        ctrl_state->gait_step = 0;
    }
}

/**
 * @brief 获取腿部在步态周期中的位置
 */
bool hexapod_gait_get_leg_position(const control_state_t *ctrl_state,
                                   leg_index_t leg_index,
                                   uint8_t *gait_pos)
{
    if (!ctrl_state || !gait_pos || leg_index >= CNT_LEGS) {
        return false;
    }
    
    const gait_t *gait = &ctrl_state->gait_cur;
    
    /* 计算该腿在步态周期中的位置 */
    int16_t leg_step = ctrl_state->gait_step - gait->gait_leg_nr[leg_index];
    
    /* 归一化到 [0, steps_in_gait) */
    while (leg_step < 0) {
        leg_step += gait->steps_in_gait;
    }
    while (leg_step >= gait->steps_in_gait) {
        leg_step -= gait->steps_in_gait;
    }
    
    *gait_pos = (uint8_t)leg_step;
    
    /* 判断是否在抬腿阶段 */
    return (leg_step < gait->nr_lifted_pos);
}

/**
 * @brief 步态序列计算（子步态插值版）
 *
 * 原理：旧版用整数 gait_step 计算位置，每个步态步（如 60ms）内
 * 舵机刷新 3 次（20ms×3）都输出相同角度，形成台阶式跳动。
 * 新版用 gait_sub_phase（0-99）线性细分每个步态步，使 12 步态
 * 变成 12×100 = 1200 个微位置，消除跳变。
 *
 * 精度：1 gait step = 100 sub-units，所有计算用 int32_t 避免溢出。
 */
bool hexapod_gait_sequence(const control_state_t *ctrl_state,
                          leg_index_t leg_index,
                          coord3d_t *leg_pos,
                          int16_t *lift_height,
                          int16_t gait_sub_phase)
{
    if (!ctrl_state || !leg_pos || !lift_height || leg_index >= CNT_LEGS) {
        return false;
    }

    const gait_t *gait = &ctrl_state->gait_cur;

    /* ---- 计算该腿在当前步态周期中的连续相位 ----
     * leg_phase ∈ [0, steps_in_gait * 100)，1 step = 100 units */
    int32_t total_phase = (int32_t)ctrl_state->gait_step * 100 + (int32_t)gait_sub_phase;
    int32_t leg_offset  = (int32_t)gait->gait_leg_nr[leg_index] * 100;
    int32_t leg_phase   = total_phase - leg_offset;

    int32_t cycle_len = (int32_t)gait->steps_in_gait * 100;
    while (leg_phase < 0)         leg_phase += cycle_len;
    while (leg_phase >= cycle_len) leg_phase -= cycle_len;

    bool is_lifting = (leg_phase < (int32_t)gait->nr_lifted_pos * 100);

    if (is_lifting) {
        /* ---- 抬腿阶段：0→峰值→0 二次曲线 ----
         *
         * 旧抛物线 1-(t/nr)² 在首尾有 44-55% 的残余高度，
         * 导致足端凭空"弹起"再"砸下"。
         * 改用 h ∝ 4·x·(1-x) 二次型，x∈[0,1)：
         *   h(0)=0, h(0.5)=max, h(1)→0
         * 足端从地面平滑升起，到中点达最高，再平滑落回地面。
         *
         * X-Z 扫掠：线性（抬腿 3 步扫过全位移，约为支撑 8 步的 2.7× 速度）。
         * 速度差是少步抬腿的固有特性；方向反转由"支撑向后 / 抬腿向前"决定。 */
        int32_t nr         = (int32_t)gait->nr_lifted_pos;
        int32_t nr_scaled  = nr * 100;                    /* 抬腿阶段总 sub-unit 数 */

        /* 高度: h_pct = 400 * leg_phase * (nr_scaled - leg_phase) / (nr² * 100)
         *        = 4 * leg_phase * (nr_scaled - leg_phase) / (nr² * 100)  ... 简化 */
        int32_t h_pct = (4 * leg_phase * (nr_scaled - leg_phase)) / (nr * nr * 100);
        if (h_pct > 100) h_pct = 100;
        if (h_pct < 0)   h_pct = 0;

        *lift_height = (ctrl_state->leg_lift_height * h_pct) / 100;

        /* X-Z 位移：以抬腿中点为 0，足端从后向前线性扫 */
        int32_t lift_phase = leg_phase - nr_scaled / 2;
        int32_t travel_x = (ctrl_state->travel_length.x * lift_phase) / nr_scaled;
        int32_t travel_z = (ctrl_state->travel_length.z * lift_phase) / nr_scaled;

        leg_pos->x = travel_x;
        leg_pos->z = travel_z;
        leg_pos->y = -(*lift_height);

        return true;
    } else {
        /* ---- 地面支撑阶段：足端从前向后扫 ----
         *
         * 支撑时脚着地，身体向前移动 → 脚相对身体向后滑动。
         * 所以 travel_x 应从正(前)到负(后)，与抬腿方向相反。
         * 取反 ground_centered 使位移方向正确：
         *   start → foot forward (+X), end → foot backward (-X) */
        *lift_height = 0;

        int32_t ground_phase = leg_phase - (int32_t)gait->nr_lifted_pos * 100;
        int32_t half_tl = (int32_t)gait->tl_div_factor * 50;    /* *100/2 */
        int32_t ground_centered = ground_phase - half_tl;

        int32_t div = (int32_t)gait->tl_div_factor * 100;
        /* 取反: 支撑位移与抬腿位移方向相反 */
        int32_t travel_x = -(ctrl_state->travel_length.x * ground_centered) / div;
        int32_t travel_z = -(ctrl_state->travel_length.z * ground_centered) / div;

        leg_pos->x = travel_x;
        leg_pos->z = travel_z;
        leg_pos->y = 0;

        return false;
    }
}

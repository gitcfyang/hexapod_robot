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
static const gait_t gait_table[GAIT_MAX] = {
    /* GAIT_RIPPLE_12 - 波纹步态12步 (仿生优化版)
     *   4 抬腿步 + 8 地面步 = 12
     *   抬腿:支撑速度比 = (120/4):(120/8) = 2:1 (原 2.67:1)
     *   占空比 = 8/12 = 67% (脚 67% 时间着地)
     *   抬腿多 1 步 → 足端空中轨迹更平滑、速度更接近支撑速度 */
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
    
    /* GAIT_TRIPOD_6 - 三脚步态6步
     * 组A(RR,RM,RF)和组B(LR,LM,LF)交替
     * 每组3条腿同时抬起/放下，间隔 steps/2 = 3步 */
    {
        .nom_gait_speed = 150,
        .steps_in_gait = 6,
        .nr_lifted_pos = 2,
        .front_down_pos = 1,
        .lift_div_factor = 2,
        .tl_div_factor = 4,
        .half_lift_height = 3,
        .gait_leg_nr = {0, 0, 0, 3, 3, 3}  // RR,RM,RF同组; LR,LM,LF同组
    },
    
    /* GAIT_TRIPOD_8 - 三脚步态8步 */
    {
        .nom_gait_speed = 150,
        .steps_in_gait = 8,
        .nr_lifted_pos = 3,
        .front_down_pos = 2,
        .lift_div_factor = 2,
        .tl_div_factor = 5,
        .half_lift_height = 3,
        .gait_leg_nr = {0, 0, 0, 4, 4, 4}  // RR,RM,RF同组; LR,LM,LF同组
    },
    
    /* GAIT_WAVE_24 - 波浪步态24步
     *   3 抬腿步 + 20 地面步 + 1 过渡步 = 24 */
    {
        .nom_gait_speed = 80,
        .steps_in_gait = 24,
        .nr_lifted_pos = 3,
        .front_down_pos = 2,
        .lift_div_factor = 2,
        .tl_div_factor = 20,
        .half_lift_height = 3,
        .gait_leg_nr = {1, 5, 9, 13, 17, 21}
    },
    
    /* GAIT_TRIPOD_4 - 快速三脚步态4步 */
    {
        .nom_gait_speed = 200,
        .steps_in_gait = 4,
        .nr_lifted_pos = 1,
        .front_down_pos = 1,
        .lift_div_factor = 2,
        .tl_div_factor = 3,
        .half_lift_height = 1,
        .gait_leg_nr = {0, 0, 0, 2, 2, 2}  // RR,RM,RF同组; LR,LM,LF同组
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

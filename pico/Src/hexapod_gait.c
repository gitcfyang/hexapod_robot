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
    /* GAIT_RIPPLE_12 - 波纹步态12步
     *   3 抬腿步 + 8 地面步 + 1 过渡步 = 12 */
    {
        .nom_gait_speed = 100,
        .steps_in_gait = 12,
        .nr_lifted_pos = 3,
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
 * @brief 步态序列计算
 */
bool hexapod_gait_sequence(const control_state_t *ctrl_state,
                          leg_index_t leg_index,
                          coord3d_t *leg_pos,
                          int16_t *lift_height)
{
    if (!ctrl_state || !leg_pos || !lift_height || leg_index >= CNT_LEGS) {
        return false;
    }
    
    uint8_t gait_pos;
    bool is_lifting = hexapod_gait_get_leg_position(ctrl_state, leg_index, &gait_pos);
    
    const gait_t *gait = &ctrl_state->gait_cur;
    
    if (is_lifting) {
        /* ---- 抬腿阶段 ----
           抛物线轨迹：从0上升到最高点再下降到0
           nr_lifted_pos 表示抬腿占用的步数
           使用对称的抛物线高度计算 */
        
        /* 计算抛物线抬腿高度 */
        /* 映射到对称范围，计算抛物线 y = 1 - t^2/nr^2
           t从 -(nr-1) 到 +(nr-1)，避免首尾为0导致拖地 */
        int16_t nr = (int16_t)gait->nr_lifted_pos;
        int16_t t = (int16_t)gait_pos * 2 - (nr - 1);  // 范围 [-(nr-1), +(nr-1)]
        int16_t t_sq = (t * t * 100) / (nr * nr);
        
        /* 抛物线高度: h = leg_lift_height * (100 - t_sq%) / 100 */
        *lift_height = (ctrl_state->leg_lift_height * (100 - t_sq)) / 100;
        if (*lift_height < 0) *lift_height = 0;
        
        /* 计算X-Z平面的运动（足端在抬起时从后向前移动） */
        int16_t lift_phase = (int16_t)gait_pos - (nr / 2);  // 中心对齐
        
        int32_t travel_x = (ctrl_state->travel_length.x * lift_phase) / (int16_t)gait->nr_lifted_pos;
        int32_t travel_z = (ctrl_state->travel_length.z * lift_phase) / (int16_t)gait->nr_lifted_pos;
        
        leg_pos->x = travel_x;
        leg_pos->z = travel_z;
        /* Y轴约定：Y正=机身向上方向
         * 抬腿时足端向上移动（靠近机身），所以y取负值
         * 支撑阶段足端在地面，y=0 */
        leg_pos->y = -(*lift_height);
        
        return true;
    } 
    else {
        /* 地面支撑阶段 */
        *lift_height = 0;
        
        /* 撑地阶段足端从前向后移动（与抬腿方向相反） */
        int16_t ground_pos = (int16_t)gait_pos - (int16_t)gait->nr_lifted_pos;
        int16_t half_tl = gait->tl_div_factor / 2;
        int16_t ground_phase = ground_pos - half_tl;
        
        int32_t travel_x = (ctrl_state->travel_length.x * ground_phase) / (int16_t)gait->tl_div_factor;
        int32_t travel_z = (ctrl_state->travel_length.z * ground_phase) / (int16_t)gait->tl_div_factor;
        
        leg_pos->x = travel_x;
        leg_pos->z = travel_z;
        leg_pos->y = 0;
        
        return false;
    }
}

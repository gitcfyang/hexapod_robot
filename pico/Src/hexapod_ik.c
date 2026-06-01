/**
 * @file hexapod_ik.c
 * @brief 六足机器人逆运动学解算实现
 */

#include "hexapod_ik.h"
#include "hexapod_config.h"
#include "hexapod_math.h"
#include <string.h>

/**
 * @brief 检查角度限位
 */
bool hexapod_ik_check_angle_limit(int16_t angle, int16_t min_angle, int16_t max_angle)
{
    return (angle >= min_angle) && (angle <= max_angle);
}

/**
 * @brief 单腿三自由度逆运动学解算
 * 使用几何法求解Coxa、Femur、Tibia三个关节角度
 */
bool hexapod_ik_leg(const coord3d_t *target_pos, 
                    const leg_config_t *leg_cfg,
                    ik_solution_t *solution)
{
    if (!target_pos || !leg_cfg || !solution) {
        return false;
    }
    
    /* 初始化结果 */
    memset(solution, 0, sizeof(ik_solution_t));
    
    /* 提取目标坐标 */
    int32_t pos_x = target_pos->x;
    int32_t pos_y = target_pos->y;
    int32_t pos_z = target_pos->z;
    
    /* 步骤1：计算Coxa角度（水平旋转关节）
     * 减去腿部初始安装角度：atan4 得到的是足端相对于腿基座的方向角，
     * coxa_angle 是舵机在机体上的安装偏角（如 RR=-60°, RF=+60°），
     * 实际舵机角度 = 几何方向角 - 安装偏角 */
    int16_t coxa_angle = hexapod_atan4(pos_z, pos_x) - leg_cfg->coxa_angle;
    
    /* 步骤2：计算Coxa在XZ平面的投影长度 */
    int32_t coxa_feet_dist = hexapod_sqrt(pos_x * pos_x + pos_z * pos_z);
    
    /* 减去Coxa长度，得到Femur-Tibia平面的X投影 */
    /* 当 coxa_feet_dist < coxa_length 时，足端在Coxa关节内侧，不可达 */
    int32_t ik_feet_dist = coxa_feet_dist - leg_cfg->coxa_length;
    if (ik_feet_dist < 0) {
        ik_feet_dist = 0;
        solution->solution_warning = true;
    }
    
    /* 步骤3：在Femur-Tibia平面求解 */
    /* 计算足端到Femur关节的距离 */
    uint32_t ik_feet_dist_sq = ik_feet_dist * ik_feet_dist;
    uint32_t ik_a1_sq = pos_y * pos_y;
    uint32_t ik_a_sq = ik_feet_dist_sq + ik_a1_sq;
    uint32_t ik_a = hexapod_sqrt(ik_a_sq);
    
    /* 使用余弦定理计算角度 */
    int32_t femur_len = leg_cfg->femur_length;
    int32_t tibia_len = leg_cfg->tibia_length;
    
    /* 计算Tibia角度 */
    /* cos(tibia) = (femur² + tibia² - a²) / (2 * femur * tibia) */
    int32_t cos_tibia_num = femur_len * femur_len + tibia_len * tibia_len - ik_a_sq;
    int32_t cos_tibia_den = 2 * femur_len * tibia_len;
    
    if (cos_tibia_den == 0) {
        solution->solution_error = true;
        return false;
    }
    
    int32_t cos_tibia = (cos_tibia_num * 10000) / cos_tibia_den;
    
    /* 限幅确保cos值有效 */
    if (cos_tibia > 10000) { cos_tibia = 10000; solution->solution_warning = true; }
    if (cos_tibia < -10000) { cos_tibia = -10000; solution->solution_warning = true; }
    
    int16_t tibia_angle = hexapod_acos((int16_t)cos_tibia);
    tibia_angle = TIBIA_SERVO_ZERO - tibia_angle;  /* 舵机零位 → 几何角度映射 */
    
    /* 计算Femur角度 */
    /* 先计算辅助角度 angle_a1 = atan2(pos_y, ik_feet_dist) */
    int16_t angle_a1;
    if (ik_feet_dist == 0) {
        angle_a1 = (pos_y >= 0) ? 900 : -900;
    } else {
        angle_a1 = hexapod_atan4(pos_y, ik_feet_dist);
    }
    
    /* cos(angle_a2) = (femur² + a² - tibia²) / (2 * femur * a) */
    int32_t cos_a2_num = femur_len * femur_len + ik_a_sq - tibia_len * tibia_len;
    int32_t cos_a2_den = 2 * femur_len * ik_a;
    
    if (cos_a2_den == 0) {
        solution->solution_error = true;
        return false;
    }
    
    int32_t cos_a2 = (cos_a2_num * 10000) / cos_a2_den;
    
    /* 限幅确保cos值有效 */
    if (cos_a2 > 10000) { cos_a2 = 10000; solution->solution_warning = true; }
    if (cos_a2 < -10000) { cos_a2 = -10000; solution->solution_warning = true; }
    
    int16_t angle_a2 = hexapod_acos((int16_t)cos_a2);
    /* 股节几何角 = α_2 - α_1 (股节在足端线上方, 向上朝身体)
     * 若股节向下悬挂则用 α_1 + α_2 */
    int16_t femur_angle = angle_a2 - angle_a1 - FEMUR_SERVO_ZERO;
    
    /* 应用舵机方向和偏移 */
    if (leg_cfg->coxa_invert) {
        coxa_angle = -coxa_angle;
    }
    if (leg_cfg->femur_invert) {
        femur_angle = -femur_angle;
    }
    if (leg_cfg->tibia_invert) {
        tibia_angle = -tibia_angle;
    }

    /* 应用舵盘安装偏移（三关节都支持） */
    coxa_angle  += leg_cfg->coxa_horn_offset;
    femur_angle += leg_cfg->femur_horn_offset;
    tibia_angle += leg_cfg->tibia_horn_offset;
    
    /* 检查角度限位并限幅 */
    bool in_limit = true;
    if (coxa_angle < leg_cfg->coxa_min) {
        coxa_angle = leg_cfg->coxa_min;
        solution->solution_warning = true;
        in_limit = false;
    } else if (coxa_angle > leg_cfg->coxa_max) {
        coxa_angle = leg_cfg->coxa_max;
        solution->solution_warning = true;
        in_limit = false;
    }
    if (femur_angle < leg_cfg->femur_min) {
        femur_angle = leg_cfg->femur_min;
        solution->solution_warning = true;
        in_limit = false;
    } else if (femur_angle > leg_cfg->femur_max) {
        femur_angle = leg_cfg->femur_max;
        solution->solution_warning = true;
        in_limit = false;
    }
    if (tibia_angle < leg_cfg->tibia_min) {
        tibia_angle = leg_cfg->tibia_min;
        solution->solution_warning = true;
        in_limit = false;
    } else if (tibia_angle > leg_cfg->tibia_max) {
        tibia_angle = leg_cfg->tibia_max;
        solution->solution_warning = true;
        in_limit = false;
    }
    
    /* 保存结果（即使越界也输出限幅后的角度） */
    solution->coxa_angle = coxa_angle;
    solution->femur_angle = femur_angle;
    solution->tibia_angle = tibia_angle;
    
    return in_limit;
}

/**
 * @brief 机身逆运动学
 * 计算机身姿态变化对足端位置的影响
 * 
 * 旋转顺序：先绕Y轴(航向Yaw)，再绕X轴(俯仰Pitch)，最后绕Z轴(横滚Roll)
 * 坐标约定：X-前, Y-上（机身抬升方向）, Z-右
 * 
 * 输出：在机身的原始零姿态坐标系中，机身运动后腿部基座的新位置。
 * 该值用于从目标足端位置中减去，得到相对于腿部基座的足端位置。
 */
void hexapod_ik_body(const coord3d_t *body_pos,
                     const coord3d_t *body_rot,
                     leg_index_t leg_index,
                     const leg_config_t *leg_cfg,
                     coord3d_t *leg_pos)
{
    (void)leg_index;  /* 预留：未来支持非对称机身旋转 */

    if (!body_pos || !body_rot || !leg_cfg || !leg_pos) {
        return;
    }

    int32_t offset_x = leg_cfg->offset_x;
    int32_t offset_z = leg_cfg->offset_z;

    /* 快速路径：无机身旋转时跳过三角函数和旋转矩阵 (6 次查表 + 9 次乘法 → 0)
     * 正常模式行走时 body_rot 始终为零，此优化每次 IK 调用省 ~15 条指令。
     * 6 腿 × 50Hz = 每秒省 4500 次 trig 查表。 */
    if (body_rot->x == 0 && body_rot->y == 0 && body_rot->z == 0) {
        leg_pos->x = offset_x + body_pos->x;
        leg_pos->y = -body_pos->y;          /* Y 轴取反见下方注释 */
        leg_pos->z = offset_z + body_pos->z;
        return;
    }

    /* 预计算三角函数值
       body_rot->x = Pitch (俯仰), body_rot->y = Yaw (航向), body_rot->z = Roll (横滚) */
    int16_t sin_pitch = hexapod_sin(body_rot->x);
    int16_t cos_pitch = hexapod_cos(body_rot->x);
    int16_t sin_yaw   = hexapod_sin(body_rot->y);
    int16_t cos_yaw   = hexapod_cos(body_rot->y);
    int16_t sin_roll  = hexapod_sin(body_rot->z);
    int16_t cos_roll  = hexapod_cos(body_rot->z);
    
    /* 腿基座相对于机身中心的原始位置 */
    int32_t px = offset_x;
    int32_t py = 0;  /* 腿部基座在Y方向无偏移 */
    int32_t pz = offset_z;
    
    /* ---- 旋转矩阵: R = Rz(Roll) * Rx(Pitch) * Ry(Yaw) ----
       将原始腿基座位置旋转到当前机身姿态下的世界坐标 */
    
    /* 第一步: 绕Y轴旋转 (Yaw) */
    int32_t x1 = (px * cos_yaw + pz * sin_yaw) / 10000;
    int32_t y1 = py;
    int32_t z1 = (-px * sin_yaw + pz * cos_yaw) / 10000;
    
    /* 第二步: 绕X轴旋转 (Pitch) */
    int32_t x2 = x1;
    int32_t y2 = (y1 * cos_pitch + z1 * sin_pitch) / 10000;
    int32_t z2 = (-y1 * sin_pitch + z1 * cos_pitch) / 10000;
    
    /* 第三步: 绕Z轴旋转 (Roll) */
    int32_t x3 = (x2 * cos_roll + y2 * sin_roll) / 10000;
    int32_t y3 = (-x2 * sin_roll + y2 * cos_roll) / 10000;
    int32_t z3 = z2;
    
    /* 输出结果：旋转后的腿基座世界位置 + 机身平移
     * 
     * 对于X和Z轴：body_ pos平移方向与足端相对位置变化方向相反
     *   相对位置 = 目标足端 - 偏移
     *   例如：机身向前(X+)→腿基座向前→足端需向后(相对位置减)
     *   公式：body_offset.x = x3 + body_pos.x (正确)
     *         relative_pos.x = target_foot.x - body_offset.x ✓
     * 
     * 对于Y轴：特殊处理，因为 init_pos_y 是正值(足端在机身下方)
     *   body_pos.y > 0(机身抬升)→腿基座向上→足端需更向下(相对位置增)
     *   公式：需要 body_offset.y = y3 - body_pos.y
     *         relative_pos.y = target_foot.y - (y3 - body_pos.y) = target_foot.y + body_pos.y ✓
     */
    leg_pos->x = x3 + body_pos->x;
    leg_pos->y = y3 - body_pos->y;
    leg_pos->z = z3 + body_pos->z;
}

/**
 * @brief 完整IK解算（机身+腿部）
 */
bool hexapod_ik_complete(const coord3d_t *target_foot,
                        const coord3d_t *body_pos,
                        const coord3d_t *body_rot,
                        leg_index_t leg_index,
                        const leg_config_t *leg_cfg,
                        ik_solution_t *solution)
{
    if (!target_foot || !body_pos || !body_rot || !leg_cfg || !solution) {
        return false;
    }
    
    /* 计算机身姿态对腿基座位置的影响 */
    coord3d_t body_offset;
    hexapod_ik_body(body_pos, body_rot, leg_index, leg_cfg, &body_offset);
    
    /* 计算相对于机身的足端位置（世界坐标→机身坐标） */
    coord3d_t relative_pos;
    relative_pos.x = target_foot->x - body_offset.x;
    relative_pos.y = target_foot->y - body_offset.y;
    relative_pos.z = target_foot->z - body_offset.z;
    
    /* 腿部IK解算 */
    return hexapod_ik_leg(&relative_pos, leg_cfg, solution);
}

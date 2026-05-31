/**
 * @file hexapod_ik.h
 * @brief 六足机器人逆运动学解算模块
 * @note 负责将目标坐标转换为关节角度
 */

#ifndef HEXAPOD_IK_H
#define HEXAPOD_IK_H

#include "hexapod_types.h"

/**
 * @brief 单腿逆运动学解算
 * @param target_pos 目标位置（相对于机身坐标系）
 * @param leg_cfg 腿部配置参数
 * @param solution 输出：关节角度解算结果
 * @return true表示成功，false表示无解或超出范围
 */
bool hexapod_ik_leg(const coord3d_t *target_pos, 
                    const leg_config_t *leg_cfg,
                    ik_solution_t *solution);

/**
 * @brief 机身逆运动学解算
 * @param body_pos 机身位置
 * @param body_rot 机身旋转角度（X-俯仰, Y-航向, Z-横滚）
 * @param leg_index 腿索引
 * @param leg_cfg 腿部配置
 * @param leg_pos 输出：腿部相对位置
 */
void hexapod_ik_body(const coord3d_t *body_pos,
                     const coord3d_t *body_rot,
                     leg_index_t leg_index,
                     const leg_config_t *leg_cfg,
                     coord3d_t *leg_pos);

/**
 * @brief 完整的机身+腿部IK解算
 * @param target_foot 足端目标位置（世界坐标）
 * @param body_pos 机身位置
 * @param body_rot 机身旋转
 * @param leg_index 腿索引
 * @param leg_cfg 腿部配置
 * @param solution 输出：关节角度解算结果
 * @return true表示成功
 */
bool hexapod_ik_complete(const coord3d_t *target_foot,
                        const coord3d_t *body_pos,
                        const coord3d_t *body_rot,
                        leg_index_t leg_index,
                        const leg_config_t *leg_cfg,
                        ik_solution_t *solution);

/**
 * @brief 检查角度是否在限位范围内
 * @param angle 角度值（0.1度单位）
 * @param min_angle 最小角度
 * @param max_angle 最大角度
 * @return true表示在范围内
 */
bool hexapod_ik_check_angle_limit(int16_t angle, int16_t min_angle, int16_t max_angle);

/**
 * @brief 计算机身姿态对足端位置的补偿值
 *
 * 当机身平移/旋转时，要使足端在世界空间中保持不动，
 * 腿基座坐标系中的足端位置需要反向补偿。
 *
 * 此函数是 hexapod_ik_body() 的别名/等价实现，返回相同的 body_offset。
 *
 * @param body_pos 机身平移量 (mm)
 * @param body_rot 机身旋转角度 (0.1度单位, X-俯仰, Y-航向, Z-横滚)
 * @param leg_index 腿索引
 * @param leg_cfg 腿部配置
 * @param compensation 输出：足端位置补偿值（从目标足端减去此值得到腿基座相对坐标）
 */
static inline void hexapod_ik_body_compensation(const coord3d_t *body_pos,
                                                const coord3d_t *body_rot,
                                                leg_index_t leg_index,
                                                const leg_config_t *leg_cfg,
                                                coord3d_t *compensation)
{
    hexapod_ik_body(body_pos, body_rot, leg_index, leg_cfg, compensation);
}

#endif /* HEXAPOD_IK_H */


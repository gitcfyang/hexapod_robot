/**
 * @file hexapod_gait.h
 * @brief 六足机器人步态控制模块
 * @note 负责生成不同的步态序列
 */

#ifndef HEXAPOD_GAIT_H
#define HEXAPOD_GAIT_H

#include "hexapod_types.h"

/* 步态类型定义 */
typedef enum {
    GAIT_RIPPLE_12 = 0,     // 波纹步态12步
    GAIT_TRIPOD_6,          // 三脚步态6步
    GAIT_TRIPOD_8,          // 三脚步态8步
    GAIT_WAVE_24,           // 波浪步态24步
    GAIT_TRIPOD_4,          // 快速三脚步态4步
    GAIT_MAX
} gait_type_t;

/**
 * @brief 初始化步态系统
 */
void hexapod_gait_init(void);

/**
 * @brief 选择步态
 * @param gait_type 步态类型
 * @param ctrl_state 控制状态
 */
void hexapod_gait_select(gait_type_t gait_type, control_state_t *ctrl_state);

/**
 * @brief 步态序列更新（支持子步态插值）
 * @param ctrl_state 控制状态
 * @param leg_index 腿索引
 * @param leg_pos 输出：腿部位置
 * @param lift_height 输出：当前抬腿高度
 * @param gait_sub_phase 子步态相位 0-99 (0=步态步起点, 99=即将进入下一步)
 *                        传 0 等效于旧版无插值行为
 * @return true表示该腿正在抬起，false表示在地面
 */
bool hexapod_gait_sequence(const control_state_t *ctrl_state,
                          leg_index_t leg_index,
                          coord3d_t *leg_pos,
                          int16_t *lift_height,
                          int16_t gait_sub_phase);

/**
 * @brief 获取预定义步态
 * @param gait_type 步态类型
 * @return 步态定义指针
 */
const gait_t* hexapod_gait_get(gait_type_t gait_type);

/**
 * @brief 步态步进（每个控制周期调用）
 * @param ctrl_state 控制状态
 */
void hexapod_gait_step(control_state_t *ctrl_state);

/**
 * @brief 计算腿部在步态周期中的位置
 * @param ctrl_state 控制状态
 * @param leg_index 腿索引
 * @param gait_pos 输出：步态周期位置 [0, steps_in_gait)
 * @return true表示该腿处于抬起阶段
 */
bool hexapod_gait_get_leg_position(const control_state_t *ctrl_state,
                                   leg_index_t leg_index,
                                   uint8_t *gait_pos);

#endif /* HEXAPOD_GAIT_H */

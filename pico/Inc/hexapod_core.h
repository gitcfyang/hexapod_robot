/**
 * @file hexapod_core.h
 * @brief 六足机器人核心控制模块
 * @note 主要控制循环和状态管理
 */

#ifndef HEXAPOD_CORE_H
#define HEXAPOD_CORE_H

#include "hexapod_types.h"
#include "hexapod_hal.h"
#include "hexapod_ik.h"
#include "hexapod_gait.h"

/* ==================== 配置参数 ==================== */

#define DEFAULT_GAIT_SPEED      100     // 默认步态速度
#define DEFAULT_LEG_LIFT_HEIGHT 40      // 默认抬腿高度（mm）(完整站立高度80mm，抬一半即40mm)
#define DEFAULT_INIT_Y          80      // 默认初始足端Y高度（mm） (站立时机身离地高)
#define MIN_VOLTAGE_MV          10000   // 最低电压（10V）
#define CONTROL_LOOP_PERIOD_MS  10        // 舵机刷新周期（5ms = 200Hz，匹配 PCA9685 200Hz PWM）
#define GAIT_STEP_PERIOD_MS     60       // 步态推进周期（ms）。越大=越慢。RIPPLE_12有12步，
                                         // 60ms×12=720ms/周期。步长80mm时速度=80/0.72≈111mm/s。

/* ==================== 机器人实例 ==================== */

typedef struct {
    control_state_t  state;                     // 控制状态
    leg_config_t     leg_configs[CNT_LEGS];     // 腿部配置
    bool             initialized;               // 初始化标志
    bool             servos_enabled;            // 舵机使能标志
    uint32_t         last_update_time;          // 上次舵机刷新时间
    uint32_t         last_gait_time;            // 上次步态推进时间
} hexapod_t;

/* ==================== 核心功能 ==================== */

/**
 * @brief 初始化六足机器人
 * @param robot 机器人实例
 * @param configs 腿部配置数组（6个腿）
 * @return true表示成功
 */
bool hexapod_init(hexapod_t *robot, const leg_config_t *configs);

/**
 * @brief 主控制循环（应在定时器中调用）
 * @param robot 机器人实例
 */
void hexapod_update(hexapod_t *robot);

/**
 * @brief 使能/禁用舵机
 * @param robot 机器人实例
 * @param enable true表示使能
 */
void hexapod_enable_servos(hexapod_t *robot, bool enable);

/**
 * @brief 设置机身位置
 * @param robot 机器人实例
 * @param pos 位置坐标
 */
void hexapod_set_body_position(hexapod_t *robot, const coord3d_t *pos);

/**
 * @brief 设置机身旋转
 * @param robot 机器人实例
 * @param rot 旋转角度（X-横滚, Y-航向, Z-俯仰）
 */
void hexapod_set_body_rotation(hexapod_t *robot, const coord3d_t *rot);

/**
 * @brief 设置行走参数
 * @param robot 机器人实例
 * @param travel 行走距离和旋转（X-前后, Z-左右, Y-旋转）
 */
void hexapod_set_travel(hexapod_t *robot, const coord3d_t *travel);

/**
 * @brief 设置抬腿高度
 * @param robot 机器人实例
 * @param height 高度（mm）
 */
void hexapod_set_leg_lift_height(hexapod_t *robot, int16_t height);

/**
 * @brief 设置步态类型
 * @param robot 机器人实例
 * @param gait_type 步态类型
 */
void hexapod_set_gait(hexapod_t *robot, gait_type_t gait_type);

/**
 * @brief 设置平衡模式
 * @param robot 机器人实例
 * @param enable true表示启用平衡
 */
void hexapod_set_balance_mode(hexapod_t *robot, bool enable);

/**
 * @brief 紧急停止
 * @param robot 机器人实例
 */
void hexapod_emergency_stop(hexapod_t *robot);

/**
 * @brief 获取当前状态
 * @param robot 机器人实例
 * @return 控制状态指针
 */
const control_state_t* hexapod_get_state(const hexapod_t *robot);

/**
 * @brief 获取最近一次 IK 解算结果（调试用）
 * @param robot 机器人实例
 * @param leg_index 腿索引 (0-5)
 * @return IK 角度，robot 为空时返回 NULL
 */
const ik_solution_t* hexapod_get_last_ik(const hexapod_t *robot, leg_index_t leg_index);

/**
 * @brief 执行单步（调试用）
 * @param robot 机器人实例
 */
void hexapod_single_step(hexapod_t *robot);

/* ==================== 辅助功能 ==================== */

/**
 * @brief 获取腿部XZ平面长度
 * @param robot 机器人实例
 * @return XZ平面长度
 */
uint16_t hexapod_get_legs_xz_length(const hexapod_t *robot);

/**
 * @brief 调整腿部位置到指定长度
 * @param robot 机器人实例
 * @param xz_length 目标XZ长度
 */
void hexapod_adjust_leg_positions(hexapod_t *robot, uint16_t xz_length);

/**
 * @brief 调整腿部位置到机身高度
 * @param robot 机器人实例
 */
void hexapod_adjust_to_body_height(hexapod_t *robot);

/**
 * @brief 重置腿部初始角度
 * @param robot 机器人实例
 */
void hexapod_reset_leg_init_angles(hexapod_t *robot);

/**
 * @brief 旋转腿部初始角度
 * @param robot 机器人实例
 * @param delta_angle 旋转角度增量（0.1度）
 */
void hexapod_rotate_leg_init_angles(hexapod_t *robot, int16_t delta_angle);

#endif /* HEXAPOD_CORE_H */


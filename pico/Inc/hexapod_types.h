/**
 * @file hexapod_types.h
 * @brief 六足机器人核心数据类型定义
 * @note 移植自Phoenix项目，适配STM32平台
 */

#ifndef HEXAPOD_TYPES_H
#define HEXAPOD_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* 基础常量定义 */
#define CNT_LEGS            6       // 腿的数量
#define BALANCE_DIV_FACTOR  CNT_LEGS

/*
 * 坐标系统约定（重要 — 所有模块统一）：
 *   原点：机身几何中心在地面的投影
 *   X轴：正方向为前进方向（机头方向）
 *   Y轴：正方向为向上（机身抬升方向）
 *   Z轴：正方向为右（从机尾看向机头方向）
 *
 *   ⚠️ 注意区分：
 *     - body_pos.y > 0 表示机身抬升（腿需向下伸展）
 *     - 足端位置中 y > 0 表示足端在机身下方更远处
 *     - init_pos_y = 25 表示足端初始在机身下方25mm处
 *     - 抬腿时足端y坐标减小（靠近机身），所以 leg_pos.y = -lift_height
 */

/* 腿部索引枚举 */
typedef enum {
    LEG_RR = 0,  // 右后腿 Right Rear
    LEG_RM = 1,  // 右中腿 Right Middle
    LEG_RF = 2,  // 右前腿 Right Front
    LEG_LR = 3,  // 左后腿 Left Rear
    LEG_LM = 4,  // 左中腿 Left Middle
    LEG_LF = 5   // 左前腿 Left Front
} leg_index_t;

/* 三维坐标结构体 */
typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} coord3d_t;

/* 步态定义结构体 */
typedef struct {
    int16_t  nom_gait_speed;        // 标称速度
    uint8_t  steps_in_gait;         // 步态中的步数
    uint8_t  nr_lifted_pos;         // 抬腿位置数 [1-3]
    uint8_t  front_down_pos;        // 前腿落地位置
    uint8_t  lift_div_factor;       // 抬腿除数因子（通常为2，当nr_lifted_pos=5时为4）
    uint8_t  tl_div_factor;         // 腿在地面上的步数
    uint8_t  half_lift_height;      // 半程抬起高度
    uint8_t  gait_leg_nr[CNT_LEGS]; // 每条腿的初始位置
} gait_t;

/* 控制状态结构体 */
typedef struct {
    bool        robot_on;           // 机器人开关状态
    bool        prev_robot_on;      // 上一循环状态
    
    /* 机身位置 */
    coord3d_t   body_pos;           // 机身位置
    coord3d_t   body_rot_offset;    // 机身旋转偏移
    coord3d_t   body_rot;           // 机身旋转 (X-俯仰, Y-旋转, Z-滚转)
    
    /* 步态控制 */
    uint8_t     gait_type;          // 步态类型
    uint8_t     gait_step;          // 当前步态步数
    gait_t      gait_cur;           // 当前步态定义
    
    int16_t     leg_lift_height;    // 当前抬腿高度
    coord3d_t   travel_length;      // X-Z或长度，Y为旋转
    
    /* 平衡模式 */
    bool        balance_mode;
    
    /* 时序控制 */
    uint8_t     input_time_delay;   // 输入延迟（"潜行"效果）
    uint16_t    speed_control;      // 可调延迟
    uint8_t     force_gait_step_cnt;// 强制步进计数
    
    /* 腿部初始角度（动态调整选项） */
    int16_t     coxa_init_angle[CNT_LEGS];
} control_state_t;

/* 机器人配置结构体 */
typedef struct {
    /* 腿部尺寸 (mm) */
    uint16_t coxa_length;
    uint16_t femur_length;
    uint16_t tibia_length;
    
    /* 机身偏移 */
    int16_t offset_x;
    int16_t offset_z;
    
    /* 初始角度 */
    int16_t coxa_angle;
    
    /* 初始位置 */
    int16_t init_pos_x;
    int16_t init_pos_y;
    int16_t init_pos_z;
    
    /* 舵机限位 (0.1度单位) */
    int16_t coxa_min;
    int16_t coxa_max;
    int16_t femur_min;
    int16_t femur_max;
    int16_t tibia_min;
    int16_t tibia_max;
    
    /* 舵机方向反转标志 */
    bool coxa_invert;
    bool femur_invert;
    bool tibia_invert;
    
    /* 舵机偏移校准 */
    int16_t femur_horn_offset;
    int16_t tibia_horn_offset;
} leg_config_t;

/* 逆运动学解算结果 */
typedef struct {
    int16_t coxa_angle;     // 0.1度单位
    int16_t femur_angle;
    int16_t tibia_angle;
    bool    solution_error; // 求解错误标志
    bool    solution_warning; // 求解警告标志
} ik_solution_t;

#endif /* HEXAPOD_TYPES_H */

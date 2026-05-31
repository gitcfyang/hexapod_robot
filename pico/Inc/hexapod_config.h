/**
 * @file hexapod_config.h
 * @brief 六足机器人配置文件
 * @note 用户应根据实际机器人参数修改此文件
 */

#ifndef HEXAPOD_CONFIG_H
#define HEXAPOD_CONFIG_H

#include "hexapod_types.h"

/* ==================== 机械参数配置 ==================== */

/* 通用腿部尺寸 (mm) - TH3-R腿部 */
#define LEG_COXA_LENGTH     29
#define LEG_FEMUR_LENGTH    76
#define LEG_TIBIA_LENGTH    106

/* 机身尺寸
 * 坐标系：X轴前后（正=前），Y轴上下（正=上），Z轴左右（正=右）
 * 右腿offset_z为正数，左腿offset_z为负数 */
#define BODY_OFFSET_RR_X    -43
#define BODY_OFFSET_RR_Z    82
#define BODY_OFFSET_RM_X    -63
#define BODY_OFFSET_RM_Z    0
#define BODY_OFFSET_RF_X    -43
#define BODY_OFFSET_RF_Z    -82

#define BODY_OFFSET_LR_X    -43
#define BODY_OFFSET_LR_Z    -82
#define BODY_OFFSET_LM_X    -63
#define BODY_OFFSET_LM_Z    0
#define BODY_OFFSET_LF_X    -43
#define BODY_OFFSET_LF_Z    82

/* 初始Coxa角度 (0.1度单位) */
#define COXA_ANGLE_RR       -600
#define COXA_ANGLE_RM       0
#define COXA_ANGLE_RF       600
#define COXA_ANGLE_LR       -600
#define COXA_ANGLE_LM       0
#define COXA_ANGLE_LF       600

/* 初始足端位置 */
#define INIT_XZ             105
#define INIT_XZ_COS60       53   // cos(60°) * 105
#define INIT_XZ_SIN60       91   // sin(60°) * 105
#define INIT_Y              25

/* ==================== 舵机参数配置 ==================== */

/* 舵机角度限位 (0.1度单位) */
#define SERVO_COXA_MIN_RR   -260
#define SERVO_COXA_MAX_RR   740
#define SERVO_FEMUR_MIN_RR  -1010
#define SERVO_FEMUR_MAX_RR  950
#define SERVO_TIBIA_MIN_RR  -1060
#define SERVO_TIBIA_MAX_RR  770

/* 对于其他腿部，可以使用相同的限位或单独定义 */
#define SERVO_COXA_MIN_RM   -530
#define SERVO_COXA_MAX_RM   530
#define SERVO_FEMUR_MIN_RM  -1010
#define SERVO_FEMUR_MAX_RM  950
#define SERVO_TIBIA_MIN_RM  -1060
#define SERVO_TIBIA_MAX_RM  770

/* 右前腿 (RF) - 与RR镜像对称 */
#define SERVO_COXA_MIN_RF   -580
#define SERVO_COXA_MAX_RF   740
#define SERVO_FEMUR_MIN_RF  -1010
#define SERVO_FEMUR_MAX_RF  950
#define SERVO_TIBIA_MIN_RF  -1060
#define SERVO_TIBIA_MAX_RF  770

/* 左后腿 (LR) */
#define SERVO_COXA_MIN_LR   -740
#define SERVO_COXA_MAX_LR   260
#define SERVO_FEMUR_MIN_LR  -950
#define SERVO_FEMUR_MAX_LR  1010
#define SERVO_TIBIA_MIN_LR  -770
#define SERVO_TIBIA_MAX_LR  1060

/* 左中腿 (LM) */
#define SERVO_COXA_MIN_LM   -530
#define SERVO_COXA_MAX_LM   530
#define SERVO_FEMUR_MIN_LM  -950
#define SERVO_FEMUR_MAX_LM  1010
#define SERVO_TIBIA_MIN_LM  -770
#define SERVO_TIBIA_MAX_LM  1060

/* 左前腿 (LF) */
#define SERVO_COXA_MIN_LF   -740
#define SERVO_COXA_MAX_LF   580
#define SERVO_FEMUR_MIN_LF  -950
#define SERVO_FEMUR_MAX_LF  1010
#define SERVO_TIBIA_MIN_LF  -770
#define SERVO_TIBIA_MAX_LF  1060

/* ==================== 舵机ID映射 ==================== */

/* 舵机控制器引脚映射 */
#define SERVO_RR_COXA       0
#define SERVO_RR_FEMUR      1
#define SERVO_RR_TIBIA      2

#define SERVO_RM_COXA       3
#define SERVO_RM_FEMUR      4
#define SERVO_RM_TIBIA      5

#define SERVO_RF_COXA       6
#define SERVO_RF_FEMUR      7
#define SERVO_RF_TIBIA      8

#define SERVO_LR_COXA       9
#define SERVO_LR_FEMUR      10
#define SERVO_LR_TIBIA      11

#define SERVO_LM_COXA       12
#define SERVO_LM_FEMUR      13
#define SERVO_LM_TIBIA      14

#define SERVO_LF_COXA       15
#define SERVO_LF_FEMUR      16
#define SERVO_LF_TIBIA      17

/* ==================== 配置数据结构 ==================== */

/**
 * @brief 获取默认腿部配置
 * @param configs 输出配置数组（至少6个元素）
 */
static inline void hexapod_get_default_config(leg_config_t *configs)
{
    /* 右后腿 (RR) */
    configs[LEG_RR].coxa_length = LEG_COXA_LENGTH;
    configs[LEG_RR].femur_length = LEG_FEMUR_LENGTH;
    configs[LEG_RR].tibia_length = LEG_TIBIA_LENGTH;
    configs[LEG_RR].offset_x = BODY_OFFSET_RR_X;
    configs[LEG_RR].offset_z = BODY_OFFSET_RR_Z;
    configs[LEG_RR].coxa_angle = COXA_ANGLE_RR;
    configs[LEG_RR].init_pos_x = INIT_XZ_COS60;
    configs[LEG_RR].init_pos_y = INIT_Y;
    configs[LEG_RR].init_pos_z = INIT_XZ_SIN60;
    configs[LEG_RR].coxa_min = SERVO_COXA_MIN_RR;
    configs[LEG_RR].coxa_max = SERVO_COXA_MAX_RR;
    configs[LEG_RR].femur_min = SERVO_FEMUR_MIN_RR;
    configs[LEG_RR].femur_max = SERVO_FEMUR_MAX_RR;
    configs[LEG_RR].tibia_min = SERVO_TIBIA_MIN_RR;
    configs[LEG_RR].tibia_max = SERVO_TIBIA_MAX_RR;
    configs[LEG_RR].coxa_invert = true;
    configs[LEG_RR].femur_invert = true;
    configs[LEG_RR].tibia_invert = true;
    configs[LEG_RR].femur_horn_offset = 0;
    configs[LEG_RR].tibia_horn_offset = 0;
    
    /* 右中腿 (RM) */
    configs[LEG_RM].coxa_length = LEG_COXA_LENGTH;
    configs[LEG_RM].femur_length = LEG_FEMUR_LENGTH;
    configs[LEG_RM].tibia_length = LEG_TIBIA_LENGTH;
    configs[LEG_RM].offset_x = BODY_OFFSET_RM_X;
    configs[LEG_RM].offset_z = BODY_OFFSET_RM_Z;
    configs[LEG_RM].coxa_angle = COXA_ANGLE_RM;
    configs[LEG_RM].init_pos_x = INIT_XZ;
    configs[LEG_RM].init_pos_y = INIT_Y;
    configs[LEG_RM].init_pos_z = 0;
    configs[LEG_RM].coxa_min = SERVO_COXA_MIN_RM;
    configs[LEG_RM].coxa_max = SERVO_COXA_MAX_RM;
    configs[LEG_RM].femur_min = SERVO_FEMUR_MIN_RM;
    configs[LEG_RM].femur_max = SERVO_FEMUR_MAX_RM;
    configs[LEG_RM].tibia_min = SERVO_TIBIA_MIN_RM;
    configs[LEG_RM].tibia_max = SERVO_TIBIA_MAX_RM;
    configs[LEG_RM].coxa_invert = true;
    configs[LEG_RM].femur_invert = true;
    configs[LEG_RM].tibia_invert = true;
    configs[LEG_RM].femur_horn_offset = 0;
    configs[LEG_RM].tibia_horn_offset = 0;
    
    /* 右前腿 (RF) */
    configs[LEG_RF].coxa_length = LEG_COXA_LENGTH;
    configs[LEG_RF].femur_length = LEG_FEMUR_LENGTH;
    configs[LEG_RF].tibia_length = LEG_TIBIA_LENGTH;
    configs[LEG_RF].offset_x = BODY_OFFSET_RF_X;
    configs[LEG_RF].offset_z = BODY_OFFSET_RF_Z;
    configs[LEG_RF].coxa_angle = COXA_ANGLE_RF;
    configs[LEG_RF].init_pos_x = INIT_XZ_COS60;
    configs[LEG_RF].init_pos_y = INIT_Y;
    configs[LEG_RF].init_pos_z = -INIT_XZ_SIN60;
    configs[LEG_RF].coxa_min = SERVO_COXA_MIN_RF;
    configs[LEG_RF].coxa_max = SERVO_COXA_MAX_RF;
    configs[LEG_RF].femur_min = SERVO_FEMUR_MIN_RF;
    configs[LEG_RF].femur_max = SERVO_FEMUR_MAX_RF;
    configs[LEG_RF].tibia_min = SERVO_TIBIA_MIN_RF;
    configs[LEG_RF].tibia_max = SERVO_TIBIA_MAX_RF;
    configs[LEG_RF].coxa_invert = true;
    configs[LEG_RF].femur_invert = true;
    configs[LEG_RF].tibia_invert = true;
    configs[LEG_RF].femur_horn_offset = 0;
    configs[LEG_RF].tibia_horn_offset = 0;
    
    /* 左后腿 (LR) - 与RR镜像对称 */
    configs[LEG_LR].coxa_length = LEG_COXA_LENGTH;
    configs[LEG_LR].femur_length = LEG_FEMUR_LENGTH;
    configs[LEG_LR].tibia_length = LEG_TIBIA_LENGTH;
    configs[LEG_LR].offset_x = BODY_OFFSET_LR_X;
    configs[LEG_LR].offset_z = BODY_OFFSET_LR_Z;
    configs[LEG_LR].coxa_angle = COXA_ANGLE_LR;
    configs[LEG_LR].init_pos_x = INIT_XZ_COS60;
    configs[LEG_LR].init_pos_y = INIT_Y;
    configs[LEG_LR].init_pos_z = -INIT_XZ_SIN60;  /* 左腿Z镜像（左负） */
    configs[LEG_LR].coxa_min = SERVO_COXA_MIN_LR;
    configs[LEG_LR].coxa_max = SERVO_COXA_MAX_LR;
    configs[LEG_LR].femur_min = SERVO_FEMUR_MIN_LR;
    configs[LEG_LR].femur_max = SERVO_FEMUR_MAX_LR;
    configs[LEG_LR].tibia_min = SERVO_TIBIA_MIN_LR;
    configs[LEG_LR].tibia_max = SERVO_TIBIA_MAX_LR;
    configs[LEG_LR].coxa_invert = false;
    configs[LEG_LR].femur_invert = false;
    configs[LEG_LR].tibia_invert = false;
    configs[LEG_LR].femur_horn_offset = 0;
    configs[LEG_LR].tibia_horn_offset = 0;
    
    /* 左中腿 (LM) */
    configs[LEG_LM].coxa_length = LEG_COXA_LENGTH;
    configs[LEG_LM].femur_length = LEG_FEMUR_LENGTH;
    configs[LEG_LM].tibia_length = LEG_TIBIA_LENGTH;
    configs[LEG_LM].offset_x = BODY_OFFSET_LM_X;
    configs[LEG_LM].offset_z = BODY_OFFSET_LM_Z;
    configs[LEG_LM].coxa_angle = COXA_ANGLE_LM;
    configs[LEG_LM].init_pos_x = INIT_XZ;
    configs[LEG_LM].init_pos_y = INIT_Y;
    configs[LEG_LM].init_pos_z = 0;
    configs[LEG_LM].coxa_min = SERVO_COXA_MIN_LM;
    configs[LEG_LM].coxa_max = SERVO_COXA_MAX_LM;
    configs[LEG_LM].femur_min = SERVO_FEMUR_MIN_LM;
    configs[LEG_LM].femur_max = SERVO_FEMUR_MAX_LM;
    configs[LEG_LM].tibia_min = SERVO_TIBIA_MIN_LM;
    configs[LEG_LM].tibia_max = SERVO_TIBIA_MAX_LM;
    configs[LEG_LM].coxa_invert = false;
    configs[LEG_LM].femur_invert = false;
    configs[LEG_LM].tibia_invert = false;
    configs[LEG_LM].femur_horn_offset = 0;
    configs[LEG_LM].tibia_horn_offset = 0;
    
    /* 左前腿 (LF) */
    configs[LEG_LF].coxa_length = LEG_COXA_LENGTH;
    configs[LEG_LF].femur_length = LEG_FEMUR_LENGTH;
    configs[LEG_LF].tibia_length = LEG_TIBIA_LENGTH;
    configs[LEG_LF].offset_x = BODY_OFFSET_LF_X;
    configs[LEG_LF].offset_z = BODY_OFFSET_LF_Z;
    configs[LEG_LF].coxa_angle = COXA_ANGLE_LF;
    configs[LEG_LF].init_pos_x = INIT_XZ_COS60;
    configs[LEG_LF].init_pos_y = INIT_Y;
    configs[LEG_LF].init_pos_z = -INIT_XZ_SIN60;
    configs[LEG_LF].coxa_min = SERVO_COXA_MIN_LF;
    configs[LEG_LF].coxa_max = SERVO_COXA_MAX_LF;
    configs[LEG_LF].femur_min = SERVO_FEMUR_MIN_LF;
    configs[LEG_LF].femur_max = SERVO_FEMUR_MAX_LF;
    configs[LEG_LF].tibia_min = SERVO_TIBIA_MIN_LF;
    configs[LEG_LF].tibia_max = SERVO_TIBIA_MAX_LF;
    configs[LEG_LF].coxa_invert = false;
    configs[LEG_LF].femur_invert = false;
    configs[LEG_LF].tibia_invert = false;
    configs[LEG_LF].femur_horn_offset = 0;
    configs[LEG_LF].tibia_horn_offset = 0;
}

#endif /* HEXAPOD_CONFIG_H */



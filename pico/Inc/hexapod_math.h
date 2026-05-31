/**
 * @file hexapod_math.h
 * @brief 六足机器人数学运算库
 * @note 包含三角函数查找表和基础数学运算
 */

#ifndef HEXAPOD_MATH_H
#define HEXAPOD_MATH_H

#include <stdint.h>

/* 角度单位转换 (0.1度 <-> 弧度) */
#define ANGLE_TO_RAD(angle)     ((float)(angle) * 0.001745329f)  // angle/10 * PI/180
#define RAD_TO_ANGLE(rad)       ((int16_t)((rad) * 572.9578f))   // rad * 1800/PI

/* 三角函数查找表精度定义 */
#define SIN_TABLE_SIZE          181  // 0-180度，步进0.5度
#define ACOS_TABLE_SIZE_1       127  // cos 0-0.9
#define ACOS_TABLE_SIZE_2       127  // cos 0.9-0.99
#define ACOS_TABLE_SIZE_3       64   // cos 0.99-1

/**
 * @brief 获取正弦值（使用查找表）
 * @param angle 角度（0.1度单位，0-1800对应0-180度）
 * @return 正弦值 * 10000
 */
int16_t hexapod_sin(int16_t angle);

/**
 * @brief 获取余弦值（使用查找表）
 * @param angle 角度（0.1度单位，0-1800对应0-180度）
 * @return 余弦值 * 10000
 */
int16_t hexapod_cos(int16_t angle);

/**
 * @brief 获取反余弦值（使用查找表）
 * @param cos_val 余弦值 * 10000
 * @return 角度（0.1度单位）
 */
int16_t hexapod_acos(int16_t cos_val);

/**
 * @brief 平方根运算（整数优化）
 * @param value 输入值
 * @return 平方根值
 */
uint32_t hexapod_sqrt(uint32_t value);

/**
 * @brief Atan4运算（四象限反正切）
 * @param y Y坐标
 * @param x X坐标
 * @return 角度（0.1度单位，-1800到1800）
 */
int16_t hexapod_atan4(int32_t y, int32_t x);

/**
 * @brief 平滑控制函数
 * @param ctrl_input 输入控制值
 * @param ctrl_output 输出控制值
 * @param divider 除数因子
 * @return 平滑后的控制值
 */
int16_t smooth_control(int16_t ctrl_input, int16_t ctrl_output, uint8_t divider);

#endif /* HEXAPOD_MATH_H */

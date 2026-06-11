/**
 * @file hexapod_hal.h
 * @brief 六足机器人硬件抽象层
 * @note 用户需要根据具体硬件实现这些接口
 */

#ifndef HEXAPOD_HAL_H
#define HEXAPOD_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "hexapod_types.h"

/* ==================== 舵机控制接口 ==================== */

/**
 * @brief 初始化舵机控制器
 * @return true表示成功
 */
bool hal_servo_init(void);

/**
 * @brief 设置单个舵机角度
 * @param servo_id 舵机ID (0-17，6腿x3关节)
 * @param angle 角度（0.1度单位）
 * @param move_time 运动时间（毫秒）
 * @return true表示成功
 */
bool hal_servo_set_angle(uint8_t servo_id, int16_t angle, uint16_t move_time);

/**
 * @brief 批量设置舵机角度（更高效）
 * @param servo_ids 舵机ID数组
 * @param angles 角度数组（0.1度单位）
 * @param count 舵机数量
 * @param move_time 运动时间（毫秒）
 * @return true表示成功
 */
bool hal_servo_set_angles(const uint8_t *servo_ids, 
                         const int16_t *angles,
                         uint8_t count,
                         uint16_t move_time);

/**
 * @brief 释放所有舵机（去扭矩）
 */
void hal_servo_free_all(void);

/**
 * @brief 检查舵机是否运动完成
 * @return true表示所有舵机运动完成
 */
bool hal_servo_is_movement_done(void);

/**
 * @brief 刷新所有缓存的舵机角度到硬件
 *        调用完 set_angle/set_angles 后调用此函数批量输出
 */
void hal_servo_flush(void);

/* ==================== 定时器接口 ==================== */

/**
 * @brief 获取系统滴答计数（毫秒）
 * @return 毫秒计数
 */
uint32_t hal_get_tick_ms(void);

/**
 * @brief 延时函数（毫秒）
 * @param ms 延时时间
 */
void hal_delay_ms(uint32_t ms);

/* ==================== 电源管理接口 ==================== */

/**
 * @brief 获取电池电压
 * @return 电压值（毫伏）
 */
uint16_t hal_get_battery_voltage(void);

/**
 * @brief 检查电压是否正常
 * @return true表示电压正常
 */
bool hal_check_battery(void);

/* ==================== 输入设备接口 ==================== */

/**
 * @brief 输入控制器类型
 */
typedef enum {
    INPUT_TYPE_PS2,      // PS2手柄
    INPUT_TYPE_SERIAL,   // 串口命令
    INPUT_TYPE_BLUETOOTH,// 蓝牙
    INPUT_TYPE_WIFI,     // WiFi
    INPUT_TYPE_CRSF      // CRSF接收器 (ELRS)
} input_type_t;

/**
 * @brief 输入控制器初始化
 * @param type 输入类型
 * @return true表示成功
 */
bool hal_input_init(input_type_t type);

/**
 * @brief 读取输入数据并更新控制状态
 * @param ctrl_state 控制状态
 * @return true表示有新数据
 */
bool hal_input_update(control_state_t *ctrl_state);

/**
 * @brief 允许/禁止输入中断
 * @param allow true表示允许
 */
void hal_input_allow_interrupts(bool allow);

/* ==================== 调试输出接口 ==================== */

/**
 * @brief 调试输出初始化
 */
void hal_debug_init(void);

/**
 * @brief 调试打印
 * @param format 格式化字符串
 * @param ... 可变参数
 */
void hal_debug_printf(const char *format, ...);

/* ==================== 蜂鸣器/声音接口 ==================== */

/**
 * @brief 播放音符
 * @param note_count 音符数量
 * @param notes 音符频率数组
 * @param durations 持续时间数组（毫秒）
 */
void hal_play_sound(uint8_t note_count, 
                   const uint16_t *notes,
                   const uint16_t *durations);

/* ==================== LED指示接口 ==================== */

/**
 * @brief 设置LED状态
 * @param led_id LED ID
 * @param state true为亮，false为灭
 */
void hal_led_set(uint8_t led_id, bool state);

/**
 * @brief LED闪烁
 * @param led_id LED ID
 * @param times 闪烁次数
 */
void hal_led_blink(uint8_t led_id, uint8_t times);

/* ==================== IMU 姿态传感器接口 ==================== */

/**
 * @brief 初始化 IMU 姿态传感器 (BNO055)
 * @return true 表示传感器就绪，false 表示未检测到 (姿态补偿不可用)
 */
bool hal_imu_init(void);

/**
 * @brief 读取 IMU 姿态数据
 * @param data 输出姿态数据 (0.1° 单位, valid 标志指示数据是否可信)
 * @return true 表示读取成功
 */
bool hal_imu_read(imu_data_t *data);

/* ==================== 舵机ID映射（用户需要定义） ==================== */

/**
 * @brief 获取舵机ID
 * @param leg_index 腿索引
 * @param joint 关节 (0=Coxa, 1=Femur, 2=Tibia)
 * @return 舵机ID
 */
uint8_t hal_get_servo_id(leg_index_t leg_index, uint8_t joint);

/* ==================== 调试辅助接口 ==================== */

/**
 * @brief 获取 CRSF 内部状态指针（只读，用于调试输出）
 * @return CRSF 状态指针，非 CRSF 模式下返回 NULL
 */
const void* hal_debug_get_crsf_state(void);

/**
 * @brief 获取 CRSF 帧计数增量（用于检测是否有新数据到达）
 * @return 自上次调用后的帧计数增量，非 CRSF 模式返回 0
 */
uint32_t hal_debug_get_crsf_frame_delta(void);

/**
 * @brief 获取最后一次 servo flush 的舵机数量
 * @return 舵机数量 (0-18)
 */
uint8_t hal_debug_get_last_servo_count(void);

/**
 * @brief 设置运行时调试等级
 * @param level 调试等级 (0=关, 1=CRSF, 2=+控制, 3=+舵机)
 */
void hal_debug_set_level(uint8_t level);

/**
 * @brief 获取当前调试等级
 * @return 调试等级
 */
uint8_t hal_debug_get_level(void);

/**
 * @brief 查询舵机校准模式是否激活
 * @return true 表示校准模式激活中（主循环应跳过 servo flush）
 */
bool hal_is_calibration_active(void);

#endif /* HEXAPOD_HAL_H */

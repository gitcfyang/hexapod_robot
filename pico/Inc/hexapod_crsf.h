/**
 * @file hexapod_crsf.h
 * @brief CRSF (Crossfire) 接收器协议解析
 * @note 用于 ELRS 接收器，通过 UART 接收遥控信号
 *       将摇杆值映射到六足机器人的运动控制
 * 
 * CRSF 包格式：
 *   0xC8 (Sync) | len | type | payload... | crc
 * 
 * RC通道数据包 (type=0x16):
 *   payload: ch0(16bit) ch1(16bit) ... ch15(16bit)
 *   每个通道值范围: 172~1811 (PWM us)
 *   中位值: 992
 */

#ifndef HEXAPOD_CRSF_H
#define HEXAPOD_CRSF_H

#include <stdint.h>
#include <stdbool.h>
#include "hexapod_types.h"

/* ==================== CRSF 协议常量 ==================== */

#define CRSF_SYNC_BYTE          0xC8
#define CRSF_FRAME_SIZE_MAX     64

/* CRSF 包类型 */
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED   0x16
#define CRSF_FRAMETYPE_SUBS_RX               0x00
#define CRSF_FRAMETYPE_LINK_STATISTICS       0x14
#define CRSF_FRAMETYPE_DEVICE_INFO           0x29

/* RC 通道映射 — 默认值（可被 hexapod_config.h 覆盖） */
#ifndef CRSF_CHANNEL_FORWARD
#define CRSF_CHANNEL_FORWARD      0   // 前后
#endif
#ifndef CRSF_CHANNEL_STRAFE
#define CRSF_CHANNEL_STRAFE       1   // 左右平移
#endif
#ifndef CRSF_CHANNEL_TURN
#define CRSF_CHANNEL_TURN         3   // 旋转
#endif
#ifndef CRSF_CHANNEL_HEIGHT
#define CRSF_CHANNEL_HEIGHT       2   // 升降
#endif
#ifndef CRSF_CHANNEL_ARM
#define CRSF_CHANNEL_ARM          4   // 解锁/上电
#endif
#ifndef CRSF_CHANNEL_GAIT
#define CRSF_CHANNEL_GAIT         5   // 步态切换
#endif
#ifndef CRSF_CHANNEL_SPEED
#define CRSF_CHANNEL_SPEED        6   // 速度控制
#endif
#ifndef CRSF_CHANNEL_BALANCE
#define CRSF_CHANNEL_BALANCE      7   // 平衡模式
#endif

/* CRSF 通道值范围（PWM us 等效值） */
#define CRSF_CH_VALUE_MIN         172
#define CRSF_CH_VALUE_MID         992
#define CRSF_CH_VALUE_MAX         1811

/* 死区参数：优先使用 hexapod_config.h 中的配置，否则使用默认值 */
#ifndef CRSF_CH_VALUE_DEADBAND
#define CRSF_CH_VALUE_DEADBAND    40   // 原始通道死区（CRSF units）
#endif
#ifndef CONTROL_DEADBAND
#define CONTROL_DEADBAND          15   // 控制量死区（-500~+500 范围）
#endif
#ifndef HEIGHT_CONTROL_THRESHOLD
#define HEIGHT_CONTROL_THRESHOLD  (CONTROL_DEADBAND * 2)  // 高度积分控制阈值
#endif
#ifndef BODY_HEIGHT_RANGE_MM
#define BODY_HEIGHT_RANGE_MM      40   // 机身高度线性控制范围 (mm)
#endif
#ifndef BODY_ROTATION_MAX
#define BODY_ROTATION_MAX         200  // 机身姿态旋转范围 (0.1°, 200=20°)
#endif

/* ==================== CRSF 状态结构体 ==================== */

typedef struct {
    /* 原始通道数据 (0-15) */
    uint16_t channels[16];
    
    /* 解析后的控制值 */
    int16_t  forward;        // -500 ~ +500 (mm/s 或步长比例)
    int16_t  strafe;         // -500 ~ +500
    int16_t  turn;           // -500 ~ +500 (旋转速度)
    int16_t  height;         // -500 ~ +500 (抬腿高度调整)
    
    /* 开关/按钮状态 */
    bool     armed;          // 解锁状态
    uint8_t  gait_select;    // 步态选择 (0~4)
    uint8_t  speed_level;    // 速度等级 (0~100)
    bool     balance_mode;   // 平衡模式开关
    
    /* 帧状态 */
    uint32_t last_frame_time_ms;   // 最后有效帧时间
    uint32_t frame_count;          // 帧计数
    bool     link_connected;       // 链接状态
} crsf_state_t;

/* ==================== CRSF 解析器状态 ==================== */

typedef struct {
    uint8_t  rx_buf[CRSF_FRAME_SIZE_MAX];   // 接收缓冲区
    uint8_t  rx_index;                      // 当前接收位置
    bool     sync_found;                    // 同步头找到
    uint8_t  expected_len;                  // 期望帧长度
} crsf_parser_t;

/* ==================== 函数声明 ==================== */

/**
 * @brief 初始化 CRSF 解析器
 * @param parser 解析器实例
 */
void crsf_parser_init(crsf_parser_t *parser);

/**
 * @brief CRSF 状态初始化
 * @param state CRSF 状态实例
 */
void crsf_state_init(crsf_state_t *state);

/**
 * @brief 处理一个接收到的字节（在 UART IRQ 中调用）
 * @param parser 解析器实例
 * @param byte 接收到的字节
 * @param timestamp_ms 当前时间戳（毫秒），用于记录帧接收时间
 * @param state 输出：完整帧解析后的状态更新
 * @return true 表示解析出一个完整帧
 */
bool crsf_parse_byte(crsf_parser_t *parser, uint8_t byte, uint32_t timestamp_ms, crsf_state_t *state);

/**
 * @brief 将 CRSF 通道值映射到机器人控制
 * @param state CRSF 状态（包含原始通道值）
 * @param ctrl_state 输出：机器人控制状态
 */
void crsf_to_control(const crsf_state_t *state, control_state_t *ctrl_state);

/**
 * @brief 检查链接超时
 * @param state CRSF 状态
 * @param timeout_ms 超时时间（毫秒）
 * @param current_ms 当前时间
 * @return true 表示链接正常
 */
bool crsf_check_link(const crsf_state_t *state, uint32_t timeout_ms, uint32_t current_ms);

/**
 * @brief 计算 CRSF CRC
 * @param data 数据指针
 * @param len 数据长度
 * @return CRC 值
 */
uint8_t crsf_crc8(const uint8_t *data, uint8_t len);

#endif /* HEXAPOD_CRSF_H */

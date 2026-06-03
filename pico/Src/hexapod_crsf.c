/**
 * @file hexapod_crsf.c
 * @brief CRSF (Crossfire) 接收器协议解析实现
 *
 * 将 ELRS 接收器的摇杆信号转换为六足机器人的运动控制命令
 *
 * 摇杆映射 (ELRS 标准通道顺序):
 *
 *   正常模式 (CH8=低位):
 *     CH1 (Ail/Roll)  → 左右平移 (Strafe)
 *     CH2 (Ele/Pitch) → 前进/后退 (Forward)
 *     CH3 (Throttle)  → 机身高度 (线性直接映射)
 *     CH4 (Rud/Yaw)   → 原地旋转 (Turn)
 *     CH5 (SWA)       → 解锁/上电 (ARM)
 *     CH6 (SWB)       → 步态选择 (三段开关)
 *     CH7 (SWC)       → 速度控制 (三段开关)
 *     CH8 (SWD)       → 平衡模式 (二段开关)
 *
 *   平衡模式 (CH8=高位):
 *     CH1 (Ail/Roll)  → 机身横滚 Roll
 *     CH2 (Ele/Pitch) → 机身俯仰 Pitch
 *     CH3 (Throttle)  → 机身高度 (线性直接映射)
 *     CH4 (Rud/Yaw)   → 机身偏航 Yaw
 *     机器人原地不动
 */

#include "hexapod_config.h"
#include "hexapod_crsf.h"
#include "hexapod_gait.h"
#include <string.h>

/**
 * @brief CRSF CRC8 表 (多项式 0xD5)
 */
static const uint8_t crsf_crc8_table[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
    0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
    0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
    0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
    0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
    0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
    0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
    0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
    0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
    0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

/* ==================== CRC 计算 ==================== */

uint8_t crsf_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = crsf_crc8_table[crc ^ data[i]];
    }
    return crc;
}

/* ==================== 解析器初始化和状态初始化 ==================== */

void crsf_parser_init(crsf_parser_t *parser)
{
    if (!parser) return;
    memset(parser, 0, sizeof(crsf_parser_t));
}

void crsf_state_init(crsf_state_t *state)
{
    if (!state) return;
    memset(state, 0, sizeof(crsf_state_t));
    
    /* 初始化通道中位值 */
    for (int i = 0; i < 16; i++) {
        state->channels[i] = CRSF_CH_VALUE_MID;
    }
    
    state->link_connected = false;
}

/* ==================== 字节级解析 ==================== */

bool crsf_parse_byte(crsf_parser_t *parser, uint8_t byte, uint32_t timestamp_ms, crsf_state_t *state)
{
    if (!parser || !state) return false;
    
    bool frame_complete = false;
    
    switch (parser->rx_index) {
        case 0:
            /* 等待同步头 */
            if (byte == CRSF_SYNC_BYTE) {
                parser->rx_buf[0] = byte;
                parser->rx_index = 1;
                parser->sync_found = true;
            }
            break;
            
        case 1:
            /* 接收长度字节（包含 type + payload + crc，不包含 sync 和 len 自身） */
            if (byte >= 2 && byte <= CRSF_FRAME_SIZE_MAX - 2) {
                parser->rx_buf[1] = byte;
                parser->expected_len = byte + 2;  // 总帧长 = sync(1) + len(1) + len 指示的字节数
                parser->rx_index = 2;
            } else {
                /* 非法长度，重置 */
                parser->rx_index = 0;
                parser->sync_found = false;
            }
            break;
            
        default:
            /* 接收剩余数据 */
            if (parser->rx_index < CRSF_FRAME_SIZE_MAX) {
                parser->rx_buf[parser->rx_index] = byte;
                parser->rx_index++;
                
                /* 检查帧是否完整 */
                if (parser->rx_index >= parser->expected_len) {
                    frame_complete = true;
                }
            } else {
                /* 缓冲区溢出，重置 */
                parser->rx_index = 0;
                parser->sync_found = false;
            }
            break;
    }
    
    if (frame_complete) {
        uint8_t frame_len = parser->rx_buf[1];          // 长度字段
        uint8_t type = parser->rx_buf[2];               // 类型字段
        (void)type;
        uint8_t crc_received = parser->rx_buf[frame_len + 1];  // 最后一个字节是CRC
        
        /* 验证 CRC（计算 type + payload） */
        uint8_t crc_calc = crsf_crc8(&parser->rx_buf[2], frame_len - 1);
        bool crc_ok = (crc_calc == crc_received);
        
        if (crc_ok && type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
            /* 解析 RC 通道数据
             * payload 是 16 通道 × 11bit = 176bit = 22 字节
             * CRSF 使用小端序打包 11bit 每通道 */
            const uint8_t *payload = &parser->rx_buf[3];
            
            uint8_t bit_pos = 0;
            for (int ch = 0; ch < 16; ch++) {
                uint8_t byte_index = bit_pos / 8;
                uint8_t bit_offset = bit_pos % 8;
                
                /* 安全检查：16通道×11bit需要22字节payload
                 * byte_index最大为20 (ch=15时, bit_pos=165, byte_index=20)
                 * 确保 byte_index+2 < 22 不越界 */
                if (byte_index + 2 >= 22) {
                    break;
                }
                
                /* 小端序读取 2 或 3 字节，然后右移 bit_offset */
                uint32_t raw = (uint32_t)payload[byte_index] |
                              ((uint32_t)payload[byte_index + 1] << 8);
                if (bit_offset > 5) {
                    raw |= ((uint32_t)payload[byte_index + 2] << 16);
                }
                raw >>= bit_offset;
                
                state->channels[ch] = (uint16_t)(raw & 0x07FF);  // 11bit mask
                bit_pos += 11;
            }
            
            state->frame_count++;
            state->last_frame_time_ms = timestamp_ms;
            state->link_connected = true;
        }
        
        /* 重置解析器，准备下一帧 */
        parser->rx_index = 0;
        parser->sync_found = false;
    }
    
    return frame_complete;
}

/* ==================== 通道值映射 ==================== */

/**
 * @brief 将 CRSF 通道值 (172~1811) 映射到 -500~+500 范围
 * @param ch_value 通道原始值
 * @return 映射后的控制值 (-500 ~ +500)
 */
static int16_t map_channel_to_control(uint16_t ch_value)
{
    int16_t result;
    
    if (ch_value < CRSF_CH_VALUE_MID - CRSF_CH_VALUE_DEADBAND) {
        /* 负方向 */
        int16_t diff = CRSF_CH_VALUE_MID - CRSF_CH_VALUE_DEADBAND - ch_value;
        int32_t range = CRSF_CH_VALUE_MID - CRSF_CH_VALUE_DEADBAND - CRSF_CH_VALUE_MIN;
        if (range > 0) {
            result = (int16_t)(-500 * (int32_t)diff / range);
        } else {
            result = 0;
        }
        if (result < -500) result = -500;
    } else if (ch_value > CRSF_CH_VALUE_MID + CRSF_CH_VALUE_DEADBAND) {
        /* 正方向 */
        int16_t diff = ch_value - CRSF_CH_VALUE_MID - CRSF_CH_VALUE_DEADBAND;
        int32_t range = CRSF_CH_VALUE_MAX - (CRSF_CH_VALUE_MID + CRSF_CH_VALUE_DEADBAND);
        if (range > 0) {
            result = (int16_t)(500 * (int32_t)diff / range);
        } else {
            result = 0;
        }
        if (result > 500) result = 500;
    } else {
        /* 死区内，输出 0 */
        result = 0;
    }
    
    return result;
}

/**
 * @brief 第 2 级死区：将映射后的控制量中小于阈值的值归零
 * @param value 映射后的控制值 (-500 ~ +500)
 * @return 死区处理后的值
 *
 * 与第 1 级死区 (map_channel_to_control 中的 CRSF_CH_VALUE_DEADBAND) 串联，
 * 确保摇杆偏离中位足够远时才产生运动。
 */
static int16_t apply_control_deadband(int16_t value)
{
    if (value > -CONTROL_DEADBAND && value < CONTROL_DEADBAND) {
        return 0;
    }
    return value;
}

/**
 * @brief 将 CRSF 通道值映射到三档开关 (-500, 0, +500)
 * 用于步态选择
 */
static int8_t map_to_3pos(uint16_t ch_value)
{
    if (ch_value < CRSF_CH_VALUE_MID - 200) return -1;   // 低档
    if (ch_value > CRSF_CH_VALUE_MID + 200) return 1;    // 高档
    return 0;                                              // 中档
}

/**
 * @brief 将 CRSF 通道值映射到二档开关 (0/1)
 * 用于解锁和平衡模式
 */
static bool map_to_2pos(uint16_t ch_value)
{
    return (ch_value > CRSF_CH_VALUE_MID + 200);
}

/* ==================== CRSF 到机器人控制 ==================== */

void crsf_to_control(const crsf_state_t *state, control_state_t *ctrl_state)
{
    if (!state || !ctrl_state) return;

    /* ---- 开关通道解析（正常/平衡模式共用） ---- */
    bool  arm       = map_to_2pos(state->channels[CRSF_CHANNEL_ARM]);
    int8_t gait_pos = map_to_3pos(state->channels[CRSF_CHANNEL_GAIT]);
    bool  balance   = map_to_2pos(state->channels[CRSF_CHANNEL_BALANCE]);

    /* ---- 解锁/上电 : 边沿触发 ---- */
    {
        static bool prev_arm = false;
        if (arm && !prev_arm) {
            ctrl_state->robot_on = true;
        } else if (!arm && prev_arm) {
            ctrl_state->robot_on = false;
        }
        prev_arm = arm;
    }

    /* ---- 步态选择 ---- */
    if (gait_pos == -1) {
        if (ctrl_state->gait_type != GAIT_RIPPLE_12) {
            ctrl_state->gait_type = GAIT_RIPPLE_12;
            hexapod_gait_select(GAIT_RIPPLE_12, ctrl_state);
        }
    } else if (gait_pos == 0) {
        if (ctrl_state->gait_type != GAIT_TRIPOD_6) {
            ctrl_state->gait_type = GAIT_TRIPOD_6;
            hexapod_gait_select(GAIT_TRIPOD_6, ctrl_state);
        }
    } else {
        if (ctrl_state->gait_type != GAIT_WAVE_24) {
            ctrl_state->gait_type = GAIT_WAVE_24;
            hexapod_gait_select(GAIT_WAVE_24, ctrl_state);
        }
    }

    /* ---- 平衡模式开关 ---- */
    ctrl_state->balance_mode = balance;

    if (balance) {
        /* ====== 平衡模式 ======
         *
         * 摇杆映射:
         *   CH1 (Ail/Roll)  → body_rot.z  机身横滚 Roll
         *   CH2 (Ele/Pitch) → body_rot.x  机身俯仰 Pitch
         *   CH3 (Throttle)  → body_pos.y  机身高度 (线性)
         *   CH4 (Rud/Yaw)   → body_rot.y  机身偏航 Yaw
         *
         * 机器人原地不动 (travel_length 全部置零) */

        int16_t roll_stick  = apply_control_deadband(map_channel_to_control(state->channels[CRSF_CHANNEL_STRAFE]));
        int16_t pitch_stick = apply_control_deadband(map_channel_to_control(state->channels[CRSF_CHANNEL_FORWARD]));
        int16_t height_stick = apply_control_deadband(map_channel_to_control(state->channels[CRSF_CHANNEL_HEIGHT]));
        int16_t yaw_stick   = apply_control_deadband(map_channel_to_control(state->channels[CRSF_CHANNEL_TURN]));

#if STRAFE_DIRECTION_INVERT
        roll_stick = -roll_stick;
#endif
#if FORWARD_DIRECTION_INVERT
        pitch_stick = -pitch_stick;
#endif
#if HEIGHT_DIRECTION_INVERT
        height_stick = -height_stick;
#endif
#if TURN_DIRECTION_INVERT
        yaw_stick = -yaw_stick;
#endif

        /* 机身姿态旋转 (0.1° 单位) */
        ctrl_state->body_rot.x =  (pitch_stick * BODY_ROTATION_MAX) / 500;  /* 俯仰 */
        ctrl_state->body_rot.y = -(yaw_stick   * BODY_ROTATION_MAX) / 500;  /* 偏航, 取反使摇杆方向与机头转向一致 */
        ctrl_state->body_rot.z =  (roll_stick  * BODY_ROTATION_MAX) / 500;  /* 横滚 */

        /* 机身高度 (线性): 摇杆推高→机身抬升, 摇杆拉低→机身下降 */
        ctrl_state->body_pos.y = (height_stick * BODY_HEIGHT_RANGE_MM) / 500;

        /* 停止行走 */
        ctrl_state->travel_length.x = 0;
        ctrl_state->travel_length.y = 0;
        ctrl_state->travel_length.z = 0;
    } else {
        /* ====== 正常模式 ======
         *
         * 摇杆映射:
         *   CH1 (Ail/Roll)  → travel_length.z  左右平移
         *   CH2 (Ele/Pitch) → travel_length.x  前进/后退
         *   CH3 (Throttle)  → body_pos.y       机身高度 (线性)
         *   CH4 (Rud/Yaw)   → travel_length.y  原地旋转 */

        int16_t strafe  = apply_control_deadband(map_channel_to_control(state->channels[CRSF_CHANNEL_STRAFE]));
        int16_t forward = apply_control_deadband(map_channel_to_control(state->channels[CRSF_CHANNEL_FORWARD]));
        int16_t height_ctrl = apply_control_deadband(map_channel_to_control(state->channels[CRSF_CHANNEL_HEIGHT]));
        int16_t turn    = apply_control_deadband(map_channel_to_control(state->channels[CRSF_CHANNEL_TURN]));

#if STRAFE_DIRECTION_INVERT
        strafe = -strafe;
#endif
#if FORWARD_DIRECTION_INVERT
        forward = -forward;
#endif
#if HEIGHT_DIRECTION_INVERT
        height_ctrl = -height_ctrl;
#endif
#if TURN_DIRECTION_INVERT
        turn = -turn;
#endif

        /* 步长映射: 摇杆 -500~+500 → 步长 mm */
        ctrl_state->travel_length.x =  (forward * TRAVEL_MAX_FORWARD_MM) / 500;
        ctrl_state->travel_length.z = -(strafe  * TRAVEL_MAX_STRAFE_MM)  / 500;  /* 取反使摇杆右推→右平移 */
        ctrl_state->travel_length.y =  (turn    * TRAVEL_MAX_TURN_MM)    / 500;

        /* 机身高度 (线性): 摇杆偏离中位直接映射到 body_pos.y
         * 替代了原来的积分器, 响应更直接, 中位→高度不变 */
        ctrl_state->body_pos.y = (height_ctrl * BODY_HEIGHT_RANGE_MM) / 500;

        /* 正常模式下无姿态旋转 */
        ctrl_state->body_rot.x = 0;
        ctrl_state->body_rot.y = 0;
        ctrl_state->body_rot.z = 0;

        /* ---- 仿生连续变速：摇杆幅度 → 步态频率 ----
         *
         * 取三轴摇杆的最大绝对值作为“运动意图强度”(0-500)。
         * 强度越大 → 步态周期越短 (频率越高)。
         * 低摇杆 = 短步长 + 低频率 → 精细缓动
         * 高摇杆 = 大步长 + 高频率 → 快速行进 */
        const int16_t period_max = GAIT_PERIOD_MAX_MS;
        const int16_t period_min = GAIT_PERIOD_MIN_MS;
        const int32_t range = (int32_t)period_max - (int32_t)period_min;

        int16_t stick_mag = (forward >= 0) ? forward : -forward;
        int16_t tmp = (strafe >= 0) ? strafe : -strafe;
        if (tmp > stick_mag) stick_mag = tmp;
        tmp = (turn >= 0) ? turn : -turn;
        if (tmp > stick_mag) stick_mag = tmp;

        if (stick_mag <= CONTROL_DEADBAND) {
            ctrl_state->speed_control = period_max;
        } else {
            ctrl_state->speed_control = period_max
                - (int16_t)(range * (int32_t)stick_mag / 500);
        }
    }

    /* 抬腿高度边界钳位 (由 DEFAULT_LEG_LIFT_HEIGHT 初始化, 用户可通过串口 !U/!D 微调) */
    if (ctrl_state->leg_lift_height > LIFT_HEIGHT_MAX_MM)
        ctrl_state->leg_lift_height = LIFT_HEIGHT_MAX_MM;
    if (ctrl_state->leg_lift_height < LIFT_HEIGHT_MIN_MM)
        ctrl_state->leg_lift_height = LIFT_HEIGHT_MIN_MM;
}

bool crsf_check_link(const crsf_state_t *state, uint32_t timeout_ms, uint32_t current_ms)
{
    if (!state) return false;
    
    if (!state->link_connected) return false;
    
    /* 检查最后收到帧的时间 */
    if (state->last_frame_time_ms > 0) {
        uint32_t elapsed = current_ms - state->last_frame_time_ms;
        if (elapsed > timeout_ms) {
            return false;  /* 链接超时 */
        }
    }
    
    return true;
}

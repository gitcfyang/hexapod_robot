/**
 * @file hexapod_crsf.c
 * @brief CRSF (Crossfire) 接收器协议解析实现
 *
 * 将 ELRS 接收器的摇杆信号转换为六足机器人的运动控制命令
 *
 * 本文件只负责「字节流 → 通道数组」的解析; 「通道 → 机器人动作」在
 * hexapod_input.c (统一输入层), 与 PS2 共用同一套参数与模式矩阵。
 *
 * 默认映射 (ELRS 标准通道顺序, 全部可在网页改):
 *
 *   正常模式:
 *     CH1 (Ail/Roll)  → 左右平移 (Strafe)
 *     CH2 (Ele/Pitch) → 前进/后退 (Forward)
 *     CH3 (Throttle)  → 机身高度 (默认积分=速率输入, 见 height_integrate)
 *     CH4 (Rud/Yaw)   → 原地旋转 (Turn)
 *
 *   平衡模式:
 *     CH1 (Ail/Roll)  → 机身横滚 Roll
 *     CH2 (Ele/Pitch) → 机身俯仰 Pitch
 *     CH3 (Throttle)  → 机身高度
 *     CH4 (Rud/Yaw)   → 机身偏航 Yaw
 *     机器人原地不动
 *
 *   CH5~CH8 开关走模式矩阵 (mode_*): 默认 CH5 高位=解锁 (电平),
 *   CH6 三段=步态, CH7 三段=站立姿态, CH8 高位=平衡模式。
 */

#include "hexapod_config.h"
#include "hexapod_crsf.h"
#include "hexapod_input.h"
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

/* ==================== CRSF 到机器人控制 ==================== */

/**
 * @brief 把一帧 CRSF 通道值交给统一输入层
 *
 * 本函数只做"翻译": CH1~CH16 原值填进统一通道 0~15, 其余伪通道
 * (16~19, 只有 PS2 才有对应物) 填中位。真正的映射、死区、模式矩阵、
 * 组合键都在 hexapod_input.c —— 于是 CRSF 与 PS2 共享同一套参数。
 */
void crsf_to_control(const crsf_state_t *state, control_state_t *ctrl_state)
{
    if (!state || !ctrl_state) return;

    uint16_t ch[HEXINP_CH_COUNT];
    for (int i = 0; i < 16; i++) ch[i] = state->channels[i];
    for (int i = 16; i < HEXINP_CH_COUNT; i++) ch[i] = CRSF_CH_VALUE_MID;

    /* 按键掩码 = CH5~CH16 派生 (按钮下标 i ↔ 通道 4+i): 值 > 1192 算按下,
     * 阈值与模式矩阵的 HIGH 档一致 —— 开关拨到高位即"按下", 组合键因此
     * 也能用遥控器开关表达 (两个开关按住 = 和弦)。伪通道 16~19 恒中位,
     * 所以高 4 位 (△○×□ 对应的下标 12~15) 永远不会置位。
     * 某目标在矩阵里有分配行时该目标的组合键会被让位, 见 hexapod_input.c。 */
    uint16_t btns = 0;
    for (int i = 0; i < 16; i++) {
        if (ch[HEXINP_CH_BTN_BASE + i] > CRSF_CH_VALUE_MID + 200) {
            btns |= (uint16_t)(1u << i);
        }
    }

    hexapod_input_apply(HEXINP_PROTO_CRSF, ch, btns, ctrl_state);
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

/**
 * @file hexapod_ps2.c
 * @brief PS2 无线手柄接收器 bit-bang SPI 驱动实现
 * @note 通过软件模拟 SPI 协议与 PS2 接收器通信
 *
 * 参考: Bill Porter PS2X 库, STM32 PS2 解码库
 *
 * 协议特点:
 *   - LSB-first 数据传输
 *   - 时钟空闲高 (CPOL=1), 下降沿改变数据, 上升沿采样
 *   - SEL 在整个 9 字节帧期间保持低电平
 *   - DAT 线开漏, 需外部或内部上拉
 *   - 标准时钟频率 250kHz, 本实现使用 ~62.5kHz (可靠, 裕量大)
 *
 * 本文件只负责「时序 → 一帧原始数据」; 「摇杆/按键 → 机器人动作」在
 * hexapod_input.c (统一输入层), 与 CRSF 共用同一套参数、模式矩阵与组合键表。
 *
 * 摇杆插槽 (统一通道 0~3, 与 CRSF 同语义; 通道号可在网页改):
 *   RX (右摇杆 X) → 平移/横滚     RY (右摇杆 Y) → 前进/俯仰
 *   LY (左摇杆 Y) → 机身高度       LX (左摇杆 X) → 转向/偏航
 * 姿态模式 (组合键 SELECT 切换) 下同样的摇杆改驱动机身姿态。
 *
 * 按键 (默认组合键, 全部可在网页「模式」页改配):
 *   START      → 解锁/锁定           SELECT → 平衡模式开关 (进入时蜂鸣一声)
 *   D-Pad ↑/↓  → 步态上/下一个       D-Pad ←/→ → 站立姿态循环
 *   × 按住 + ↑/↓ → 抬腿高度 ±5       ○ → 紧急停止
 *   △ (Triangle) → 已禁用 (曾: 调试等级循环)
 *   L1/R1/L2/R2/L3/R3/□ → 保留 (未分配)
 */

#include "hexapod_ps2.h"
#include "hexapod_config.h"
#include "hexapod_input.h"
#include "hexapod_hal.h"
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

/* ==================== 内部帮助函数 ==================== */

/**
 * @brief bit-bang 传输一个字节 (同时发送和接收)
 * @param cmd 要发送的字节
 * @return 接收到的字节
 *
 * 时序: CLK 空闲高 → CMD 置位 → CLK↓ (数据变化) → 读 DAT → CLK↑ (数据保持)
 * LSB first: bit 0 先发/先收
 */
static uint8_t ps2_xfer_byte(uint8_t cmd)
{
    uint8_t data = 0;

    for (uint16_t ref = 0x01; ref < 0x100; ref <<= 1) {
        /* 设置 CMD 位 */
        gpio_put(PS2_CMD_PIN, (cmd & ref) ? 1 : 0);

        /* 下降沿: 数据变化 */
        gpio_put(PS2_CLK_PIN, 0);
        sleep_us(PS2_CLK_DELAY_US);

        /* 读取 DAT 位 */
        if (gpio_get(PS2_DAT_PIN))
            data |= ref;

        /* 上升沿: 数据保持 */
        gpio_put(PS2_CLK_PIN, 1);
        sleep_us(PS2_CLK_DELAY_US);
    }

    return data;
}

/**
 * @brief 发送/接收完整 9 字节帧
 * @param cmd 9 字节命令数组 (主机→手柄)
 * @param resp 9 字节响应数组 (手柄→主机), 可为 NULL
 *
 * SEL 在整个传输期间拉低, 传完后拉高。
 */
static void ps2_send_frame(const uint8_t *cmd, uint8_t *resp)
{
    gpio_put(PS2_SEL_PIN, 0);
    sleep_us(PS2_CLK_DELAY_US);

    for (uint8_t i = 0; i < 9; i++) {
        uint8_t rx = ps2_xfer_byte(cmd[i]);
        if (resp) resp[i] = rx;
    }

    gpio_put(PS2_SEL_PIN, 1);
    sleep_us(PS2_CLK_DELAY_US);
}

/* ==================== 初始化和配置 ==================== */

void ps2_init(void)
{
    /* DAT: 输入 + 内部上拉 (手柄开漏输出) */
    gpio_init(PS2_DAT_PIN);
    gpio_set_dir(PS2_DAT_PIN, GPIO_IN);
    gpio_pull_up(PS2_DAT_PIN);

    /* CMD/SEL/CLK: 推挽输出 */
    gpio_init(PS2_CMD_PIN);
    gpio_set_dir(PS2_CMD_PIN, GPIO_OUT);

    gpio_init(PS2_SEL_PIN);
    gpio_set_dir(PS2_SEL_PIN, GPIO_OUT);
    gpio_put(PS2_SEL_PIN, 1);   /* 未选中 */

    gpio_init(PS2_CLK_PIN);
    gpio_set_dir(PS2_CLK_PIN, GPIO_OUT);
    gpio_put(PS2_CLK_PIN, 1);   /* 空闲高 */
}

bool ps2_read_gamepad(ps2_state_t *state)
{
    if (!state) return false;

    static const uint8_t poll_cmd[9] = {
        0x01, 0x42,       /* 开始 + 请求数据 */
        0x00, 0x00,       /* 马达控制 (不使用) */
        0x00, 0x00, 0x00, 0x00, 0x00
    };

    uint8_t buf[9] = {0};
    ps2_send_frame(poll_cmd, buf);

    /* 帧头校验: PS2 帧首字节恒为 0xFF, 丢弃噪声/时序错误产生的垃圾帧
     * (垃圾帧会造成幽灵按键: 如 △ 误触发调试等级循环、START 误解锁) */
    if (buf[0] != 0xFF) {
        state->connected = false;
        return false;
    }

    /* 验证手柄 ID */
    uint8_t id = buf[1];
    if (id != PS2_ID_DIGITAL && id != PS2_ID_ANALOG_RED
        && id != PS2_ID_ANALOG_GREEN && id != PS2_ID_WIRELESS) {
        state->connected = false;
        return false;
    }

    /* 验证数据就绪标志 */
    if (buf[2] != PS2_DATA_READY) {
        /* 某些手柄或配置下 buf[2] 可能不是 0x5A, 放宽检查 */
        /* 仍继续解析，但标记为非标准 */
    }

    /* 存储 */
    memcpy(state->data, buf, 9);
    state->id     = id;
    state->buttons = ((uint16_t)buf[4] << 8) | buf[3];
    state->joy_rx = buf[5];
    state->joy_ry = buf[6];
    state->joy_lx = buf[7];
    state->joy_ly = buf[8];
    state->analog_mode = (id == PS2_ID_ANALOG_RED || id == PS2_ID_ANALOG_GREEN
                          || id == PS2_ID_WIRELESS);   /* 无线接收器 0x79 = 模拟模式 */
    state->last_read_ms = to_ms_since_boot(get_absolute_time());
    state->frame_count++;
    state->connected = true;

    /* 摇杆中位校准: 进入模拟模式后采样前 N 帧的滚动平均值
     * ⚠️ 采样期间摇杆需保持居中, 否则中位会偏移 */
    if (state->analog_mode && state->center_samples < PS2_CENTER_CALIB_SAMPLES) {
        uint16_t n = state->center_samples;
        state->center_lx = (uint8_t)(((uint16_t)state->center_lx * n + buf[7]) / (n + 1));
        state->center_ly = (uint8_t)(((uint16_t)state->center_ly * n + buf[8]) / (n + 1));
        state->center_rx = (uint8_t)(((uint16_t)state->center_rx * n + buf[5]) / (n + 1));
        state->center_ry = (uint8_t)(((uint16_t)state->center_ry * n + buf[6]) / (n + 1));
        state->center_samples = (uint8_t)(n + 1);
    }

    return true;
}

bool ps2_enter_analog_mode(ps2_state_t *state)
{
    if (!state) return false;

    uint8_t buf[9];

    /* 重置摇杆中位校准: 进入模拟模式的瞬间要求摇杆居中,
     * 之后的 10 帧采样平均值作为各轴中位 */
    state->center_lx = 128; state->center_ly = 128;
    state->center_rx = 128; state->center_ry = 128;
    state->center_samples = 0;

    /* 步骤 1: 3 次短轮询建立连接 */
    for (int i = 0; i < 3; i++) {
        ps2_read_gamepad(state);
        sleep_ms(10);
    }

    /* 步骤 2: 进入配置模式
     * 命令: 0x01 0x43 0x00 0x01 0x00 ... */
    {
        static const uint8_t enter_cfg[9] = {
            0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
        };
        ps2_send_frame(enter_cfg, buf);
        sleep_ms(1);
    }

    /* 步骤 3: 启用模拟模式 (红灯)
     * 命令: 0x01 0x44 0x00 0x01 0x03 0x00 0x00 0x00 0x00
     * 最后一位 0x03 表示启用摇杆模拟 + 锁定模式 */
    {
        static const uint8_t analog_cmd[9] = {
            0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00
        };
        ps2_send_frame(analog_cmd, buf);
        sleep_ms(1);
    }

    /* 步骤 4: 退出配置模式 (锁定设置)
     * 命令: 0x01 0x43 0x00 0x00 0x5A 0x5A 0x5A 0x5A 0x5A */
    {
        static const uint8_t exit_cfg[9] = {
            0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A
        };
        ps2_send_frame(exit_cfg, buf);
        sleep_ms(10);
    }

    /* 验证: 再读一次，看是否进入红灯模式 */
    ps2_read_gamepad(state);

    return state->analog_mode;
}

/* ==================== PS2 → 统一输入层 ==================== */

/**
 * @brief 摇杆值 (0~255) → 统一通道量程 (172~1811)
 * @param val    摇杆值
 * @param center 中位校准值 (连接后自动采样, 见 ps2_read_gamepad)
 *
 * 以校准中位为 0, 两侧分别线性映射到中位到端点的行程 —— 于是中位漂移
 * 不会变成"松手还在慢慢走"。死区与曲线不在这里做: 统一输入层按
 * ch_deadband / expo 统一处理, 这样两种手柄共用同一套手感参数。
 */
static uint16_t ps2_stick_to_raw(uint8_t val, uint8_t center)
{
    int32_t d = (int32_t)val - (int32_t)center;
    int32_t raw;

    if (d >= 0) {
        raw = (int32_t)CRSF_CH_VALUE_MID
            + d * (CRSF_CH_VALUE_MAX - CRSF_CH_VALUE_MID) / 127;
    } else {
        raw = (int32_t)CRSF_CH_VALUE_MID
            - (-d) * (CRSF_CH_VALUE_MID - CRSF_CH_VALUE_MIN) / 128;
    }

    if (raw < CRSF_CH_VALUE_MIN) raw = CRSF_CH_VALUE_MIN;
    if (raw > CRSF_CH_VALUE_MAX) raw = CRSF_CH_VALUE_MAX;
    return (uint16_t)raw;
}

/**
 * @brief 把一帧手柄数据交给统一输入层
 *
 * 摇杆填统一通道 0~3 (数字模式无比例输出 → 全部填中位, 机器人不动),
 * 按键填 4~19 (按下 = 高位)。组合键、模式矩阵、行程映射都在
 * hexapod_input.c, 与 CRSF 走同一段代码。
 */
void ps2_to_control(const ps2_state_t *state, control_state_t *ctrl_state)
{
    if (!state || !ctrl_state || !state->connected) return;

    uint16_t ch[HEXINP_CH_COUNT];
    const uint16_t mid = CRSF_CH_VALUE_MID;

    /* 数字模式 (未进入模拟红灯模式) 没有摇杆比例输出 → 中位 = 不动 */
    const bool analog = state->analog_mode;
    ch[HEXINP_CH_STR]  = analog ? ps2_stick_to_raw(state->joy_rx, state->center_rx) : mid;
    ch[HEXINP_CH_FWD]  = analog ? ps2_stick_to_raw(state->joy_ry, state->center_ry) : mid;
    ch[HEXINP_CH_HGT]  = analog ? ps2_stick_to_raw(state->joy_ly, state->center_ly) : mid;
    ch[HEXINP_CH_TURN] = analog ? ps2_stick_to_raw(state->joy_lx, state->center_lx) : mid;

    /* 16 键 → 统一通道 4~19, 位序同 PSB_*; 原始帧里按下 = 0, 这里翻过来 */
    for (uint8_t i = 0; i < 16; i++) {
        ch[HEXINP_CH_BTN_BASE + i] = (state->buttons & (1u << i))
            ? CRSF_CH_VALUE_MAX : CRSF_CH_VALUE_MIN;
    }

    hexapod_input_apply(HEXINP_PROTO_PS2, ch, (uint16_t)~state->buttons, ctrl_state);
}

/* ==================== 链接检查 ==================== */

bool ps2_check_link(const ps2_state_t *state, uint32_t timeout_ms, uint32_t current_ms)
{
    if (!state || !state->connected) return false;

    if (state->last_read_ms > 0) {
        uint32_t elapsed = current_ms - state->last_read_ms;
        if (elapsed > timeout_ms) {
            return false;
        }
    }

    return true;
}

/* ==================== 状态获取 (供扩展模块使用) ==================== */

#if PS2_ENABLED
/* g_ps2_state 定义在 hexapod_hal_pico.c, 此处 extern 引用 */
extern ps2_state_t g_ps2_state;
#endif

const ps2_state_t* ps2_get_state(void)
{
#if PS2_ENABLED
    if (g_ps2_state.connected) {
        return &g_ps2_state;
    }
#endif
    return NULL;
}

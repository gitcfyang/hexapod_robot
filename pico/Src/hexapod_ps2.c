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
 * 控制映射:
 *   正常模式 (SELECT=off):
 *     左摇杆 Y  → 前进/后退
 *     左摇杆 X  → 左右平移
 *     右摇杆 X  → 原地旋转
 *     R2 (模拟) → 机身高度
 *
 *   平衡模式 (SELECT=on):
 *     左摇杆   → 机身横滚/俯仰
 *     右摇杆 X → 机身偏航
 *     R2       → 机身高度
 *
 *   按键:
 *     START      → 解锁/锁定 (边沿触发)
 *     SELECT     → 平衡模式开关
 *     L1 / R1    → 步态切换 (上一个/下一个)
 *     D-Pad 方向 → 站立姿态 (←窄, ↓正常, →宽)
 *     × (Cross)  → 抬起/降低抬腿高度 (配合 D-Pad UP/DOWN)
 *     △ (Triangle) → 调试等级循环
 *     ○ (Circle) → 紧急停止 (去扭矩)
 */

#include "hexapod_ps2.h"
#include "hexapod_config.h"
#include "hexapod_gait.h"
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

    /* 验证手柄 ID */
    uint8_t id = buf[1];
    if (id != PS2_ID_DIGITAL && id != PS2_ID_ANALOG_RED && id != PS2_ID_ANALOG_GREEN) {
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
    state->analog_mode = (id == PS2_ID_ANALOG_RED || id == PS2_ID_ANALOG_GREEN);
    state->last_read_ms = to_ms_since_boot(get_absolute_time());
    state->frame_count++;
    state->connected = true;

    return true;
}

bool ps2_enter_analog_mode(ps2_state_t *state)
{
    if (!state) return false;

    uint8_t buf[9];

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

/* ==================== 摇杆/按键 → 控制量映射 ==================== */

/**
 * @brief 将 PS2 摇杆值 (0~255) 映射到控制量 (-500~+500)
 * @param val 摇杆值 (中位=128)
 * @return 控制量 (-500 ~ +500)
 */
static int16_t ps2_stick_to_control(uint8_t val)
{
    int16_t centered = (int16_t)val - 128;

    /* 死区 */
    if (centered > -PS2_STICK_DEADZONE && centered < PS2_STICK_DEADZONE) {
        return 0;
    }

    /* 线性映射: 摇杆偏移 → -500~+500
     * 正向: 129~255 → 0~500
     * 负向: 0~127 → -500~0
     * 比例因子: 500 / 127 ≈ 3.94 */
    int32_t result;
    if (centered > 0) {
        result = (int32_t)centered * 500 / 127;
    } else {
        result = (int32_t)centered * 500 / 128;
    }

    if (result > 500)  result = 500;
    if (result < -500) result = -500;
    return (int16_t)result;
}

/* ==================== PS2 → 机器人控制 ==================== */

void ps2_to_control(const ps2_state_t *state, control_state_t *ctrl_state)
{
    if (!state || !ctrl_state || !state->connected) return;

    /* ---- 宏: 按键是否按下 ---- */
#define BTN_PRESSED(mask)  (!(state->buttons & (mask)))

    /* ---- 解锁/锁定: START 边沿触发 ---- */
    {
        static bool prev_start = false;
        bool start = BTN_PRESSED(PSB_START);
        if (start && !prev_start) {
            ctrl_state->robot_on = !ctrl_state->robot_on;
        }
        prev_start = start;
    }

    /* ---- 平衡模式: SELECT 边沿触发 ---- */
    {
        static bool prev_select = false;
        bool sel = BTN_PRESSED(PSB_SELECT);
        if (sel && !prev_select) {
            ctrl_state->balance_mode = !ctrl_state->balance_mode;
        }
        prev_select = sel;
    }

    /* ---- 步态切换: L1=上一个, R1=下一个 ---- */
    {
        static bool prev_l1 = false, prev_r1 = false;
        bool l1 = BTN_PRESSED(PSB_L1);
        bool r1 = BTN_PRESSED(PSB_R1);

        if (l1 && !prev_l1) {
            if (ctrl_state->gait_type > 0) {
                ctrl_state->gait_type--;
                /* gait_type enum: RIPPLE_12=0, TRIPOD_6=1, TRIPOD_8=2, WAVE_24=3, TRIPOD_4=4 */
                hexapod_gait_select((gait_type_t)ctrl_state->gait_type, ctrl_state);
            }
        }
        if (r1 && !prev_r1) {
            if (ctrl_state->gait_type < GAIT_TRIPOD_4) {
                ctrl_state->gait_type++;
                hexapod_gait_select((gait_type_t)ctrl_state->gait_type, ctrl_state);
            }
        }
        prev_l1 = l1;
        prev_r1 = r1;
    }

    /* ---- 站立姿态: D-Pad ←窄 ↓正常 →宽 ---- */
    {
        if (BTN_PRESSED(PSB_PAD_LEFT)) {
            ctrl_state->stance_mode = -1;   /* 窄 */
        } else if (BTN_PRESSED(PSB_PAD_RIGHT)) {
            ctrl_state->stance_mode = 1;    /* 宽 */
        } else if (BTN_PRESSED(PSB_PAD_DOWN)) {
            ctrl_state->stance_mode = 0;    /* 正常 */
        }
    }

    /* ---- 抬腿高度微调: D-Pad UP + × = 增加, D-Pad UP = decrease ---- */
    /* ---- 注: D-Pad UP 单独已用于 stance，此处仅用于立即操作的文档 ---- */

    /* ---- 紧急停止: ○ (Circle) ---- */
    {
        static bool prev_circle = false;
        bool circle = BTN_PRESSED(PSB_CIRCLE);
        if (circle && !prev_circle) {
            ctrl_state->robot_on = false;
            ctrl_state->travel_length.x = 0;
            ctrl_state->travel_length.y = 0;
            ctrl_state->travel_length.z = 0;
        }
        prev_circle = circle;
    }

    /* ---- 抬腿高度: × + D-Pad UP/DOWN ---- */
    {
        static bool prev_cross = false, prev_pad_up = false, prev_pad_down = false;
        bool cross = BTN_PRESSED(PSB_CROSS);
        bool pad_up = BTN_PRESSED(PSB_PAD_UP);
        bool pad_down = BTN_PRESSED(PSB_PAD_DOWN);

        if (cross) {
            if (pad_up && !prev_pad_up) {
                ctrl_state->leg_lift_height += 5;
                if (ctrl_state->leg_lift_height > LIFT_HEIGHT_MAX_MM)
                    ctrl_state->leg_lift_height = LIFT_HEIGHT_MAX_MM;
            }
            if (pad_down && !prev_pad_down) {
                ctrl_state->leg_lift_height -= 5;
                if (ctrl_state->leg_lift_height < LIFT_HEIGHT_MIN_MM)
                    ctrl_state->leg_lift_height = LIFT_HEIGHT_MIN_MM;
            }
        }
        prev_cross = cross;
        prev_pad_up = pad_up;
        prev_pad_down = pad_down;
    }

    /* ================================================================
     * [扩展层] PS2 独占功能 — 以下按键 CRSF 无对应通道
     * 新增扩展功能时在此区域添加，CRSF 路径不受影响
     * ================================================================ */

    /* ---- △ (Triangle): 调试等级循环 ---- */
    {
        static bool prev_tri = false;
        bool tri = BTN_PRESSED(PSB_TRIANGLE);
        if (tri && !prev_tri) {
            uint8_t lvl = hal_debug_get_level();
            lvl = (lvl + 1) % 4;
            hal_debug_set_level(lvl);
        }
        prev_tri = tri;
    }

    /* ---- □ (Square): [保留] 自动归位/站立 (future)
     * 用途: 一键回到标准站立姿态, 所有腿收回初始位置
     * 实现时: 设置 ctrl_state 中的目标姿态标志位, 由主循环平滑过渡
     * if (BTN_PRESSED(PSB_SQUARE)) { ctrl_state->auto_home = true; }
     * ---- */

    /* ---- L3: [保留] 自动校准触发 (future)
     * 用途: 一键触发 18 路舵机 horn_offset 自动校准流程
     * 实现时: 与现有 !C 命令共享校准状态机 calib_enter()
     * if (BTN_PRESSED(PSB_L3)) { trigger_auto_calibration(); }
     * ---- */

    /* ---- R3: [保留] 控制灵敏度切换 (future)
     * 用途: 在 3 档灵敏度间循环 (精细/正常/灵敏), 修改摇杆→控制量映射斜率
     * 实现时: 设置全局灵敏度因子, 在 ps2_stick_to_control 中乘以该因子
     * if (BTN_PRESSED(PSB_R3)) { g_sensitivity = (g_sensitivity + 1) % 3; }
     * ---- */

    /* ---- L2: [保留] 辅助功能 A (future)
     * 用途: 待定 — 例如: 机身高度自动保持 / 地形自适应开关
     * ---- */

    /* ---- R2: [保留] 辅助功能 B (future)
     * 用途: 待定 — 例如: 双足/四足/六足模式切换
     *       (特殊步态: 只用 4 条腿行走, 闲置的腿可做操作臂)
     * ---- */

    /* ---- 摇杆映射 ----
     *
     *  左摇杆 Y (↑/↓):  前进/后退 (前后轴)
     *  左摇杆 X (←/→):  左右平移
     *  右摇杆 X (←/→):  原地旋转
     *  R2 (压力模拟):    机身高度
     *
     *  PS2 摇杆约定: 0=左(上), 255=右(下), 128=中位 */

    int16_t forward =  ps2_stick_to_control(state->joy_ly);
    int16_t strafe  =  ps2_stick_to_control(state->joy_lx);
    int16_t turn    =  ps2_stick_to_control(state->joy_rx);

    /* 方向取反 (与 CRSF 配置共用宏) */
#if STRAFE_DIRECTION_INVERT
    strafe = -strafe;
#endif
#if FORWARD_DIRECTION_INVERT
    forward = -forward;
#endif
#if TURN_DIRECTION_INVERT
    turn = -turn;
#endif

    /* 高度控制: 右摇杆 Y (PS2 标准帧无独立压力通道, 用摇杆代替) */
    int16_t height_ctrl = ps2_stick_to_control(state->joy_ry);
#if HEIGHT_DIRECTION_INVERT
    height_ctrl = -height_ctrl;
#endif

    if (ctrl_state->balance_mode) {
        /* ====== 平衡模式 ======
         * 左摇杆 → body_rot.x (Roll) / body_rot.z (Pitch)
         * 右摇杆 X → body_rot.y (Yaw)
         * 右摇杆 Y → body_pos.y (高度) */

        /* 摇杆方向取反，使摇杆推的方向 = 机身倾斜方向 */
        ctrl_state->body_rot.x = -(strafe  * BODY_ROTATION_MAX) / 500;  /* Roll */
        ctrl_state->body_rot.y = -(turn    * BODY_ROTATION_MAX) / 500;  /* Yaw */
        ctrl_state->body_rot.z = -(forward * BODY_ROTATION_MAX) / 500;  /* Pitch */

        ctrl_state->body_pos.y = (height_ctrl * BODY_HEIGHT_RANGE_MM) / 500;

        ctrl_state->travel_length.x = 0;
        ctrl_state->travel_length.y = 0;
        ctrl_state->travel_length.z = 0;
    } else {
        /* ====== 正常模式 ====== */

        ctrl_state->travel_length.x =  (forward * TRAVEL_MAX_FORWARD_MM) / 500;
        ctrl_state->travel_length.z = -(strafe  * TRAVEL_MAX_STRAFE_MM)  / 500;
        ctrl_state->travel_length.y =  (turn    * TRAVEL_MAX_TURN_MM)    / 500;

        ctrl_state->body_pos.y = (height_ctrl * BODY_HEIGHT_RANGE_MM) / 500;

        ctrl_state->body_rot.x = 0;
        ctrl_state->body_rot.y = 0;
        ctrl_state->body_rot.z = 0;

        /* ---- 仿生连续变速: 摇杆幅度 → 步态频率 ---- */
        const int16_t period_max = GAIT_PERIOD_MAX_MS;
        const int16_t period_min = GAIT_PERIOD_MIN_MS;
        const int32_t range = (int32_t)period_max - (int32_t)period_min;

        int16_t stick_mag = (forward >= 0) ? forward : -forward;
        int16_t tmp = (strafe >= 0) ? strafe : -strafe;
        if (tmp > stick_mag) stick_mag = tmp;
        tmp = (turn >= 0) ? turn : -turn;
        if (tmp > stick_mag) stick_mag = tmp;

        if (stick_mag <= 5) {
            ctrl_state->speed_control = period_max;
        } else {
            ctrl_state->speed_control = period_max
                - (int16_t)(range * (int32_t)stick_mag / 500);
        }
    }

    /* 抬腿高度边界钳位 */
    if (ctrl_state->leg_lift_height > LIFT_HEIGHT_MAX_MM)
        ctrl_state->leg_lift_height = LIFT_HEIGHT_MAX_MM;
    if (ctrl_state->leg_lift_height < LIFT_HEIGHT_MIN_MM)
        ctrl_state->leg_lift_height = LIFT_HEIGHT_MIN_MM;

#undef BTN_PRESSED
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

/**
 * @file hexapod_hal_pico.c
 * @brief Raspberry Pi Pico硬件抽象层实现
 * @note 舵机通过I2C总线控制两个PCA9685模块
 *       I2C总线仅用于舵机PWM输出
 *       非运动信号（传感器、输入等）通过Pico本地GPIO/ADC/USB处理
 */

#include "hexapod_hal.h"
#include "hexapod_config.h"
#include "hexapod_i2c_protocol.h"
#include "hexapod_core.h"
#include "hexapod_gait.h"
#include "hexapod_crsf.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* Pico SDK头文件 */
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "pico/time.h"

/* ==================== 舵机控制实现 ==================== */

/**
 * @brief 舵机控制批量缓存
 *        运动学计算完成后缓存所有角度，通过pca9685批量输出
 */
typedef struct {
    bool     pending[18];                     // 待输出标志
    uint8_t  servo_ids[18];                   // 舵机ID
    int16_t  angles[18];                      // 角度（0.1度单位）
    uint16_t pulses[18];                      // 预计算脉宽
    uint8_t  count;                           // 待输出数量
} servo_batch_t;

static servo_batch_t g_servo_batch;
static uint8_t  g_last_servo_count = 0;      /* 上次 flush 的舵机数（调试用） */
static bool     g_servo_available   = false; /* 舵机硬件是否可用 */

bool hal_servo_init(void)
{
    /* 清零舵机缓存 */
    memset(&g_servo_batch, 0, sizeof(g_servo_batch));

    /* 初始化PCA9685（I2C初始化 + 两块PCA9685配置） */
    if (!pca9685_init()) {
        g_servo_available = false;
        return false;
    }

    g_servo_available = true;
    return true;
}

bool hal_servo_set_angle(uint8_t servo_id, int16_t angle, uint16_t move_time)
{
    /* move_time 参数预留用于舵机速度控制（平滑过渡）。
     * 当前实现为即时角度设置，move_time 未生效。
     * 未来可通过在多个控制周期内插值角度来实现缓动效果。 */
    (void)move_time;

    if (servo_id >= 18) return false;
    
    g_servo_batch.servo_ids[servo_id] = servo_id;
    g_servo_batch.angles[servo_id] = angle;
    g_servo_batch.pulses[servo_id] = pca9685_angle_to_pulse(angle);
    
    if (!g_servo_batch.pending[servo_id]) {
        g_servo_batch.pending[servo_id] = true;
        g_servo_batch.count++;
    }
    
    return true;
}

bool hal_servo_set_angles(const uint8_t *servo_ids,
                         const int16_t *angles,
                         uint8_t count,
                         uint16_t move_time)
{
    (void)move_time;  /* 预留参数：舵机速度控制（参见 hal_servo_set_angle 注释） */

    if (!servo_ids || !angles || count == 0 || count > 18) {
        return false;
    }
    
    for (uint8_t i = 0; i < count; i++) {
        uint8_t id = servo_ids[i];
        if (id < 18) {
            g_servo_batch.servo_ids[id] = id;
            g_servo_batch.angles[id] = angles[i];
            g_servo_batch.pulses[id] = pca9685_angle_to_pulse(angles[i]);
            
            if (!g_servo_batch.pending[id]) {
                g_servo_batch.pending[id] = true;
                g_servo_batch.count++;
            }
        }
    }
    
    return true;
}

/**
 * @brief 将所有缓存的舵机角度通过I2C批量输出到PCA9685
 *        由主循环在完成角度计算后调用
 *        将同一PCA9685上的通道合并为一次I2C事务，大幅提升效率
 */
void hal_servo_flush(void)
{
    if (!g_servo_available) { g_last_servo_count = 0; return; }
    if (g_servo_batch.count == 0) { g_last_servo_count = 0; return; }

    g_last_servo_count = g_servo_batch.count;  /* 记录调试信息 */

    uint8_t board_cnt = pca9685_get_board_count();

    /* 处理每块 PCA9685 板：板 0→舵机 ID 0~8, 板 1→舵机 ID 9~17 */
    for (uint8_t b = 0; b < board_cnt; b++) {
        uint16_t pulses[16] = {0};
        bool has_pending = false;
        uint8_t id_start = (b == 0) ? 0 : 9;
        uint8_t id_end   = (b == 0) ? 8 : 17;

        for (uint8_t id = id_start; id <= id_end; id++) {
            if (g_servo_batch.pending[id]) {
                uint8_t ch = id - id_start;  /* 映射到 PCA9685 通道 0~8 */
                pulses[ch] = g_servo_batch.pulses[id];
                g_servo_batch.pending[id] = false;
                has_pending = true;
            }
        }

        if (has_pending) {
            pca9685_write_all_channels(pca9685_get_board_addr(b), pulses, 16);
        }
    }
    
    g_servo_batch.count = 0;
}

void hal_servo_free_all(void)
{
    memset(&g_servo_batch, 0, sizeof(g_servo_batch));
    if (g_servo_available) {
        pca9685_free_all();
    }
}

bool hal_servo_is_movement_done(void)
{
    return g_servo_batch.count == 0;
}

uint8_t hal_get_servo_id(leg_index_t leg_index, uint8_t joint)
{
    if (leg_index >= CNT_LEGS || joint >= 3) return 0;
    
    /* 舵机ID布局：
     *   0~2: RR, 3~5: RM, 6~8: RF  -> PCA9685 #1 (0x40)
     *   9~11: LR, 12~14: LM, 15~17: LF -> PCA9685 #2 (0x41)
     */
    return leg_index * 3 + joint;
}

/* ==================== 定时器实现 ==================== */

uint32_t hal_get_tick_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

void hal_delay_ms(uint32_t ms)
{
    sleep_ms(ms);
}

/* ==================== 电源管理实现（本地ADC，不走I2C） ==================== */

#define BATTERY_ADC_PIN        26
#define BATTERY_ADC_INPUT      0
#define ADC_REF_VOLTAGE        3300
#define ADC_RESOLUTION         4095
#define VOLTAGE_DIVIDER_RATIO  2.0f

static bool adc_initialized = false;

uint16_t hal_get_battery_voltage(void)
{
    if (!adc_initialized) {
        adc_init();
        adc_gpio_init(BATTERY_ADC_PIN);
        adc_select_input(BATTERY_ADC_INPUT);
        adc_initialized = true;
    }
    
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += adc_read();
        sleep_us(10);
    }
    uint16_t adc_val = (uint16_t)(sum / 8);
    
    uint32_t voltage_mv = (uint32_t)((float)adc_val / ADC_RESOLUTION * ADC_REF_VOLTAGE * VOLTAGE_DIVIDER_RATIO);
    return (uint16_t)voltage_mv;
}

bool hal_check_battery(void)
{
    uint16_t voltage = hal_get_battery_voltage();
    return (voltage >= MIN_VOLTAGE_MV);
}

/* ==================== 输入设备实现（UART1串口，不走I2C） ==================== */

#define INPUT_UART_ID           uart1
#define INPUT_UART_TX_PIN       4
#define INPUT_UART_RX_PIN       5
#define INPUT_UART_BAUD_SERIAL  115200
#define INPUT_UART_BAUD_CRSF    420000

#define INPUT_BUF_SIZE          64
static uint8_t input_rx_buf[INPUT_BUF_SIZE];
static volatile uint8_t input_rx_head = 0;
static volatile uint8_t input_rx_tail = 0;

/* CRSF 解析器和状态（全局） */
static crsf_parser_t g_crsf_parser;
static crsf_state_t  g_crsf_state;
static bool g_crsf_mode = false;           // true=CRSF模式, false=串口命令模式
static uint32_t g_last_crsf_frame_ms = 0;  // 最后CRSF帧时间
static uint32_t g_last_crsf_frame_count = 0; // 上次查询时的帧计数（调试用）

/* ==================== 舵机校准模式 ==================== */
static bool     g_calib_active = false;
static uint8_t  g_calib_servo = 0;
static int16_t  g_calib_angle = 900;
static int16_t  g_calib_best[18];
static bool     g_calib_done[18];

static const char *g_calib_leg_name[6]   = {"RR","RM","RF","LR","LM","LF"};
static const char *g_calib_joint_name[3] = {"coxa","femur","tibia"};

/* 将当前校准角度写入选中舵机 */
static void calib_apply(void)
{
    uint16_t pulse = pca9685_angle_to_pulse(g_calib_angle);
    pca9685_set_servo_pulse(g_calib_servo, pulse);
}

/* 打印当前校准状态 */
static void calib_print_status(void)
{
    uint8_t leg   = g_calib_servo / 3;
    uint8_t joint = g_calib_servo % 3;
    hal_debug_printf("[CAL] Servo %u (%s %s) angle=%d | "
                     "!+/- fine !++/-- coarse !N next !S save !D dump !Q quit\r\n",
                     g_calib_servo, g_calib_leg_name[leg],
                     g_calib_joint_name[joint], g_calib_angle);
}

/* 进入校准模式 */
static void calib_enter(uint8_t start_servo, control_state_t *ctrl_state)
{
    if (ctrl_state) ctrl_state->robot_on = false;

    memset(g_calib_best, 0, sizeof(g_calib_best));
    memset(g_calib_done, 0, sizeof(g_calib_done));

    g_calib_active = true;
    g_calib_servo  = (start_servo < 18) ? start_servo : 0;
    g_calib_angle  = 900;

    /* 所有舵机回中位 */
    uint16_t pulse = pca9685_angle_to_pulse(900);
    for (uint8_t i = 0; i < 18; i++) {
        pca9685_set_servo_pulse(i, pulse);
    }

    hal_debug_printf("\r\n");
    hal_debug_printf("╔══════════════════════════════╗\r\n");
    hal_debug_printf("║   SERVO CALIBRATION MODE    ║\r\n");
    hal_debug_printf("║   !Q quit  !D dump offsets  ║\r\n");
    hal_debug_printf("╚══════════════════════════════╝\r\n");
    calib_print_status();
}

/* 保存当前舵机的最佳角度 */
static void calib_save(void)
{
    g_calib_best[g_calib_servo]  = g_calib_angle;
    g_calib_done[g_calib_servo]  = true;

    int16_t offset = g_calib_angle - 900;
    uint8_t leg   = g_calib_servo / 3;
    uint8_t joint = g_calib_servo % 3;
    hal_debug_printf("[CAL] ✓ Saved: %s %s best=%d → horn_offset=%d\r\n",
                     g_calib_leg_name[leg], g_calib_joint_name[joint],
                     g_calib_angle, offset);
}

/* 保存并切换到下一个舵机 */
static void calib_next(void)
{
    calib_save();

    if (g_calib_servo < 17) {
        g_calib_servo++;
        g_calib_angle = 900;
        calib_apply();
    } else {
        hal_debug_printf("[CAL] ★ All 18 servos cycled! Use !D to dump, !Q to quit.\r\n");
    }
    calib_print_status();
}

/* 打印所有校准结果（可直接复制到 hexapod_config.h） */
static void calib_dump(void)
{
    uint8_t done_cnt = 0;
    for (uint8_t i = 0; i < 18; i++) { if (g_calib_done[i]) done_cnt++; }

    hal_debug_printf("\r\n");
    hal_debug_printf("╔══════════════════════════════════════╗\r\n");
    hal_debug_printf("║  HORN OFFSET RESULTS (%u/18 done)    ║\r\n", done_cnt);
    hal_debug_printf("║  Copy into hexapod_config.h          ║\r\n");
    hal_debug_printf("╚══════════════════════════════════════╝\r\n");

    for (uint8_t leg = 0; leg < 6; leg++) {
        hal_debug_printf("\r\n  /* %s leg */\r\n", g_calib_leg_name[leg]);
        for (uint8_t j = 0; j < 3; j++) {
            uint8_t id = leg * 3 + j;
            int16_t offset = g_calib_best[id] - 900;
            char marker = g_calib_done[id] ? ' ' : '?';  /* ? = 未校准 */
            hal_debug_printf("  configs[LEG_%s].%s_horn_offset = %d;%c\r\n",
                             g_calib_leg_name[leg], g_calib_joint_name[j],
                             offset, marker);
        }
    }
    hal_debug_printf("\r\n");
}

/* 退出校准模式 */
static void calib_quit(void)
{
    calib_dump();

    uint16_t pulse = pca9685_angle_to_pulse(900);
    for (uint8_t i = 0; i < 18; i++) {
        pca9685_set_servo_pulse(i, pulse);
    }

    g_calib_active = false;
    hal_debug_printf("[CAL] Calibration exited. All servos → 900.\r\n");
}

static void input_uart_irq_handler(void)
{
    while (uart_is_readable(INPUT_UART_ID)) {
        uint8_t byte = uart_getc(INPUT_UART_ID);
        uint8_t next = (input_rx_head + 1) % INPUT_BUF_SIZE;
        if (next != input_rx_tail) {
            input_rx_buf[input_rx_head] = byte;
            input_rx_head = next;
        }
    }
}

/* CRSF UART IRQ 处理（更快的处理，直接在 IRQ 中解析 CRSF 帧） */
static void crsf_uart_irq_handler(void)
{
    uint32_t now = to_ms_since_boot(get_absolute_time());
    while (uart_is_readable(INPUT_UART_ID)) {
        uint8_t byte = uart_getc(INPUT_UART_ID);
        /* 直接在 IRQ 里解析 CRSF，减少丢失风险 */
        crsf_parse_byte(&g_crsf_parser, byte, now, &g_crsf_state);
    }
}

bool hal_input_init(input_type_t type)
{
    static bool init_done = false;
    static input_type_t current_type = INPUT_TYPE_CRSF;
    
    /* 如果是首次初始化，或者模式切换了，需要重新配置 */
    if (init_done && type == current_type) {
        return true;  /* 同模式重复调用，直接返回 */
    }
    
    /* 如果之前已初始化且模式不同，先停用UART中断 */
    if (init_done) {
        uart_set_irq_enables(INPUT_UART_ID, false, false);
        irq_set_enabled(UART1_IRQ, false);
        uart_deinit(INPUT_UART_ID);
    }
    
    g_crsf_mode = (type == INPUT_TYPE_CRSF);
    current_type = type;
    
    /* 初始化/重置 CRSF 解析器 */
    crsf_parser_init(&g_crsf_parser);
    crsf_state_init(&g_crsf_state);
    
    if (g_crsf_mode) {
        /* CRSF 模式：420000 baud */
        uart_init(INPUT_UART_ID, INPUT_UART_BAUD_CRSF);
        gpio_set_function(INPUT_UART_TX_PIN, GPIO_FUNC_UART);
        gpio_set_function(INPUT_UART_RX_PIN, GPIO_FUNC_UART);
        
        /* 设置 UART 为 8N1（CRSF 使用 8 数据位） */
        uart_set_format(INPUT_UART_ID, 8, 1, UART_PARITY_NONE);
        
        irq_set_exclusive_handler(UART1_IRQ, crsf_uart_irq_handler);
        irq_set_enabled(UART1_IRQ, true);
        uart_set_irq_enables(INPUT_UART_ID, true, false);
        
        hal_debug_printf("CRSF input initialized (420000 baud)\r\n");
    } else {
        /* 串口命令模式：115200 baud */
        uart_init(INPUT_UART_ID, INPUT_UART_BAUD_SERIAL);
        gpio_set_function(INPUT_UART_TX_PIN, GPIO_FUNC_UART);
        gpio_set_function(INPUT_UART_RX_PIN, GPIO_FUNC_UART);
        
        irq_set_exclusive_handler(UART1_IRQ, input_uart_irq_handler);
        irq_set_enabled(UART1_IRQ, true);
        uart_set_irq_enables(INPUT_UART_ID, true, false);
        
        /* 清空串口接收缓冲 */
        input_rx_head = 0;
        input_rx_tail = 0;
        
        hal_debug_printf("Serial input initialized (115200 baud)\r\n");
    }
    
    init_done = true;
    return true;
}

/**
 * @brief 简单串口命令协议 (USB CDC + UART1 双通道)
 *
 * 通过 Pico USB 虚拟串口 (CDC) 直接发送命令，无需额外硬件。
 * USB CDC 命令在 CRSF 和串口命令两种输入模式下均可用。
 *
 * 运动控制:
 *   !F 前进  !B 后退  !L 左移  !R 右移  !Q 左转  !E 右转  !S 停止
 * 状态控制:
 *   !O 解锁/锁定  !G<n> 步态  !U/D 抬腿高度  !T 平衡  !V 调试等级
 * 舵机校准:
 *   !C[<id>] 进入校准模式  !+/- 精调  !++/-- 粗调
 *   !N 保存+下一个  !S 保存  !D dump offset  !Q 退出校准
 * 舵机直控 (排查硬件):
 *   !P<id> <angle>   设置单个舵机 (例: !P0 900 → 舵机0 到90度)
 *   !W<id>           舵机扫摆测试 (例: !W0 → 舵机0 来回扫摆)
 *   !Z               所有舵机回中位 (900=90度)
 *   !A               打印 18 路舵机当前角度
 */
static int16_t parse_int(const uint8_t *buf, uint8_t start, uint8_t len)
{
    int16_t val = 0;
    bool neg = false;
    if (start < len && buf[start] == '-') { neg = true; start++; }
    for (uint8_t i = start; i < len && buf[i] >= '0' && buf[i] <= '9'; i++) {
        val = val * 10 + (buf[i] - '0');
    }
    return neg ? -val : val;
}

/**
 * @brief 处理校准模式下的按键命令
 * @return true 表示命令被校准模式消费
 */
static bool calib_handle_command(uint8_t *buf, uint8_t len)
{
    switch (buf[1]) {
        case '+':
            if (len >= 3 && buf[2] == '+') {
                /* !++ 大步进 +20 (2°) */
                g_calib_angle += 20;
            } else {
                /* !+ 小步进 +5 (0.5°) */
                g_calib_angle += 5;
            }
            if (g_calib_angle >  1800) g_calib_angle =  1800;
            calib_apply();
            calib_print_status();
            return true;

        case '-':
            if (len >= 3 && buf[2] == '-') {
                /* !-- 大步进 -20 (2°) */
                g_calib_angle -= 20;
            } else {
                /* !- 小步进 -5 (0.5°) */
                g_calib_angle -= 5;
            }
            if (g_calib_angle < -1800) g_calib_angle = -1800;
            calib_apply();
            calib_print_status();
            return true;

        case 'N': case 'n':
            calib_next();
            return true;

        case 'S': case 's':
            calib_save();
            calib_print_status();
            return true;

        case 'D': case 'd':
            calib_dump();
            return true;

        case 'Q': case 'q':
            calib_quit();
            return true;

        case 'C': case 'c':
            /* !C 或 !C<id> — 重新选择起始舵机 */
            {
                uint8_t sid = (len >= 3) ? (uint8_t)parse_int(buf, 2, len) : g_calib_servo;
                calib_enter(sid, NULL);
            }
            return true;

        default:
            /* 校准模式下忽略其他命令 */
            hal_debug_printf("[CAL] Unknown. Use: !+ !- !++ !-- !N !S !D !Q\r\n");
            return true;
    }
}

static bool parse_serial_command(control_state_t *ctrl_state, uint8_t *buf, uint8_t len)
{
    if (len < 2 || buf[0] != '!') return false;

    /* ---- 校准模式命令路由 ---- */
    if (g_calib_active) {
        return calib_handle_command(buf, len);
    }

    /* ---- 进入校准模式 (!C 或 !C<id>) ---- */
    if (buf[1] == 'C' || buf[1] == 'c') {
        uint8_t sid = (len >= 3) ? (uint8_t)parse_int(buf, 2, len) : 0;
        calib_enter(sid, ctrl_state);
        return true;
    }

    switch (buf[1]) {
        /* ---- 运动控制 ---- */
        case 'F': ctrl_state->travel_length.x = 60;  ctrl_state->travel_length.z = 0;  ctrl_state->travel_length.y = 0;  break;
        case 'B': ctrl_state->travel_length.x = -60; ctrl_state->travel_length.z = 0;  ctrl_state->travel_length.y = 0;  break;
        case 'L': ctrl_state->travel_length.z = 40;  break;
        case 'R': ctrl_state->travel_length.z = -40; break;
        case 'Q': ctrl_state->travel_length.y = 40;  break;
        case 'E': ctrl_state->travel_length.y = -40; break;
        case 'S': ctrl_state->travel_length.x = 0;   ctrl_state->travel_length.y = 0;   ctrl_state->travel_length.z = 0;  break;

        /* ---- 状态控制 ---- */
        case 'O':
            ctrl_state->robot_on = !ctrl_state->robot_on;
            hal_debug_printf("Robot %s\r\n", ctrl_state->robot_on ? "ON" : "OFF");
            break;
        case 'G':
            if (len >= 3 && buf[2] >= '0' && buf[2] < ('0' + GAIT_MAX)) {
                ctrl_state->gait_type = buf[2] - '0';
                hexapod_gait_select((gait_type_t)ctrl_state->gait_type, ctrl_state);
                hal_debug_printf("Gait: %d\r\n", ctrl_state->gait_type);
            }
            break;
        case 'U': ctrl_state->leg_lift_height += 5; if (ctrl_state->leg_lift_height > 100) ctrl_state->leg_lift_height = 100; break;
        case 'D': ctrl_state->leg_lift_height -= 5; if (ctrl_state->leg_lift_height < 20)  ctrl_state->leg_lift_height = 20;  break;
        case 'T': ctrl_state->balance_mode = !ctrl_state->balance_mode; hal_debug_printf("Balance: %d\r\n", ctrl_state->balance_mode); break;
        case 'V': { uint8_t lvl = hal_debug_get_level(); lvl = (lvl + 1) % 4; hal_debug_set_level(lvl); } break;

        /* ---- 舵机直控 (硬件排查) ---- */
        case 'P': {
            /* !P<servo_id> <angle>  例: !P0 900  */
            if (len < 5) { hal_debug_printf("Usage: !P<id> <angle>\r\n"); break; }
            uint8_t sid = (uint8_t)parse_int(buf, 2, len);
            /* 跳过 id 找到空格后的 angle */
            uint8_t pos = 2; while (pos < len && buf[pos] != ' ') pos++;
            int16_t ang = parse_int(buf, pos + 1, len);
            if (sid < 18 && ang >= -1800 && ang <= 1800) {
                /* 直接写入舵机，绕过 IK/步态 */
                uint16_t pulse = pca9685_angle_to_pulse(ang);
                uint8_t addr;
                if (sid <= 8) addr = pca9685_get_board_addr(0);
                else          addr = pca9685_get_board_addr(1);
                if (addr) {
                    pca9685_set_servo_pulse(sid, pulse);
                    hal_debug_printf("Servo %u → angle %d (pulse %u us, addr 0x%02X)\r\n", sid, ang, pulse, addr);
                } else {
                    hal_debug_printf("Servo %u: board not present\r\n", sid);
                }
            } else {
                hal_debug_printf("Invalid: id=%u angle=%d\r\n", sid, ang);
            }
            break;
        }
        case 'W': {
            /* !W<servo_id> 扫摆测试 */
            if (len < 3) { hal_debug_printf("Usage: !W<id>\r\n"); break; }
            uint8_t sid = (uint8_t)parse_int(buf, 2, len);
            if (sid >= 18) { hal_debug_printf("Invalid servo id\r\n"); break; }
            hal_debug_printf("Sweep test servo %u: 300→1500→300\r\n", sid);
            for (int16_t a = 300; a <= 1500; a += 50) {
                pca9685_set_servo_pulse(sid, pca9685_angle_to_pulse(a));
                sleep_ms(80);
            }
            for (int16_t a = 1500; a >= 300; a -= 50) {
                pca9685_set_servo_pulse(sid, pca9685_angle_to_pulse(a));
                sleep_ms(80);
            }
            pca9685_set_servo_pulse(sid, pca9685_angle_to_pulse(0));
            hal_debug_printf("Sweep test done\r\n");
            break;
        }
        case 'Z': {
            /* 所有舵机回中位 (900 = 90°) */
            hal_debug_printf("All servos → 90° (900)\r\n");
            uint16_t pulse = pca9685_angle_to_pulse(900);
            for (uint8_t i = 0; i < 18; i++) {
                pca9685_set_servo_pulse(i, pulse);
            }
            break;
        }
        case 'A': {
            /* 打印所有舵机当前角度（从 servo batch 缓存读取） */
            hal_debug_printf("=== Servo Snapshot ===\r\n");
            hal_debug_printf("Board 0 (0x%02X): ", pca9685_get_board_addr(0));
            for (uint8_t i = 0; i <= 8; i++) {
                hal_debug_printf("[%u]:%d ", i, g_servo_batch.angles[i]);
            }
            hal_debug_printf("\r\n");
            if (pca9685_get_board_count() >= 2) {
                hal_debug_printf("Board 1 (0x%02X): ", pca9685_get_board_addr(1));
                for (uint8_t i = 9; i < 18; i++) {
                    hal_debug_printf("[%u]:%d ", i, g_servo_batch.angles[i]);
                }
                hal_debug_printf("\r\n");
            }
            break;
        }
        default:
            return false;
    }

    return true;
}

bool hal_input_update(control_state_t *ctrl_state)
{
    if (!ctrl_state) return false;

    /* ---- USB CDC 串口命令 (始终可用，独立于输入模式) ----
     * 通过 Pico 的 USB 虚拟串口接收命令，无需额外硬件。
     * 非阻塞轮询：getchar_timeout_us(0) 无数据立即返回。
     * 无论在 CRSF 还是串口命令模式下，USB 命令都可用。 */
    {
        static uint8_t usb_buf[16];
        static uint8_t usb_len = 0;

        int ch = getchar_timeout_us(0);
        while (ch != PICO_ERROR_TIMEOUT) {
            if (ch == '\n' || ch == '\r') {
                if (usb_len > 0) {
                    parse_serial_command(ctrl_state, usb_buf, usb_len);
                    usb_len = 0;
                }
            } else if (usb_len < sizeof(usb_buf)) {
                usb_buf[usb_len++] = (uint8_t)ch;
            }
            ch = getchar_timeout_us(0);
        }
    }

    if (g_crsf_mode) {
        /* ========== CRSF 模式 ========== */
        uint32_t now = hal_get_tick_ms();

        /* 检查是否有新帧 - 使用 last_frame_time_ms 检测 */
        uint32_t last_frame_time = g_crsf_state.last_frame_time_ms;
        if (last_frame_time != g_last_crsf_frame_ms) {
            g_last_crsf_frame_ms = last_frame_time;

            /* 检查链接状态 */
            if (g_crsf_state.link_connected) {
                /* 将 CRSF 状态映射到机器人控制 */
                crsf_to_control(&g_crsf_state, ctrl_state);
                return true;
            }
        }

        /* 链接断开处理：如果超过200ms无数据，自动停止 */
        if (g_last_crsf_frame_ms > 0 && (now - g_last_crsf_frame_ms > 200)) {
            if (ctrl_state->robot_on) {
                ctrl_state->travel_length.x = 0;
                ctrl_state->travel_length.y = 0;
                ctrl_state->travel_length.z = 0;
            }
        }

        return false;
    } else {
        /* ========== 串口命令模式 ========== */
        uint8_t cmd_buf[16];
        uint8_t cmd_len = 0;
        bool cmd_ready = false;
        
        while (input_rx_head != input_rx_tail) {
            uint8_t byte = input_rx_buf[input_rx_tail];
            input_rx_tail = (input_rx_tail + 1) % INPUT_BUF_SIZE;
            
            if (byte == '\n' || byte == '\r') {
                if (cmd_len > 0) {
                    cmd_ready = true;
                    break;
                }
            } else if (cmd_len < sizeof(cmd_buf)) {
                cmd_buf[cmd_len++] = byte;
            }
        }
        
        if (cmd_ready) {
            return parse_serial_command(ctrl_state, cmd_buf, cmd_len);
        }
    }
    
    return false;
}

void hal_input_allow_interrupts(bool allow)
{
    static uint32_t irq_stack[8];
    static uint8_t irq_depth = 0;

    if (allow) {
        if (irq_depth > 0) {
            irq_depth--;
            restore_interrupts(irq_stack[irq_depth]);
        }
        /* irq_depth == 0 表示没有匹配的 disable 调用，安全忽略 */
    } else {
        if (irq_depth < 8) {
            irq_stack[irq_depth++] = save_and_disable_interrupts();
        }
        /* irq_depth >= 8 时：栈满，静默丢弃本次 disable 请求。
         * 这意味着在极深的嵌套调用场景下，最内层的中断状态不会被保存。
         * 正常使用不应超过 8 层嵌套。若需更高深度，请增大 irq_stack 数组。 */
    }
}

/* ==================== 调试输出实现 ==================== */

void hal_debug_init(void)
{
    /* USB CDC已在stdio_init_all中初始化 */
}

void hal_debug_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

/* ==================== 蜂鸣器实现（GPIO 高低电平驱动，本地 GPIO，不走 I2C） ==================== */

#define BUZZER_PIN              15

void hal_play_sound(uint8_t note_count,
                   const uint16_t *notes,
                   const uint16_t *durations)
{
    if (!notes || !durations || note_count == 0) return;

    /* 初始化蜂鸣器 GPIO 为推挽输出 */
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    gpio_put(BUZZER_PIN, 0);

    for (uint8_t i = 0; i < note_count && i < 10; i++) {
        if (notes[i] > 0) {
            gpio_put(BUZZER_PIN, 1);
            sleep_ms(durations[i]);
        }
        gpio_put(BUZZER_PIN, 0);
        sleep_ms(50);
    }
}

/* ==================== LED实现（本地GPIO，不走I2C） ==================== */

#define LED_BUILTIN             25

static bool led_initialized = false;

void hal_led_set(uint8_t led_id, bool state)
{
    /* led_id 参数预留用于多 LED 扩展；当前仅支持板载 LED (GPIO 25) */
    (void)led_id;

    if (!led_initialized) {
        gpio_init(LED_BUILTIN);
        gpio_set_dir(LED_BUILTIN, GPIO_OUT);
        led_initialized = true;
    }
    
    gpio_put(LED_BUILTIN, state);
}

void hal_led_blink(uint8_t led_id, uint8_t times)
{
    for (uint8_t i = 0; i < times; i++) {
        hal_led_set(led_id, true);
        sleep_ms(100);
        hal_led_set(led_id, false);
        sleep_ms(100);
    }
}

/* ==================== 调试辅助接口实现 ==================== */

static uint8_t g_debug_level = DEBUG_LEVEL;

void hal_debug_set_level(uint8_t level)
{
    if (level > 3) level = 0;
    g_debug_level = level;
    hal_debug_printf("[DBG] Level set to %u\r\n", g_debug_level);
}

uint8_t hal_debug_get_level(void)
{
    return g_debug_level;
}

const void* hal_debug_get_crsf_state(void)
{
    if (!g_crsf_mode) return NULL;
    return &g_crsf_state;
}

uint32_t hal_debug_get_crsf_frame_delta(void)
{
    if (!g_crsf_mode) return 0;
    uint32_t current = g_crsf_state.frame_count;
    uint32_t delta = current - g_last_crsf_frame_count;
    g_last_crsf_frame_count = current;
    return delta;
}

uint8_t hal_debug_get_last_servo_count(void)
{
    return g_last_servo_count;
}

/* ==================== 校准模式接口 ==================== */

bool hal_is_calibration_active(void)
{
    return g_calib_active;
}

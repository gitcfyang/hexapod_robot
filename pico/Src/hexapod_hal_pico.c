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

bool hal_servo_init(void)
{
    /* 清零舵机缓存 */
    memset(&g_servo_batch, 0, sizeof(g_servo_batch));
    
    /* 初始化PCA9685（I2C初始化 + 两块PCA9685配置） */
    if (!pca9685_init()) {
        return false;
    }
    
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
    if (g_servo_batch.count == 0) { g_last_servo_count = 0; return; }

    g_last_servo_count = g_servo_batch.count;  /* 记录调试信息 */
    
    /* 收集第一片PCA9685 (0x40) 的脉宽：舵机ID 0~8 */
    uint16_t pulses_pca1[16] = {0};
    bool has_pca1 = false;
    for (uint8_t i = 0; i <= 8; i++) {
        if (g_servo_batch.pending[i]) {
            pulses_pca1[i] = g_servo_batch.pulses[i];
            g_servo_batch.pending[i] = false;
            has_pca1 = true;
        }
    }
    
    /* 收集第二片PCA9685 (0x41) 的脉宽：舵机ID 9~17 */
    uint16_t pulses_pca2[16] = {0};
    bool has_pca2 = false;
    for (uint8_t i = 9; i < 18; i++) {
        if (g_servo_batch.pending[i]) {
            uint8_t ch = i - 9;  // 映射到PCA9685通道0~8
            pulses_pca2[ch] = g_servo_batch.pulses[i];
            g_servo_batch.pending[i] = false;
            has_pca2 = true;
        }
    }
    
    /* 批量写入（各1次I2C事务） */
    if (has_pca1) {
        pca9685_write_all_channels(PCA9685_ADDR_0x40, pulses_pca1, 16);
    }
    if (has_pca2) {
        pca9685_write_all_channels(PCA9685_ADDR_0x41, pulses_pca2, 16);
    }
    
    g_servo_batch.count = 0;
}

void hal_servo_free_all(void)
{
    memset(&g_servo_batch, 0, sizeof(g_servo_batch));
    pca9685_free_all();
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
 * @brief 简单串口命令协议
 * 格式: !<CMD>[PARAM]\n
 *   !F 前进 | !B 后退 | !L 左移 | !R 右移
 *   !Q 左转 | !E 右转 | !S 停止
 *   !O 开关 | !G<n> 步态 | !U 高度+ | !D 高度- | !T 平衡
 */
static bool parse_serial_command(control_state_t *ctrl_state, uint8_t *buf, uint8_t len)
{
    if (len < 2 || buf[0] != '!') return false;
    
    switch (buf[1]) {
        case 'F':
            ctrl_state->travel_length.x = 30;
            ctrl_state->travel_length.z = 0;
            ctrl_state->travel_length.y = 0;
            break;
        case 'B':
            ctrl_state->travel_length.x = -30;
            ctrl_state->travel_length.z = 0;
            ctrl_state->travel_length.y = 0;
            break;
        case 'L':
            ctrl_state->travel_length.z = 20;
            break;
        case 'R':
            ctrl_state->travel_length.z = -20;
            break;
        case 'Q':
            ctrl_state->travel_length.y = 30;
            break;
        case 'E':
            ctrl_state->travel_length.y = -30;
            break;
        case 'S':
            ctrl_state->travel_length.x = 0;
            ctrl_state->travel_length.y = 0;
            ctrl_state->travel_length.z = 0;
            break;
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
        case 'U':
            ctrl_state->leg_lift_height += 5;
            if (ctrl_state->leg_lift_height > 100) ctrl_state->leg_lift_height = 100;
            break;
        case 'D':
            ctrl_state->leg_lift_height -= 5;
            if (ctrl_state->leg_lift_height < 20) ctrl_state->leg_lift_height = 20;
            break;
        case 'T':
            ctrl_state->balance_mode = !ctrl_state->balance_mode;
            hal_debug_printf("Balance: %d\r\n", ctrl_state->balance_mode);
            break;
        case 'V':
            /* 切换调试等级: 0→1→2→3→0 */
            {
                uint8_t lvl = hal_debug_get_level();
                lvl = (lvl + 1) % 4;
                hal_debug_set_level(lvl);
            }
            break;
        default:
            return false;
    }
    
    return true;
}

bool hal_input_update(control_state_t *ctrl_state)
{
    if (!ctrl_state) return false;
    
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

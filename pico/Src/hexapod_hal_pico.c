/**
 * @file hexapod_hal_pico.c
 * @brief Raspberry Pi Pico硬件抽象层实现
 * @note 舵机通过I2C总线控制两个PCA9685模块
 *       I2C总线仅用于舵机PWM输出
 *       非运动信号（传感器、输入等）通过Pico本地GPIO/ADC/USB处理
 */

#include "hexapod_hal.h"
#include "hexapod_config.h"
#include "bno055.h"
#include "hexapod_i2c_protocol.h"
#include "hexapod_core.h"
#include "hexapod_gait.h"
#include "hexapod_crsf.h"
#if PS2_ENABLED
#include "hexapod_ps2.h"
#endif
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
#include "hardware/pwm.h"
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
    static uint32_t i2c_fail_total = 0;
    static uint8_t  i2c_fail_consecutive = 0;
    bool all_ok = true;

    /* 处理每块 PCA9685 板 (PCB 定版, 按 I2C 地址区分):
     *   0x40 = 左侧腿组 (舵机 ID 9~17)
     *   0x41 = 右侧腿组 (舵机 ID 0~8) */
    for (uint8_t b = 0; b < board_cnt; b++) {
        uint16_t pulses[16] = {0};
        bool has_pending = false;
        uint8_t addr = pca9685_get_board_addr(b);
        uint8_t id_start = (addr == PCA9685_ADDR_LEFT) ? 9 : 0;
        uint8_t id_end   = id_start + 8;

        for (uint8_t id = id_start; id <= id_end; id++) {
            if (g_servo_batch.pending[id]) {
                uint8_t ch = pca9685_servo_to_channel(id);  /* PCB 定版物理通道 */
                pulses[ch] = g_servo_batch.pulses[id];
                has_pending = true;
            }
        }

        if (has_pending) {
            if (pca9685_write_all_channels(pca9685_get_board_addr(b), pulses, 16)) {
                /* 写入成功 → 清除 pending */
                for (uint8_t id = id_start; id <= id_end; id++) {
                    g_servo_batch.pending[id] = false;
                }
            } else {
                /* 写入失败 → 保留 pending, 下周期重试 */
                all_ok = false;
            }
        }
    }

    if (all_ok) {
        g_servo_batch.count = 0;
        i2c_fail_consecutive = 0;  /* 成功则清零连续失败计数 */
    } else {
        i2c_fail_total++;
        i2c_fail_consecutive++;

        /* 每 50 次失败告警一次 (约 1 秒) */
        if (i2c_fail_total % 50 == 1) {
            hal_debug_printf("[WARN] I2C flush fail total=%lu consec=%u\r\n",
                             i2c_fail_total, i2c_fail_consecutive);
        }

        /* 连续失败 3 次 (约 60ms) → 执行 I2C 总线恢复
         * 标准恢复流程：发送 9 个 SCL 脉冲释放被卡死的 SDA，
         * 然后重新初始化 I2C 外设。整个过程约 1-2ms。 */
        if (i2c_fail_consecutive >= 3) {
            hal_debug_printf("[WARN] I2C consecutive fail %u, recovering bus...\r\n",
                             i2c_fail_consecutive);
            pca9685_i2c_recover();
            i2c_fail_consecutive = 0;
        }
    }
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

/*
 * 电池检测 (2S 18650, 新 PCB 2026-08: GP28/ADC2, 原 GP26 已因尖峰损坏):
 *   分压: R1=330kΩ (电池+), R2=47kΩ (地), ADC 抽头在 R2
 *   (330k 高串阻限制尖峰电流入 ADC, 与齐纳钳位配合更安全)
 *   ADC 电压 = 电池 × 47/377 → 电池 = ADC × 377/47 ≈ 8.02×
 */

static bool adc_initialized = false;

uint16_t hal_get_battery_voltage(void)
{
    if (!adc_initialized) {
        adc_init();
        adc_gpio_init(BATTERY_ADC_PIN);
        adc_select_input(BATTERY_ADC_INPUT);
        adc_initialized = true;
    }

    /* 8 次采样平均, 抑制舵机 EMI 尖峰 */
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += adc_read();
        sleep_us(10);
    }
    uint16_t adc_val = (uint16_t)(sum / 8);

    uint32_t voltage_mv = (uint32_t)((float)adc_val / ADC_RESOLUTION
                                     * ADC_REF_VOLTAGE * BATTERY_DIVIDER_RATIO);
    return (uint16_t)voltage_mv;
}

bool hal_check_battery(void)
{
    uint16_t voltage = hal_get_battery_voltage();
    /* 电压必须在安全窗口内: 低压截止 ~ 过压保护 */
    return (voltage >= BATTERY_CUTOFF_MV && voltage <= BATTERY_OVERVOLTAGE_MV);
}

/* ==================== 舵机供电控制实现 (GP10/GP11) ==================== */

/*
 * 舵机供电控制:
 *   GP10 = 左侧舵机供电 (PCA9685 0x40, 左腿组)
 *   GP11 = 右侧舵机供电 (PCA9685 0x41, 右腿组)
 *   高电平 = 供电, 低电平 = 断电
 *
 * 安全设计:
 *   - 上电默认断电 (低电平)
 *   - 电池电压检测通过后才允许供电
 *   - 电压异常时自动断开, 防止电池过放
 */

#define SERVO_PWR_LEFT_PIN   10
#define SERVO_PWR_RIGHT_PIN  11
#define SERVO_PWR_ACTIVE     1      /* 高电平有效 (N-MOS 高端或继电器) */

static bool servo_pwr_initialized = false;

void hal_servo_power_init(void)
{
    gpio_init(SERVO_PWR_LEFT_PIN);
    gpio_set_dir(SERVO_PWR_LEFT_PIN, GPIO_OUT);
    gpio_put(SERVO_PWR_LEFT_PIN, !SERVO_PWR_ACTIVE);   /* 默认断电 */

    gpio_init(SERVO_PWR_RIGHT_PIN);
    gpio_set_dir(SERVO_PWR_RIGHT_PIN, GPIO_OUT);
    gpio_put(SERVO_PWR_RIGHT_PIN, !SERVO_PWR_ACTIVE);  /* 默认断电 */

    servo_pwr_initialized = true;
}

void hal_servo_power_set(uint8_t side, bool enable)
{
    if (!servo_pwr_initialized) hal_servo_power_init();

    uint8_t pin = (side == 0) ? SERVO_PWR_LEFT_PIN : SERVO_PWR_RIGHT_PIN;
    gpio_put(pin, enable ? SERVO_PWR_ACTIVE : !SERVO_PWR_ACTIVE);
}

void hal_servo_power_set_all(bool enable)
{
    hal_servo_power_set(0, enable);
    hal_servo_power_set(1, enable);
}

/* ==================== 直流电机实现 (GP2/GP3, PWM 调速) ==================== */

/*
 * 直流电机控制:
 *   GP2 = 电机 1 (PWM1A)
 *   GP3 = 电机 2 (PWM1B)
 *   PWM 频率 10kHz, 占空比 0~100% (0=停转, 100%=全速)
 *
 * 说明: 每路仅一个引脚, 单向控制 (无方向引脚)。
 * 未来如需双向, 可配合外部 H 桥的方向引脚扩展。
 */

#define DC_MOTOR1_PIN       2
#define DC_MOTOR2_PIN       3
#define DC_MOTOR_PWM_FREQ   10000   /* 10kHz, 直流电机理想范围 */
#define DC_MOTOR_PWM_WRAP   12500   /* 125MHz / 10kHz = 12500 */

static bool dc_motor_initialized = false;

void hal_dc_motor_init(void)
{
    /* 电机1: GP2 = PWM1A, 电机2: GP3 = PWM1B — 同属 slice 1 */
    gpio_set_function(DC_MOTOR1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(DC_MOTOR2_PIN, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(DC_MOTOR1_PIN);
    pwm_set_wrap(slice, DC_MOTOR_PWM_WRAP - 1);
    pwm_set_chan_level(slice, PWM_CHAN_A, 0);   /* 电机1 停转 */
    pwm_set_chan_level(slice, PWM_CHAN_B, 0);   /* 电机2 停转 */
    pwm_set_enabled(slice, true);

    dc_motor_initialized = true;
}

void hal_dc_motor_set(uint8_t motor, uint16_t duty_percent)
{
    if (!dc_motor_initialized) hal_dc_motor_init();
    if (motor > 1) return;
    if (duty_percent > 1000) duty_percent = 1000;

    /* 占空比 0~1000 → PWM 电平 0~WRAP-1 */
    uint32_t level = (uint32_t)duty_percent * (DC_MOTOR_PWM_WRAP - 1) / 1000;
    uint slice = pwm_gpio_to_slice_num(DC_MOTOR1_PIN);
    pwm_set_chan_level(slice, (motor == 0) ? PWM_CHAN_A : PWM_CHAN_B, level);
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

/* PS2 状态 (PS2_ENABLED) */
#if PS2_ENABLED
ps2_state_t g_ps2_state;          /* 非 static: 供 hexapod_ps2.c extern 引用 */
static bool g_ps2_initialized = false;
static bool g_ps2_debug_active = false;   /* !PS2DBG 调试观察模式: 只打印通道值, 机器人不响应 */

/* 运行时输入模式 (编译期默认由 INPUT_CONTROL_MODE 决定, !MODE 可运行时切换)
 *  0 = INPUT_MODE_CRSF — CRSF
 *  1 = INPUT_MODE_PS2  — PS2 */
#define INPUT_MODE_CRSF    0
#define INPUT_MODE_PS2     1
static uint8_t  g_input_mode = INPUT_MODE_CRSF;
static uint32_t g_mode_switch_time_ms = 0;   /* 进入当前模式的时间 */
#endif

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

    /* 校准直接驱动舵机, 必须先开启舵机供电 */
    hal_servo_power_set_all(true);
    hal_delay_ms(100);

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
#if INPUT_CONTROL_MODE != 2
        uart_set_irq_enables(INPUT_UART_ID, false, false);
        irq_set_enabled(UART1_IRQ, false);
        uart_deinit(INPUT_UART_ID);
#endif
    }

#if INPUT_CONTROL_MODE != 2
    /* 无线电模式 (CRSF/PS2): UART1 恒以 CRSF 运行 —
     * 即使默认输入是 PS2 也保持接收, 保证 !MODE crsf 随时可切换 */
    g_crsf_mode = true;
#else
    g_crsf_mode = false;  /* USB 串口模式：不启用 UART1 */
#endif
    current_type = type;

    /* 初始化/重置 CRSF 解析器 */
    crsf_parser_init(&g_crsf_parser);
    crsf_state_init(&g_crsf_state);

#if INPUT_CONTROL_MODE != 2
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
#else
    /* USB CDC 串口模式：无需 UART1 硬件，USB 虚拟串口由 stdio 提供 */
    hal_debug_printf("USB CDC Serial input mode (no UART1)\r\n");
#endif

#if PS2_ENABLED
    /* PS2 接口初始化: 仅配置 GPIO, 不阻断 CRSF
     * 无论当前是 CRSF/AUTO/PS2 模式都初始化, 便于运行时切换 */
    if (!g_ps2_initialized) {
        ps2_init();
        memset(&g_ps2_state, 0, sizeof(g_ps2_state));
        /* 中位校准初始值 128 (进入模拟模式后自动重新采样) */
        g_ps2_state.center_lx = 128; g_ps2_state.center_ly = 128;
        g_ps2_state.center_rx = 128; g_ps2_state.center_ry = 128;
        g_ps2_initialized = true;
    }

    /* 设置运行时模式 */
    if (type == INPUT_TYPE_PS2) {
        g_input_mode = INPUT_MODE_PS2;
        g_mode_switch_time_ms = hal_get_tick_ms();
        hal_debug_printf("PS2 input mode (default)\r\n");
    } else {
        g_input_mode = INPUT_MODE_CRSF;
    }
#endif

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

    /* 跳过前导空格 (容忍 "!M -1" 与 "!M-1" 两种写法) */
    while (start < len && buf[start] == ' ') start++;

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

/* ==================== I2C 总线检测 (!I2C) ==================== */

/**
 * @brief 检测 I2C 总线上的 PCA9685 和 BNO055
 * @note 独立于机器人初始化, 启动失败/等待状态也可执行
 */
static void i2c_bus_check(void)
{
    /* 确保 I2C 硬件已初始化 (机器人初始化失败时也可检测) */
    i2c_init(PCA9685_I2C_INSTANCE, PCA9685_I2C_BAUD);
    gpio_set_function(PCA9685_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PCA9685_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PCA9685_I2C_SDA_PIN);
    gpio_pull_up(PCA9685_I2C_SCL_PIN);

    hal_debug_printf("=== I2C Bus Check (SDA=GP14, SCL=GP15) ===\r\n");

    /* 舵机供电引脚状态 (诊断 PCA9685 供电极性是否正确) */
    hal_debug_printf("Servo power: GP10(left)=%d  GP11(right)=%d\r\n",
                     gpio_get(SERVO_PWR_LEFT_PIN), gpio_get(SERVO_PWR_RIGHT_PIN));

    /* PCA9685 定向检测: 0x40=左腿板, 0x41=右腿板 (读 MODE1 寄存器) */
    for (uint8_t addr = 0x40; addr <= 0x41; addr++) {
        uint8_t mode1 = 0;
        int ret = i2c_read_blocking_until(PCA9685_I2C_INSTANCE, addr, &mode1, 1, false,
                                          make_timeout_time_us(PCA9685_I2C_TIMEOUT_US));
        if (ret == 1) {
            hal_debug_printf("PCA9685 0x%02X (%s legs): DETECTED  MODE1=0x%02X\r\n",
                             addr, (addr == PCA9685_ADDR_LEFT) ? "left " : "right", mode1);
        } else {
            hal_debug_printf("PCA9685 0x%02X (%s legs): NOT FOUND (ret=%d)\r\n",
                             addr, (addr == PCA9685_ADDR_LEFT) ? "left " : "right", ret);
        }
    }

    /* BNO055 定向检测: 0x28 默认, 0x29 备用 (读 CHIP_ID 寄存器, 期望 0xA0) */
    for (uint8_t addr = 0x28; addr <= 0x29; addr++) {
        uint8_t reg = BNO055_REG_CHIP_ID;
        uint8_t chip_id = 0;
        int ret = i2c_write_blocking_until(PCA9685_I2C_INSTANCE, addr, &reg, 1, true,
                                           make_timeout_time_us(PCA9685_I2C_TIMEOUT_US));
        if (ret == 1) {
            ret = i2c_read_blocking_until(PCA9685_I2C_INSTANCE, addr, &chip_id, 1, false,
                                          make_timeout_time_us(PCA9685_I2C_TIMEOUT_US));
        }
        if (ret == 1 && chip_id == BNO055_CHIP_ID_VAL) {
            hal_debug_printf("BNO055 0x%02X: DETECTED  CHIP_ID=0x%02X\r\n", addr, chip_id);
        } else if (ret == 1) {
            hal_debug_printf("BNO055 0x%02X: present but CHIP_ID=0x%02X (expect 0xA0)\r\n",
                             addr, chip_id);
        } else {
            hal_debug_printf("BNO055 0x%02X: NOT FOUND\r\n", addr);
        }
    }

    /* 全总线扫描 (列出所有 ACK 的设备) */
    pca9685_scan_bus();

    /* 总线空闲电平诊断: 临时切为 GPIO 输入上拉读取, 再恢复 I2C 功能。
     * 0 = 被器件拉低 (器件损坏把总线拖死) 或 GPIO 引脚本身损坏;
     * 1 = 线路空闲正常 → 设备在线但 I2C 模块已死 (需换器件)。 */
    gpio_set_function(PCA9685_I2C_SDA_PIN, GPIO_FUNC_SIO);
    gpio_set_function(PCA9685_I2C_SCL_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(PCA9685_I2C_SDA_PIN, GPIO_IN);
    gpio_set_dir(PCA9685_I2C_SCL_PIN, GPIO_IN);
    gpio_pull_up(PCA9685_I2C_SDA_PIN);
    gpio_pull_up(PCA9685_I2C_SCL_PIN);
    sleep_ms(2);   /* 等待上拉稳定 */
    hal_debug_printf("Bus idle: SDA(GP14)=%d SCL(GP15)=%d (1=空闲正常, 0=被拉低/短路/引脚损坏)\r\n",
                     gpio_get(PCA9685_I2C_SDA_PIN), gpio_get(PCA9685_I2C_SCL_PIN));
    /* 恢复 I2C 引脚功能 */
    gpio_set_function(PCA9685_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PCA9685_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PCA9685_I2C_SDA_PIN);
    gpio_pull_up(PCA9685_I2C_SCL_PIN);
}

/* 前置声明: 周期校准命令处理 (定义在本文件后部) */
static bool period_calib_handle_command(uint8_t *buf, uint8_t len);
/* 前置声明: IMU 状态打印 (定义在本文件后部) */
static void imu_status_print(void);

#if PS2_ENABLED
/* PS2 状态全通道打印 (!PS2 单次输出 / !PS2DBG 观察模式每秒一次, 共用) */
static void ps2_state_print(void)
{
    if (!g_ps2_initialized) {
        hal_debug_printf("[PS2] Not initialized\r\n");
        return;
    }
    const ps2_state_t *s = ps2_get_state();
    if (!s) {
        hal_debug_printf("[PS2] No controller connected\r\n");
        return;
    }
    hal_debug_printf("=== PS2 Controller State ===\r\n");
    hal_debug_printf("ID: 0x%02X (%s)  Buttons: 0x%04X\r\n",
        s->id, s->analog_mode ? "analog" : "digital", s->buttons);
    hal_debug_printf("Pressed:");
    bool any_btn = false;
    if (!(s->buttons & PSB_SELECT))   { hal_debug_printf(" SEL");   any_btn = true; }
    if (!(s->buttons & PSB_L3))       { hal_debug_printf(" L3");    any_btn = true; }
    if (!(s->buttons & PSB_R3))       { hal_debug_printf(" R3");    any_btn = true; }
    if (!(s->buttons & PSB_START))    { hal_debug_printf(" START"); any_btn = true; }
    if (!(s->buttons & PSB_PAD_UP))   { hal_debug_printf(" UP");    any_btn = true; }
    if (!(s->buttons & PSB_PAD_RIGHT)) { hal_debug_printf(" RIGHT"); any_btn = true; }
    if (!(s->buttons & PSB_PAD_DOWN)) { hal_debug_printf(" DOWN");  any_btn = true; }
    if (!(s->buttons & PSB_PAD_LEFT)) { hal_debug_printf(" LEFT");  any_btn = true; }
    if (!(s->buttons & PSB_L2))       { hal_debug_printf(" L2");    any_btn = true; }
    if (!(s->buttons & PSB_R2))       { hal_debug_printf(" R2");    any_btn = true; }
    if (!(s->buttons & PSB_L1))       { hal_debug_printf(" L1");    any_btn = true; }
    if (!(s->buttons & PSB_R1))       { hal_debug_printf(" R1");    any_btn = true; }
    if (!(s->buttons & PSB_TRIANGLE)) { hal_debug_printf(" TRI");   any_btn = true; }
    if (!(s->buttons & PSB_CIRCLE))   { hal_debug_printf(" CIR");   any_btn = true; }
    if (!(s->buttons & PSB_CROSS))    { hal_debug_printf(" X");     any_btn = true; }
    if (!(s->buttons & PSB_SQUARE))   { hal_debug_printf(" SQ");    any_btn = true; }
    if (!any_btn) hal_debug_printf(" (none)");
    hal_debug_printf("\r\n");
    hal_debug_printf("Sticks: LX=%3u LY=%3u RX=%3u RY=%3u  "
        "Frames: %lu  Mode: %s\r\n",
        s->joy_lx, s->joy_ly, s->joy_rx, s->joy_ry,
        s->frame_count,
        g_input_mode == INPUT_MODE_PS2 ? "PS2" : "CRSF");
    hal_debug_printf("Ctr(calib): LX=%u LY=%u RX=%u RY=%u samples=%u\r\n",
        s->center_lx, s->center_ly, s->center_rx, s->center_ry,
        s->center_samples);
}
#endif /* PS2_ENABLED */

static bool parse_serial_command(control_state_t *ctrl_state, uint8_t *buf, uint8_t len)
{
    if (len < 2 || buf[0] != '!') return false;

    /* ---- !I2C: I2C 总线设备检测 (无需控制状态, 启动失败时也可用) ---- */
    if (buf[1] == 'I' && len >= 3 && buf[2] == '2') {
        i2c_bus_check();
        return true;
    }

    /* ---- !IMU: IMU 状态 (欧拉角/中断/校准) ---- */
    if (buf[1] == 'I' && len >= 3 && buf[2] == 'M') {
        imu_status_print();
        return true;
    }

    /* 其余命令需要控制状态 */
    if (!ctrl_state) return false;

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
        case 'M':
#if PS2_ENABLED
            /* ---- !MODE: 输入源运行时切换 (PS2_ENABLED) ----
             * !MODE crsf → 切换到 CRSF 模式
             * !MODE ps2  → 切换到 PS2 模式 */
            if (len >= 3 && buf[2] == 'O') {
                uint8_t p = 5;
                while (p < len && buf[p] == ' ') p++;
                if (p + 3 < len && buf[p] == 'c' && buf[p+1] == 'r'
                    && buf[p+2] == 's' && buf[p+3] == 'f') {
                    g_input_mode = INPUT_MODE_CRSF;
                    g_mode_switch_time_ms = hal_get_tick_ms();
                    hal_debug_printf("[MODE] Switched to CRSF\r\n");
                } else if (p + 2 < len && buf[p] == 'p' && buf[p+1] == 's'
                           && buf[p+2] == '2') {
                    g_input_mode = INPUT_MODE_PS2;
                    g_mode_switch_time_ms = hal_get_tick_ms();
                    hal_debug_printf("[MODE] Switched to PS2\r\n");
                } else {
                    hal_debug_printf("Usage: !MODE crsf|ps2\r\n");
                }
                break;
            }
#endif /* PS2_ENABLED */
            /* !M<n>  站立姿态: -1=窄, 0=正常, 1=宽 */
            if (len >= 3) {
                int8_t m = (int8_t)parse_int(buf, 2, len);
                if (m >= -1 && m <= 1) {
                    ctrl_state->stance_mode = m;
                    hal_debug_printf("Stance: %d\r\n", m);
                }
            }
            break;
        case 'V': { uint8_t lvl = hal_debug_get_level(); lvl = (lvl + 1) % 4; hal_debug_set_level(lvl); } break;

        /* ---- 舵机直控 (硬件排查) ---- */
        case 'P': {
            /* ---- !PER: PCA9685 PWM 周期校准模式 ---- */
            if (len >= 3 && buf[2] == 'E' && buf[3] == 'R') {
                period_calib_handle_command(buf, len);
                break;
            }
            /* ---- !PS2: PS2 手柄原始状态调试输出 ---- */
#if PS2_ENABLED
            if (len >= 3 && buf[2] == 'S' && (len == 3 || (len == 4 && buf[3] == '2'))) {
                ps2_state_print();
                break;
            }

            /* ---- !PS2DBG: PS2 调试观察模式开关 ----
             * 开启: 每秒打印一次全部通道值 (摇杆/按键/模式/中位校准),
             *       机器人忽略手柄输入保持静止 — 仅观察通道与数据解析 */
            if (len >= 7 && buf[2] == 'S' && buf[3] == '2' &&
                buf[4] == 'D' && buf[5] == 'B' && buf[6] == 'G') {
                g_ps2_debug_active = !g_ps2_debug_active;
                hal_debug_printf("[PS2] Debug observe mode: %s\r\n",
                                 g_ps2_debug_active ? "ON (robot ignores input)" : "OFF");
                break;
            }
#endif /* PS2_ENABLED */
            /* !P<servo_id> <angle>  例: !P0 900  */
            if (len < 5) { hal_debug_printf("Usage: !P<id> <angle>\r\n"); break; }
            uint8_t sid = (uint8_t)parse_int(buf, 2, len);
            /* 跳过 id 找到空格后的 angle */
            uint8_t pos = 2; while (pos < len && buf[pos] != ' ') pos++;
            int16_t ang = parse_int(buf, pos + 1, len);
            if (sid < 18 && ang >= -1800 && ang <= 1800) {
                /* 直接写入舵机，绕过 IK/步态
                 * PCB 定版: ID 0~8=右腿→0x41, ID 9~17=左腿→0x40 */
                uint16_t pulse = pca9685_angle_to_pulse(ang);
                uint8_t addr = (sid <= 8) ? PCA9685_ADDR_RIGHT : PCA9685_ADDR_LEFT;
                if (pca9685_get_board_idx_by_addr(addr) < pca9685_get_board_count()) {
                    pca9685_set_servo_pulse(sid, pulse);
                    hal_debug_printf("Servo %u → angle %d (pulse %u us, addr 0x%02X)\r\n", sid, ang, pulse, addr);
                } else {
                    hal_debug_printf("Servo %u: board 0x%02X not present\r\n", sid, addr);
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
            /* 打印所有舵机当前角度（从 servo batch 缓存读取）
             * PCB 定版: 0x40=左腿板(ID 9~17), 0x41=右腿板(ID 0~8) */
            hal_debug_printf("=== Servo Snapshot ===\r\n");
            for (uint8_t b = 0; b < pca9685_get_board_count(); b++) {
                uint8_t addr = pca9685_get_board_addr(b);
                uint8_t id_start = (addr == PCA9685_ADDR_LEFT) ? 9 : 0;
                uint8_t id_end   = id_start + 8;
                hal_debug_printf("Board 0x%02X (%s): ", addr,
                                 (addr == PCA9685_ADDR_LEFT) ? "left" : "right");
                for (uint8_t i = id_start; i <= id_end; i++) {
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

#if INPUT_CONTROL_MODE == 2
    /* ========== USB CDC 串口模式 ==========
     * 仅通过 USB 虚拟串口接收命令，无 UART1 硬件参与。
     * 非阻塞轮询：getchar_timeout_us(0) 无数据立即返回。 */
    {
        static uint8_t usb_buf[16];
        static uint8_t usb_len = 0;

        int ch = getchar_timeout_us(0);
        while (ch != PICO_ERROR_TIMEOUT) {
            if (ch == '\n' || ch == '\r') {
                if (usb_len > 0) {
                    parse_serial_command(ctrl_state, usb_buf, usb_len);
                    usb_len = 0;
                    return true;
                }
            } else if (usb_len < sizeof(usb_buf)) {
                usb_buf[usb_len++] = (uint8_t)ch;
            }
            ch = getchar_timeout_us(0);
        }
    }
    return false;

#else
    /* ========== CRSF 模式 ==========
     * USB CDC 命令在 CRSF 链路断开时可用作后备控制，
     * 但 CRSF 激活时会覆写 control_state。 */

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

    uint32_t now = hal_get_tick_ms();

#if PS2_ENABLED
    /* ========== PS2 模式: 轮询手柄 ========== */
    if (g_input_mode == INPUT_MODE_PS2 && g_ps2_initialized) {
        /* 最小轮询间隔: 无线接收器对过快轮询敏感 (易返回垃圾帧),
         * 标准实现 ~20ms 一次; 控制环 10ms 只会重复读到同一帧, 无收益 */
        static uint32_t last_poll_ms = 0;
        uint32_t poll_t = hal_get_tick_ms();
        if ((poll_t - last_poll_ms) >= PS2_POLL_INTERVAL_MS) {
            last_poll_ms = poll_t;
        if (ps2_read_gamepad(&g_ps2_state)) {
            /* 数字模式 (绿灯) → 自动进入模拟模式 (红灯):
             * 数字帧无摇杆比例输出, 不切换摇杆无法正常控制。
             * 每秒重试一次直到成功; 进入模拟模式时摇杆需居中 (中位采样)。 */
            if (!g_ps2_state.analog_mode) {
                static uint32_t last_analog_attempt_ms = 0;
                uint32_t t = hal_get_tick_ms();
                if ((t - last_analog_attempt_ms) >= 1000) {
                    last_analog_attempt_ms = t;
                    ps2_enter_analog_mode(&g_ps2_state);
                    hal_debug_printf("[PS2] Requesting analog mode...\r\n");
                }
            }
            /* ---- PS2 调试观察模式: 每秒打印通道值, 机器人不响应 ---- */
            if (g_ps2_debug_active) {
                static uint32_t last_dbg_print_ms = 0;
                if ((poll_t - last_dbg_print_ms) >= 1000) {
                    last_dbg_print_ms = poll_t;
                    ps2_state_print();
                }
                /* 清空全部运动/姿态指令, 机器人保持站立不动 */
                ctrl_state->travel_length.x = 0;
                ctrl_state->travel_length.y = 0;
                ctrl_state->travel_length.z = 0;
                ctrl_state->body_rot.x = 0;
                ctrl_state->body_rot.y = 0;
                ctrl_state->body_rot.z = 0;
                return true;
            }

            /* 映射到控制状态 */
            ps2_to_control(&g_ps2_state, ctrl_state);
            return true;
        }
        }
    }
#endif /* PS2_ENABLED */

    if (g_crsf_mode) {
        /* ========== CRSF 模式 ========== */

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
#endif
}

bool hal_poll_usb_commands(control_state_t *ctrl_state)
{
    static uint8_t usb_buf[16];
    static uint8_t usb_len = 0;

    int ch = getchar_timeout_us(0);
    while (ch != PICO_ERROR_TIMEOUT) {
        if (ch == '\n' || ch == '\r') {
            if (usb_len > 0) {
                parse_serial_command(ctrl_state, usb_buf, usb_len);
                usb_len = 0;
                return true;
            }
        } else if (usb_len < sizeof(usb_buf)) {
            usb_buf[usb_len++] = (uint8_t)ch;
        }
        ch = getchar_timeout_us(0);
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
#if USB_DEBUG_ENABLED
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
#else
    (void)format;   /* USB 数据模式: 调试输出静默, USB CDC 通道专供上位机 */
#endif
}

/* ==================== 蜂鸣器实现（无源蜂鸣器, PWM 方波驱动, 本地 GPIO, 不走 I2C） ==================== */

/*
 * 无源蜂鸣器 (GP13 = PWM6B):
 *   无内部振荡器, 需要外部方波信号才能发声。
 *   通过 PWM 改变频率 → 不同音高; 50% 占空比 → 最大音量。
 *
 * 时钟计算:
 *   PWM 时钟 = 125MHz / 16 (clkdiv) = 7.8125MHz
 *   wrap = 7.8125MHz / 频率
 *   最低频率 200Hz → wrap = 39062 (16-bit 上限内)
 */

#define BUZZER_PIN              13      /* GP13 (PWM6B), 无源蜂鸣器 */
#define BUZZER_PWM_CLKDIV       16.0f   /* 125MHz / 16 = 7.8125MHz */
#define BUZZER_MIN_FREQ_HZ      200     /* 最低频率 (16-bit wrap 上限约束) */

static bool buzzer_initialized = false;

void hal_play_sound(uint8_t note_count,
                   const uint16_t *notes,
                   const uint16_t *durations)
{
    if (!notes || !durations || note_count == 0) return;

    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint chan  = pwm_gpio_to_channel(BUZZER_PIN);

    /* 初始化蜂鸣器 GPIO 为 PWM 功能 */
    if (!buzzer_initialized) {
        gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
        pwm_set_clkdiv(slice, BUZZER_PWM_CLKDIV);
        pwm_set_enabled(slice, true);   /* ★ 必须使能 slice, 计数器才运行 */
        buzzer_initialized = true;
    }

    const uint32_t pwm_clk = 125000000UL / 16;   /* 7.8125 MHz */

    for (uint8_t i = 0; i < note_count && i < 10; i++) {
        if (notes[i] > 0) {
            uint32_t freq = notes[i];
            if (freq < BUZZER_MIN_FREQ_HZ) freq = BUZZER_MIN_FREQ_HZ;

            uint32_t wrap = pwm_clk / freq;   /* 200Hz → 39062 */
            pwm_set_wrap(slice, (uint16_t)(wrap - 1));
            pwm_set_chan_level(slice, chan, wrap / 2);   /* 50% 占空比 */
            sleep_ms(durations[i]);
        } else {
            /* 音符频率 0 = 休止 */
            pwm_set_chan_level(slice, chan, 0);
            sleep_ms(durations[i]);
        }

        /* 音符间短停顿, 避免连音 */
        pwm_set_chan_level(slice, chan, 0);
        sleep_ms(30);
    }

    /* 结束后静音 */
    pwm_set_chan_level(slice, chan, 0);
}

/* ==================== 足端微动开关实现（6路 GPIO 输入，上拉，落地→GND=低电平） ==================== */

/*
 * 六足机器人每条腿的足端安装微动开关，用于检测足端是否接触地面。
 *
 * 电气连接：
 *   微动开关一端接 GPIO，另一端接 GND。
 *   GPIO 启用内部上拉 (~50kΩ)，开关断开时读为高电平 (未着地)，
 *   开关闭合时读为低电平 (足端着地)。
 *
 * GPIO 分配 (连续引脚，便于排线):
 *   GP16 → 腿 0 (RR) 右后
 *   GP17 → 腿 1 (RM) 右中
 *   GP18 → 腿 2 (RF) 右前
 *   GP19 → 腿 3 (LR) 左后
 *   GP20 → 腿 4 (LM) 左中
 *   GP21 → 腿 5 (LF) 左前
 */

#define FOOT_SW_PIN_RR   16
#define FOOT_SW_PIN_RM   17
#define FOOT_SW_PIN_RF   18
#define FOOT_SW_PIN_LR   19
#define FOOT_SW_PIN_LM   20
#define FOOT_SW_PIN_LF   21

/* 快速查找表：leg_index → GPIO 引脚号 */
static const uint8_t g_foot_sw_pins[6] = {
    FOOT_SW_PIN_RR,  /* LEG_RR = 0 */
    FOOT_SW_PIN_RM,  /* LEG_RM = 1 */
    FOOT_SW_PIN_RF,  /* LEG_RF = 2 */
    FOOT_SW_PIN_LR,  /* LEG_LR = 3 */
    FOOT_SW_PIN_LM,  /* LEG_LM = 4 */
    FOOT_SW_PIN_LF   /* LEG_LF = 5 */
};

static bool g_foot_sw_initialized = false;

void hal_foot_switch_init(void)
{
    for (uint8_t i = 0; i < 6; i++) {
        gpio_init(g_foot_sw_pins[i]);
        gpio_set_dir(g_foot_sw_pins[i], GPIO_IN);
        gpio_pull_up(g_foot_sw_pins[i]);  /* 内部上拉，开关闭合→GND 读低 */
    }
    g_foot_sw_initialized = true;
}

bool hal_foot_switch_read(leg_index_t leg, bool *contact)
{
    if (!g_foot_sw_initialized || !contact) return false;
    if (leg >= CNT_LEGS) return false;

    /* 低电平 = 开关闭合 = 足端着地 */
    *contact = !gpio_get(g_foot_sw_pins[leg]);
    return true;
}

uint8_t hal_foot_switch_read_all(bool contacts[6])
{
    if (!g_foot_sw_initialized || !contacts) return 0;

    uint8_t count = 0;
    for (uint8_t i = 0; i < 6; i++) {
        contacts[i] = !gpio_get(g_foot_sw_pins[i]);
        if (contacts[i]) count++;
    }
    return count;  /* 返回着地足数 */
}

/* ==================== LED实现（本地GPIO，不走I2C） ==================== */

/*
 * 双 LED 系统:
 *   0 = 绿色 LED (GP25, Pico 板载) — 状态指示 (心跳/运行状态)
 *   1 = 红色 LED (GP12, 外部)      — 错误/报警指示 (低电压/I2C故障)
 * 两者功能分离, 绿色=正常, 红色=异常。
 */

#define LED_GREEN_PIN           25
#define LED_RED_PIN             12

static bool led_initialized = false;

static void led_init_once(void)
{
    if (led_initialized) return;

    gpio_init(LED_GREEN_PIN);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_put(LED_GREEN_PIN, 0);

    gpio_init(LED_RED_PIN);
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_put(LED_RED_PIN, 0);

    led_initialized = true;
}

void hal_led_set(uint8_t led_id, bool state)
{
    led_init_once();

    switch (led_id) {
        case 1:
            gpio_put(LED_RED_PIN, state);
            break;
        case 0:
        default:
            gpio_put(LED_GREEN_PIN, state);
            break;
    }
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

/* ==================== PCA9685 PWM 周期校准模式 (!PER) ==================== */

/*
 * 无示波器校准法:
 *   1. 进入模式后, 所有 6 个 coxa 舵机收到 90° 指令 (脉宽 1500µs),
 *      其余舵机 (femur/tibia) 不发送任何指令
 *   2. 用户目测/量角器观察 coxa 摆臂, 输入 !PER0/!PER1 调整周期值
 *   3. 每次调整立即以新周期重写 coxa 脉冲 → 舵机实时响应
 *   4. 当 coxa 摆臂到达正确机械位置时, 该周期值即为实测周期
 *   5. !PERQ 退出并打印最终值, 填入 hexapod_i2c_protocol.h
 */

static bool g_period_calib_active = false;

/* horn_offset 开关: true=叠加各腿 coxa_horn_offset (摆臂到校准后的中位),
 * false=纯 1500µs 原始中位。默认开启, 与机器人实际运行一致。 */
static bool g_period_calib_use_offset = true;

/* 6 个 coxa 舵机 ID (每腿第 0 关节) 及其对应腿索引 */
static const uint8_t g_coxa_servo_ids[6] = {
    SERVO_RR_COXA, SERVO_RM_COXA, SERVO_RF_COXA,
    SERVO_LR_COXA, SERVO_LM_COXA, SERVO_LF_COXA
};
static const leg_index_t g_coxa_leg_idx[6] = {
    LEG_RR, LEG_RM, LEG_RF, LEG_LR, LEG_LM, LEG_LF
};

/**
 * @brief 将所有 coxa 舵机置中位, 使用当前周期校准值
 * @note 固件角度约定: 0 = 舵机中位 (1500µs), ±900 = ±90° 极限
 *       目标位置 = 0 + horn_offset (开关控制), 与机器人运行公式一致:
 *       最终舵机角度 = IK 计算角度 + horn_offset
 *       注意 900 会被 pca9685_angle_to_pulse 钳位到 2500µs 极限, 勿用
 */
static void period_calib_apply_coxas(void)
{
    leg_config_t configs[CNT_LEGS];
    hexapod_get_default_config(configs);

    uint8_t ok = 0;
    for (uint8_t i = 0; i < 6; i++) {
        int16_t target = 0;   /* 中位基准 */
        if (g_period_calib_use_offset) {
            target = configs[g_coxa_leg_idx[i]].coxa_horn_offset;
        }
        uint16_t pulse = pca9685_angle_to_pulse(target);
        if (pca9685_set_servo_pulse(g_coxa_servo_ids[i], pulse)) {
            ok++;
        }
    }
    hal_debug_printf("[PER] Coxa write: %u/6 OK (horn_offset: %s)\r\n",
                     ok, g_period_calib_use_offset ? "ON" : "OFF");
}

static void period_calib_print_status(void)
{
    for (uint8_t b = 0; b < pca9685_get_board_count(); b++) {
        uint8_t addr = pca9685_get_board_addr(b);
        hal_debug_printf("[PER] Board %u (0x%02X, %s legs): period=%u us\r\n",
                         b, addr,
                         (addr == PCA9685_ADDR_LEFT) ? "left " : "right",
                         pca9685_get_pwm_period_us(b));
    }
    hal_debug_printf("[PER] Coxa @center, horn_offset=%s. "
                     "Cmds: !PER0 <us>  !PER1 <us>  !PERO 0/1  !PERS  !PERQ\r\n",
                     g_period_calib_use_offset ? "ON" : "OFF");
}

/**
 * @brief 处理周期校准模式命令
 * @return true 表示命令已被消费
 */
static bool period_calib_handle_command(uint8_t *buf, uint8_t len)
{
    if (len < 4) return false;
    if (buf[2] != 'E' || buf[3] != 'R') return false;

    if (len == 4) {
        /* !PER: 进入/刷新校准模式 */
        if (!g_period_calib_active) {
            g_period_calib_active = true;
            /* 校准直接驱动舵机, 必须先开启舵机供电 */
            hal_servo_power_set_all(true);
            hal_delay_ms(100);
            hal_debug_printf("[PER] Period calibration mode entered\r\n");
            period_calib_apply_coxas();
        }
        period_calib_print_status();
        return true;
    }

    switch (buf[4]) {
        case 'Q': case 'q':
            /* !PERQ: 退出并打印最终值 (复制到 hexapod_i2c_protocol.h) */
            g_period_calib_active = false;
            hal_debug_printf("[PER] Calibration ended. Final values:\r\n");
            for (uint8_t b = 0; b < pca9685_get_board_count(); b++) {
                uint8_t addr = pca9685_get_board_addr(b);
                hal_debug_printf("  #define PWM_PERIOD_US_BOARD%u    %u   /* 0x%02X (%s) */\r\n",
                                 b, pca9685_get_pwm_period_us(b), addr,
                                 (addr == PCA9685_ADDR_LEFT) ? "left" : "right");
            }
            return true;

        case 'S': case 's':
            /* !PERS: 重新置 coxa 中位 (用当前周期值和偏移开关) */
            period_calib_apply_coxas();
            return true;

        case 'O': case 'o':
            /* !PERO 0/1: horn_offset 开关
             *   1 = 叠加各腿 coxa_horn_offset (与机器人运行公式一致, 默认)
             *   0 = 纯 1500µs 原始中位
             * 切换后立即重新写 coxa */
            if (len >= 6) {
                int16_t v = parse_int(buf, 5, len);
                g_period_calib_use_offset = (v != 0);
            }
            period_calib_apply_coxas();
            hal_debug_printf("[PER] horn_offset mode: %s\r\n",
                             g_period_calib_use_offset ? "ON" : "OFF");
            return true;

        case '0': case '1': {
            /* !PER0 <us> / !PER1 <us>: 设置周期并实时生效 */
            if (len < 6) {
                hal_debug_printf("Usage: !PER0 <us>  or  !PER1 <us>\r\n");
                return true;
            }
            /* 板号映射: 0 = 0x40 (左腿板), 1 = 0x41 (右腿板) */
            uint8_t addr = (buf[4] == '0') ? PCA9685_ADDR_LEFT : PCA9685_ADDR_RIGHT;
            uint8_t idx  = pca9685_get_board_idx_by_addr(addr);
            if (idx >= 2) {
                hal_debug_printf("[PER] Board 0x%02X not present\r\n", addr);
                return true;
            }

            uint16_t us = (uint16_t)parse_int(buf, 5, len);
            pca9685_set_pwm_period_us(idx, us);
            period_calib_apply_coxas();   /* 立即以新周期重写 coxa → 舵机实时响应 */

            hal_debug_printf("[PER] Board %u (0x%02X) period=%u us, coxa re-applied\r\n",
                             idx, addr, pca9685_get_pwm_period_us(idx));
            return true;
        }

        default:
            hal_debug_printf("Usage: !PER | !PER0 <us> | !PER1 <us> | !PERO 0/1 | !PERS | !PERQ\r\n");
            return true;
    }
}

bool hal_is_period_calib_active(void)
{
    return g_period_calib_active;
}

/* ==================== IMU 姿态传感器接口 ==================== */

static bool g_imu_available = false;

#if IMU_ENABLED
static imu_data_t g_imu_last;           /* 最近一次有效读数 (调试用) */
#endif

bool hal_imu_init(void)
{
#if IMU_ENABLED
    g_imu_available = false;
    memset(&g_imu_last, 0, sizeof(g_imu_last));

#if IMU_BOOT_GPIO_ENABLED
    /* BOOT 引脚: 上电保持高 → BNO055 正常应用模式启动
     * (拉低则进入 bootloader) */
    gpio_init(IMU_BOOT_PIN);
    gpio_set_dir(IMU_BOOT_PIN, GPIO_OUT);
    gpio_put(IMU_BOOT_PIN, 1);
    sleep_ms(10);   /* 稳定 BOOT 电平后再访问 I2C */
#endif

    /* 初始化 + 自动恢复: 失败时循环 BOOT 引脚强制重启 BNO055 */
    for (uint8_t attempt = 0; attempt < IMU_INIT_RETRY_MAX; attempt++) {
        if (bno055_init(BNO055_I2C_ADDR)) {
            g_imu_available = true;
            break;
        }
#if IMU_BOOT_GPIO_ENABLED
        hal_debug_printf("[IMU] init failed (attempt %u/%u), cycling BOOT pin...\r\n",
                         attempt + 1, IMU_INIT_RETRY_MAX);
        gpio_put(IMU_BOOT_PIN, 0);    /* 进入 boot 模式 */
        sleep_ms(50);
        gpio_put(IMU_BOOT_PIN, 1);    /* 释放 → 重新上电启动序列 */
        sleep_ms(BNO055_POR_WAIT_MS);
#endif
    }

    if (g_imu_available) {
        hal_debug_printf("IMU: BNO055 detected at 0x%02X\r\n", BNO055_I2C_ADDR);
    } else {
        hal_debug_printf("WARNING: BNO055 not found at 0x%02X after %u retries, IMU disabled\r\n",
                         BNO055_I2C_ADDR, IMU_INIT_RETRY_MAX);
    }
    return g_imu_available;
#else
    g_imu_available = false;
    return false;
#endif
}

bool hal_imu_read(imu_data_t *data)
{
    if (!data) return false;

#if IMU_ENABLED
    if (!g_imu_available) {
        data->valid = false;
        return false;
    }

    bno055_euler_t raw;
    if (!bno055_read_euler(&raw)) {
        data->valid = false;
        return false;
    }

    /* BNO055 欧拉角: 1° = 16 LSB → 代码 0.1° = raw * 10 / 16
     *
     * BNO055 输出约定 (Windows 默认, UNIT_SEL bit0=0):
     *   Roll  → 绕 Y 轴 (左右倾斜)
     *   Pitch → 绕 X 轴 (前后倾斜)
     *   Heading → 绕 Z 轴
     *
     * 机器人坐标映射 (正常安装, 芯片朝上, 2026-08 新 PCB):
     *
     *   安装几何 (右手定则推导 + 实测验证):
     *     Xc = rX   芯片 X 轴 → 机器人正前方 (用户确认)
     *     Yc = rZ   芯片 Y 轴 → 机器人右侧
     *     Zc = rY   芯片 Z 轴 → 机器人上方 (正常安装, 不再倒扣)
     *
     *   平放读数: chip pitch≈0, chip roll≈0
     *   (旧板倒扣 Zc=-rY, 平放 chip pitch≈180° 需 -1800 修正,
     *    新板已删除该修正项, 轴对应关系不变)
     *
     *   倾角映射 (同轴同向):
     *     robot_roll  (绕 rX=Xc) ← chip pitch        [chip pitch 绕 Xc]
     *     robot_pitch (绕 rZ=Yc) ← chip roll          [chip roll 绕 Yc]
     *     robot_yaw   (绕 rY)     ← chip heading      (暂不使用)
     *
     * 符号由 IMU_ROLL_SIGN / IMU_PITCH_SIGN 配置,
     * 若补偿加剧倾斜 (正反馈) → 取反对应符号。 */

    data->roll  = IMU_ROLL_SIGN  * ((raw.pitch * 10) / 16);
    data->pitch = IMU_PITCH_SIGN * ((raw.roll  * 10) / 16);
    data->yaw   = (raw.heading * 10) / 16; /* 暂不使用，保留 */
    data->valid = true;

    g_imu_last = *data;   /* 缓存最近读数, 供 !IMU 调试 */
    return true;
#else
    data->valid = false;
    return false;
#endif
}

/* ==================== IMU 调试状态 (!IMU) ==================== */

/**
 * @brief 打印 IMU 当前状态: 可用性/中断计数/校准/欧拉角
 */
static void imu_status_print(void)
{
#if IMU_ENABLED
    hal_debug_printf("=== IMU Status ===\r\n");
    hal_debug_printf("available: %s  addr: 0x%02X\r\n",
                     g_imu_available ? "YES" : "NO", BNO055_I2C_ADDR);

    if (g_imu_available) {
        bno055_calib_t calib;
        if (bno055_get_calib(&calib)) {
            hal_debug_printf("calib: sys=%u gyr=%u acc=%u mag=%u %s\r\n",
                             calib.sys, calib.gyro, calib.accel, calib.mag,
                             bno055_is_calibrated() ? "(fully)" : "");
        }

        /* INT 诊断: 状态/配置读回 + 软件版本 */
        uint8_t sta = 0;
        if (bno055_get_int_status(&sta)) {
            hal_debug_printf("INT_STA=0x%02X (bit7=BSX_DRDY)\r\n", sta);
        }
        uint8_t en = 0, msk = 0, cntl = 0;
        if (bno055_get_int_config(&en, &msk, &cntl)) {
            hal_debug_printf("INT cfg readback: EN=0x%02X MSK=0x%02X CNTL=0x%02X\r\n",
                             en, msk, cntl);
        }
        uint16_t sw_rev = 0;
        if (bno055_get_sw_rev(&sw_rev)) {
            hal_debug_printf("SW rev: 0x%04X (BSX DRDY 需 ≥0x0314)\r\n", sw_rev);
        }

        hal_debug_printf("last: roll=%d pitch=%d yaw=%d (0.1deg) valid=%d\r\n",
                         g_imu_last.roll, g_imu_last.pitch, g_imu_last.yaw,
                         g_imu_last.valid);
    } else {
        hal_debug_printf("IMU not available (check !I2C and BOOT wiring)\r\n");
    }
    hal_debug_printf("===================\r\n");
#else
    hal_debug_printf("[IMU] IMU disabled (IMU_ENABLED=0)\r\n");
#endif
}

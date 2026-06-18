/**
 * @file hexapod_i2c_protocol.c
 * @brief I2C舵机驱动 - PCA9685控制实现
 * @note Pico作为I2C Master，通过两个PCA9685模块控制18个舵机
 *       I2C总线仅用于舵机PWM输出，不传输任何其他信号
 * 
 * 连接方式：
 *   Pico GP2(SDA) - PCA9685#1 SDI - PCA9685#2 SDI（共用总线）
 *   Pico GP3(SCL) - PCA9685#1 SCL - PCA9685#2 SCL（共用总线）
 *   PCA9685#1 ADDR接GND -> 地址0x40
 *   PCA9685#2 ADDR接VCC -> 地址0x41
 */

#include "hexapod_i2c_protocol.h"
#include "hexapod_config.h"
#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

/* ==================== 板载追踪 ==================== */

/* 候选地址列表（按优先级排列） */
static const uint8_t PCA9685_CANDIDATE_ADDRS[] = {0x40, 0x41};

/* 运行时检测结果 */
static uint8_t  g_board_addrs[2] = {0};
static uint8_t  g_board_count = 0;
static uint16_t g_board_pwm_period_us[2] = {PWM_PERIOD_US_BOARD0, PWM_PERIOD_US_BOARD1};

uint8_t pca9685_get_board_count(void) { return g_board_count; }
uint8_t pca9685_get_board_addr(uint8_t index) {
    return (index < g_board_count) ? g_board_addrs[index] : 0;
}
uint16_t pca9685_get_pwm_period_us(uint8_t board_idx) {
    return (board_idx < 2) ? g_board_pwm_period_us[board_idx] : PWM_PERIOD_US_BOARD0;
}

/* ==================== 内部帮助函数 ==================== */

/**
 * @brief 向PCA9685指定寄存器写入一个字节
 * @param addr PCA9685 I2C地址
 * @param reg 寄存器地址
 * @param data 要写入的数据
 * @return true表示成功
 */
static inline bool pca9685_write_reg(uint8_t addr, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    int ret = i2c_write_blocking(PCA9685_I2C_INSTANCE, addr, buf, 2, false);
    return (ret == 2);
}

/**
 * @brief 从PCA9685指定寄存器读取一个字节
 * @param addr PCA9685 I2C地址
 * @param reg 寄存器地址
 * @param data 输出缓冲区
 * @return true表示成功
 */
static inline bool pca9685_read_reg(uint8_t addr, uint8_t reg, uint8_t *data)
{
    int ret = i2c_write_blocking(PCA9685_I2C_INSTANCE, addr, &reg, 1, true);
    if (ret != 1) return false;
    ret = i2c_read_blocking(PCA9685_I2C_INSTANCE, addr, data, 1, false);
    return (ret == 1);
}

/**
 * @brief 向PCA9685连续写入多个字节（使用自动递增功能）
 * @param addr PCA9685 I2C地址
 * @param reg 起始寄存器地址
 * @param data 数据缓冲区
 * @param len 数据长度
 * @return true表示成功
 */
static inline bool pca9685_write_burst(uint8_t addr, uint8_t reg, 
                                       const uint8_t *data, uint8_t len)
{
    /* 构建发送缓冲区：寄存器地址 + 数据 */
    uint8_t buf[128];
    if (len > 127) len = 127;
    buf[0] = reg;
    memcpy(buf + 1, data, len);
    
    int ret = i2c_write_blocking(PCA9685_I2C_INSTANCE, addr, buf, len + 1, false);
    return (ret == (int)(len + 1));
}

/* ==================== 初始化与配置 ==================== */

bool pca9685_init(void)
{
    /* 初始化I2C硬件 */
    i2c_init(PCA9685_I2C_INSTANCE, PCA9685_I2C_BAUD);
    gpio_set_function(PCA9685_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PCA9685_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PCA9685_I2C_SDA_PIN);
    gpio_pull_up(PCA9685_I2C_SCL_PIN);

    /* 自动探测可用的 PCA9685 板 */
    g_board_count = 0;
    uint8_t max_boards = (PCA9685_BOARD_COUNT <= 2) ? PCA9685_BOARD_COUNT : 2;

    for (uint8_t i = 0; i < max_boards; i++) {
        uint8_t addr = PCA9685_CANDIDATE_ADDRS[i];
        uint8_t mode1;

        /* 尝试读取 MODE1 寄存器验证芯片是否存在 */
        if (pca9685_read_reg(addr, PCA9685_MODE1, &mode1)) {
            /* 唤醒：清除 SLEEP 位，使能自动递增 */
            mode1 &= ~PCA9685_MODE1_SLEEP;
            mode1 |= PCA9685_MODE1_AI;
            if (pca9685_write_reg(addr, PCA9685_MODE1, mode1)) {
                g_board_addrs[g_board_count++] = addr;
                printf("  PCA9685 #%u found at 0x%02X\r\n", g_board_count, addr);
            }
        }
    }

    if (g_board_count == 0) {
        printf("  No PCA9685 detected on I2C bus!\r\n");
        printf("  Check: SDA=GP%d SCL=GP%d power to PCA9685\r\n",
               PCA9685_I2C_SDA_PIN, PCA9685_I2C_SCL_PIN);
        return false;
    }

    /* 等待振荡器稳定 */
    sleep_ms(1);

    /* 配置 MODE2：推挽输出 */
    for (uint8_t i = 0; i < g_board_count; i++) {
        pca9685_write_reg(g_board_addrs[i], PCA9685_MODE2, PCA9685_MODE2_OUTDRV);
    }

    /* 设置 PWM 频率为 50Hz */
    if (!pca9685_set_freq(PCA9685_FREQUENCY)) return false;

    /* 所有通道初始输出低电平 */
    pca9685_free_all();

    return true;
}

bool pca9685_set_freq(uint16_t freq)
{
    if (freq < 1 || freq > 1000) return false;

    /* 计算预分频值
     * PRE_SCALE = round(osc_clock / (4096 * freq)) - 1
     * 内部振荡器 = 25MHz
     */
    float pre_scale_val = 25000000.0f / (4096.0f * freq) - 1.0f;
    uint8_t pre_scale = (uint8_t)(pre_scale_val + 0.5f);
    if (pre_scale < 3) pre_scale = 3;

    for (uint8_t i = 0; i < g_board_count; i++) {
        uint8_t addr = g_board_addrs[i];
        uint8_t mode1;

        /* 进入 SLEEP 模式 */
        pca9685_read_reg(addr, PCA9685_MODE1, &mode1);
        mode1 |= PCA9685_MODE1_SLEEP;
        pca9685_write_reg(addr, PCA9685_MODE1, mode1);

        /* 写入预分频 */
        pca9685_write_reg(addr, PCA9685_PRE_SCALE, pre_scale);

        /* 唤醒 */
        pca9685_read_reg(addr, PCA9685_MODE1, &mode1);
        mode1 &= ~PCA9685_MODE1_SLEEP;
        mode1 |= PCA9685_MODE1_AI;
        pca9685_write_reg(addr, PCA9685_MODE1, mode1);
    }

    sleep_ms(1);

    /* 发送 RESTART */
    for (uint8_t i = 0; i < g_board_count; i++) {
        uint8_t addr = g_board_addrs[i];
        uint8_t mode1;
        pca9685_read_reg(addr, PCA9685_MODE1, &mode1);
        mode1 |= PCA9685_MODE1_RESTART;
        pca9685_write_reg(addr, PCA9685_MODE1, mode1);
    }

    return true;
}

uint16_t pca9685_angle_to_pulse(int16_t angle)
{
    /* 舵机角度 → PWM 脉宽映射：
     *   角度 0   = 舵机中位 → 1500us
     *   角度 +900 = +90°  → 2500us (顺时针极限)
     *   角度 -900 = -90°  →  500us (逆时针极限)
     *
     *   右腿 invert=true 时 IK 输出负角度，此处正确处理正负双向映射 */
#define SERVO_PULSE_MID 1500

    if (angle <= -900) return SERVO_PULSE_MIN;
    if (angle >=  900) return SERVO_PULSE_MAX;

    /* pulse = 1500 + angle * (1000 / 900)，即每 0.9° 偏移 10us */
    int32_t pulse = SERVO_PULSE_MID + ((int32_t)angle * 1000) / 900;
    if (pulse < SERVO_PULSE_MIN) pulse = SERVO_PULSE_MIN;
    if (pulse > SERVO_PULSE_MAX) pulse = SERVO_PULSE_MAX;
    return (uint16_t)pulse;
}

/* ==================== 舵机PWM输出 ==================== */

/**
 * @brief 将脉宽（微秒）转换为PCA9685的12位计数值
 * @param pulse_us       目标脉宽 (微秒)
 * @param pwm_period_us  该 PCA9685 板的实测 PWM 周期 (微秒)
 */
static inline uint16_t pulse_to_count(uint16_t pulse_us, uint16_t pwm_period_us)
{
    if (pulse_us >= pwm_period_us) return PCA9685_RESOLUTION - 1;
    return (uint16_t)(((uint32_t)pulse_us * PCA9685_RESOLUTION) / pwm_period_us);
}

/**
 * @brief 写入所有缓存的脉宽到指定 PCA9685（使用自动递增，一次 I2C 事务）
 * @param addr PCA9685 I2C地址
 * @param pulses 脉宽数组（16通道）
 * @param count 有效通道数量
 */
bool pca9685_write_all_channels(uint8_t addr, const uint16_t *pulses, uint8_t count)
{
    if (!pulses || count == 0) return false;
    if (count > 16) count = 16;

    /* 查找该地址对应的板索引，获取其校准周期 */
    uint16_t pwm_period_us = PWM_PERIOD_US_BOARD0;  /* 默认 */
    for (uint8_t i = 0; i < g_board_count; i++) {
        if (g_board_addrs[i] == addr) {
            pwm_period_us = g_board_pwm_period_us[i];
            break;
        }
    }

    /* 构建数据：寄存器地址 + 64字节（16通道 × 4字节）
       自动递增模式下，从 LED0_ON_L 开始写入 */
    uint8_t data[65];
    data[0] = PCA9685_LED0_ON_L;

    for (uint8_t ch = 0; ch < 16; ch++) {
        uint16_t off_count = pulse_to_count(pulses[ch], pwm_period_us);
        uint8_t idx = 1 + ch * 4;
        data[idx]     = 0;                              // ON_L = 0
        data[idx + 1] = 0;                              // ON_H = 0
        data[idx + 2] = (uint8_t)(off_count & 0xFF);    // OFF_L
        data[idx + 3] = (uint8_t)((off_count >> 8) & 0x0F); // OFF_H
    }

    int ret = i2c_write_blocking(PCA9685_I2C_INSTANCE, addr, data, sizeof(data), false);
    return (ret == (int)sizeof(data));
}

bool pca9685_set_servo_pulse(uint8_t servo_id, uint16_t pulse_us)
{
    if (servo_id >= SERVO_TOTAL_COUNT) return false;

    /* 确定使用的 PCA9685 板索引和内部通道号
     * 板 0: 舵机 ID 0~8 (右腿组)
     * 板 1: 舵机 ID 9~17 (左腿组) — 仅当 g_board_count >= 2 时可用 */
    uint8_t board_idx;
    uint8_t channel;

    if (servo_id <= PCA9685_1_SERVO_MAX) {
        board_idx = 0;
        channel = servo_id;
    } else {
        board_idx = 1;
        channel = servo_id - PCA9685_2_SERVO_MIN;
    }

    /* 目标板不存在则静默跳过 */
    if (board_idx >= g_board_count) return false;

    uint8_t  pca_addr       = g_board_addrs[board_idx];
    uint16_t pwm_period_us  = g_board_pwm_period_us[board_idx];

    /* 限幅 */
    if (pulse_us < SERVO_PULSE_MIN) pulse_us = SERVO_PULSE_MIN;
    if (pulse_us > SERVO_PULSE_MAX) pulse_us = SERVO_PULSE_MAX;

    /* 转换为12位计数值 */
    uint16_t off_count = pulse_to_count(pulse_us, pwm_period_us);
    
    /* 写入LEDn_OFF寄存器
     * PCA9685_LED0_ON_L + 4*channel 为起始寄存器
     * 写入4字节：ON_L, ON_H, OFF_L, OFF_H
     * ON=0（立即从0开始输出）
     */
    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    uint8_t buf[4] = {
        0,                                  // ON_L = 0
        0,                                  // ON_H = 0
        (uint8_t)(off_count & 0xFF),        // OFF_L
        (uint8_t)((off_count >> 8) & 0x0F)  // OFF_H
    };
    
    return pca9685_write_burst(pca_addr, reg, buf, 4);
}

bool pca9685_set_servo_pulses(const uint8_t *servo_ids, 
                              const uint16_t *pulse_us,
                              uint8_t count)
{
    if (!servo_ids || !pulse_us || count == 0) return false;
    
    bool all_ok = true;
    
    /* 为效率考虑，按PCA9685分组写入 */
    /* 方案：遍历每个请求的舵机，分别写入对应的PCA9685 */
    for (uint8_t i = 0; i < count; i++) {
        if (!pca9685_set_servo_pulse(servo_ids[i], pulse_us[i])) {
            all_ok = false;
        }
    }
    
    return all_ok;
}

void pca9685_free_all(void)
{
    uint8_t data[65];
    data[0] = PCA9685_LED0_ON_L;
    memset(data + 1, 0, 64);

    for (uint8_t i = 0; i < g_board_count; i++) {
        i2c_write_blocking(PCA9685_I2C_INSTANCE, g_board_addrs[i], data, sizeof(data), false);
    }
}

/* ==================== I2C 总线恢复 ==================== */

/**
 * @brief I2C 总线恢复：当 SDA 被从设备拉死时，发送 9 个 SCL 时钟脉冲释放总线
 *
 * 标准 I2C 恢复流程（来自 I2C 规范 3.1.16）：
 *   1. 将 SCL/SDA 临时配置为 GPIO 开漏输出
 *   2. 发送最多 9 个 SCL 脉冲，让卡死的从设备完成未完成的字节传输
 *   3. 发送 STOP 条件 (SDA 低→高 当 SCL 为高)
 *   4. 重新初始化 I2C 外设并恢复引脚功能
 *
 * 调用时机：连续多次 I2C 写入失败后。
 */
void pca9685_i2c_recover(void)
{
    printf("[I2C] Bus recovery started...\r\n");

    /* 禁用 I2C 外设，释放引脚 */
    i2c_deinit(PCA9685_I2C_INSTANCE);

    /* 将 SDA/SCL 配置为 GPIO 开漏输出（模拟 I2C 电气特性） */
    gpio_init(PCA9685_I2C_SDA_PIN);
    gpio_init(PCA9685_I2C_SCL_PIN);
    gpio_set_dir(PCA9685_I2C_SDA_PIN, GPIO_OUT);
    gpio_set_dir(PCA9685_I2C_SCL_PIN, GPIO_OUT);
    gpio_put(PCA9685_I2C_SDA_PIN, 1);
    gpio_put(PCA9685_I2C_SCL_PIN, 1);
    sleep_us(10);

    /* 检查 SDA 是否确实被拉低（被从设备卡住） */
    gpio_set_dir(PCA9685_I2C_SDA_PIN, GPIO_IN);
    bool sda_stuck = !gpio_get(PCA9685_I2C_SDA_PIN);
    gpio_set_dir(PCA9685_I2C_SDA_PIN, GPIO_OUT);
    gpio_put(PCA9685_I2C_SDA_PIN, 1);

    if (sda_stuck) {
        printf("[I2C] SDA stuck LOW, sending recovery clocks...\r\n");

        /* 发送最多 9 个 SCL 脉冲。
         * 每个脉冲让从设备释放 1 bit。9 个脉冲 = 完整 1 字节 + ACK。
         * 在任意时刻 SDA 被释放，剩余的脉冲无害。 */
        for (int i = 0; i < 9; i++) {
            gpio_put(PCA9685_I2C_SCL_PIN, 0);
            sleep_us(20);
            gpio_put(PCA9685_I2C_SCL_PIN, 1);
            sleep_us(20);

            /* 检查 SDA 是否已释放 */
            gpio_set_dir(PCA9685_I2C_SDA_PIN, GPIO_IN);
            bool released = gpio_get(PCA9685_I2C_SDA_PIN);
            gpio_set_dir(PCA9685_I2C_SDA_PIN, GPIO_OUT);
            if (released) {
                printf("[I2C] SDA released after %d clocks\r\n", i + 1);
                break;
            }
        }
    }

    /* 发送 STOP 条件：SDA 从低跳高，SCL 保持高 */
    gpio_put(PCA9685_I2C_SCL_PIN, 1);
    sleep_us(10);
    gpio_put(PCA9685_I2C_SDA_PIN, 0);
    sleep_us(10);
    gpio_put(PCA9685_I2C_SDA_PIN, 1);
    sleep_us(10);

    /* 重新初始化 I2C 外设 */
    i2c_init(PCA9685_I2C_INSTANCE, PCA9685_I2C_BAUD);
    gpio_set_function(PCA9685_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PCA9685_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PCA9685_I2C_SDA_PIN);
    gpio_pull_up(PCA9685_I2C_SCL_PIN);

    printf("[I2C] Bus recovery complete\r\n");
}

/* ==================== 调试工具 ==================== */

void pca9685_scan_bus(void)
{
    printf("Scanning I2C bus...\r\n");
    
    for (uint8_t addr = 0; addr < 128; addr++) {
        uint8_t dummy;
        int ret = i2c_read_blocking(PCA9685_I2C_INSTANCE, addr, &dummy, 1, false);
        if (ret > 0) {
            printf("  Device found at 0x%02X\r\n", addr);
        }
    }
    
    printf("Scan complete.\r\n");
}

/**
 * @file hexapod_i2c_protocol.h
 * @brief I2C舵机驱动 - PCA9685控制
 * @note Pico作为I2C Master，通过两个PCA9685驱动18个舵机
 *       第一个PCA9685(地址0x40): RR, RM, RF 三腿 (9通道)
 *       第二个PCA9685(地址0x41): LR, LM, LF 三腿 (9通道)
 *       此模块只负责舵机PWM输出，不传输其他任何非运动信号
 */

#ifndef HEXAPOD_I2C_PROTOCOL_H
#define HEXAPOD_I2C_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== I2C硬件配置 ==================== */

#define PCA9685_I2C_INSTANCE    i2c1
#define PCA9685_I2C_BAUD        400000
#define PCA9685_I2C_SDA_PIN     2
#define PCA9685_I2C_SCL_PIN     3
#define PCA9685_I2C_TIMEOUT_US  5000

/* ==================== PCA9685器件配置 ==================== */

#define PCA9685_ADDR_0x40       0x40        /* 第一片：右腿组 */
#define PCA9685_ADDR_0x41       0x41        /* 第二片：左腿组 */

#define PCA9685_MODE1           0x00
#define PCA9685_MODE2           0x01
#define PCA9685_PRE_SCALE       0xFE
#define PCA9685_LED0_ON_L       0x06

#define PCA9685_MODE1_RESTART   (1 << 7)
#define PCA9685_MODE1_EXTCLK    (1 << 6)
#define PCA9685_MODE1_AI        (1 << 5)
#define PCA9685_MODE1_SLEEP     (1 << 4)
#define PCA9685_MODE2_OUTDRV    (1 << 2)

/* ==================== 舵机参数 ==================== */

/* PWM 实际周期 (微秒)。
 *
 * PCA9685 计数器 = 脉宽目标 × 4096 / PWM 周期。
 * 不管 PCA9685 振荡器偏差多大, 用示波器实测周期填在此处,
 * 代码直接按实际周期反算计数值, 保证输出的脉宽绝对准确。
 *
 *   50Hz 标准: 20000 μs
 *   58Hz 实测: 17241 μs  (1,000,000 / 58)
 *
 * 不要通过改 PCA9685_FREQUENCY 来补偿 — 那是间接修正。
 * 改这个值直接修正脉宽。 */
#define PWM_PERIOD_US           8475   /* 用示波器实测后修改 */

#define PCA9685_FREQUENCY       100      /* 目标频率, 不改 */
#define SERVO_PULSE_MIN         500
#define SERVO_PULSE_MAX         2500
#define PCA9685_RESOLUTION      4096

/* ==================== 舵机映射 ==================== */

#define PCA9685_1_SERVO_MIN     0
#define PCA9685_1_SERVO_MAX     8
#define PCA9685_2_SERVO_MIN     9
#define PCA9685_2_SERVO_MAX     17
#define SERVO_TOTAL_COUNT       18

/* ==================== 函数声明 ==================== */

bool pca9685_init(void);
bool pca9685_set_freq(uint16_t freq);
bool pca9685_set_servo_pulse(uint8_t servo_id, uint16_t pulse_us);
bool pca9685_set_servo_pulses(const uint8_t *servo_ids, 
                              const uint16_t *pulse_us,
                              uint8_t count);
void pca9685_free_all(void);
uint16_t pca9685_angle_to_pulse(int16_t angle);
bool pca9685_write_all_channels(uint8_t addr, const uint16_t *pulses, uint8_t count);
void pca9685_scan_bus(void);
uint8_t pca9685_get_board_count(void);
uint8_t pca9685_get_board_addr(uint8_t index);

#endif /* HEXAPOD_I2C_PROTOCOL_H */

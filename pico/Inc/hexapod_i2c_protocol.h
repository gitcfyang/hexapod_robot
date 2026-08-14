/**
 * @file hexapod_i2c_protocol.h
 * @brief I2C舵机驱动 - PCA9685控制
 * @note Pico作为I2C Master，通过两个PCA9685驱动18个舵机
 *       PCB 定版地址分配:
 *       0x40 (ADDR=GND): 左侧腿组 LR, LM, LF (LED7~LED15)
 *       0x41 (ADDR=VCC): 右侧腿组 RR, RM, RF (LED8~LED0)
 *       此模块只负责舵机PWM输出，不传输其他任何非运动信号
 */

#ifndef HEXAPOD_I2C_PROTOCOL_H
#define HEXAPOD_I2C_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== I2C硬件配置 ==================== */

#define PCA9685_I2C_INSTANCE    i2c1
#define PCA9685_I2C_BAUD        400000
#define PCA9685_I2C_SDA_PIN     14   /* GP14: PCB 走线靠近 PCA9685，减少干扰 */
#define PCA9685_I2C_SCL_PIN     15   /* GP15: PCB 走线靠近 PCA9685，减少干扰 */
#define PCA9685_I2C_TIMEOUT_US  5000

/* ==================== PCA9685器件配置 ==================== */

#define PCA9685_ADDR_LEFT       0x40        /* PCB 定版: 左腿组 (ADDR=GND) */
#define PCA9685_ADDR_RIGHT      0x41        /* PCB 定版: 右腿组 (ADDR=VCC) */

/* 兼容旧命名 */
#define PCA9685_ADDR_0x40       0x40
#define PCA9685_ADDR_0x41       0x41

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

/* 每片 PCA9685 的 PWM 实际周期 (微秒)。
 *
 * 两片 PCA9685 的内部振荡器可能有偏差，所以各自用示波器实测后
 * 填入独立的校准值。代码根据目标板自动选择对应周期。
 *
 * PCA9685 计数器 = 脉宽目标 × 4096 / PWM 周期。
 * 直接按实际周期反算计数值, 保证输出的脉宽绝对准确。
 *
 *   200Hz 标称: 5000 μs
 *   实测值取决于每片 PCA9685 的振荡器精度 (~±15%)。
 *
 * 不要通过改 PCA9685_FREQUENCY 来补偿 — 那是间接修正。
 * 改这两个值直接修正脉宽。 */
// 8475  //4310
#define PWM_PERIOD_US_BOARD0    9500   /* 板 0x40 (左腿板) 示波器实测后修改 */
#define PWM_PERIOD_US_BOARD1    9500   /* 板 0x41 (右腿板) 示波器实测后修改 */

#define PCA9685_FREQUENCY       100      /* 目标频率, 100Hz 匹配控制循环 */
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
void pca9685_i2c_recover(void);
uint8_t pca9685_get_board_count(void);
uint8_t pca9685_get_board_addr(uint8_t index);
uint16_t pca9685_get_pwm_period_us(uint8_t board_idx);

/**
 * @brief 舵机 ID → PCA9685 物理通道号映射 (PCB 定版)
 * @param servo_id 舵机 ID (0-17)
 * @return PCA9685 物理通道号 (0-15, 即 LEDn 编号)
 *
 * PCB 布线决定了舵机插座到 PCA9685 通道的物理顺序:
 *   左侧板 (0x40, ID 9~17 LR/LM/LF): 从前至后 LED7~LED15
 *     LF: LED7,8,9  → LM: LED10,11,12 → LR: LED13,14,15
 *   右侧板 (0x41, ID 0~8  RR/RM/RF): 从前至后 LED8~LED0
 *     RF: LED8,7,6  → RM: LED5,4,3    → RR: LED2,1,0
 */
uint8_t pca9685_servo_to_channel(uint8_t servo_id);

/**
 * @brief 按 I2C 地址查找板索引
 * @param addr PCA9685 I2C 地址 (0x40 或 0x41)
 * @return 板索引 (0/1), 未找到返回 0xFF
 */
uint8_t pca9685_get_board_idx_by_addr(uint8_t addr);

/**
 * @brief 运行时设置某块板的 PWM 周期校准值 (µs)
 * @param board_idx 板索引 (0=0x40 左腿板, 1=0x41 右腿板)
 * @param period_us 实测 PWM 周期 (限幅 5000~15000 µs)
 * @note 用于 !PER 校准模式: 设置后立即对后续脉冲计算生效,
 *       校准完成后将最终值填入 hexapod_i2c_protocol.h 重新编译
 */
void pca9685_set_pwm_period_us(uint8_t board_idx, uint16_t period_us);

#endif /* HEXAPOD_I2C_PROTOCOL_H */

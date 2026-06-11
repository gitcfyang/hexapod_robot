/**
 * @file bno055.h
 * @brief BNO055 9轴IMU传感器驱动接口
 *
 * 通过 I2C 读取融合后的欧拉角，用于机身姿态补偿。
 * 与 PCA9685 共享 i2c1 总线 (GP2/GP3, 400kHz)。
 */

#ifndef BNO055_H
#define BNO055_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== I2C 地址 ==================== */
#define BNO055_I2C_ADDR_DEFAULT    0x28    /* COM3=低电平 (默认) */
#define BNO055_I2C_ADDR_ALT        0x29    /* COM3=高电平 */

/* ==================== 寄存器地址 (Page 0) ==================== */

/* ---- 标识寄存器 ---- */
#define BNO055_REG_CHIP_ID          0x00    /* 芯片 ID, 应为 0xA0 */
#define BNO055_REG_ACC_ID           0x01    /* 加速度计 ID, 应为 0xFB */
#define BNO055_REG_MAG_ID           0x02    /* 磁力计 ID, 应为 0x32 */
#define BNO055_REG_GYR_ID           0x03    /* 陀螺仪 ID, 应为 0x0F */
#define BNO055_REG_SW_REV_ID_LSB    0x04    /* 软件版本 LSB */
#define BNO055_REG_SW_REV_ID_MSB    0x05    /* 软件版本 MSB */
#define BNO055_REG_BL_REV_ID        0x06    /* Bootloader 版本 */
#define BNO055_REG_PAGE_ID          0x07    /* 寄存器页选择 */

/* ---- 欧拉角数据 (只读) ---- */
#define BNO055_REG_EUL_HEADING_LSB  0x1A    /* 航向/Yaw (X) LSB */
#define BNO055_REG_EUL_HEADING_MSB  0x1B    /* 航向/Yaw (X) MSB */
#define BNO055_REG_EUL_ROLL_LSB     0x1C    /* 横滚/Roll (Y) LSB */
#define BNO055_REG_EUL_ROLL_MSB     0x1D    /* 横滚/Roll (Y) MSB */
#define BNO055_REG_EUL_PITCH_LSB    0x1E    /* 俯仰/Pitch (Z) LSB */
#define BNO055_REG_EUL_PITCH_MSB    0x1F    /* 俯仰/Pitch (Z) MSB */

/* ---- 四元数数据 (只读, 备选) ---- */
#define BNO055_REG_QUA_W_LSB        0x20
#define BNO055_REG_QUA_W_MSB        0x21
#define BNO055_REG_QUA_X_LSB        0x22
#define BNO055_REG_QUA_X_MSB        0x23
#define BNO055_REG_QUA_Y_LSB        0x24
#define BNO055_REG_QUA_Y_MSB        0x25
#define BNO055_REG_QUA_Z_LSB        0x26
#define BNO055_REG_QUA_Z_MSB        0x27

/* ---- 状态与控制寄存器 ---- */
#define BNO055_REG_CALIB_STAT       0x35    /* 校准状态 */
#define BNO055_REG_SYS_STATUS       0x39    /* 系统状态 */
#define BNO055_REG_SYS_ERR          0x3A    /* 系统错误码 */
#define BNO055_REG_UNIT_SEL         0x3B    /* 单位选择 */
#define BNO055_REG_OPR_MODE         0x3D    /* 工作模式 */
#define BNO055_REG_PWR_MODE         0x3E    /* 电源模式 */
#define BNO055_REG_SYS_TRIGGER      0x3F    /* 系统触发 */

/* ---- 温度 ---- */
#define BNO055_REG_TEMP             0x34    /* 温度 (1°C/LSB) */

/* ==================== 工作模式 (OPR_MODE) ==================== */
#define BNO055_MODE_CONFIG          0x00    /* 配置模式 (仅此模式可写寄存器) */
#define BNO055_MODE_ACC_ONLY        0x01    /* 仅加速度计 */
#define BNO055_MODE_MAG_ONLY        0x02    /* 仅磁力计 */
#define BNO055_MODE_GYRO_ONLY       0x03    /* 仅陀螺仪 */
#define BNO055_MODE_ACC_MAG         0x04    /* 加速度计+磁力计 */
#define BNO055_MODE_ACC_GYRO        0x05    /* 加速度计+陀螺仪 */
#define BNO055_MODE_MAG_GYRO        0x06    /* 磁力计+陀螺仪 */
#define BNO055_MODE_AMG             0x07    /* 三传感器原始数据 */
#define BNO055_MODE_IMU             0x08    /* IMU: 加速度计+陀螺仪融合 (相对姿态) */
#define BNO055_MODE_COMPASS         0x09    /* 罗盘: 加速度计+磁力计融合 (绝对航向) */
#define BNO055_MODE_M4G             0x0A    /* 磁力计辅助陀螺仪 */
#define BNO055_MODE_NDOF_FMC_OFF    0x0B    /* 9-DOF 融合, 快速磁校准关闭 */
#define BNO055_MODE_NDOF            0x0C    /* 9-DOF 全融合 (推荐, 绝对姿态) */

/* ==================== 电源模式 (PWR_MODE) ==================== */
#define BNO055_PWR_NORMAL           0x00    /* 正常模式 */
#define BNO055_PWR_LOW              0x01    /* 低功耗 */
#define BNO055_PWR_SUSPEND          0x02    /* 挂起 */

/* ==================== 系统触发 (SYS_TRIGGER) ==================== */
#define BNO055_TRIG_RST_SYS         (1 << 5)  /* 系统复位 */
#define BNO055_TRIG_RST_INT         (1 << 6)  /* 中断复位 */
#define BNO055_TRIG_SELF_TEST       (1 << 0)  /* 自检 */

/* ==================== 校准状态位 (CALIB_STAT) ==================== */
#define BNO055_CALIB_MASK_SYS       0xC0    /* 系统校准 [7:6] */
#define BNO055_CALIB_MASK_GYR       0x30    /* 陀螺仪校准 [5:4] */
#define BNO055_CALIB_MASK_ACC       0x0C    /* 加速度计校准 [3:2] */
#define BNO055_CALIB_MASK_MAG       0x03    /* 磁力计校准 [1:0] */
#define BNO055_CALIB_FULLY          0xFF    /* 全部校准完成 (各字段 = 3) */

/* ==================== 芯片 ID 期望值 ==================== */
#define BNO055_CHIP_ID_VAL          0xA0
#define BNO055_ACC_ID_VAL           0xFB
#define BNO055_MAG_ID_VAL           0x32
#define BNO055_GYR_ID_VAL           0x0F

/* ==================== 时序常量 (ms) ==================== */
#define BNO055_POR_WAIT_MS          650     /* 上电到 I2C 就绪 */
#define BNO055_MODE_SWITCH_MS       19      /* 模式切换等待 */
#define BNO055_CONFIG_SWITCH_MS     7       /* CONFIG 模式内切换等待 */

/* ==================== 数据结构 ==================== */

/**
 * @brief BNO055 欧拉角数据 (原始 int16, 1° = 16 LSB)
 */
typedef struct {
    int16_t heading;    /* 航向 (绕 Z 轴): 0 ~ 360°    */
    int16_t roll;       /* 横滚 (绕 Y 轴): -180° ~ 180° */
    int16_t pitch;      /* 俯仰 (绕 X 轴): -90° ~ 90°   */
} bno055_euler_t;

/**
 * @brief BNO055 校准状态
 */
typedef struct {
    uint8_t sys;        /* 系统校准: 0~3 */
    uint8_t gyro;       /* 陀螺仪校准: 0~3 */
    uint8_t accel;      /* 加速度计校准: 0~3 */
    uint8_t mag;        /* 磁力计校准: 0~3 */
} bno055_calib_t;

/* ==================== 函数声明 ==================== */

/**
 * @brief 初始化 BNO055
 *
 * 执行上电等待、芯片 ID 验证、切换到 NDOF 融合模式。
 *
 * @param i2c_addr  I2C 7-bit 地址 (通常 0x28)
 * @return true 初始化成功
 */
bool bno055_init(uint8_t i2c_addr);

/**
 * @brief 读取欧拉角 (原始 int16)
 *
 * 突发读取 6 字节从 EUL_HEADING_LSB (0x1A) 到 EUL_PITCH_MSB (0x1F)。
 *
 * @param euler  输出欧拉角 (1° = 16 LSB)
 * @return true 读取成功
 */
bool bno055_read_euler(bno055_euler_t *euler);

/**
 * @brief 读取校准状态
 *
 * @param calib  输出校准状态
 * @return true 读取成功
 */
bool bno055_get_calib(bno055_calib_t *calib);

/**
 * @brief 检查传感器是否完成全部校准
 *
 * @return true 系统、陀螺仪、加速度计、磁力计全部校准 (CALIB_STAT = 0xFF)
 */
bool bno055_is_calibrated(void);

#ifdef __cplusplus
}
#endif

#endif /* BNO055_H */

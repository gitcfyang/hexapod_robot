/**
 * @file bno055.c
 * @brief BNO055 9轴IMU传感器 I2C 驱动实现
 *
 * 与 PCA9685 共享 i2c1 总线 (GP2/GP3, 400kHz)。
 * I2C 读写模式与 pca9685_read_reg() 一致：写寄存器地址 (1 字节) + 读数据。
 */

#include "bno055.h"
#include "hexapod_i2c_protocol.h"   /* PCA9685_I2C_INSTANCE, I2C 超时等 */
#include "hardware/i2c.h"

/* 缓存当前 I2C 地址和校准状态 */
static uint8_t  g_bno055_addr = 0;
static uint8_t  g_calib_stat  = 0;
static bool     g_initialized = false;

/* ==================== 内部辅助函数 ==================== */

/**
 * @brief 写单个寄存器 (无数据)
 */
static inline bool bno055_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    int ret = i2c_write_blocking(PCA9685_I2C_INSTANCE, g_bno055_addr,
                                  buf, 2, false);
    return (ret == 2);
}

/**
 * @brief 读单个寄存器
 */
static inline bool bno055_read_reg(uint8_t reg, uint8_t *data)
{
    int ret = i2c_write_blocking(PCA9685_I2C_INSTANCE, g_bno055_addr,
                                  &reg, 1, true);   /* nostop=true: 保持总线 */
    if (ret != 1) return false;
    ret = i2c_read_blocking(PCA9685_I2C_INSTANCE, g_bno055_addr,
                             data, 1, false);        /* STOP 后释放总线 */
    return (ret == 1);
}

/**
 * @brief 突发读取多个寄存器
 */
static inline bool bno055_read_burst(uint8_t start_reg, uint8_t *data, uint8_t len)
{
    int ret = i2c_write_blocking(PCA9685_I2C_INSTANCE, g_bno055_addr,
                                  &start_reg, 1, true);   /* nostop=true */
    if (ret != 1) return false;
    ret = i2c_read_blocking(PCA9685_I2C_INSTANCE, g_bno055_addr,
                             data, len, false);
    return (ret == len);
}

/* ==================== 公开函数 ==================== */

bool bno055_init(uint8_t i2c_addr)
{
    g_bno055_addr = i2c_addr;
    g_initialized = false;
    g_calib_stat  = 0;

    /* 上电等待：BNO055 上电后需 650ms 才能响应 I2C */
    sleep_ms(BNO055_POR_WAIT_MS);

    /* 验证芯片 ID */
    uint8_t chip_id = 0;
    if (!bno055_read_reg(BNO055_REG_CHIP_ID, &chip_id)) {
        return false;
    }
    if (chip_id != BNO055_CHIP_ID_VAL) {
        return false;   /* 未检测到 BNO055 或地址错误 */
    }

    /* 确保在 Page 0 */
    if (!bno055_write_reg(BNO055_REG_PAGE_ID, 0x00)) {
        return false;
    }

    /* 切换到 CONFIG 模式 (只有此模式可写其他寄存器) */
    if (!bno055_write_reg(BNO055_REG_OPR_MODE, BNO055_MODE_CONFIG)) {
        return false;
    }
    sleep_ms(BNO055_CONFIG_SWITCH_MS);

    /* 设置电源模式为 Normal */
    if (!bno055_write_reg(BNO055_REG_PWR_MODE, BNO055_PWR_NORMAL)) {
        return false;
    }

    /* 使用默认单位: 欧拉角=度, 加速度=m/s², 角速度=dps, 温度=°C
     * UNIT_SEL 默认为 0x00 (Windows 方向约定), 不需要修改。 */

    /* 配置 INT 引脚: 融合数据就绪中断 (ACC_BSX_DRDY)
     * - INT_MSK/INT_EN bit7 使能并路由到 INT 引脚
     * - NDOF 模式下按融合输出速率触发 (~100Hz)
     * - INT_CNTL 在 Page 1 (0x07): 空闲高 + 下降沿触发
     * 主机侧 (GP28) 用双沿中断接收, 对极性不敏感 */
    if (!bno055_write_reg(BNO055_REG_INT_MSK, BNO055_INT_BSX_DRDY)) return false;
    if (!bno055_write_reg(BNO055_REG_INT_EN,  BNO055_INT_BSX_DRDY)) return false;
    if (!bno055_write_reg(BNO055_REG_PAGE_ID, 0x01)) return false;      /* 切 Page 1 */
    if (!bno055_write_reg(BNO055_REG_INT_CNTL, BNO055_INT_CNTL_IDLE_H_FALL)) return false;
    if (!bno055_write_reg(BNO055_REG_PAGE_ID, 0x00)) return false;      /* 回 Page 0 */

    /* 切换到 NDOF 融合模式 (9-DOF 绝对姿态) */
    if (!bno055_write_reg(BNO055_REG_OPR_MODE, BNO055_MODE_NDOF)) {
        return false;
    }
    sleep_ms(BNO055_MODE_SWITCH_MS);

    g_initialized = true;
    return true;
}

bool bno055_read_euler(bno055_euler_t *euler)
{
    if (!g_initialized || !euler) {
        return false;
    }

    /* 突发读取 6 字节: HEADING_LSB ~ PITCH_MSB (0x1A ~ 0x1F) */
    uint8_t buf[6];
    if (!bno055_read_burst(BNO055_REG_EUL_HEADING_LSB, buf, 6)) {
        return false;
    }

    /* 小端序组装 int16: LSB | (MSB << 8) */
    euler->heading = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    euler->roll    = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    euler->pitch   = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));

    return true;
}

bool bno055_get_calib(bno055_calib_t *calib)
{
    if (!g_initialized || !calib) {
        return false;
    }

    uint8_t stat;
    if (!bno055_read_reg(BNO055_REG_CALIB_STAT, &stat)) {
        return false;
    }

    g_calib_stat = stat;
    calib->sys   = (stat >> 6) & 0x03;
    calib->gyro  = (stat >> 4) & 0x03;
    calib->accel = (stat >> 2) & 0x03;
    calib->mag   = (stat >> 0) & 0x03;

    return true;
}

bool bno055_is_calibrated(void)
{
    /* 读取最新校准状态 */
    uint8_t stat;
    if (!bno055_read_reg(BNO055_REG_CALIB_STAT, &stat)) {
        return false;
    }
    g_calib_stat = stat;
    return (stat == BNO055_CALIB_FULLY);
}

bool bno055_get_int_status(uint8_t *sta)
{
    if (!g_initialized || !sta) return false;
    return bno055_read_reg(BNO055_REG_INT_STA, sta);
}

bool bno055_get_int_config(uint8_t *en, uint8_t *msk, uint8_t *cntl)
{
    if (!g_initialized || !en || !msk || !cntl) return false;

    if (!bno055_read_reg(BNO055_REG_INT_EN,  en))  return false;
    if (!bno055_read_reg(BNO055_REG_INT_MSK, msk)) return false;

    /* INT_CNTL 在 Page 1: 切页读回再切回 */
    if (!bno055_write_reg(BNO055_REG_PAGE_ID, 0x01)) return false;
    bool ok = bno055_read_reg(BNO055_REG_INT_CNTL, cntl);
    bno055_write_reg(BNO055_REG_PAGE_ID, 0x00);
    return ok;
}

bool bno055_get_sw_rev(uint16_t *rev)
{
    if (!g_initialized || !rev) return false;

    uint8_t lsb, msb;
    if (!bno055_read_reg(BNO055_REG_SW_REV_ID_LSB, &lsb)) return false;
    if (!bno055_read_reg(BNO055_REG_SW_REV_ID_MSB, &msb)) return false;

    *rev = ((uint16_t)msb << 8) | lsb;
    return true;
}

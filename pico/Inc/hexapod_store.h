/**
 * @file hexapod_store.h
 * @brief 非易失存储模块: 舵机校准参数 + 运行事件日志 (RP2040 板载 QSPI Flash)
 *
 * 扇区布局 (2MB Flash 末两扇区, 与固件代码区间隔约 1MB):
 *   0x1FE000 (4KB): 校准记录 store_calib_record_t — 仅 !SAVE 时整扇区擦除+写入
 *   0x1FF000 (4KB): 事件日志环形缓冲      — 页编程追加, 仅 !LOGC 时擦除
 *
 * 写安全性 (单核 + XIP 关键约束):
 *   flash_range_erase/program 期间 XIP 停摆, 所有取指必须来自 RAM。
 *   本模块写路径全部置于 __no_inline_not_in_flash_func, 关中断执行,
 *   并在擦除与编程之间直接写 watchdog_hw->load 喂狗
 *   (watchdog_update() 本身在 flash 中, 此窗口内不可调用)。
 *   CRC32 采用位操作实现, 避免查表访问 .rodata 触发 XIP 取指。
 *
 * 调用策略:
 *   - 整扇区擦除仅在安全上下文 (校准模式 / 未解锁 / 启动期) 允许;
 *   - 行走中事件只进 RAM 缓冲, 由主循环在安全上下文回写。
 */
#ifndef HEXAPOD_STORE_H
#define HEXAPOD_STORE_H

#include <stdint.h>
#include <stdbool.h>
#include "hexapod_core.h"

/* ==================== 扇区与记录布局 ==================== */

#define STORE_CALIB_FLASH_OFFSET   0x1FE000u   /* 校准记录扇区 (XIP 偏移) */
#define STORE_LOG_FLASH_OFFSET     0x1FF000u   /* 事件日志扇区 */
#define STORE_SECTOR_SIZE          4096u       /* W25Q16 扇区大小 */
#define STORE_PAGE_SIZE            256u        /* 编程页大小 */
#define STORE_LOG_ENTRY_SIZE       8u
#define STORE_LOG_ENTRIES_PER_PAGE (STORE_PAGE_SIZE / STORE_LOG_ENTRY_SIZE)    /* 32 */
#define STORE_LOG_MAX_ENTRIES      (STORE_SECTOR_SIZE / STORE_LOG_ENTRY_SIZE)  /* 512 */
#define STORE_LOG_PAGES            (STORE_SECTOR_SIZE / STORE_PAGE_SIZE)       /* 16 */

#define STORE_PENDING_MAX          16u         /* 行走中 RAM 缓冲上限 */
#define STORE_CALIB_MAGIC          0x43584548u /* "HEXC" (小端) */
#define STORE_CALIB_VERSION        1u

/* ==================== 数据结构 ==================== */

/** 校准记录: 56 字节, 位于校准扇区偏移 0 (编程时按整页 256B 写入) */
typedef struct __attribute__((packed)) {
    uint32_t magic;              /* STORE_CALIB_MAGIC */
    uint16_t version;            /* STORE_CALIB_VERSION */
    uint16_t reserved;           /* 对齐填充, 恒 0 */
    int16_t  horn_offsets[18];   /* 舵盘偏移 (0.1°), ID = leg*3 + joint */
    uint32_t done_mask;          /* bit i = 保存时 g_calib_done[i] */
    uint16_t pwm_period_us[2];   /* PCA9685 左/右板 PWM 周期 (us) */
    uint32_t crc32;              /* 覆盖本字段之前的全部字节 */
} store_calib_record_t;

/** 事件日志条目: 8 字节, 32 条/页, 512 条/扇区 */
typedef struct __attribute__((packed)) {
    uint8_t  type;               /* store_event_t; 0xFF = 已擦除 (无效) */
    uint8_t  flags;              /* 保留, 恒 0 */
    uint16_t data;               /* 事件数据 (电压 mV / 复位原因 / 计数) */
    uint32_t t_ms;               /* to_ms_since_boot 时间戳 */
} store_log_entry_t;

/** 事件类型 */
typedef enum {
    STORE_EVT_BOOT = 1,          /* data = watchdog_hw->reason */
    STORE_EVT_BATT_OV,           /* data = 电压 mV (过压) */
    STORE_EVT_BATT_CUTOFF,       /* data = 电压 mV (过放截止) */
    STORE_EVT_BATT_WARN,         /* data = 电压 mV (低压警告) */
    STORE_EVT_BATT_RECOVER,      /* data = 电压 mV (电压恢复) */
    STORE_EVT_CALIB_SAVED,       /* data = 已保存舵机数 */
    STORE_EVT_LOG_CLEARED,       /* data = 0 */
} store_event_t;

/* ==================== API ==================== */

/**
 * @brief 初始化: 扫描日志环头, 记录 BOOT 事件
 * @note  须在 robot_init() 之前调用 (看门狗尚未启用, 写安全)
 */
void store_init(void);

/**
 * @brief 读取并校验校准记录到内部缓存
 * @return true = 记录有效且已通过 CRC 校验
 */
bool store_load(void);

/**
 * @brief 将已加载的校准值写入运行时配置 (舵盘偏移 + PWM 周期)
 * @param robot 机器人实例 (须已 robot_init)
 */
void store_apply_to_robot(hexapod_t *robot);

/**
 * @brief 保存当前校准数据到 Flash (!SAVE)
 * @param safe_context true = 机器人静止 (校准模式/未解锁/启动), 允许整扇区擦除
 * @return true = 写入并校验成功
 */
bool store_save_calib(bool safe_context);

/**
 * @brief 记录一条事件 (行走安全: 仅进 RAM 缓冲, 满则丢弃并计数)
 */
void store_log_event(uint8_t type, uint16_t data);

/**
 * @brief 把 RAM 缓冲的事件回写到 Flash 日志
 * @param safe_context true 时才实际写入
 */
void store_flush_pending(bool safe_context);

/** @brief 打印日志环全部条目 (!LOG) */
void store_dump_log(void);

/** @brief 清空日志扇区并写入 LOG_CLEARED 条目 (!LOGC) */
void store_clear_log(void);

/** @brief 查询校准记录是否已成功加载 (供启动提示) */
bool store_calib_loaded(void);

#endif /* HEXAPOD_STORE_H */

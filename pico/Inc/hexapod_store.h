/**
 * @file hexapod_store.h
 * @brief 非易失存储模块: 舵机校准参数 + 运行事件日志 (RP2040 板载 QSPI Flash)
 *
 * 扇区布局 (2MB Flash 末三扇区, 与固件代码区间隔约 1MB):
 *   0x1FD000 (4KB): 运行时参数 store_params_record_t — 仅 !CFGW 时整扇区擦除+写入
 *   0x1FE000 (4KB): 校准记录 store_calib_record_t    — 仅 !SAVE 时整扇区擦除+写入
 *   0x1FF000 (4KB): 事件日志环形缓冲                — 页编程追加, 仅 !LOGC 时擦除
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

#define STORE_PARAMS_FLASH_OFFSET  0x1FD000u   /* 运行时参数扇区 (XIP 偏移) */
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

#define STORE_PARAMS_MAGIC         0x50584548u /* "HEXP" (小端) */
/* 版本沿革:
 *   1: 原始布局
 *   2: 名字长度 12 → 20 (记录变大)
 *   3: STORE_PARAMS_MAX 48 → 64 (entries 变大, crc32 偏移随之改变)
 *   4: STORE_PARAMS_MAX 64 → 96 (输入参数统一 + 模式矩阵/组合键表)
 * 版本号一跳, 旧记录整体作废 (回落默认值) —— 本版默认值本就重定义过,
 * 与其把旧值套到新名字上, 不如让用户重存一次。 */
#define STORE_PARAMS_VERSION       4u
#define STORE_PARAMS_MAX           96u         /* 单次可存参数条数上限 */
/* 含结尾 NUL。必须 > 参数表里最长的名字 (当前 16: batt_interval_ms),
 * 否则名字被截断后既查不到 (!CFG 失效) 也读不回 (存了等于没存)。
 * params_init() 会在启动时校验, 服务器侧 PARAM_NAME_OK 也要同步。 */
#define STORE_PARAM_NAME_LEN       20u

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

/**
 * 运行时参数条目: 24 字节 ("名字 → 值")
 *
 * 刻意按名字而不是按索引存: 固件升级增删表项后, 旧记录仍能正确落到
 * 对应参数上 (表里没有的名字直接跳过)。按索引存的话, 表中间插一项
 * 就会把后面所有参数错位写坏。
 */
typedef struct __attribute__((packed)) {
    char    name[STORE_PARAM_NAME_LEN];
    int32_t value;
} store_param_entry_t;

/** 参数记录: 8 + 96×24 + 4 = 2316 字节 (按 10 个编程页 2560B 写入) */
typedef struct __attribute__((packed)) {
    uint32_t magic;                            /* STORE_PARAMS_MAGIC */
    uint16_t version;                          /* STORE_PARAMS_VERSION */
    uint16_t count;                            /* entries 中有效条数 */
    store_param_entry_t entries[STORE_PARAMS_MAX];
    uint32_t crc32;                            /* 覆盖本字段之前的全部字节 */
} store_params_record_t;

/** 参数记录占用的编程字节数 (向上取整到 256B 页边界) */
#define STORE_PARAMS_RECORD_BYTES \
    (((sizeof(store_params_record_t) + STORE_PAGE_SIZE - 1u) / STORE_PAGE_SIZE) * STORE_PAGE_SIZE)

/* 记录写不下就会溢出到校准扇区 —— 编译期挡住, 别等擦错了才发现 */
_Static_assert(STORE_PARAMS_RECORD_BYTES <= STORE_SECTOR_SIZE,
               "params record exceeds its flash sector");
_Static_assert((STORE_CALIB_FLASH_OFFSET - STORE_PARAMS_FLASH_OFFSET) == STORE_SECTOR_SIZE,
               "params sector must be exactly one sector wide");

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
    STORE_EVT_PARAM_SAVED,       /* data = 已保存参数条数 */
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
 *
 * 只套用记录里 done_mask 置位的舵机, 未校准的保持 robot_init() 设的
 * config.h 默认值 —— 校准到一半就保存是正常用法, 不能把没调过的腿冲掉。
 * @param robot 机器人实例 (须已 robot_init)
 * @return 实际套用的舵机数 (0 = 无有效记录 / 无已校准项)
 */
uint8_t store_apply_to_robot(hexapod_t *robot);

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

/**
 * @brief 保存运行时参数 (!CFGW)
 * @param entries  名字→值 数组 (仅需传入与默认值不同的项)
 * @param count    条数, 上限 STORE_PARAMS_MAX
 * @param safe_context true = 机器人静止, 允许整扇区擦除
 * @return true = 写入并校验成功
 */
bool store_save_params(const store_param_entry_t *entries, uint16_t count,
                       bool safe_context);

/**
 * @brief 读取参数记录并校验 CRC
 * @param out  接收缓冲 (容量 ≥ max 项)
 * @param max  缓冲容量
 * @return 实际读出的条数; 0 = 无有效记录 (首次上电 / CRC 错 / 版本不符)
 */
uint16_t store_load_params(store_param_entry_t *out, uint16_t max);

#endif /* HEXAPOD_STORE_H */

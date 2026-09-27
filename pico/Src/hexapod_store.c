/**
 * @file hexapod_store.c
 * @brief 非易失存储: 舵机校准参数 + 运行事件日志
 *
 * 设计要点见 hexapod_store.h 头部注释。
 *
 * 日志环形缓冲采用"页镜像"追加:
 *   每次写入读出当前 256B 页 → 补一条 8B 条目 → 整页重编程。
 *   已擦除字节 0xFF 与新值的组合始终是合法的 1→0 位转变,
 *   且重编程未触及字节 (保持 0xFF) 与相同值字节均符合 Flash 规范。
 *   仅 !LOGC 触发整扇区擦除 (须安全上下文)。
 */

#include "hexapod_store.h"

#include <string.h>

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/structs/watchdog.h"

#include "hexapod_hal.h"
#include "hexapod_i2c_protocol.h"

/* ==================== 内部状态 ==================== */

/** 校准记录缓存 (store_load 填充, store_apply_to_robot 消费) */
static store_calib_record_t s_calib;
static bool                 s_calib_valid = false;

/** 页镜像: 当前正在追加的日志页 (256B) */
static uint8_t  s_log_page[STORE_PAGE_SIZE];
static uint16_t s_log_page_idx = 0xFFFFu;  /* 镜像当前对应的页号, 0xFFFF = 无效 */

/** 日志写指针与状态 */
static uint16_t s_log_next  = 0;           /* 下一条待写条目索引 (0..511) */
static bool     s_log_full  = false;

/** 行走中事件 RAM 缓冲 (环形) */
static store_log_entry_t s_pending[STORE_PENDING_MAX];
static uint8_t           s_pending_head = 0;   /* 读指针 */
static uint8_t           s_pending_tail = 0;   /* 写指针 */
static uint8_t           s_pending_cnt  = 0;
static uint16_t          s_dropped      = 0;   /* 缓冲满丢弃计数 */

/* ==================== CRC32 (位操作, 无查表) ==================== */

/**
 * @brief 位操作 CRC32 (IEEE 802.3 多项式, 反射)
 * @note  刻意不用查表: 表在 .rodata (flash), XIP 停摆窗口中取指会挂死。
 *        56 字节 ≈ 4.5k 次迭代 ≈ 0.4ms, 相对数百 ms 的擦除可忽略。
 */
static uint32_t store_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1u));
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

/* ==================== Flash 写路径 (RAM 驻留) ==================== */

/**
 * @brief 喂看门狗 (RAM 版本)
 *
 * watchdog_update() 是 flash 中的函数, XIP 停摆期间不可调用,
 * 故直接写 LOAD 寄存器。10000000 = 5000ms x 1000 x 2
 * (RP2040-E1 errata: 实际周期为设定值的一半, SDK 同样乘 2)。
 * 覆盖启动期 5s 看门狗, 主循环 1.5s 更不在话下。
 */
static void __no_inline_not_in_flash_func(store_feed_watchdog)(void)
{
    watchdog_hw->load = 10000000u;
}

/**
 * @brief 整扇区擦除 + 编程前 len 字节 (仅安全上下文调用)
 * @param offset   Flash 偏移 (XIP 基址 0x10000000 之上, 此处传裸偏移)
 * @param buf      RAM 缓冲
 * @param len      编程长度, 必须是 256 的整数倍 (SDK 要求)
 */
static void __no_inline_not_in_flash_func(store_flash_erase_program_n)(
        uint32_t offset, const uint8_t *buf, uint32_t len)
{
    uint32_t irq = save_and_disable_interrupts();
    store_feed_watchdog();
    flash_range_erase(offset, STORE_SECTOR_SIZE);
    store_feed_watchdog();
    flash_range_program(offset, buf, len);
    restore_interrupts(irq);
}

/** @brief 整扇区擦除 + 编程首页 (256B) */
static void __no_inline_not_in_flash_func(store_flash_erase_program)(
        uint32_t offset, const uint8_t *page_buf)
{
    store_flash_erase_program_n(offset, page_buf, STORE_PAGE_SIZE);
}

/** @brief 仅页编程 (无擦除), 用于日志追加 */
static void __no_inline_not_in_flash_func(store_flash_program_page)(
        uint32_t offset, const uint8_t *page_buf)
{
    uint32_t irq = save_and_disable_interrupts();
    flash_range_program(offset, page_buf, STORE_PAGE_SIZE);
    restore_interrupts(irq);
}

/** @brief 仅整扇区擦除 (擦除后全 0xFF), 用于日志清空 (仅安全上下文调用) */
static void __no_inline_not_in_flash_func(store_flash_erase_only)(uint32_t offset)
{
    uint32_t irq = save_and_disable_interrupts();
    store_feed_watchdog();
    flash_range_erase(offset, STORE_SECTOR_SIZE);
    store_feed_watchdog();
    restore_interrupts(irq);
}

/** @brief 读 Flash 到 RAM (正常上下文, 走 XIP) */
static void store_flash_read(uint32_t offset, void *dst, uint32_t len)
{
    memcpy(dst, (const void *)(XIP_BASE + offset), len);
}

/* ==================== 日志页镜像 ==================== */

/** @brief 确保 s_log_page 镜像与指定页号一致 */
static void store_log_load_page(uint16_t page_idx)
{
    if (s_log_page_idx == page_idx) return;
    store_flash_read(STORE_LOG_FLASH_OFFSET + (uint32_t)page_idx * STORE_PAGE_SIZE,
                     s_log_page, STORE_PAGE_SIZE);
    s_log_page_idx = page_idx;
}

/**
 * @brief 追加一条日志条目到 Flash
 * @return true = 写入成功; false = 环满
 */
static bool store_log_append(uint8_t type, uint16_t data, uint32_t t_ms)
{
    if (s_log_full) return false;

    uint16_t page_idx = (uint16_t)(s_log_next / STORE_LOG_ENTRIES_PER_PAGE);
    if (page_idx >= STORE_LOG_PAGES) {
        s_log_full = true;
        return false;
    }

    store_log_load_page(page_idx);

    uint16_t slot = (uint16_t)(s_log_next % STORE_LOG_ENTRIES_PER_PAGE);
    uint8_t *p = &s_log_page[slot * STORE_LOG_ENTRY_SIZE];
    p[0] = type;
    p[1] = 0;
    p[2] = (uint8_t)(data & 0xFFu);
    p[3] = (uint8_t)(data >> 8);
    p[4] = (uint8_t)(t_ms & 0xFFu);
    p[5] = (uint8_t)((t_ms >> 8) & 0xFFu);
    p[6] = (uint8_t)((t_ms >> 16) & 0xFFu);
    p[7] = (uint8_t)((t_ms >> 24) & 0xFFu);

    store_flash_program_page(STORE_LOG_FLASH_OFFSET + (uint32_t)page_idx * STORE_PAGE_SIZE,
                             s_log_page);

    s_log_next++;
    if (s_log_next >= STORE_LOG_MAX_ENTRIES) {
        s_log_full = true;
    }
    return true;
}

/* ==================== 对外接口 ==================== */

void store_init(void)
{
    /* 扫描日志环头: 首个 type == 0xFF 即下一条待写位置。
     * 追加严格顺序且仅 !LOGC 重置, 线性扫描足够。 */
    store_log_entry_t e;
    uint16_t idx = 0;
    for (; idx < STORE_LOG_MAX_ENTRIES; idx++) {
        store_flash_read(STORE_LOG_FLASH_OFFSET + (uint32_t)idx * STORE_LOG_ENTRY_SIZE,
                         &e, STORE_LOG_ENTRY_SIZE);
        if (e.type == 0xFFu) break;
    }
    s_log_next = idx;
    s_log_full = (idx >= STORE_LOG_MAX_ENTRIES);
    s_log_page_idx = 0xFFFFu;   /* 使页镜像失效, 首次写入时重新读取 */

    hal_debug_printf("[STORE] Flash storage init, log %u/%u entries%s\r\n",
                     (unsigned)s_log_next, (unsigned)STORE_LOG_MAX_ENTRIES,
                     s_log_full ? " (FULL)" : "");

    /* BOOT 事件: 此阶段看门狗尚未启用, 写安全 */
    uint16_t reason = (uint16_t)(watchdog_hw->reason & 0xFFFFu);
    store_log_append(STORE_EVT_BOOT, reason, 0);
}

bool store_load(void)
{
    store_calib_record_t rec;
    store_flash_read(STORE_CALIB_FLASH_OFFSET, &rec, sizeof(rec));

    if (rec.magic != STORE_CALIB_MAGIC) {
        s_calib_valid = false;
        return false;
    }
    if (rec.version != STORE_CALIB_VERSION) {
        s_calib_valid = false;
        return false;
    }

    uint32_t crc_calc = store_crc32((const uint8_t *)&rec,
                                    (uint32_t)((const uint8_t *)&rec.crc32
                                               - (const uint8_t *)&rec));
    if (crc_calc != rec.crc32) {
        s_calib_valid = false;
        return false;
    }

    s_calib = rec;
    s_calib_valid = true;
    return true;
}

bool store_calib_loaded(void)
{
    return s_calib_valid;
}

void store_apply_to_robot(hexapod_t *robot)
{
    if (!robot || !s_calib_valid) return;

    for (int leg = 0; leg < CNT_LEGS; leg++) {
        robot->leg_configs[leg].coxa_horn_offset  = s_calib.horn_offsets[leg * 3 + 0];
        robot->leg_configs[leg].femur_horn_offset = s_calib.horn_offsets[leg * 3 + 1];
        robot->leg_configs[leg].tibia_horn_offset = s_calib.horn_offsets[leg * 3 + 2];
    }

    /* PWM 周期: 纯 RAM 写入 (PRE_SCALE 与周期值无关), 无 I2C 流量。
     * 舵机供电在解锁前关闭, 此处写入时序安全。
     * 上限取 min(板数, 记录容量) 防越界 (记录仅容纳 2 块板)。 */
    uint8_t nb = pca9685_get_board_count();
    if (nb > 2) nb = 2;
    for (uint8_t b = 0; b < nb; b++) {
        pca9685_set_pwm_period_us(b, s_calib.pwm_period_us[b]);
    }
}

bool store_save_calib(bool safe_context)
{
    if (!safe_context) {
        hal_debug_printf("[STORE] Refuse: robot is armed, disarm first (!S)\r\n");
        return false;
    }

    /* 从 HAL 导出当前校准数据 */
    int16_t  offsets[18];
    uint32_t done_mask = hal_calib_export(offsets);

    if (done_mask == 0u) {
        hal_debug_printf("[STORE] Refuse: no calibration data (run !C first)\r\n");
        return false;
    }

    store_calib_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic    = STORE_CALIB_MAGIC;
    rec.version  = STORE_CALIB_VERSION;
    rec.reserved = 0;
    memcpy(rec.horn_offsets, offsets, sizeof(rec.horn_offsets));
    rec.done_mask = done_mask;
    for (uint8_t b = 0; b < 2; b++) {
        rec.pwm_period_us[b] = pca9685_get_pwm_period_us(b);
    }
    rec.crc32 = store_crc32((const uint8_t *)&rec,
                            (uint32_t)((const uint8_t *)&rec.crc32
                                       - (const uint8_t *)&rec));

    /* 整页写入 (SDK 要求 256B 对齐) */
    static uint8_t page[STORE_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    memcpy(page, &rec, sizeof(rec));

    for (int attempt = 0; attempt < 2; attempt++) {
        store_flash_erase_program(STORE_CALIB_FLASH_OFFSET, page);

        /* 回读校验 (SDK 在编程后已 flush cache, XIP 读取一致) */
        store_calib_record_t back;
        store_flash_read(STORE_CALIB_FLASH_OFFSET, &back, sizeof(back));
        if (back.magic == rec.magic && back.crc32 == rec.crc32) {
            s_calib = rec;
            s_calib_valid = true;

            uint16_t n = 0;
            for (int i = 0; i < 18; i++) {
                if (done_mask & (1u << i)) n++;
            }
            hal_debug_printf("[STORE] Saved OK (%u servos, periods %u/%u, CRC 0x%08X)\r\n",
                             (unsigned)n,
                             (unsigned)rec.pwm_period_us[0],
                             (unsigned)rec.pwm_period_us[1],
                             (unsigned)rec.crc32);
            store_log_event(STORE_EVT_CALIB_SAVED, n);
            store_flush_pending(true);
            return true;
        }
    }

    hal_debug_printf("[STORE] Save FAILED (verify mismatch)\r\n");
    return false;
}

/* ==================== 运行时参数持久化 ==================== */

/* 记录 ~780B, 按 4 页 1024B 写入。缓冲一律 static 而非局部:
 * PICO_STACK_SIZE 默认仅 4KB, 两个近 1KB 的局部结构体会把栈顶穿。 */
static store_params_record_t s_params_rec;
static uint8_t               s_params_page[STORE_PARAMS_RECORD_BYTES];

bool store_save_params(const store_param_entry_t *entries, uint16_t count,
                       bool safe_context)
{
    if (!safe_context) {
        hal_debug_printf("[STORE] Refuse: robot is armed, disarm first (!S)\r\n");
        return false;
    }
    if (count > STORE_PARAMS_MAX) count = STORE_PARAMS_MAX;

    memset(&s_params_rec, 0, sizeof(s_params_rec));
    s_params_rec.magic   = STORE_PARAMS_MAGIC;
    s_params_rec.version = STORE_PARAMS_VERSION;
    s_params_rec.count   = count;
    if (count) {
        memcpy(s_params_rec.entries, entries,
               (size_t)count * sizeof(store_param_entry_t));
    }
    s_params_rec.crc32 = store_crc32((const uint8_t *)&s_params_rec,
                                     (uint32_t)((const uint8_t *)&s_params_rec.crc32
                                                - (const uint8_t *)&s_params_rec));

    /* 编程区必须整页对齐, 尾部以 0xFF 填充 (擦除态) */
    memset(s_params_page, 0xFF, sizeof(s_params_page));
    memcpy(s_params_page, &s_params_rec, sizeof(s_params_rec));

    for (int attempt = 0; attempt < 2; attempt++) {
        store_flash_erase_program_n(STORE_PARAMS_FLASH_OFFSET, s_params_page,
                                    sizeof(s_params_page));

        /* 回读校验 (SDK 在编程后已 flush cache, XIP 读取一致) */
        store_params_record_t back;
        store_flash_read(STORE_PARAMS_FLASH_OFFSET, &back, sizeof(back));
        if (back.magic == s_params_rec.magic &&
            back.version == s_params_rec.version &&
            back.count == s_params_rec.count &&
            back.crc32 == s_params_rec.crc32) {
            hal_debug_printf("[STORE] Params saved OK (%u 项, CRC 0x%08X)\r\n",
                             (unsigned)count, s_params_rec.crc32);
            store_log_event(STORE_EVT_PARAM_SAVED, count);
            store_flush_pending(true);
            return true;
        }
    }

    hal_debug_printf("[STORE] Params save FAILED (verify mismatch)\r\n");
    return false;
}

uint16_t store_load_params(store_param_entry_t *out, uint16_t max)
{
    if (!out || max == 0) return 0;

    store_flash_read(STORE_PARAMS_FLASH_OFFSET, &s_params_rec, sizeof(s_params_rec));

    if (s_params_rec.magic != STORE_PARAMS_MAGIC) return 0;
    if (s_params_rec.version != STORE_PARAMS_VERSION) return 0;
    if (s_params_rec.count > STORE_PARAMS_MAX) return 0;

    uint32_t crc_calc = store_crc32((const uint8_t *)&s_params_rec,
                                    (uint32_t)((const uint8_t *)&s_params_rec.crc32
                                               - (const uint8_t *)&s_params_rec));
    if (crc_calc != s_params_rec.crc32) return 0;

    uint16_t n = s_params_rec.count;
    if (n > max) n = max;

    /* 名字必须 NUL 结尾 —— 否则是记录损坏, 直接整体丢弃,
     * 不让 params_find 去 strcmp 一段越界的字节 */
    for (uint16_t i = 0; i < n; i++) {
        if (s_params_rec.entries[i].name[STORE_PARAM_NAME_LEN - 1] != '\0') return 0;
    }

    memcpy(out, s_params_rec.entries, (size_t)n * sizeof(store_param_entry_t));
    return n;
}

void store_log_event(uint8_t type, uint16_t data)
{
    /* 仅进 RAM 缓冲: 行走中写 Flash 会因 XIP 停摆导致 CRSF 丢帧 */
    if (s_pending_cnt >= STORE_PENDING_MAX) {
        s_dropped++;
        return;
    }
    store_log_entry_t *e = &s_pending[s_pending_tail];
    e->type  = type;
    e->flags = 0;
    e->data  = data;
    e->t_ms  = to_ms_since_boot(get_absolute_time());

    s_pending_tail = (uint8_t)((s_pending_tail + 1u) % STORE_PENDING_MAX);
    s_pending_cnt++;
}

void store_flush_pending(bool safe_context)
{
    if (!safe_context || s_pending_cnt == 0) return;

    /* 每次调用最多刷一条: 页编程 ~1-3ms 关中断, 避免长时间阻塞 */
    store_log_entry_t *e = &s_pending[s_pending_head];
    if (store_log_append(e->type, e->data, e->t_ms)) {
        s_pending_head = (uint8_t)((s_pending_head + 1u) % STORE_PENDING_MAX);
        s_pending_cnt--;
    }
}

void store_dump_log(void)
{
    uint32_t total = STORE_LOG_MAX_ENTRIES * STORE_LOG_ENTRY_SIZE;
    uint32_t used  = (uint32_t)s_log_next * STORE_LOG_ENTRY_SIZE;
    uint32_t used_pct = (used * 100u) / total;

    hal_debug_printf("[LOG] Ring: %u/%u entries (%u bytes, %u%% free)%s, pending %u, dropped %u\r\n",
                     (unsigned)s_log_next, (unsigned)STORE_LOG_MAX_ENTRIES,
                     (unsigned)used, (unsigned)(100u - used_pct),
                     s_log_full ? " FULL" : "",
                     (unsigned)s_pending_cnt, (unsigned)s_dropped);

    static const char *names[] = {
        "?", "BOOT", "BATT_OV", "BATT_CUTOFF", "BATT_WARN",
        "BATT_RECOVER", "CALIB_SAVED", "LOG_CLEARED", "PARAM_SAVED"
    };

    store_log_entry_t e;
    for (uint16_t i = 0; i < s_log_next; i++) {
        store_flash_read(STORE_LOG_FLASH_OFFSET + (uint32_t)i * STORE_LOG_ENTRY_SIZE,
                         &e, STORE_LOG_ENTRY_SIZE);
        const char *name = (e.type < sizeof(names) / sizeof(names[0]))
                           ? names[e.type] : "UNKNOWN";
        hal_debug_printf("[LOG]  %04u  %-12s t=%8ums  data=%u\r\n",
                         (unsigned)(i + 1), name,
                         (unsigned)e.t_ms, (unsigned)e.data);
    }
}

void store_clear_log(void)
{
    store_flash_erase_only(STORE_LOG_FLASH_OFFSET);

    s_log_page_idx = 0xFFFFu;   /* 页镜像失效, 强制重读全 0xFF 页 */
    s_log_next = 0;
    s_log_full = false;

    store_log_append(STORE_EVT_LOG_CLEARED, 0, to_ms_since_boot(get_absolute_time()));
    hal_debug_printf("[STORE] Log cleared\r\n");
}

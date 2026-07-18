/**
 * @file hexapod_ps2.h
 * @brief PS2 无线手柄接收器协议驱动
 * @note 通过 bit-bang SPI 协议与 PS2 接收器通信
 *       LSB-first, 时钟空闲高, 下降沿采样
 *
 * 硬件连接 (GP6~GP9):
 *   GP6 → DAT (输入, 内部上拉, 开漏)
 *   GP7 → CMD (推挽输出)
 *   GP8 → SEL/ATT (推挽输出, 通讯期间拉低)
 *   GP9 → CLK (推挽输出, 空闲高)
 *
 * 协议帧 (9 字节, SEL 全程拉低):
 *   Host 发: 0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
 *   PS2  回: 0x--, ID,   0x5A, BTN1, BTN2, RX,   RY,   LX,   LY
 *
 * ID = 0x41 数字绿灯模式, 0x73 模拟红灯模式
 * BTN1/2: 16 键 (按下=0, 松开=1)
 * RX/RY/LX/LY: 摇杆 0~255, 中位 128
 */

#ifndef HEXAPOD_PS2_H
#define HEXAPOD_PS2_H

#include <stdint.h>
#include <stdbool.h>
#include "hexapod_types.h"

/* ==================== GPIO 引脚定义 ==================== */

#define PS2_DAT_PIN     6    /* DI/DAT — 手柄→主机数据, 输入+上拉 */
#define PS2_CMD_PIN     7    /* DO/CMD — 主机→手柄命令, 推挽输出 */
#define PS2_SEL_PIN     8    /* CS/SEL — 片选, 通讯全程拉低 */
#define PS2_CLK_PIN     9    /* CLK — 时钟, 空闲高 */

/* ==================== 协议时序 ==================== */

#define PS2_CLK_DELAY_US    8   /* 半周期延时, ~62.5kHz 时钟 */
                                /* 9字节×8bit×16µs ≈ 1.15ms/帧 */

/* ==================== 手柄 ID ==================== */

#define PS2_ID_DIGITAL      0x41  /* 数字模式 (绿灯) */
#define PS2_ID_ANALOG_RED   0x73  /* 模拟模式 (红灯) */
#define PS2_ID_ANALOG_GREEN 0x53  /* 模拟模式 (绿灯) */
#define PS2_DATA_READY      0x5A  /* 数据就绪标志 */

/* ==================== 按键位掩码与功能分层 ====================

 * PS2 有 16 个按键 + 4 个模拟轴，远多于 CRSF 的 8 通道。
 * 功能分为两层:
 *
 *   [核心层] CRSF & PS2 通用 — 运动、解锁、步态、姿态、平衡、急停
 *   [扩展层] PS2 独占 — 调试、自动归位、校准触发、灵敏度切换等
 *
 * 扩展层功能仅在 PS2 模式下生效。CRSF 模式下无对应通道，
 * 系统正常运行不受影响 (无新增功能的退化降级是安全的)。
 *
 * Data[3] (BTN1, LSB first):
 *   bit 0: SELECT        [核心] 平衡模式开关
 *   bit 1: L3            [保留] 自动校准触发 (future)
 *   bit 2: R3            [保留] 控制灵敏度切换 (future)
 *   bit 3: START         [核心] 解锁/锁定
 *   bit 4: D-Pad UP      [核心] 抬腿高度配合键
 *   bit 5: D-Pad RIGHT   [核心] 站立姿态: 宽
 *   bit 6: D-Pad DOWN    [核心] 站立姿态: 正常
 *   bit 7: D-Pad LEFT    [核心] 站立姿态: 窄
 *
 * Data[4] (BTN2, LSB first):
 *   bit 0: L2            [保留] 辅助功能 A (future)
 *   bit 1: R2            [保留] 辅助功能 B (future)
 *   bit 2: L1            [核心] 步态: 上一个
 *   bit 3: R1            [核心] 步态: 下一个
 *   bit 4: △ (Triangle)  [扩展] 调试等级循环
 *   bit 5: ○ (Circle)    [核心] 紧急停止
 *   bit 6: × (Cross)     [核心] 抬腿高度配合键
 *   bit 7: □ (Square)    [保留] 自动归位/站立 (future)
 *
 * 合并为 16-bit: buttons = (Data[4] << 8) | Data[3]
 * 按下 = 0, 松开 = 1
 *
 * 新增扩展功能时: 只需修改 ps2_to_control() 中的对应 case,
 * 调用现有的 ctrl_state 字段或新增字段即可。CRSF 路径不受影响。 */

#define PSB_SELECT     (1 << 0)
#define PSB_L3         (1 << 1)
#define PSB_R3         (1 << 2)
#define PSB_START      (1 << 3)
#define PSB_PAD_UP     (1 << 4)
#define PSB_PAD_RIGHT  (1 << 5)
#define PSB_PAD_DOWN   (1 << 6)
#define PSB_PAD_LEFT   (1 << 7)
#define PSB_L2         (1 << 8)
#define PSB_R2         (1 << 9)
#define PSB_L1         (1 << 10)
#define PSB_R1         (1 << 11)
#define PSB_TRIANGLE   (1 << 12)
#define PSB_CIRCLE     (1 << 13)
#define PSB_CROSS      (1 << 14)
#define PSB_SQUARE     (1 << 15)

/* ==================== 摇杆索引 ==================== */

#define PSS_RX  5   /* 右摇杆 X — Data[5] */
#define PSS_RY  6   /* 右摇杆 Y — Data[6] */
#define PSS_LX  7   /* 左摇杆 X — Data[7] */
#define PSS_LY  8   /* 左摇杆 Y — Data[8] */

/* ==================== 摇杆死区 (读数偏离中位小于此值视为 0) ==================== */

#define PS2_STICK_DEADZONE  8   /* ±8/127 ≈ ±6.3% */

/* ==================== 配置模式命令序列 ==================== */

#define PS2_CMD_ENTER_CONFIG  0x43   /* 进入配置模式 */
#define PS2_CMD_ENABLE_ANALOG 0x44   /* 启用模拟 (红灯) 模式 */
#define PS2_CMD_ENABLE_RUMBLE 0x4D   /* 启用震动 */
#define PS2_CMD_EXIT_CONFIG   0x43   /* 退出配置 (特殊参数) */

/* ==================== 数据结构 ==================== */

typedef struct {
    uint8_t  data[9];           /* 原始 9 字节帧 */
    uint16_t buttons;           /* 16 键位掩码 (按下=0) */
    uint8_t  id;                /* 手柄 ID/模式 */
    uint8_t  joy_lx;            /* 左摇杆 X (0~255) */
    uint8_t  joy_ly;            /* 左摇杆 Y (0~255) */
    uint8_t  joy_rx;            /* 右摇杆 X (0~255) */
    uint8_t  joy_ry;            /* 右摇杆 Y (0~255) */
    uint32_t last_read_ms;      /* 上次成功读取时间 */
    uint32_t frame_count;       /* 有效帧计数 */
    bool     connected;         /* 手柄已连接 */
    bool     analog_mode;       /* 当前为模拟模式 (红灯) */
} ps2_state_t;

/* ==================== 函数声明 ==================== */

/**
 * @brief 初始化 PS2 接口 GPIO
 *        配置 GP6(输入+上拉), GP7/8/9(推挽输出)
 */
void ps2_init(void);

/**
 * @brief 读取 PS2 手柄一帧数据
 * @param state PS2 状态输出
 * @return true 表示读取成功且数据有效
 */
bool ps2_read_gamepad(ps2_state_t *state);

/**
 * @brief 尝试配置手柄进入模拟红灯模式
 * @param state PS2 状态
 * @return true 表示配置成功 (手柄返回 ID=0x73)
 */
bool ps2_enter_analog_mode(ps2_state_t *state);

/**
 * @brief 将 PS2 摇杆/按键映射到机器人控制
 * @param state PS2 状态
 * @param ctrl_state 输出: 机器人控制状态
 */
void ps2_to_control(const ps2_state_t *state, control_state_t *ctrl_state);

/**
 * @brief 检查 PS2 连接状态
 * @param state PS2 状态
 * @param timeout_ms 超时阈值
 * @param current_ms 当前时间
 * @return true 表示连接正常 (最近有数据)
 */
bool ps2_check_link(const ps2_state_t *state, uint32_t timeout_ms, uint32_t current_ms);

/**
 * @brief 获取 PS2 原始状态指针 (只读)
 * @note 供扩展功能模块读取原始按键/摇杆数据。
 *       返回 NULL 表示 PS2 未启用或未连接。
 *       不要在 ISR 中调用，不要修改返回的数据。
 * @return PS2 状态指针，不可用时返回 NULL
 */
const ps2_state_t* ps2_get_state(void);

#endif /* HEXAPOD_PS2_H */

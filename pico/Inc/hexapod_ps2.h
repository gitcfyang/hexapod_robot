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
#define PS2_POLL_INTERVAL_MS 16  /* 最小轮询间隔: 无线接收器对过快轮询敏感 (标准 ~20ms) */
                                /* 9字节×8bit×16µs ≈ 1.15ms/帧 */

/* ==================== 手柄 ID ==================== */

#define PS2_ID_DIGITAL      0x41  /* 数字模式 (绿灯) */
#define PS2_ID_ANALOG_RED   0x73  /* 模拟模式 (红灯) */
#define PS2_ID_ANALOG_GREEN 0x53  /* 模拟模式 (绿灯) */
#define PS2_ID_WIRELESS     0x79  /* 2.4G 无线接收器 (模拟模式) */
#define PS2_DATA_READY      0x5A  /* 数据就绪标志 */

/* ==================== 按键位掩码 ====================
 *
 * PS2 有 16 个按键 + 4 个模拟轴。在本固件里它们被翻译成统一通道
 * (见 hexapod_input.h): 摇杆 → 通道 0~3, 按键 → 通道 4~19 (位序如下)。
 * 哪个键干什么由**模式矩阵与组合键参数**决定 (网页「模式」页), 不在这
 * 里硬编码 —— 下面只是位序表, 方括号里是出厂默认动作。
 *
 * Data[3] (BTN1, LSB first):
 *   bit 0: SELECT        [平衡模式开关]
 *   bit 1: L3            [未分配]
 *   bit 2: R3            [未分配]
 *   bit 3: START         [解锁/上锁]
 *   bit 4: D-Pad UP      [步态: 下一个 / × 和弦: 抬腿 +]
 *   bit 5: D-Pad RIGHT   [站立姿态: 下一档]
 *   bit 6: D-Pad DOWN    [步态: 上一个 / × 和弦: 抬腿 -]
 *   bit 7: D-Pad LEFT    [站立姿态: 上一档]
 *
 * Data[4] (BTN2, LSB first):
 *   bit 0: L2            [未分配]
 *   bit 1: R2            [未分配]
 *   bit 2: L1            [未分配]
 *   bit 3: R1            [未分配]
 *   bit 4: △ (Triangle)  [未分配] (曾: 调试等级循环, 已按用户要求删除)
 *   bit 5: ○ (Circle)    [紧急停止]
 *   bit 6: × (Cross)     [抬腿高度和弦的配合键]
 *   bit 7: □ (Square)    [未分配]
 *
 * 合并为 16-bit: buttons = (Data[4] << 8) | Data[3]
 * 按下 = 0, 松开 = 1
 *
 * 新增动作时: 在 hexapod_input.c 的组合键枚举里加一项 + 参数表加一行,
 * 这里只跟着补一个位序宏。 */

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

/* ==================== 摇杆中位校准 ====================
 * 死区不在这一层: 摇杆先按中位校准换算到统一量程, 再由统一输入层
 * 按 ch_deadband 判死区 (两种手柄一套值, 见 hexapod_config.h)。 */

#define PS2_CENTER_CALIB_SAMPLES  10   /* 连接后摇杆中位采样帧数 (采样期间摇杆需居中) */

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
    uint8_t  center_lx;         /* 左摇杆 X 中位校准值 */
    uint8_t  center_ly;         /* 左摇杆 Y 中位校准值 */
    uint8_t  center_rx;         /* 右摇杆 X 中位校准值 */
    uint8_t  center_ry;         /* 右摇杆 Y 中位校准值 */
    uint8_t  center_samples;    /* 中位采样计数 (0=未校准) */
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
 * @note 实际映射在统一输入层 (hexapod_input.c), 本函数只做量程换算
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

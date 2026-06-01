/**
 * @file main_pico.c
 * @brief Raspberry Pi Pico六足机器人主程序
 * @note 使用Pico SDK编译，支持CRSF接收器(ELRS)和串口命令双输入模式
 *
 * 调试流水线（通过 DEBUG_LEVEL 控制，USB CDC 串口输出）：
 *   Level 1:  接收器 → 原始通道值
 *   Level 2:  通道值  → 运动控制量 (travel/gait/lift)
 *   Level 3:  控制量  → 舵机角度 (每腿 coxa/femur/tibia)
 *
 * 运行时命令（串口 115200 baud）：
 *   !V         切换调试等级 (0→1→2→3→0)
 *   !I         打印 CRSF 链路信息和 8 通道原始值
 *   !A         打印 18 路舵机角度/脉宽快照
 */

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hexapod_core.h"
#include "hexapod_config.h"
#include "hexapod_crsf.h"
#include "hexapod_i2c_protocol.h"

/* 全局机器人实例 */
static hexapod_t g_robot;

/**
 * @brief 机器人初始化
 */
bool robot_init(void)
{
    leg_config_t leg_configs[CNT_LEGS];
    hexapod_get_default_config(leg_configs);

    if (!hexapod_init(&g_robot, leg_configs)) {
        hal_debug_printf("Robot init failed!\r\n");
        return false;
    }

    hexapod_set_leg_lift_height(&g_robot, 50);
    hexapod_set_gait(&g_robot, GAIT_RIPPLE_12);

    hal_debug_printf("Robot initialized successfully!\r\n");
    return true;
}

/* ==================== 调试辅助函数 ==================== */

/**
 * @brief Level 1 调试：打印 CRSF 链接状态 + 8 通道原始值
 *
 * 预期值参考（CRSF 协议）：
 *   CH1~CH8 范围 172~1811，中位 992
 *   摇杆居中 → ~992，推到底 → ~172 或 ~1811
 *   拨杆低位 → <792，中位 → ~992，高位 → >1192
 */
static void debug_print_crsf(const crsf_state_t *crsf)
{
    if (!crsf) {
        hal_debug_printf("[DBG1] CRSF state unavailable\r\n");
        return;
    }

    hal_debug_printf("[DBG1] Link:%s Frames:%lu Age:%lums\r\n",
                crsf->link_connected ? "OK" : "NO",
                crsf->frame_count,
                hal_get_tick_ms() - crsf->last_frame_time_ms);

    hal_debug_printf("[DBG1] CH1(Fwd):%u CH2(Str):%u CH3(Hgt):%u CH4(Trn):%u "
                "CH5(Arm):%u CH6(Gait):%u CH7(Spd):%u CH8(Bal):%u\r\n",
                crsf->channels[0], crsf->channels[1],
                crsf->channels[2], crsf->channels[3],
                crsf->channels[4], crsf->channels[5],
                crsf->channels[6], crsf->channels[7]);
}

/**
 * @brief Level 2 调试：打印通道→控制量映射结果
 *
 * 预期值参考：
 *   robot_on:   CH5 高位→1 (解锁), 低位→0 (锁定)
 *   travel.x:   摇杆前推→+50, 后拉→-50
 *   travel.z:   摇杆右推→-40, 左推→+40 (取反)
 *   travel.y:   摇杆右转→+50, 左转→-50
 *   gait_type:  CH6 低位→0(RIPPLE), 中位→1(TRIPOD), 高位→3(WAVE)
 */
static void debug_print_control(const control_state_t *s)
{
    hal_debug_printf("[DBG2] ON:%d Travel(X:%d Y:%d Z:%d) Gait:%d Lift:%d Bal:%d\r\n",
                s->robot_on,
                s->travel_length.x, s->travel_length.y, s->travel_length.z,
                s->gait_type, s->leg_lift_height, s->balance_mode);
}

/**
 * @brief Level 3 调试：打印每腿 IK 解算角度
 */
static void debug_print_ik_angles(void)
{
    static const char* leg_name[CNT_LEGS] = {"RR","RM","RF","LR","LM","LF"};
    hal_debug_printf("[DBG3] IK angles (Cox/Fem/Tib, 0.1deg):\r\n");
    for (uint8_t i = 0; i < CNT_LEGS; i++) {
        const ik_solution_t *ik = hexapod_get_last_ik(&g_robot, (leg_index_t)i);
        if (ik) {
            hal_debug_printf("  %s: %d %d %d %s%s\r\n",
                        leg_name[i],
                        ik->coxa_angle, ik->femur_angle, ik->tibia_angle,
                        ik->solution_error ? "ERR " : "",
                        ik->solution_warning ? "WARN" : "");
        }
    }
    uint8_t cnt = hal_debug_get_last_servo_count();
    hal_debug_printf("[DBG3] Servos flushed: %u/%u\r\n", cnt,
                pca9685_get_board_count() * 9);
}

/* ==================== 主函数 ==================== */

/**
 * @brief 主函数
 */
int main(void)
{
    stdio_init_all();
    sleep_ms(2000);  /* 等待USB串口稳定 */

    hal_debug_printf("\r\n=================================\r\n");
    hal_debug_printf("Hexapod Robot - Raspberry Pi Pico\r\n");
    hal_debug_printf("=================================\r\n");

    /* 机器人初始化 */
    if (!robot_init()) {
        while(1) {
            hal_led_blink(0, 5);
            sleep_ms(1000);
        }
    }

    /* 硬件信息打印 */
    hal_debug_printf("\r\n");
    hal_debug_printf("Servo: PCA9685 x2 via I2C (GP2/GP3)\r\n");
    hal_debug_printf("Input: CRSF (ELRS) via UART1 (GP4/GP5) @420000 baud\r\n");
    hal_debug_printf("       Serial via UART1 @115200 baud (switch to INPUT_TYPE_SERIAL)\r\n");
    hal_debug_printf("Battery: ADC0 (GP26)\r\n");
    hal_debug_printf("Buzzer: GP15\r\n");
    hal_debug_printf("LED: GP25 (built-in)\r\n");
    hal_debug_printf("Debug: USB CDC\r\n");

    /* 打印 CRSF 摇杆映射 */
    hal_debug_printf("\r\nCRSF Channel Mapping (Mode 2):\r\n");
    hal_debug_printf("  CH1 (Left Y): Forward/Backward\r\n");
    hal_debug_printf("  CH2 (Left X): Strafe Left/Right\r\n");
    hal_debug_printf("  CH3 (Right Y): Lift Height\r\n");
    hal_debug_printf("  CH4 (Right X): Turn\r\n");
    hal_debug_printf("  CH5 (SWA): Arm/Disarm\r\n");
    hal_debug_printf("  CH6 (SWB): Gait (Low=RIPPLE, Mid=TRIPOD, High=WAVE)\r\n");
    hal_debug_printf("  CH8 (SWD): Balance Mode\r\n");

    hal_debug_printf("\r\nWaiting for ELRS link...\r\n");
    hal_debug_printf("Send '!V' to toggle debug level (current=%u)\r\n", hal_debug_get_level());

    /* 启动提示音 */
    uint16_t startup_notes[] = {1000, 1500, 2000};
    uint16_t startup_durations[] = {100, 100, 200};
    hal_play_sound(3, startup_notes, startup_durations);

    /* 主循环 */
    uint32_t last_status_time = 0;
    uint32_t last_debug_time  = 0;
    uint32_t last_update      = 0;
    uint32_t frame_delta_total = 0;

    while (1) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        /* ---- 每20ms控制循环 ---- */
        if (now - last_update >= CONTROL_LOOP_PERIOD_MS) {
            hexapod_update(&g_robot);
            last_update = now;

            /* 统计 CRSF 帧到达数 */
            if (hal_debug_get_level() >= 1) {
                frame_delta_total += hal_debug_get_crsf_frame_delta();
            }
        }

        /* ---- 分级调试输出 ---- */
        if (hal_debug_get_level() >= 1 && (now - last_debug_time >= DEBUG_PRINT_INTERVAL_MS)) {
            last_debug_time = now;

            /* Level 1: 接收器原始数据 */
            const crsf_state_t *crsf = (const crsf_state_t *)hal_debug_get_crsf_state();
            debug_print_crsf(crsf);

            /* Level 2: 控制量映射 */
            if (hal_debug_get_level() >= 2) {
                const control_state_t *state = hexapod_get_state(&g_robot);
                debug_print_control(state);
            }

            /* Level 3: 舵机输出 */
            if (hal_debug_get_level() >= 3) {
                debug_print_ik_angles();
            }

            /* 帧率统计 */
            hal_debug_printf("[DBG] FPS:%lu LVL:%u\r\n",
                        frame_delta_total * 1000 / DEBUG_PRINT_INTERVAL_MS,
                        hal_debug_get_level());
            frame_delta_total = 0;
        }

        /* ---- 每2秒运行状态摘要 ---- */
        if (now - last_status_time >= 2000) {
            last_status_time = now;
            const control_state_t *state = hexapod_get_state(&g_robot);

            if (state->robot_on) {
                hal_debug_printf("[RUN] Travel X:%d Y:%d Z:%d Gait:%d Lift:%d\r\n",
                            state->travel_length.x,
                            state->travel_length.y,
                            state->travel_length.z,
                            state->gait_type,
                            state->leg_lift_height);
                hal_led_set(0, true);
            } else {
                hal_debug_printf("[IDLE] Waiting for Arm signal...\r\n");
                hal_led_set(0, false);

                /* 待机时每5秒闪一下 */
                if (now % 5000 < 100) {
                    hal_led_set(0, true);
                }
            }
        }

        tight_loop_contents();
    }

    return 0;
}

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
#include "hardware/watchdog.h"
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

    hexapod_set_leg_lift_height(&g_robot, DEFAULT_LEG_LIFT_HEIGHT);
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

    hal_debug_printf("[DBG1] CH1(Str/Rol):%u CH2(Fwd/Pit):%u CH3(Hgt/Hgt):%u CH4(Trn/Yaw):%u "
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
    hal_debug_printf("[DBG2] ON:%d Bal:%d Travel(X:%d Y:%d Z:%d) Gait:%d Lift:%d\r\n",
                s->robot_on, s->balance_mode,
                s->travel_length.x, s->travel_length.y, s->travel_length.z,
                s->gait_type, s->leg_lift_height);
    hal_debug_printf("[DBG2] BodyPos Y:%d  BodyRot(P:%d Y:%d R:%d)\r\n",
                s->body_pos.y,
                s->body_rot.x, s->body_rot.y, s->body_rot.z);
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

    /* ==================== 上电电池检测 (先于舵机供电) ====================
     *
     * 安全设计:
     *   1. 舵机供电引脚 (GP10/GP11) 默认断电
     *   2. 先读电池电压:
     *      > 8.8V: 过压, 拒绝启动 (红灯 + 蜂鸣), 每 5s 复查
     *      < 6.6V: 过放, 拒绝启动 (红灯 + 蜂鸣), 每 5s 复查
     *   3. 电压在安全窗口 → 开启舵机供电 → 初始化舵机
     *   4. 防止过压烧舵机 + 过放伤电池 + 低压舵机抖动
     */
    hal_servo_power_init();  /* 初始化供电引脚, 保持断电状态 */

#if BATTERY_CHECK_ENABLED
    uint16_t boot_voltage = hal_get_battery_voltage();
    hal_debug_printf("Battery: %u mV\r\n", boot_voltage);

    if (boot_voltage > BATTERY_OVERVOLTAGE_MV) {
        /* 电压过高: 拒绝启动, 红色 LED 常亮 + 蜂鸣报警
         * 可能原因: 误接 3S 电池 / 电源故障 / 分压电阻焊接错误 */
        hal_debug_printf("BATTERY OVERVOLTAGE (%u mV > %u mV)! Servo power DISABLED.\r\n",
                         boot_voltage, BATTERY_OVERVOLTAGE_MV);
        hal_led_set(1, true);  /* 红色 LED 常亮 */
        uint16_t ov_alarm_notes[] = {1200, 0, 1200, 0, 1200};
        uint16_t ov_alarm_dur[]   = {200, 100, 200, 100, 400};
        hal_play_sound(5, ov_alarm_notes, ov_alarm_dur);

        while (1) {
            /* 每 5s 复查电压, 降到阈值以下后自动继续启动
             * 分 50 次轮询, 保持 !I2C 等 USB 诊断命令可用 */
            for (int i = 0; i < 50; i++) {
                hal_poll_usb_commands(NULL);
                sleep_ms(100);
            }
            boot_voltage = hal_get_battery_voltage();
            if (boot_voltage <= BATTERY_OVERVOLTAGE_MV) {
                hal_led_set(1, false);
                hal_debug_printf("Battery recovered: %u mV, continuing boot...\r\n",
                                 boot_voltage);
                break;
            }
            /* 仍在过压: 周期性蜂鸣提醒 */
            hal_play_sound(5, ov_alarm_notes, ov_alarm_dur);
        }
    } else if (boot_voltage < BATTERY_CUTOFF_MV) {
        /* 电压过低: 拒绝启动, 红色 LED 常亮 + 蜂鸣报警 */
        hal_debug_printf("BATTERY TOO LOW (%u mV < %u mV)! Servo power DISABLED.\r\n",
                         boot_voltage, BATTERY_CUTOFF_MV);
        hal_led_set(1, true);  /* 红色 LED 常亮 */
        uint16_t alarm_notes[] = {200, 0, 200, 0, 200};
        uint16_t alarm_dur[]   = {200, 100, 200, 100, 400};
        hal_play_sound(5, alarm_notes, alarm_dur);

        while (1) {
            /* 等待换电池/充电, 每 5s 复查一次电压, 恢复后自动继续
             * 分 50 次轮询, 保持 !I2C 等 USB 诊断命令可用 */
            for (int i = 0; i < 50; i++) {
                hal_poll_usb_commands(NULL);
                sleep_ms(100);
            }
            boot_voltage = hal_get_battery_voltage();
            if (boot_voltage >= BATTERY_RECOVERY_MV) {
                hal_led_set(1, false);
                hal_debug_printf("Battery recovered: %u mV, continuing boot...\r\n",
                                 boot_voltage);
                break;
            }
        }
    } else if (boot_voltage < BATTERY_WARNING_MV) {
        /* 低压警告: 允许启动但红灯闪烁提醒 */
        hal_debug_printf("BATTERY LOW (%u mV)! Charge soon.\r\n", boot_voltage);
    }
#else
    /* 电池检测已禁用 (BATTERY_CHECK_ENABLED=0): 跳过 ADC 读取 */
    hal_debug_printf("Battery check DISABLED (BATTERY_CHECK_ENABLED=0)\r\n");
#endif

    /* 电池检测通过 → 正式开机。
     * 注意: 舵机供电 (GP10/GP11) 保持断电, 由 ARM 解锁开关控制 —
     * 解锁时 hexapod_update 才开启供电并输出 PWM。 */
    hal_debug_printf("Boot OK. Servo power OFF — waiting for ARM switch.\r\n");

    /* 机器人初始化 (PCA9685 I2C 检测不受舵机供电影响, 逻辑侧 3.3V) */
    if (!robot_init()) {
        /* 初始化失败: 红灯闪烁, 但保持 USB 命令可用 (如 !I2C 诊断) */
        hal_debug_printf("Robot init failed! Use !I2C to check bus, !P to test servos.\r\n");
        while(1) {
            hal_poll_usb_commands(NULL);
            hal_led_blink(1, 2);
            sleep_ms(500);
        }
    }

    /* 硬件信息打印 */
    hal_debug_printf("\r\n");
    hal_debug_printf("Servo: PCA9685 x2 via I2C (GP14/GP15)\r\n");
#if INPUT_CONTROL_MODE == 0
    hal_debug_printf("Input: CRSF (ELRS) via UART1 (GP4/GP5) @420000 baud\r\n");
#else
    hal_debug_printf("Input: USB CDC Serial (no UART1 required)\r\n");
#endif
    hal_debug_printf("Battery: ADC0 (GP26, divider 15/115)\r\n");
    hal_debug_printf("Buzzer: GP13 (passive, PWM)\r\n");
    hal_debug_printf("LED: GP25 green (status), GP12 red (alarm)\r\n");
    hal_debug_printf("DC Motor: GP2/GP3 (PWM)\r\n");
    hal_debug_printf("Servo Power: GP10 left, GP11 right\r\n");
    hal_debug_printf("Debug: USB CDC\r\n");

#if INPUT_CONTROL_MODE == 0
    /* 打印 CRSF 摇杆映射 */
    hal_debug_printf("\r\nCRSF Channel Mapping:\r\n");
    hal_debug_printf("  Normal mode (CH8=LOW):\r\n");
    hal_debug_printf("    CH1 (Ail): Strafe    CH2 (Ele): Forward\r\n");
    hal_debug_printf("    CH3 (Thr): Height    CH4 (Rud): Turn\r\n");
    hal_debug_printf("  Balance mode (CH8=HIGH):\r\n");
    hal_debug_printf("    CH1 (Ail): Roll      CH2 (Ele): Pitch\r\n");
    hal_debug_printf("    CH3 (Thr): Height    CH4 (Rud): Yaw\r\n");
    hal_debug_printf("  Switches:\r\n");
    hal_debug_printf("    CH5 (SWA): Arm    CH6 (SWB): Gait\r\n");
    hal_debug_printf("    CH7 (SWC): Speed  CH8 (SWD): Balance\r\n");

    hal_debug_printf("\r\nWaiting for ELRS link...\r\n");
#else
    hal_debug_printf("\r\nUSB Serial Commands:\r\n");
    hal_debug_printf("  Movement: !F !B !L !R !Q !E !S\r\n");
    hal_debug_printf("  State:   !O(arm) !G<n>(gait) !T(balance) !U/!D(lift)\r\n");
    hal_debug_printf("  Servo:   !P<id> <angle> !W<id> !Z !A\r\n");
    hal_debug_printf("  Calib:   !C[<id>] !+/- !N !D\r\n");
    hal_debug_printf("\r\nReady for USB commands.\r\n");
#endif
    hal_debug_printf("Send '!V' to toggle debug level (current=%u)\r\n", hal_debug_get_level());

    /* 启动提示音 */
    uint16_t startup_notes[] = {1000, 1500, 2000};
    uint16_t startup_durations[] = {100, 100, 200};
    hal_play_sound(3, startup_notes, startup_durations);

    /* ---- 硬件看门狗 ----
     * 超时 1500ms：任何死锁（I2C 卡死、ISR 风暴）都将在 1.5s 内自动复位。
     * pause_on_debug=1：调试器挂接时暂停看门狗，防止断点触发复位。 */
    watchdog_enable(1500, 1);
    hal_debug_printf("Watchdog enabled (1500ms timeout)\r\n");

    /* 主循环 */
    uint32_t last_status_time = 0;
    uint32_t last_debug_time  = 0;
    uint32_t last_update      = 0;
    uint32_t frame_delta_total = 0;
#if BATTERY_CHECK_ENABLED
    uint32_t last_battery_check = 0;
    bool     servo_power_cut    = false;   /* 舵机供电是否已被过放/过压保护切断 */
#endif

    while (1) {
        watchdog_update();
        uint32_t now = to_ms_since_boot(get_absolute_time());

        /* ---- 每20ms控制循环 ---- */
        if (now - last_update >= CONTROL_LOOP_PERIOD_MS) {
            hexapod_update(&g_robot);
            last_update = now;

#if INPUT_CONTROL_MODE == 0
            /* 统计 CRSF 帧到达数 */
            if (hal_debug_get_level() >= 1) {
                frame_delta_total += hal_debug_get_crsf_frame_delta();
            }
#endif
        }

        /* ---- 分级调试输出 ---- */
        if (hal_debug_get_level() >= 1 && (now - last_debug_time >= DEBUG_PRINT_INTERVAL_MS)) {
            last_debug_time = now;

#if INPUT_CONTROL_MODE == 0
            /* Level 1: CRSF 接收器原始数据 */
            const crsf_state_t *crsf = (const crsf_state_t *)hal_debug_get_crsf_state();
            debug_print_crsf(crsf);
#endif

            /* Level 2: 控制量映射 */
            if (hal_debug_get_level() >= 2) {
                const control_state_t *state = hexapod_get_state(&g_robot);
                debug_print_control(state);
            }

            /* Level 3: 舵机输出 */
            if (hal_debug_get_level() >= 3) {
                debug_print_ik_angles();
            }

#if INPUT_CONTROL_MODE == 0
            /* CRSF 帧率统计 */
            hal_debug_printf("[DBG] FPS:%lu LVL:%u\r\n",
                        frame_delta_total * 1000 / DEBUG_PRINT_INTERVAL_MS,
                        hal_debug_get_level());
            frame_delta_total = 0;
#endif
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
#if INPUT_CONTROL_MODE == 0
                hal_debug_printf("[IDLE] Waiting for Arm signal...\r\n");
#else
                hal_debug_printf("[IDLE] Send !O to arm, !F/!B/!L/!R to move\r\n");
#endif
                hal_led_set(0, false);

                /* 待机时每5秒闪一下 */
                if (now % 5000 < 100) {
                    hal_led_set(0, true);
                }
            }
        }

#if BATTERY_CHECK_ENABLED
        /* ---- 运行电压监测 (每 1s) ----
         * 分级保护:
         *   > OV (8.8V): 过压保护 (红灯常亮 + 蜂鸣, 断开两路舵机供电)
         *   ≥ RECOVERY: 正常 (红灯灭, 舵机供电保持)
         *   WARNING ~ RECOVERY: 低压警告 (红灯闪, 舵机供电保持, 提示尽快回充)
         *   CUTOFF ~ WARNING: 严重低压 (红灯快闪 + 蜂鸣, 舵机供电保持到最后)
         *   < CUTOFF: 过放保护 (红灯常亮 + 蜂鸣, 断开两路舵机供电) */
        if (now - last_battery_check >= BATTERY_CHECK_INTERVAL_MS) {
            last_battery_check = now;
            uint16_t voltage = hal_get_battery_voltage();

            if (voltage > BATTERY_OVERVOLTAGE_MV) {
                /* 过压保护: 断开舵机供电 + 报警 */
                if (!servo_power_cut) {
                    servo_power_cut = true;
                    hal_servo_power_set_all(false);
                    hal_debug_printf("[BATT] OVERVOLTAGE %u mV! Servo power DISCONNECTED.\r\n",
                                     voltage);
                    hal_led_set(1, true);  /* 红灯常亮 */
                    uint16_t ov_notes[] = {1200, 0, 1200, 0, 1200};
                    uint16_t ov_dur[]   = {150, 100, 150, 100, 300};
                    hal_play_sound(5, ov_notes, ov_dur);
                }
            } else if (voltage < BATTERY_CUTOFF_MV) {
                /* 过放保护: 断开舵机供电 */
                if (!servo_power_cut) {
                    servo_power_cut = true;
                    hal_servo_power_set_all(false);
                    hal_debug_printf("[BATT] CUTOFF %u mV! Servo power DISCONNECTED.\r\n",
                                     voltage);
                    hal_led_set(1, true);  /* 红灯常亮 */
                    uint16_t alarm_notes[] = {200, 0, 200, 0, 200};
                    uint16_t alarm_dur[]   = {150, 100, 150, 100, 300};
                    hal_play_sound(5, alarm_notes, alarm_dur);
                }
            } else if (voltage < BATTERY_WARNING_MV) {
                /* 低压警告: 红灯闪烁 (2Hz), 舵机供电保持 */
                if (servo_power_cut) {
                    /* 电压回升但未达恢复阈值: 保持断电, 等待进一步回升 */
                    if (voltage >= BATTERY_RECOVERY_MV) {
                        servo_power_cut = false;
                        hal_led_set(1, false);
                        /* 仅在机器人解锁状态下恢复供电 (锁定 = 断电由 ARM 控制) */
                        const control_state_t *bstate = hexapod_get_state(&g_robot);
                        if (bstate && bstate->robot_on) {
                            hal_servo_power_set_all(true);
                            hal_debug_printf("[BATT] Recovered %u mV. Servo power restored.\r\n",
                                             voltage);
                        } else {
                            hal_debug_printf("[BATT] Recovered %u mV. Power stays OFF (locked).\r\n",
                                             voltage);
                        }
                    }
                } else {
                    hal_led_set(1, (now % 500) < 250);  /* 2Hz 闪烁 */
                }
            } else {
                /* 正常 */
                if (servo_power_cut) {
                    servo_power_cut = false;
                    /* 仅在机器人解锁状态下恢复供电 */
                    const control_state_t *bstate = hexapod_get_state(&g_robot);
                    if (bstate && bstate->robot_on) {
                        hal_servo_power_set_all(true);
                        hal_debug_printf("[BATT] Recovered %u mV. Servo power restored.\r\n",
                                         voltage);
                    } else {
                        hal_debug_printf("[BATT] Recovered %u mV. Power stays OFF (locked).\r\n",
                                         voltage);
                    }
                }
                hal_led_set(1, false);
            }
        }
#endif /* BATTERY_CHECK_ENABLED */

        tight_loop_contents();
    }

    return 0;
}

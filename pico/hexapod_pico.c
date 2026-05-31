/**
 * @file main_pico.c
 * @brief Raspberry Pi Pico六足机器人主程序
 * @note 使用Pico SDK编译，支持CRSF接收器(ELRS)和串口命令双输入模式
 */

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hexapod_core.h"
#include "hexapod_config.h"
#include "hexapod_crsf.h"

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
    
    /* 启动提示音 */
    uint16_t startup_notes[] = {1000, 1500, 2000};
    uint16_t startup_durations[] = {100, 100, 200};
    hal_play_sound(3, startup_notes, startup_durations);
    
    /* 主循环 */
    uint32_t last_status_time = 0;
    uint32_t last_update = 0;
    
    while (1) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        /* 每20ms更新一次控制循环 */
        if (now - last_update >= CONTROL_LOOP_PERIOD_MS) {
            hexapod_update(&g_robot);
            last_update = now;
        }
        
        /* 每2秒打印一次状态信息 */
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
                
                /* 运行时LED亮 */
                hal_led_set(0, true);
            } else {
                hal_debug_printf("[IDLE] Waiting for Arm signal...\r\n");
                hal_led_set(0, false);
                
                /* 每5秒闪烁一次LED表示待机 */
                if (now % 5000 < 100) {
                    hal_led_set(0, true);
                }
            }
        }
        
        /* 等待到下一个控制周期开始，避免忙等待占用CPU
         * 精确控制周期由 CONTROL_LOOP_PERIOD_MS 保证，此处仅做最低限度延时 */
        tight_loop_contents();
    }
    
    return 0;
}

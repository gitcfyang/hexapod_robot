/**
 * @file hexapod_core.c
 * @brief 六足机器人核心控制实现
 */

#include "hexapod_core.h"
#include "hexapod_config.h"
#include "hexapod_math.h"
#include <string.h>

/* 静态函数声明 */
static void compute_leg_ik(hexapod_t *robot, leg_index_t leg_index);
static void update_servos(hexapod_t *robot);

/* 调试：缓存最近一次 IK 解算结果 */
static ik_solution_t g_debug_ik[CNT_LEGS];

/**
 * @brief 初始化六足机器人
 */
bool hexapod_init(hexapod_t *robot, const leg_config_t *configs)
{
    if (!robot || !configs) {
        return false;
    }
    
    /* 清零 */
    memset(robot, 0, sizeof(hexapod_t));
    
    /* 复制腿部配置 */
    memcpy(robot->leg_configs, configs, sizeof(leg_config_t) * CNT_LEGS);
    
    /* 初始化控制状态 */
    robot->state.robot_on = false;
    robot->state.prev_robot_on = false;
    robot->state.balance_mode = false;
    robot->state.gait_type = GAIT_RIPPLE_12;
    robot->state.gait_step = 0;
    robot->state.leg_lift_height = DEFAULT_LEG_LIFT_HEIGHT;
    robot->state.speed_control = DEFAULT_GAIT_SPEED;
    
    /* 初始化步态 */
    hexapod_gait_init();
    hexapod_gait_select(GAIT_RIPPLE_12, &robot->state);
    
    /* 初始化腿部角度 */
    for (int i = 0; i < CNT_LEGS; i++) {
        robot->state.coxa_init_angle[i] = configs[i].coxa_angle;
    }
    
    /* 初始化舵机硬件 */
    if (!hal_servo_init()) {
#if HEADLESS_MODE
        hal_debug_printf("WARNING: Servo init failed (HEADLESS_MODE=1, continuing)\r\n");
        /* 不返回 false，允许无舵机调试 */
#else
        hal_debug_printf("ERROR: Servo init failed!\r\n");
        return false;
#endif
    }

    hal_debug_init();
    hal_input_init(INPUT_TYPE_CRSF);  /* 使用 CRSF 接收器输入 */
    
    robot->initialized = true;
    robot->servos_enabled = false;
    robot->last_update_time = hal_get_tick_ms();
    
    hal_debug_printf("Hexapod initialized\r\n");
    
    return true;
}

/**
 * @brief 使能/禁用舵机
 */
void hexapod_enable_servos(hexapod_t *robot, bool enable)
{
    if (!robot) return;
    
    robot->servos_enabled = enable;
    
    if (!enable) {
        hal_servo_free_all();
    }
}

/**
 * @brief 设置机身位置
 */
void hexapod_set_body_position(hexapod_t *robot, const coord3d_t *pos)
{
    if (!robot || !pos) return;
    robot->state.body_pos = *pos;
}

/**
 * @brief 设置机身旋转
 */
void hexapod_set_body_rotation(hexapod_t *robot, const coord3d_t *rot)
{
    if (!robot || !rot) return;
    robot->state.body_rot = *rot;
}

/**
 * @brief 设置行走参数
 */
void hexapod_set_travel(hexapod_t *robot, const coord3d_t *travel)
{
    if (!robot || !travel) return;
    robot->state.travel_length = *travel;
}

/**
 * @brief 设置抬腿高度
 */
void hexapod_set_leg_lift_height(hexapod_t *robot, int16_t height)
{
    if (!robot) return;
    robot->state.leg_lift_height = height;
}

/**
 * @brief 设置步态类型
 */
void hexapod_set_gait(hexapod_t *robot, gait_type_t gait_type)
{
    if (!robot) return;
    hexapod_gait_select(gait_type, &robot->state);
}

/**
 * @brief 设置平衡模式
 */
void hexapod_set_balance_mode(hexapod_t *robot, bool enable)
{
    if (!robot) return;
    robot->state.balance_mode = enable;
}

/**
 * @brief 紧急停止
 */
void hexapod_emergency_stop(hexapod_t *robot)
{
    if (!robot) return;
    
    robot->state.robot_on = false;
    robot->state.travel_length.x = 0;
    robot->state.travel_length.y = 0;
    robot->state.travel_length.z = 0;
    
    hal_servo_free_all();
}

/**
 * @brief 获取当前状态
 */
const control_state_t* hexapod_get_state(const hexapod_t *robot)
{
    if (!robot) return NULL;
    return &robot->state;
}

/**
 * @brief 获取最近一次 IK 解算结果（调试用）
 */
const ik_solution_t* hexapod_get_last_ik(const hexapod_t *robot, leg_index_t leg_index)
{
    if (!robot || leg_index >= CNT_LEGS) return NULL;
    return &g_debug_ik[leg_index];
}

/**
 * @brief 计算单腿逆运动学
 */
static void compute_leg_ik(hexapod_t *robot, leg_index_t leg_index)
{
    if (!robot || leg_index >= CNT_LEGS) return;
    
    /* 获取步态序列位置 */
    coord3d_t gait_pos;
    int16_t lift_height;
    
    hexapod_gait_sequence(&robot->state, leg_index, &gait_pos, &lift_height);
    
    /* 计算目标足端位置 */
    coord3d_t target_foot;
    const leg_config_t *cfg = &robot->leg_configs[leg_index];
    
    /* 初始位置 + 步态偏移 */
    target_foot.x = cfg->init_pos_x + gait_pos.x;
    target_foot.y = cfg->init_pos_y + gait_pos.y;
    target_foot.z = cfg->init_pos_z + gait_pos.z;
    
    /* IK解算 */
    ik_solution_t solution;
    bool ok = hexapod_ik_complete(&target_foot,
                                 &robot->state.body_pos,
                                 &robot->state.body_rot,
                                 leg_index,
                                 cfg,
                                 &solution);

    /* 缓存调试用 */
    g_debug_ik[leg_index] = solution;

    if (robot->servos_enabled) {
        /* 仅当无严重求解错误时输出舵机角度
         * solution_warning（角度越界）时仍然输出限幅后的角度，保持姿态稳定
         * solution_error（几何无解，如分母为0）时不输出，避免错误角度损坏舵机 */
        if (!solution.solution_error) {
            uint8_t servo_id_coxa = hal_get_servo_id(leg_index, 0);
            uint8_t servo_id_femur = hal_get_servo_id(leg_index, 1);
            uint8_t servo_id_tibia = hal_get_servo_id(leg_index, 2);
            
            hal_servo_set_angle(servo_id_coxa, solution.coxa_angle, 
                               robot->state.speed_control);
            hal_servo_set_angle(servo_id_femur, solution.femur_angle,
                               robot->state.speed_control);
            hal_servo_set_angle(servo_id_tibia, solution.tibia_angle,
                               robot->state.speed_control);
        }
    }
}

/**
 * @brief 更新所有舵机
 *        先缓存所有角度，再通过I2C批量输出到PCA9685
 */
static void update_servos(hexapod_t *robot)
{
    if (!robot || !robot->servos_enabled) return;
    
    /* 批量更新所有腿 */
    for (leg_index_t i = 0; i < CNT_LEGS; i++) {
        compute_leg_ik(robot, i);
    }
    
    /* 将所有缓存的角度通过I2C一次性发送到PCA9685 */
    hal_servo_flush();
}

/**
 * @brief 主控制循环
 */
void hexapod_update(hexapod_t *robot)
{
    if (!robot || !robot->initialized) return;
    
    uint32_t current_time = hal_get_tick_ms();
    uint32_t elapsed = current_time - robot->last_update_time;
    
    /* 控制周期检查 */
    if (elapsed < CONTROL_LOOP_PERIOD_MS) {
        return;
    }
    
    robot->last_update_time = current_time;
    
    /* 检查电池电压（通过 BATTERY_CHECK_ENABLED 宏控制开关） */
#if BATTERY_CHECK_ENABLED
    if (!hal_check_battery()) {
        hal_debug_printf("Low battery!\r\n");
        hexapod_emergency_stop(robot);
        return;
    }
#endif
    
    /* 读取输入 */
    hal_input_update(&robot->state);
    
    /* 机器人开关状态变化处理 */
    if (robot->state.robot_on != robot->state.prev_robot_on) {
        if (robot->state.robot_on) {
            /* 机器人开启 → 使能舵机 */
            robot->servos_enabled = true;
            hal_debug_printf("Robot ON\r\n");
        } else {
            /* 机器人关闭 → 释放舵机 */
            robot->servos_enabled = false;
            hal_servo_free_all();
            hal_debug_printf("Robot OFF\r\n");
        }
    }
    robot->state.prev_robot_on = robot->state.robot_on;

    
    /* 如果机器人开启 */
    if (robot->state.robot_on) {
        /* 检查是否需要步进 */
        bool need_step = (robot->state.travel_length.x != 0) ||
                        (robot->state.travel_length.y != 0) ||
                        (robot->state.travel_length.z != 0);
        
        if (need_step || robot->state.force_gait_step_cnt) {
            /* 步态步进 */
            hexapod_gait_step(&robot->state);
            
            if (robot->state.force_gait_step_cnt > 0) {
                robot->state.force_gait_step_cnt--;
            }
        }
        
        /* 更新舵机 */
        update_servos(robot);
    }
}

/**
 * @brief 执行单步
 */
void hexapod_single_step(hexapod_t *robot)
{
    if (!robot) return;
    
    hexapod_gait_step(&robot->state);
    update_servos(robot);
}

/**
 * @brief 获取腿部XZ平面长度
 */
uint16_t hexapod_get_legs_xz_length(const hexapod_t *robot)
{
    if (!robot) return 0;
    
    /* 使用第一条腿的初始位置计算 */
    const leg_config_t *cfg = &robot->leg_configs[LEG_RR];
    int32_t x = cfg->init_pos_x;
    int32_t z = cfg->init_pos_z;
    
    return (uint16_t)hexapod_sqrt(x * x + z * z);
}

/**
 * @brief 调整腿部位置
 */
void hexapod_adjust_leg_positions(hexapod_t *robot, uint16_t xz_length)
{
    if (!robot || xz_length == 0) return;
    
    uint16_t current_length = hexapod_get_legs_xz_length(robot);
    if (current_length == 0) return;
    
    /* 按比例缩放所有腿的初始位置 */
    int32_t scale = (int32_t)xz_length * 100 / current_length;
    
    for (int i = 0; i < CNT_LEGS; i++) {
        leg_config_t *cfg = &robot->leg_configs[i];
        cfg->init_pos_x = (cfg->init_pos_x * scale) / 100;
        cfg->init_pos_z = (cfg->init_pos_z * scale) / 100;
    }
}

/**
 * @brief 重置腿部初始角度
 */
void hexapod_reset_leg_init_angles(hexapod_t *robot)
{
    if (!robot) return;

    /* 将 state 中保存的初始角度重新加载到 leg_configs
     * state.coxa_init_angle 在 init 时从 config 默认值复制，
     * 并通过 hexapod_rotate_leg_init_angles() 与 leg_configs 同步修改，
     * 故此处的值始终与 leg_configs.coxa_angle 一致。
     * 如需恢复到真正的出厂默认值，请保存原始 config 并重新调用 hexapod_init()。 */
    for (int i = 0; i < CNT_LEGS; i++) {
        robot->leg_configs[i].coxa_angle = robot->state.coxa_init_angle[i];
    }
}

/**
 * @brief 旋转腿部初始角度
 */
void hexapod_rotate_leg_init_angles(hexapod_t *robot, int16_t delta_angle)
{
    if (!robot) return;

    for (int i = 0; i < CNT_LEGS; i++) {
        robot->state.coxa_init_angle[i] += delta_angle;
        /* 同步更新 leg_configs，使 IK 解算使用新的角度 */
        robot->leg_configs[i].coxa_angle = robot->state.coxa_init_angle[i];
    }
}

/**
 * @brief 调整到机身高度
 *        将腿部初始位置Y设置为目标高度
 *        公式：init_pos_y = DEFAULT_INIT_Y - body_pos.y（身体抬升则腿需向下伸展）
 */
void hexapod_adjust_to_body_height(hexapod_t *robot)
{
    if (!robot) return;
    
    /* 根据机身高度调整所有腿的Y坐标
     * body_pos.y 正值 = 抬升机身，腿需要向下伸展（init_pos_y 减小/变负） */
    for (int i = 0; i < CNT_LEGS; i++) {
        robot->leg_configs[i].init_pos_y = DEFAULT_INIT_Y - robot->state.body_pos.y;
    }
}

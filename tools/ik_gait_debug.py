#!/usr/bin/env python3
"""
六足机器人 IK + 步态核心算法 Python 复现
与 pico C 代码逐行对应，用于调试轨迹问题。

用法:
  python3 ik_gait_debug.py          # 默认: RIPPLE_12 前进, 6腿全画
  python3 ik_gait_debug.py --leg RR # 只看 RR 腿
  python3 ik_gait_debug.py --gait TRIPOD_6  # 换步态

依赖: pip install matplotlib numpy
"""

import math
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field

# ============================================================
# 查找表 — 与 hexapod_math.c 完全一致
# ============================================================

SIN_TABLE = [
    0, 87, 174, 261, 348, 436, 523, 610, 697, 784, 871, 958, 1045, 1132, 1218, 1305,
    1391, 1478, 1564, 1650, 1736, 1822, 1908, 1993, 2079, 2164, 2249, 2334, 2419, 2503,
    2588, 2672, 2756, 2840, 2923, 3007, 3090, 3173, 3255, 3338, 3420, 3502, 3583, 3665,
    3746, 3826, 3907, 3987, 4067, 4146, 4226, 4305, 4383, 4461, 4539, 4617, 4694, 4771,
    4848, 4924, 4999, 5075, 5150, 5224, 5299, 5372, 5446, 5519, 5591, 5664, 5735, 5807,
    5877, 5948, 6018, 6087, 6156, 6225, 6293, 6360, 6427, 6494, 6560, 6626, 6691, 6755,
    6819, 6883, 6946, 7009, 7071, 7132, 7193, 7253, 7313, 7372, 7431, 7489, 7547, 7604,
    7660, 7716, 7771, 7826, 7880, 7933, 7986, 8038, 8090, 8141, 8191, 8241, 8290, 8338,
    8386, 8433, 8480, 8526, 8571, 8616, 8660, 8703, 8746, 8788, 8829, 8870, 8910, 8949,
    8987, 9025, 9063, 9099, 9135, 9170, 9205, 9238, 9271, 9304, 9335, 9366, 9396, 9426,
    9455, 9483, 9510, 9537, 9563, 9588, 9612, 9636, 9659, 9681, 9702, 9723, 9743, 9762,
    9781, 9799, 9816, 9832, 9848, 9862, 9876, 9890, 9902, 9914, 9925, 9935, 9945, 9953,
    9961, 9969, 9975, 9981, 9986, 9990, 9993, 9996, 9998, 9999, 10000,
]

ACOS_TABLE_1 = [
    0,10,14,18,20,23,25,27,29,31,33,34,36,37,39,40,
    41,43,44,45,46,48,49,50,51,52,53,54,55,56,57,58,
    59,60,61,62,63,64,65,66,67,67,68,69,70,71,72,72,
    73,74,75,76,77,77,78,79,80,80,81,82,83,84,84,85,
    86,86,87,88,89,89,90,91,92,92,93,94,94,95,96,96,
    97,98,99,99,100,101,101,102,103,103,104,105,105,106,107,107,
    108,109,109,110,111,111,112,113,113,114,115,115,116,117,117,118,
    118,119,120,120,121,122,122,123,124,124,125,126,126,127,128
]

ACOS_TABLE_2 = [
    11,12,12,13,13,13,14,14,14,15,15,15,16,16,16,17,
    17,17,17,18,18,18,18,19,19,19,19,20,20,20,20,21,
    21,21,21,22,22,22,22,22,23,23,23,23,23,24,24,24,
    24,24,25,25,25,25,25,26,26,26,26,26,26,27,27,27,
    27,27,28,28,28,28,28,28,29,29,29,29,29,29,30,30,
    30,30,30,30,31,31,31,31,31,31,31,32,32,32,32,32,
    32,33,33,33,33,33,33,33,34,34,34,34,34,34,34,35,
    35,35,35,35,35,35,36,36,36,36,36,36,36,36,37
]

ACOS_TABLE_3 = [
    0,1,2,2,3,3,3,4,4,4,4,5,5,5,5,6,
    6,6,6,6,6,7,7,7,7,7,7,7,8,8,8,8,
    8,8,8,9,9,9,9,9,9,9,9,9,10,10,10,10,
    10,10,10,10,10,11,11,11,11,11,11,11,11,11,11,11
]

ATAN_TABLE = [
    0,1,1,2,2,3,3,4,5,5,
    6,6,7,7,8,9,9,10,10,11,
    11,12,12,13,13,14,15,15,16,16,
    17,17,18,18,19,19,20,20,21,21,
    22,22,23,23,24,24,25,25,26,26,
    27,27,27,28,28,29,29,30,30,31,
    31,31,32,32,33,33,33,34,34,35,
    35,35,36,36,37,37,37,38,38,38,
    39,39,39,40,40,40,41,41,41,42,
    42,42,43,43,43,44,44,44,44,45,
    45,
]

# ============================================================
# 数学函数 — 与 hexapod_math.c 完全一致
# ============================================================

def hexapod_sin(angle):
    """angle 单位 0.1°, 返回 sin*10000"""
    negative = angle < 0
    if negative:
        angle = -angle
    angle %= 3600
    sign = 1
    if angle >= 1800:
        angle -= 1800
        sign = -1
    if angle > 900:
        angle = 1800 - angle
    index = angle // 5
    if index >= len(SIN_TABLE):
        index = len(SIN_TABLE) - 1
    result = SIN_TABLE[index] * sign
    return -result if negative else result

def hexapod_cos(angle):
    return hexapod_sin(angle + 900)

def hexapod_acos(cos_val):
    """cos_val = cos*10000, 返回 angle*10 (0.1°单位)"""
    if cos_val > 10000: cos_val = 10000
    if cos_val < -10000: cos_val = -10000
    negative = cos_val < 0
    if negative:
        cos_val = -cos_val

    if cos_val <= 9000:
        index = (10000 - cos_val) * (len(ACOS_TABLE_1) - 1) // 10000
        if index >= len(ACOS_TABLE_1): index = len(ACOS_TABLE_1) - 1
        angle_raw = ACOS_TABLE_1[index]
    elif cos_val <= 9900:
        index = (9900 - cos_val) * (len(ACOS_TABLE_2) - 1) // 900
        if index >= len(ACOS_TABLE_2): index = len(ACOS_TABLE_2) - 1
        angle_raw = ACOS_TABLE_2[index]
    else:
        index = (10000 - cos_val) * (len(ACOS_TABLE_3) - 1) // 100
        if index >= len(ACOS_TABLE_3): index = len(ACOS_TABLE_3) - 1
        angle_raw = ACOS_TABLE_3[index]

    angle = angle_raw * 1800 // 255
    return (1800 - angle) if negative else angle

def hexapod_atan4(y, x):
    """返回 angle*10 (0.1°单位), 范围 -1800~1800"""
    if x == 0 and y == 0:
        return 0
    if x == 0:
        return 900 if y >= 0 else -900

    ax = -x if x < 0 else x
    ay = -y if y < 0 else y
    quad = (2 if x < 0 else 0) | (1 if y < 0 else 0)

    if ay <= ax:
        if ax == 0: ax = 1
        index = (ay * 100) // ax
        if index > 100: index = 100
        angle = ATAN_TABLE[index] * 10
    else:
        if ay == 0: ay = 1
        index = (ax * 100) // ay
        if index > 100: index = 100
        angle = 900 - ATAN_TABLE[index] * 10

    if quad == 0: pass
    elif quad == 1: angle = -angle
    elif quad == 2: angle = 1800 - angle
    elif quad == 3: angle = -1800 + angle
    return angle

# ============================================================
# 数据结构 — 与 hexapod_types.h / hexapod_config.h 一致
# ============================================================

LEG_NAMES = ["RR", "RM", "RF", "LR", "LM", "LF"]
LEG_RR, LEG_RM, LEG_RF, LEG_LR, LEG_LM, LEG_LF = 0, 1, 2, 3, 4, 5

@dataclass
class LegConfig:
    coxa_length:  int = 45
    femur_length: int = 75
    tibia_length: int = 120
    offset_x: int = 0
    offset_z: int = 0
    coxa_angle: int = 900       # 0.1° 单位
    init_pos_x: int = 0
    init_pos_y: int = 80
    init_pos_z: int = 0
    coxa_invert:  bool = True
    femur_invert: bool = True
    tibia_invert: bool = True
    coxa_horn_offset:  int = 0
    femur_horn_offset: int = 0
    tibia_horn_offset: int = 0

@dataclass
class Gait:
    nom_gait_speed: int = 100
    steps_in_gait:  int = 12
    nr_lifted_pos:  int = 4
    front_down_pos: int = 2
    lift_div_factor: int = 2
    tl_div_factor:   int = 8
    half_lift_height: int = 3
    gait_leg_nr: list = field(default_factory=lambda: [1, 3, 5, 7, 9, 11])

@dataclass
class ControlState:
    gait_step: int = 0
    gait_type: int = 0
    travel_length: tuple = (0, 0, 0)  # x, y(rotation), z
    leg_lift_height: int = 80
    speed_control: int = 120          # gait step period ms

# ============================================================
# 腿部配置 — 与 hexapod_config.h 一致
# ============================================================

def make_leg_configs():
    """创建 6 腿配置, 与 hexapod_get_default_config() 一致"""
    cfgs = []

    # RR
    cfg = LegConfig()
    cfg.offset_x = -104; cfg.offset_z = 63
    cfg.coxa_angle = 1350
    cfg.init_pos_x = -104 + (-78)
    cfg.init_pos_y = 80
    cfg.init_pos_z = 63 + 78
    cfg.coxa_invert = True; cfg.femur_invert = True; cfg.tibia_invert = True
    cfgs.append(cfg)

    # RM
    cfg = LegConfig()
    cfg.offset_x = 0; cfg.offset_z = 79
    cfg.coxa_angle = 900
    cfg.init_pos_x = 0 + 0
    cfg.init_pos_y = 80
    cfg.init_pos_z = 79 + 110
    cfg.coxa_invert = True; cfg.femur_invert = True; cfg.tibia_invert = True
    cfgs.append(cfg)

    # RF
    cfg = LegConfig()
    cfg.offset_x = 104; cfg.offset_z = 63
    cfg.coxa_angle = 450
    cfg.init_pos_x = 104 + 78
    cfg.init_pos_y = 80
    cfg.init_pos_z = 63 + 78
    cfg.coxa_invert = True; cfg.femur_invert = True; cfg.tibia_invert = True
    cfgs.append(cfg)

    # LR (mirror of RR)
    cfg = LegConfig()
    cfg.offset_x = -104; cfg.offset_z = -63
    cfg.coxa_angle = -1350
    cfg.init_pos_x = -104 + (-78)
    cfg.init_pos_y = 80
    cfg.init_pos_z = -63 + (-78)
    cfg.coxa_invert = False; cfg.femur_invert = False; cfg.tibia_invert = False
    cfgs.append(cfg)

    # LM (mirror of RM)
    cfg = LegConfig()
    cfg.offset_x = 0; cfg.offset_z = -79
    cfg.coxa_angle = -900
    cfg.init_pos_x = 0 + 0
    cfg.init_pos_y = 80
    cfg.init_pos_z = -79 + (-110)
    cfg.coxa_invert = False; cfg.femur_invert = False; cfg.tibia_invert = False
    cfgs.append(cfg)

    # LF (mirror of RF)
    cfg = LegConfig()
    cfg.offset_x = 104; cfg.offset_z = -63
    cfg.coxa_angle = -450
    cfg.init_pos_x = 104 + 78
    cfg.init_pos_y = 80
    cfg.init_pos_z = -63 + (-78)
    cfg.coxa_invert = False; cfg.femur_invert = False; cfg.tibia_invert = False
    cfgs.append(cfg)

    return cfgs

# ============================================================
# 步态定义 — 与 hexapod_gait.c 一致
# ============================================================

GAIT_TABLE = {
    "RIPPLE_12": Gait(
        steps_in_gait=12, nr_lifted_pos=4, front_down_pos=2,
        lift_div_factor=2, tl_div_factor=8, half_lift_height=3,
        gait_leg_nr=[1, 3, 5, 7, 9, 11],
    ),
    "TRIPOD_6": Gait(
        steps_in_gait=6, nr_lifted_pos=2, front_down_pos=1,
        lift_div_factor=2, tl_div_factor=4, half_lift_height=3,
        gait_leg_nr=[0, 0, 0, 3, 3, 3],
    ),
    "WAVE_24": Gait(
        steps_in_gait=24, nr_lifted_pos=3, front_down_pos=2,
        lift_div_factor=2, tl_div_factor=20, half_lift_height=3,
        gait_leg_nr=[1, 5, 9, 13, 17, 21],
    ),
    "TRIPOD_8": Gait(
        steps_in_gait=8, nr_lifted_pos=3, front_down_pos=2,
        lift_div_factor=2, tl_div_factor=5, half_lift_height=3,
        gait_leg_nr=[0, 0, 0, 4, 4, 4],
    ),
    "TRIPOD_4": Gait(
        steps_in_gait=4, nr_lifted_pos=1, front_down_pos=1,
        lift_div_factor=2, tl_div_factor=3, half_lift_height=1,
        gait_leg_nr=[0, 0, 0, 2, 2, 2],
    ),
}

# ============================================================
# IK 解算 — 与 hexapod_ik.c 完全一致
# ============================================================

FEMUR_SERVO_ZERO = 450
TIBIA_SERVO_ZERO = 900

def ik_leg(target_x, target_y, target_z, cfg: LegConfig):
    """单腿 IK, 返回 (coxa_angle, femur_angle, tibia_angle) 单位 0.1°"""
    pos_x, pos_y, pos_z = target_x, target_y, target_z
    warn = False

    # Step 1: Coxa 角度
    coxa_angle = hexapod_atan4(pos_z, pos_x) - cfg.coxa_angle

    # Step 2: Coxa 在 XZ 平面的投影
    dist_sq = pos_x * pos_x + pos_z * pos_z
    coxa_feet_dist = int(math.isqrt(dist_sq))  # sqrt
    ik_feet_dist = coxa_feet_dist - cfg.coxa_length
    if ik_feet_dist < 0:
        ik_feet_dist = 0
        warn = True

    # Step 3: Femur-Tibia 平面
    ik_feet_dist_sq = ik_feet_dist * ik_feet_dist
    ik_a1_sq = pos_y * pos_y
    ik_a_sq = ik_feet_dist_sq + ik_a1_sq
    ik_a = int(math.isqrt(ik_a_sq))

    femur_len = cfg.femur_length
    tibia_len = cfg.tibia_length

    # Tibia 角度 (余弦定理)
    cos_tibia_num = femur_len * femur_len + tibia_len * tibia_len - ik_a_sq
    cos_tibia_den = 2 * femur_len * tibia_len
    if cos_tibia_den == 0:
        return (0, 0, 0)  # error
    cos_tibia = (cos_tibia_num * 10000) // cos_tibia_den
    if cos_tibia > 10000: cos_tibia = 10000; warn = True
    if cos_tibia < -10000: cos_tibia = -10000; warn = True
    tibia_angle = TIBIA_SERVO_ZERO - hexapod_acos(cos_tibia)

    # Femur 角度
    if ik_feet_dist == 0:
        angle_a1 = 900 if pos_y >= 0 else -900
    else:
        angle_a1 = hexapod_atan4(pos_y, ik_feet_dist)

    cos_a2_num = femur_len * femur_len + ik_a_sq - tibia_len * tibia_len
    cos_a2_den = 2 * femur_len * ik_a
    if cos_a2_den == 0:
        return (0, 0, 0)
    cos_a2 = (cos_a2_num * 10000) // cos_a2_den
    if cos_a2 > 10000: cos_a2 = 10000; warn = True
    if cos_a2 < -10000: cos_a2 = -10000; warn = True
    angle_a2 = hexapod_acos(cos_a2)
    femur_angle = angle_a2 - angle_a1 - FEMUR_SERVO_ZERO

    # Invert
    if cfg.coxa_invert:  coxa_angle = -coxa_angle
    if cfg.femur_invert: femur_angle = -femur_angle
    if cfg.tibia_invert: tibia_angle = -tibia_angle

    # Horn offset
    coxa_angle  += cfg.coxa_horn_offset
    femur_angle += cfg.femur_horn_offset
    tibia_angle += cfg.tibia_horn_offset

    return (coxa_angle, femur_angle, tibia_angle)

# ============================================================
# 步态序列 — 与 hexapod_gait.c 完全一致 (含所有最新修复)
# ============================================================

def gait_sequence(ctrl: ControlState, gait: Gait, leg_index: int, gait_sub_phase: int = 0):
    """
    计算该腿在当前步态中的足端偏移。
    gait_sub_phase: 0-99, 步态步内的微进度。

    返回: (travel_x, travel_y, travel_z, lift_height, is_lifting)
      单位: mm
    """
    # 连续相位 (1 gait step = 100 sub-units)
    total_phase = ctrl.gait_step * 100 + gait_sub_phase
    leg_offset = gait.gait_leg_nr[leg_index] * 100
    leg_phase = total_phase - leg_offset

    cycle_len = gait.steps_in_gait * 100
    while leg_phase < 0:
        leg_phase += cycle_len
    while leg_phase >= cycle_len:
        leg_phase -= cycle_len

    is_lifting = leg_phase < gait.nr_lifted_pos * 100

    tx, ty, tz = ctrl.travel_length

    if is_lifting:
        # ---- 抬腿: 0→峰值→0 二次曲线 ----
        nr = gait.nr_lifted_pos
        nr_scaled = nr * 100

        # 高度 h ∝ 4·x·(1-x), x∈[0,1)
        h_pct = (4 * leg_phase * (nr_scaled - leg_phase)) // (nr * nr * 100)
        if h_pct > 100: h_pct = 100
        if h_pct < 0:   h_pct = 0
        lift_h = (ctrl.leg_lift_height * h_pct) // 100

        # X-Z 位移: 线性扫掠, 中点=0
        lift_phase = leg_phase - nr_scaled // 2
        travel_x = (tx * lift_phase) // nr_scaled
        travel_z = (tz * lift_phase) // nr_scaled

        return (travel_x, -lift_h, travel_z, lift_h, True)

    else:
        # ---- 地面支撑: 从前向后扫 (取反, 与抬腿方向相反) ----
        lift_h = 0
        ground_phase = leg_phase - gait.nr_lifted_pos * 100
        half_tl = gait.tl_div_factor * 50
        ground_centered = ground_phase - half_tl
        div = gait.tl_div_factor * 100

        # ★ 取反 — 这是修复"来回两次"bug 的关键
        travel_x = -(tx * ground_centered) // div
        travel_z = -(tz * ground_centered) // div

        return (travel_x, 0, travel_z, 0, False)


def ik_complete(ctrl, gait, leg_index, cfg: LegConfig, gait_sub_phase=0):
    """完整 IK: 步态偏移 + 身体变换 + 腿部 IK (与 hexapod_ik_complete 一致)"""
    gait_x, gait_y, gait_z, lift_h, is_lifting = gait_sequence(
        ctrl, gait, leg_index, gait_sub_phase)

    # 目标足端 = 初始位置 + 步态偏移 (身体坐标系)
    target_x = cfg.init_pos_x + gait_x
    target_y = cfg.init_pos_y + gait_y
    target_z = cfg.init_pos_z + gait_z

    # ★ 身体→coxa 局部坐标变换 (与 hexapod_ik_body 一致)
    # body_offset = (offset_x, 0, offset_z) 当 body_pos=(0,0,0), body_rot=(0,0,0)
    rel_x = target_x - cfg.offset_x
    rel_y = target_y           # body_offset.y = 0
    rel_z = target_z - cfg.offset_z

    coxa, femur, tibia = ik_leg(rel_x, rel_y, rel_z, cfg)

    return {
        'gait_x': gait_x, 'gait_y': gait_y, 'gait_z': gait_z,
        'lift_h': lift_h, 'is_lift': is_lifting,
        'target_x': target_x, 'target_y': target_y, 'target_z': target_z,
        'coxa': coxa, 'femur': femur, 'tibia': tibia,
    }

# ============================================================
# 周期遍历 — 生成完整步态周期数据
# ============================================================

def apply_compensation(cfgs, strategy):
    """补偿 coxa 摆幅不对称。

    'center':  调整 COXA_ANGLE 使 coxa 摆幅中心回到 0°（快，但站立时 coxa ≠ 0°）
    'foot0':   所有 FOOT_DX 设为 0（脚在 coxa 正侧方，几何上唯一能完全对称的方案）
    """
    if strategy == 'foot0':
        for li, cfg in enumerate(cfgs):
            sign = -1 if li >= 3 else 1
            cfg.init_pos_x = cfg.offset_x  # FOOT_DX = 0
            cfg.init_pos_z = cfg.offset_z + sign * 110 if li in (LEG_RM, LEG_LM) else \
                             cfg.offset_z + sign * 78
            cfg.coxa_angle = sign * 900
        print("Compensation: FOOT_DX=0, COXA_ANGLE=±90° for all legs")
    elif strategy == 'center':
        print("Compensation: Use --no-plot first, note the 'Coxa Mid' values,")
        print("  then add/subtract them from COXA_ANGLE in hexapod_config.h")
        print("  This centers the swing but changes standing coxa angle.")
    return cfgs


def simulate_gait_cycle(gait_name="RIPPLE_12", travel_x=120, travel_z=0,
                        travel_y=0, leg_filter=None, sub_steps=20,
                        compensate=None):
    """
    模拟一个完整步态周期。

    参数:
      gait_name: 步态名
      travel_x: 前进位移 (mm/周期)
      travel_z: 平移位移
      travel_y: 旋转位移
      leg_filter: 只显示指定腿 (None=全部)
      sub_steps: 每个步态步的细分数 (默认 20 对应 3ms@60ms period)

    返回: frames 列表, 每个元素 {leg_name: ik_result, ...}
    """
    gait = GAIT_TABLE[gait_name]
    ctrl = ControlState()
    ctrl.travel_length = (travel_x, travel_y, travel_z)
    ctrl.gait_type = list(GAIT_TABLE.keys()).index(gait_name)
    cfgs = make_leg_configs()
    if compensate:
        apply_compensation(cfgs, compensate)

    leg_indices = range(6) if leg_filter is None else [LEG_NAMES.index(leg_filter)]

    frames = []
    total_sub_steps = gait.steps_in_gait * sub_steps

    for phase in range(total_sub_steps):
        gait_step = phase // sub_steps
        sub_phase = (phase % sub_steps) * 100 // sub_steps

        ctrl.gait_step = gait_step

        frame = {}
        for li in leg_indices:
            result = ik_complete(ctrl, gait, li, cfgs[li], sub_phase)
            frame[LEG_NAMES[li]] = result
        frames.append(frame)

    return frames, gait, cfgs

# ============================================================
# 可视化
# ============================================================

def plot_analysis(frames, gait, cfgs, title_suffix=""):
    """绘制多维度分析图"""
    leg_indices = list(range(6))
    colors = ['tab:red', 'tab:orange', 'tab:green', 'tab:blue', 'tab:purple', 'tab:brown']
    total_frames = len(frames)

    fig, axes = plt.subplots(3, 3, figsize=(18, 14))
    fig.suptitle(f'Hexapod Gait Analysis — {title_suffix}', fontsize=14, fontweight='bold')

    # Row 1: Foot X trajectory (gait offset only)
    ax = axes[0, 0]
    for li in leg_indices:
        name = LEG_NAMES[li]
        xs = [f[name]['gait_x'] for f in frames]
        ax.plot(xs, label=name, color=colors[li], linewidth=1.2)
    ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
    ax.set_ylabel('Gait X (mm)')
    ax.set_title('Foot X Displacement (gait offset)')
    ax.legend(fontsize=7, ncol=6)
    ax.grid(True, alpha=0.3)

    # Row 1, Col 2: Foot Z trajectory
    ax = axes[0, 1]
    for li in leg_indices:
        name = LEG_NAMES[li]
        zs = [f[name]['gait_z'] for f in frames]
        ax.plot(zs, label=name, color=colors[li], linewidth=1.2)
    ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
    ax.set_ylabel('Gait Z (mm)')
    ax.set_title('Foot Z Displacement (gait offset)')
    ax.grid(True, alpha=0.3)

    # Row 1, Col 3: Lift height
    ax = axes[0, 2]
    for li in leg_indices:
        name = LEG_NAMES[li]
        hs = [f[name]['lift_h'] for f in frames]
        ax.plot(hs, label=name, color=colors[li], linewidth=1.2)
    ax.set_ylabel('Lift Height (mm)')
    ax.set_title('Lift Height')
    ax.grid(True, alpha=0.3)

    # Row 2: Coxa / Femur / Tibia angles
    for col, (joint, ylabel) in enumerate([
        ('coxa', 'Coxa Angle (0.1°)'),
        ('femur', 'Femur Angle (0.1°)'),
        ('tibia', 'Tibia Angle (0.1°)'),
    ]):
        ax = axes[1, col]
        for li in leg_indices:
            name = LEG_NAMES[li]
            angles = [f[name][joint] for f in frames]
            ax.plot(angles, label=name, color=colors[li], linewidth=1.2)
        ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
        ax.set_ylabel(ylabel)
        ax.set_title(f'{joint.capitalize()} Servo Angle')
        ax.grid(True, alpha=0.3)

    # Row 3: Foot trajectory in body coords (top-down view) + single leg detail
    # Top-down: X-Z plane
    ax = axes[2, 0]
    for li in leg_indices:
        name = LEG_NAMES[li]
        xs = [f[name]['target_x'] for f in frames]
        zs = [f[name]['target_z'] for f in frames]
        ax.plot(xs, zs, label=name, color=colors[li], linewidth=1.2)
        # mark start
        ax.scatter([xs[0]], [zs[0]], color=colors[li], s=30, marker='o')
        # mark end
        ax.scatter([xs[-1]], [zs[-1]], color=colors[li], s=30, marker='s')

    # Draw coxa positions
    for li in leg_indices:
        cfg = cfgs[li]
        ax.scatter([cfg.offset_x], [cfg.offset_z], color=colors[li], s=80,
                   marker='x', linewidths=2)
    ax.set_xlabel('X (mm) forward →')
    ax.set_ylabel('Z (mm) right →')
    ax.set_title('Foot Trajectory Top-Down (body coords)\nx = coxa position')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, ncol=3)

    # X-Y side view
    ax = axes[2, 1]
    for li in leg_indices:
        name = LEG_NAMES[li]
        xs = [f[name]['target_x'] for f in frames]
        ys = [f[name]['target_y'] for f in frames]
        ax.plot(xs, ys, label=name, color=colors[li], linewidth=1.2)
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm) up →')
    ax.set_title('Foot Trajectory Side View')
    ax.grid(True, alpha=0.3)

    # Coxa angle phase portrait: coxa angle vs foot X in coxa-local
    ax = axes[2, 2]
    for li in leg_indices:
        name = LEG_NAMES[li]
        cfg = cfgs[li]
        # foot in coxa-local coords
        coxa_angles = [f[name]['coxa'] for f in frames]
        foot_x_local = [f[name]['target_x'] - cfg.offset_x for f in frames]
        ax.plot(foot_x_local, coxa_angles, label=name, color=colors[li], linewidth=1.2)
    ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
    ax.axvline(0, color='gray', linestyle='--', linewidth=0.5)
    ax.set_xlabel('Foot X in coxa-local (mm)')
    ax.set_ylabel('Coxa Servo Angle (0.1°)')
    ax.set_title('Coxa Angle vs Foot X Position')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def print_stats(frames, gait, cfgs):
    """打印每条腿的关键统计数据"""
    print(f"\n{'='*80}")
    print(f"GAIT STATISTICS  (steps={gait.steps_in_gait}, lift={gait.nr_lifted_pos}, "
          f"ground={gait.tl_div_factor})")
    print(f"{'='*80}")
    print(f"{'Leg':<6} {'Coxa Range':<20} {'Coxa Mid':<12} {'Femur Range':<20} "
          f"{'Tibia Range':<20} {'Lift Peak':<12}")
    print(f"{'-'*90}")

    for li in range(6):
        name = LEG_NAMES[li]
        coxas = [f[name]['coxa'] for f in frames]
        femurs = [f[name]['femur'] for f in frames]
        tibias = [f[name]['tibia'] for f in frames]
        lifts = [f[name]['lift_h'] for f in frames]

        coxa_range = (min(coxas), max(coxas))
        coxa_swing = coxa_range[1] - coxa_range[0]
        coxa_mid = (coxa_range[0] + coxa_range[1]) // 2
        femur_swing = max(femurs) - min(femurs)
        tibia_swing = max(tibias) - min(tibias)

        print(f"{name:<6} [{coxa_range[0]:>5d}, {coxa_range[1]:>5d}] "
              f"Δ={coxa_swing:>4d} mid={coxa_mid:>+4d}  "
              f"FemΔ={femur_swing:>4d}  TibΔ={tibia_swing:>4d}  "
              f"Hmax={max(lifts):>3d}")

    # Symmetry check for coxa
    print(f"\n--- Coxa Symmetry Check ---")
    for li in range(6):
        name = LEG_NAMES[li]
        coxas = [f[name]['coxa'] for f in frames]
        forward_max = max(coxas)
        backward_max = abs(min(coxas))
        ratio = max(forward_max, backward_max) / max(min(forward_max, backward_max), 1)
        bias = "◀ BIASED FORWARD" if forward_max > backward_max * 1.3 else (
               "▶ BIASED BACKWARD" if backward_max > forward_max * 1.3 else "✓ SYMMETRIC")
        print(f"  {name}: fwd={forward_max:>5d}, bwd={backward_max:>5d}, "
              f"ratio={ratio:.2f}  {bias}")


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser(description='Hexapod IK + Gait Debug Tool')
    parser.add_argument('--gait', default='RIPPLE_12',
                        choices=list(GAIT_TABLE.keys()))
    parser.add_argument('--leg', default=None, help='Single leg to analyze (RR, RM, etc)')
    parser.add_argument('--travel-x', type=int, default=120, help='Forward travel mm/cycle')
    parser.add_argument('--travel-z', type=int, default=0, help='Strafe travel mm/cycle')
    parser.add_argument('--travel-y', type=int, default=0, help='Turn travel mm/cycle')
    parser.add_argument('--sub-steps', type=int, default=20,
                        help='Sub-steps per gait step (default 20)')
    parser.add_argument('--no-plot', action='store_true', help='Text stats only')
    parser.add_argument('--compensate', choices=['foot0', 'center'], default=None,
                        help='Compensation strategy for coxa asymmetry')
    args = parser.parse_args()

    print(f"Simulating: {args.gait}, travel=({args.travel_x}, {args.travel_y}, {args.travel_z})")
    print(f"Sub-steps per gait step: {args.sub_steps}")
    if args.compensate:
        print(f"Compensation: {args.compensate}")

    frames, gait, cfgs = simulate_gait_cycle(
        args.gait, args.travel_x, args.travel_z, args.travel_y,
        args.leg, args.sub_steps,
        compensate=args.compensate)

    print(f"Total frames: {len(frames)}")
    print(f"Gait cycle: {gait.steps_in_gait} steps × {args.sub_steps} sub-steps")

    print_stats(frames, gait, cfgs)

    if not args.no_plot:
        title = f"{args.gait}  travel=({args.travel_x},{args.travel_y},{args.travel_z})"
        if args.leg:
            title += f"  leg={args.leg}"
        plot_analysis(frames, gait, cfgs, title)
        plt.show()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""网页监视台回归测试 (无需硬件)。

[1] 解析器单元测试: 用固件真实输出样本喂 parse_line, 断言解析出的字段。
    固件输出格式若变动, 这里会先报警 (解析失效只影响显示, 不会崩)。
[2] 固件契约: 从 hexapod_params.c 抽出参数表, 逐条校验服务器/前端的隐含前提
    (名字长度、字符集、与 config.h 宏的对应), 这类不一致在固件侧是静默的。
[3] 端到端: socat pty 对 + 回放假固件 + serial_console.py + server.py ——
    验证 HTTP 静态页 / POST 命令下发 / SSE 广播 / 遥测解析整条链路,
    含 /param 参数读写的完整回环 (改 → 固件回显 → SSE → 前端)。

用法: python3 tools/webconfig/test_webconfig.py
"""

import html
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
ROOT = os.path.dirname(TOOLS)
sys.path.insert(0, HERE)

import server as wc                                     # noqa: E402

# 端口行正则: 差分检查 (把固件+server 回退成旧版) 时 server 上还没有这个属性,
# 按"一行都不认"处理, 让断言红而不是 AttributeError 把整段跑挂
RE_PORTS = getattr(wc, "RE_PORTS", re.compile(r"(?!x)x"))

PTY_A, PTY_B = "/tmp/wc_fakerobot", "/tmp/wc_robotport"
BRIDGE_PORT, HTTP_PORT = 7199, 8099
# 第二套实例: 连接门那组用例要开轮询, 不能和主用例的 --no-poll 挤在一起
PTY_A2, PTY_B2 = "/tmp/wc_fakerobot2", "/tmp/wc_robotport2"
BRIDGE_PORT2, HTTP_PORT2 = 7198, 8098

procs = []
passed, failed = [], []


def check(name, cond, detail=""):
    (passed if cond else failed).append(name)
    print(("  PASS  " if cond else "  FAIL  ") + name
          + (f"  [{detail}]" if detail else ""))
    sys.stdout.flush()


# ==================== [1] 解析器单元测试 ====================

# 以下样本逐字取自固件 printf (hexapod_hal_pico.c), 改动固件输出时同步更新
S_BATT = [
    "=== Battery ADC (GP28/ADC2, divider 47/377) ===",
    "Raw: avg=1277 min=1270 max=1285 (spread 15 counts = 12 mV)",
    "Pin: 1029 mV  →  Battery: 8370 mV",
    "Limits: 6600 ~ 8800 mV → IN RANGE",
    "Protection: BATTERY_CHECK_ENABLED=0",
    "State: OK",
]
# 未接电池 (USB 供电): 分压抽头被 R2 拉到 ~0mV。不是故障, 前端不该显示成低压告警。
S_BATT_ABSENT = [
    "=== Battery ADC (GP28/ADC2, divider 47/377) ===",
    "Raw: avg=0 min=0 max=1 (spread 1 counts = 0 mV)",
    "Pin: 0 mV  →  Battery: 0 mV",
    "Limits: 6600 ~ 8800 mV → OUT OF RANGE",
    "Protection: BATTERY_CHECK_ENABLED=1",
    "State: ABSENT (未接电池, USB 供电 — 不算故障)",
]
S_I2C = [
    "=== I2C Bus Check (SDA=GP14, SCL=GP15) ===",
    "Servo power: GP10(left)=1  GP11(right)=1",
    "PCA9685 0x40 (left legs): DETECTED  MODE1=0x11",
    "PCA9685 0x41 (right legs): DETECTED  MODE1=0x11",
    "BNO055 0x28: NOT FOUND",
    "BNO055 0x29: DETECTED  CHIP_ID=0xA0",
    "Scanning I2C bus...",
    "  Device found at 0x29",
    "  Device found at 0x40",
    "Scan complete.",
    "Bus idle: SDA(GP14)=1 SCL(GP15)=1 (1=空闲正常, 0=被拉低/短路/引脚损坏)",
]
# !I2C q —— 网页轮询用的定向检测: 没有 128 地址全扫描那几行
S_I2C_Q = [
    "=== I2C Bus Check (SDA=GP14, SCL=GP15) quick ===",
    "Servo power: GP10(left)=0  GP11(right)=0",
    "PCA9685 0x40 (left  legs): DETECTED  MODE1=0x00",
    "PCA9685 0x41 (right legs): DETECTED  MODE1=0x00",
    "BNO055 0x28: NOT FOUND",
    "BNO055 0x29: NOT FOUND",
    "Bus idle: SDA(GP14)=1 SCL(GP15)=1 (1=空闲正常, 0=被拉低/短路/引脚损坏)",
]
S_IMU = [
    "=== IMU Status ===",
    "available: YES  addr: 0x29",
    "fail: 连续 3 次 (已持续 31ms) / 累计 9 次",   # 块内唯一的故障信号, 折叠时放行
    "calib: sys=3 gyr=3 acc=3 mag=0 (fully)",
    "INT_STA=0x80 (bit7=BSX_DRDY)",
    # 原始传感器行 (imu_status_print): 每行挑一个负轴, 钉住 -?\d+ 的负号分支
    "accel: x=12 y=5 z=-1002 (0.01 m/s2)",
    "gyro: x=0 y=0 z=-45 (1/16 dps)",
    "mag: x=-32 y=64 z=0 (1/16 uT)",
    "temp: 31 (C)",
    "last: roll=-12 pitch=34 yaw=567 (0.1deg) valid=1",
    "===================",
]
S_SERVO = [
    "=== Servo Snapshot ===",
    "Board 0x41 (right): [0]:1500 [1]:1480 [2]:1520 [3]:1500 [4]:1500 [5]:1500 "
    "[6]:1500 [7]:1500 [8]:1500 ",
    "Board 0x40 (left): [9]:1500 [10]:1500 [11]:1500 [12]:1500 [13]:1500 "
    "[14]:1500 [15]:1500 [16]:1500 [17]:1500 ",
]

# 遥控通道遥测 (hexapod_hal_pico.c: hal_debug_print_channel_telemetry, 自由推送 5Hz)
S_CH_CRSF = [
    "[CH] m=crsf link=1 fc=123456 c0=172 c1=1500 c2=992 c3=992 c4=1811 c5=992 "
    "c6=992 c7=992 c8=992 c9=992 c10=992 c11=992 c12=992 c13=992 c14=992 c15=992",
]
S_CH_PS2 = [
    "[CH] m=ps2 con=1 btns=65535 lx=128 ly=130 rx=127 ry=126 fc=987",
]

# 固件版本 (hexapod_hal_pico.c: hal_fw_version_print): 开机横幅 + !VER 共用两行。
# 固件内容是 git describe 的结果, 所以样本取个"长得像"的字符串即可;
# [HW] 是 HEXAPOD_HW_VERSION 常量 (换板时人工改)。
S_VER = [
    "[VER] Hexapod v0.3.0-16-g8f5b04f-dirty",
    "[HW] PCB v2",
]

# 遥控门回执 (hexapod_hal_pico.c: !RC 命令 + 6s 超时自解锁): 严格两单词,
# 不带任何后缀 —— server 的正则锚定整行, 加后缀就等于解析不出来
S_RC = [
    "[RC] locked",
    "[RC] unlocked",
]

# 供电/运行状态 (hexapod_core.c 跳变行 + hexapod_pico.c 每 2s 摘要) ——
# 控制页「供电」徽标的数据源, 两条路都写同一个状态键
# 尾缀 Bal/St/Hi = 平衡模式 / 站立姿态 (-1 窄, 0 正常, 1 宽) / 高度积分开关 ——
# 模式页的实时徽标全靠它, 三种 [IDLE] 变体都带 (待机时切姿态也看得见)
S_RUN = [
    "Robot ON (servo power enabled)",
    "Robot OFF (servo power cut)",
    "[RUN] Travel X:60 Y:0 Z:0 Gait:0 Lift:40 Bal:0 St:0 Hi:1",
    "[IDLE] Send !O to arm, !F/!B/!L/!R to move Bal:0 St:0 Hi:1",
    "[IDLE] Waiting for START (PS2 arm)... Bal:0 St:-1 Hi:1",
]

# 外设状态 (hexapod_hal_pico.c: periph_status_print / !PERIPH) —— 一行一节,
# 节名是 server.py 的白名单契约。轮询线程每 2 秒问一次。
S_PER = [
    "[PER] motors: m1=500 m2=0",
    "[PER] leds: g=1 r=0 hb=1 alarm=1",
    "[PER] buzzer: freq=1500 ms=300",
    "[PER] uart0: en=1 baud=115200 rx=42 tx=7 lines=3",
    "[PER] ext: mode=2 found=0 a0=1234 a1=5678",
    "[PER] pwm: 0=0 1=1500 2=0 3=5000 4=0 5=0 6=0 7=0 8=0 9=0 10=0 11=0 12=0 13=0",
    "[PER] pwmperiod: l=9500 r=9500",
]
# 端口状态 (hexapod_hal_pico.c: ports_status_print / !PORTS) —— 一行一节,
# 节名是 server.py 的白名单契约。轮询线程每 2 秒问一次。逐字取自固件格式:
# 足端报的是引脚原始电平 (低=触地), input 的波特率是 uart_init 返回的实际值。
S_PORTS = [
    "[PORTS] uart0: en=1 baud=115200",
    "[PORTS] input: mode=0 baud=420000",
    "[PORTS] motors: en=1 lv1=0 lv2=1",
    "[PORTS] foot: en=1 s0=1 s1=0 s2=1 s3=1 s4=1 s5=1",
    "[PORTS] ext: mode=0 a0=0 a1=0",
    "[PORTS] gpio: g23fn=1 g23lv=1 g24fn=0 g24lv=0 g29fn=0 g29lv=1",
    "[PORTS] fixed: spwl=1 spwr=1 ledr=0 ledg=1 boot=1",
]

# 舵盘机械中心偏移 (固件 hal_pico.c: calib_horn_print_all / calib_horn_record)。
# 18 行逐字取自固件格式; 值是 config.h 里各腿的默认偏移。
S_HO = [f"[HO] id={i} off={v}" for i, v in enumerate(
    [-10, 55, 8, 12, 85, 68, 0, 40, 30, -25, 60, 70, 15, 35, 45, 5, 20, 25])]

# 外部 UART0 的收发 ([U0TX] 是 !UART0T 的回执, [U0] 是收到的整行)
S_U0 = [
    "[U0TX] sent 5 bytes: hello",
    "[U0] hello",
]

# 参数输出 (hexapod_params.c: params_print_all) —— 注意 unit= 后为空 token
S_PARAMS = [
    "[P] === 3 params (1 changed) ===",
    "[P] name=batt_check val=0 min=0 max=1 def=0 unit= grp=batt chg=0",
    "[P] name=batt_ov_mv val=9000 min=8000 max=9500 def=8800 unit=mV grp=batt chg=1",
    "[P] name=travel_fwd_mm val=150 min=20 max=250 def=150 unit=mm grp=motion chg=0",
    "[P] === end ===",
    "[P] WARN travel_fwd_mm clamped 999 -> 250 (范围 20~250)",
    "[P] FAIL unknown param 'nosuch' (试 !CFG 列出全部)",
]


def test_param_parser():
    print("\n[1b] 参数行单元测试")
    t = wc.Telemetry()
    kinds = [wc.parse_line(t, line) for line in S_PARAMS]

    check("参数: 三行参数被识别, 汇总行与提示行走日志",
          [k[0] if k else None for k in kinds]
          == [None, "params", "params", "params", None, None, None])
    snap = t.snapshot()["params"]
    check("参数: 汇总条数/改动数来自固件回显",
          snap["count"] == 3 and snap["changed"] == 1, str(snap))
    check("参数: val/min/max/def/unit/grp 全解析",
          snap["list"]["batt_ov_mv"] == {"name": "batt_ov_mv", "val": 9000, "min": 8000,
                                         "max": 9500, "def": 8800, "unit": "mV",
                                         "grp": "batt", "chg": True},
          str(snap["list"].get("batt_ov_mv")))
    check("参数: chg=0 判为未改动", snap["list"]["batt_check"]["chg"] is False)
    check("参数: 空 unit 解析为空串而非缺字段", snap["list"]["batt_check"]["unit"] == "")

    # 同一参数再次出现就地覆盖 (前端只关心最新值), 而不是新增一项
    t2 = wc.Telemetry()
    wc.parse_line(t2, "[P] name=travel_fwd_mm val=150 min=20 max=250 def=150 unit=mm grp=motion chg=0")
    wc.parse_line(t2, "[P] name=travel_fwd_mm val=200 min=20 max=250 def=150 unit=mm grp=motion chg=1")
    s2 = t2.snapshot()["params"]
    check("参数: 同名重复出现就地更新 (不重复计数)",
          s2["count"] == 1 and s2["list"]["travel_fwd_mm"]["val"] == 200 and s2["changed"] == 1)

    for bad in ["[P] name=x val=1",                       # 缺字段
                "[P] name=x val=abc min=0 max=1 def=0 unit= grp=sys chg=0"]:  # 非整数
        check(f"参数: 残缺行解析为 None ({bad[:22]}…)",
              wc.parse_param_line(bad) is None)


# ==================== [2] 固件契约 ====================

PARAMS_C = os.path.join(ROOT, "pico", "Src", "hexapod_params.c")
CONFIG_H = os.path.join(ROOT, "pico", "Inc", "hexapod_config.h")
STORE_H = os.path.join(ROOT, "pico", "Inc", "hexapod_store.h")
STORE_C = os.path.join(ROOT, "pico", "Src", "hexapod_store.c")
CORE_C = os.path.join(ROOT, "pico", "Src", "hexapod_core.c")
I2C_C = os.path.join(ROOT, "pico", "Src", "hexapod_i2c_protocol.c")
HAL_C = os.path.join(ROOT, "pico", "Src", "hexapod_hal_pico.c")
HAL_H = os.path.join(ROOT, "pico", "Inc", "hexapod_hal.h")
BNO_C = os.path.join(ROOT, "pico", "Src", "bno055.c")
BNO_H = os.path.join(ROOT, "pico", "Inc", "bno055.h")
PICO_C = os.path.join(ROOT, "pico", "hexapod_pico.c")
INDEX_HTML = os.path.join(HERE, "index.html")

RE_TABLE_ROW = re.compile(
    r'^\s*\{\s*"([a-z0-9_]+)"\s*,\s*&g_params\.(\w+)[^,]*,[^,]*,[^,]*,[^,]*,[^,]*,\s*"(\w+)"',
    re.M)
# 同上, 但把 min/max 数字也捕获出来 (名字-max 校验用; 只认同行写数字的行)
RE_TABLE_ROW_MAX = re.compile(
    r'^\s*\{\s*"([a-z0-9_]+)"\s*,\s*&g_params\.\w+\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,', re.M)
RE_NAME_LEN = re.compile(r"#define\s+STORE_PARAM_NAME_LEN\s+(\d+)u")
RE_PARAMS_MAX = re.compile(r"#define\s+STORE_PARAMS_MAX\s+(\d+)u")
# 前端用来认「刚存盘成功」的那条固件回显 (捕获括号里就是文案本身)
RE_SAVE_ACK = re.compile(r"if \(/\\\[STORE\\\] ([^/]+)/\.test\(ev\.line\)\)")

# 网页的分页表 (index.html 的 PAGES) 必须覆盖固件里出现的每一个分组,
# 否则那组参数在网页上无处可去 —— 加参数时忘了配页面就会静默丢一个组。
WEB_GROUPS = {"batt", "motion", "stance", "geo", "tune", "dir", "imu", "chan", "per",
              "sys", "modes", "port"}


def test_firmware_contract():
    """
    服务器与前端对参数表有几个隐含前提 (名字长度上限、字符集、与 config.h 的
    宏对应)。固件侧违反任何一条都不会编译报错, 只会静默失效 —— 例如名字超过
    STORE_PARAM_NAME_LEN 时 !CFG 查不到、存 flash 也被截断, 存了等于没存。
    """
    print("\n[2] 固件参数表契约")
    src = open(PARAMS_C, encoding="utf-8").read()
    rows = RE_TABLE_ROW.findall(src)
    store_h = open(STORE_H, encoding="utf-8").read()
    name_len = int(RE_NAME_LEN.search(store_h).group(1))
    params_max = int(RE_PARAMS_MAX.search(store_h).group(1))

    check("参数表可解析", len(rows) >= 30, f"{len(rows)} 行")
    check("名字与结构体字段同名 (宏重定向的前提)",
          all(n == f for n, f, _ in rows), str([n for n, f, _ in rows if n != f]))
    check("无重名参数", len({n for n, _, _ in rows}) == len(rows))

    # 分组: 每组都得在网页上有归属 (index.html 的 PAGES), 否则参数无处显示
    grps = {g for _, _, g in rows}
    check("参数表每个分组都被网页分页覆盖",
          grps <= WEB_GROUPS, "无归属: " + ", ".join(sorted(grps - WEB_GROUPS)))

    # 名字长度: 固件缓冲 20 字节, 服务器正则 {0,18} —— 两者必须一致
    too_long = [n for n, _, _ in rows if len(n) >= name_len]
    check(f"名字都能装进 STORE_PARAM_NAME_LEN={name_len}",
          not too_long, "超长: " + ", ".join(too_long))
    check(f"服务器 PARAM_NAME_OK 接受全部名字 (上限 {name_len - 1})",
          all(wc.PARAM_NAME_OK.match(n) for n, _, _ in rows),
          "被拒: " + ", ".join(n for n, _, _ in rows if not wc.PARAM_NAME_OK.match(n)))

    # 存 flash 时 params_save() 会把条数静默截到 STORE_PARAMS_MAX —— 参数表一旦
    # 超过它, 排在后面的参数就是"改得动、存不下", 重启回默认且全程没有报错。
    check(f"参数表条数不超过 STORE_PARAMS_MAX={params_max} (超出会被静默截断)",
          len(rows) <= params_max, f"{len(rows)} 行")

    cfg = open(CONFIG_H, encoding="utf-8").read()
    missing = [f for _, f, _ in rows if f"g_params.{f}" not in cfg]
    check("每个参数都有 config.h 宏指向它 (否则改了没人读)",
          not missing, "缺: " + ", ".join(missing))

    # 模式/组合键参数的取值范围必须容得下打包的全部位 (click 在 bit8 → 511,
    # hold 在 bit10 → 2047)。表里的 max 小于页面拼出来的值时, !CFG 会被
    # 静默夹掉高位 —— 勾了单击, 存下去变成按住, 全程无报错。
    pk_max = {n: int(hi) for n, _lo, hi in RE_TABLE_ROW_MAX.findall(src)
              if n.startswith(("mode_", "combo_"))}
    bad_max = [n for n, hi in pk_max.items()
               if (n.startswith("mode_") and hi != 511)
               or (n.startswith("combo_") and hi != 2047)]
    check("mode_* max=511 / combo_* max=2047 (装得下 click/hold 高位)",
          len(pk_max) == 19 and not bad_max,
          "越界: " + ", ".join(f"{n}={pk_max[n]}" for n in bad_max)
          + (" 漏解析" if len(pk_max) != 19 else ""))

    # ---- 状态行契约 ----
    # 解析样本 ([1]) 是从固件 printf 抄来的; 这里反过来钉住固件那几行。固件改了
    # 文案而样本没跟上时没有任何报错, 只是页面上的徽标永远停在"—"。
    hal = open(HAL_C, encoding="utf-8").read()
    core = open(CORE_C, encoding="utf-8").read()
    pico = open(PICO_C, encoding="utf-8").read()

    check("固件遥控门回执是严格两单词 (server 正则锚定整行, 加后缀就认不出)",
          '"[RC] %s\\r\\n"' in hal and '"locked" : "unlocked"' in hal)
    # 供电状态有两条独立来源: !O 的回显 (hal) 与状态跳变行 (core) —— 网页两个都收
    check("固件 !O 回显能被供电正则认出来",
          '"Robot %s\\r\\n"' in hal and '"ON" : "OFF"' in hal
          and wc.RE_ROBOT_PWR.match("Robot ON") is not None)
    check("固件状态跳变行 (带后缀) 同样被认出来",
          '"Robot ON (servo power enabled)\\r\\n"' in core
          and wc.RE_ROBOT_PWR.match("Robot ON (servo power enabled)") is not None)
    check("固件每 2s 状态摘要 [RUN]/[IDLE] 与样本一致",
          "[RUN] Travel X:%d Y:%d Z:%d Gait:%d Lift:%d Bal:%d St:%d Hi:%d" in pico
          and "[IDLE] " in pico
          and wc.RE_RUN_STATE.match(S_RUN[2]) is not None
          and wc.RE_RUN_STATE.match(S_RUN[3]) is not None)
    # 模式尾缀 (Bal/St/Hi): 网页「模式」页的实时徽标就靠它。三种 [IDLE] 变体都得
    # 带上, 否则待机时那几行会把徽标留在上一次的值上 —— 固件漏一处, 页面无报错
    check("三条 [IDLE] 变体都带模式尾缀",
          pico.count("Hi:%d\\r\\n") >= 4
          and wc.RE_RUN_MODE.search(S_RUN[3]) is not None
          and wc.RE_RUN_MODE.search(S_RUN[4]) is not None)
    # 外设行: 节名是 server 的白名单, 对不上就整行丢掉 (外设页少一行, 无报错)
    check("固件 !PERIPH 的 pwmperiod 行节名在 server 白名单里",
          "[PER] pwmperiod: l=%u r=%u" in hal
          and wc.RE_PER.match("[PER] pwmperiod: l=9500 r=9500") is not None)

    # IMU 校准复位 (!IMUR): 复位 BNO055、清空芯片里的校准值。两个易错点都只在
    # 真机上才现形, 静态钉住:
    #   ① 命令分发必须排在 !IMU 状态查询之前 —— 后者只看到 buf[2]=='M' 就认了
    #      (与 !I2C2/!I2C、!V/!VER 同一类前缀冲突), 排在后面就永远收不到 !IMUR;
    #   ② 复位后必须重新初始化 —— 融合停在复位前, 姿态读数会一直不动。
    bno = open(BNO_C, encoding="utf-8").read()
    check("固件 !IMUR 分发排在 !IMU 状态查询之前 (前缀冲突)",
          "imu_calib_reset(ctrl_state);" in hal
          and hal.index("!IMUR: IMU 校准复位") < hal.index("!IMU: IMU 状态"))
    check("BNO055 复位走 RST_SYS 并作废驱动状态 (读函数立刻失败)",
          "bool bno055_reset_sys(void)" in bno
          and "BNO055_REG_SYS_TRIGGER, BNO055_TRIG_RST_SYS" in bno
          and "g_initialized = false;" in bno)
    check("!IMUR 复位后重新初始化 (否则融合停在复位前)",
          "if (hal_imu_init()) {" in hal)
    check("!IMUR 已解锁时拒绝 (与 !SAVE 的 safe_context 同一判据)",
          "robot is armed, disarm first (!S)" in hal
          and "ctrl_state && ctrl_state->robot_on" in hal)

    # IMU 读看门狗 (2026-09-28 真机现场): I2C 事务会整段卡住几分钟, 期间八行读数
    # 全缺、last: 冻结, 而空闲时唯一的 I2C 恢复挂在舵机 flush 上 (它没待写舵机就
    # 提前返回) —— 等于没有任何自恢复。这几条静态钉住"看门狗确实挂上了", 因为
    # 挂漏了在真机上只表现为"IMU 又不动了", 没有任何报错。
    hal_h = open(HAL_H, encoding="utf-8").read()
    check("IMU 读看门狗挂在控制回路每周期 + !IMU 命令里 (校准模式控制回路提前返回)",
          "hal_imu_recover_step(robot->state.robot_on);" in core
          and "hal_imu_recover_step(ctrl_state && ctrl_state->robot_on);" in hal)
    check("看门狗两级恢复: 总线恢复 → 重新初始化 (含声明与失败计数)",
          "void hal_imu_recover_step(bool robot_on);" in hal_h
          and "void hal_imu_recover_step(bool robot_on)" in hal
          and "pca9685_i2c_recover();" in hal
          and "if (hal_imu_init()) {" in hal
          and "g_imu_fail_run++;" in hal)
    check("看门狗在解锁状态下不做重新初始化 (会阻塞主循环数秒)",
          "机器人已解锁, 跳过重新初始化" in hal)
    check("看门狗与失败计数都有打印 (原来整段卡住是全静默的)",
          '"[IMU] 读连续失败 %u 次' in hal
          and '"[IMU] 读数恢复 (连续失败 %u 次' in hal
          and '"fail: 连续 %u 次' in hal)

    # 硬件版本行 ([HW]) 与四条原始传感器行 (!IMU): 网页状态栏/曲线的数据源。
    # 文案对不上就整行丢掉 —— 页面没有任何报错, 只是版本栏永远"—"、曲线停在
    # "等待遥测数据…" (样本在 [1] 里, 解析正则在这里反过来钉固件的 printf)
    check("固件版本应答带 [HW] 行, 硬件版本来自 config.h 常量",
          '"[HW] %s\\r\\n"' in hal and "HEXAPOD_HW_VERSION" in hal
          and '#define HEXAPOD_HW_VERSION "PCB v2"' in cfg)
    def _imu_fixture(prefix):
        """夹具里那条以 prefix 开头的行 (按内容找, 不按下标 —— 表格增删行时不碎)"""
        return next(l for l in S_IMU if l.startswith(prefix))

    check("固件 !IMU 输出四条原始传感器行 (前缀与解析正则对齐)",
          all(f'"{k}: x=%d y=%d z=%d' in hal
              for k in ("accel", "gyro", "mag"))
          and '"temp: %d' in hal
          and wc.RE_IMU_ACC.match(_imu_fixture("accel:")) is not None
          and wc.RE_IMU_GYR.match(_imu_fixture("gyro:")) is not None
          and wc.RE_IMU_MAG.match(_imu_fixture("mag:")) is not None
          and wc.RE_IMU_TEMP.match(_imu_fixture("temp:")) is not None)
    # 日志面板折叠的边界就是这个块的表头/页脚, 一字之差折叠静默失效 (面板又回到
    # 25 行/秒); 块内放行的故障行前缀 "fail:" 也得是固件那行的开头
    check("IMU 状态块折叠边界与固件打印一字不差 (表头/页脚/块内故障行前缀)",
          f'"{IMU_BLOCK_HEAD}\\r\\n"' in hal
          and f'"{IMU_BLOCK_TAIL}\\r\\n"' in hal
          and _imu_fixture("fail:").startswith("fail:"))
    # 驱动侧: 寄存器地址 (BNO055 数据手册 Page 0) 与四个读函数, 以及单位注释 ——
    # 单位错了曲线还在画, 只是数值全错, 比不画更难发现
    bno_h = open(BNO_H, encoding="utf-8").read()
    check("BNO055 驱动有加速度/磁力/陀螺/温度读取 (寄存器地址对表)",
          all(s in bno for s in ("bool bno055_read_accel", "bool bno055_read_mag",
                                 "bool bno055_read_gyro", "bool bno055_read_temp"))
          and all(s in bno_h for s in ("BNO055_REG_ACC_DATA_X_LSB   0x08",
                                       "BNO055_REG_MAG_DATA_X_LSB   0x0E",
                                       "BNO055_REG_GYR_DATA_X_LSB   0x14",
                                       "BNO055_REG_TEMP             0x34")))
    check("BNO055 原始数据的单位在头文件里写明 (0.01 m/s² / 1/16 dps / 1/16 uT)",
          "0.01 m/s²" in bno_h and "1/16 dps" in bno_h and "1/16 uT" in bno_h)

    # 舵机机械中心 (!HO 单路 / !HOS 全量): 网页校准页的写入与回读通道。四个易错点
    # 都只在真机上才现形, 静态钉住:
    #   ① 角度约定 0=中位 1500µs —— 走查旧码拿 pulse(900) 当"回中位" (实为 2500µs
    #      端止点), 导出的偏移整体差 900。!PER 修过同款 bug, 这里钉死"回中位一律 0";
    #   ② 偏移真源只能是 leg_configs 的 horn_offset 字段 (!C 与 !HO 共用
    #      calib_horn_record) —— 另存一份数组就等着两边不一致;
    #   ③ !HO 与 !C 走的都是安全路径: armed 时拒绝 (网页上改了也当没改);
    #   ④ 回读必须经 boot 回放种子 (hal_calib_import) —— 不然 boot 后网页只标一路
    #      再 !SAVE 就把其余 17 路存量偏移抹掉。
    re_ho = getattr(wc, "RE_HO", None)   # 缺失时按"不认"处理, 不炸整段契约
    check("server 与固件对 [HO] 行的格式约定一致",
          '"[HO] id=%u off=%d' in hal and re_ho is not None
          and all(re_ho.match(l) is not None for l in S_HO))
    check("固件「回中位」一律用 0 (=1500µs), 无 pulse(900) 残留",
          "pca9685_angle_to_pulse(0)" in hal
          and "pca9685_angle_to_pulse(900)" not in hal)
    check("!HO 的偏移真源就是 leg_configs 的 horn_offset 字段 (与 !C 共用)",
          "static void calib_horn_record(uint8_t id, int16_t angle)" in hal
          and hal.count("calib_horn_record(") >= 3
          and "&cfg->coxa_horn_offset" in hal
          and "g_calib_robot->leg_configs[id / 3]" in hal)
    check("!HO 在 armed 时拒绝 (与 !SAVE 同一判据)",
          "[HO] Refuse: robot is armed, disarm first (!S)" in hal)
    check("boot 回放把校准记录种回真源 (hal_calib_import)",
          "hal_calib_import(" in hal
          and "hal_calib_import(" in open(STORE_C, encoding="utf-8").read())
    check("机器人指针在 robot_init 之后绑定 (hal_calib_bind_robot)",
          "hal_calib_bind_robot(&g_robot);" in pico)

    # 保存按钮的基线靠这句固件回显重建 (网页认回显而不是「点了按钮」, 手敲 !CFGW
    # 和被固件拒绝的情况才都能正确)。改文案而不同步前端 → 保存后按钮一直亮着。
    m = RE_SAVE_ACK.search(open(INDEX_HTML, encoding="utf-8").read())
    check("网页按固件回显重建保存基线", m is not None)
    if m:
        store_c = open(STORE_C, encoding="utf-8").read()
        check(f"固件确实回显 {m.group(1)!r}", m.group(1) in store_c)

    # 校准页的逐舵机面板: 前端写 !HO、后端认 [HO] 行, 两边的名字与语法对不上时
    # 页面上没有任何报错 (点「设为中心」静默无效)。这里钉住接线的两端。
    html = open(INDEX_HTML, encoding="utf-8").read()
    check("校准页有逐舵机面板 (选中格 → 滑条 → 设为中心)",
          'id="cal-edit"' in html and 'id="cal-ang"' in html
          and 'id="cal-center"' in html and "设为中心" in html)
    check("「设为中心」下发 !HO <id> <angle> (空格是分隔符)",
          "`!HO ${CAL.sel} ${v}`" in html)
    # !P 这条相反: id 必须紧跟命令字, 中间不给空格。固件从 buf[2] 读 id、第一个
    # 空格之后读角度, "!P 0 300" 会被读成"舵机 0 → 角度 0"(角度取到 id 的首位),
    # 静默写到别处 —— 页面上只有"滑条拖了舵机几乎不动", 没有任何报错。
    check("面板滑条转动走 !P<id> <angle> (id 紧跟 !P)",
          "`!P${CAL.sel} ${v}`" in html and "`!P ${CAL.sel} ${v}`" not in html)
    check("固件 !P 的形状与源码写的一致 (页面照它发)",
          "!P<servo_id> <angle>" in hal and "Usage: !P<id> <angle>" in hal)
    # 形状对不上要打 Usage 拒绝, 不许"猜" —— 老固件 (len<5 + parse_int(buf,2) +
    # "跳过 id 找到空格后的 angle") 对 "!P 0 300" 会静默读成舵机 0 角度 0。
    # 两处 Usage 对应两种残形: id 不是数字 (含 "!P" 后面直接没东西)、角度缺失。
    check("固件 !P 形状不对就拒绝 (id 与角度各一处 Usage, 不静默猜)",
          hal.count("Usage: !P<id> <angle>") >= 2
          and "跳过 id 找到空格后的 angle" not in hal)
    check("页码缓存 ho 帧并挂到 SSE 分发上",
          'case "ho": renderHo(ev.d); break;' in html)
    # !A 报的是角度而不是脉宽 (同样是 0=中位那套单位, 与 !P/!HO 一致)。改成
    # .pulses 会让校准页的滑条整体差 1500, 被自己的 min/max 夹成恒 ±900 ——
    # [5] 的探针会真喂一帧抓这个 (那边是行为检查, 这里是格式约定)
    check("!A 报的是角度 (与 !P 同一套单位), 不是脉宽",
          "g_servo_batch.angles[i]" in hal)

    # 端口页 (!PORTS / !GPO 与 port 组的六个参数)。这批的坑全在"静默"上:
    #   ① 分发顺序 —— !PORTS 若落到 switch 里, case 'P' 会把它读成 "!P<id>
    #      <angle>", 舵机 0 真的会转到中位; !GPO 落到 case 'G' 则一声不吭。
    #      两者的判据只有"拦在 switch 之前", 静态可查;
    #   ② 关掉的外设必须拒绝, 不能静默记账 —— !MOTOR 会回显一个没发生的占空比;
    #   ③ 六个参数挂在 PARAM_HIDE 里, 页面若拼错名字, 参数就从页面上凭空消失。
    check("server 与固件对 [PORTS] 行的格式约定一致",
          '"[PORTS] uart0: en=%ld baud=%ld' in hal
          and getattr(wc, "RE_PORTS", None) is not None
          and all(wc.RE_PORTS.match(l) is not None for l in S_PORTS))
    # find 而不是 index: 差分检查时旧固件里没有这两段, 要的是"红"不是 ValueError
    i_ports = hal.find("if (buf[1] == 'P' && len >= 6 &&")
    i_gpo = hal.find("if (buf[1] == 'G' && len >= 4 &&")
    check("!PORTS 拦在 switch 之前 (case 'P' 会让舵机 0 真转到中位)",
          -1 < i_ports < hal.index("case 'P':") and "ports_status_print();" in hal)
    check("!GPO 拦在 switch 之前 (case 'G' 会静默吞掉整条命令)",
          -1 < i_gpo < hal.index("case 'G':") and "cmd_gpo(buf, len);" in hal)
    check("!GPO 只认三个空闲脚, 且输入态拒绝写电平",
          "free_gpio_index(pin)" in hal and "g_free_gpio_pins[3] = {23, 24, 29}" in hal
          and "[PORTS] GPO Refuse: gp%ld fn=%ld (need 1=output)" in hal)
    check("电机被收回 (dc_motor_en=0) 时 !MOTOR 明确拒绝, 不静默记账",
          "[PORTS] motors: disabled (dc_motor_en=0)" in hal
          and "if (!DC_MOTOR_ENABLED) return;" in hal)
    check("足端检测关掉时读 API 一律报未触地 (不把上拉当触地)",
          hal.count("if (!FOOT_SW_ENABLED)") >= 2)
    check("端口功能由快照-应用驱动 (改完即时生效, 无需重启)",
          "static int32_t s_dc_motor  = -1;" in hal
          and "static int32_t s_foot_sw   = -1;" in hal
          and "static int32_t s_gp_fn[3]  = {-1, -1, -1};" in hal)
    check("串口输入的波特率可运行期生效 (真机 CRSF 不受影响)",
          "static void input_uart_serial_setup(void)" in hal
          and "!g_crsf_mode && INPUT_BAUD_SERIAL != s_serial_baud" in hal)
    check("端口页上报的是实际生效的 UART1 波特率, 不是参数值",
          "s_input_baud_actual = uart_init(" in hal
          # 取定义那一份 (前面还有前置声明与分发点两处同名)
          and "s_input_baud_actual" in hal[hal.rindex("void ports_status_print(void)"):])
    # 六个 port 参数必须真的在固件表里 (缺一个 = 页面上那个下拉永远下发失败)。
    # [2] 段已校验名字长度/字符集, 这里只管"是不是 port 组的、是不是六个都全"。
    port_rows = [n for n, _, g in rows if g == "port"]
    check("固件表里有 port 组的六个参数",
          set(port_rows) == {"dc_motor_en", "foot_sw_en", "gp23_fn", "gp24_fn",
                             "gp29_fn", "input_baud_serial"},
          str(sorted(port_rows)))
    # PARAM_HIDE 把它们的通用滑条摘掉了, 所以页面必须在别处引用每个名字 ——
    # 名字拼错时下拉不认它, 参数就整个从页面上消失, 且不报任何错
    hide_blk = re.search(r"const PARAM_HIDE = new Set\(\[.*?\]\);", html, re.S)
    rest = html.replace(hide_blk.group(0), "") if hide_blk else html
    missing = [n for n in port_rows if f'"{n}"' not in rest]
    check("六个 port 参数都挂进了端口页的下拉 (PARAM_HIDE 之外有引用)",
          not missing, "无引用: " + ", ".join(missing))
    check("网页有端口页 (tab + PAGES + SSE 分发 + 初始化)",
          'data-tab="ports"' in html and 'id: "ports"' in html
          and 'case "ports": renderPorts(ev.d); break;' in html
          and "initPortsPage();" in html and "renderPortsConfig();" in html)
    check("空闲脚的电平走即时命令 !GPO, 不走参数 (电平本来就不是能存的状态)",
          "`!GPO ${r.pin} ${e.target.checked ? 1 : 0}`" in html
          and 'id="pt-stale"' in html)


def test_launcher_contract():
    """启停脚本 (run.py) 的契约。

    这里钉的是两条"错了会伤到别的东西"的性质: 默认端口必须跟着 server.py 走,
    进程识别必须保持两道核实 —— 退回"命令行文本里出现脚本名就算"的话, 一条
    git commit / grep 所在的 shell 会被当成 server 进程, stop 就把人家终端杀了。
    """
    print("\n[2b] 启停脚本")
    launcher = os.path.join(HERE, "run.py")
    check("run.py 在位", os.path.isfile(launcher))
    src = open(launcher, encoding="utf-8").read()

    r = subprocess.run([sys.executable, launcher, "help"],
                       capture_output=True, text=True, timeout=20)
    check("help 一条命令列全五个子命令且退出码 0",
          r.returncode == 0
          and all(f" {c} " in r.stdout or f" {c}\n" in r.stdout
                  for c in ("start", "stop", "restart", "status", "logs")),
          r.stdout.splitlines()[0] if r.stdout else r.stderr.strip()[:60])

    # 默认端口: server.py 的 argparse 默认值 = 脚本的默认值 = 8080
    srv = open(os.path.join(HERE, "server.py"), encoding="utf-8").read()
    m = re.search(r'"--port",\s*type=int,\s*default=(\d+)', srv)
    check("默认端口与 server.py 一致 (改了服务端端口, 脚本会自动跟上)",
          bool(m) and f'HEXAPOD_WC_PORT") or {m.group(1)}' in src,
          m.group(1) if m else "server.py 里没找到 --port 默认值")

    check("进程识别保持两道核实 (exe 是 python + 参数就是脚本路径), "
          "不接受纯文本匹配",
          "/proc/" in src and "_pid_exe" in src and "args[1:]" in src
          and "webconfig/server.py" in src)
    check("没有退回按命令行文本收网的写法 (老 bug: 误杀别人的终端)",
          "pgrep" not in src)
    check("Windows 那条路也在 (netstat 反查端口 PID → tasklist 核映像名)",
          '"netstat", "-ano"' in src and '"tasklist"' in src)
    check("串口桥只能被查询, 不能被终止 (BRIDGE_SCRIPT 只进 bridge_pid)",
          src.count("BRIDGE_SCRIPT") == 2
          and "BRIDGE_SCRIPT" in src.split("def bridge_pid")[1].split("def ")[0])


# ==================== [3] 端到端 ====================

# 假固件的参数表: name → [val, min, max, def, unit, grp]
# 名字取自真实固件表 (由 [2] 校验), 覆盖开关 / 空 unit / 带 unit 三种形态
FAKE_PARAMS = {
    "batt_check":    [0,    0,    1,    0,    "",   "batt"],
    "imu_enabled":   [0,    0,    1,    0,    "",   "imu"],
    "batt_ov_mv":    [8800, 8000, 9500, 8800, "mV", "batt"],
    "travel_fwd_mm": [150,  20,   250,  150,  "mm", "motion"],
    "imu_roll_sign": [-1,   -1,   1,    -1,   "",   "imu"],
    # 模式页的两个打包参数 (mode_arm = CH5 高, combo_arm = START 单击):
    # 它们在页面上是下拉框 + 勾选框, 但走的仍是同一套 参数帧 / set 通道
    "mode_arm":      [36,   0,    511,  36,   "",   "modes"],
    "combo_arm":     [515,  0,    2047, 515,  "",   "modes"],
    # 几何页的一个普通滑条参数 (带单位, 只为覆盖页面生成)
    "leg_coxa_mm":   [45,   20,   80,   45,   "mm", "geo"],
    # 端口页的六个 (port 组): 页面上是下拉框, 走同一套 参数帧 / set 通道 ——
    # PARAM_HIDE 把它们从通用滑条里摘掉, 但值仍在参数表里, 保存/恢复默认照常
    "dc_motor_en":   [1,    0,    1,    1,    "",   "port"],
    "foot_sw_en":    [1,    0,    1,    1,    "",   "port"],
    "gp23_fn":       [1,    0,    1,    0,    "",   "port"],
    "gp24_fn":       [0,    0,    1,    0,    "",   "port"],
    "gp29_fn":       [0,    0,    1,    0,    "",   "port"],
    "input_baud_serial": [115200, 2400, 1000000, 115200, "baud", "port"],
    # 外设页那一组里与端口页共用显示的两个 (端口页只换了个控件的样子)
    "uart0_en":      [1,    0,    1,    1,    "",   "per"],
    "uart0_baud":    [115200, 2400, 1000000, 115200, "baud", "per"],
}

# 假固件的空闲脚电平 (真固件里在 SIO 的输出寄存器上): !GPO 写它, !PORTS 报它。
# 归属 (gpXX_fn) 是参数, 留在 FAKE_PARAMS 里 —— 与真固件一样, "这脚是谁的"与
# "这脚现在什么电平"是两回事。
FAKE_PORTS = {"g23lv": 1, "g24lv": 0, "g29lv": 1}


def gpio_line():
    """按 ports_print_gpio() 的格式生成 gpio 行 (归属取参数, 电平取 FAKE_PORTS)"""
    out = ["[PORTS] gpio:"]
    for pin in (23, 24, 29):
        out.append(f"g{pin}fn={FAKE_PARAMS[f'gp{pin}_fn'][0]}")
        out.append(f"g{pin}lv={FAKE_PORTS[f'g{pin}lv']}")
    return " ".join(out)


def param_line(name):
    """按 params_print_one() 的格式生成一行 (unit 为空时就是 "unit= grp=")"""
    v, lo, hi, d, unit, grp = FAKE_PARAMS[name]
    return (f"[P] name={name} val={v} min={lo} max={hi} def={d} "
            f"unit={unit} grp={grp} chg={1 if v != d else 0}")


def fake_cfg(cmd):
    """假固件的 !CFG 家族: 按参数表动态应答, 含钳位与错误提示"""
    parts = cmd.split()
    op = parts[0]

    if op == "!CFGW":
        n = sum(1 for v in FAKE_PARAMS.values() if v[0] != v[3])
        return [f"[STORE] Params saved OK ({n} 项, CRC 0x1A2B3C4D)"]

    if op == "!CFGR":
        if len(parts) == 1:
            for v in FAKE_PARAMS.values():
                v[0] = v[3]
            return (["[P] 全部参数已恢复默认 (尚未写 flash, 需 !CFGW 才持久)"]
                    + [param_line(n) for n in FAKE_PARAMS])
        if parts[1] not in FAKE_PARAMS:
            return [f"[P] FAIL unknown param '{parts[1]}'"]
        FAKE_PARAMS[parts[1]][0] = FAKE_PARAMS[parts[1]][3]
        return [param_line(parts[1])]

    if len(parts) == 1:                                   # !CFG → 全量
        changed = sum(1 for v in FAKE_PARAMS.values() if v[0] != v[3])
        return ([f"[P] === {len(FAKE_PARAMS)} params ({changed}) ==="]
                + [param_line(n) for n in FAKE_PARAMS] + ["[P] === end ==="])

    name = parts[1]
    if name not in FAKE_PARAMS:
        return [f"[P] FAIL unknown param '{name}' (试 !CFG 列出全部)"]
    if len(parts) == 2:                                   # 只查询
        return [param_line(name)]
    try:
        want = int(parts[2])
    except ValueError:
        return [f"[P] FAIL bad value for '{name}' (需要整数)"]
    row = FAKE_PARAMS[name]
    got = max(row[1], min(row[2], want))
    row[0] = got
    out = []
    if got != want:
        out.append(f"[P] WARN {name} clamped {want} -> {got} (范围 {row[1]}~{row[2]})")
    out.append(param_line(name))                          # 回显实际生效值
    return out


def test_imu_block_fold():
    """IMU 状态块的折叠状态机 (server.py 的 imu_block_fold, 页面同款)。

    测的是真函数 (不是抄一份): 四个角 —— 块外不折、表头到页脚之间折、块内的
    fail: 行放行、页脚没等到时按上限兜底 (一次半行丢失不能把日志永久堵死)。"""
    f = wc.imu_block_fold
    st, fold = f(0, "calib: sys=3 gyr=3 acc=3 mag=0")
    check("块外: 不折叠", (st, fold) == (0, False), f"st={st} fold={fold}")
    st, fold = f(st, IMU_BLOCK_HEAD)
    check("表头: 进块, 且表头自己不显示", st == 1 and fold is True, f"st={st} fold={fold}")
    st, fold = f(st, "INT_STA=0x80 (bit7=BSX_DRDY)")
    check("块内杂项行 (INT_STA 等): 折叠", st == 2 and fold is True, f"st={st} fold={fold}")
    st, fold = f(st, "fail: 连续 3 次 (已持续 31ms) / 累计 9 次")
    check("块内的 fail: 行: 放行 (折叠不藏故障)", st == 3 and fold is False, f"st={st} fold={fold}")
    st, fold = f(st, IMU_BLOCK_TAIL)
    check("页脚: 出块, 且页脚自己不显示", (st, fold) == (0, True), f"st={st} fold={fold}")
    st, fold = f(st, "[IMU] 读连续失败 20 次 (约 210ms, 累计 57 次) → 总线恢复")
    check("出块后的 [IMU] 警告行: 不折叠 (它本来就在块外)", (st, fold) == (0, False))
    st = 1
    for _ in range(IMU_BLOCK_MAX):
        st, fold = f(st, "表头之后页脚迟迟不来")
    check("页脚丢失: 到上限自己出块, 别把日志面板堵死", (st, fold) == (0, True), f"st={st}")
    st, fold = f(st, "兜底出块之后的行照常显示")
    check("兜底出块后的行: 不折叠", (st, fold) == (0, False))


def test_parser():
    print("\n[1] 解析器单元测试")
    t = wc.Telemetry()

    for line in S_BATT:
        wc.parse_line(t, line)
    b = t.snapshot()["batt"]
    check("电池: 解析出换算电压/引脚电压", b.get("batt_mv") == 8370 and b.get("pin_mv") == 1029)
    check("电池: 解析出离散度", b.get("spread_mv") == 12 and b.get("spread_counts") == 15)
    check("电池: 解析出阈值与判定", b.get("lo_mv") == 6600 and b.get("in_range") is True)
    check("电池: 解析出保护开关状态", b.get("enabled") == 0)
    check("电池: 解析出三态判定", b.get("state") == "OK")

    # 未接电池 (USB 供电) —— 电压 0mV 且 OUT OF RANGE, 但 state=ABSENT,
    # 前端据此显示"USB 供电"而不是低压告警 (独立于 in_range)
    t = wc.Telemetry()
    for line in S_BATT_ABSENT:
        wc.parse_line(t, line)
    ba = t.snapshot()["batt"]
    check("电池: USB 供电解析为 ABSENT", ba.get("state") == "ABSENT", str(ba.get("state")))
    check("电池: USB 供电时电压为 0 且窗口外",
          ba.get("batt_mv") == 0 and ba.get("in_range") is False)
    check("电池: USB 供电时保护开关仍如实上报", ba.get("enabled") == 1)

    t = wc.Telemetry()
    for line in S_I2C:
        wc.parse_line(t, line)
    i = t.snapshot()["i2c"]
    check("I2C: 两块 PCA9685 均在线",
          [d["addr"] for d in i["pca"]] == [0x40, 0x41] and all(d["ok"] for d in i["pca"]))
    check("I2C: BNO055 区分在线/未找到",
          {d["addr"]: d["ok"] for d in i["bno"]} == {0x28: False, 0x29: True})
    check("I2C: 全扫描结果", i["scan"] == [0x29, 0x40])
    check("I2C: 总线空闲电平", i["idle"] == {"sda": 1, "scl": 1})
    check("I2C: 舵机供电脚", i["servo_power"] == {"left": 1, "right": 1})

    # 每个 !I2C 头行都是一轮新检测: 必须先作废上一轮结果, 否则设备掉线后
    # 卡片会永远显示一个已经不在的设备。
    # snapshot() 会丢掉空子系统, 故用 .get —— 头行之后 i2c 整个为空是预期
    r = wc.parse_line(t, "=== I2C Bus Check (SDA=GP14, SCL=GP15) ===")
    i = t.snapshot().get("i2c", {})
    check("I2C: 新一轮检测作废上一轮设备 (掉线不残留)",
          "pca" not in i and "bno" not in i, str(sorted(i)))
    check("I2C: 全扫描头行要求连扫描列表一起清",
          r[1].get("reset") is True and r[1].get("reset_scan") is True, str(r[1]))

    # 轮询用的 !I2C q 不带扫描结果, 若也清列表, 用户刚手动扫出来的东西
    # 会在 3 秒后消失。
    wc.parse_line(t, "  Device found at 0x40")
    r = wc.parse_line(t, "=== I2C Bus Check (SDA=GP14, SCL=GP15) quick ===")
    check("I2C: quick 头行保留扫描列表",
          r[1].get("reset") is True and r[1].get("reset_scan") is False, str(r[1]))
    scan = t.snapshot().get("i2c", {}).get("scan")
    check("I2C: quick 头行不清服务端扫描列表 (与前端语义一致)",
          scan == [0x40], str(scan))

    t = wc.Telemetry()
    for line in S_IMU:
        wc.parse_line(t, line)
    m = t.snapshot()["imu"]
    check("IMU: 在线/地址", m.get("available") is True and m.get("addr") == 0x29)
    check("IMU: 欧拉角 (0.1deg 原始值)", (m.get("roll"), m.get("pitch"), m.get("yaw"))
          == (-12, 34, 567) and m.get("valid") == 1)
    check("IMU: 校准等级与 fully 标记",
          m["calib"] == {"sys": 3, "gyr": 3, "acc": 3, "mag": 0} and m["fully"] is True)
    # 原始传感器 (网页曲线数据源): 三轴完整 + 负号保真 + 温度有符号
    check("IMU: 加速度三轴 (含负轴)", m.get("acc") == {"x": 12, "y": 5, "z": -1002},
          str(m.get("acc")))
    check("IMU: 陀螺仪三轴 (含负轴)", m.get("gyr") == {"x": 0, "y": 0, "z": -45},
          str(m.get("gyr")))
    check("IMU: 磁力计三轴 (含负轴)", m.get("mag") == {"x": -32, "y": 64, "z": 0},
          str(m.get("mag")))
    check("IMU: 温度 (可负的 int8)", m.get("temp") == 31, str(m.get("temp")))

    t = wc.Telemetry()
    for line in S_SERVO:
        wc.parse_line(t, line)
    s = t.snapshot()["servo"]["boards"]
    check("舵机: 左右板 18 路角度",
          s["41"]["side"] == "right" and s["41"]["angles"][2] == 1520
          and len(s["40"]["angles"]) == 9 and s["40"]["angles"][17] == 1500)

    # 通道遥测: 每帧都是完整的, 所以整份覆盖 —— 切模式后不能残留上一种模式的字段
    t = wc.Telemetry()
    for line in S_CH_CRSF:
        wc.parse_line(t, line)
    c = t.snapshot()["ch"]
    check("通道: CRSF 模式与链路/帧计数",
          c.get("mode") == "crsf" and c.get("link") is True and c.get("fc") == 123456)
    check("通道: 16 个原始值逐个解析",
          c.get("ch", [])[:5] == [172, 1500, 992, 992, 1811] and len(c["ch"]) == 16)
    check("通道: 未报的通道位回填中位 992", c["ch"][15] == 992)

    t2 = wc.Telemetry()
    wc.parse_line(t2, S_CH_PS2[0])
    p = t2.snapshot()["ch"]
    check("通道: PS2 摇杆与键位",
          p.get("mode") == "ps2" and p.get("lx") == 128 and p.get("ry") == 126
          and p.get("btns") == 65535 and p.get("connected") is True)

    # 切回 CRSF (同一 Telemetry) —— ps2 的摇杆字段必须消失, 否则前端会把
    # 上一个模式的 lx 当成 CRSF 帧的字段去渲染
    wc.parse_line(t2, S_CH_CRSF[0])
    c2 = t2.snapshot()["ch"]
    check("通道: 切模式后不残留上一模式的字段",
          "lx" not in c2 and "btns" not in c2 and c2["mode"] == "crsf", str(sorted(c2)))

    # 外设状态: 一次 !PERIPH 是 7 行, 每行一节。分节合并 (而不是整份覆盖),
    # 是为了让前端在只收到一半时也有东西可渲染。
    t = wc.Telemetry()
    kinds = [wc.parse_line(t, l) for l in S_PER]
    check("外设: 7 行各成一节", [k[0] if k else None for k in kinds] == ["per"] * 7)
    per = t.snapshot()["per"]
    check("外设: 电机占空比", per.get("motors") == {"m1": 500, "m2": 0}, str(per.get("motors")))
    check("外设: LED 实况与所有权参数一起报",
          per.get("leds") == {"g": 1, "r": 0, "hb": 1, "alarm": 1}, str(per.get("leds")))
    check("外设: 蜂鸣器最后一次发声",
          per.get("buzzer") == {"freq": 1500, "ms": 300}, str(per.get("buzzer")))
    check("外设: UART0 使能/波特率/收发计数",
          per.get("uart0") == {"en": 1, "baud": 115200, "rx": 42, "tx": 7, "lines": 3},
          str(per.get("uart0")))
    check("外设: 第二路 I2C/ADC 模式与读数",
          per.get("ext") == {"mode": 2, "found": 0, "a0": 1234, "a1": 5678},
          str(per.get("ext")))
    check("外设: 14 路空闲 PWM 按 idx 编号",
          len(per.get("pwm", {})) == 14 and per["pwm"]["3"] == 5000 and per["pwm"]["1"] == 1500,
          str(per.get("pwm")))
    # PWM 周期校准值 (µs): 控制页滑条的回读源 —— 两板各自的数, 不是一份
    check("外设: 两板 PWM 周期各自回报",
          per.get("pwmperiod") == {"l": 9500, "r": 9500}, str(per.get("pwmperiod")))

    # 分节合并: 只来一行时, 之前那几节必须还在 (前端整页渲染, 缺节会闪 0)
    wc.parse_line(t, "[PER] motors: m1=0 m2=0")
    per2 = t.snapshot()["per"]
    check("外设: 单节更新不动其他节",
          per2["motors"] == {"m1": 0, "m2": 0} and per2["uart0"]["baud"] == 115200,
          str(sorted(per2)))

    # 端口状态: 一次 !PORTS 是 7 行, 每行一节 (与外设页同一套分节合并)
    t = wc.Telemetry()
    kinds = [wc.parse_line(t, l) for l in S_PORTS]
    check("端口: 7 行各成一节", [k[0] if k else None for k in kinds] == ["ports"] * 7)
    # 缺 "ports" 键时按空表处理 —— 让上面那条 FAIL 说明问题, 而不是 KeyError
    # 把整段测试打断 (差分检查时后面的契约段就再也跑不到了)
    pt = t.snapshot().get("ports", {})
    check("端口: UART0 使能与波特率",
          pt.get("uart0") == {"en": 1, "baud": 115200}, str(pt.get("uart0")))
    check("端口: 输入模式与实际生效波特率",
          pt.get("input") == {"mode": 0, "baud": 420000}, str(pt.get("input")))
    check("端口: 电机使能与两路电平",
          pt.get("motors") == {"en": 1, "lv1": 0, "lv2": 1}, str(pt.get("motors")))
    # 六路足端开关: 保持原始电平 (低=触地), 不下任何"触地"判定 ——
    # 极性是硬件接法, 固件这一页只报引脚真相
    check("端口: 六路足端电平逐路",
          pt.get("foot") == {"en": 1, "s0": 1, "s1": 0, "s2": 1, "s3": 1, "s4": 1, "s5": 1},
          str(pt.get("foot")))
    check("端口: 外部排针模式与 ADC 读数",
          pt.get("ext") == {"mode": 0, "a0": 0, "a1": 0}, str(pt.get("ext")))
    check("端口: 三路空闲脚的用途与电平成对出现",
          pt.get("gpio") == {"g23fn": 1, "g23lv": 1, "g24fn": 0, "g24lv": 0,
                             "g29fn": 0, "g29lv": 1}, str(pt.get("gpio")))
    check("端口: 固定脚只报有电平意义的几个",
          pt.get("fixed") == {"spwl": 1, "spwr": 1, "ledr": 0, "ledg": 1, "boot": 1},
          str(pt.get("fixed")))
    # 分节合并: !GPO 之后固件只重打 gpio 一行, 另外六节必须还在
    wc.parse_line(t, "[PORTS] gpio: g23fn=1 g23lv=0 g24fn=0 g24lv=0 g29fn=0 g29lv=1")
    pt2 = t.snapshot().get("ports", {})
    check("端口: 单节更新不动其他节 (GPO 只回 gpio 行)",
          pt2.get("gpio", {}).get("g23lv") == 0
          and pt2.get("uart0", {}).get("baud") == 115200, str(sorted(pt2)))

    # 舵盘偏移: 18 行累积成 {id: 偏移}, 单路回执与全量同一格式
    t = wc.Telemetry()
    kinds = [wc.parse_line(t, l) for l in S_HO]
    check("舵盘偏移: 18 行各产生一帧 ho", [k[0] if k else None for k in kinds] == ["ho"] * 18)
    # 缺 "ho" 状态键时按空表处理 —— 让上面那条 FAIL 说明问题, 而不是在后面
    # KeyError 把整段测试打断 (契约段就再也跑不到了)
    ho = t.snapshot().get("ho", {}).get("offs", {})
    check("舵盘偏移: 18 路齐全且带负值",
          len(ho) == 18 and ho.get(0) == -10 and ho.get(3) == 12 and ho.get(9) == -25,
          str(sorted(ho.items())[:3]))
    # 单路回执只动那一路 (校准页点一次「设为中心」就靠这个即时更新)
    r = wc.parse_line(t, "[HO] id=3 off=-77")
    ho = t.snapshot().get("ho", {}).get("offs", {})
    check("舵盘偏移: 单路回执并入同一张表",
          r and r[0] == "ho" and ho.get(3) == -77 and ho.get(0) == -10, str(r))
    check("舵盘偏移: 非法行不产生帧 (id 越界/别的 [H] 前缀)",
          wc.parse_line(wc.Telemetry(), "[HO] Invalid: id=99 (0..17)") is None)

    # 外部 UART0: 回执 + 收行。收行留在环形缓冲里累积 (不是整份覆盖)
    t = wc.Telemetry()
    kinds = [wc.parse_line(t, l) for l in S_U0]
    check("[U0TX]/[U0] 各产生一帧", [k[0] if k else None for k in kinds] == ["per", "per"])
    p = t.snapshot()["per"]
    check("UART0 回执: 字节数与内容",
          p.get("u0_tx") == "hello" and p.get("u0_tx_bytes") == 5, str(p.get("u0_tx")))
    check("UART0 收到整行进环形缓冲", p.get("u0_rx") == ["hello"], str(p.get("u0_rx")))

    for i in range(wc.U0_RX_RING + 5):
        wc.parse_line(t, f"[U0] line{i}")
    ring = t.snapshot()["per"]["u0_rx"]
    check(f"UART0 收行环形缓冲只留最近 {wc.U0_RX_RING} 行",
          len(ring) == wc.U0_RX_RING and ring[-1] == f"line{wc.U0_RX_RING + 4}"
          and ring[0] == "line5", f"{len(ring)} 行: {ring[:2]}")

    # [PER] 这个前缀同时是 PCA9685 周期校准 (!PER) 的输出前缀 —— 白名单必须挡住,
    # 否则校准行会被当成外设节, 在页面上显示成一台不存在的设备
    t = wc.Telemetry()
    for cal in ["[PER] Coxa write: 6/6 OK (horn_offset: off)",
                "[PER] Board 0 (0x40, left legs): period=20000 us (~50 Hz)",
                "[PER] i2c2 scan: 0x40 0x41",
                "[PER] ext_i2c_mode=0, 先 !CFG ext_i2c_mode 1"]:
        check(f"校准/诊断行不算外设节 ({cal[6:26]}…)",
              wc.parse_line(t, cal) is None)
    check("白名单外的前缀没有污染外设状态", "per" not in t.snapshot(), str(t.snapshot().get("per")))

    # 固件版本 ([VER] 横幅/!VER): 开始页在"连接"之前就靠它显示板上固件
    t = wc.Telemetry()
    r = wc.parse_line(t, S_VER[0])
    check("[VER] 解析为 ver 帧", r is not None and r[0] == "ver", str(r))
    check("[VER] 版本原文整行保留 (含 git 哈希/dirty 标记)",
          r and r[1].get("text") == "Hexapod v0.3.0-16-g8f5b04f-dirty", str(r[1]))

    # 硬件版本 ([HW], 紧跟在 [VER] 后): 状态栏与开始页的"跑在哪版板子上"
    r = wc.parse_line(t, S_VER[1])
    check("[HW] 解析为 hw 帧", r is not None and r[0] == "hw", str(r))
    check("[HW] 硬件版本原文保留", r and r[1].get("text") == "PCB v2", str(r[1]))
    check("[HW] 与 [VER] 分属两个子系统 (不互相覆盖)",
          t.snapshot().get("ver", {}).get("text") == "Hexapod v0.3.0-16-g8f5b04f-dirty"
          and t.snapshot().get("hw", {}).get("text") == "PCB v2")

    # 设备在线状态行 ([TCP] dev ...) 由 serial_console 发布, 走的是桥接路径
    # (on_line 里最先判定) —— 这里只验正则本身认哪些、不认哪些
    check("[TCP] dev 行: 在线/占用/离线都认",
          [bool(wc.RE_DEV.match(l)) for l in
           ("[TCP] dev on /dev/ttyACM0", "[TCP] dev busy /dev/ttyACM0",
            "[TCP] dev off")] == [True, True, True])
    m = wc.RE_DEV.match("[TCP] dev on /dev/ttyACM0")
    check("[TCP] dev 行: 解析出状态与端口",
          (m.group(1), m.group(2)) == ("on", "/dev/ttyACM0"), str(m.groups()))
    check("[TCP] dev 行: 端口可缺省 (dev off 不带端口)",
          wc.RE_DEV.match("[TCP] dev off").group(2) is None)
    check("[TCP] dev 行: 不误吞杂项 [TCP] 行",
          all(wc.RE_DEV.match(l) is None for l in
              ("[TCP] bridge connected. Type commands, e.g. !A",
               "[TCP] 客户端接入 127.0.0.1:1234 (共 1)")))

    # 遥控门 ([RC]): 控制页「遥控门」徽标的数据源。格式契约 = 严格两单词
    t = wc.Telemetry()
    r = wc.parse_line(t, S_RC[0])
    check("[RC] 锁定行 → rc 帧 locked", r is not None and r[0] == "rc"
          and r[1] == {"locked": True}, str(r))
    r = wc.parse_line(t, S_RC[1])
    check("[RC] 解锁行 → 同一状态键覆盖", r is not None and r[1] == {"locked": False}, str(r))
    check("[RC] 带后缀的行不认 (固件只回两单词)",
          wc.parse_line(t, "[RC] locked (timeout)") is None)

    # 供电状态: 跳变行 (Robot ON/OFF) 与 2s 摘要 ([RUN]/[IDLE]) 写同一个键 ——
    # 点 !O 立即回, 没点按钮时靠摘要兜底
    t = wc.Telemetry()
    for line, want in ((S_RUN[0], True), (S_RUN[1], False),
                       (S_RUN[2], True), (S_RUN[3], False)):
        r = wc.parse_line(t, line)
        check(f"供电状态: {line[:26]}… → on={want}",
              r is not None and r[0] == "run" and r[1].get("on") == want, str(r))
    # [RUN] 行还带行程/步态/抬腿 (控制页的「固件回报」) 与模式尾缀
    # (模式页的实时徽标), 别把 X/Y/Z 解析成别的
    r = wc.parse_line(t, S_RUN[2])
    check("[RUN] 行带出行程/步态/抬腿",
          r is not None and {k: r[1].get(k) for k in ("x", "y", "z", "gait", "lift")}
          == {"x": 60, "y": 0, "z": 0, "gait": 0, "lift": 40}, str(r))
    check("[RUN] 行带出模式尾缀 Bal/St/Hi",
          r is not None and {k: r[1].get(k) for k in ("bal", "st", "hi")}
          == {"bal": 0, "st": 0, "hi": 1}, str(r))
    check("[IDLE] 行只有供电状态 + 模式尾缀 (不去猜行程)",
          wc.parse_line(wc.Telemetry(), S_RUN[3])[1]
          == {"on": False, "bal": 0, "st": 0, "hi": 1}, str(r))
    check("[IDLE] 站姿的 -1 (窄) 能解析成负数",
          wc.parse_line(wc.Telemetry(), S_RUN[4])[1].get("st") == -1, str(r))
    # 快照是合并的: 行程字段进了 run 之后不会被后面的 Robot ON/OFF 行冲掉 ——
    # 前端靠这一点常显"机器现在在怎么走"
    check("行程字段在后续状态行里保留",
          wc.parse_line(t, "Robot ON")[1].get("x") == 60, str(r))

    check("未知行不产生事件 (原样进日志)", wc.parse_line(t, "some random debug line") is None)


# ==================== [2.5] 烧录模块 ====================
#
# 只测纯函数与拒绝路径。真烧录会动硬件 (进 BOOTSEL + 覆写 flash), 不进回归。

def test_flasher():
    print("\n[2.5] 烧录模块 (设备检测 + 路径白名单, 不碰硬件)")
    import tempfile
    from pathlib import Path
    import flasher as F

    check("固件目录指向 pico/build",
          F.FIRMWARE_DIR.name == "build" and F.FIRMWARE_DIR.parent.name == "pico",
          str(F.FIRMWARE_DIR))

    # /sys 下非 RP2040 的 USB 条目必须被滤掉
    devs = F.usb_devices()
    check("设备检测只返回 RP2040 (VID 2e8a)",
          all(d["vid"] == F.RP2040_VID for d in devs), str(devs))
    check("设备条目字段完整",
          all({"bus_id", "mode", "product", "serial"} <= set(d) for d in devs), str(devs))

    # 路径白名单: 全项目唯一一处"用户输入决定读哪个文件"的地方。
    # 真正的不变量是"解析后的真实路径必须落在白名单目录内" —— 带 .. 但绕回
    # 白名单里的写法是合法的, 不该拒; 要拒的是最终落在外面。
    for bad in ["/etc/passwd", "../../etc/passwd", "", "nope.uf2", "hexapod_pico.elf"]:
        try:
            F.resolve_uf2(bad)
            check(f"拒绝越界固件路径 {bad!r}", False, "竟然通过了")
        except ValueError:
            check(f"拒绝越界固件路径 {bad!r}", True)

    outside = Path(tempfile.gettempdir()) / "not_whitelisted.uf2"
    outside.write_bytes(b"\x00")
    try:
        F.resolve_uf2(str(outside))
        check("拒绝白名单目录之外的 .uf2 (绝对路径)", False, "竟然通过了")
    except ValueError:
        check("拒绝白名单目录之外的 .uf2 (绝对路径)", True)

    # 反向: 名字里带 .. 但解析后仍在白名单内 → 放行 (不误杀)
    inside = F.FIRMWARE_DIR / ".." / "build" / "x.uf2"
    inside.write_bytes(b"\x00")
    try:
        check("带 .. 但绕回白名单内的路径放行", F.resolve_uf2(str(inside)).name == "x.uf2")
    finally:
        inside.unlink()

    try:
        F.save_upload(b"x", "evil.exe")
        check("上传拒绝非 .uf2", False)
    except ValueError:
        check("上传拒绝非 .uf2", True)

    p = F.save_upload(b"\x00\x01", "../../../tmp/escaped.uf2")
    check("上传剥掉路径成分 (只取文件名)",
          p.parent == F._UPLOAD_DIR and p.name == "escaped.uf2", str(p))
    check("上传后的文件在白名单内", F.resolve_uf2(p) == p.resolve())
    check("picotool 解析出绝对路径 (缺了则烧录不可用, 不算失败)",
          F.find_picotool() is None or "/" in F.find_picotool(),
          str(F.find_picotool()))


# ==================== [3] 端到端 ====================

# 回放假固件: 收到命令 → 原样吐出固件真实输出 (无硬件也能验证整条链路)。
# 按"命令头 + 空格"前缀匹配, 认的是不带参数的命令 —— 外设那批是
# !MOTOR 1 500 / !PWM 3 5000 这种带参形式, 精确匹配够不着。
# 空格这条限制同时保证了 !PWMOFF 不会撞上 !PWM (与固件自己的字面量分发一致)。
REPLAY = {"!BATT": S_BATT, "!I2C": S_I2C, "!I2C q": S_I2C_Q,
          "!IMU": S_IMU, "!A": S_SERVO, "!PERIPH": S_PER, "!UART0T": S_U0,
          "!HOS": S_HO,
          "!VER": S_VER,
          "!MOTOR": ["[PER] motors: m1=500 m2=0"],
          "!LED": ["[PER] leds: g=0 r=0 hb=0 alarm=1"],
          "!PWM": ["[PER] pwm: 3=5000"],
          "!PWMOFF": ["[PER] pwm: all=0"],
          "!BUZZ": ["[PER] buzzer: freq=1500 ms=300"],
          "!PER0": ["[PER] Board 0 (0x40) period=9500 us (~105 Hz), coxa re-applied"],
          "!PER1": ["[PER] Board 1 (0x41) period=9500 us (~105 Hz), coxa re-applied"]}

# 假固件里会被命令改掉的状态 (真固件里在 control_state 上)
FAKE_ROBOT = {"locked": False, "on": False}


def fake_response(cmd):
    if cmd.startswith("!CFG"):          # !CFG / !CFGR / !CFGW
        return fake_cfg(cmd)
    if cmd.startswith("!HO "):          # 设中心: 回显 "偏移 = 该角度" (0=中位约定)
        sid, _, ang = cmd[4:].partition(" ")
        return [f"[HO] id={int(sid)} off={int(ang)}"]
    if cmd.startswith("!GPO "):         # 空闲脚电平: 输入态拒绝, 输出态回打 gpio 行
        pin, _, lv = cmd[5:].partition(" ")
        if f"gp{pin}_fn" not in FAKE_PARAMS:
            return ["[PORTS] Usage: !GPO <23|24|29> <0|1>"]
        fn = FAKE_PARAMS[f"gp{pin}_fn"][0]
        if fn != 1:                     # 与固件同一道门: 输入脚不给写电平
            return [f"[PORTS] GPO Refuse: gp{pin} fn={fn} (need 1=output)"]
        FAKE_PORTS[f"g{pin}lv"] = int(lv)
        return [gpio_line()]            # 立刻回打 gpio 行, 与固件一致
    if cmd == "!PORTS":                 # 端口状态: uart0 与 gpio 两节反映"当前"
        return [f"[PORTS] uart0: en={FAKE_PARAMS['uart0_en'][0]}"
                f" baud={FAKE_PARAMS['uart0_baud'][0]}",
                "[PORTS] input: mode=0 baud=420000",
                "[PORTS] motors: en=1 lv1=0 lv2=1",
                "[PORTS] foot: en=1 s0=1 s1=0 s2=1 s3=1 s4=1 s5=1",
                "[PORTS] ext: mode=0 a0=0 a1=0",
                gpio_line(),
                "[PORTS] fixed: spwl=1 spwr=1 ledr=0 ledg=1 boot=1"]
    # 两个"有状态"的命令: 应答取决于之前收到过什么, 否则"锁没锁上/供没供电"
    # 在测试里看不出区别 (回放假固件只做回声时最容易漏掉的一类)
    if cmd == "!RC" or cmd.startswith("!RC "):      # 无参 = 锁, 有参 = 按参数
        FAKE_ROBOT["locked"] = (cmd != "!RC 1")
        return ["[RC] locked" if FAKE_ROBOT["locked"] else "[RC] unlocked"]
    if cmd == "!O" or cmd.startswith("!O "):        # 无参 = 切换, 有参 = 置位
        FAKE_ROBOT["on"] = (not FAKE_ROBOT["on"]) if cmd == "!O" else (cmd == "!O 1")
        return ["Robot ON" if FAKE_ROBOT["on"] else "Robot OFF"]
    # 长的命令头先试: 否则 "!I2C q" 会被 "!I2C" 这条前缀吃掉, 回放出全扫描版本
    for key in sorted(REPLAY, key=len, reverse=True):
        if cmd == key or cmd.startswith(key + " "):
            return REPLAY[key]
    return []


def replay_robot(pty):
    """假固件 (仅测试用): 在 pty 上回放假固件输出。

    除应答命令外还每秒无条件推一遍 [CH] 行 —— 固件的通道遥测是自由推送
    (没有对应的查询命令), 只靠"发命令→收应答"测不到那条链路。
    非阻塞读是为了让推送不被"等下一条命令"挡住。"""
    fd = None
    while fd is None:
        try:
            fd = os.open(pty, os.O_RDWR | os.O_NOCTTY)
        except OSError:
            time.sleep(0.2)
    os.set_blocking(fd, False)
    buf = b""
    last_push = 0.0
    last_run_push = 0.0
    while True:
        try:
            data = os.read(fd, 4096)
        except BlockingIOError:
            data = b""
        except OSError:
            time.sleep(0.05)
            continue
        if data:
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                cmd = raw.decode("utf-8", "replace").strip()
                for line in fake_response(cmd):
                    os.write(fd, line.encode() + b"\r\n")
        now = time.time()
        # 20Hz: 只要间隔明显小于 serial_console.py 那个"排空开机日志"循环的
        # 200ms 静默超时, 该循环就会一直有数据可读 —— 于是它是不是有"总时长"
        # 出口就成了关键 (没有出口 → 主循环卡死, 网页读数正常但发命令没反应)。
        # 真机是 5Hz [CH] 叠加 2s 状态行, 密度足以触发; 这里用 20Hz 让它稳定复现,
        # 免得测试变成"看调度运气"。
        if now - last_push >= 0.05:
            last_push = now
            for line in S_CH_CRSF:
                try:
                    os.write(fd, line.encode() + b"\r\n")
                except OSError:
                    pass
        # 状态摘要每秒一遍 (真机 2s, 这里加密让它稳定落进测试窗口): 供电状态
        # 除了 !O 的回声之外还有这条独立来源, 两条路都得能解析出来
        if now - last_run_push >= 1.0:
            last_run_push = now
            line = S_RUN[2] if FAKE_ROBOT["on"] else S_RUN[3]
            try:
                os.write(fd, line.encode() + b"\r\n")
            except OSError:
                pass
        time.sleep(0.02)


def spawn(args, log, cwd):
    f = open(log, "wb")
    p = subprocess.Popen(args, stdout=f, stderr=subprocess.STDOUT, cwd=cwd,
                         stdin=subprocess.DEVNULL)      # DEVNULL → headless 模式
    procs.append(p)
    return p


def cleanup():
    for p in procs:
        try:
            p.terminate()
        except Exception:
            pass
    time.sleep(0.4)
    for p in procs:
        try:
            p.kill()
        except Exception:
            pass
    for path in (PTY_A, PTY_B, PTY_A2, PTY_B2):
        try:
            os.unlink(path)
        except OSError:
            pass


def _raw_request(req, port=HTTP_PORT):
    """发一个 HTTP/1.0 请求, 返回完整响应文本 (含响应头)"""
    s = socket.create_connection(("127.0.0.1", port), timeout=5)
    s.sendall(req.encode())
    data = b""
    while True:
        try:
            c = s.recv(65536)
        except socket.timeout:
            break
        if not c:
            break
        data += c
    s.close()
    return data.decode("utf-8", "replace")


def http_get(path, port=HTTP_PORT):
    """返回响应体 (已剥离响应头)"""
    return _raw_request(f"GET {path} HTTP/1.0\r\nHost: 127.0.0.1\r\n\r\n", port
                        ).split("\r\n\r\n", 1)[-1]


def http_post(path, body, port=HTTP_PORT):
    b = body.encode()
    return _raw_request(f"POST {path} HTTP/1.0\r\nHost: 127.0.0.1\r\n"
                        f"Content-Length: {len(b)}\r\n\r\n" + body, port
                        ).split("\r\n\r\n", 1)[-1]


def sse_open(port=HTTP_PORT, path="/events"):
    """裸套接字打开 SSE —— urllib 的 read() 会阻塞等满缓冲, 不适合长连接流"""
    s = socket.create_connection(("127.0.0.1", port), timeout=2)
    s.sendall(f"GET {path} HTTP/1.0\r\nHost: 127.0.0.1\r\n\r\n".encode())
    return s


def sse_drain(s, seconds, stop_when=None, stamps=None):
    """收 seconds 秒的 SSE 事件。stamps 非空时, 逐个追加到达时刻
    (time.time()), 与 events 一一对应 —— 只有时延类断言需要它。"""
    events, buf = [], ""
    deadline = time.time() + seconds
    while time.time() < deadline:
        try:
            chunk = s.recv(65536)
        except socket.timeout:
            continue
        except OSError:
            break
        if not chunk:
            break
        buf += chunk.decode("utf-8", "replace")
        while "\n\n" in buf:
            frame, buf = buf.split("\n\n", 1)
            for line in frame.split("\n"):
                if line.startswith("data: "):
                    try:
                        events.append(json.loads(line[6:]))
                        if stamps is not None:
                            stamps.append(time.time())
                    except json.JSONDecodeError:
                        pass
        if stop_when and stop_when(events):
            break
    return events


def wait_params(sse, pred, timeout=6):
    """等一个满足 pred(参数快照) 的 SSE params 事件 → (快照 | None, 本批全部事件)"""
    hit = []

    def stop(evs):
        m = [e["d"] for e in evs if e.get("t") == "params" and pred(e["d"])]
        if m:
            hit.append(m[-1])
            return True
        return False

    evs = sse_drain(sse, timeout, stop_when=stop)   # 必须先抽完, hit 才有内容
    return (hit[-1] if hit else None), evs


def val_of(snap, name):
    return (snap or {}).get("list", {}).get(name, {}).get("val")


def test_bridge_latency(sse):
    print("\n[3c] 桥接转发时延 (TCP 线程 0.5s 批处理回归)")

    # 假固件以 20Hz 推 [CH] (间隔 50ms)。TCP 线程的 select 只等"客户端可读 /
    # 新连接", 空闲链路上没有任何事件 —— 曾经它要睡满 0.5s 超时才刷发送队列,
    # 于是 20Hz 的流被压成 500ms 一批, 所有遥测与命令应答被押后 0~500ms
    # (端到端实测 500ms, 而固件侧只有 ~10ms; 网页"反应慢"的根因)。
    # 判据直接用到达间隔: 批处理会留下一个 ~500ms 的空档。
    stamps = []
    evs = sse_drain(sse, 2.5, stamps=stamps)
    ch_t = [t for t, e in zip(stamps, evs) if e.get("t") == "ch"]
    gaps = [b - a for a, b in zip(ch_t, ch_t[1:])]
    mx = max(gaps) if gaps else None
    check("桥接随到随转: 20Hz 遥测最大到达间隔 < 250ms",
          len(ch_t) >= 20 and mx is not None and mx < 0.25,
          f"{len(ch_t)} 帧, 最大间隔 {mx*1000:.0f}ms" if mx else f"只收到 {len(ch_t)} 帧")

    # 命令全链路往返 (浏览器 POST → 桥接 → 假固件 → 回 SSE): 批处理时代中位
    # ≈250ms (相位随机), 修好后十几毫秒
    lat = []
    for _ in range(4):
        t0 = time.time()
        http_post("/cmd", "!VER")
        st2 = []
        evs2 = sse_drain(sse, 0.6, stamps=st2,
                         stop_when=lambda e: any(x.get("t") == "raw"
                                                 and "[VER]" in x.get("line", "") for x in e))
        hit = next((t for t, e in zip(st2, evs2)
                    if e.get("t") == "raw" and "[VER]" in e.get("line", "")), None)
        if hit is not None:
            lat.append(hit - t0)
    med = sorted(lat)[len(lat) // 2] if lat else None
    check("命令往返中位 < 150ms (批处理时代 ≈250ms)",
          len(lat) == 4 and med is not None and med < 0.15,
          f"中位 {med*1000:.0f}ms ({len(lat)}/4 次应答)" if med else "没收到应答")


def test_param_e2e(sse):
    print("\n[3b] /param 端到端 (浏览器 → 服务器 → 串口 → 固件 → SSE)")

    # 开始页点「连接」是调试页的唯一入口: 服务器此时才拉参数表
    # (连接前浏览器连参数都不该看见, 更别说改)
    r = json.loads(http_post("/connect", ""))
    check("POST /connect 连上 (设备在线)", r.get("ok") is True, str(r))
    snap, _ = wait_params(sse, lambda d: d["count"] == len(FAKE_PARAMS))
    check("连接后自动拉取参数表", snap is not None,
          f"count={snap['count'] if snap else None}")
    check("/state 也带参数快照",
          json.loads(http_get("/state"))["telemetry"]["params"]["count"] == len(FAKE_PARAMS))

    # 改值: 服务器应生成 !CFG <name> <value>, 固件回显后前端才改显示
    r = json.loads(http_post("/param", "set travel_fwd_mm 200"))
    check("set 生成 !CFG 指令",
          r.get("ok") is True and r.get("cmd") == "!CFG travel_fwd_mm 200", str(r))
    snap, _ = wait_params(sse, lambda d: val_of(d, "travel_fwd_mm") == 200)
    check("改动经串口回环到 SSE", snap is not None)
    check("改动项标记 chg=1 供前端高亮",
          snap and snap["list"]["travel_fwd_mm"]["chg"] is True)
    check("附带范围/单位随回显一起回传 (前端画滑条要用)",
          snap and (snap["list"]["travel_fwd_mm"]["min"],
                    snap["list"]["travel_fwd_mm"]["max"],
                    snap["list"]["travel_fwd_mm"]["unit"]) == (20, 250, "mm"))

    # 越界: 服务器不夹, 交给固件夹, 但必须把固件提示透到日志
    http_post("/param", "set travel_fwd_mm 999")
    snap, evs = wait_params(sse, lambda d: val_of(d, "travel_fwd_mm") == 250)
    check("越界值由固件夹到 max 并回显", snap is not None)
    raws = [e.get("line", "") for e in evs if e.get("t") == "raw"]
    check("固件的 WARN 走原样日志", any("[P] WARN" in l for l in raws), str(raws[-2:]))
    check("参数行本身不再重复进日志 (已解析)",
          not any("[P] name=" in l for l in raws))

    # 服务器侧的校验: 非法输入根本不该发到串口
    for body, why in [("set Travel 1", "大写名字"), ("set travel_fwd_mm abc", "非整数值"),
                      ("bogus", "未知指令"), ("set", "缺参数名"), ("", "空指令")]:
        r = json.loads(http_post("/param", body))
        check(f"服务器拒绝非法请求: {why}", r.get("ok") is False, str(r))
    check("被拒的请求没有改动固件状态",
          val_of(json.loads(http_get("/state"))["telemetry"]["params"], "travel_fwd_mm") == 250)

    # 按组恢复默认 (网页每页的「本页恢复默认」走这条; 固件没有 !CFGR <组>,
    # 由服务器按参数快照展开成多条单参数命令)
    http_post("/param", "set batt_ov_mv 9000")
    wait_params(sse, lambda d: val_of(d, "batt_ov_mv") == 9000)
    r = json.loads(http_post("/param", "resetgrp batt"))
    check("resetgrp 展开为该组每个参数一条 !CFGR",
          r.get("ok") and r.get("n") == 2, str(r))   # batt_check + batt_ov_mv
    snap, _ = wait_params(sse, lambda d: val_of(d, "batt_ov_mv") == 8800)
    check("该组参数回到默认值", snap is not None)
    check("同组其他参数一起复位且 chg 归零",
          snap and snap["list"]["batt_check"]["chg"] is False)
    check("不碰其他组 (travel_fwd_mm 仍是刚才的越界夹后值)",
          snap and val_of(snap, "travel_fwd_mm") == 250)

    for body, why in [("resetgrp", "缺组名"), ("resetgrp BATT", "大写组名"),
                      ("resetgrp nosuchgrp", "快照里没有的组")]:
        r = json.loads(http_post("/param", body))
        check(f"resetgrp 拒绝: {why}", r.get("ok") is False, str(r))

    # 恢复默认
    r = json.loads(http_post("/param", "reset travel_fwd_mm"))
    check("reset 单项生成 !CFGR", r.get("ok") and r.get("cmd") == "!CFGR travel_fwd_mm", str(r))
    snap, _ = wait_params(sse, lambda d: val_of(d, "travel_fwd_mm") == 150
                          and d["list"]["travel_fwd_mm"]["chg"] is False)
    check("恢复默认后 chg 归零 (前端取消高亮)", snap is not None)

    # 保存到 flash
    http_post("/param", "set batt_ov_mv 9000")
    wait_params(sse, lambda d: val_of(d, "batt_ov_mv") == 9000)
    r = json.loads(http_post("/param", "save"))
    check("save 生成 !CFGW", r.get("ok") and r.get("cmd") == "!CFGW", str(r))
    evs = sse_drain(sse, 3)
    check("保存结果回到日志",
          any("Params saved OK" in e.get("line", "") for e in evs if e.get("t") == "raw"))

    # 全部恢复默认
    r = json.loads(http_post("/param", "resetall"))
    check("resetall 生成裸 !CFGR", r.get("ok") and r.get("cmd") == "!CFGR", str(r))
    snap, _ = wait_params(sse, lambda d: d["changed"] == 0 and d["count"] == len(FAKE_PARAMS))
    check("全部恢复默认后 changed=0", snap is not None)


# ==================== [4] 连接门 (开始页 → 连接 → 调试页) ====================
#
# 第二套实例: 连接门要测的是"轮询有没有在发", 所以这组必须开着轮询,
# 不能和主用例的 --no-poll 挤在一起。间隔调到 0.5s 让判定快而稳定。
# 遥控门 (!RC) 也挂在这组: 它的心跳是同一个轮询线程发的, 间隔同样调快。

GATE_POLL_IV = 0.5


# 页面脚本本身能不能跑到底。顶层脚本任何一处抛异常 —— 语法错、TDZ (const/let 在
# 声明前被读)、取到空元素 —— 后面的语句就全不执行, 而 connect() 恰好是最后一行
# (index.html 末尾): 页面于是永远停在「检测中…」, 浏览器一个请求都不发。
# 上面所有断言都是 grep 静态 HTML, 对"页面整体没跑起来"完全无感 (curParams 的
# TDZ 故障就是这么漏出去的), 所以这里真开一次无头浏览器, 看页面自己的 JS 有没有
# 把状态行改掉。
#
# 页面直接按文件加载 (与 server 现读现发的是同一份字节): /events 一上来就失败,
# 页面不再挂 SSE 长连 —— 否则 --dump-dom 永远等不到加载完成。而连不上时的文案
# 正是 connect() 的 onerror 分支写的, 于是「状态行被改过」= 脚本执行到了最后一行。
CHROME = next((p for p in (shutil.which(n) for n in
                           ("google-chrome", "chromium", "chromium-browser")) if p), None)

# 「脚本跑到底」只证明顶层那一遍没抛异常: 页面里吃 SSE 帧的渲染函数 (renderPorts/
# renderPortsConfig/renderHo…) 在 file:// 下一次也不会被调到, 它们内部抛异常照样
# PASS (端口页那批全是新增函数, 正是最容易漏的一类)。所以再复制一份页面, 在末尾
# 追加一段探针: 直接喂合成帧, 把算出来的东西写进 DOM, dump 出来一比对就知道
# "这些函数真跑过、且结果是对的"。探针自己 try/catch, 抛了就写 THROW 进 DOM。
PORTS_FRAME = {
    "uart0": {"en": 1, "baud": 115200},
    "input": {"mode": 0, "baud": 420000},
    "motors": {"en": 1, "lv1": 0, "lv2": 1},
    "foot": {"en": 1, "s0": 1, "s1": 0, "s2": 1, "s3": 1, "s4": 1, "s5": 1},
    "ext": {"mode": 0, "a0": 7, "a1": 8},
    "gpio": {"g23fn": 1, "g23lv": 1, "g24fn": 0, "g24lv": 0, "g29fn": 0, "g29lv": 1},
    "fixed": {"spwl": 1, "spwr": 1, "ledr": 0, "ledg": 1, "boot": 1},
}


def _port_param(name, val, lo, hi, dfn, unit="", grp="port"):
    """合成一个参数对象 (字段与固件 params_print_one 一一对应)"""
    return {"name": name, "val": val, "min": lo, "max": hi, "def": dfn,
            "unit": unit, "grp": grp, "chg": int(val != dfn)}


# dc_motor_en=0 (帧里 gpio 的 g23fn=1 相反) → 下拉必须回填到「GPIO 输入」;
# gp23_fn=0 → 电平复选框必须是灰的 (输出态才给点); input_baud_serial 取一个
# 不在下拉候选里的值, 顺带验证"选项外临时插一条"的兜底。
PROBE_PARAMS = {"list": {
    "dc_motor_en": _port_param("dc_motor_en", 0, 0, 1, 1),
    "gp23_fn": _port_param("gp23_fn", 0, 0, 1, 0),
    "input_baud_serial": _port_param("input_baud_serial", 74880, 2400, 1000000,
                                     115200, "baud"),
    "uart0_en": _port_param("uart0_en", 1, 0, 1, 1, "", "per"),
}}

# 舵机帧与偏移帧 (校准页的三段接线: !A 建格子 → !HOS 填偏移 → 选中一路看面板)。
# ⚠️ !A 报的是 g_servo_batch.angles[i] —— 与 !P 同一套角度单位 (0=中位 1500µs,
# ±900=±90°), 不是脉宽。校准面板的滑条就是这套单位, 两边必须一致: 差一个 1500
# 的偏置会让滑条被自己的 min/max 夹住 (看着像"角度恒为 ±900", 不会报任何错)。
# 喂第 3 路 -120 是想同时钉住"面板显示的是这一路的角度"而不是碰巧的默认值。
PROBE_SERVO = {"boards": {"0": {"angles": {str(i): i * 20 - 180 for i in range(18)}}}}
PROBE_HO = {"offs": {str(i): v for i, v in
                     enumerate([-10, 55, 8, 12, 85, 68, 0, 40, 30,
                                -25, 60, 70, 15, 35, 45, 5, 20, 25])}}

PROBE_JS = """
<div id="probe"></div>
<script>
try {
  renderPorts(""" + json.dumps(PORTS_FRAME) + """);
  renderParams(""" + json.dumps(PROBE_PARAMS) + """);
  renderServo(""" + json.dumps(PROBE_SERVO) + """);
  renderHo(""" + json.dumps(PROBE_HO) + """);
  selectCalServo(3);
  var out = {
    rows: document.querySelectorAll("#pt-rows .ptrow").length,
    uart0: $("pts-uart0").textContent,
    dcm: $("pt-sel-motors").value,          /* 行 key 是 motors, 参数名才是 dc_motor_en */
    g23dis: $("pt-cfgl-gp23").disabled,     /* gp23_fn=0 (输入) → 复选框必须是灰的 */
    g23: $("pts-gp23").textContent,
    g24: $("pts-gp24").textContent,
    foot: $("pts-foot").textContent,
    inb: $("pt-cfgs-input").value,
    fixed1: $("ptf-1").textContent,
    fixed2: $("ptf-2").textContent,
    calCells: document.querySelectorAll("#cal-servos .servo").length,
    calOff3: document.querySelector("#calsv3 .off").textContent,
    calSel: $("cal-sel").textContent,
    calOffBox: $("cal-off").textContent,
    calAng: $("cal-ang").value,
    calHidden: $("cal-edit").hidden,
  };
  /* 第二拍: 把 gp23_fn 换成「输出」再喂一帧低电平 —— 复选框要跟着引脚的实测
   * 电平走 (勾选状态是"这一脚现在什么电平", 不是"这次会话里我点过什么") */
  renderParams({list: {"gp23_fn": """ + json.dumps(_port_param("gp23_fn", 1, 0, 1, 0)) + """}});
  renderPorts({gpio: {"g23fn": 1, "g23lv": 0, "g24fn": 0, "g24lv": 0,
                      "g29fn": 0, "g29lv": 1}});
  out.g23dis2 = $("pt-cfgl-gp23").disabled;
  out.g23chk2 = $("pt-cfgl-gp23").checked;
  /* 第三拍: 把面板真的按下去, 捕获 sendCmd 拿到的**原样命令串**。静态 check 只能
   * 证明页面里写着某个模板 —— 模板自身抄错也照样 PASS (这正是 "!P 0 300" 那次的
   * 教训: 固件从 buf[2] 读 id, 多一个空格把角度读成 id 首位, 静默转到别处)。 */
  var CMDS = [];
  sendCmd = function (cmd) { CMDS.push(cmd); };
  $("cal-ang").value = 300; $("cal-ang").onchange();   /* 拖动滑条松手 */
  $("cal-p20").onclick();                              /* 微调 +20 */
  $("cal-center").onclick();                           /* 设为中心 */
  out.cmds = CMDS;
  $("probe").textContent = JSON.stringify(out);
} catch (e) { $("probe").textContent = "THROW " + e; }
</script>
"""


def test_page_js():
    if not CHROME:
        print("  SKIP  没有无头浏览器, 跳过页面执行检查")
        return
    prof = "/tmp/wc_chrome_profile"
    shutil.rmtree(prof, ignore_errors=True)
    src = open(INDEX_HTML, encoding="utf-8").read()
    probe_path = "/tmp/wc_page_probe.html"
    with open(probe_path, "w", encoding="utf-8") as f:
        f.write(src.replace("</body>", PROBE_JS + "</body>"))
    page = "file://" + probe_path + "?mode=server"   # 钉住服务端态: 探针页面是 file:// 加载的
    with open("/tmp/wc_chrome.log", "wb") as errlog:
        try:
            r = subprocess.run(
                [CHROME, "--headless=new", "--no-sandbox", "--disable-gpu",
                 f"--user-data-dir={prof}", "--virtual-time-budget=5000",
                 "--dump-dom", page],
                stdout=subprocess.PIPE, stderr=errlog,
                stdin=subprocess.DEVNULL, timeout=90)
        except subprocess.TimeoutExpired:
            check("页面脚本执行到底 (状态行不再停在「检测中…」)", False,
                  "浏览器 90s 没吐出 DOM, 见 /tmp/wc_chrome.log")
            return
    dom = r.stdout.decode("utf-8", "replace")
    m = re.search(r'id="wel-web"[^>]*>([^<]*)<', dom)
    got = m.group(1) if m else None
    check("页面脚本执行到底 (状态行不再停在「检测中…」)", got == "断开，重连中…",
          f"wel-web = {got!r}; 浏览器日志 /tmp/wc_chrome.log")

    # 加在页面末尾的探针 HTML 就是上面写进去的那段: </body> 之前
    if "id=\"probe\"" not in dom:
        check("探针脚本挂进了页面 (否则下面的断言无从谈起)", False, "没找到 #probe")
        return
    m = re.search(r'id="probe">([^<]*)<', dom)
    got = html.unescape(m.group(1)) if m else ""
    if got.startswith("THROW"):
        check("端口页渲染函数能跑通 (喂合成帧不抛异常)", False, got)
        return
    try:
        pr = json.loads(got)
    except json.JSONDecodeError:
        check("端口页渲染函数能跑通 (喂合成帧不抛异常)", False, repr(got))
        return
    check("端口页渲染函数能跑通 (喂合成帧不抛异常)", True)
    check("端口页建出 8 行引脚 (uart0/输入/电机/足端/外部 + 三路空闲)",
          pr.get("rows") == 8, str(pr.get("rows")))
    check("端口页状态列按帧渲染 (UART0 与六路足端的实测值)",
          pr.get("uart0") == "使能 · 115200 波特"
          and pr.get("foot") == "检测 · 高 低 高 高 高 高 · 触地 1/6",
          str({k: pr.get(k) for k in ("uart0", "foot")}))
    check("空闲脚状态列报「归属 · 电平」(输入/输出各一行)",
          pr.get("g23") == "输出 · 高" and pr.get("g24") == "输入 · 低",
          str((pr.get("g23"), pr.get("g24"))))
    check("固定脚两行的实测电平也回填 (来自 fixed 节)",
          pr.get("fixed1") == "GP10 高 · GP11 高"
          and pr.get("fixed2") == "LED 红 低 · 绿 高 · IMU BOOT 高",
          str((pr.get("fixed1"), pr.get("fixed2"))))
    check("功能下拉按参数值回填, 且输入脚的电平复选框是灰的",
          pr.get("dcm") == "0" and pr.get("g23dis") is True,
          str((pr.get("dcm"), pr.get("g23dis"))))
    check("切成输出后复选框可点, 且勾选状态跟着引脚的实测电平",
          pr.get("g23dis2") is False and pr.get("g23chk2") is False,
          str((pr.get("g23dis2"), pr.get("g23chk2"))))
    check("下拉外的波特率值临时插一条, 不显示成空选",
          pr.get("inb") == "74880", str(pr.get("inb")))

    # 校准页的逐舵机面板 (用户诉求的那个): 同一个探针里顺手喂 !A 与 !HOS 两帧,
    # 再点选一路 —— 选中格高亮、格子里的偏移、面板上的偏移/角度三者要同时对上
    check("校准页建出 18 格, 且 !HOS 帧把偏移写进对应格",
          pr.get("calCells") == 18 and pr.get("calOff3") == "偏移 12",
          str((pr.get("calCells"), pr.get("calOff3"))))
    check("选中一路后面板报「哪一路 / 当前偏移 / 当前角度」",
          pr.get("calSel") == "#3 · R · coxa" and pr.get("calOffBox") == "12"
          and pr.get("calAng") == "-120" and pr.get("calHidden") is False,
          str({k: pr.get(k) for k in ("calSel", "calOffBox", "calAng", "calHidden")}))
    # 页面 → 固件的唯一接缝就是这几个命令串 (探针捕获的真实字符串, 不是模板)。
    # !P 的 id 紧跟命令字; !HO 相反要空格 —— 两种形状在这里并排钉住。
    check("滑条/微调/设为中心发出去的命令串 (id 位置与空格)",
          pr.get("cmds") == ["!P3 300", "!P3 320", "!HO 3 320"],
          str(pr.get("cmds")))


# ==================== [6] 直连引擎 (Web Serial, 页面内解析) ====================
#
# 直连模式下固件输出由页面自己解析, 于是解析器有两份实现 (server.py 与
# index.html)。单侧漂移是静的: 同一条固件输出在两种打开方式下显示成两样,
# 两边都不报错。这里拿同一条时间线喂两边做差分 —— 帧类型/字段/顺序/raw
# 白名单必须逐帧一致 —— 再补几条定向断言, 把差分里不好读的语义单独钉住。

# 同一块板再报一遍: 板级覆盖 (新角度是整份, 旧角度不许残留)
S_SERVO_AGAIN = [
    "Board 0x41 (right): [0]:1600 [1]:1580",
]

# 时间线覆盖三种累积策略 (ch 整帧替换 / per+ports 分节合并 / ho+params 逐条
# 累积) 与全部帧类型, 顺序照真实开机流排; 末尾一条固件未来新增的怪行 (谁都不认
# → 原样日志), 是"解析不了也不能崩"的那条底线。
DIRECT_LINES = (S_VER + S_BATT + S_BATT_ABSENT + S_I2C + S_I2C_Q + S_IMU
                + S_SERVO + S_SERVO_AGAIN + S_CH_CRSF + S_CH_PS2 + S_CH_CRSF
                + S_RUN + S_PER + S_PORTS + S_HO + S_U0 + S_PARAMS + S_RC
                + ["[IMU] 读连续失败 20 次 (约 210ms, 累计 57 次) → 总线恢复",
                   "[FOO] 固件以后新加的行"])

# 解析出来的行不再当 raw 重发 (server.py on_line 与页面 RAW_SKIP 各一份)
RAW_SKIP = ["params", "per", "ch", "imu", "rc", "run", "ho", "ports"]

# IMU 状态块整块折叠 (server.py 的 IMU_BLOCK_* 与页面同名常量各一份, 必须同值)
IMU_BLOCK_HEAD = "=== IMU Status ==="
IMU_BLOCK_TAIL = "==================="
IMU_BLOCK_MAX = 16


def _first_diff(a, b):
    """差分报告: 第一处不一致的帧 (整表 diff 打出来没人看得懂)"""
    for i in range(max(len(a), len(b))):
        x = a[i] if i < len(a) else None
        y = b[i] if i < len(b) else None
        if x != y:
            d = json.dumps(x, ensure_ascii=False, sort_keys=True)
            e = json.dumps(y, ensure_ascii=False, sort_keys=True)
            return f"第 {i} 帧 页面={d[:160]} server={e[:160]}"
    return ""


def _python_frames(lines):
    """server.py 的解析路径 (块折叠 + parse_line + on_line 的 raw 白名单)。
    每帧当场冻结成 JSON 树: 广播出去的就是那一刻的字节, 之后同引用的快照
    (params/servo/u0) 再被改写, 也不该回头改已经发出去的那一帧。"""
    t = wc.Telemetry()
    out = []
    imu_block = 0
    for line in lines:
        imu_block, folded = wc.imu_block_fold(imu_block, line)   # 真的那份, 不抄
        parsed = wc.parse_line(t, line)
        if not folded and not (parsed and parsed[0] in RAW_SKIP):
            out.append(json.loads(json.dumps({"t": "raw", "line": line})))
        if parsed:
            out.append(json.loads(json.dumps({"t": parsed[0], "d": parsed[1]})))
    return out


PROBE_DIRECT_JS = """
<div id="probe"></div>
<script>
try {
  var LINES = """ + json.dumps(DIRECT_LINES, ensure_ascii=False) + """;
  var S_IMU_LINES = """ + json.dumps(S_IMU, ensure_ascii=False) + """;
  var S_VER_LINES = """ + json.dumps(S_VER, ensure_ascii=False) + """;
  var S_BATT_LINES = """ + json.dumps(S_BATT, ensure_ascii=False) + """;
  var out = {renderErrs: [], frames: []};
  var realFrame = handleFrame;   /* 第二拍会换成记录器, 先留一份真的 */
  /* 第一拍: 每一步都过**真的** handleFrame —— 渲染函数 (renderBatt/renderI2c/…)
     抛异常在前端是"页面卡住", 什么回执都没有, 只能在这里拦 */
  Direct.telem = Direct.newTelem();
  for (var i = 0; i < LINES.length; i++) {
    try { Direct.handleLine(LINES[i]); }
    catch (e) { out.renderErrs.push(String(e) + " @" + i + " " + LINES[i].slice(0, 40)); }
  }
  /* 第二拍: 换成记录用的 handleFrame 收帧。帧当场深拷: 快照里嵌的子对象是活
     引用 (与 Python 侧 dict() 浅拷同理), 不冻结就会看到"最后状态"而不是"这一帧" */
  handleFrame = function (ev) { out.frames.push(JSON.parse(JSON.stringify(ev))); };
  Direct.telem = Direct.newTelem();
  for (i = 0; i < LINES.length; i++) Direct.handleLine(LINES[i]);

  /* ---- 定向抽查 (各自独立的小时间线) ---- */
  var last = function (t, line) { return Direct.parseLine(t, line); };

  /* i2c 头行清场: 全扫头行连扫描列表一起清, quick 头行只清设备 */
  Direct.telem = Direct.newTelem();
  last(Direct.telem, "=== I2C Bus Check (SDA=GP14, SCL=GP15) ===");
  last(Direct.telem, "  Device found at 0x40");
  var before = Direct.telem.i2c.scan;
  var q = last(Direct.telem,
               "=== I2C Bus Check (SDA=GP14, SCL=GP15) quick ===")[1];
  out.i2c = {before: before, reset: q.reset, reset_scan: q.reset_scan,
             after: Direct.telem.i2c.scan,
             devWiped: !("pca" in Direct.telem.i2c)};

  /* ch 整帧替换: 切到 ps2 后 crsf 的字段一个都不许留, 切回来缺项回填中位 */
  Direct.telem = Direct.newTelem();
  last(Direct.telem, "[CH] m=crsf link=1 fc=100 c0=200 c3=1000");
  var ps2 = last(Direct.telem,
                 "[CH] m=ps2 con=1 btns=1 lx=2 ly=3 rx=4 ry=5 fc=6")[1];
  var crsf = last(Direct.telem, "[CH] m=crsf link=0 fc=9 c0=250")[1];
  out.ch = {ps2Keys: Object.keys(ps2).sort(), crsfKeys: Object.keys(crsf).sort(),
            crsfCh: crsf.ch};

  /* 参数帧: 以固件回显为准重算 count/changed; 帧是快照不是活对象;
     unit= 空 token 必须落成 "" 而不是 undefined (undefined 会在 JSON 里整键消失) */
  Direct.telem = Direct.newTelem();
  var p1 = last(Direct.telem,
                "[P] name=aaa val=1 min=0 max=2 def=1 unit= grp=g chg=0")[1];
  var p2 = last(Direct.telem,
                "[P] name=bbb val=2 min=0 max=3 def=2 unit=mV grp=g chg=1")[1];
  out.params = {first: [p1.count, p1.changed, Object.keys(p1.list)],
                then: [p2.count, p2.changed, Object.keys(p2.list)],
                unit: [p2.list.aaa.unit, p2.list.bbb.unit],
                /* 提示行解析不出来 —— 不能变成帧, 只能进日志 */
                warn: last(Direct.telem, "[P] WARN aaa clamped 999 -> 2 (范围 0~2)"),
                sep: last(Direct.telem, "[P] === 2 params (1 changed) ===")};

  /* per/ports 分节合并: 一行一节, 分节存才不至于只收到一半就渲染半张表 */
  Direct.telem = Direct.newTelem();
  var e1 = last(Direct.telem, "[PER] motors: m1=500 m2=0")[1];
  var e2 = last(Direct.telem, "[PER] uart0: en=1 baud=115200 rx=42 tx=7 lines=3")[1];
  /* [U0] 环形缓冲: 只留最近 U0_RX_RING 行 */
  for (var k = 0; k < 22; k++) last(Direct.telem, "[U0] line" + k);
  var e3 = last(Direct.telem, "[U0] 兜底")[1];
  var s1 = last(Direct.telem, "[PORTS] uart0: en=1 baud=115200")[1];
  var s2 = last(Direct.telem, "[PORTS] foot: en=1 s0=1 s1=0")[1];
  out.per = {one: Object.keys(e1), two: Object.keys(e2).sort(),
             ringLen: e3.u0_rx.length, ringHead: e3.u0_rx[0],
             ringTail: e3.u0_rx[e3.u0_rx.length - 1]};
  out.ports = {one: Object.keys(s1), two: Object.keys(s2).sort(), foot: s2.foot};

  /* 舵机: 板级整份覆盖 (同一块板再报时旧角度是删掉, 不是留着)。
     帧里的 boards 是活引用 (server.py 侧 dict() 浅拷同理), 所以板名当场取 */
  Direct.telem = Direct.newTelem();
  var b1 = last(Direct.telem, "Board 0x41 (right): [0]:1500 [1]:1480 [2]:1520 ")[1];
  var firstBoards = Object.keys(b1.boards);
  var b1Angles = Object.keys(b1.boards["41"].angles);   /* 键是 0x40/0x41 的十六进制对 */
  last(Direct.telem, "Board 0x40 (left): [9]:1500 ");
  var b3 = last(Direct.telem, "Board 0x41 (right): [0]:1600")[1];
  out.servo = {first: firstBoards, firstAngles: b1Angles,
               boards: Object.keys(b3.boards).sort(),
               b41: b3.boards["41"], b40: b3.boards["40"]};

  /* 版本行/曲线: 走真的 handleFrame (第二拍换成了记录器, 这里换回来) */
  handleFrame = realFrame;
  var feed = function (lines) {
    for (var i = 0; i < lines.length; i++) Direct.handleLine(lines[i]);
  };
  var hlen = function () {
    return ["att", "acc", "gyr", "mag", "tmp"]
      .map(function (k) { return IMU_HIST[k].length; }).join(",");
  };
  /* 把每条序列最后一个点往过去推 ms 毫秒 —— 探针里连喂几遍是同一瞬间,
     真实轮询的间隔得自己造出来 (闸门放不放行看的就是这个差) */
  var backdate = function (ms) {
    for (var k in IMU_HIST) {
      var h = IMU_HIST[k];
      if (h.length) h[h.length - 1].t -= ms;
    }
  };
  for (var hk in IMU_HIST) IMU_HIST[hk].length = 0;
  feed(S_IMU_LINES);
  out.imuLen1 = hlen();
  out.accPt = JSON.stringify(IMU_HIST.acc[0]);
  feed(S_IMU_LINES);   /* 同一周期内的重复快照 (紧接着到): 闸门没过, 一个点都不加 */
  out.imuLen2 = hlen();
  backdate(1000);      /* 一秒后的下一次轮询: 值没变, 但曲线得照长 */
  feed(S_IMU_LINES);
  out.imuLen3 = hlen();
  /* 窗口: 把最老的点推到一分钟以外 —— 再推一个点就得把它挤出去, 条数不涨 */
  for (var wk in IMU_HIST) if (IMU_HIST[wk].length > 1) IMU_HIST[wk][0].t -= 61000;
  backdate(1000);
  feed(S_IMU_LINES);
  out.imuLen4 = hlen();
  out.imuOldestAge = Math.round(Date.now() - IMU_HIST.acc[0].t);   /* 老点已被挤掉 */
  feed(S_VER_LINES);
  out.verBar = [$("fwver").textContent, $("hwver").textContent];
  out.welVer = [$("wel-ver").textContent, $("wel-hw").textContent];
  feed(["available: NO  addr: 0x29"]);   /* 掉线: 五条曲线全清 */
  out.imuLenClear = hlen();

  /* 电池曲线同一套历史 (同一个 histPush), 也照这几条来一遍 */
  hist.length = 0;
  feed(S_BATT_LINES);
  out.battLen1 = hist.length;
  feed(S_BATT_LINES);                    /* 重复快照: 闸门挡住 */
  out.battLen2 = hist.length;
  hist[hist.length - 1].t -= 2000;       /* 电池轮询 2s */
  feed(S_BATT_LINES);
  out.battLen3 = hist.length;
  out.battPt = JSON.stringify(hist[hist.length - 1]);

  /* 直连态的开始页: 桥接行/固件卡/桥接日志区必须**算出来**不可见。只看 hidden
     属性会被样式表骗 —— .row{display:flex} 盖掉浏览器默认的 [hidden]{display:none},
     于是 el.hidden = true 看着设了却没藏住 (直连态还显示"串口桥接"就是它)。
     版本行反着来: 从固件卡挪进通用状态区, 直连态必须可见 */
  MODE = "direct"; applyModeUI(); renderWelcome();
  out.hideDirect = ["wel-row-bridge", "wel-fw-card", "wel-log-sect"]
    .map(function (id) { return getComputedStyle($(id)).display; });
  out.showVer = ["wel-row-ver", "wel-row-hw"]
    .map(function (id) { return getComputedStyle($(id)).display; });
  MODE = "server"; applyModeUI(); renderWelcome();
  out.hideServer = getComputedStyle($("wel-row-bridge")).display;   /* 正对照 */

  /* 真画一遍 (welcome 挡着时 canvas clientWidth=0, drawMultiChart 会早退) */
  enterDebugUI(); drawAllCharts();
  out.drawW = [$("chart").clientWidth, $("ch-att").clientWidth];
  out.drew = true;
  $("probe").textContent = JSON.stringify(out);
} catch (e) { $("probe").textContent = "THROW " + e; }
</script>
"""


def _probe_json(js, tag, mode=None):
    """把探针追加到页面末尾, 无头浏览器跑一遍, 返回 (解析好的 JSON, 失败原因)

    mode= 会钉在 URL 上 (?mode=): 探针页面是 file:// 加载的, 不钉的话页面自己
    探出来的态要看这台机器给不给 Web Serial —— 除"探探测本身"的用例, 其余都该钉。"""
    if not CHROME:
        return None, "没有无头浏览器"
    prof = f"/tmp/wc_chrome_profile_{tag}"
    shutil.rmtree(prof, ignore_errors=True)
    src = open(INDEX_HTML, encoding="utf-8").read()
    probe_path = f"/tmp/wc_page_probe_{tag}.html"
    with open(probe_path, "w", encoding="utf-8") as f:
        f.write(src.replace("</body>", js + "</body>"))
    page = "file://" + probe_path + (f"?mode={mode}" if mode else "")
    with open(f"/tmp/wc_chrome_{tag}.log", "wb") as errlog:
        try:
            r = subprocess.run(
                [CHROME, "--headless=new", "--no-sandbox", "--disable-gpu",
                 f"--user-data-dir={prof}", "--virtual-time-budget=5000",
                 "--dump-dom", page],
                stdout=subprocess.PIPE, stderr=errlog,
                stdin=subprocess.DEVNULL, timeout=90)
        except subprocess.TimeoutExpired:
            return None, f"浏览器 90s 没吐出 DOM, 见 /tmp/wc_chrome_{tag}.log"
    dom = r.stdout.decode("utf-8", "replace")
    m = re.search(r'id="probe">([^<]*)<', dom)
    if not m:
        return None, "页面里没有 #probe (探针没挂上?)"
    got = html.unescape(m.group(1))
    if got.startswith("THROW"):
        return None, got
    try:
        return json.loads(got), ""
    except json.JSONDecodeError:
        return None, repr(got[:200])


def test_direct_engine():
    """页面内的直连解析器 vs server.py: 同一条时间线逐帧差分"""
    pr, why = _probe_json(PROBE_DIRECT_JS, "direct", mode="direct")
    if pr is None:
        check("直连引擎探针能跑通", False, why)
        return
    check("直连引擎探针能跑通", True)

    # 渲染层: 帧喂给真的 handleFrame, 每一帧都得能过 (抛了就记在 renderErrs)
    check("每一帧都能过渲染层 (19 种帧喂真的 handleFrame 不抛)",
          pr["renderErrs"] == [], str(pr["renderErrs"][:2]))

    py_frames = _python_frames(DIRECT_LINES)
    check(f"直连解析与 server.py 逐帧一致 ({len(py_frames)} 帧: 类型/字段/顺序)",
          pr["frames"] == py_frames, _first_diff(pr["frames"], py_frames))
    # 帧类型要真的覆盖到, 否则"两边一致"可能只是两边都没解析出东西
    kinds = sorted({f["t"] for f in py_frames})
    check("时间线覆盖到 14 类帧 (含 raw 日志路)",
          kinds == ["batt", "ch", "ho", "hw", "i2c", "imu", "params", "per",
                    "ports", "raw", "rc", "run", "servo", "ver"], str(kinds))
    # 原始传感器字段真的解析出来了 (两边都漏 = 差分一致但全空, 所以单独钉)
    check("时间线里的 imu 帧带 acc/gyr/mag/temp (不是空壳)",
          any(f["t"] == "imu" and f["d"].get("acc") == {"x": 12, "y": 5, "z": -1002}
              and f["d"].get("gyr", {}).get("z") == -45
              and f["d"].get("mag", {}).get("x") == -32
              and f["d"].get("temp") == 31
              for f in py_frames))
    check("时间线里的 hw 帧带硬件版本",
          any(f["t"] == "hw" and f["d"].get("text") == "PCB v2" for f in py_frames))
    # raw 白名单: 白名单里的行不再进日志面板 (一次 !CFG 几十行会把日志冲掉),
    # 认不出来的行必须原样进日志
    raws = [f["line"] for f in py_frames if f["t"] == "raw"]
    check("raw 白名单: 高频帧的行不进日志 ([P]/[PER]/[PORTS]/[HO]/[CH]/[RC]/[RUN])",
          not any(r.startswith(("[P] name=", "[PER]", "[PORTS]", "[HO] id=",
                                "[CH]", "[RC]", "[RUN]", "[IDLE]", "[U0]",
                                "[U0TX]", "Robot "))
                  for r in raws), str(raws[:3]))
    # IMU 输出在日志面板里的另一半: 状态块 (=== IMU Status === … =============)
    # 整块折叠 —— 表头到页脚之间是没有解析器的杂项行 (表头/INT_STA/INT cfg/
    # SW rev/页脚), 5Hz 下合计 25 行/秒, 600 行面板二十几秒冲干净。折叠只压噪
    # 不藏故障, 所以块内的 fail: 行与块外的 [IMU] 警告行各有专门一条钉着。
    check("raw: IMU 数据行不进日志 (5Hz 推送; 块折叠之外的兜底白名单)",
          not any(r.startswith(("calib:", "accel:", "gyro:", "mag:", "temp:",
                                "last:", "available:")) for r in raws),
          str([r for r in raws if r.startswith(("calib:", "accel:", "last:"))][:3]))
    check("raw: IMU 状态块整块折叠 (表头/INT_STA/INT cfg/SW rev/页脚都不进日志)",
          not any(r.startswith(("=== IMU Status ===", "INT_STA=", "INT cfg readback:",
                                "SW rev:", "===================")) for r in raws),
          str([r for r in raws if r.startswith(("INT_STA", "SW rev", "==="))][:3]))
    check("raw: 块内的 fail: 行放行 (折叠不藏故障)",
          any(r.startswith("fail: 连续 3 次") for r in raws),
          str([r for r in raws if r.startswith("fail:")][:2]))
    check("raw: 固件看门狗的 [IMU] 警告行照常进日志 (在块外, 认不出来 = 进日志)",
          any(r.startswith("[IMU] 读连续失败") for r in raws),
          str([r for r in raws if r.startswith("[IMU]")][:3]))
    check("raw 白名单: 认不出来的行原样进日志",
          "=== Battery ADC (GP28/ADC2, divider 47/377) ===" in raws
          and "[P] FAIL unknown param 'nosuch' (试 !CFG 列出全部)" in raws
          and "[FOO] 固件以后新加的行" in raws)
    # ver 不在白名单里是有意的: 版本行既是 ver 帧, 也留在日志里 ——
    # 开始页的启动横幅就是从日志区读的 (没连接时唯一的版本来源)
    check("raw 白名单: ver 例外 —— 版本行同时留在日志里",
          "[VER] Hexapod v0.3.0-16-g8f5b04f-dirty" in raws)
    check("raw 白名单与 server.py 同表 (顺序即契约)",
          [f["line"] for f in pr["frames"] if f["t"] == "raw"] == raws)

    check("i2c: 全扫头行之后的扫描列表, quick 头行保留 (reset_scan=False)",
          pr["i2c"] == {"before": [0x40], "reset": True, "reset_scan": False,
                        "after": [0x40], "devWiped": True}, str(pr["i2c"]))
    check("ch: 切到 ps2 后 crsf 字段不残留 (整帧替换)",
          pr["ch"]["ps2Keys"] == ["btns", "connected", "fc", "lx", "ly", "mode",
                                  "rx", "ry"], str(pr["ch"]["ps2Keys"]))
    check("ch: 切回 crsf 时缺项回填中位 992, 16 通道一个不少",
          pr["ch"]["crsfKeys"] == ["ch", "fc", "link", "mode"]
          and pr["ch"]["crsfCh"] == [250] + [992] * 15, str(pr["ch"]))
    check("params: count/changed 以固件回显为准重算",
          pr["params"]["first"] == [1, 0, ["aaa"]]
          and pr["params"]["then"] == [2, 1, ["aaa", "bbb"]], str(pr["params"]))
    check("params: 帧是快照 (后一帧不回头改前一帧), unit= 空 token 落成空串",
          pr["params"]["unit"] == ["", "mV"], str(pr["params"]["unit"]))
    check("params: WARN/=== 提示行解析不出来 (走日志, 不产帧)",
          pr["params"]["warn"] is None and pr["params"]["sep"] is None)
    check("per/ports: 分节合并 (后到的节不冲掉先到的)",
          pr["per"]["one"] == ["motors"]
          and pr["per"]["two"] == ["motors", "uart0"]
          and pr["ports"]["one"] == ["uart0"]
          and pr["ports"]["two"] == ["foot", "uart0"]
          and pr["ports"]["foot"] == {"en": 1, "s0": 1, "s1": 0}, str(pr["per"]))
    check("[U0] 环形缓冲只留最近 20 行",
          pr["per"]["ringLen"] == 20 and pr["per"]["ringHead"] == "line3"
          and pr["per"]["ringTail"] == "兜底", str(pr["per"]))
    check("舵机: 板级整份覆盖 (同一块板重报后旧角度不残留)",
          pr["servo"]["first"] == ["41"]
          and pr["servo"]["firstAngles"] == ["0", "1", "2"]
          and pr["servo"]["boards"] == ["40", "41"]
          and pr["servo"]["b41"] == {"side": "right", "angles": {"0": 1600}}
          and pr["servo"]["b40"] == {"side": "left", "angles": {"9": 1500}},
          str(pr["servo"]))
    # 隐藏态要按**算出来的 display** 判, 不能只看 hidden 属性 —— 属性设了但被
    # 样式表的 display 盖掉, 在用户眼里就是"直连态还显示串口桥接"
    check("直连态真的藏住了桥接行/固件卡/桥接日志 (算 display, 不看属性)",
          pr["hideDirect"] == ["none", "none", "none"], str(pr["hideDirect"]))
    check("服务态桥接行照旧可见 (正对照: 免得上面那条靠「全藏了」蒙过)",
          pr["hideServer"] != "none", str(pr["hideServer"]))
    # 版本行与固件卡相反: 直连态必须可见 (挪出卡外就是为了这个)
    check("直连态版本行可见 (与隐藏的固件卡对照)",
          pr["showVer"] == ["flex", "flex"], str(pr["showVer"]))

    # 曲线: 一帧一个点; 同一周期内的重复快照不推点 (时间闸门), 但静止时值不变
    # 也要照长 —— 这两条合起来才是用户要的"曲线在动, 且不是假的在动"
    check("曲线: 一帧喂出五条序列各一个点",
          pr["imuLen1"] == "1,1,1,1,1", pr["imuLen1"])
    check("曲线: 同一周期的重复快照一个点都不推 (闸门 250ms)",
          pr["imuLen2"] == pr["imuLen1"], pr["imuLen2"])
    check("曲线: 一秒后的下一次轮询值没变也照样长 (不是按值去重)",
          pr["imuLen3"] == "2,2,2,2,2", pr["imuLen3"])
    acc = json.loads(pr["accPt"])
    check("曲线: 点里是解析后的原始值 (含负轴)",
          (acc["x"], acc["y"], acc["z"]) == (12, 5, -1002), pr["accPt"])
    # 窗口: 最老的点被挤出窗口后条数不涨 (进来一个、出去一个), 留下的那个是新的
    check("曲线: 只留最近 1 分钟 (老点挤出窗口, 条数不涨)",
          pr["imuLen4"] == "2,2,2,2,2" and pr["imuOldestAge"] < 5000,
          f'{pr["imuLen4"]} 最老的点 {pr["imuOldestAge"]}ms 前')
    check("曲线: 掉线 (available: NO) 清空五条序列",
          pr["imuLenClear"] == "0,0,0,0,0", pr["imuLenClear"])
    # 电池曲线共用同一套 (同一个 histPush): 闸门/时长都照上面的规矩
    check("电池曲线: 重复快照不推点, 2s 后的下一轮推一个",
          pr["battLen1"] == 1 and pr["battLen2"] == 1 and pr["battLen3"] == 2,
          f'{pr["battLen1"]}/{pr["battLen2"]}/{pr["battLen3"]}')
    batt = json.loads(pr["battPt"])
    check("电池曲线: 点里是解析后的 mV",
          batt["mv"] == 8370 and batt.get("t", 0) > 0, pr["battPt"])
    check("版本帧进状态栏 (固件 … / 硬件 …, 原文保留)",
          pr["verBar"] == ["固件 Hexapod v0.3.0-16-g8f5b04f-dirty", "硬件 PCB v2"],
          str(pr["verBar"]))
    check("版本帧同时写开始页两行",
          pr["welVer"] == ["Hexapod v0.3.0-16-g8f5b04f-dirty", "PCB v2"],
          str(pr["welVer"]))
    # 画图真跑过: 断言画布宽度非零, 否则 drawMultiChart 一直在早退,
    # "没抛异常"就变成空断言 (隐藏页 clientWidth=0)
    check("五张图画布有实际宽度 (真走过绘制路径, 不是早退)",
          pr["drew"] and pr["drawW"][0] > 0 and pr["drawW"][1] > 0, str(pr["drawW"]))


# 「双击本地 html」那条路: 页面不能因为 scheme 是 file 就退回服务端态 (那样开始页
# 显示"串口桥接"、点连接什么也不会发生 —— 板子插在自己电脑上也用不了)。
# 真开一次浏览器, 让它自己探完模式再读: 探针不钉 ?mode=, 钉了就测不到探测本身。
# 断言随浏览器给不给 Web Serial 分叉 —— 给 → direct, 不给 → none (页面提示换浏览器);
# 这条要钉死的是「绝不回 server」。
PROBE_FILEMODE_JS = """
<div id="probe"></div>
<script>
/* detectMode 是异步的 (fetch 失败也要等一个 task), 等一拍再读 */
setTimeout(function () {
  try {
    document.getElementById("probe").textContent = JSON.stringify({
      mode: MODE, hasSerial: !!navigator.serial,
      label: document.getElementById("wel-web-label").textContent,
      web: document.getElementById("wel-web").textContent});
  } catch (e) { document.getElementById("probe").textContent = "THROW " + e; }
}, 50);
</script>
"""


def test_file_url_mode():
    """双击本地 html (file://): 该直连就直连, 不能退回服务端态"""
    pr, why = _probe_json(PROBE_FILEMODE_JS, "filemode")
    if pr is None:
        check("file:// 探模式探针能跑通", False, why)
        return
    check("file:// 探模式探针能跑通", True)
    want = "direct" if pr["hasSerial"] else "none"
    check(f"file:// 不再退回服务端态 (本机浏览器有 Web Serial = {pr['hasSerial']})",
          pr["mode"] == want,
          f"mode={pr['mode']!r} 该是 {want!r}; label={pr['label']!r} web={pr['web']!r}")


# 页面里的解析器与 server.py 必须同源: 正则表逐条同体 (改一处就得改另一处),
# 常量与 raw 白名单同值。这些是静态检查 —— 不需要浏览器, 也不需要跑固件。
def test_direct_contract():
    print("\n[6] 直连契约 (静态: 与 server.py 同源)")
    src = open(INDEX_HTML, encoding="utf-8").read()
    srv = open(os.path.join(HERE, "server.py"), encoding="utf-8").read()

    for name, needle in [
            ("Web Serial 入口", "navigator.serial"),
            ("帧分发器提到顶层 (两条数据路共用)", "function handleFrame(ev)"),
            ("直连引擎命名空间", "const Direct = {"),
            ("直连的行入口 (白名单 + 转帧)", "handleLine(line) {"),
            ("连接前先探模式 (?mode= 覆盖)", 'get("mode")'),
    ]:
        check(f"页面直连标志: {name}", needle in src, "index.html 里没有 " + needle)
    # file:// 不许被特判成服务端态: 双击本地 html 走的就是这条路 (探测里 fetch
    # /state 必然失败 → 落到直连, 行为另有 test_file_url_mode 真开浏览器验)
    check("file:// 不再被特判成服务端态 (双击本地 html 也能直连)",
          "location.protocol" not in src and "detectMode().then(pick);" in src,
          "index.html 里还有 file:// 特判, 或探测入口没了")

    # 正则表同源: 名字相同、模式体逐字节相同 (Python r"..." 与 JS /.../ 都是
    # 字面量, 所以体可以直接比字符串)。只取两边都有的; RE_DEV 是桥接侧专有
    # ([TCP] dev 由 serial_console.py 发布), 直连没有它。
    py_re = dict(re.findall(r'^((?:RE_)?[A-Z][A-Z0-9_]*) = re\.compile\(\s*r"'
                            r'((?:[^"\\]|\\.)*)"', srv, re.M))
    js_re = {n: (b, f) for n, b, f in
             re.findall(r'^const ((?:RE_)?[A-Z][A-Z0-9_]*) = /(.*)/([a-z]*);',
                        src, re.M)}
    both = sorted(set(py_re) & set(js_re))
    check(f"正则同源: 两边共有的模式都有 {len(both)} 条 (≥30, 防提取器空跑)",
          len(both) >= 30, f"交集只有 {len(both)} 条")
    drift = [n for n in both if py_re[n] != js_re[n][0]]
    check("正则同源: 每一条的模式体逐字节相同 (单侧改了另一侧没跟上)",
          not drift, "; ".join(f"{n}: server={py_re[n]!r} 页面={js_re[n][0]!r}"
                               for n in drift[:3]))
    flag_bad = [n for n in both if js_re[n][1] not in ("", "g")]
    check("正则同源: 页面侧只多一个 g 标志 (matchAll 要它, Python 侧靠 findall)",
          not flag_bad, str(flag_bad))
    # 常量: 通道数/中位值与环形缓冲长度写死在两边, 对不上就是静默的显示差异
    for const in ("CRSF_CH_COUNT", "CRSF_CH_MID", "U0_RX_RING"):
        a = re.search(rf'^{const} = (\d+)', srv, re.M)
        b = re.search(rf'^const {const} = (\d+)', src, re.M)
        check(f"常量同值: {const}",
              bool(a and b) and a.group(1) == b.group(1),
              f"server={a and a.group(1)} 页面={b and b.group(1)}")

    # 新协议行 (原始传感器 + 硬件版本): 正则体由上面的逐条比对钉住, 这里钉
    # "接进了行解析" —— 光定义正则不接进解析器 = 数据到了却没人认 (时序测试
    # 只喂固定夹具, 覆盖不到这种漏接)
    for name in ("RE_IMU_ACC", "RE_IMU_GYR", "RE_IMU_MAG", "RE_IMU_TEMP", "RE_HW"):
        check(f"直连契约: {name} 两边都接进了行解析",
              f"{name}.match(line)" in srv and f"{name}.exec(line)" in src, name)

    # raw 白名单: 两边的表必须一字不差 (差了就有行在一边进日志、另一边不进)
    m_py = re.search(r"parsed\[0\] in \(([^)]+)\)", srv, re.S)
    kinds = [x.strip().strip('"') for x in m_py.group(1).split(",") if x.strip()]
    m_js = re.search(r"RAW_SKIP = new Set\(\[([^\]]+)\]\)", src)
    js_kinds = [x.strip().strip('"') for x in m_js.group(1).split(",") if x.strip()]
    check("raw 白名单同表 (server on_line 与页面 RAW_SKIP)",
          kinds == js_kinds == RAW_SKIP, f"server={kinds} 页面={js_kinds}")

    # IMU 状态块折叠: 常量两边同值 (差一个字符就一边折一边不折, 用户只看到"日志
    # 有时吵有时不吵"), 且折叠必须挂在 raw 那条路径上 —— 折叠只能挡日志面板,
    # 挡了解析就等于曲线没有数据
    def _block_consts(text):
        out = {}
        for name in ("IMU_BLOCK_HEAD", "IMU_BLOCK_TAIL"):
            m = re.search(name + r'\s*=\s*"([^"]*)"', text)
            out[name] = m.group(1) if m else None
        m = re.search(r"IMU_BLOCK_MAX\s*=\s*(\d+)", text)
        out["IMU_BLOCK_MAX"] = int(m.group(1)) if m else None
        return out
    py_b, js_b = _block_consts(srv), _block_consts(src)
    check("IMU 状态块常量两边同值 (server 与页面)",
          py_b == js_b == {"IMU_BLOCK_HEAD": IMU_BLOCK_HEAD,
                           "IMU_BLOCK_TAIL": IMU_BLOCK_TAIL,
                           "IMU_BLOCK_MAX": IMU_BLOCK_MAX},
          f"server={py_b} 页面={js_b}")
    check("IMU 状态块折叠挂在日志路径上, 解析照走 (folded 只进 raw 的判断)",
          "if not folded and not (parsed" in srv
          and "if (!folded && !(parsed" in src)

    # !IMU 轮询间隔: 直连态 (页面 iv.imu) 与 server 态 (--imu-interval 默认) 同值,
    # 否则同一块板在两种模式下"跟手程度"不一样, 而用户看不出是模式差异
    m_iv = re.search(r"const iv = \{[^}]*?imu: ([0-9.]+)", src)
    m_si = re.search(r'"--imu-interval", type=float, default=([0-9.]+)', srv)
    check("!IMU 轮询间隔同值 (页面 iv.imu = server --imu-interval 默认)",
          bool(m_iv and m_si) and m_iv.group(1) == m_si.group(1),
          f"页面={m_iv and m_iv.group(1)} server={m_si and m_si.group(1)}")
    check("!IMU 轮询间隔是 0.2s (5Hz —— 姿态数字与曲线要跟手)",
          bool(m_iv) and m_iv.group(1) == "0.2", m_iv and m_iv.group(1))


def test_pages_workflow():
    """直连模式的托管: 单文件页面由 Pages 工作流发到站点根 (无构建步骤)"""
    wf = os.path.join(ROOT, ".github", "workflows", "pages.yml")
    check("GitHub Pages 工作流存在", os.path.isfile(wf), wf)
    if not os.path.isfile(wf):
        return
    y = open(wf, encoding="utf-8").read()
    head = y.split("jobs:")[0]                     # 触发条件区 (别拿 job 里的字符串顶数)
    check("工作流把 index.html 发成站点根",
          "tools/webconfig/index.html" in y and "_site/index.html" in y
          and "path: _site" in y)
    check("工作流在页面改动时触发, 也可手动跑",
          "tools/webconfig/index.html" in head and "workflow_dispatch:" in head)
    check("工作流用官方 Pages 三件套 (configure/upload/deploy)",
          all(a in y for a in ("actions/configure-pages@",
                               "actions/upload-pages-artifact@",
                               "actions/deploy-pages@")))
    # 缺 permissions 时 deploy 会失败得很难懂 (OIDC 拿不到 token)
    check("工作流声明 pages/id-token 权限",
          "pages: write" in y and "id-token: write" in y)
    # 站点没建过时 configure-pages 会以「Get Pages site failed」失败 —— 首次部署
    # 就卡在这里 (enablement: true 让它自己把站点建出来)
    check("工作流会自己启用 Pages 站点 (enablement)",
          "enablement: true" in y)


def test_release_workflow():
    """发固件的那条工作流: 版本串/发布说明/附件 —— 都是错了要等下一次发版才发现的东西"""
    wf = os.path.join(ROOT, ".github", "workflows", "release.yml")
    check("Release 工作流存在", os.path.isfile(wf), wf)
    if not os.path.isfile(wf):
        return
    y = open(wf, encoding="utf-8").read()
    check("Release 工作流按 v* tag 触发", "tags:" in y and "'v*'" in y)
    # 版本串 = 正在发的那个 tag, 由工作流显式传给构建: 实测 v0.4.0 那次 CI 里
    # fetch-depth: 0 也没让 git describe 认出刚推的 tag, 编出来的 uf2 报的是
    # v0.3.0-56-g9be97c7 —— 靠 git 状态猜不可靠 (实跑见 test_fw_version)
    check("Release 把 tag 名当版本串传给构建 (不靠 CI 里的 git describe)",
          "-DHEXAPOD_FW_GIT_OVERRIDE=${{ github.ref_name }}" in y)
    check("Release 注释写明 tag 要 annotated (git describe 不认轻量标签)",
          "annotated" in y)
    check("Release 说明 = 要点文件 + 自动提交清单",
          "body_path: .github/release-body.md" in y
          and "generate_release_notes: true" in y)
    body = os.path.join(ROOT, ".github", "release-body.md")
    check("发布要点文件存在 (发版时改它, 内容面向使用者)", os.path.isfile(body), body)
    check("Release 附件含固件与网页配置台 (单文件, 下载双击即用)",
          "pico/build/hexapod_pico.uf2" in y and "tools/webconfig/index.html" in y)


def _yaml_step_script(y, name):
    """从工作流文本里抠出某个 step 的 run: | 脚本 (不引第三方 YAML 库)"""
    lines = y.splitlines()
    start = next((i for i, l in enumerate(lines)
                  if l.strip() == f"- name: {name}"), None)
    if start is None:
        return None
    out, inrun, indent = [], False, 0
    for l in lines[start + 1:]:
        if not inrun:
            if l.strip().startswith("run: |"):
                inrun, indent = True, len(l) - len(l.lstrip()) + 2
            elif l.strip().startswith("- name:"):
                return None
        else:
            if l.strip() and (len(l) - len(l.lstrip())) < indent:
                break
            out.append(l[indent:] if len(l) > indent else "")
    return "\n".join(out) if inrun else None


def test_release_body_step():
    """发布说明的提交清单: 实跑工作流里那段脚本 (在共享克隆里跑, 不动工作区)"""
    wf = os.path.join(ROOT, ".github", "workflows", "release.yml")
    if not os.path.isfile(wf):
        check("Release 工作流存在 (清单步)", False, wf)
        return
    y = open(wf, encoding="utf-8").read()
    script = _yaml_step_script(y, "Append commit list to release body")
    check("Release 有「接提交清单」一步", bool(script))
    if not script:
        return
    # 顺序: 清单得在发布那一步之前写进 body 文件, 否则发出去的说明里没有清单
    check("清单步排在发布步之前",
          y.index("Append commit list to release body") < y.index("softprops/action-gh-release"))
    # 当前 tag 取本地最新那个 v* tag (测试跑在仓库任意状态上都不该假红)
    t = subprocess.run(["git", "tag", "-l", "v*", "--sort=-v:refname"],
                       cwd=ROOT, capture_output=True, text=True)
    tags = [x for x in t.stdout.split() if x]
    if len(tags) < 2:
        print("  (v* tag 不足两个, 清单步实跑跳过)")
        return
    cur, prev = tags[0], tags[1]
    tmp = "/tmp/wc_release_clone"
    subprocess.run(["rm", "-rf", tmp], check=False)
    r = subprocess.run(["git", "clone", "-q", "--shared", "--no-checkout", ROOT, tmp],
                       capture_output=True, text=True)
    if r.returncode != 0:
        check("共享克隆建得起来 (清单步实跑)", False, r.stderr.strip()[:120])
        return
    subprocess.run(["git", "-C", tmp, "checkout", "-q", "HEAD", "--", ".github"], check=False)
    env = dict(os.environ, GITHUB_REF_NAME=cur)
    r = subprocess.run(["bash", "-c", script], cwd=tmp, env=env,
                       capture_output=True, text=True)
    check("清单步脚本能跑通 (bash -e, 实测)", r.returncode == 0,
          (r.stderr or r.stdout).strip()[-160:])
    body = open(os.path.join(tmp, ".github", "release-body.md"), encoding="utf-8").read()
    want = subprocess.run(["git", "-C", tmp, "log", "--oneline", "--no-decorate",
                           f"{prev}..HEAD"], capture_output=True, text=True).stdout.splitlines()
    # 只数「提交」段里的行: 上面的要点本身就是个项目符号列表, 混在一起会数错
    marker = f"**提交 ({prev} → {cur})**"
    seg = body.split(marker, 1)[1] if marker in body else ""
    got = [l for l in seg.splitlines() if l.strip()]
    check(f"清单里是本版新增的提交 ({prev}..{cur} 共 {len(want)} 条)",
          got == ["- " + w for w in want],
          f"body 里 {len(got)} 条, git log 给 {len(want)} 条")
    check("清单不影响原有要点 (要点仍在 body 开头)",
          body.lstrip().startswith("**这一版最大的变化"))


def test_fw_version():
    """固件版本串怎么来的 —— 实跑 pico/cmake/gen_version.cmake 两种入口

    2026-09-28 实测: v0.4.0 那个 release 里 fetch-depth: 0 也没让 git describe
    看见刚推的 v0.4.0 (老 tag 在), 编出来的 uf2 报 v0.3.0-56-g9be97c7。所以发布
    走显式覆盖, 本地构建仍按 describe (能看出 dirty / 短哈希)"""
    script = os.path.join(ROOT, "pico", "cmake", "gen_version.cmake")
    cml = os.path.join(ROOT, "pico", "CMakeLists.txt")
    check("版本头生成脚本存在", os.path.isfile(script), script)
    if not os.path.isfile(script):
        return
    src = open(cml, encoding="utf-8").read() if os.path.isfile(cml) else ""
    # 少这一行, 工作流传了也白传: 脚本收不到覆盖串, 静默退回 git describe
    check("CMakeLists 把覆盖串原样转给版本脚本",
          "-DHEXAPOD_FW_GIT_OVERRIDE=${HEXAPOD_FW_GIT_OVERRIDE}" in src)
    cmake = shutil.which("cmake")
    if not cmake:
        print("  (无 cmake, 版本脚本实跑跳过)")
        return

    def gen(out, override=None):
        cmd = [cmake, f"-DSRC_DIR={os.path.join(ROOT, 'pico')}", f"-DOUT={out}"]
        # 空串也要传 (CMakeLists 变量为空时就是 -DVAR= 这个形状, 脚本按空处理)
        cmd.append(f"-DHEXAPOD_FW_GIT_OVERRIDE={override if override is not None else ''}")
        cmd += ["-P", script]
        r = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)
        if r.returncode != 0:
            return "EXIT %d %s" % (r.returncode, r.stderr.strip())
        return open(out, encoding="utf-8").read()

    ov = gen("/tmp/wc_ver_override.h", "v9.9.9")
    check("给覆盖串时版本头原样用它 (发布固件报的就是正在发的 tag)",
          '"v9.9.9"' in ov, ov)
    # 本地构建: 与本仓库此刻的 git describe 逐字一致 (dirty 也带出来)
    d = subprocess.run(["git", "describe", "--always", "--dirty"],
                       cwd=os.path.join(ROOT, "pico"),
                       capture_output=True, text=True)
    want = d.stdout.strip() if d.returncode == 0 and d.stdout.strip() else "unknown"
    df = gen("/tmp/wc_ver_describe.h")
    check("不给覆盖串时仍按 git describe (本地能看出 dirty/短哈希)",
          f'"{want}"' in df, f"脚本给了 {df!r}, git describe 给的是 {want!r}")


def wait_dev(port, up, timeout=20):
    """等 /state 里的设备在线状态变成 up"""
    end = time.time() + timeout
    while time.time() < end:
        try:
            st = json.loads(http_get("/state", port))
            if bool(st.get("dev", {}).get("up")) is up:
                return st
        except (OSError, json.JSONDecodeError):
            pass
        time.sleep(0.3)
    return None


def wait_conn(port, want, timeout=5):
    """等 /state 的连接态变成 want → 状态快照 (超时返回 None)"""
    end = time.time() + timeout
    while time.time() < end:
        try:
            st = json.loads(http_get("/state", port))
            if bool(st.get("connected")) is want:
                return st
        except (OSError, json.JSONDecodeError):
            pass
        time.sleep(0.2)
    return None


def gate_batt_count(sse, seconds, skip=0.0):
    """一段时间内收到的 batt 帧数量 (轮询在跑就一定 >0)

    skip: 先空转这么久丢掉在途帧。刚发的 !BATT 的应答会晚几十毫秒才到, 断言
    "停止轮询" 时必须先把它冲掉, 否则数到的是那条命令的回声而不是新命令。
    空转期间的帧也一并返回 —— 事件断言要能看到 (conn 就常常落在这一窗口里)。"""
    evs = sse_drain(sse, skip) if skip else []
    tail = sse_drain(sse, seconds)
    return sum(1 for e in tail if e.get("t") == "batt"), evs + tail


def locked_frames(evs):
    """只挑出"门是锁的"那些 rc 帧 (解锁回执不是心跳)"""
    return [e for e in evs if e.get("t") == "rc" and e.get("d", {}).get("locked")]


def test_connect_gate():
    print("\n[4] 连接门: 连接前不轮询 / 连接后轮询 / 遥控门锁定与心跳 / 掉线回开始页")

    for args, log, cwd in [
        (["socat", "-d", "-d", f"pty,raw,echo=0,link={PTY_A2}",
          f"pty,raw,echo=0,link={PTY_B2}"], "/tmp/wc_socat2.log", TOOLS),
        ([sys.executable, __file__, "--replay", PTY_A2], "/tmp/wc_fake2.log", HERE),
        ([sys.executable, "serial_console.py", PTY_B2, "--port", str(BRIDGE_PORT2)],
         "/tmp/wc_console2.log", TOOLS),
        ([sys.executable, "server.py", "--port", str(HTTP_PORT2),
          "--bridge-port", str(BRIDGE_PORT2),
          "--batt-interval", str(GATE_POLL_IV), "--imu-interval", str(GATE_POLL_IV),
          "--servo-interval", str(GATE_POLL_IV), "--i2c-interval", str(GATE_POLL_IV),
          "--periph-interval", str(GATE_POLL_IV),
          "--ports-interval", str(GATE_POLL_IV),
          "--rc-interval", str(GATE_POLL_IV)], "/tmp/wc_server2.log", HERE),
    ]:
        spawn(args, log, cwd)

    # 设备状态要一路传到 server: serial_console 打开 pty → 发布 [TCP] dev on
    # → server 解析成 dev 事件。这条链断了开始页就永远显示"未检测到设备"
    st = wait_dev(HTTP_PORT2, True)
    check("设备在线状态经桥接传到网页服务", st is not None,
          "20s 内 /state 的 dev.up 仍为 false")
    if st is None:
        return
    check("在线的设备带端口号 (开始页要显示)",
          bool(st["dev"].get("port")), str(st["dev"]))

    dev = json.loads(http_get("/device", HTTP_PORT2))
    check("GET /device 也带设备在线状态与连接态",
          dev.get("dev", {}).get("up") is True and dev.get("connected") is False,
          str({k: dev.get(k) for k in ("dev", "connected")}))

    sse = sse_open(HTTP_PORT2)
    evs = sse_drain(sse, 0.5)
    hello = next((e for e in evs if e.get("t") == "hello"), None)
    check("开场帧带设备状态与连接态 (刷新页面据此选页面)",
          hello is not None and hello.get("dev", {}).get("up") is True
          and hello.get("connected") is False, str(hello))

    # ---- 连接之前: 一个轮询命令都不该发 ----
    n, evs0 = gate_batt_count(sse, 2.5)
    check("连接前不轮询 (开始页不打扰串口)", n == 0, f"{n} 个 batt 帧")
    check("连接前不碰遥控门 (站在开始页时遥控器照常能开)",
          not [e for e in evs0 if e.get("t") == "rc"], str(evs0[:3]))

    # ---- 点连接: 不带 body = 没勾「调试时允许遥控器控制」 ----
    r = json.loads(http_post("/connect", "", HTTP_PORT2))
    check("POST /connect 成功", r.get("ok") is True, str(r))
    n, evs = gate_batt_count(sse, 3.0)
    check("连接后开始轮询", n > 0, f"{n} 个 batt 帧")
    check("连接事件推给所有浏览器 (多标签页一致)",
          any(e.get("t") == "conn" and e.get("on") is True for e in evs),
          str([e.get("t") for e in evs][:8]))

    # ---- 遥控门: 默认锁上, 靠心跳续租 (固件 6s 收不到就自己解锁) ----
    rc = [e for e in evs if e.get("t") == "rc"]
    locks = locked_frames(rc)
    check("连接后锁上遥控门 (rc 帧 locked=True)", locks, str(rc[:3]))
    check("锁定期间持续重发 !RC 0 (心跳, 3s 内 ≥3 条)",
          len(locks) >= 3, f"{len(locks)} 条锁定帧")
    check("遥控门状态推给浏览器 (控制页徽标的数据源)",
          all(e.get("d", {}).get("locked") is not None for e in rc), str(rc[:3]))

    # ---- 断开: 回开始页, 轮询停 ----
    r = json.loads(http_post("/disconnect", "", HTTP_PORT2))
    check("POST /disconnect 成功", r.get("ok") is True, str(r))
    # skip: 断开前刚发的那条 !BATT 的应答还在路上, 先冲掉再数 (它在途, 不是新命令)
    n, evs = gate_batt_count(sse, 2.5, skip=0.6)
    check("断开后停止轮询", n == 0, f"{n} 个 batt 帧")
    check("断开事件推给所有浏览器",
          any(e.get("t") == "conn" and e.get("on") is False for e in evs),
          str([e.get("t") for e in evs][:8]))
    # 遥控门要交还: 先看最后一条 rc 帧是不是"放开", 再看放开之后有没有被锁回去
    # (在途的 !RC 0 可能比 !RC 1 早到, 但绝不会晚到 —— 两者同锁互斥)
    rc = [e for e in evs if e.get("t") == "rc"]
    unlock = max((i for i, f in enumerate(rc) if not f["d"].get("locked")), default=None)
    check("断开连接放开遥控门 (!RC 1 → rc 帧 unlocked)", unlock is not None,
          str(rc[-3:]))
    check("放开之后不再被锁回去 (心跳停了)",
          unlock is not None and not locked_frames(rc[unlock + 1:]),
          str(rc[unlock:][:3]))

    # 断开只是不连了, 设备还在线 —— 可以再连回来
    r = json.loads(http_post("/connect", json.dumps({"radio": True}), HTTP_PORT2))
    check("断开后可再次连接 (设备仍在线)", r.get("ok") is True, str(r))
    n, evs = gate_batt_count(sse, 3.0)
    check("重新连接后轮询恢复", n > 0, f"{n} 个 batt 帧")
    # 勾了「调试时允许遥控器控制」: 只回一条解锁 (固件可能上一轮还锁着), 没有心跳
    rc = [e for e in evs if e.get("t") == "rc"]
    check("radio=true 时不锁遥控门 (只有一条解锁回执)",
          any(not f["d"].get("locked") for f in rc) and not locked_frames(rc),
          str(rc[:3]))

    # ---- 拔线: 设备消失 → 连接态作废, 浏览器自动回开始页 ----
    socat2 = procs[-4]                      # 本组第一个 spawn 的是 socat
    assert socat2.args[0] == "socat", socat2.args   # 杀错进程 = 用例前提不成立
    socat2.kill()
    evs = sse_drain(sse, 10, stop_when=lambda e: any(
        x.get("t") == "dev" and x.get("up") is False for x in e))
    off = next((e for e in evs if e.get("t") == "dev" and e.get("up") is False), None)
    check("设备掉线推到浏览器 (前端据此跳回开始页)", off is not None)
    evs += sse_drain(sse, 1.0)      # dev 与 conn 是两个 TCP 分段, 别只看第一段
    check("设备掉线同时作废连接态",
          any(e.get("t") == "conn" and e.get("on") is False for e in evs),
          str([e.get("t") for e in evs][:8]))
    n, _ = gate_batt_count(sse, 2.5)
    check("掉线后不再轮询 (命令会被串口端排队)", n == 0, f"{n} 个 batt 帧")

    # ---- 插回来: 设备在线, 但**不会**自动重连 ----
    # 拔插 = 设备和线一起重新接入: socat 重建的是一对**新的** pty, 只重启 socat
    # 的话旧假固件还挂在已经作废的那个 pty 上, 新 pty 上没人应答 —— 轮询发出去
    # 只会石沉大海, 那是"假设备"的假象, 不是服务器的问题。
    spawn(["socat", "-d", "-d", f"pty,raw,echo=0,link={PTY_A2}",
           f"pty,raw,echo=0,link={PTY_B2}"], "/tmp/wc_socat3.log", TOOLS)
    spawn([sys.executable, __file__, "--replay", PTY_A2], "/tmp/wc_fake3.log", HERE)
    st = wait_dev(HTTP_PORT2, True)
    check("设备插回后重新在线", st is not None)
    n, _ = gate_batt_count(sse, 2.5)
    check("插回后不自动重连 (得用户再点连接)", n == 0, f"{n} 个 batt 帧")
    check("插回后连接态仍为断开", st is not None and st.get("connected") is False,
          str(st.get("connected") if st else None))

    r = json.loads(http_post("/connect", "not json at all", HTTP_PORT2))
    check("再次连接成功", r.get("ok") is True, str(r))
    n, evs = gate_batt_count(sse, 3.0)
    check("再连后轮询恢复", n > 0, f"{n} 个 batt 帧")
    # body 解析不出来时按"没勾开关"处理 —— 宁可锁住也不能让遥控器误接管
    check("连接 body 不是 JSON 时按默认锁定",
          locked_frames(evs), str([e.get("t") for e in evs][:8]))
    sse.close()

    # 最后一个浏览器走了: 连接态作废 (否则重新打开页面时"没人连却在轮询")
    st = wait_conn(HTTP_PORT2, False)
    check("最后一个浏览器离开后连接态复位", st is not None,
          "5s 内未复位" if st is None else f"connected={st.get('connected')}")


def main():
    if len(sys.argv) > 2 and sys.argv[1] == "--replay":
        replay_robot(sys.argv[2])
        return

    test_parser()
    test_imu_block_fold()
    test_param_parser()
    test_firmware_contract()
    test_launcher_contract()
    test_flasher()

    print("\n[3] 端到端 (socat pty + 假固件 + 串口控制台 + 网页服务)")
    for args, log, cwd in [
        (["socat", "-d", "-d", f"pty,raw,echo=0,link={PTY_A}",
          f"pty,raw,echo=0,link={PTY_B}"], "/tmp/wc_socat.log", TOOLS),
        ([sys.executable, __file__, "--replay", PTY_A], "/tmp/wc_fake.log", HERE),
        ([sys.executable, "serial_console.py", PTY_B, "--port", str(BRIDGE_PORT)],
         "/tmp/wc_console.log", TOOLS),
        ([sys.executable, "server.py", "--port", str(HTTP_PORT),
          "--bridge-port", str(BRIDGE_PORT), "--no-poll"], "/tmp/wc_server.log", HERE),
    ]:
        spawn(args, log, cwd)
    time.sleep(3.0)

    # 桥接连上为止
    ok = False
    for _ in range(30):
        try:
            st = json.loads(http_get("/state"))
            if st.get("bridge"):
                ok = True
                break
        except (OSError, json.JSONDecodeError):
            pass
        time.sleep(0.5)
    check("网页服务起来并连上串口桥接", ok)

    raw_page = _raw_request("GET / HTTP/1.0\r\nHost: 127.0.0.1\r\n\r\n")
    check("静态页可访问", raw_page.startswith("HTTP/1.0 200")
          and "text/html" in raw_page and "HEXAPOD" in raw_page)
    # 校准保存按钮: 走 data-cmd 快捷按钮 → /cmd, 没有服务端解析环节,
    # 唯一会坏的方式是按钮被删/改名, 所以直接断言页面里有它
    check("校准页含 !SAVE 保存校准按钮", 'data-cmd="!SAVE"' in raw_page
          and "!SAVE 保存校准" in raw_page)
    check("校准页含 !C 走查与 !I2C 扫描按钮",
          'data-cmd="!C"' in raw_page and 'data-cmd="!I2C"' in raw_page)
    check("校准页有 18 路舵机格子与 I2C 摘要",
          'id="cal-servos"' in raw_page and 'id="cal-i2c"' in raw_page)
    # 校准页另外两张卡: 电压标定 (页面侧算分压比) 与 IMU 校准 (显示 + !IMUR 重置)。
    # 电压卡的几个 id 互相喂数据 (读数/引脚电压来自 !BATT 帧, 分压比来自参数帧),
    # 任何一个改名都会让卡片停在「—」而全程无报错, 所以逐个钉住。
    check("校准页有电压标定卡 (读数/引脚/分压比/实测输入/下发)",
          all(f'id="cal-v-{i}"' in raw_page
              for i in ("batt", "pin", "ratio", "meas", "apply", "new")))
    check("电压标定按 实测÷引脚电压 算分压比 (与固件注释同一条式子)",
          "Math.round(meas / BATT.pin_mv * 1000)" in raw_page
          and 'sendSet("batt_ratio_milli", nv)' in raw_page)
    check("校准页有 IMU 校准卡 (在线/姿态/等级) 与 !IMUR 重置按钮",
          all(f'id="calimu-{i}"' in raw_page for i in ("av", "rp", "calib"))
          and 'data-cmd="!IMUR"' in raw_page)
    check("IMU 校准卡复用 !IMU 帧 (等级/姿态与状态页同源)",
          'const cav = $("calimu-av")' in raw_page
          and '$("calimu-calib").className = d.fully ? "ok" : "warn"' in raw_page)

    # 版本与曲线 (2026-09 加): 版本行必须在固件卡**外面** —— 直连态整卡隐藏,
    # 版本行留在卡里就是"开始页显示不了固件版本" (位置用索引先后钉住)
    check("开始页版本行在固件卡之外 (直连态才看得见)",
          'id="wel-row-ver"' in raw_page and 'id="wel-hw"' in raw_page
          and raw_page.index('id="wel-row-ver"') < raw_page.index('id="wel-fw-card"'))
    check("状态栏有固件/硬件版本两个位 (x 行/秒旁边)",
          'id="fwver"' in raw_page and 'id="hwver"' in raw_page)
    check("直连态版本靠 localStorage 记住 (连过再开开始页也有)",
          'lsGet("hexapod_ver")' in raw_page and 'lsGet("hexapod_hw")' in raw_page
          and 'lsSet("hexapod_ver"' in raw_page and 'lsSet("hexapod_hw"' in raw_page)
    check("状态总览有五张 IMU 曲线 (姿态/加速度/陀螺仪/磁力计/温度)",
          all(f'id="ch-{k}"' in raw_page for k in ("att", "acc", "gyr", "mag", "tmp"))
          and 'id="chart"' in raw_page)
    # 曲线推点必须过时间闸门 (轮询帧是累积快照, 不挡会以数倍速灌重复点),
    # 但**不能**按值去重 —— 静止时值不变也得长点, 不然曲线看着像死了。
    # 横轴按真时间铺 (窗口 1 分钟), 电池与 IMU 五张共用同一套 histPush。
    check("曲线推点按时间闸门挡重复快照 (旧的值去重已拆掉)",
          "function histPush(h, vals)" in raw_page
          and "t - h[h.length - 1].t < CH_GATE_MS" in raw_page
          and "IMU_PREV" not in raw_page)
    check("六张图都是 1 分钟时间窗 + 真时间横轴",
          "CH_WINDOW_MS = 60 * 1000" in raw_page
          and "function histXOf(" in raw_page
          and raw_page.count("const X = histXOf(") == 2
          and "t - h[0].t > CH_WINDOW_MS" in raw_page)
    check("切页/出隐藏/缩放/掉线清空都重画全部曲线",
          raw_page.count("drawAllCharts") >= 4
          and "if (name === \"dash\") drawAllCharts();" in raw_page
          and 'window.addEventListener("resize", drawAllCharts);' in raw_page)

    # 参数分页: 每页一个 tab 按钮 + 一个卡片宿主。页面本身由 JS 按 PAGES 生成,
    # 所以断言生成器的输入 (PAGES 里的 id) 与 HTML 里的 tab 按钮两边都在。
    for pid in ("gait", "stance", "geo", "radio", "modes", "power", "balance", "per",
                "ctrl", "ports"):
        check(f"页面含参数分页 tab: {pid}",
              f'data-tab="{pid}"' in raw_page and f'id: "{pid}"' in raw_page)
    check("分页的组覆盖固件全部分组",
          all(f'"{g}"' in raw_page for g in WEB_GROUPS))
    check("旧的单页参数 tab 已移除", 'data-tab="params"' not in raw_page)

    # 保存按钮置灰 (无未保存改动时压下去只是空写一次 flash)。判定依据是 pBase
    # 基线 = 上次存盘时的值, 不是「与默认值不同」—— 后者是「本页恢复默认」按钮
    # 的语义。两条线必须各走各的, 所以这里分别断言。
    check("每页生成 保存/恢复默认/统计 三件套",
          all(f'id="p-{k}-${{pg.id}}"' in raw_page
              for k in ("save", "reset", "summary")))
    check("保存键由「未保存改动数」驱动",
          "function isUnsaved" in raw_page and "sv.disabled = !unsaved" in raw_page)
    check("恢复默认键由「偏离默认值」驱动 (另一条线)",
          "function star(p) { return p.val !== p.def; }" in raw_page
          and "rs.disabled = !mine.some(star)" in raw_page)

    # 遥控页: 输入源切换按钮 + 通道实时面板 (由 PAGES 的 custom 字段生成,
    # 所以断言生成器的输入在页面里, 与上面分页断言同一套路)
    check("遥控页有 PS2/CRSF 切换按钮",
          'data-cmd="!MODE crsf"' in raw_page and 'data-cmd="!MODE ps2"' in raw_page)
    check("遥控页有通道实时面板", 'id="ch-panel"' in raw_page and 'id="ch-mode"' in raw_page)
    check("通道格子按功能名反查 (改滑条后跟着变)",
          "ch_fwd" in raw_page and "renderCh" in raw_page)

    # 输入源有两个入口 (按钮 / input_mode 滑条), 必须只剩一个说法:
    # 滑条被 PARAM_HIDE 挡掉, 按钮点亮当前生效的模式。
    # 模式页的 19 个 mode_*/combo_* 同样走自定义控件, 也一并挡掉通用滑条
    import re as _re
    m_hide = _re.search(r"const PARAM_HIDE = new Set\(\[(.*?)\]\)", raw_page, _re.S)
    hidden = _re.findall(r'"(\w+)"', m_hide.group(1)) if m_hide else []
    firm = {n for n, _, _ in
            RE_TABLE_ROW.findall(open(PARAMS_C, encoding="utf-8").read())}
    # 端口页的六个同理 (下拉比滑条直观), 所以也在这份名单里
    want_hide = ({"input_mode"} | {n for n in firm if n.startswith(("mode_", "combo_"))}
                 | {"dc_motor_en", "foot_sw_en", "gp23_fn", "gp24_fn", "gp29_fn",
                    "input_baud_serial"})
    check("input_mode/19 个 mode_*/combo_*/6 个 port 参数不生成通用滑条 (自定义控件接管)",
          set(hidden) == want_hide and len(hidden) == len(want_hide)
          and len(hidden) == 26, f"PARAM_HIDE={hidden}")
    check("PARAM_HIDE 里的名字都是真参数 (防拼错导致参数凭空消失)",
          set(hidden) <= firm,
          "不存在: " + ", ".join(sorted(set(hidden) - firm)))

    # 模式页 (Betaflight 式矩阵 + PS2 组合键): 控件由 JS 按 MODES/COMBOS 表建,
    # 所以断言生成器的输入在页面里 —— 与分页断言同一套路。参数名后缀 = 控件 id
    # 后缀 (mode_arm → md-arm), 两边拼错一个字母就是"改了没反应且无报错"。
    check("模式页有矩阵与组合键两张表的宿主",
          'id="md-rows"' in raw_page and 'id="cb-rows"' in raw_page
          and "const MODES = [" in raw_page and "const COMBOS = [" in raw_page)
    check("模式矩阵 10 行 (解锁/平衡/5 步态/3 站姿) 控件齐全",
          all(f'["{p}", ' in raw_page for p in
              ("mode_arm", "mode_bal", "mode_g0", "mode_g1", "mode_g2", "mode_g3",
               "mode_g4", "mode_sn", "mode_sp", "mode_sw"))
          and 'id="md-sel-${key}"' in raw_page
          and all(f'id="md-{lv}-${{key}}"' in raw_page for lv in ("lo", "md", "hi")))
    check("组合键 9 行 (解锁/平衡/步态±/站姿±/抬腿±/急停) 控件齐全",
          all(f'["{p}", ' in raw_page for p in
              ("combo_arm", "combo_bal", "combo_gnext", "combo_gprev", "combo_snext",
               "combo_sprev", "combo_lup", "combo_ldn", "combo_estop"))
          and 'id="cb-a-${key}"' in raw_page and 'id="cb-b-${key}"' in raw_page)
    # 单击触发勾选框: 两张表每行一个 (mode → bit8, combo → 勾=单击=hold 清 0)
    check("矩阵与组合键每行都有「单击触发」勾选框",
          'id="md-ck-${key}"' in raw_page and 'id="cb-ck-${key}"' in raw_page
          and "单击触发" in raw_page)
    # 勾选框的位置: .mdrow 是 4 列网格 (名字|通道|低中高|单击), 矩阵行与组合键行共用。
    # 塞进第 3 列 (低/中/高 那格, 本身就要 150px) 会挤爆, 组合键行的第 4 个元素则
    # 会折到下一行去 —— 两种都只是"看着奇怪"、不报错, 所以在这里钉住。
    check("矩阵行/组合键行的「单击触发」都在第 4 列 (class=\"ck\", 不在 .lv 里)",
          raw_page.count('<div class="lv">') == 1
          and raw_page.count('<label class="ck"><input type="checkbox" id="md-ck-${key}">'
                             '单击触发</label>') == 1
          and raw_page.count('<label class="ck"><input type="checkbox" id="cb-ck-${key}">'
                             '单击触发</label>') == 1
          and ".mdrow .ck {" in raw_page)
    check("模式页网格是 4 列 (组合键行不再折到第二行)",
          "grid-template-columns: 150px 190px 150px auto;" in raw_page)
    # 打包格式必须与固件 hexapod_input.h 一致: ch<<3|bits|click<<8 / a|b<<5|hold<<10
    check("矩阵值按 ch<<3|bits|click<<8 打包 (ch=31 未分配, click 在 bit8)",
          "MODE_NONE << 3" in raw_page and "(ch << 3) | bits" in raw_page
          and "(click << 8)" in raw_page and "const MODE_NONE = 31" in raw_page)
    check("组合键按 a|b<<5|hold<<10 打包 (16 = 无, 勾单击 = hold 0)",
          "a | (b << 5)" in raw_page and "(hold << 10)" in raw_page
          and "const COMBO_NONE = 16" in raw_page)
    check("回显先掩低 8 位再拆 ch (click 位不渗进通道号)",
          "p.val & 0xFF" in raw_page and "const lo = p.val & 0xFF" in raw_page)
    check("模式页实时徽标 (解锁/步态/平衡/站姿/高度/输入源)",
          all(f'id="mds-{i}"' in raw_page
              for i in ("arm", "gait", "bal", "st", "hi", "in"))
          and "function renderModes()" in raw_page)
    check("高度积分开关有专门的解释 (与线性模式的区别)",
          "height_integrate:" in raw_page and "回中保持" in raw_page)
    # 几何页: 21 项走通用滑条 (无自定义控件), 关键是「重启后生效」的说明 ——
    # 快照模板默认给每页追加「改完即时生效」, geo 页必须显式关掉 (instant: false)
    check("几何页存在且注明重启后生效",
          'data-tab="geo"' in raw_page and 'id: "geo"' in raw_page
          and "重启后生效" in raw_page and 'instant: false' in raw_page
          and 'pg.instant === false ? "" :' in raw_page)
    check("输入源按钮按固件回报的模式点亮 (不是按点击)",
          'id="ch-btn-crsf"' in raw_page and 'classList.toggle("on"' in raw_page)

    # 外设页: 手动控制卡片 (data-cmd 走通用接线) + 状态回显面板。
    # 这些按钮没有服务端解析环节, 唯一会坏的方式是名字被改/删
    check("外设页有手动驱动按钮",
          all(f'data-cmd="{c}"' in raw_page
              for c in ("!MOTOR", "!LED g 1", "!LED r 0", "!I2C2", "!ADC2",
                        "!PWMOFF", "!PERIPH")))
    check("外设页有状态回显面板",
          all(f'id="{i}"' in raw_page
              for i in ("per-motors", "per-leds", "per-buzzer", "per-uart0",
                        "per-rx", "per-ext", "per-pwm", "per-stale")))
    check("外设页的 14 路 PWM 滑条按固件编号生成",
          "PER_PWM" in raw_page and "idx ${i} · ${PER_PWM[i]}" in raw_page)

    # 控制页: 主动接管 (供电/行走/遥控门/两块板的 PWM 周期)。命令按钮全是
    # data-cmd 走通用接线, 没有服务端解析环节 —— 会坏的方式只有被改名/删掉
    check("控制页有供电按钮与回报徽标",
          'data-cmd="!O 1"' in raw_page and 'data-cmd="!O 0"' in raw_page
          and 'id="run-badge"' in raw_page)
    check("控制页有行走/步态/抬腿按钮",
          all(f'data-cmd="{c}"' in raw_page
              for c in ("!F", "!B", "!L", "!R", "!Q", "!E", "!S",
                        "!G0", "!G1", "!G2", "!G3", "!G4", "!U", "!D")))
    check("控制页显示固件回报的行程 (只读)",
          'id="run-travel"' in raw_page and "RUN_DIR" in raw_page)
    check("控制页有遥控门徽标 (锁没锁以固件回执为准)",
          'id="rc-badge"' in raw_page and "已锁定 · 遥控器不驱动机器人" in raw_page)
    check("控制页两块板各一条周期滑条 + 各自 Hz 标签",
          all(f'id="{i}"' in raw_page
              for i in ("per-p0", "per-p0n", "per-p0hz",
                        "per-p1", "per-p1n", "per-p1hz")))
    check("控制页有 PWM 周期的 !SAVE 按钮",
          'id="ctrl-persave" data-cmd="!SAVE"' in raw_page)

    # 周期滑条的范围 = 固件 clamp: 超出部分固件会静默夹紧, 那滑条上显示的 Hz
    # 就不是板上真实频率了 (两个数字分处两边, 改一个忘另一个不会有任何报错)
    m_rng = _re.search(r'id="per-p0" min="(\d+)" max="(\d+)"', raw_page)
    i2c_c = open(I2C_C, encoding="utf-8").read()
    check("周期滑条范围与固件 clamp 一致 (5000~15000µs)",
          m_rng is not None and m_rng.groups() == ("5000", "15000")
          and "period_us < 5000" in i2c_c and "period_us > 15000" in i2c_c,
          str(m_rng.groups() if m_rng else None))

    # 参数名后的「?」: 内容在 HELP 表里, 悬停由 CSS 出气泡。键名拼错的唯一
    # 表现是那个「?」不出现 —— 静默失效, 所以这里钉住键名都在固件参数表里
    m_help = _re.search(r"const HELP = \{(.*?)\n\};", raw_page, _re.S)
    hkeys = set(_re.findall(r"^\s*(\w+):", m_help.group(1), _re.M)) if m_help else set()
    check("HELP 解释表存在且有多条", len(hkeys) >= 5, str(sorted(hkeys)))
    check("HELP 的键都是真参数 (防拼错导致「?」静默不出现)",
          hkeys <= firm, "不存在: " + ", ".join(sorted(hkeys - firm)))
    check("参数行的「?」由 HELP 表驱动, 悬停出气泡",
          'q.className = "phelp"' in raw_page and 'q.dataset.tip = HELP[p.name]' in raw_page
          and ".phelp:hover::after" in raw_page
          and "content: attr(data-tip)" in raw_page)

    # 开始页的遥控开关: 勾没勾决定连接时锁不锁遥控器 (随 POST /connect 的 body 走)
    check("开始页有「调试时允许遥控器控制」开关",
          'id="rc-keep"' in raw_page and "调试时允许遥控器控制" in raw_page)
    check("连接请求带上开关状态 (不勾 = 锁)",
          _re.search(r'post\("/connect",\s*JSON\.stringify\(\{\s*radio:', raw_page)
          is not None)

    sse = sse_open()
    time.sleep(0.3)
    r = json.loads(http_post("/cmd", "!BATT"))
    check("POST /cmd 下发成功", r.get("ok") is True, str(r))
    events = sse_drain(sse, 8, stop_when=lambda evs: any(
        e.get("t") == "batt" and e.get("d", {}).get("batt_mv") == 8370 for e in evs))
    sse.close()

    types = [e.get("t") for e in events]
    check("SSE 收到开场帧", "hello" in types)
    check("SSE 收到原始日志行", any(
        e.get("t") == "raw" and "Battery ADC" in e.get("line", "") for e in events))
    batt = next((e["d"] for e in events if e.get("t") == "batt"
                 and "batt_mv" in e.get("d", {})), None)
    check("SSE 收到解析后的电池事件", batt is not None and batt["batt_mv"] == 8370,
          str(batt))

    # ---- 通道遥测: 固件自由推送 (无对应命令), 所以要等它自己到 ----
    # 这段必须在 /poll off 之前 —— 暂停轮询不影响推送, 但保持与页面一致的时序
    # (浏览器接入后先看到的就是推送帧)
    sse_p = sse_open()
    time.sleep(0.3)
    evs = sse_drain(sse_p, 4, stop_when=lambda e: any(x.get("t") == "ch" for x in e))
    sse_p.close()
    ch_ev = next((e["d"] for e in evs if e.get("t") == "ch"), None)
    check("SSE 收到固件自由推送的通道帧 (无命令触发)", ch_ev is not None, str(evs[:6]))
    if ch_ev:
        check("推送的通道帧内容正确",
              ch_ev.get("mode") == "crsf" and ch_ev.get("ch", [])[:2] == [172, 1500],
              str(ch_ev))
        check("通道帧不进日志面板 (被解析的不再当 raw 重发)",
              not any(e.get("t") == "raw" and "[CH]" in e.get("line", "") for e in evs))

    # ---- 外设状态: 服务端用 --no-poll 起的, 所以这里先确认"没人主动问",
    # 再手动问一次 —— !PERIPH 是轮询线程发的, 而 --no-poll 必须连新加的那个
    # 间隔一起关掉 (漏了就会在用户明确要求静默时仍然每 2 秒打扰一次串口) ----
    sse_p = sse_open()
    time.sleep(0.3)
    evs = sse_drain(sse_p, 2.5)
    check("--no-poll 时没有 !PERIPH 主动轮询 (外设轮询也被关掉)",
          not any(e.get("t") == "per" for e in evs), str(evs[:6]))

    r = json.loads(http_post("/cmd", "!PERIPH"))
    check("POST /cmd 下发 !PERIPH", r.get("ok") is True, str(r))
    # 6 行是一行一行到的, 每行只带自己那一节 —— 等最后一节也到了再看快照
    evs = sse_drain(sse_p, 4, stop_when=lambda e: any(
        x.get("t") == "per" and {"motors", "leds", "pwm"} <= set(x.get("d", {}))
        for x in e))
    sse_p.close()
    per_ev = next((e["d"] for e in reversed(evs) if e.get("t") == "per"), None)
    check("SSE 收到解析后的外设帧", per_ev is not None, str(evs[:6]))
    if per_ev:
        check("外设帧含电机/LED/PWM 三节",
              per_ev.get("motors") == {"m1": 500, "m2": 0}
              and per_ev.get("leds", {}).get("hb") == 1
              and per_ev.get("pwm", {}).get("3") == 5000, str(sorted(per_ev)))
        check("外设行不进日志面板 (6 行一条, 每 2 秒会把日志冲掉)",
              not any(e.get("t") == "raw" and wc.RE_PER.match(e.get("line", ""))
                      for e in evs),
              str([e.get("line") for e in evs if e.get("t") == "raw"][:3]))

    # 带参数的外设命令 (回放按"命令头 + 空格"认, 固件侧也是按命令头分发的)。
    # 断言 u0_rx —— 前面的用例没碰过它, 所以这一帧只可能来自这条命令
    sse_p = sse_open()
    time.sleep(0.3)
    http_post("/cmd", "!UART0T hello")
    evs = sse_drain(sse_p, 3, stop_when=lambda e: any(
        x.get("t") == "per" and "u0_rx" in x.get("d", {}) for x in e))
    sse_p.close()
    rx = next((e["d"].get("u0_rx") for e in evs if e.get("t") == "per"
               and "u0_rx" in e.get("d", {})), None)
    check("带参数的外设命令也能过 (命令头前缀匹配)", rx == ["hello"], str(rx))

    # ---- 舵机机械中心: !HOS 全量回读 / !HO <id> <angle> 单路写入 ----
    # 两路都汇进同一条 "ho" 帧 (页面靠它刷格子里的偏移)。服务端用 --no-poll 起的,
    # 所以这一帧只可能来自下面这两条 POST。
    sse_p = sse_open()
    time.sleep(0.3)
    r = json.loads(http_post("/cmd", "!HOS"))
    check("POST /cmd 下发 !HOS", r.get("ok") is True, str(r))
    evs = sse_drain(sse_p, 3, stop_when=lambda e: any(
        len(x.get("d", {}).get("offs", {})) == 18 for x in e if x.get("t") == "ho"))
    offs = next((e["d"]["offs"] for e in reversed(evs)
                 if e.get("t") == "ho" and len(e.get("d", {}).get("offs", {})) == 18),
                None)
    check("SSE 收到 !HOS 的 18 路偏移帧", offs is not None, str(evs[:4]))
    if offs:
        want = [int(l.split("off=")[1]) for l in S_HO]
        # JSON 把 int 键变成字符串 (SSE 线上如此, 前端拿到的就是 str 键)
        check("偏移值与固件回显逐路一致 (含负值)",
              [offs.get(str(i)) for i in range(18)] == want, str(offs))

    # 单路写入: 两个参数都要过 (id 与角度), 回执后再看同一张表里那一路
    http_post("/cmd", "!HO 0 12")
    evs = sse_drain(sse_p, 3, stop_when=lambda e: any(
        x.get("d", {}).get("offs", {}).get("0") == 12 for x in e if x.get("t") == "ho"))
    sse_p.close()
    check("!HO <id> <angle> 写回同一路 (id 与角度都没被吃掉)",
          any(e.get("d", {}).get("offs", {}).get("0") == 12
              for e in evs if e.get("t") == "ho"),
          str([e for e in evs if e.get("t") == "ho"][:2]))
    check("[HO] 行不进日志面板 (18 行一条, 每轮询会把日志冲掉)",
          not any(e.get("t") == "raw" and wc.RE_HO.match(e.get("line", ""))
                  for e in evs),
          str([e.get("line") for e in evs if e.get("t") == "raw"][:3]))

    # ---- 端口页: !PORTS 全量回读 / !GPO 写空闲脚电平 ----
    # 与 !HOS 同一套路: 服务端仍是 --no-poll, 所以端口帧只可能来自下面这几条 POST
    sse_p = sse_open()
    time.sleep(0.3)
    r = json.loads(http_post("/cmd", "!PORTS"))
    check("POST /cmd 下发 !PORTS", r.get("ok") is True, str(r))
    evs = sse_drain(sse_p, 3, stop_when=lambda e: any(
        x.get("t") == "ports" and {"uart0", "motors", "gpio", "fixed"} <= set(x.get("d", {}))
        for x in e))
    pt = next((e["d"] for e in reversed(evs)
               if e.get("t") == "ports" and "fixed" in e.get("d", {})), None)
    check("SSE 收到 !PORTS 的七节端口帧", pt is not None, str(evs[:4]))
    if pt:
        check("端口帧: UART0 与电机节",
              pt.get("uart0") == {"en": 1, "baud": 115200}
              and pt.get("motors") == {"en": 1, "lv1": 0, "lv2": 1}, str(sorted(pt)))
        check("端口帧: 六路足端电平 (低=触地, 不下判定) 与固定脚电平",
              pt.get("foot", {}).get("s1") == 0 and pt.get("foot", {}).get("s0") == 1
              and pt.get("fixed", {}).get("ledg") == 1,
              str(pt.get("foot")) + " " + str(pt.get("fixed")))
        check("端口帧: 三路空闲脚的归属与电平成对出现",
              pt.get("gpio") == {"g23fn": 1, "g23lv": 1, "g24fn": 0, "g24lv": 0,
                                 "g29fn": 0, "g29lv": 1}, str(pt.get("gpio")))

    # !GPO: 写完电平固件立刻回打 gpio 行 (不用等下一次轮询) —— 前端就靠这条
    # 即时刷新, 否则勾选框会等到 2 秒后的轮询才反映出来
    http_post("/cmd", "!GPO 23 0")
    evs = sse_drain(sse_p, 3, stop_when=lambda e: any(
        x.get("t") == "ports" and x.get("d", {}).get("gpio", {}).get("g23lv") == 0
        for x in e))
    check("!GPO <pin> <0|1> 写电平并即时回打 gpio 行",
          any(e.get("d", {}).get("gpio", {}).get("g23lv") == 0
              for e in evs if e.get("t") == "ports"),
          str([e for e in evs if e.get("t") == "ports"][:2]))
    check("[PORTS] 行不进日志面板 (7 行一条, 每 2 秒会把日志冲掉)",
          not any(e.get("t") == "raw" and RE_PORTS.match(e.get("line", ""))
                  for e in evs),
          str([e.get("line") for e in evs if e.get("t") == "raw"][:3]))

    # 输入态的空闲脚不给写电平 (页面上复选框灰掉了, 但命令通道不设权限 ——
    # 手敲 /cmd 也能到, 所以固件必须拒绝, 而且得说明白不肯在哪)
    http_post("/cmd", "!GPO 24 1")
    evs = sse_drain(sse_p, 2, stop_when=lambda e: any(
        "GPO Refuse" in x.get("line", "") for x in e if x.get("t") == "raw"))
    sse_p.close()
    check("输入态的空闲脚拒绝写电平 (拒绝理由进日志, 不是静默)",
          any("GPO Refuse" in e.get("line", "")
              for e in evs if e.get("t") == "raw"),
          str([e.get("line") for e in evs if e.get("t") == "raw"][:3]))

    r = json.loads(http_post("/poll", "off"))
    check("POST /poll 暂停轮询", r.get("paused") is True, str(r))

    # ---- 设备/烧录端点 (只探只读与拒绝路径, 绝不真烧) ----
    dev = json.loads(http_get("/device"))
    check("GET /device 报设备/固件/picotool",
          {"bridge", "devices", "firmware", "picotool"} <= set(dev), str(sorted(dev)))
    check("GET /device 的 bridge 与 /state 一致",
          dev["bridge"] is json.loads(http_get("/state"))["bridge"])
    # 开始页 (Betaflight 语义): 默认只显示开始页, 整块调试界面藏在 #app 里;
    # 进出调试页由服务端的连接态驱动 (conn/dev/link 事件), 页面自己不做主
    check("页面默认是开始页 (调试页整块初始隐藏)",
          'id="welcome"' in raw_page and 'id="app" hidden' in raw_page)
    check("开始页三行状态: 网页服务 / 串口桥接 / 机器人设备",
          all(f'id="{i}"' in raw_page for i in ("wel-web", "wel-bridge", "wel-dev")))
    check("桥接没跑时设备行说「未知」而不是拿旧快照说在线",
          "未知 · 桥接未运行" in raw_page)
    check("开始页有连接按钮, 且由「桥接+设备都在线」使能",
          'id="connectbtn"' in raw_page
          and "const can = BRIDGE_UP && DEVS.up;" in raw_page
          and '$("connectbtn").disabled = !can;' in raw_page)
    check("开始页能看固件版本 (无需连接)",
          'id="wel-ver"' in raw_page and 'case "ver":' in raw_page)
    check("开始页有 USB/可烧录列表与烧录入口",
          'id="wel-usb"' in raw_page and 'id="wel-fw"' in raw_page
          and 'id="devbtn"' in raw_page and "烧录固件" in raw_page
          and 'id="flashbtn"' in raw_page)
    check("开始页有桥接日志区 (未连接也能看启动横幅)",
          'id="welcome-log"' in raw_page and "welLog(ev.line)" in raw_page
          and "welLog(ev.msg)" in raw_page)
    check("调试页有「断开连接」按钮", 'id="discbtn"' in raw_page
          and "断开连接" in raw_page)
    check("拔线/掉线事件把调试页送回开始页",
          'case "dev":' in raw_page and 'case "conn":' in raw_page
          and 'leaveDebug("设备已断开")' in raw_page)

    for bad, why in [("../../etc/passwd", "路径逃逸"), ("/etc/passwd", "非 uf2"),
                     ("nope.uf2", "文件不存在")]:
        r = json.loads(http_post("/flash", json.dumps({"uf2": bad})))
        check(f"POST /flash 拒绝{why}", r.get("ok") is False and r.get("err"), str(r))

    # 桥接转发时延: 另开一路 SSE 量"固件输出 → 浏览器"的到达节拍
    sse_lat = sse_open()
    time.sleep(0.3)
    test_bridge_latency(sse_lat)
    sse_lat.close()

    # 参数页另开一路 SSE (上一路已关闭) —— 连接时服务器会下发一次 !CFG
    sse2 = sse_open()
    time.sleep(0.3)
    test_param_e2e(sse2)
    sse2.close()

    test_connect_gate()

    test_direct_contract()
    test_pages_workflow()
    test_release_workflow()
    test_release_body_step()
    test_fw_version()

    print("\n[5] 页面脚本 (无头浏览器)")
    test_page_js()
    test_direct_engine()
    test_file_url_mode()

    print(f"\n{len(passed)} 项通过, {len(failed)} 项失败")
    if failed:
        for f in failed:
            print("  FAILED: " + f)
        print("日志: /tmp/wc_socat.log /tmp/wc_fake.log /tmp/wc_console.log /tmp/wc_server.log")
    return 1 if failed else 0


if __name__ == "__main__":
    try:
        code = main()
    finally:
        cleanup()
    sys.exit(code)

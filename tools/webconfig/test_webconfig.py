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
    "calib: sys=3 gyr=3 acc=3 mag=0 (fully)",
    "INT_STA=0x80 (bit7=BSX_DRDY)",
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

# 固件版本 (hexapod_hal_pico.c: hal_fw_version_print): 开机横幅 + !VER 共用。
# 内容是 git describe 的结果, 所以样本取个"长得像"的字符串即可。
S_VER = [
    "[VER] Hexapod v0.3.0-16-g8f5b04f-dirty",
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
BNO_C = os.path.join(ROOT, "pico", "Src", "bno055.c")
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
    page = "file://" + probe_path
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

    print("\n[5] 页面脚本 (无头浏览器)")
    test_page_js()

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

#!/usr/bin/env python3
"""
六足机器人网页 Configurator —— Phase 2 (状态面板 + 参数编辑 + 命令行)

架构: 串口逻辑完全复用 serial_console.py, 本服务不碰串口

    Pico ──USB CDC──> serial_console.py ──TCP:7100──> 本服务 ──SSE──> 浏览器
                      (串口守护: 自动重连/广播)         <──POST──┘

用法:
    # 终端 1: 串口守护 (headless 运行即可, 无需交互)
    python3 tools/serial_console.py
    # 终端 2: 网页服务
    python3 tools/webconfig/server.py --port 8080
    # 浏览器打开 http://127.0.0.1:8080
    # (浏览器在 Windows 上时, 用 VSCode 的端口转发把 8080 转过去)

上行入口 (信任级别递增):
    POST /poll       暂停/恢复自动轮询
    POST /connect    会话连接: 要求桥接 + 设备都在线, 之后才开始轮询/拉参数表
    POST /disconnect 断开连接 (回开始页; 设备通常仍在线, 可再连)
    POST /param      结构化参数指令 (set/reset/resetall/save/dump), 名字与值都校验
    POST /cmd        任意串口命令原文 —— 与直接敲串口等价, 不做任何过滤

设计取舍 (改架构前请先读这段):
  - 不用 WebSocket: 纯标准库实现需手写握手+分帧, 否则要引入 pip 依赖;
    本场景遥测是下行高频、命令是上行低频, SSE(下行, 浏览器自带断线重连)
    + POST(上行) 已足够。真需要低延迟双向时再换。
  - 不自己开串口: serial_console.py 已解决端口扫描/断线重连/多客户端广播/
    慢客户端剔除/命令排队, 并有 test_bridge.py 回归测试 —— 重复实现只会
    多一份要维护的 bug。代价是轮询命令会同时出现在串口终端里 (共享总线)。
  - 默认只绑 127.0.0.1: 网页能驱动真实机器人, 暴露到网络等于把机器人
    交出去。远程访问走 SSH/VSCode 端口转发。
  - 参数按名字传输 (不是索引): 固件升级增删参数不会让前端错位。旧名字在
    固件侧查不到时返回 FAIL, 前端只提示不改值。

连接语义 (Betaflight 式): 页面打开先是「开始页」—— 只看得到硬件是否在线、固件
版本、可烧录固件, 改不了任何参数; 设备在线时点「连接」才进调试页。连接态按
"桥接在线 + 设备在线"判定, 设备掉线或桥接断开即作废, 浏览器据此跳回开始页。

轮询仅在"有浏览器 + 已连接 + 未暂停"时进行, 空闲时不打扰串口。
参数表只在「点连接 / 桥接重连 / 用户主动刷新」时拉取 —— 它不会自己变, 不必进轮询。
"""

import argparse
import copy
import json
import queue
import re
import select
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote

import flasher

HERE = Path(__file__).resolve().parent

# ==================== 遥测解析 ====================
#
# 固件输出是给人看的文本, Phase 1 按行正则解析。
# Phase 2 计划在固件加 `!J 1` 切换 JSON 行输出, 届时本段可整体替换
# (见 STATUS.md「网页 Configurator」路线图)。
#
# 解析策略: 逐行匹配 → 更新对应子系统状态 → 推一帧完整快照给前端。
# 未匹配的行原样进日志面板, 所以固件加新输出不会破坏这里, 只是不显示。

RE_BATT_RAW = re.compile(r"^Raw: avg=(\d+) min=(\d+) max=(\d+) \(spread (\d+) counts = (\d+) mV\)")
RE_BATT_MV = re.compile(r"^Pin: (\d+) mV.*Battery: (\d+) mV")
RE_BATT_LIM = re.compile(r"^Limits: (\d+) ~ (\d+) mV . (IN RANGE|OUT OF RANGE)")
RE_BATT_EN = re.compile(r"^Protection: BATTERY_CHECK_ENABLED=(\d)")
# 三态判定 (固件 hexapod_hal_pico.c: hal_battery_state → !BATT 的 State: 行):
#   ABSENT = 未接电池 (USB 供电), 前端不该当成故障显示
RE_BATT_STATE = re.compile(r"^State: (OK|ABSENT|FAULT)\b")

# 每次 !I2C / !I2C q 的起始行 —— 用它清掉上一轮的设备/扫描列表,
# 否则设备掉线后旧条目会一直留在卡片上。
RE_I2C_CHECK = re.compile(r"^=== I2C Bus Check\b")
RE_I2C_PWR = re.compile(r"^Servo power: GP10\(left\)=(\d)\s+GP11\(right\)=(\d)")
RE_I2C_PCA = re.compile(r"^PCA9685 0x([0-9A-F]{2}) \((\w+)\s*legs\): (DETECTED|NOT FOUND)")
RE_I2C_BNO = re.compile(r"^BNO055 0x([0-9A-F]{2}): (DETECTED|NOT FOUND|present)")
RE_I2C_SCAN = re.compile(r"^\s*Device found at 0x([0-9A-F]{2})")
RE_I2C_IDLE = re.compile(r"^Bus idle: SDA\(GP14\)=(\d) SCL\(GP15\)=(\d)")

RE_IMU_AVAIL = re.compile(r"^available: (YES|NO)\s+addr: 0x([0-9A-F]{2})")
RE_IMU_CALIB = re.compile(r"^calib: sys=(\d+) gyr=(\d+) acc=(\d+) mag=(\d+)(.*)$")
RE_IMU_LAST = re.compile(r"^last: roll=(-?\d+) pitch=(-?\d+) yaw=(-?\d+).*valid=(\d)")

RE_SERVO_BOARD = re.compile(r"^Board 0x([0-9A-F]{2}) \((\w+)\): (.*)$")
RE_SERVO_ITEM = re.compile(r"\[(\d+)\]:(-?\d+)")

# 遥控通道遥测 (固件 hal_debug_print_channel_telemetry, 5Hz 自由推送, 无需命令):
#   [CH] m=crsf link=1 fc=123456 c0=992 c1=1500 ... c15=1811
#   [CH] m=ps2 con=1 btns=65535 lx=128 ly=130 rx=127 ry=126 fc=987
# 通道值 172~1811 中位 992; CRSF 帧率由前端按 fc 差值算, 固件不报。
RE_CH_CRSF = re.compile(r"^\[CH\] m=crsf link=(\d) fc=(\d+) (.*)$")
RE_CH_PS2 = re.compile(
    r"^\[CH\] m=ps2 con=(\d) btns=(\d+) lx=(\d+) ly=(\d+) rx=(\d+) ry=(\d+) fc=(\d+)$")

# CRSF 通道数 (11bit 打包固定 16 个) 与中位值, 缺项回填用
CRSF_CH_COUNT = 16
CRSF_CH_MID = 992

# 预留外设状态 (固件 hal_pico.c: periph_status_print / !PERIPH):
#   [PER] motors: m1=0 m2=0
#   [PER] leds: g=0 r=0 hb=1 alarm=1
#   [PER] buzzer: freq=0 ms=0
#   [PER] uart0: en=0 baud=115200 rx=0 tx=0 lines=0
#   [PER] ext: mode=0 found=0 a0=0 a1=0
#   [PER] pwm: 0=0 1=0 ... 13=0
#
# ⚠️ [PER] 这个前缀同时也是 PCA9685 周期校准 (!PER / !PERQ) 的输出前缀,
#    那些行的格式是 "[PER] Board 0 (0x40, left legs): period=9500 us" 之类。
#    所以节名必须走白名单, 不能写成 (\w+): 否则 !PER 的输出会当成外设状态
#    塞进前端 (而且字段名对不上, 表现为卡片上冒出莫名其妙的数字)。
RE_PER = re.compile(r"^\[PER\] (motors|leds|buzzer|uart0|ext|pwm): (.*)$")

# 外部 UART0 的收发回显 (固件 hal_pico.c: cmd_uart0_text / uart0_rx_poll):
#   [U0TX] sent 5 bytes: hello
#   [U0] hello
RE_U0TX = re.compile(r"^\[U0TX\] sent (\d+) bytes: (.*)$")
RE_U0 = re.compile(r"^\[U0\] (.*)$")
U0_RX_RING = 20          # 外设页只显示最近这些行

# 运行时参数 (!CFG 输出)。固件侧格式见 hexapod_params.c: params_print_one():
#   [P] name=travel_fwd_mm val=150 min=20 max=250 def=150 unit=mm grp=motion chg=0
# 只认带 name=/val= 的行; FAIL/WARN/=== 等提示行走原样日志。
RE_PARAM_KV = re.compile(r"(\w+)=(\S+)")
# 长度上限必须与固件 STORE_PARAM_NAME_LEN-1 一致 (当前 19):
# 放宽只是让非法名字走到固件去吃 FAIL, 收紧则会拒掉合法参数。
PARAM_NAME_OK = re.compile(r"^[a-z][a-z0-9_]{0,18}$")

# 固件版本 (开机横幅 + !VER 命令共用一行):
#   [VER] Hexapod v0.3.0-16-g8f5b04f-dirty
RE_VER = re.compile(r"^\[VER\] (.*)$")

# 设备在线状态 —— 由 serial_console.py 发布 (桥接连着 ≠ 机器人在线):
#   [TCP] dev on /dev/ttyACM0    设备已打开
#   [TCP] dev busy /dev/ttyACM0  端口在但打不开 (多为被别的程序占用)
#   [TCP] dev off                设备不在
RE_DEV = re.compile(r"^\[TCP\] dev (on|off|busy)(?: (\S+))?$")


class Telemetry:
    """各子系统的最新状态。每次解析更新后整体推给前端 (前端只渲染最后一份)。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.data = {
            "batt": {},
            "i2c": {},
            "imu": {},
            "servo": {},
            "ch": {},
            "per": {},
            "params": {"list": {}, "count": 0, "changed": 0},
            "ver": {},
        }

    def update(self, key, fields):
        """就地合并字段并返回该子系统的完整快照 (供广播)"""
        with self.lock:
            self.data.setdefault(key, {}).update(fields)
            return dict(self.data[key])

    def snapshot(self):
        with self.lock:
            return {k: copy.deepcopy(v) for k, v in self.data.items() if v}


def parse_param_line(line):
    """
    解析一行 [P] 参数输出 → {name, val, min, max, def, unit, grp, chg} 或 None。
    非参数行 (=== 分隔 / FAIL / WARN 提示) 返回 None, 走原样日志。
    """
    if not line.startswith("[P] ") or "name=" not in line:
        return None
    kv = dict(RE_PARAM_KV.findall(line))
    try:
        return {
            "name": kv["name"],
            "val": int(kv["val"]),
            "min": int(kv["min"]),
            "max": int(kv["max"]),
            "def": int(kv["def"]),
            "unit": kv.get("unit", ""),     # 空 unit 会让 \S+ 失配, 故用 get
            "grp": kv.get("grp", "?"),
            "chg": kv.get("chg") == "1",
        }
    except (KeyError, ValueError):
        return None


def parse_line(telem, line):
    """
    解析一行固件输出 → 返回 (子系统名, 快照) 或 None。
    只做纯解析, 不做 IO —— 便于单独测试。
    """
    p = parse_param_line(line)
    if p:
        with telem.lock:
            st = telem.data["params"]
            st["list"][p["name"]] = p
            # 以固件回显为准重新统计, 不靠本地加减
            st["count"] = len(st["list"])
            st["changed"] = sum(1 for x in st["list"].values() if x["chg"])
            snap = copy.deepcopy(st)
        return "params", snap

    m = RE_VER.match(line)
    if m:
        return "ver", telem.update("ver", {"text": m.group(1)})

    m = RE_BATT_RAW.match(line)
    if m:
        return "batt", telem.update("batt", {
            "avg": int(m.group(1)), "min": int(m.group(2)), "max": int(m.group(3)),
            "spread_counts": int(m.group(4)), "spread_mv": int(m.group(5)),
        })

    m = RE_BATT_MV.match(line)
    if m:
        return "batt", telem.update("batt", {
            "pin_mv": int(m.group(1)), "batt_mv": int(m.group(2)),
        })

    m = RE_BATT_LIM.match(line)
    if m:
        return "batt", telem.update("batt", {
            "lo_mv": int(m.group(1)), "hi_mv": int(m.group(2)),
            "in_range": m.group(3) == "IN RANGE",
        })

    m = RE_BATT_EN.match(line)
    if m:
        return "batt", telem.update("batt", {"enabled": int(m.group(1))})

    m = RE_BATT_STATE.match(line)
    if m:
        return "batt", telem.update("batt", {"state": m.group(1)})

    m = RE_I2C_CHECK.match(line)
    if m:
        # 只有全扫描那次才作废扫描列表 —— 否则手动 !I2C 的结果会在 3s 后
        # 被常规的 !I2C q 轮询抹掉, 用户根本来不及看。
        full = "quick" not in line
        with telem.lock:
            old = telem.data.get("i2c", {})
            keep = {} if full or "scan" not in old else {"scan": old["scan"]}
            telem.data["i2c"] = keep
        # 显式 reset: 空对象过不了前端 Object.assign, 得让前端自己清
        return "i2c", {"reset": True, "reset_scan": full}

    m = RE_I2C_PWR.match(line)
    if m:
        return "i2c", telem.update("i2c", {
            "servo_power": {"left": int(m.group(1)), "right": int(m.group(2))},
        })

    m = RE_I2C_PCA.match(line)
    if m:
        with telem.lock:
            devs = telem.data.setdefault("i2c", {}).setdefault("pca", [])
            entry = {"addr": int(m.group(1), 16), "side": m.group(2),
                     "ok": m.group(3) == "DETECTED"}
            devs[:] = [d for d in devs if d["addr"] != entry["addr"]] + [entry]
            snap = dict(telem.data["i2c"])
        return "i2c", snap

    m = RE_I2C_BNO.match(line)
    if m:
        with telem.lock:
            devs = telem.data.setdefault("i2c", {}).setdefault("bno", [])
            entry = {"addr": int(m.group(1), 16), "ok": m.group(2) == "DETECTED"}
            devs[:] = [d for d in devs if d["addr"] != entry["addr"]] + [entry]
            snap = dict(telem.data["i2c"])
        return "i2c", snap

    m = RE_I2C_SCAN.match(line)
    if m:
        with telem.lock:
            found = telem.data.setdefault("i2c", {}).setdefault("scan", [])
            a = int(m.group(1), 16)
            if a not in found:
                found.append(a)
            snap = dict(telem.data["i2c"])
        return "i2c", snap

    m = RE_I2C_IDLE.match(line)
    if m:
        return "i2c", telem.update("i2c", {
            "idle": {"sda": int(m.group(1)), "scl": int(m.group(2))},
        })

    m = RE_IMU_AVAIL.match(line)
    if m:
        return "imu", telem.update("imu", {
            "available": m.group(1) == "YES", "addr": int(m.group(2), 16),
        })

    m = RE_IMU_CALIB.match(line)
    if m:
        return "imu", telem.update("imu", {
            "calib": {"sys": int(m.group(1)), "gyr": int(m.group(2)),
                      "acc": int(m.group(3)), "mag": int(m.group(4))},
            "fully": "fully" in m.group(5),
        })

    m = RE_IMU_LAST.match(line)
    if m:
        return "imu", telem.update("imu", {
            "roll": int(m.group(1)), "pitch": int(m.group(2)),
            "yaw": int(m.group(3)), "valid": int(m.group(4)),
        })

    m = RE_SERVO_BOARD.match(line)
    if m:
        angles = {int(i): int(a) for i, a in RE_SERVO_ITEM.findall(m.group(3))}
        if angles:
            with telem.lock:
                boards = telem.data.setdefault("servo", {}).setdefault("boards", {})
                boards[m.group(1)] = {"side": m.group(2), "angles": angles}
                snap = dict(telem.data["servo"])
            return "servo", snap

    # 通道遥测: 每次都是完整一帧 (16 通道全报), 所以整份覆盖而不是合并 ——
    # 否则切模式后残留的另一种模式的字段会混在一起 (比如 ps2 的 lx 留在 crsf 帧里)
    m = RE_CH_CRSF.match(line)
    if m:
        kv = dict(RE_PARAM_KV.findall(m.group(3)))
        chans = [int(kv.get(f"c{i}", CRSF_CH_MID)) for i in range(CRSF_CH_COUNT)]
        with telem.lock:
            telem.data["ch"] = {
                "mode": "crsf", "link": int(m.group(1)) == 1,
                "fc": int(m.group(2)), "ch": chans,
            }
            snap = dict(telem.data["ch"])
        return "ch", snap

    m = RE_CH_PS2.match(line)
    if m:
        with telem.lock:
            telem.data["ch"] = {
                "mode": "ps2", "connected": int(m.group(1)) == 1,
                "btns": int(m.group(2)),
                "lx": int(m.group(3)), "ly": int(m.group(4)),
                "rx": int(m.group(5)), "ry": int(m.group(6)),
                "fc": int(m.group(7)),
            }
            snap = dict(telem.data["ch"])
        return "ch", snap

    # 外设状态: 每节独立合并 (固件每次都是全量报 6 行, 但一行一行地到,
    # 分节存才能让前端在只收到一半时也有东西可渲染)
    m = RE_PER.match(line)
    if m:
        fields = {}
        for k, v in RE_PARAM_KV.findall(m.group(2)):
            try:
                fields[k] = int(v)
            except ValueError:
                fields[k] = v
        return "per", telem.update("per", {m.group(1): fields})

    m = RE_U0TX.match(line)
    if m:
        return "per", telem.update("per", {"u0_tx": m.group(2),
                                           "u0_tx_bytes": int(m.group(1))})

    m = RE_U0.match(line)
    if m:
        with telem.lock:
            per = telem.data.setdefault("per", {})
            ring = per.setdefault("u0_rx", [])
            ring.append(m.group(1))
            del ring[:-U0_RX_RING]
            snap = dict(per)
        return "per", snap

    return None


# ==================== 桥接客户端 ====================

class BridgeClient:
    """
    连接 serial_console.py 的 TCP 桥, 断线自动重连。

    on_line(line)  每收到一行固件输出调用一次 (桥接线程上下文)
    on_state(bool, str)  连接状态变化 (桥接线程上下文)
    """

    def __init__(self, host, port, on_line, on_state):
        self.host, self.port = host, port
        self.on_line, self.on_state = on_line, on_state
        self.sock = None
        self.lock = threading.Lock()
        self.connected = False
        self._stop = False

    def run(self):
        backoff = 1.0
        while not self._stop:
            try:
                s = socket.create_connection((self.host, self.port), timeout=5)
                s.settimeout(None)
                with self.lock:
                    self.sock = s
                self.connected = True
                self.on_state(True, f"已连接桥接 {self.host}:{self.port}")
                backoff = 1.0
                buf = b""
                while not self._stop:
                    chunk = s.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        line = raw.decode("utf-8", "replace").rstrip("\r\n")
                        if line.strip():        # 保留行首缩进 (!I2C 扫描结果靠缩进对齐)
                            self.on_line(line)
            except OSError as e:
                self.on_state(False, f"桥接断开: {e}")
            finally:
                self.connected = False
                with self.lock:
                    if self.sock:
                        try:
                            self.sock.close()
                        except OSError:
                            pass
                    self.sock = None
                if not self._stop:
                    self.on_state(False, f"{backoff:.0f}s 后重试…")
                    time.sleep(backoff)
                    backoff = min(backoff * 1.5, 10.0)
        self.on_state(False, "已停止")

    def send(self, cmd):
        """下发一条命令。未连接时返回 False (不排队 —— 见下)"""
        with self.lock:
            s = self.sock
        if not s:
            return False
        try:
            s.sendall(cmd.encode() + b"\n")
            return True
        except OSError:
            return False

    def stop(self):
        self._stop = True
        with self.lock:
            if self.sock:
                try:
                    self.sock.close()
                except OSError:
                    pass


# ==================== HTTP / SSE 服务 ====================

_subscribers = set()          # {queue.Queue}
_subs_lock = threading.Lock()
_poll_paused = threading.Event()
_stats = {"lines": 0, "started": time.time()}

# ---- 设备在线状态 (由 serial_console 的 [TCP] dev 行驱动) ----
# ⚠️ 桥接连着 ≠ 机器人在线。而且设备不在时 serial_console 会把命令**排队**等
#    设备回来重放 —— 所以"连接"必须同时要求桥接在线 + 设备在线, 否则拔插一次
#    就会等来一堆积压命令。
_dev_lock = threading.Lock()
_dev = {"up": False, "port": None, "busy": False}

# ---- 会话连接状态 (开始页点「连接」后为真) ----
# 一台机器人 + 一条上游串口 = 全局一个连接态, 多浏览器共享: 轮询与参数表只在
# 连接后下发; 任一浏览器断开则都回开始页 (简单, 且不会互相打架)。
_connected = threading.Event()


def dev_state():
    with _dev_lock:
        return dict(_dev)


def set_dev(state, port=None):
    """设备在线状态变化 (来自 serial_console 的 [TCP] dev 行)"""
    up = (state == "on")
    with _dev_lock:
        _dev.update(up=up, port=port, busy=(state == "busy"))
        snap = dict(_dev)
    broadcast({"t": "dev", **snap})
    if not up and _connected.is_set():
        # 设备没了: 连接态作废, 轮询立即停 —— 否则命令会被排队, 重插时洪水
        _connected.clear()
        broadcast({"t": "conn", "on": False, "msg": "设备已断开"})
    return up


def broadcast(event):
    """向所有浏览器非阻塞广播一帧。慢客户端直接丢弃该帧, 绝不阻塞桥接线程。"""
    payload = json.dumps(event, ensure_ascii=False)
    with _subs_lock:
        subs = list(_subscribers)
    for q in subs:
        try:
            q.put_nowait(payload)
        except queue.Full:
            pass


# ==================== 固件烧录 ====================
#
# 设备检测与 picotool 调用都在 flasher.py; 这里只做"后台跑 + 进度推给浏览器"。
# picotool 要几十秒, 绝不能占住 HTTP 请求线程, 所以走后台线程 + SSE。

_flash_lock = threading.Lock()
_flash = {"state": "idle", "uf2": None, "err": None, "log": []}
FLASH_LOG_MAX = 200


def flash_state():
    with _flash_lock:
        return {"state": _flash["state"], "uf2": _flash["uf2"],
                "err": _flash["err"], "log": list(_flash["log"])}


def _flash_emit():
    broadcast({"t": "flash", "d": flash_state()})


def _flash_update(state=None, err=None, uf2=None, log_line=None, reset_log=False):
    with _flash_lock:
        if reset_log:
            _flash["log"] = []
        if state is not None:
            _flash["state"] = state
        if err is not None:
            _flash["err"] = err
        if uf2 is not None:
            _flash["uf2"] = uf2
        if log_line:
            # picotool 的进度条用 \r 原地刷新, 拆成行后会有上百条 —— 原地替换
            # 上一条, 否则 200 行上限全被 "Loading into Flash: [===] 37%" 占满
            if (log_line.startswith("Loading into Flash")
                    and _flash["log"] and _flash["log"][-1].startswith("Loading into Flash")):
                _flash["log"][-1] = log_line
            else:
                _flash["log"].append(log_line)
                del _flash["log"][:-FLASH_LOG_MAX]


def _flash_worker(uf2):
    _flash_update(state="running", err="", uf2=uf2.name, reset_log=True)
    _flash_emit()
    try:
        flasher.flash(uf2, on_line=lambda l: (_flash_update(log_line=l), _flash_emit()))
        _flash_update(state="ok")
    except (flasher.FlashError, ValueError, OSError) as e:
        _flash_update(state="err", err=str(e), log_line=f"[!] {e}")
    _flash_emit()


def handle_flash(body):
    """POST /flash  {"uf2": "hexapod_pico.uf2"}  —— 名字须在白名单目录内"""
    with _flash_lock:
        if _flash["state"] == "running":
            return {"ok": False, "err": "已有烧录在进行中"}
    try:
        data = json.loads(body or "{}")
    except ValueError:
        return {"ok": False, "err": "请求体不是 JSON"}
    try:
        path = flasher.resolve_uf2(str(data.get("uf2") or ""))
    except ValueError as e:
        return {"ok": False, "err": str(e)}
    if not flasher.find_picotool():
        return {"ok": False, "err": f"找不到 picotool (可用 {flasher.PICOTOOL_ENV} 指定)"}
    threading.Thread(target=_flash_worker, args=(path,), daemon=True,
                     name="flash").start()
    return {"ok": True, "uf2": path.name}


def device_info():
    """设备/固件现状 —— 开始页与烧录面板按这个渲染"""
    return {
        "bridge": bool(BRIDGE and BRIDGE.connected),
        "bridge_host": BRIDGE.host if BRIDGE else None,
        "bridge_port": BRIDGE.port if BRIDGE else None,
        "picotool": flasher.find_picotool(),
        "devices": flasher.usb_devices(),
        "firmware": flasher.list_firmware(),
        "dev": dev_state(),
        "connected": _connected.is_set(),
    }


def handle_connect():
    """
    开始页点「连接」: 桥接与设备**都在线**才放行, 并拉一次参数表。

    这是 UI 层的门 (开始页在连接前不显示任何参数/命令), 不是权限层 ——
    HTTP 只绑本机, 与 POST /cmd 的信任模型一致 (能开本机端口的人本来就能
    直接发命令)。加这道门是为了不让"没插机器人"时误改参数、误发命令。
    """
    if not (BRIDGE and BRIDGE.connected):
        return {"ok": False, "err": "串口桥接未连接 (serial_console.py 没在跑?)"}
    if not dev_state()["up"]:
        return {"ok": False, "err": "机器人未在线"}
    _connected.set()
    BRIDGE.send("!CFG")           # 参数表随连接拉一次, 调试页首屏要用
    broadcast({"t": "conn", "on": True})
    return {"ok": True, **dev_state()}


def handle_disconnect():
    """调试页点「断开连接」: 停轮询, 回开始页 (设备仍在线, 可再连)"""
    _connected.clear()
    broadcast({"t": "conn", "on": False, "msg": "已断开连接"})
    return {"ok": True}


class Handler(BaseHTTPRequestHandler):
    server_version = "hexapod-webconfig/0.1"

    def log_message(self, fmt, *args):
        pass    # 静音访问日志 (SSE 连接长期存在, 刷屏无意义)

    def _send_file(self, path, ctype):
        try:
            body = path.read_bytes()
        except OSError:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send_file(HERE / "index.html", "text/html; charset=utf-8")
        elif self.path == "/events":
            self._serve_sse()
        elif self.path == "/device":
            self._send_json(device_info())
        elif self.path == "/state":
            self._send_json({"telemetry": TELEM.snapshot(),
                             "bridge": BRIDGE.connected if BRIDGE else False,
                             "paused": _poll_paused.is_set(),
                             "flash": flash_state(),
                             "stats": _stats,
                             "dev": dev_state(),
                             "connected": _connected.is_set()})
        else:
            self.send_error(404)

    def _client_gone(self):
        """
        浏览器是否已经关掉这个 SSE 连接。

        SSE 是单向下发, 对端关页时不会通知我们 —— 只有 socket 变成可读且读到
        EOF 才知道。不主动查的话, 得等下面 keepalive 写失败 (最长 15 秒) 才发
        现, 连接态与轮询就会多挂十几秒 ("没人连却在轮询")。
        """
        try:
            if not select.select([self.connection], [], [], 0)[0]:
                return False
            return not self.connection.recv(1)      # b"" = 对端发了 FIN
        except OSError:
            return True

    def _serve_sse(self):
        """SSE: 长期保持的响应, 每条事件一个 data: 帧"""
        q = queue.Queue(maxsize=256)
        with _subs_lock:
            first = not _subscribers
            _subscribers.add(q)
        # 首个浏览器只要设备在线就补一次版本 —— 开始页在"连接"之前也要显示固件版本
        if first and BRIDGE and BRIDGE.connected and dev_state()["up"]:
            BRIDGE.send("!VER")
        try:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()

            # 开场: 当前快照 + 状态, 让刷新页面后不是空白
            hello = {"t": "hello", "bridge": BRIDGE.connected if BRIDGE else False,
                     "paused": _poll_paused.is_set(),
                     "connected": _connected.is_set(), "dev": dev_state()}
            self.wfile.write(f"data: {json.dumps(hello, ensure_ascii=False)}\n\n".encode())
            for key, snap in TELEM.snapshot().items():
                ev = json.dumps({"t": key, "d": snap}, ensure_ascii=False)
                self.wfile.write(f"data: {ev}\n\n".encode())
            self.wfile.flush()

            last_write = time.monotonic()
            while True:
                try:
                    payload = q.get(timeout=0.5)
                except queue.Empty:
                    payload = None
                if self._client_gone():
                    break
                if payload is None:
                    if time.monotonic() - last_write >= 15:
                        self.wfile.write(b": keepalive\n\n")   # SSE 注释帧, 保活
                        self.wfile.flush()
                        last_write = time.monotonic()
                    continue
                self.wfile.write(f"data: {payload}\n\n".encode())
                self.wfile.flush()
                last_write = time.monotonic()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass          # 浏览器关页/断网, 正常路径
        finally:
            with _subs_lock:
                _subscribers.discard(q)
                last = not _subscribers
            if last:
                # 最后一个浏览器走了: 没有调试页了, 连接态作废 (轮询随之停)
                _connected.clear()

    MAX_UPLOAD = 8 * 1024 * 1024

    def _handle_upload(self, raw):
        """固件上传: 二进制必须走原始字节 (下面的文本入口会破坏 uf2)"""
        name = unquote(self.headers.get("X-Filename") or "upload.uf2")
        try:
            path = flasher.save_upload(raw, name)
        except (ValueError, OSError) as e:
            self._send_json({"ok": False, "err": str(e)}, 400)
            return
        self._send_json({"ok": True, "path": str(path), "name": path.name})

    def do_POST(self):
        try:
            n = int(self.headers.get("Content-Length") or 0)
            if n > self.MAX_UPLOAD:
                self._send_json({"ok": False, "err": "请求体过大"}, 413)
                return
            raw = self.rfile.read(n)
        except (ValueError, OSError):
            self._send_json({"ok": False, "err": "bad request"}, 400)
            return

        if self.path == "/flash-upload":
            self._handle_upload(raw)
            return

        body = raw.decode("utf-8", "replace").strip()

        if self.path == "/flash":
            self._send_json(handle_flash(body))
        elif self.path == "/cmd":
            ok = BRIDGE.send(body) if BRIDGE else False
            self._send_json({"ok": ok, "cmd": body,
                             "err": None if ok else "桥接未连接"})
        elif self.path == "/param":
            self._send_json(handle_param(body))
        elif self.path == "/poll":
            if body == "off":
                _poll_paused.set()
            else:
                _poll_paused.clear()
            self._send_json({"ok": True, "paused": _poll_paused.is_set()})
        elif self.path == "/connect":
            self._send_json(handle_connect())
        elif self.path == "/disconnect":
            self._send_json(handle_disconnect())
        else:
            self.send_error(404)


# ==================== 参数写入 ====================

# 前端 POST /param 的指令集 (结构化, 不把任意文本透传给固件 ——
# 想发任意命令请走 /cmd, 两条路径的信任级别不同):
#   set <name> <value>    !CFG <name> <value>
#   reset <name>          !CFGR <name>
#   resetgrp <group>      该组每个参数各发一条 !CFGR <name> (网页单页恢复默认)
#   resetall              !CFGR
#   save                  !CFGW  (存 flash, 需机器人未解锁)
#   dump                  !CFG   (重新读取全部)
GROUP_NAME_OK = re.compile(r"^[a-z]{1,12}$")


def _reset_group(grp):
    """把某一组参数逐个恢复默认。固件没有 !CFGR <组> 语法, 所以由这里按
    参数快照里的 grp 展开成多条单参数命令。"""
    with TELEM.lock:
        names = [n for n, p in TELEM.data["params"]["list"].items()
                 if p.get("grp") == grp]
    if not names:
        return {"ok": False, "err": f"参数快照里没有分组 {grp} (先读取一次?)"}
    if not BRIDGE:
        return {"ok": False, "err": "桥接未连接"}
    for n in names:
        BRIDGE.send(f"!CFGR {n}")
    return {"ok": True, "cmd": f"!CFGR ×{len(names)} ({grp})",
            "n": len(names), "err": None}


def handle_param(body):
    parts = body.split()
    if not parts:
        return {"ok": False, "err": "空指令"}

    op = parts[0]

    if op == "dump":
        cmd = "!CFG"
    elif op == "resetall":
        cmd = "!CFGR"
    elif op == "save":
        cmd = "!CFGW"
    elif op == "resetgrp":
        if len(parts) != 2 or not GROUP_NAME_OK.match(parts[1]):
            return {"ok": False, "err": f"分组名不合法: {parts[1:2]}"}
        return _reset_group(parts[1])
    elif op in ("set", "reset"):
        if len(parts) < 2 or not PARAM_NAME_OK.match(parts[1]):
            return {"ok": False, "err": f"参数名不合法: {parts[1:2]}"}
        if op == "reset":
            cmd = f"!CFGR {parts[1]}"
        else:
            if len(parts) != 3:
                return {"ok": False, "err": "set 需要 <name> <value>"}
            try:
                int(parts[2])
            except ValueError:
                return {"ok": False, "err": f"值必须是整数: {parts[2]}"}
            cmd = f"!CFG {parts[1]} {parts[2]}"
    else:
        return {"ok": False, "err": f"未知指令: {op}"}

    ok = BRIDGE.send(cmd) if BRIDGE else False
    return {"ok": ok, "cmd": cmd, "err": None if ok else "桥接未连接"}


# ==================== 轮询 ====================

def poller_thread(batt_iv, imu_iv, servo_iv, i2c_iv, periph_iv=2.0):
    """
    定时下发只读命令喂仪表盘。仅在"有浏览器 + 已连接 + 未暂停 + 桥接在线"时发,
    其余时候完全不打扰串口 (否则会污染同时在用的串口终端)。

    已连接 (开始页点过「连接」) 是必要条件: 设备不在时 serial_console 会把命令
    排队, 攒下的 !BATT/!A 会在设备回来的瞬间一股脑灌给刚上电的机器人。

    I2C 用 `!I2C q` (仅定向检测) 而非 `!I2C` —— 后者含 128 地址全总线扫描。
    健康总线上两者都是 ~20ms, 但命令处理在固件的 20ms 控制循环内 (= 舵机停更
    时长), 而总线被拉死时每个地址都吃满 5ms 超时, 128 个最坏累计 640ms。
    """
    next_batt = next_imu = next_servo = next_i2c = next_periph = 0.0
    while True:
        time.sleep(0.1)
        with _subs_lock:
            has_client = bool(_subscribers)
        if (not has_client or not _connected.is_set()
                or _poll_paused.is_set() or not (BRIDGE and BRIDGE.connected)):
            continue
        now = time.monotonic()
        if batt_iv > 0 and now >= next_batt:
            BRIDGE.send("!BATT")
            next_batt = now + batt_iv
        if imu_iv > 0 and now >= next_imu:
            BRIDGE.send("!IMU")
            next_imu = now + imu_iv
        if servo_iv > 0 and now >= next_servo:
            BRIDGE.send("!A")
            next_servo = now + servo_iv
        if i2c_iv > 0 and now >= next_i2c:
            BRIDGE.send("!I2C q")
            next_i2c = now + i2c_iv
        # 外设状态只有被问才报 (固件不主动推), 所以外设页要靠这条轮询喂
        if periph_iv > 0 and now >= next_periph:
            BRIDGE.send("!PERIPH")
            next_periph = now + periph_iv


# ==================== 入口 ====================

TELEM = Telemetry()
BRIDGE = None


def main():
    global BRIDGE
    ap = argparse.ArgumentParser(description="六足机器人网页监视台 (Phase 1)")
    ap.add_argument("--host", default="127.0.0.1",
                    help="监听地址 (默认 127.0.0.1; 改 0.0.0.0 等于把机器人暴露到网络)")
    ap.add_argument("--port", type=int, default=8080, help="HTTP 端口 (默认 8080)")
    ap.add_argument("--bridge-host", default="127.0.0.1", help="串口桥接地址")
    ap.add_argument("--bridge-port", type=int, default=7100, help="串口桥接端口")
    ap.add_argument("--batt-interval", type=float, default=2.0, help="!BATT 轮询间隔秒 (0=关)")
    ap.add_argument("--imu-interval", type=float, default=1.0, help="!IMU 轮询间隔秒 (0=关)")
    ap.add_argument("--servo-interval", type=float, default=2.0, help="!A 轮询间隔秒 (0=关)")
    ap.add_argument("--i2c-interval", type=float, default=3.0,
                    help="!I2C q (I2C 总线) 轮询间隔秒 (0=关)")
    ap.add_argument("--periph-interval", type=float, default=2.0,
                    help="!PERIPH (外设状态) 轮询间隔秒 (0=关)")
    ap.add_argument("--no-poll", action="store_true", help="完全禁用自动轮询")
    args = ap.parse_args()

    if args.no_poll:
        args.batt_interval = args.imu_interval = args.servo_interval = 0
        args.i2c_interval = args.periph_interval = 0

    def on_line(line):
        _stats["lines"] += 1
        m = RE_DEV.match(line)
        if m:
            set_dev(m.group(1), m.group(2))
            broadcast({"t": "bridge", "msg": line})   # 仍进日志面板
            return
        if line.startswith("[TCP]"):
            broadcast({"t": "bridge", "msg": line})
            return
        parsed = parse_line(TELEM, line)
        # 参数行/外设行不再进日志面板: 一次 !CFG 就是几十行, 轮询的 !PERIPH 又是
        # 每 2 秒 6 行, 都会把日志冲掉。两者在各自的页面上有专门显示。
        # (FAIL/WARN 提示行解析不出来, 仍然照常进日志)
        if not (parsed and parsed[0] in ("params", "per")):
            broadcast({"t": "raw", "line": line})
        if parsed:
            key, snap = parsed
            broadcast({"t": key, "d": snap})

    def on_state(up, msg):
        broadcast({"t": "link", "up": up, "msg": msg})
        if not (up and BRIDGE):
            # 桥接断了: 设备状态无从得知 (serial_console 重连后会自己补发),
            # 连接态与轮询先停 —— 命令只会被排队, 攒着等设备回来重放
            if _connected.is_set():
                _connected.clear()
                broadcast({"t": "conn", "on": False, "msg": "桥接已断开"})
            return
        with _subs_lock:
            has_client = bool(_subscribers)
        if not has_client:
            return
        if _connected.is_set():
            BRIDGE.send("!CFG")      # 桥接(重)连后补参数表, 否则断线期间改的看不到
        elif dev_state()["up"]:
            BRIDGE.send("!VER")      # 未连接时只需刷新开始页的版本行

    BRIDGE = BridgeClient(args.bridge_host, args.bridge_port, on_line, on_state)
    threading.Thread(target=BRIDGE.run, daemon=True, name="bridge").start()
    threading.Thread(target=poller_thread, daemon=True, name="poller",
                     args=(args.batt_interval, args.imu_interval, args.servo_interval,
                           args.i2c_interval, args.periph_interval)
                     ).start()

    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    httpd.daemon_threads = True

    print(f"网页监视台: http://{args.host}:{args.port}")
    print(f"串口桥接:   {args.bridge_host}:{args.bridge_port} "
          f"(需先运行 python3 tools/serial_console.py)")
    if args.host != "127.0.0.1":
        print("⚠️  监听非本地地址 —— 局域网内任何人都能查看并控制机器人")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        BRIDGE.stop()
        httpd.server_close()


if __name__ == "__main__":
    main()

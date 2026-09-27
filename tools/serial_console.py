#!/usr/bin/env python3
"""
六足机器人 USB 串口交互控制台 (+ TCP/IP 网络传输桥接)
用法: python3 tools/serial_console.py [端口名] [--port 7100] [--no-tcp]

功能:
  - 自动发现设备: 优先 /dev/serial/by-id 中的 Pico, 其次其他串口, 再 /dev/ttyACM*
    (可用参数指定固定端口, 如 serial_console.py ttyACM0)
  - 持续监视端口: 设备出现立即连接; 拔掉/复位/重新上电后不退出, 自动重新连接。
    ★ 先启动本脚本再给机器人上电, 即可捕获完整开机日志
  - 未连接时输入的命令自动排队, 连接成功后立即发送
  - 快捷舵机微调: <id> <angle> 自动加 !P 前缀
  - 手动行编辑器: 半行输入不被串口数据打断, ↑↓ 命令历史, 退格
  - Ctrl+D 退出, Ctrl+C 清空当前行
  - TCP 桥接: 串口数据经 TCP/IP 广播给多个客户端, 客户端命令反向下发
    (配合 tools/tcp_monitor.py 使用; --no-tcp 可禁用)

TCP 桥接数据流与线程安全约定:
    串口 fd  ──读──>  主线程 ──bridge_publish()──> tcp_tx 队列 ──> TCP 线程 ──> 各客户端
    客户端  ──> TCP 线程 ──> tcp_rx 队列 ──自唤醒管道──> 主线程 ──send_line()──> 串口 fd

  ★ 串口 fd 只由主线程读写; TCP 线程只碰套接字; 两者仅通过线程安全队列通信。
    客户端命令复用主线程的 send_line(), 因此长度限制/前缀/排队语义与控制台输入完全一致。

  ★ 桥接连着 ≠ 机器人在线: 设备状态由 "[TCP] dev on|busy|off [端口]" 单独发布
    (publish_dev_state), 客户端接入时补发一次当前值。网页配置台靠它决定能否连接。
"""

import argparse
import collections
import glob
import os
import queue
import socket
import sys
import time
import fcntl
import termios
import select
import threading

DEFAULT_BAUD = termios.B115200
MAX_CMD_LEN = 47          # 固件命令行缓冲 SERIAL_CMD_BUF-1=47 (hexapod_hal_pico.c)
                          # 必须 ≥ 最长的一条 "!CFG <参数名 19> <值>" ≈ 36
SCAN_INTERVAL = 0.5       # 端口扫描周期 (秒)
STATUS_INTERVAL = 2.0     # 等待状态提示周期 (秒)

FIXED_PORT = None         # 命令行指定的固定端口 (None=自动发现)

# ---- TCP 桥接配置 ----
TCP_HOST = "127.0.0.1"        # 仅监听本机 (评测演示足够; 暴露到局域网改 0.0.0.0)
TCP_PORT = 7100
TCP_ENABLED = True
BRIDGE_QUEUE_LIMIT = 65536    # 单客户端积压上限 (字节), 超限判定为慢消费者并断开
BRIDGE_LINE_MAX = 8192        # 行重组缓冲上限, 防止异常长行撑爆内存

# ---- TCP 桥接运行时状态 ----
tcp_tx = queue.Queue()               # 机器人 -> 客户端 (完整行 bytes, 含换行)
tcp_rx = queue.Queue()               # 客户端 -> 机器人 (str 命令行)
_tcp_linebuf = bytearray()           # 跨数据块的行重组缓冲 (仅主线程访问)
_clients_lock = threading.Lock()
_clients = {}                        # conn -> bytearray (待发送积压)
_wake_r, _wake_w = os.pipe()         # 自唤醒管道: TCP 线程通知主 select 立即返回
os.set_blocking(_wake_r, False)
os.set_blocking(_wake_w, False)
_tcp_server = None
_listener = None
_dev_state = "[TCP] dev off"         # 最近一次设备状态行 (主线程写, TCP 线程读来补发)

# stdio 是否为终端。headless 运行 (如 N100 上的后台服务, 或由测试脚本派生)
# 时 stdin 不是 tty: termios 配置会失败, 且 stdin 恒可读会把主循环拖成忙等。
# 此时跳过原始模式并完全不监听键盘输入, 控制通道只剩 TCP 桥接。
HAS_TTY = sys.stdin.isatty()


def open_port(port, baud=DEFAULT_BAUD):
    """
    用 O_NONBLOCK 打开串口，避免内核 TTY 层阻塞。
    USB CDC ACM 设备不提供 DCD 信号，默认 open 会永久阻塞。
    之后配置 termios：波特率、CLOCAL、CREAD、8N1、raw 模式。
    """
    fd = os.open(port, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)

    # 配置终端属性（必须在清除 O_NONBLOCK 之前，以便 tcgetattr 正常工作）
    attrs = termios.tcgetattr(fd)

    # 波特率（USB CDC 忽略实际速率，但内核 TTY 层需要合法值）
    attrs[4] = baud  # ispeed
    attrs[5] = baud  # ospeed

    # 控制标志：CLOCAL=忽略调制解调器控制线（USB CDC 无 DCD）
    #           CREAD=启用接收器
    attrs[2] |= termios.CLOCAL | termios.CREAD

    # 8N1：8 数据位，无校验，1 停止位
    attrs[2] &= ~termios.CSIZE
    attrs[2] |= termios.CS8
    attrs[2] &= ~(termios.PARENB | termios.PARODD)
    attrs[2] &= ~termios.CSTOPB

    # 禁用硬件流控（USB CDC 不支持 RTS/CTS）
    attrs[2] &= ~termios.CRTSCTS

    # Raw 模式：关闭行规则处理（ICANON）、回显（ECHO）、信号字符（ISIG）
    attrs[3] &= ~(termios.ICANON | termios.ECHO | termios.ISIG)
    # 也关闭输入处理：不要转换 CR/NL
    attrs[0] &= ~(termios.INLCR | termios.ICRNL | termios.IGNCR)

    # 输出：不做 NL→CRNL 转换
    attrs[1] &= ~termios.ONLCR

    # VMIN / VTIME：至少 1 字节，无超时（由 select 控制）
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0

    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    # 清除 O_NONBLOCK（之后由 select 控制阻塞行为）
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags & ~os.O_NONBLOCK)
    return fd


def find_port():
    """查找可用串口: 用户指定 > by-id 中的 Pico > 其他 by-id > /dev/ttyACM*"""
    if FIXED_PORT:
        return FIXED_PORT if os.path.exists(FIXED_PORT) else None

    byid = sorted(glob.glob("/dev/serial/by-id/*"))
    pico = [p for p in byid if "pico" in os.path.basename(p).lower()]
    cands = pico + [p for p in byid if p not in pico] + sorted(glob.glob("/dev/ttyACM*"))
    for c in cands:
        if os.path.exists(c):
            return c
    return None


def redraw(state):
    sys.stdout.write("\r\033[K> " + state["line"])
    sys.stdout.flush()


def print_serial(data, state):
    # 清除当前输入行 → 打印数据 → 恢复用户正在输入的内容
    sys.stdout.write("\r\033[K")
    sys.stdout.buffer.write(data)
    sys.stdout.buffer.flush()
    sys.stdout.write("> " + state["line"])
    sys.stdout.flush()
    bridge_publish(data)     # 同步广播给 TCP 客户端


def bridge_publish(data):
    """
    把串口收到的原始数据按行重组后排队广播给 TCP 客户端。
    仅主线程调用 (与 print_serial 同路径), 因此 _tcp_linebuf 无需加锁。
    无客户端连接时直接丢弃, 保证队列不会无限增长。
    """
    if not TCP_ENABLED:
        return
    with _clients_lock:
        if not _clients:
            _tcp_linebuf.clear()
            return

    _tcp_linebuf.extend(data)
    while True:
        idx = _tcp_linebuf.find(b"\n")
        if idx < 0:
            break
        line = bytes(_tcp_linebuf[:idx + 1])
        del _tcp_linebuf[:idx + 1]
        tcp_tx.put(line)

    # 异常长行 (固件不应出现): 截断防内存膨胀
    if len(_tcp_linebuf) > BRIDGE_LINE_MAX:
        tcp_tx.put(bytes(_tcp_linebuf) + b"\n")
        _tcp_linebuf.clear()


def publish_dev_state(state, port=None):
    """
    发布设备在线状态给 TCP 客户端: "[TCP] dev on /dev/ttyACM0" / "busy <port>" / "off"。

    桥接连着 ≠ 机器人在线 —— 网页配置台靠这行判断"能不能连接", 所以每次状态**变化**
    都发一次 (重复调用自动去重), 新客户端接入时也由 TCP 线程补发当前值。
    仅主线程调用 (与 bridge_publish 同路径)。
    """
    global _dev_state
    line = f"[TCP] dev {state}" + (f" {port}" if port else "")
    if line == _dev_state:
        return
    _dev_state = line
    bridge_publish((line + "\n").encode())


def tcp_server_thread():
    """TCP 服务线程: 仅操作套接字, 绝不触碰串口 fd。"""
    global _listener
    try:
        _listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        _listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        _listener.bind((TCP_HOST, TCP_PORT))
        _listener.listen(8)
        _listener.setblocking(False)
    except OSError as e:
        sys.stdout.write(f"\r\033[K[TCP] 无法监听 {TCP_HOST}:{TCP_PORT}: {e} (桥接已禁用)\r\n")
        sys.stdout.flush()
        return

    sys.stdout.write(f"\r\033[K[TCP] 监听 {TCP_HOST}:{TCP_PORT} "
                     f"(tools/tcp_monitor.py 可连接)\r\n")
    sys.stdout.flush()

    while True:
        with _clients_lock:
            fds = [_listener] + list(_clients.keys())
        try:
            r, _, _ = select.select(fds, [], [], 0.5)
        except (OSError, ValueError):
            continue

        # ---- 接受新连接 ----
        if _listener in r:
            try:
                conn, addr = _listener.accept()
                conn.setblocking(False)
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with _clients_lock:
                    _clients[conn] = bytearray()
                    n = len(_clients)
                conn.sendall(b"[TCP] bridge connected. Type commands, e.g. !A\r\n")
                conn.sendall(_dev_state.encode() + b"\n")   # 立即告知设备在线状态
                sys.stdout.write(f"\r\033[K[TCP] 客户端接入 {addr[0]}:{addr[1]} (共 {n})\r\n")
                sys.stdout.flush()
            except OSError:
                pass

        # ---- 收集待广播数据 ----
        pending = []
        while True:
            try:
                pending.append(tcp_tx.get_nowait())
            except queue.Empty:
                break

        # ---- 接收客户端命令 ----
        for conn in [c for c in r if c is not _listener]:
            try:
                data = conn.recv(4096)
            except (BlockingIOError, InterruptedError):
                continue
            except OSError:
                data = b""
            if not data:
                _drop_client(conn, "closed by peer")
                continue
            for line in data.replace(b"\r", b"\n").split(b"\n"):
                cmd = line.decode("utf-8", "replace").strip()
                if cmd:
                    tcp_rx.put(cmd)
                    try:
                        os.write(_wake_w, b"\x00")   # 唤醒主 select 循环
                    except (BlockingIOError, OSError):
                        pass   # 管道满 = 主循环必然已醒, 忽略

        # ---- 锁内: 仅累积积压并判定慢消费者 ----
        # 发送与断开都放在锁外: 主线程 bridge_publish() 也取这把锁 (非递归),
        # 持锁做 send() 会挡住串口读路径; 且 _drop_client() 会重入取锁 → 自死锁。
        # out 缓冲仅由本线程改动, 故锁外访问安全。
        slow = []                    # [(conn, why)]
        with _clients_lock:
            if pending:
                for conn, out in list(_clients.items()):
                    if len(out) > BRIDGE_QUEUE_LIMIT:
                        slow.append((conn, "slow consumer (backlog > %dB)"
                                     % BRIDGE_QUEUE_LIMIT))
                        continue     # 不再喂给它, 稍后断开
                    for chunk in pending:
                        out.extend(chunk)
            drop = {c for c, _ in slow}
            flush = [(c, o) for c, o in _clients.items() if c not in drop]

        # ---- 锁外: 刷出各客户端积压 ----
        for conn, out in flush:
            while out:
                try:
                    sent = conn.send(bytes(out[:4096]))
                except (BlockingIOError, InterruptedError):
                    break
                except OSError:
                    slow.append((conn, "send failed"))
                    break
                del out[:sent]

        # ---- 锁外: 断开 (含慢消费者) ----
        for conn, why in slow:
            _drop_client(conn, why)


def _drop_client(conn, why):
    """关闭并移除一个客户端 (调用方须保证未持锁或可重入)。"""
    with _clients_lock:
        _clients.pop(conn, None)
        n = len(_clients)
    try:
        conn.close()
    except OSError:
        pass
    sys.stdout.write(f"\r\033[K[TCP] 客户端断开 ({why}, 剩余 {n})\r\n")
    sys.stdout.flush()


def drain_tcp_commands(state, fd):
    """
    排空自唤醒管道与 tcp_rx 队列, 把客户端命令交给主线程发送。
    返回 'lost' 表示串口写入失败 (设备已断开), 否则 None。
    """
    try:
        while True:
            if not os.read(_wake_r, 4096):
                break
    except (BlockingIOError, InterruptedError):
        pass
    except OSError:
        pass

    while True:
        try:
            cmd = tcp_rx.get_nowait()
        except queue.Empty:
            break
        if fd is None:
            # 未连接: 复用控制台排队机制 (state["pending"] 为单值, 保留最新命令)
            state["pending"] = cmd
            sys.stdout.write(f"\r\033[K[TCP] 未连接, 已排队: {cmd}\r\n")
            sys.stdout.flush()
            continue
        act = send_line(cmd, state, fd)
        if act == "lost":
            return "lost"
        redraw(state)
    return None


def send_line(text, state, fd):
    """转换并发送一条命令。返回 'sent' / 'rejected' / 'lost' (写入失败=设备已断开)"""
    out = text if text.startswith("!") else "!P" + text
    if len(out) > MAX_CMD_LEN:
        # 拒发而不是截断: 截断后的命令可能仍然"合法", 于是静默做错事
        # (固件侧同样拒收超长命令, 两边行为一致)
        sys.stdout.write(f"\r\033[KCommand too long ({len(out)}>{MAX_CMD_LEN}), NOT sent: {out[:MAX_CMD_LEN]}…\r\n")
        redraw(state)        # 保留用户输入, 便于删几个字符重发
        return "rejected"
    try:
        os.write(fd, (out + "\n").encode())
    except OSError:
        return "lost"
    sys.stdout.write("\r\n")
    sys.stdout.flush()
    state["line"] = ""
    state["hist_idx"] = None
    return "sent"


def handle_key(b, state, connected, fd):
    """
    处理一个输入字节 (手动行编辑器)。
    connected=False 时 Enter 不发送, 而是把命令排队, 连接成功后自动发送。
    返回: None=继续, 'quit'=退出程序, 'lost'=连接已断, 'sent'=已发送
    """
    if b in (0x03,):        # Ctrl+C → 清空当前行
        state["line"] = ""
        state["hist_idx"] = None
        redraw(state)
    elif b in (0x04,):      # Ctrl+D → 空行时退出
        if not state["line"]:
            return "quit"
        # 行非空时 Ctrl+D 视为删除光标处字符
    elif b in (0x7f, 0x08):  # Backspace / DEL
        if state["line"]:
            state["line"] = state["line"][:-1]
            redraw(state)
    elif b in (0x0d, 0x0a):  # Enter → 执行 (或排队)
        cmd = state["line"].strip()
        if cmd in ("/quit", "/exit"):
            return "quit"
        if cmd:
            if not state["history"] or state["history"][-1] != cmd:
                state["history"].append(cmd)
            if not connected:
                state["pending"] = cmd
                sys.stdout.write(f"\r\033[K[排队] 连接后自动发送: {cmd}\r\n")
                sys.stdout.flush()
                state["line"] = ""
                state["hist_idx"] = None
                redraw(state)
                return None
            act = send_line(cmd, state, fd)
            if act == "sent":
                redraw(state)
            return act
        state["line"] = ""
        state["hist_idx"] = None
        redraw(state)
    elif b == 0x1b:          # ESC 序列 (方向键)
        try:
            r2, _, _ = select.select([sys.stdin], [], [], 0.03)
            if not r2:
                redraw(state)
                return None
            seq = os.read(sys.stdin.fileno(), 2)
        except OSError:
            seq = b""
        if seq == b"[A" and state["history"]:      # ↑
            if state["hist_idx"] is None:
                state["hist_idx"] = len(state["history"]) - 1
            elif state["hist_idx"] > 0:
                state["hist_idx"] -= 1
            state["line"] = state["history"][state["hist_idx"]]
            redraw(state)
        elif seq == b"[B" and state["hist_idx"] is not None:  # ↓
            if state["hist_idx"] < len(state["history"]) - 1:
                state["hist_idx"] += 1
                state["line"] = state["history"][state["hist_idx"]]
            else:
                state["hist_idx"] = None
                state["line"] = ""
            redraw(state)
        else:
            redraw(state)
    elif 0x20 <= b <= 0x7e:  # 可打印字符
        if len(state["line"]) >= MAX_CMD_LEN + 1:
            sys.stdout.write("\a")  # 超长提示音
        else:
            state["line"] += chr(b)
            redraw(state)
    # 其他控制字符忽略
    return None


def run_session(fd, port, state):
    """已连接状态的主循环。返回 'quit' (用户退出) 或 'lost' (设备断开)"""
    redraw(state)
    while True:
        watch = ([sys.stdin] if HAS_TTY else []) + [fd, _wake_r]
        r, _, _ = select.select(watch, [], [], 0.5)
        # 主动检测设备节点消失 (如 Pico 切换到 BOOTSEL U 盘模式):
        # 立即释放端口, 避免内核因 tty 被占用而延迟 USB 重枚举
        if not os.path.exists(port):
            return "lost"
        if fd in r:
            try:
                data = os.read(fd, 4096)
            except OSError:
                return "lost"
            if not data:
                return "lost"
            print_serial(data, state)
        if _wake_r in r:
            # TCP 客户端下发的命令: 复用同一套 send_line 语义
            if drain_tcp_commands(state, fd) == "lost":
                return "lost"
        if HAS_TTY and sys.stdin in r:
            ch = os.read(sys.stdin.fileno(), 1)
            if not ch:
                return "quit"
            action = handle_key(ch[0], state, True, fd)
            if action in ("quit", "lost"):
                return action


def main():
    global FIXED_PORT, TCP_ENABLED, TCP_PORT, TCP_HOST
    ap = argparse.ArgumentParser(
        description="六足机器人串口控制台 (含 TCP/IP 桥接)",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("device", nargs="?",
                    help="串口设备 (如 ttyACM0 或 /dev/ttyACM0), 缺省自动发现")
    ap.add_argument("--port", type=int, default=TCP_PORT,
                    help=f"TCP 桥接监听端口 (默认 {TCP_PORT})")
    ap.add_argument("--host", default=TCP_HOST,
                    help=f"TCP 桥接监听地址 (默认 {TCP_HOST}; 局域网访问用 0.0.0.0)")
    ap.add_argument("--no-tcp", action="store_true", help="禁用 TCP 桥接")
    args = ap.parse_args()

    if args.device:
        FIXED_PORT = args.device
        if "/" not in FIXED_PORT:  # 裸设备名 (如 ttyACM0) → 补 /dev/ 前缀; 完整路径原样使用
            FIXED_PORT = "/dev/" + FIXED_PORT

    TCP_ENABLED = not args.no_tcp
    TCP_PORT = args.port
    TCP_HOST = args.host

    # ---- stdin 进入 raw 模式 (提前设置: 未连接时也支持 Ctrl+D 退出 / 预输入) ----
    old_tc = None
    if HAS_TTY:
        old_tc = termios.tcgetattr(sys.stdin.fileno())
        raw_tc = termios.tcgetattr(sys.stdin.fileno())
        raw_tc[3] &= ~(termios.ICANON | termios.ECHO | termios.ISIG)
        raw_tc[0] &= ~(termios.IXON | termios.ICRNL | termios.INLCR)
        raw_tc[6][termios.VMIN] = 1
        raw_tc[6][termios.VTIME] = 0
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, raw_tc)

    # ---- 编辑器状态 (跨连接保持: 历史/当前行/排队命令不因掉线丢失) ----
    state = {"line": "", "history": [], "hist_idx": None, "pending": None}

    print("Hexapod Serial Console")
    print("  自动扫描设备, 出现即连接, 掉线自动重连 (Ctrl+D 退出)")
    print("  ★ 先启动本脚本再给机器人上电, 可捕获完整开机日志")
    print("  未连接时输入的命令自动排队, 连接后立即发送")
    print()
    print("  舵机直控:  <id> <angle>   例: 0 900  (= !P0 900)")
    print("  姿态:     !M -1 (窄)  !M 0 (正常)  !M 1 (宽)")
    print("  步态:     !G0~!G4 (步态切换)")
    print("  运动:     !O(解锁) !F !B !L !R !S(停) !U/!D(抬腿)")
    print("  校准:     !C 舵机校准  !PER PCA9685周期校准")
    print("  诊断:     !I2C(总线检测) !A(舵机快照) !V(调试) !PS2(手柄状态)")
    print("  存储:     !SAVE(保存校准到flash) !LOG(事件日志) !LOGC(清空日志)")
    print("  模式:     !MODE crsf|ps2|auto")
    print("  ↑↓ 历史  Ctrl+C 清行  Ctrl+D 或 /quit 退出")

    if not HAS_TTY:
        print("  [headless] stdin 非终端: 已跳过 termios 原始模式, 键盘输入不可用")
        print("             控制通道仅剩 TCP 桥接 — 用 tools/tcp_monitor.py 连接")
        if not TCP_ENABLED:
            print("  [警告] 同时指定了 --no-tcp: 将无法下发任何命令 (只能旁观日志)")
        print()

    # ---- 启动 TCP 桥接服务线程 (仅操作套接字, 不碰串口 fd) ----
    if TCP_ENABLED:
        threading.Thread(target=tcp_server_thread, daemon=True,
                         name="tcp-bridge").start()
    else:
        print("  [TCP] 桥接已禁用 (--no-tcp)")
    print()

    last_notice = 0.0
    last_open_error = None
    try:
        while True:
            port = find_port()

            if port is None:
                # ---- 等待设备出现, 同时支持预输入 (排队命令) ----
                publish_dev_state("off")     # 设备消失 (拔线) 或 "占用" 后端口没了
                if time.monotonic() - last_notice >= STATUS_INTERVAL:
                    last_notice = time.monotonic()
                    sys.stdout.write("\r\033[K[wait] 未发现串口设备, 持续扫描... (Ctrl+D 退出)\r\n")
                    sys.stdout.flush()
                    redraw(state)
                watch = ([sys.stdin] if HAS_TTY else []) + [_wake_r]
                r, _, _ = select.select(watch, [], [], SCAN_INTERVAL)
                if _wake_r in r:
                    # 未连接时 TCP 命令进排队 (fd=None), 连接后自动发送
                    drain_tcp_commands(state, None)
                if HAS_TTY and sys.stdin in r:
                    ch = os.read(sys.stdin.fileno(), 1)
                    if not ch:
                        break
                    if handle_key(ch[0], state, False, None) == "quit":
                        break
                continue

            # ---- 端口出现: 尝试打开 (刚枚举的设备可能暂时打不开, 循环重试) ----
            try:
                fd = open_port(port)
            except OSError as e:
                # 端口存在但打不开: 提示真实原因 (权限/占用/设备未就绪),
                # 避免误报成"未发现设备"误导排查
                if last_open_error != str(e):
                    last_open_error = str(e)
                    sys.stdout.write(f"\r\033[K[wait] 发现 {port} 但打开失败: {e} — 持续重试...\r\n")
                    sys.stdout.flush()
                    redraw(state)
                    # 端口在但打不开: 多半是别的程序占着 (第二个读者), 网页端要能提示
                    publish_dev_state("busy", port)
                time.sleep(0.3)
                continue
            last_open_error = None

            sys.stdout.write(f"\r\033[K[conn] 已连接 {port}\r\n")
            sys.stdout.flush()
            publish_dev_state("on", port)

            # 排空缓冲的固件输出 (可能包含连接前的开机日志)
            # ⚠️ 出口必须同时有"静默 0.2s"和"总时长上限": 固件有 5Hz 的 [CH]
            # 遥测行, 间隔正好 200ms —— 只看静默的话这个循环永远退不出去,
            # 主循环卡在这里, TCP 命令全部积压 (现象: 网页读数正常但发命令没反应)。
            try:
                drain_deadline = time.time() + 1.5
                while time.time() < drain_deadline:
                    r, _, _ = select.select([fd], [], [], 0.2)
                    if not r:
                        break
                    data = os.read(fd, 4096)
                    if not data:
                        break
                    sys.stdout.buffer.write(data)
                    bridge_publish(data)     # 开机日志也广播给 TCP 客户端
            except OSError:
                pass  # 排空期间设备断开: 按连接失败处理
            sys.stdout.buffer.flush()

            # 发送排队命令 (若有)
            if state["pending"]:
                cmd = state["pending"]
                state["pending"] = None
                sys.stdout.write(f"[conn] 发送排队命令: {cmd}\r\n")
                sys.stdout.flush()
                act = send_line(cmd, state, fd)
                redraw(state)
                if act == "lost":
                    os.close(fd)
                    sys.stdout.write("\r\033[K[lost] 连接断开, 继续扫描...\r\n")
                    sys.stdout.flush()
                    publish_dev_state("off")
                    last_notice = 0.0
                    continue

            # 连接建立后才到达的 TCP 命令
            if drain_tcp_commands(state, fd) == "lost":
                os.close(fd)
                sys.stdout.write("\r\033[K[lost] 连接断开, 继续扫描...\r\n")
                sys.stdout.flush()
                publish_dev_state("off")
                last_notice = 0.0
                continue

            # ---- 已连接交互循环 ----
            result = run_session(fd, port, state)
            try:
                os.close(fd)
            except OSError:
                pass
            if result == "quit":
                break
            sys.stdout.write("\r\033[K[lost] 连接断开, 继续扫描...\r\n")
            sys.stdout.flush()
            publish_dev_state("off")
            last_notice = 0.0

    except KeyboardInterrupt:
        pass
    finally:
        print()
        if old_tc is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_tc)


if __name__ == "__main__":
    main()

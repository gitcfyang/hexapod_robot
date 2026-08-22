#!/usr/bin/env python3
"""
六足机器人 USB 串口交互控制台
用法: python3 tools/serial_console.py [端口名]

功能:
  - 自动发现设备: 优先 /dev/serial/by-id 中的 Pico, 其次其他串口, 再 /dev/ttyACM*
    (可用参数指定固定端口, 如 serial_console.py ttyACM0)
  - 持续监视端口: 设备出现立即连接; 拔掉/复位/重新上电后不退出, 自动重新连接。
    ★ 先启动本脚本再给机器人上电, 即可捕获完整开机日志
  - 未连接时输入的命令自动排队, 连接成功后立即发送
  - 快捷舵机微调: <id> <angle> 自动加 !P 前缀
  - 手动行编辑器: 半行输入不被串口数据打断, ↑↓ 命令历史, 退格
  - Ctrl+D 退出, Ctrl+C 清空当前行
"""

import glob
import os
import sys
import time
import fcntl
import termios
import select

DEFAULT_BAUD = termios.B115200
MAX_CMD_LEN = 15          # 固件命令缓冲区 16 字节，! + 命令 ≤ 15 字节可打印字符
SCAN_INTERVAL = 0.5       # 端口扫描周期 (秒)
STATUS_INTERVAL = 2.0     # 等待状态提示周期 (秒)

FIXED_PORT = None         # 命令行指定的固定端口 (None=自动发现)


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


def send_line(text, state, fd):
    """转换并发送一条命令。返回 'sent' 或 'lost' (写入失败=设备已断开)"""
    out = text if text.startswith("!") else "!P" + text
    if len(out) > MAX_CMD_LEN:
        sys.stdout.write(f"\r\033[KCommand too long ({len(out)}>{MAX_CMD_LEN}), truncated: {out[:MAX_CMD_LEN]}\r\n")
        out = out[:MAX_CMD_LEN]
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
        r, _, _ = select.select([sys.stdin, fd], [], [], 0.5)
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
        if sys.stdin in r:
            ch = os.read(sys.stdin.fileno(), 1)
            if not ch:
                return "quit"
            action = handle_key(ch[0], state, True, fd)
            if action in ("quit", "lost"):
                return action


def main():
    global FIXED_PORT
    if len(sys.argv) > 1:
        FIXED_PORT = sys.argv[1]
        if "/" not in FIXED_PORT:  # 裸设备名 (如 ttyACM0) → 补 /dev/ 前缀; 完整路径原样使用
            FIXED_PORT = "/dev/" + FIXED_PORT

    # ---- stdin 进入 raw 模式 (提前设置: 未连接时也支持 Ctrl+D 退出 / 预输入) ----
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
    print("  模式:     !MODE crsf|ps2|auto")
    print("  ↑↓ 历史  Ctrl+C 清行  Ctrl+D 或 /quit 退出")
    print()

    last_notice = 0.0
    last_open_error = None
    try:
        while True:
            port = find_port()

            if port is None:
                # ---- 等待设备出现, 同时支持预输入 (排队命令) ----
                if time.monotonic() - last_notice >= STATUS_INTERVAL:
                    last_notice = time.monotonic()
                    sys.stdout.write("\r\033[K[wait] 未发现串口设备, 持续扫描... (Ctrl+D 退出)\r\n")
                    sys.stdout.flush()
                    redraw(state)
                r, _, _ = select.select([sys.stdin], [], [], SCAN_INTERVAL)
                if sys.stdin in r:
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
                time.sleep(0.3)
                continue
            last_open_error = None

            sys.stdout.write(f"\r\033[K[conn] 已连接 {port}\r\n")
            sys.stdout.flush()

            # 排空缓冲的固件输出 (可能包含连接前的开机日志)
            try:
                while True:
                    r, _, _ = select.select([fd], [], [], 0.2)
                    if not r:
                        break
                    data = os.read(fd, 4096)
                    if not data:
                        break
                    sys.stdout.buffer.write(data)
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
            last_notice = 0.0

    except KeyboardInterrupt:
        pass
    finally:
        print()
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_tc)


if __name__ == "__main__":
    main()

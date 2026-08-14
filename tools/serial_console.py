#!/usr/bin/env python3
"""
六足机器人 USB 串口交互控制台
用法: python3 tools/serial_console.py [ttyACM0]

功能:
  - 快捷舵机微调: <id> <angle> 自动加 !P 前缀
  - 手动行编辑器: 半行输入不被串口数据打断, ↑↓ 命令历史, 退格
  - 固件输出实时显示 (主循环 select 双路监听 stdin + 串口)
  - Ctrl+D 或 /quit 退出, Ctrl+C 清空当前行
"""

import os
import sys
import time
import fcntl
import termios
import select

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = termios.B115200
MAX_CMD_LEN = 15  # 固件命令缓冲区 16 字节，! + 命令 ≤ 15 字节可打印字符


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


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    if not port.startswith("/dev/"):
        port = "/dev/" + port

    # 打开设备（O_NONBLOCK 防止卡死）
    fd = open_port(port)
    print(f"Connected to {port}")

    # 等待并显示启动信息
    time.sleep(0.6)
    # 排空已有数据
    while True:
        r, _, _ = select.select([fd], [], [], 0.2)
        if not r:
            break
        data = os.read(fd, 4096)
        if not data:
            break
        sys.stdout.buffer.write(data)
    sys.stdout.buffer.flush()

    print("Hexapod Serial Console")
    print("  舵机直控:  <id> <angle>   例: 0 900  (= !P0 900)")
    print("  姿态:     !M -1 (窄)  !M 0 (正常)  !M 1 (宽)")
    print("  步态:     !G0~!G4 (步态切换)")
    print("  运动:     !O(解锁) !F !B !L !R !S(停) !U/!D(抬腿)")
    print("  校准:     !C 舵机校准  !PER PCA9685周期校准")
    print("  诊断:     !I2C(总线检测) !A(舵机快照) !V(调试) !PS2(手柄状态)")
    print("  模式:     !MODE crsf|ps2|auto")
    print("  ↑↓ 历史  Ctrl+C 清行  Ctrl+D 或 /quit 退出")
    print()

    # ---- stdin 进入 raw 模式 (手动行编辑器) ----
    old_tc = termios.tcgetattr(sys.stdin.fileno())
    raw_tc = termios.tcgetattr(sys.stdin.fileno())
    raw_tc[3] &= ~(termios.ICANON | termios.ECHO | termios.ISIG)
    raw_tc[0] &= ~(termios.IXON | termios.ICRNL | termios.INLCR)
    raw_tc[6][termios.VMIN] = 1
    raw_tc[6][termios.VTIME] = 0
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, raw_tc)

    line = ""          # 当前输入行
    history = []       # 命令历史
    hist_idx = None    # 历史浏览位置 (None=最新)

    def redraw():
        sys.stdout.write("\r\033[K> " + line)
        sys.stdout.flush()

    def print_serial(data):
        # 清除当前输入行 → 打印数据 → 恢复用户正在输入的内容
        sys.stdout.write("\r\033[K")
        sys.stdout.buffer.write(data)
        sys.stdout.buffer.flush()
        sys.stdout.write("> " + line)
        sys.stdout.flush()

    def send_line(text):
        """转换并发送一条命令"""
        nonlocal line, hist_idx
        if text.startswith("!"):
            out = text
        else:
            out = "!P" + text
        if len(out) > MAX_CMD_LEN:
            sys.stdout.write(f"\r\033[KCommand too long ({len(out)}>{MAX_CMD_LEN}), truncated: {out[:MAX_CMD_LEN]}\r\n")
            out = out[:MAX_CMD_LEN]
        os.write(fd, (out + "\n").encode())
        sys.stdout.write("\r\n")
        sys.stdout.flush()
        line = ""
        hist_idx = None
        return out

    redraw()
    try:
        while True:
            r, _, _ = select.select([sys.stdin, fd], [], [])
            if fd in r:
                try:
                    data = os.read(fd, 4096)
                except OSError:
                    break
                if not data:
                    break
                print_serial(data)
            if sys.stdin in r:
                ch = os.read(sys.stdin.fileno(), 1)
                if not ch:
                    break
                b = ch[0]

                if b in (0x03,):        # Ctrl+C → 清空当前行
                    line = ""
                    hist_idx = None
                    redraw()
                elif b in (0x04,):      # Ctrl+D → 退出
                    if not line:
                        break
                    # 行非空时 Ctrl+D 视为删除光标处字符
                elif b in (0x7f, 0x08):  # Backspace / DEL
                    if line:
                        line = line[:-1]
                        redraw()
                elif b in (0x0d, 0x0a):  # Enter → 执行
                    cmd = line.strip()
                    if cmd == "/quit" or cmd == "/exit":
                        break
                    if cmd:
                        if not history or history[-1] != cmd:
                            history.append(cmd)
                        send_line(cmd)
                        redraw()
                    else:
                        line = ""
                        redraw()
                elif b == 0x1b:          # ESC 序列 (方向键)
                    try:
                        r2, _, _ = select.select([sys.stdin], [], [], 0.03)
                        if not r2:
                            redraw()
                            continue
                        seq = os.read(sys.stdin.fileno(), 2)
                    except OSError:
                        seq = b""
                    if seq == b"[A" and history:      # ↑
                        if hist_idx is None:
                            hist_idx = len(history) - 1
                        elif hist_idx > 0:
                            hist_idx -= 1
                        line = history[hist_idx]
                        redraw()
                    elif seq == b"[B" and hist_idx is not None:  # ↓
                        if hist_idx < len(history) - 1:
                            hist_idx += 1
                            line = history[hist_idx]
                        else:
                            hist_idx = None
                            line = ""
                        redraw()
                    else:
                        redraw()
                elif 0x20 <= b <= 0x7e:  # 可打印字符
                    if len(line) >= MAX_CMD_LEN + 1:
                        sys.stdout.write("\a")  # 超长提示音
                    else:
                        line += chr(b)
                        redraw()
                # 其他控制字符忽略

    except KeyboardInterrupt:
        pass
    finally:
        print()
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_tc)
        os.close(fd)


if __name__ == "__main__":
    main()

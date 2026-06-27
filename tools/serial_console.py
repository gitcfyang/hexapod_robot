#!/usr/bin/env python3
"""
六足机器人 USB 串口交互控制台
用法: python3 tools/serial_console.py [ttyACM0]

快捷舵机微调，命令历史 (↑↓)。
"""

import os
import sys
import time
import fcntl
import termios
import select
import readline

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
    print("  servo:   <id> <angle> → !P<id> <angle>")
    print("  stance:  !M -1 (窄)  !M 0 (正常)  !M 1 (宽)")
    print("  gait:    !G0~!G4 (步态切换)")
    print("  other:   !O(解锁) !S(停) !A(舵机快照) !V(调试)")
    print("  Ctrl+D or /quit to exit")
    print()

    # 保存终端设置用于安全退出
    old_tc = termios.tcgetattr(sys.stdin.fileno())

    try:
        while True:
            # 显示串口缓冲数据
            while True:
                r, _, _ = select.select([fd], [], [], 0.05)
                if not r:
                    break
                data = os.read(fd, 4096)
                if not data:
                    break
                # 清除当前输入行 → 打印数据 → 重新显示提示符
                sys.stdout.write("\r\033[K")
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
                sys.stdout.write("> ")
                sys.stdout.flush()

            # 读取用户输入
            try:
                cmd = input("> ").strip()
            except EOFError:
                break

            if not cmd:
                continue
            if cmd in ("/quit", "/exit"):
                break

            if cmd[0] == '!':
                line = cmd
            else:
                line = "!P" + cmd

            if len(line) > MAX_CMD_LEN:
                print(f"Command too long ({len(line)} > {MAX_CMD_LEN}), truncated")
                line = line[:MAX_CMD_LEN]

            os.write(fd, (line + "\n").encode())

            # 等待响应
            time.sleep(0.15)
            while True:
                r, _, _ = select.select([fd], [], [], 0.15)
                if not r:
                    break
                data = os.read(fd, 4096)
                if not data:
                    break
                sys.stdout.buffer.write(data)
            sys.stdout.buffer.flush()

    except KeyboardInterrupt:
        pass
    finally:
        print()
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_tc)
        os.close(fd)


if __name__ == "__main__":
    main()

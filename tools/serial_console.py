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


def open_port(port):
    """
    用 O_NONBLOCK 打开串口，避免内核 TTY 层阻塞。
    USB CDC ACM 设备不提供 DCD 信号，默认 open 会永久阻塞。
    """
    fd = os.open(port, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
    # 恢复为阻塞模式（我们自己用 select 控制）
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
    print("  shortcut: <id> <angle>  →  !P<id> <angle>")
    print("  direct:   !cmds         →  send as-is")
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

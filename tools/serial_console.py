#!/usr/bin/env python3
"""
六足机器人 USB 串口交互控制台
用法: python3 tools/serial_console.py [--port /dev/ttyACM2]

无线程、无卡死。命令历史 (↑↓)，快捷舵机控制。
"""

import serial
import sys
import time
import readline

DEFAULT_PORT = "/dev/ttyACM2"
BAUD = 115200


def drain_and_print(ser):
    """读出串口缓冲区所有数据并打印"""
    while ser.in_waiting:
        data = ser.read(ser.in_waiting)
        sys.stdout.buffer.write(data)
    sys.stdout.buffer.flush()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT

    # 打开串口 — dsrdtr=False 防止 DTR 复位 Pico
    ser = serial.Serial(port, BAUD, timeout=0, dsrdtr=False)
    time.sleep(0.6)

    # 显示启动信息
    drain_and_print(ser)

    print("Hexapod Serial Console")
    print("  shortcut: <id> <angle>  →  !P<id> <angle>")
    print("  e.g.      0 900           →  !P0 900")
    print("  direct:   !O !F !Z !A   →  send as-is")
    print("  Ctrl+D or /quit to exit")
    print()

    try:
        while True:
            # 在等待用户输入前，先显示串口数据
            drain_and_print(ser)

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

            ser.write((line + "\n").encode())
            ser.flush()

            # 等待 Pico 响应并显示
            time.sleep(0.15)
            drain_and_print(ser)

    except KeyboardInterrupt:
        pass
    finally:
        print()
        ser.close()


if __name__ == "__main__":
    main()

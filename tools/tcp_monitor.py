#!/usr/bin/env python3
"""
六足机器人 TCP 监视器
用法: python3 tools/tcp_monitor.py [--host 127.0.0.1] [--port 7100]

连接 serial_console.py 的 TCP 桥接端口:
  - 实时打印机器人输出的每一行 (状态行/日志/命令回显)
  - 从标准输入输入的命令转发给机器人 (经桥接串口下发)
  - 支持多实例同时连接 (桥接会广播给所有客户端)
  - Ctrl+C / Ctrl+D 退出

注意: 需先运行 tools/serial_console.py (它负责串口连接与桥接服务)。
"""

import argparse
import select
import socket
import sys


def main():
    ap = argparse.ArgumentParser(description="六足机器人 TCP 监视器")
    ap.add_argument("--host", default="127.0.0.1", help="桥接地址 (默认 127.0.0.1)")
    ap.add_argument("--port", type=int, default=7100, help="桥接端口 (默认 7100)")
    args = ap.parse_args()

    try:
        s = socket.create_connection((args.host, args.port), timeout=5)
    except OSError as e:
        print(f"连接失败 {args.host}:{args.port}: {e}", file=sys.stderr)
        print("请确认 tools/serial_console.py 已在运行 (且未加 --no-tcp)", file=sys.stderr)
        return 1

    s.setblocking(False)
    print(f"[connected {args.host}:{args.port}] 输入命令回车发送, Ctrl+D 退出")

    try:
        while True:
            r, _, _ = select.select([sys.stdin, s], [], [], 0.5)
            if s in r:
                try:
                    data = s.recv(4096)
                except (BlockingIOError, InterruptedError):
                    continue
                if not data:
                    print("\n[服务端已关闭连接]")
                    break
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
            if sys.stdin in r:
                line = sys.stdin.readline()
                if not line:            # Ctrl+D
                    break
                if not line.strip():
                    continue
                try:
                    s.sendall(line.encode())
                except OSError as e:
                    print(f"\n[发送失败: {e}]")
                    break
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
        print("\n[disconnected]")
    return 0


if __name__ == "__main__":
    sys.exit(main())

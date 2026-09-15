#!/usr/bin/env python3
"""
假机器人: 用于无硬件验证 TCP 桥接 (配 socat pty 对使用)
用法: python3 tools/fake_robot.py [pty路径]

在指定 pty 上模拟固件行为:
  - 每 1s 打印一条状态行 (模仿 [IDLE]/[RUN] 遥测)
  - 收到以 ! 开头的行时回显 [FAKE] got: <命令>
  - !BURST <n>: 连发 n 行 (测试用, 制造足以触发慢客户端保护的数据量;
    真实固件无此命令)

pty 消失 (socat 被杀 / 控制台重启) 时会自动重开并继续计数 ——
真实机器人不会因为上位机重启而停止运行。

典型测试流程:
  socat -d -d pty,raw,echo=0,link=/tmp/fakerobot \\
                pty,raw,echo=0,link=/tmp/robotport
  python3 tools/fake_robot.py /tmp/fakerobot     # 终端 1
  python3 tools/serial_console.py /tmp/robotport # 终端 2
  python3 tools/tcp_monitor.py                   # 终端 3 (应看到同样的状态行)
"""

import os
import select
import sys
import time


def handle_line(fd, cmd):
    """处理一条收到的命令。返回 False 表示应当断开。"""
    if not cmd:
        return True
    if cmd.startswith("!BURST"):
        try:
            n = int(cmd.split()[1])
        except (IndexError, ValueError):
            n = 1000
        n = max(1, min(n, 200000))
        sys.stderr.write(f"[fake] burst {n} lines\n")
        sys.stderr.flush()
        chunk = b""
        for i in range(n):
            chunk += (f"[BURST] {i:06d} "
                      f"0123456789012345678901234567890123456789\r\n").encode()
            if len(chunk) >= 4096:
                try:
                    os.write(fd, chunk)   # 阻塞写: pty 满则等待控制台排空
                except OSError:
                    return False
                chunk = b""
        if chunk:
            try:
                os.write(fd, chunk)
            except OSError:
                return False
        return True
    try:
        os.write(fd, f"[FAKE] got: {cmd}\r\n".encode())
    except OSError:
        return False
    return True


def serve(path, counter):
    """打开 pty 并服务, 直到断开。返回更新后的计数器。"""
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    sys.stderr.write(f"[fake] 已打开 {path}\n")
    sys.stderr.flush()

    last = time.monotonic()      # 重连后不立刻补发积压的状态行
    buf = b""
    try:
        while True:
            now = time.monotonic()
            if now - last >= 1.0:
                last = now
                counter += 1
                msg = (f"[IDLE] Waiting for Arm signal (CH5)... "
                       f"tick={counter} batt=7800mV\r\n")
                try:
                    os.write(fd, msg.encode())
                except OSError:
                    break

            r, _, _ = select.select([fd], [], [], 0.2)
            if fd not in r:
                continue
            try:
                data = os.read(fd, 4096)
            except OSError:
                break
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                cmd = line.decode("utf-8", "replace").strip()
                if not handle_line(fd, cmd):
                    return counter
    except KeyboardInterrupt:
        raise
    finally:
        try:
            os.close(fd)
        except OSError:
            pass
        sys.stderr.write("[fake] pty 断开\n")
        sys.stderr.flush()
    return counter


def main():
    if len(sys.argv) < 2:
        print("用法: fake_robot.py <pty路径>", file=sys.stderr)
        return 1
    path = sys.argv[1]

    counter = 0
    try:
        while True:
            try:
                counter = serve(path, counter)
            except OSError as e:
                sys.stderr.write(f"[fake] 打开 {path} 失败: {e}\n")
                sys.stderr.flush()
            time.sleep(0.5)          # 等 pty 重新出现
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TCP 桥接端到端测试 (无硬件, 用 socat pty 对模拟串口)。

覆盖: 双向转发 / 多客户端广播 / 慢客户端断开 / 串口断开重连恢复 / 设备在线状态发布。
用法: 在 tools/ 目录下 `python3 test_bridge.py`
"""
import os, re, signal, socket, subprocess, sys, time

TOOLS = os.path.dirname(os.path.abspath(__file__))
PTY_A, PTY_B = "/tmp/fakerobot", "/tmp/robotport"
TCP_PORT = 7100
LOG_CONSOLE = "/tmp/t_console.log"

procs = []
passed, failed = [], []


def check(name, cond, detail=""):
    (passed if cond else failed).append(name)
    print(("  PASS  " if cond else "  FAIL  ") + name
          + (f"  [{detail}]" if detail else ""))
    sys.stdout.flush()


def spawn(args, log):
    f = open(log, "wb")
    p = subprocess.Popen(args, stdout=f, stderr=subprocess.STDOUT, cwd=TOOLS)
    procs.append(p)
    return p


def _argv(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            return [a.decode("utf8", "replace") for a in f.read().split(b"\0") if a]
    except OSError:
        return []


def kill_stale():
    """清理上次运行残留的同名进程。

    只认 argv[0] 为解释器/socat 的进程。若按整条命令行做子串匹配, 会误杀
    正在跑本脚本的那个 shell (它的命令行里就含有本文件名) —— 表现为 ssh
    连接中断返回 255, 而测试输出仍然正常打印, 极易误判。
    """
    me = os.getpid()
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == me:
            continue
        argv = _argv(pid)
        if not argv:
            continue
        prog = os.path.basename(argv[0])
        kill = (prog.startswith("python")
                and any(a.endswith(("fake_robot.py", "serial_console.py",
                                    "test_bridge.py")) for a in argv[1:])) \
            or (prog == "socat" and any("fakerobot" in a for a in argv[1:]))
        if kill:
            try:
                os.kill(int(pid), signal.SIGKILL)
            except OSError:
                pass
    time.sleep(0.5)


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
    for path in (PTY_A, PTY_B):
        try:
            os.unlink(path)
        except OSError:
            pass


def tcp_connect(timeout=5):
    """连接桥接端口, 返回已设非阻塞的 socket (或 None)"""
    end = time.time() + timeout
    while time.time() < end:
        try:
            s = socket.create_connection(("127.0.0.1", TCP_PORT), timeout=1)
            s.setblocking(False)
            return s
        except OSError:
            time.sleep(0.2)
    return None


def recv_until(s, pattern, timeout=6.0):
    """累积接收直到匹配 pattern (bytes 正则)。返回 (matched, buffer, drained_bytes)"""
    buf, total = b"", 0
    end = time.time() + timeout
    rx = re.compile(pattern)
    while time.time() < end:
        try:
            d = s.recv(65536)
            if not d:
                return False, buf, total          # 对端关闭
            buf += d
            total += len(d)
            if rx.search(buf):
                return True, buf, total
        except BlockingIOError:
            time.sleep(0.05)
        except OSError:
            return False, buf, total
    return False, buf, total


def drained(buf, n=80):
    return buf[-n:].decode("utf8", "replace").replace("\r", "").strip()


def console_log():
    try:
        with open(LOG_CONSOLE, "rb") as f:
            return f.read().decode("utf8", "replace")
    except OSError:
        return ""


print("=" * 62)
print("TCP 桥接端到端测试")
print("=" * 62)
cleanup()
kill_stale()

# ---------- 1. 建立 pty 对 + 假机器人 + 控制台 ----------
print("\n[1] 启动 socat pty 对 / 假机器人 / 串口控制台")
spawn(["socat", "-d", "-d", "pty,raw,echo=0,link=" + PTY_A,
       "pty,raw,echo=0,link=" + PTY_B], "/tmp/t_socat.log")
time.sleep(1.5)
check("pty 对创建成功", os.path.exists(PTY_A) and os.path.exists(PTY_B))

spawn([sys.executable, "fake_robot.py", PTY_A], "/tmp/t_fake.log")
time.sleep(0.8)
spawn([sys.executable, "serial_console.py", PTY_B, "--port", str(TCP_PORT)],
      LOG_CONSOLE)
time.sleep(2.0)

log = console_log()
check("控制台已连接串口", "[conn] 已连接" in log, log[:100].replace("\r", ""))
check("TCP 桥接已监听", "[TCP] 监听" in log,
      next((l for l in log.splitlines() if "[TCP]" in l), "无 [TCP] 行"))

# ---------- 2. 客户端 A: 接收遥测 ----------
print("\n[2] 客户端 A 接收遥测 (机器人 -> TCP)")
a = tcp_connect()
check("客户端 A 连接成功", a is not None)
if a:
    ok, buf, _ = recv_until(a, rb"\[IDLE\].*tick=\d+")
    check("客户端 A 收到 [IDLE] 状态行", ok, drained(buf))
    # 桥接连着 ≠ 设备在线: 接入那一刻就补发当前设备状态
    # (同一批数据里, 所以查累积缓冲而不是再收一轮)
    check("客户端 A 接入即收到 [TCP] dev on", b"[TCP] dev on" in buf, drained(buf))

# ---------- 3. 客户端 A 下发命令 ----------
print("\n[3] 客户端 A 下发命令 (TCP -> 机器人)")
if a:
    a.sendall(b"!A\n")
    ok, buf, _ = recv_until(a, rb"\[FAKE\] got: !A")
    check("命令经桥接下达到机器人并回显", ok, drained(buf))

# ---------- 4. 多客户端广播 ----------
print("\n[4] 多客户端广播")
b = tcp_connect()
check("客户端 B 连接成功", b is not None)
if b:
    ok, buf, _ = recv_until(b, rb"\[IDLE\].*tick=\d+")
    check("客户端 B 也收到遥测 (广播)", ok, drained(buf))
if b:
    b.sendall(b"!V\n")
    ok_b, _, _ = recv_until(b, rb"\[FAKE\] got: !V")
    ok_a, buf_a, _ = recv_until(a, rb"\[FAKE\] got: !V") if a else (False, b"", 0)
    check("B 的命令回显到 B", ok_b)
    check("B 的命令同时广播到 A (共享串口)", ok_a, drained(buf_a))

# ---------- 5. 慢客户端断开保护 ----------
# 数据量需远超内核 socket 缓冲 (snd 16KB / rcv 128KB + 自动调优),
# 否则全被内核吸收, 应用层 deque 涨不到 64KB 阈值。
print("\n[5] 慢客户端断开保护 (64KB 应用层积压)")
if a:
    a.close()
if b:
    b.close()
time.sleep(0.5)
slow = tcp_connect()
check("慢客户端连接成功", slow is not None)
if slow:
    # 连上后完全不读 → 让它自己触发 !BURST, 制造 MB 级积压
    slow.sendall(b"!BURST 60000\n")
    dropped, why = False, ""
    deadline = time.time() + 60
    while time.time() < deadline:
        if "slow consumer" in console_log():
            dropped, why = True, "控制台日志判定"
            break
        try:
            d = slow.recv(262144)
            if d == b"":
                dropped, why = True, "对端关闭连接 (EOF)"
                break
        except BlockingIOError:
            pass
        except OSError as e:
            dropped, why = True, f"连接错误 {e}"
            break
        time.sleep(0.4)
    check("慢客户端被判定并断开", dropped, why or "40s 内未触发")
    try:
        slow.close()
    except Exception:
        pass

# ---------- 6. 串口断开重连恢复 ----------
print("\n[6] 串口断开重连恢复")
c = tcp_connect()
check("新客户端 C 连接成功", c is not None)
if c:
    ok, _, _ = recv_until(c, rb"\[IDLE\].*tick=\d+")
    check("重连前 C 正常收遥测", ok)

socat_p = next((p for p in procs if "socat" in p.args[0]), None)
if socat_p:
    socat_p.kill()
time.sleep(1.5)
if c:
    ok, buf, _ = recv_until(c, rb"\[TCP\] dev off", timeout=6)
    check("串口消失后 C 收到 [TCP] dev off", ok, drained(buf))
if c:
    ok, _, _ = recv_until(c, rb"\[IDLE\]", timeout=3)
    check("串口消失后 C 保持连接 (无遥测但仍连着)", not ok)

spawn(["socat", "-d", "-d", "pty,raw,echo=0,link=" + PTY_A,
       "pty,raw,echo=0,link=" + PTY_B], "/tmp/t_socat2.log")
time.sleep(3.0)          # 假机器人每 0.5s 重试开 pty, 控制台每 0.5s 重扫
ok, buf, _ = recv_until(c, rb"\[TCP\] dev on ", timeout=20) if c else (False, b"", 0)
check("socat 重启后 C 收到 [TCP] dev on", ok, drained(buf))
ok, buf, _ = recv_until(c, rb"\[IDLE\].*tick=\d+", timeout=20) if c else (False, b"", 0)
check("socat 重启后控制台自动重连并恢复遥测", ok, drained(buf))
try:
    with open("/tmp/t_fake.log", "rb") as f:
        flog = f.read().decode("utf8", "replace")
except OSError:
    flog = ""
check("假机器人熬过 pty 消失并重新打开 (模拟真机不中断)",
      flog.count("已打开") >= 2, f"重开 {flog.count('已打开')} 次")
if c:
    c.sendall(b"!V\n")
    ok, buf, _ = recv_until(c, rb"\[FAKE\] got: !V", timeout=10)
    check("重连后双向仍可用", ok, drained(buf))

# ---------- 收尾 ----------
for s in (a, b, slow, c):
    try:
        if s:
            s.close()
    except Exception:
        pass
cleanup()

print("\n" + "=" * 62)
print(f"结果: {len(passed)} 通过, {len(failed)} 失败")
if failed:
    print("失败项:")
    for f in failed:
        print("  - " + f)
print("=" * 62)
sys.exit(1 if failed else 0)

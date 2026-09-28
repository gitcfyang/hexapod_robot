#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""网页配置台快捷启停 (Linux / WSL / Windows 通用)

用法:
    python3 tools/webconfig/run.py start [server.py 的额外参数...]   # 如 --host 0.0.0.0
    python3 tools/webconfig/run.py stop
    python3 tools/webconfig/run.py restart [额外参数...]
    python3 tools/webconfig/run.py status
    python3 tools/webconfig/run.py logs

只管 webconfig/server.py 这一个进程: 不碰 serial_console.py 串口桥
(/dev/ttyACM0 的唯一读者), 也不碰别的 python (run.sh 是本脚本的 Linux 包装)。

进程识别两道核实 —— ① 命令行里得有一个参数**就是**这个脚本的路径, 光"命令文本
里提到过它"不算 (否则 git commit / grep / 编辑器所在的 shell 都会被收进来);
② POSIX 下 /proc/<pid>/exe 必须真的是 python, Windows 下 tasklist 的映像名同理
(PID 从 netstat -ano 的监听端口反查)。

环境变量: HEXAPOD_WC_PORT (默认 8080) · HEXAPOD_WC_LOG (默认临时目录下
hexapod-webconfig.log)
"""

import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
SERVER = HERE / "server.py"
BRIDGE_SCRIPT = "tools/serial_console.py"   # 只用来显示状态, 从不启停它

IS_WIN = os.name == "nt"
PORT = int(os.environ.get("HEXAPOD_WC_PORT") or 8080)
LOG = Path(os.environ.get("HEXAPOD_WC_LOG")
           or Path(tempfile.gettempdir()) / "hexapod-webconfig.log")
BRIDGE_PORT = 7100
HOST = os.environ.get("HEXAPOD_WC_HOST") or "127.0.0.1"   # server.py 的默认值
USAGE = (f"用法: {os.path.basename(__file__)} "
         "{start|stop|restart|status|logs} [server.py 参数...]")
HELP = f"""网页配置台快捷启停 (Linux / WSL / Windows)

  {os.path.basename(__file__)} start [server.py 的额外参数...]  启动 (已在跑就什么都不做)
  {os.path.basename(__file__)} stop                             停掉 (先 TERM, 3 秒不退再 KILL)
  {os.path.basename(__file__)} restart [额外参数...]            stop + start
  {os.path.basename(__file__)} status                           进程 / 端口 / 串口桥 / 日志尾
  {os.path.basename(__file__)} logs                             跟踪日志 (Ctrl-C 退出)

只管 webconfig/server.py 这一个进程: 不碰 serial_console.py 串口桥
(/dev/ttyACM0 的唯一读者), 也不碰别的 python。run.sh 是本脚本的 Linux 包装。

环境变量: HEXAPOD_WC_HOST (默认 127.0.0.1) · HEXAPOD_WC_PORT (默认 8080) ·
          HEXAPOD_WC_LOG (默认临时目录)

想让别的电脑用浏览器连过来, 见 README「远程访问」一节 —— 这个服务没有鉴权,
别直接挂公网。"""


def probe_addr(host):
    """把监听地址换算成"能连上去的地址": 0.0.0.0 / :: 这种通配地址只能回落到本机"""
    return "127.0.0.1" if host in ("", "0.0.0.0", "::", "*") else host


def port_open(port, host=None, timeout=0.3):
    with socket.socket() as s:
        s.settimeout(timeout)
        return s.connect_ex((probe_addr(HOST if host is None else host), port)) == 0


def tail(path, n):
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - 8192))
            return f.read().decode("utf-8", "replace").splitlines()[-n:]
    except OSError:
        return []


# ==================== 找进程 ====================

def _pid_exe(pid):
    """POSIX: 进程可执行文件的名字 (不是 python 就不是我们要找的)"""
    try:
        return os.path.basename(os.readlink(f"/proc/{pid}/exe"))
    except OSError:
        return ""


def _pids_by_cmdline(script_suffix):
    """/proc 扫描: 某个参数以 script_suffix 结尾 + exe 是 python"""
    pids = []
    try:
        entries = os.listdir("/proc")
    except OSError:
        return pids                      # 没有 /proc (macOS 等): 交给按端口那条路
    for entry in entries:
        if not entry.isdigit():
            continue
        pid = int(entry)
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                args = [a.decode("utf-8", "replace") for a in f.read().split(b"\0") if a]
        except OSError:
            continue
        if not any(a.endswith(script_suffix) for a in args[1:]):
            continue
        if not _pid_exe(pid).startswith("python"):
            continue
        pids.append(pid)
    return pids


def _win_image_name(pid):
    """Windows: tasklist 的映像名 (python.exe?)"""
    try:
        r = subprocess.run(["tasklist", "/FI", f"PID eq {pid}", "/FO", "CSV", "/NH"],
                           capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return ""
    line = r.stdout.strip().splitlines()
    if not line:
        return ""
    return line[0].split('","')[0].strip('"').lower()


def _pids_by_port(port):
    """Windows: netstat -ano 反查监听端口的 PID"""
    pids = []
    try:
        r = subprocess.run(["netstat", "-ano", "-p", "TCP"],
                           capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return pids
    for line in r.stdout.splitlines():
        f = line.split()
        if len(f) >= 5 and f[0].upper() == "TCP" and f[3].upper() == "LISTENING" \
                and f[1].endswith(f":{port}"):
            try:
                pids.append(int(f[4]))
            except ValueError:
                pass
    return pids


def server_pids(port=None):
    port = PORT if port is None else port
    if IS_WIN:
        pids = _pids_by_port(port)
        # 端口上的监听者未必是我们的 python —— 映像名不是 python 就不认
        return [p for p in pids if _win_image_name(p).startswith("python")]
    return _pids_by_cmdline("webconfig/server.py")


def pid_opt(pid, name, fallback):
    """从命令行里读它实际用的选项值 —— "已在运行" 报的地址得是真的那个"""
    if IS_WIN:
        return fallback
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            args = [a.decode("utf-8", "replace") for a in f.read().split(b"\0") if a]
    except OSError:
        return fallback
    for i, a in enumerate(args):
        val = args[i + 1] if a == name and i + 1 < len(args) else None
        if a.startswith(name + "="):
            val = a.split("=", 1)[1]
        if val:
            return val
    return fallback


def pid_addr(pid):
    """运行中进程的真实 (host, port); 命令行里没有就用 server.py 的默认值。
    不能用"这次命令想用的值"兜底 —— 那会把别人的实例报成自己要的那个地址。"""
    return (pid_opt(pid, "--host", "127.0.0.1"),
            int(pid_opt(pid, "--port", 8080)))


def bridge_pid():
    if IS_WIN:
        return None if not port_open(BRIDGE_PORT) else -1   # 只报"在不在", 不报 pid
    for pid in _pids_by_cmdline(BRIDGE_SCRIPT):
        return pid
    return None


def kill_pids(pids, hard=False):
    for pid in pids:
        try:
            # Windows 上任何信号都是 TerminateProcess, 所以硬杀也不换写法
            os.kill(pid, signal.SIGKILL if (hard and not IS_WIN) else signal.SIGTERM)
        except OSError:
            pass


# ==================== 子命令 ====================

def do_stop():
    pids = server_pids()
    if not pids:
        print("未在运行")
        return 0
    print("停止: " + " ".join(str(p) for p in pids))
    kill_pids(pids)
    deadline = time.time() + 3
    while time.time() < deadline and server_pids():
        time.sleep(0.1)
    left = server_pids()
    if left:
        print("3 秒没退, 强杀: " + " ".join(str(p) for p in left))
        kill_pids(left, hard=True)
        time.sleep(0.3)
    print("已停止")
    return 0


def show_port_owner(port):
    """端口被别人占了: 列出占用者, 让人自己判断 (只读, 不动手)"""
    cmds = [["netstat", "-ano"]] if IS_WIN else [["ss", "-ltnp"], ["netstat", "-ltnp"]]
    for cmd in cmds:
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        except (OSError, subprocess.SubprocessError):
            continue
        hits = [l.strip() for l in r.stdout.splitlines() if f":{port}" in l]
        if r.returncode == 0 and hits:
            print("  " + " ".join(cmd), file=sys.stderr)
            for line in hits[:3]:
                print("  " + line, file=sys.stderr)
            return


def do_start(extra):
    global PORT, HOST
    for i, a in enumerate(extra):                 # 额外参数里的 --port/--host 优先于环境变量
        if a == "--port" and i + 1 < len(extra):
            PORT = int(extra[i + 1])
        elif a.startswith("--port="):
            PORT = int(a.split("=", 1)[1])
        elif a == "--host" and i + 1 < len(extra):
            HOST = extra[i + 1]
        elif a.startswith("--host="):
            HOST = a.split("=", 1)[1]

    pid = next(iter(server_pids()), None)
    if pid:
        # 同一时刻只跑一个实例 (桥与串口都是单份的), 所以这里不另起, 只报实际端口
        r_host, r_port = pid_addr(pid)
        print(f"已在运行 (pid {pid}, http://{probe_addr(r_host)}:{r_port})")
        return 0
    if port_open(PORT):                           # 别人的地盘, 别静默失败
        print(f"端口 {PORT} 已被别的进程占用, 先处理它再启动:", file=sys.stderr)
        print("  (占用者不是本脚本管的 server.py, 所以不代为终止)", file=sys.stderr)
        show_port_owner(PORT)
        return 1
    if not SERVER.is_file():
        print(f"找不到 {SERVER}", file=sys.stderr)
        return 1

    LOG.parent.mkdir(parents=True, exist_ok=True)
    cmd = [sys.executable, str(SERVER), "--host", HOST, "--port", str(PORT), *extra]
    log = open(LOG, "ab", buffering=0)
    kwargs = {"cwd": str(ROOT), "stdin": subprocess.DEVNULL,
              "stdout": log, "stderr": log}
    if IS_WIN:
        kwargs["creationflags"] = (subprocess.CREATE_NEW_PROCESS_GROUP
                                   | subprocess.DETACHED_PROCESS)
    else:
        kwargs["start_new_session"] = True         # 脱离本终端, 关窗口不死
    subprocess.Popen(cmd, **kwargs)
    log.close()

    deadline = time.time() + 5
    while time.time() < deadline:                  # 端口真的在听才算起来
        if port_open(PORT):
            pid = next(iter(server_pids()), "?")
            print(f"已启动 (pid {pid}, 日志 {LOG})")
            print(f"浏览器打开 http://{probe_addr(HOST)}:{PORT}"
                  + ("   (0.0.0.0 = 所有网卡, 换成这台机器的任一 IP 都能开)"
                     if probe_addr(HOST) != HOST else ""))
            if bridge_pid() is None:
                print("⚠️ 串口桥没在跑: 页面能开但数据是离线的 "
                      "(先 python3 tools/serial_console.py)")
            return 0
        time.sleep(0.1)
    print("5 秒内没起来, 日志尾部:", file=sys.stderr)
    for line in tail(LOG, 20):
        print("  " + line, file=sys.stderr)
    return 1


def do_status():
    pids = server_pids()
    # 报运行中进程的真实地址, 而不是这次命令想用的那个
    host, port = pid_addr(pids[0]) if pids else (HOST, PORT)
    print("server: " + (f"运行中 pid {' '.join(str(p) for p in pids)}"
                        if pids else "未运行"))
    print("监听:   " + (f"{host}:{port} 在听" if port_open(port, host)
                        else f"{host}:{port} 空"))
    b = bridge_pid()
    if b is None:
        print("串口桥: 未运行 (页面数据会是离线的)")
    elif b == -1:
        print(f"串口桥: 在跑 (端口 {BRIDGE_PORT} 在听)")
    else:
        print(f"串口桥: 运行中 pid {b}")
    print(f"日志:   {LOG}")
    if LOG.exists():
        for line in tail(LOG, 3):
            print("  " + line)
    return 0


def do_logs():
    if not LOG.exists():
        print(f"日志还不存在: {LOG} (先 start)")
        return 0
    print(f"跟踪 {LOG} (Ctrl-C 退出)")
    with open(LOG, "r", encoding="utf-8", errors="replace") as f:
        f.seek(0, os.SEEK_END)
        try:
            while True:
                line = f.readline()
                if line:
                    sys.stdout.write(line)
                    sys.stdout.flush()
                else:
                    time.sleep(0.3)
        except KeyboardInterrupt:
            print()
    return 0


def main(argv):
    cmd, extra = (argv[0] if argv else ""), argv[1:]
    if cmd == "start":
        return do_start(extra)
    if cmd == "stop":
        return do_stop()
    if cmd == "restart":
        do_stop()
        return do_start(extra)
    if cmd == "status":
        return do_status()
    if cmd == "logs":
        return do_logs()
    if cmd in ("", "-h", "--help", "help"):
        print(HELP)
        return 0
    print(USAGE, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

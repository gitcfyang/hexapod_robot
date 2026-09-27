#!/usr/bin/env python3
"""
固件烧录 —— Betaflight「Firmware Flasher」的服务端实现。

为什么在服务端烧: Pico 插在跑本服务的那台机器上, 浏览器够不着那侧的 USB
(与「为什么必须服务端桥接」同源, 见 STATUS.md)。所以不做 WebUSB。

为什么用 picotool 而不是自己发 BOOTSEL 命令: `picotool load -f -x <uf2>` 一条
命令就覆盖了全流程 —— 请求运行中的固件软复位进 BOOTSEL (-f), 写入 (-x 之后
重启回应用)。自己实现要处理 1200bps touch / UF2 块协议 / 重启时序, 全是为
了重造一个已经随 pico-sdk 装好的工具。

⚠️ `picotool reboot -f` 是**静默空操作**: --force 让命令得以执行, 但 reboot
   会把设备再送回应用模式, 所以那条命令什么都不做。要真重启就用 load -f -x。
"""

import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
FIRMWARE_DIR = REPO / "pico" / "build"

# RP2040 官方 VID; PID 区分设备处在哪个模式
RP2040_VID = "2e8a"
PID_APP = "000a"          # 应用模式 (CDC 串口在跑固件)
PID_BOOTSEL = "0003"      # BOOTSEL (USB 大容量存储, 等烧录)
PID_NAMES = {PID_APP: "app", PID_BOOTSEL: "bootsel"}

SYS_USB = Path("/sys/bus/usb/devices")

# picotool 搜索顺序: 环境变量 > PATH > pico-sdk 里的默认安装位置
PICOTOOL_ENV = "HEXAPOD_PICOTOOL"


def find_picotool():
    """返回 picotool 可执行文件路径, 找不到返回 None"""
    env = os.environ.get(PICOTOOL_ENV)
    if env and Path(env).is_file():
        return env
    found = shutil.which("picotool")
    if found:
        return found
    sdk = Path.home() / ".pico-sdk" / "picotool"
    if sdk.is_dir():
        # 装多个版本时取版本号最大的那个 (目录名形如 2.2.0-a4)
        for ver in sorted(sdk.iterdir(), reverse=True):
            exe = ver / "picotool" / "picotool"
            if exe.is_file():
                return str(exe)
    return None


def usb_devices():
    """扫描 /sys 上的 RP2040 设备。

    只认 2e8a, 不依赖 lsusb (pyusb/udev 都不必装); /sys 是内核直接暴露的
    事实来源, 且接口条目 (形如 1-3:1.0) 没有 idVendor, 天然被跳过。
    """
    out = []
    if not SYS_USB.is_dir():
        return out
    for entry in sorted(SYS_USB.iterdir()):
        try:
            vid = (entry / "idVendor").read_text().strip()
            pid = (entry / "idProduct").read_text().strip()
        except OSError:
            continue          # 接口条目 / 拔掉的设备, 没有这两个文件
        if vid != RP2040_VID:
            continue
        out.append({
            "bus_id": entry.name,
            "vid": vid,
            "pid": pid,
            "mode": PID_NAMES.get(pid, "unknown"),
            "product": _read(entry / "product"),
            "serial": _read(entry / "serial"),
        })
    return out


def _read(p):
    try:
        return p.read_text().strip()
    except OSError:
        return ""


# ==================== 固件文件 ====================

# 允许烧录的目录白名单: 编译产物 + 本次会话的上传目录。
# 不做路径白名单就等着被 POST 里的 ../ 读到任意文件。
_UPLOAD_DIR = Path(tempfile.mkdtemp(prefix="hexapod-fw-"))
_ALLOWED_DIRS = [FIRMWARE_DIR, _UPLOAD_DIR]


def list_firmware():
    """可烧录的本地固件 (按修改时间倒序, 最新的在前)"""
    if not FIRMWARE_DIR.is_dir():
        return []
    items = []
    for f in FIRMWARE_DIR.glob("*.uf2"):
        try:
            st = f.stat()
        except OSError:
            continue
        items.append({"name": f.name, "size": st.st_size, "mtime": int(st.st_mtime)})
    return sorted(items, key=lambda x: x["mtime"], reverse=True)


def save_upload(data, filename="upload.uf2"):
    """把浏览器上传的固件落到临时目录, 返回绝对路径。只保留 .uf2。"""
    name = Path(filename or "upload.uf2").name      # 丢掉任何目录成分
    if not name.lower().endswith(".uf2"):
        raise ValueError("只接受 .uf2 文件")
    dest = _UPLOAD_DIR / name
    dest.write_bytes(data)
    return dest


def resolve_uf2(path_or_name):
    """把请求里的名字/路径解析成允许烧录的绝对路径, 不合法就抛 ValueError。"""
    p = Path(path_or_name)
    cand = p if p.is_absolute() else (FIRMWARE_DIR / p)
    try:
        real = cand.resolve(strict=True)
    except OSError:
        raise ValueError(f"固件不存在: {path_or_name}")
    if real.suffix.lower() != ".uf2":
        raise ValueError("只接受 .uf2 文件")
    if not any(_is_inside(real, d) for d in _ALLOWED_DIRS):
        raise ValueError(f"固件不在允许的目录内: {path_or_name}")
    return real


def _is_inside(path, directory):
    try:
        path.relative_to(directory.resolve())
        return True
    except ValueError:
        return False


# ==================== 烧录 ====================

# picotool 的进度用 \r 原地刷新, 按 \r 和 \n 都要切
_SPLIT = re.compile(r"[\r\n]+")


class FlashError(RuntimeError):
    pass


def flash(uf2_path, on_line=None, timeout=120):
    """烧录一个 uf2。on_line(str) 会被逐行回调 (进度条用)。

    返回 picotool 退出码。命令以参数列表传入, 不经过 shell。
    """
    tool = find_picotool()
    if not tool:
        raise FlashError("找不到 picotool (设 HEXAPOD_PICOTOOL 或装 pico-sdk)")
    uf2 = resolve_uf2(uf2_path)

    cmd = [tool, "load", "-f", "-x", str(uf2)]
    if on_line:
        on_line("$ " + " ".join(cmd))
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1)
    try:
        buf = ""
        while True:
            chunk = proc.stdout.read(1)
            if not chunk:
                break
            if chunk in "\r\n":
                line = buf.strip()
                if line and on_line:
                    on_line(line)
                buf = ""
            else:
                buf += chunk
        if buf.strip() and on_line:
            on_line(buf.strip())
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        raise FlashError(f"picotool 超时 ({timeout}s)")
    finally:
        try:
            proc.stdout.close()
        except OSError:
            pass

    if proc.returncode != 0:
        raise FlashError(f"picotool 退出码 {proc.returncode}")
    return proc.returncode

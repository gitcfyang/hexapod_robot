#!/usr/bin/env bash
# 网页配置台快捷启停
#
#   ./run.sh start [server.py 的额外参数...]    启动 (已在跑就什么都不做)
#   ./run.sh stop                              停掉 (先 TERM, 3 秒不退再 KILL)
#   ./run.sh restart [额外参数...]              stop + start
#   ./run.sh status                            进程 / 端口 / 串口桥 / 日志尾
#   ./run.sh logs                              跟踪日志 (Ctrl-C 退出)
#
# 只管 webconfig/server.py 这一个进程: 不碰 serial_console.py (串口桥是
# /dev/ttyACM0 的唯一读者, 停了就没人读串口了), 也不碰别的 python。
#
# 环境变量: HEXAPOD_WC_PORT (默认 8080) · HEXAPOD_WC_LOG
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
SERVER="$HERE/server.py"
PORT="${HEXAPOD_WC_PORT:-8080}"
LOG="${HEXAPOD_WC_LOG:-/tmp/hexapod-webconfig.log}"
BRIDGE_PATTERN="tools/serial_console\.py"

# 找脚本进程: 先 pgrep 收个大概, 再逐条核实 ——
#   ① /proc/<pid>/exe 必须真的是 python (排除临时 shell: 一条命令的文本里
#      提到过 "python ... server.py" 也会被 pgrep 收进来, 杀它就把人家终端干了);
#   ② 命令行里得有一个参数**就是**那个脚本的路径 (不是"文本里提到")。
script_pids() {
    local pattern="$1" script="$2" pid exe arg
    for pid in $(pgrep -f "$pattern" 2>/dev/null || true); do
        exe="$(basename "$(readlink "/proc/$pid/exe" 2>/dev/null || true)" 2>/dev/null || true)"
        case "$exe" in python*) ;; *) continue ;; esac
        while IFS= read -r -d '' arg; do
            case "$arg" in
                *"$script") echo "$pid"; break ;;
            esac
        done < "/proc/$pid/cmdline"
    done
}

server_pids() { script_pids 'webconfig/server\.py' 'webconfig/server.py'; }
bridge_pid()  { script_pids "$BRIDGE_PATTERN" 'tools/serial_console.py' | head -1; }

port_open() {
    (exec 3<>"/dev/tcp/127.0.0.1/$PORT") 2>/dev/null
}

# 端口被别人占了 (不是我们的 server) 就明确报出来, 别让 start 静默失败
port_owner_other() {
    port_open || return 1
    [[ -z "$(server_pids)" ]] || return 1   # 我们的人在听, 不算"别人"
    return 0
}

do_stop() {
    local pids pid deadline
    pids="$(server_pids || true)"
    if [[ -z "$pids" ]]; then
        echo "未在运行"
        return 0
    fi
    echo "停止: $(echo "$pids" | tr '\n' ' ')"
    kill $pids 2>/dev/null || true
    deadline=$((SECONDS + 3))
    while (( SECONDS < deadline )); do
        pids="$(server_pids || true)"
        [[ -z "$pids" ]] && break
        sleep 0.1
    done
    pids="$(server_pids || true)"
    if [[ -n "$pids" ]]; then
        echo "3 秒没退, 强杀: $(echo "$pids" | tr '\n' ' ')"
        kill -9 $pids 2>/dev/null || true
        sleep 0.3
    fi
    echo "已停止"
}

do_start() {
    local -a extra=("$@")
    local i pid
    # 允许额外参数里带 --port, 但跟着它走 (启动参数优先于环境变量)
    for (( i = 0; i < ${#extra[@]}; i++ )); do
        case "${extra[i]}" in
            --port) PORT="${extra[i+1]:-$PORT}" ;;
            --port=*) PORT="${extra[i]#--port=}" ;;
        esac
    done

    pid="$(server_pids | head -1 || true)"
    if [[ -n "$pid" ]]; then
        echo "已在运行 (pid $pid, http://127.0.0.1:$PORT)"
        return 0
    fi
    if port_owner_other; then
        echo "端口 $PORT 已被别的进程占用, 先处理它再启动:" >&2
        ss -ltnp 2>/dev/null | grep ":$PORT " >&2 || true
        return 1
    fi
    [[ -f "$SERVER" ]] || { echo "找不到 $SERVER" >&2; return 1; }

    cd "$ROOT"
    nohup setsid python3 "$SERVER" --port "$PORT" "${extra[@]}" >>"$LOG" 2>&1 &

    local deadline=$((SECONDS + 5))
    while (( SECONDS < deadline )); do
        if port_open; then
            pid="$(server_pids | head -1 || true)"
            echo "已启动 (pid ${pid:-?}, 日志 $LOG)"
            echo "浏览器打开 http://127.0.0.1:$PORT"
            [[ -n "$(bridge_pid)" ]] || \
                echo "⚠️ 串口桥没在跑: 页面能开但数据是离线的 (先 python3 tools/serial_console.py)"
            return 0
        fi
        sleep 0.1
    done
    echo "5 秒内没起来, 日志尾部:" >&2
    tail -n 20 "$LOG" >&2 || true
    return 1
}

do_status() {
    local pids pid bridge
    pids="$(server_pids || true)"
    if [[ -n "$pids" ]]; then
        echo "server: 运行中 pid $(echo "$pids" | tr '\n' ' ')"
    else
        echo "server: 未运行"
    fi
    if port_open; then
        echo "端口:   $PORT 在听"
    else
        echo "端口:   $PORT 空"
    fi
    bridge="$(bridge_pid || true)"
    if [[ -n "$bridge" ]]; then
        echo "串口桥: 运行中 pid $bridge"
    else
        echo "串口桥: 未运行 (页面数据会是离线的)"
    fi
    echo "日志:   $LOG"
    [[ -f "$LOG" ]] && tail -n 3 "$LOG" || true
}

case "${1:-}" in
    start)   shift; do_start "$@" ;;
    stop)    do_stop ;;
    restart) shift; do_stop; do_start "$@" ;;
    status)  do_status ;;
    logs)    tail -f "$LOG" ;;
    ""|-h|--help|help)
        # 打印文件开头的整段注释当帮助
        awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "${BASH_SOURCE[0]}"
        ;;
    *) echo "用法: $0 {start|stop|restart|status|logs} [server.py 参数...]" >&2; exit 2 ;;
esac

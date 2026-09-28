#!/usr/bin/env bash
# 网页配置台快捷启停 (Linux 侧包装): 实现在 run.py, 这里只是省得敲 python3 ——
# 两份实现会走偏, 所以只有这一层壳, 不加任何逻辑。
#
#   ./run.sh {start|stop|restart|status|logs} [server.py 参数...]
#
# 跨平台 (Windows / WSL) 直接用: python3 tools/webconfig/run.py 同参数。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$HERE/run.py" "$@"

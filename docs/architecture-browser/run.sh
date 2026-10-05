#!/usr/bin/env bash
set -euo pipefail
browser_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$browser_dir/../.." && pwd)"
port="${1:-4173}"
if [[ ! "$port" =~ ^[0-9]{1,5}$ ]] || (( 10#$port < 1 || 10#$port > 65535 )); then
    printf '端口必须是 1–65535 的整数。\n' >&2
    exit 2
fi
port="$((10#$port))"
printf '架构与接口手册：http://127.0.0.1:%s/docs/architecture-browser/#overview\n' "$port"
printf '按 Ctrl+C 停止。Markdown 与源码均从当前仓库读取。\n'
exec python3 "$browser_dir/workspace_docs.py" --root "$repo_dir" --serve "$port"

#!/usr/bin/env bash
set -euo pipefail
browser_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
docs_dir="$(dirname -- "$browser_dir")"
port="${1:-4173}"
printf '架构浏览器：http://127.0.0.1:%s/architecture-browser/#motor-workflow\n' "$port"
exec python3 -m http.server "$port" --bind 127.0.0.1 --directory "$docs_dir"

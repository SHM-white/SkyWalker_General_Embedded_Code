#!/usr/bin/env bash
# Run from any directory; shared by local builds and Actions.
set -euo pipefail
repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo_root"
digest=$(sha256sum west.yml .devcontainer/Dockerfile | sha256sum | cut -c1-20)
printf 'env-%s\n' "$digest"

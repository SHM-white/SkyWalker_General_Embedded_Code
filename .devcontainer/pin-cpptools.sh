#!/usr/bin/env bash
# postAttach runs after the VS Code remote CLI becomes available.
# Explicit @version installation pins the extension, including in a reused container.
set -euo pipefail
if ! command -v code >/dev/null 2>&1; then
    echo 'VS Code remote CLI unavailable; reconnect the Dev Container to pin C/C++.' >&2
    exit 1
fi
code --install-extension ms-vscode.cpptools@1.17.4 --force
if ! code --list-extensions --show-versions | tr -d '\r' | grep -Fxiq 'ms-vscode.cpptools@1.17.4'; then
    echo 'C/C++ version pin failed; expected 1.17.4. Reconnect after checking Marketplace access.' >&2
    exit 1
fi

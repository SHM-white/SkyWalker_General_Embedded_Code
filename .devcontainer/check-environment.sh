#!/usr/bin/env bash
set -euo pipefail
cd /opt/skywalker/skywalker_code
if ! cmp -s west.yml /opt/skywalker-image-west.yml; then
    echo 'west.yml differs from the prebuilt image. See docs/getting-started/devcontainer.md to rebuild/publish the matching image.' >&2
    exit 1
fi
printf 'Preinstalled west dependencies and ARM SDK ready.\n'
printf 'Build: west build -p always -b dm_mc02/stm32h723xx -d build/control samples/control\n'

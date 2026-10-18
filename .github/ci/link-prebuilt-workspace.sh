#!/usr/bin/env bash
# Connect the checked-out PR repository to the immutable, preinstalled west projects.
set -euo pipefail
: "$GITHUB_WORKSPACE"
image_root=/opt/skywalker-zephyr
cd "$GITHUB_WORKSPACE"
test -f skywalker_code/west.yml
test -d "$image_root/zephyr"
test -d "$image_root/.west"
mkdir -p .west
printf '[manifest]\npath = skywalker_code\nfile = west.yml\n' > .west/config
while IFS= read -r project_path; do
    [[ -n "$project_path" && "$project_path" != "skywalker_code" ]] || continue
    source_path="$image_root/$project_path"
    test -d "$source_path" || { echo "Image project missing: $project_path" >&2; exit 1; }
    mkdir -p "$(dirname "$project_path")"
    ln -s "$source_path" "$project_path"
done < <(cd "$image_root" && west list -f '{path}')
west list -f '{name} {path}'
python -c 'import yaml, elftools, west'
echo "Prebuilt Zephyr: $(git -C zephyr rev-parse --short HEAD)"

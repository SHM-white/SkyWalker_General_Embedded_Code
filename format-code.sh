find . \( -name .git -o -name build -o -name .venv \) -prune -o -type f \
  \( -name '*.c' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \) -print0 \
| xargs -0 clang-format -style=file -i
git --no-pager diff --stat
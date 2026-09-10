# 代码行宽与 ClangFormat 配置指南

## 结论

本项目建议把 C/C++ 行宽设为 120 列，并要求 Codex 和 clang-format 遵守同一套规则：

- 120 列以内的函数声明、函数定义、函数调用和赋值语句保持一行。
- 超过 120 列时才换行，换行后尽可能在同一行容纳多个参数，不要默认“一个参数一行”。
- 函数返回类型与函数名不拆开。
- 保留项目现有的 4 空格缩进和 Linux 大括号风格。

如果你的编辑器宽并且更偏好少换行，可以把下文的 `ColumnLimit: 120` 改成 `ColumnLimit: 140`。不建议设为 `0`；`0` 表示几乎不限行宽，长日志、宏和复杂表达式会变得难以阅读。

## 两层配置的关系

```text
AGENTS.md 中的书写规则
        │
        ▼
Codex 生成尽量符合行宽的代码
        │
        ▼
项目根目录 .clang-format
        │
        ▼
IDE 保存或手动格式化，统一新旧代码
```

`AGENTS.md` 约束 Codex 在创建代码时如何排版；`.clang-format` 是可重复执行的最终格式规则。只配其中一层也能工作，两层都配置时最稳定。

## 第一步：约束 Codex 的输出

### 仅影响当前项目

由于本项目根目录已有 `AGENTS.md`，可将下面内容追加到该文件：

```markdown
## C/C++ code formatting

- Use a maximum line width of 120 columns.
- Keep function declarations and definitions on one physical line when they fit within 120 columns.
- Keep simple assignments and function calls on one physical line when they fit within 120 columns.
- Do not wrap code merely because it approaches 80 columns.
- When a signature or call exceeds 120 columns, pack as many parameters as reasonably fit on each continuation line; do not place every parameter on its own line by default.
- Never split a function's return type from its name.
- Follow the repository-root `.clang-format` file for C and C++ formatting.
```

Codex 会在新会话开始时重新读取项目指令，因此修改后应新建一个 Codex 会话。

### 影响所有项目

将同一段规则写入 `~/.codex/AGENTS.md`。全局规则会被每个项目继承，而项目或子目录中更近的 `AGENTS.md` / `AGENTS.override.md` 可以覆盖它。

自检命令：

```bash
codex --ask-for-approval never "Summarize the active code-formatting instructions."
```

预期：Codex 明确复述“120 列以内不换行”以及“参数尽量装入同一行”。

## 第二步：在项目根目录新建 `.clang-format`

下列配置适合本项目当前的 C17/C++ 混合源码、4 空格缩进和函数大括号换行风格：

```yaml
---
Language: Cpp
BasedOnStyle: LLVM

ColumnLimit: 120
IndentWidth: 4
ContinuationIndentWidth: 4
TabWidth: 4
UseTab: Never

BreakBeforeBraces: Linux
SpaceBeforeParens: ControlStatements
PointerAlignment: Right
DerivePointerAlignment: false

BinPackArguments: true
BinPackParameters: true
AllowAllArgumentsOnNextLine: true
AllowAllParametersOfDeclarationOnNextLine: true

PenaltyBreakBeforeFirstCallParameter: 100000
PenaltyBreakAssignment: 100000
PenaltyReturnTypeOnItsOwnLine: 1000000

AllowShortFunctionsOnASingleLine: None
AllowShortIfStatementsOnASingleLine: Never
AllowShortLoopsOnASingleLine: false

AlignAfterOpenBracket: Align
AlignOperands: Align
AlignConsecutiveAssignments: false
AlignConsecutiveDeclarations: false

SortIncludes: false
IncludeBlocks: Preserve
ReflowComments: false
FixNamespaceComments: false
...
```

关键项说明：

- `ColumnLimit` 决定什么时候才考虑换行。
- `BinPackArguments` 和 `BinPackParameters` 防止超长时默认每个参数占一行。
- 三个 `Penalty...` 配置让格式化器尽量保留函数首行、赋值和返回类型。它们是排版偏好，不会让真正超过 120 列的代码无限变长。
- `SortIncludes: false` 避免格式化时意外改动 Zephyr 头文件分组。
- `ReflowComments: false` 避免自动重排注释段落。

## 第三步：VS Code 使用项目配置

安装 Microsoft C/C++ 扩展后，在项目的 `.vscode/settings.json` 中加入以下键，保留文件中已有的其他键：

```json
{
    "C_Cpp.clang_format_style": "file",
    "C_Cpp.clang_format_fallbackStyle": "none",
    "[c]": {
        "editor.defaultFormatter": "ms-vscode.cpptools",
        "editor.formatOnSave": true
    },
    "[cpp]": {
        "editor.defaultFormatter": "ms-vscode.cpptools",
        "editor.formatOnSave": true
    }
}
```

注意：JSON 对象中已有的最后一个键后面需要先加逗号，再粘贴新键；不要用上面整个示例直接覆盖项目现有设置。

## 格式化与自检

先查看格式化器版本：

```bash
clang-format --version
```

只格式化当前文件时，在 VS Code 中执行“Format Document”，或在终端执行：

```bash
clang-format -i include/drivers/motor/dm_motor.hpp
clang-format -i drivers/motor/dm/dm_motor.cpp
```

在对整个仓库格式化前，先用单文件确认效果。全库格式化会造成大量与功能无关的 diff，应当单独提交。

格式化后检查：

```bash
git diff --check
git diff -- include/drivers/motor/dm_motor.hpp drivers/motor/dm/dm_motor.cpp
```

预期结果示例：

```cpp
int setPositionVelocity(const struct device *dev, float position_rad, float velocity_limit_rad_s);

const std::uint64_t now_ms = static_cast<std::uint64_t>(k_uptime_get());
```

它们都小于 120 列，因此应保持一行。当一个函数签名确实超长时，预期结果是少量续行，而不是七八行的一参数一行布局。

## 兼容性与故障排查

- 如果 clang-format 报某个配置键不支持，先升级到较新版本；若暂时不能升级，可先删除报错的非核心键。核心项是 `ColumnLimit`、`BinPackArguments`、`BinPackParameters`、`IndentWidth` 和 `BreakBeforeBraces`。
- 如果 VS Code 格式化后没有变化，检查 `.clang-format` 是否位于仓库根目录，并检查当前文件的默认格式化器是否为 Microsoft C/C++。
- 如果 Codex 仍然使用旧风格，新建会话并让它复述当前加载的格式化指令。
- 若使用其他格式化扩展（例如 Clang-Format 独立扩展），确保它也选择 `file` 风格，否则可能忽略项目根目录配置。

## 最终检查清单

- [ ] 决定使用 120 列还是 140 列。
- [ ] 将书写规则放入项目或全局 `AGENTS.md`。
- [ ] 在项目根目录新建 `.clang-format`。
- [ ] 使用一个 C/C++ 文件试格式化并查看 diff。
- [ ] 确认短函数签名和短赋值不再被拆分。
- [ ] 将全库格式化作为单独变更处理。

## 本次边界

本仓库当前的“古法编程模式”要求业务源码、项目配置和 `AGENTS.md` 保持只读，因此本次只写入这份操作指南，没有新建 `.clang-format`、没有改动 `.vscode/settings.json` 或 `AGENTS.md`，也没有格式化任何源码。

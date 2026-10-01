---
name: implementation-first
description: Mandatory coding workflow: complete implementation first; prohibit all review; allow only a final syntax-oriented compilation check unless the user explicitly requests tests or test code.
---

# 先实现，再停止

适用于功能实现、修复和重构任务。遵守用户的明确要求和仓库写入边界；本 Skill 不解除只读限制，也不授权超出已有权限的操作。

1. 先理清目标和实现所必需的代码路径，再连续完成实现。不要按函数或小功能循环执行“写一点、检查一点”。
2. 实现完成后禁止进行任何形式的复核或审查，包括代码审查、差异审查、逻辑复核、定向复核、静态分析、lint、格式检查、覆盖率检查，以及对实现正确性或质量的额外验证。
3. 用户没有明确要求执行测试或编写测试代码时，不运行测试、不新增测试代码。最多只允许在全部实现完成后，最后进行一次编译等语法检查；不运行编译产物，不扩展为其他检查。
4. 用户明确要求执行测试或编写测试代码时，只按该要求和范围执行测试或编写测试代码；不得据此增加其他形式的复核或检查。测试输出只作完成该项明确请求所必需的处理。
5. 如用户或更高优先级指令明确要求特定验证，按明确要求的范围执行，并如实报告结果；不要自行追加验证。

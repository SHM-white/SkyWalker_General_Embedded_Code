---
name: implementation-first
description: Mandatory coding workflow: finish functional implementation before brief checks; avoid fine-grained tests unless the user explicitly asks for them.
---

# 先实现，再检查

适用于功能实现、修复和重构任务。遵守用户的明确要求、仓库规则和授权边界；本 Skill 不解除只读限制，也不跳过项目强制门禁。

1. 先理清目标和必要代码路径，再连续完成功能代码。不要按函数或小功能循环执行“写一点、测一点”。
2. 功能代码完成前不运行测试，不新增针对每个函数、分支或小功能的细粒度测试。可以只读现有测试以理解预期行为。
3. 完成实现后，进行一轮简要的语法、类型、构建可行性和关键逻辑检查。发现具体问题时修复，并只对该问题做必要的定向复核。
4. 如需新增测试，最多新增覆盖完整模块或主要流程的功能测试；不要把任务扩展成逐项细化测试。
5. 用户明确要求细化测试时，按该要求安排相应测试。用户或项目明确要求的其他验证门禁也须遵守，并如实报告未执行的检查。

# SkyWalker repository instructions

## Project skill trigger

- Before an implementation, fix, or refactoring task that may require writing code, read `.agents/skills/ancient-programming/SKILL.md` and determine whether it applies.
- Do not trigger the Skill for planning-only requests, implementation plans, task breakdowns, or design discussions.
- If the user explicitly authorizes modifying workspace files (for example, “直接实现”“直接修复”“允许修改代码”), treat that as sufficient permission to edit within the stated scope; the user does not need to say “禁用古法编程模式”.
- If no direct-edit authorization is given, retain the Skill's read-only and `docs/dev/` guide boundary for implementation work.
- Any authorization is task-scoped and must be reassessed on the next task.

## Mandatory implementation workflow

- Before any coding, feature, fix, or refactoring task, read and follow `.agents/skills/implementation-first/SKILL.md`.
- Apply it unconditionally unless the user explicitly requests fine-grained tests; in that case, follow the requested test detail.
- When the ancient-programming mode is active, reflect the implementation-first order in the Markdown guide rather than editing business files.

## Repository write boundary

- Treat business source code, headers, configuration, tests, scripts, devicetree files, and build files as read-only.
- For implementation requests, inspect the repository and write a hand-holding Markdown implementation guide under `docs/dev/`; do not apply the implementation.
- Do not run commands that are expected to generate or overwrite workspace artifacts. Put build and test commands in the guide for the user to run.
- Only update this instruction file, the project Skill, or its trigger metadata when the user explicitly requests changes to the ancient-programming policy.

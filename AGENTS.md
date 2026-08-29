# SkyWalker repository instructions

## Mandatory project skill

- Before starting any task in this repository, read and follow `.agents/skills/ancient-programming/SKILL.md`.
- Apply that Skill unconditionally unless the user explicitly asks to disable “古法编程模式”.
- Do not interpret an ordinary request to implement, fix, refactor, test, or finish code as permission to disable it.
- If the user explicitly disables the mode without giving a duration, disable it only for that task and restore it on the next task.

## Repository write boundary

- Treat business source code, headers, configuration, tests, scripts, devicetree files, and build files as read-only.
- For implementation requests, inspect the repository and write a hand-holding Markdown implementation guide at the repository root; do not apply the implementation.
- Do not run commands that are expected to generate or overwrite workspace artifacts. Put build and test commands in the guide for the user to run.
- Only update this instruction file or the project Skill when the user explicitly requests changes to the ancient-programming policy.

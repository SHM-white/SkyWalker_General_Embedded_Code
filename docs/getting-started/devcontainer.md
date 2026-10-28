# VS Code 开发容器

环境配方随源码维护；初始化后的镜像存放在本项目关联的 GitHub Container Registry（GHCR）Package。换设备拉取镜像即可，不需要再次安装 SDK、执行 `west update` 或安装 Python 依赖。业务源码通过 bind mount 挂载，修改直接保存在本机仓库。

## 第一次使用 / 换设备

1. 安装 Docker（Windows 推荐 Docker Desktop + WSL2）和 VS Code 的 **Dev Containers** 扩展 `ms-vscode-remote.remote-containers`。
2. 克隆此仓库并切换到包含 `.devcontainer/` 的分支。Windows 推荐把仓库放在 WSL 的 Linux 文件系统，例如 `~/skywalker_code`，再执行 `code .`。
3. 确认 Actions 的 **Publish development container** 已成功。镜像地址位于 `.devcontainer/devcontainer.json` 的 `image`；可提前 `docker pull <image>`。
4. 在 VS Code 命令面板执行 **Dev Containers: Reopen in Container**。容器直接拉取预构建镜像。
5. 第一次连接需要联网下载与本机 VS Code 匹配的 Server 和扩展。等待 C/C++ 安装完成；容器内终端执行 `code --list-extensions --show-versions`，应包含 `ms-vscode.cpptools@1.17.4`。

无需把整个 west 工作区放进本机仓库；容器内固定布局为：

- `/opt/skywalker/skywalker_code`：本机项目目录。
- `/opt/skywalker/.west`、`zephyr`、`modules`、`bootloader` 等：镜像内预初始化的 west 工作区。
- `/opt/venv`：Python 环境，已加入 PATH。
- `/opt/zephyr-sdk`：Zephyr SDK，已加入环境变量和 OpenOCD PATH。

仅 `skywalker_code` 的源码和其中 `build/` 随本机保留。手工改动容器内其他路径后，重建容器会回到镜像状态；需要长期保留的环境变更应修改 Dockerfile 后重新发布。

## 版本和扩展锁定

| 项目 | 固定方式 |
|---|---|
| 系统 | Ubuntu 24.04，发布镜像保存实际系统依赖 |
| Zephyr / modules | 仓库 `west.yml` 的 commit 与导入 manifest |
| SDK | 读取该 Zephyr commit 的 `SDK_VERSION`（当前 1.0.1），仅安装 ARM GNU 工具链和 host tools |
| west / CMake | 1.5.0 / 3.31.6 |
| C/C++ | `ms-vscode.cpptools@1.17.4`，每次连接按指定版本安装并核对 |
| 镜像 | Dockerfile + west.yml 内容生成 `env-<hash>` 标签，不使用 latest |

镜像保留 `/opt/skywalker-image-west-frozen.yml` 和 `/opt/skywalker-image-python.txt` 供查询实际依赖。系统包和 Python 间接依赖没有逐项锁到下载快照；同一个已发布镜像保持环境一致，重新构建可能获取新的间接依赖。需要精确复用时，将 `image` 改为 Actions 构建输出的 `ghcr.io/…@sha256:…` digest，并相应更新 workflow 的地址检查。

指定版本安装会将扩展固定在该版本；`postAttachCommand` 还会在复用容器时修正手工升级带来的漂移。不安装 C/C++ Extension Pack，避免它另行引入扩展。若本机 VS Code 用户设置强制自动安装其他 C/C++ 版本，请关闭该扩展的 **Auto Update**，必要时在本机用户设置中设 `"extensions.autoUpdate": "off"`（旧版 VS Code 使用 `false`）。自动更新是应用级设置，不能依赖仓库 `.vscode/settings.json` 或容器 remote settings 来关闭。不要手工点 C/C++ 的升级按钮；手工升级后可重新连接容器恢复 1.17.4。

目前发布 `linux/amd64` 镜像，适用于常见 x86 Windows/WSL/Linux 设备。ARM64/macOS Apple Silicon 通过 Docker 的 amd64 模拟运行，速度可能较慢；尚未发布原生 ARM64 镜像。

## 编译与 IntelliSense

在容器内仓库目录执行：

```bash
west build -p always -b dm_mc02/stm32h723xx -d build/control samples/control
west build -p always -b dm_mc02/stm32h723xx -d build/sentry_chassis applications/sentry_chassis
west build -p always -b dm_mc02/stm32h723xx -d build/sentry_gimbal applications/sentry_gimbal
```

默认 IntelliSense 读取 `build/control/compile_commands.json`。切换目标时，在 VS Code 的 C/C++ 设置中将 **Compile Commands** 改为相应构建目录的 `compile_commands.json`。先编译一次才能获得生成头文件；不要把整个 Zephyr 和 modules 加入递归 includePath。容器只推荐 C/C++ 与 Cortex-Debug，现有依赖 Zephyr IDE 的任务仍可按需使用，但默认不预装 Zephyr IDE。

## 烧录、串口与调试

容器已提供 SDK 自带 OpenOCD，但 USB 调试器不会自动进入 Docker。默认容器不申请 privileged：

- Linux：按实际硬件为容器添加 `--device=/dev/bus/usb/...`（USB 探针）或 `--device=/dev/ttyACM0`（串口），并配置宿主机 udev / 对应设备组；重新插拔后设备号可能改变，需更新映射并重建容器。
- Windows/WSL2：先用 usbipd-win 将探针附加到运行 Docker 的 WSL 环境，再配置 Docker 设备映射。
- 不转发硬件时，可在容器编译后，使用宿主机的烧录工具烧录本机 `build/<目标>/zephyr/zephyr.hex`。

已配置硬件透传后可执行 `west flash -d build/control`。原 `.vscode/launch.json` 包含宿主机 HOME 下的 SDK 路径以及 Zephyr IDE 命令，不能直接当作容器的 F5 配置。容器可先使用 `west debug -d build/control`；Cortex-Debug 配置需使用 `/opt/zephyr-sdk/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-gdb` 和 PATH 中的 OpenOCD。

## 发布与维护

首次加入配置以及后续修改 `.devcontainer/` 或 `west.yml` 时，push 会触发 **Publish development container**，用仓库 `GITHUB_TOKEN` 推送 GHCR，无需保存个人令牌。PR 事件只构建，不登录或推送；源码编译由项目原有流程负责。

1. 修改 Dockerfile 或 `west.yml` 后执行 `bash .devcontainer/image-tag.sh`。
2. 将 `.devcontainer/devcontainer.json` 的 `image` 标签改为输出的 `env-…`，与配方一起提交；workflow 会检查是否一致。
3. push 后等待 Actions 成功，再让其他设备拉取 / 重建容器。`west.yml` 与镜像不匹配时，初始化会直接报错，不会悄悄联网更新依赖。
4. 镜像首次发布默认是 **private**。使用者需先在宿主机 `docker login ghcr.io -u <GitHub用户名>`，以有 `read:packages` 权限的凭据登录；不要将凭据提交到仓库。若需要所有人免登录拉取，由仓库所有者在 GitHub → Profile → Packages → 本镜像 → Package settings 中改为 **public**。
5. 配方未变时应复用已有镜像；手工重跑 workflow 会重建同标签，需精确一致时使用镜像 digest。镜像通过 OCI source label 关联本源码仓库，镜像层不加入 Git 历史。

本地手工构建作为 GHCR 不可用时的备用方式（第一次仍需下载全部依赖）：

```bash
image="ghcr.io/shm-white/skywalker_general_embedded_code-dev:$(bash .devcontainer/image-tag.sh)"
docker build --platform linux/amd64 -f .devcontainer/Dockerfile -t "$image" .
```

构建后执行 **Dev Containers: Reopen in Container**，即可使用本地同名镜像。

参考：[VS Code Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers)、[Zephyr SDK](https://docs.zephyrproject.org/latest/develop/toolchains/zephyr_sdk.html)、[GHCR 权限与拉取](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry)。

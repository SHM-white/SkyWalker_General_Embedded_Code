# syntax=docker/dockerfile:1
# ARM firmware environment; native_sim CI keeps its separate image.
FROM ubuntu:24.04
LABEL org.opencontainers.image.source="https://github.com/SHM-white/SkyWalker_General_Embedded_Code"
LABEL org.opencontainers.image.description="SkyWalker Zephyr ARM development environment"
ENV DEBIAN_FRONTEND=noninteractive \
    VIRTUAL_ENV=/opt/venv \
    ZEPHYR_BASE=/opt/skywalker/zephyr \
    ZEPHYR_TOOLCHAIN_VARIANT=zephyr
ENV PATH="/opt/venv/bin:${PATH}"
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
      bash ca-certificates curl git openssh-client sudo build-essential \
      ninja-build device-tree-compiler gperf ccache dfu-util file wget \
      python3 python3-venv python3-dev python3-tk pkg-config \
      xz-utils unzip libusb-1.0-0 libncurses-dev libsdl2-dev \
    && rm -rf /var/lib/apt/lists/* \
    && if id ubuntu &>/dev/null; then userdel -r ubuntu; fi \
    && groupadd --gid 1000 vscode \
    && useradd --uid 1000 --gid vscode --groups dialout,plugdev --create-home --shell /bin/bash vscode \
    && echo 'vscode ALL=(root) NOPASSWD:ALL' > /etc/sudoers.d/vscode \
    && chmod 0440 /etc/sudoers.d/vscode \
    && python3 -m venv /opt/venv \
    && python -m pip install --no-cache-dir 'west==1.5.0' 'cmake==3.31.6'
WORKDIR /opt/skywalker
COPY west.yml skywalker_code/west.yml
RUN west init -l skywalker_code \
    && west update --narrow -o=--depth=1 \
    && python -m pip install --no-cache-dir \
         -r zephyr/scripts/requirements-base.txt \
         -r zephyr/scripts/requirements-build-test.txt \
         -r zephyr/scripts/requirements-run-test.txt \
    && west sdk install --version "$(cat zephyr/SDK_VERSION)" \
         --install-dir /opt/zephyr-sdk --gnu-toolchains arm-zephyr-eabi \
    && cp skywalker_code/west.yml /opt/skywalker-image-west.yml \
    && west manifest --freeze > /opt/skywalker-image-west-frozen.yml \
    && python -m pip freeze > /opt/skywalker-image-python.txt \
    && chown -R vscode:vscode /opt/skywalker /opt/venv
ENV ZEPHYR_SDK_INSTALL_DIR=/opt/zephyr-sdk
ENV PATH="/opt/zephyr-sdk/hosttools/sysroots/x86_64-pokysdk-linux/usr/bin:${PATH}"
USER vscode
RUN west zephyr-export
WORKDIR /opt/skywalker/skywalker_code
CMD ["sleep", "infinity"]

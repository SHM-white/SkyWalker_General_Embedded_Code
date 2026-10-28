# Reusable ARM firmware CI toolchain; all project sources come from the PR checkout.
# The upstream Zephyr revision pins SDK_VERSION=1.0.1.
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
ENV VIRTUAL_ENV=/opt/venv
ENV PATH="/opt/venv/bin:$PATH"
ENV ZEPHYR_TOOLCHAIN_VARIANT=zephyr
ENV ZEPHYR_SDK_INSTALL_DIR=/opt/toolchains/zephyr-sdk-1.0.1
ENV ZEPHYR_BASE=/opt/skywalker-zephyr/zephyr

RUN apt-get update && apt-get install -y --no-install-recommends \
      bash ca-certificates git build-essential cmake ninja-build ccache \
      device-tree-compiler gperf python3 python3-venv python3-dev \
      pkg-config file xz-utils curl wget \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m venv /opt/venv \
    && python -m pip install --no-cache-dir 'west==1.5.0'

# Install only the ARM cross compiler and host tools, not every SDK target.
# Verify the release checksum; do not put any credentials into this image.
ARG SDK_VERSION=1.0.1
RUN set -eux; \
    mkdir -p /opt/toolchains; \
    cd /opt/toolchains; \
    sdk="zephyr-sdk-${SDK_VERSION}_linux-x86_64_minimal.tar.xz"; \
    base="https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${SDK_VERSION}"; \
    curl -fL --retry 3 -o "$sdk" "$base/$sdk"; \
    curl -fL --retry 3 -o sha256.sum "$base/sha256.sum"; \
    sha256sum --check --ignore-missing sha256.sum; \
    tar xf "$sdk"; \
    rm "$sdk" sha256.sum; \
    "zephyr-sdk-${SDK_VERSION}/setup.sh" -t arm-zephyr-eabi -h -c

# Install the immutable west projects once per manifest/SDK version.
WORKDIR /opt/skywalker-zephyr
COPY west.yml /opt/skywalker-zephyr/skywalker_code/west.yml
RUN west init -l skywalker_code \
    && west update --narrow -o=--depth=1 \
    && west zephyr-export \
    && python -m pip install --no-cache-dir \
         -r zephyr/scripts/requirements-base.txt \
         -r zephyr/scripts/requirements-build-test.txt \
         -r zephyr/scripts/requirements-run-test.txt

# Pinned native_sim (host GCC) CI image; ARM firmware builds use a separate SDK.
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
ENV VIRTUAL_ENV=/opt/venv
ENV PATH="/opt/venv/bin:$PATH"
ENV ZEPHYR_TOOLCHAIN_VARIANT=host/gnu
RUN apt-get update && apt-get install -y --no-install-recommends \
      bash ca-certificates git build-essential cmake ninja-build \
      device-tree-compiler gperf python3 python3-venv python3-dev \
      pkg-config file xz-utils \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m venv /opt/venv \
    && python -m pip install --no-cache-dir 'west==1.5.0'
WORKDIR /opt/skywalker-zephyr
COPY west.yml /opt/skywalker-zephyr/skywalker_code/west.yml
RUN west init -l skywalker_code \
    && west update --narrow -o=--depth=1 \
    && west zephyr-export \
    && python -m pip install --no-cache-dir \
         -r zephyr/scripts/requirements-base.txt \
         -r zephyr/scripts/requirements-build-test.txt \
         -r zephyr/scripts/requirements-run-test.txt

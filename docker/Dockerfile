FROM python:3.12

# Test firmware is built from Arduino sketches, and every board core ships the
# toolchain it needs, so no distribution Arm compiler is required.  Pinned so the
# image rebuilds to the same cores; bump both together.
ARG ARDUINO_CLI_VERSION=1.5.1
ARG STM32_CORE_VERSION=3.0.0

# update
RUN apt-get update && apt-get -yq dist-upgrade \
    && DEBIAN_FRONTEND="noninteractive" apt-get install -yq --no-install-recommends \
        build-essential wget curl git nano ca-certificates socat gdb-multiarch \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd -g 1001 onelife && useradd -u 1001 -g onelife -m onelife && echo "onelife:onelife" | chpasswd

WORKDIR /home/onelife

COPY README.md MANIFEST.in setup.py requirements.txt requirements-dev.txt ./
COPY src ./src

# socat exposes the simulated UART/SPI as a pty pair (the default bridge on
# POSIX); gdb-multiarch is used for the GDB stub.  Neither is required on
# Windows, where the TCP bridge is used instead.
RUN pip install -U pip && pip install --no-cache-dir -r requirements.txt
# The development image exists to run the suite, so pytest/pyflakes come with it.
RUN pip install --no-cache-dir -r requirements-dev.txt
RUN pip install --no-cache-dir ./ && rm -rf ./src

# The Arduino CLI builds the test firmware (tests/firmware/build.py).
RUN curl -fsSL -o /tmp/arduino-cli.tar.gz \
        "https://github.com/arduino/arduino-cli/releases/download/v${ARDUINO_CLI_VERSION}/arduino-cli_${ARDUINO_CLI_VERSION}_Linux_64bit.tar.gz" \
    && tar -xzf /tmp/arduino-cli.tar.gz -C /tmp arduino-cli \
    && install -m 0755 /tmp/arduino-cli /usr/local/bin/arduino-cli \
    && rm -f /tmp/arduino-cli.tar.gz /tmp/arduino-cli \
    && arduino-cli version

# Keep the cores and the sketches out of $HOME, so `docker exec -u root` sees the
# same installation as the onelife user.
ENV ARDUINO_DIRECTORIES_DATA=/home/onelife/.arduino15 \
    ARDUINO_DIRECTORIES_DOWNLOADS=/home/onelife/.arduino15/staging \
    ARDUINO_DIRECTORIES_USER=/home/onelife/Arduino \
    ARDUINO_METRICS_ENABLED=false

USER onelife

# Pre-install the cores for the three boards the test firmware targets, plus the
# STM32duino index they come from.  This is the big layer (~1.5 GB, mostly the
# Arm toolchains); it sits last so editing the source never invalidates it.
RUN arduino-cli config init --overwrite \
    && arduino-cli config set board_manager.additional_urls \
        https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json \
    && arduino-cli core update-index \
    && arduino-cli core install arduino:sam \
    && arduino-cli core install arduino:samd \
    && arduino-cli core install "STMicroelectronics:stm32@${STM32_CORE_VERSION}" \
    && arduino-cli core list \
    && rm -rf /home/onelife/.arduino15/staging/*

CMD [ "bash" ]

FROM python:3.12

# update
RUN apt-get update && apt-get -yq dist-upgrade \
    && DEBIAN_FRONTEND="noninteractive" apt-get install -yq --no-install-recommends \
        build-essential wget git nano \
        socat gdb-multiarch \
   && apt-get clean \
   && rm -rf /var/lib/apt/lists/*

RUN groupadd -g 1001 onelife && useradd -u 1001 -g onelife -m onelife && echo "onelife:onelife" | chpasswd

WORKDIR /home/onelife

COPY README.md MANIFEST.in setup.py requirements.txt ./
COPY src ./src

# socat exposes the simulated UART/SPI as a pty pair (the default bridge on
# POSIX); gdb-multiarch is used for the GDB stub.  Neither is required on
# Windows, where the TCP bridge is used instead.
RUN pip install -U pip && pip install --no-cache-dir -r requirements.txt
RUN pip install --no-cache-dir ./ && rm -rf ./src

USER onelife

CMD [ "bash" ]

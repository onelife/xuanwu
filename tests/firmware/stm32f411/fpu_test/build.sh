#!/usr/bin/env bash
# Build the floating-point acceptance firmware.
#
# The toolchain is taken from $CC, then from arm-none-eabi-gcc on PATH, then from
# the copy the Arduino STM32 core installs -- which is why the development image
# needs no distribution Arm compiler:
#   apt-get install gcc-arm-none-eabi          # or
#   arduino-cli core install STMicroelectronics:stm32
# The resulting ELF is committed so the test suite does not need a toolchain.
set -eu

HERE="$(cd "$(dirname "$0")" && pwd)"

find_cc() {
    if [ -n "${CC:-}" ]; then
        echo "$CC"
        return
    fi
    if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
        command -v arm-none-eabi-gcc
        return
    fi
    # Where the Arduino cores keep their toolchains.  Ask arduino-cli first: it is
    # the authority, and it also covers running as another user than the one the
    # cores were installed for.
    local dirs="" dir data pattern candidate
    if command -v arduino-cli >/dev/null 2>&1; then
        dirs="$(arduino-cli config get directories.data 2>/dev/null || true)"
    fi
    dirs="$dirs ${ARDUINO_DIRECTORIES_DATA:-} ${HOME:-/root}/.arduino15"
    for data in $dirs; do
        [ -d "$data" ] || continue
        for pattern in \
            "$data/packages/STMicroelectronics/tools/xpack-arm-none-eabi-gcc"/*/bin/arm-none-eabi-gcc \
            "$data/packages/STM32/tools/xpack-arm-none-eabi-gcc"/*/bin/arm-none-eabi-gcc \
            "$data/packages/arduino/tools/arm-none-eabi-gcc"/*/bin/arm-none-eabi-gcc
        do
            for candidate in $pattern; do
                if [ -x "$candidate" ]; then
                    echo "$candidate"
                    return
                fi
            done
        done
    done
    echo ""
}

CC="$(find_cc)"
if [ -z "$CC" ]; then
    echo "error: no arm-none-eabi-gcc found." >&2
    echo "  Set CC=/path/to/arm-none-eabi-gcc, install gcc-arm-none-eabi, or" >&2
    echo "  install the Arduino STM32 core (see docs/manual.zh-CN.md)." >&2
    exit 1
fi
echo "using $CC ($("$CC" -dumpversion))"

"$CC" \
    -mcpu=cortex-m4 \
    -mthumb \
    -mfloat-abi=hard \
    -mfpu=fpv4-sp-d16 \
    -O0 \
    -g3 \
    -ffreestanding \
    -fno-builtin \
    -Wall \
    -Wextra \
    -nostdlib \
    -nostartfiles \
    -T "$HERE/linker.ld" \
    -Wl,-Map="$HERE/fpu_test.map" \
    -o "$HERE/fpu_test.elf" \
    "$HERE/fpu_test.c"

"${CC%-gcc}-size" "$HERE/fpu_test.elf"
echo "built $HERE/fpu_test.elf"

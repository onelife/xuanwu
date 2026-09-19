#!/usr/bin/env bash
# Build the floating-point acceptance firmware.
#
# Requires arm-none-eabi-gcc (Debian/Ubuntu: apt-get install gcc-arm-none-eabi).
# The resulting ELF is committed so the test suite does not need a toolchain.
set -eu

HERE="$(cd "$(dirname "$0")" && pwd)"
CC="${CC:-arm-none-eabi-gcc}"

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

# -*- coding: utf-8 -*-

"""Find out what a chip description is still missing.

Booting an unfamiliar firmware leaves a trail of MMIO accesses that no peripheral
model claimed: the firmware touched a register, the simulator quietly returned the
backing buffer, and nothing said so.  The memory controller counts those accesses,
which turns "why does it hang?" into a work list.

    python examples/unclaimed_io.py
    python examples/unclaimed_io.py 1000000      # run longer first

Compare the addresses with the chip's reference manual, then add the peripheral to
the YAML and a model to ``arch/vendor`` -- see docs/add-a-chip.md.
"""

import logging
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from xuanwu import XuanWu  # noqa: E402

CHIP = "stm32f411"
FIRMWARE = REPO_ROOT / "tests/firmware/stm32f411/Blink_m4.ino.elf"
INSTRUCTIONS = 200_000

# A few STM32F4 blocks, to show what the addresses mean.
KNOWN_BLOCKS = (
    (0x4002_3800, 0x4002_3C00, "FLASH interface"),
    (0x4002_3C00, 0x4002_4000, "FLASH interface"),
    (0x4000_7000, 0x4000_7400, "TIM"),
    (0x4001_2000, 0x4001_2400, "ADC"),
    (0x4001_3000, 0x4001_3400, "SPI"),
    (0x4000_5400, 0x4000_5800, "USART"),
)


def describe(address: int) -> str:
    for low, high, name in KNOWN_BLOCKS:
        if low <= address < high:
            return name
    return "?"


def main(argv) -> int:
    steps = int(argv[1]) if len(argv) > 1 else INSTRUCTIONS
    logging.disable(logging.CRITICAL)

    device = XuanWu(CHIP, str(FIRMWARE))
    device.reset()
    device.run(count=steps)

    device.mem.show_unclaimed()  # the table as the simulator prints it

    records = device.mem.unclaimed_accesses()
    print(f"\nafter {steps:,} instructions, grouped for a work list:")
    for address, size, count in records:
        print(f"  0x{address:08x}  {size} byte(s)  x{count:<6d} {describe(address)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))

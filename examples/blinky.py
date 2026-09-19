# -*- coding: utf-8 -*-

"""Run a bundled firmware image on the simulated MCU.

Usage::

    python examples/blinky.py                 # STM32F411, no host dependencies
    python examples/blinky.py sampled         # SAM3X8E (needs socat)
    python examples/blinky.py sampled 300000  # ... for 300k instructions
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from xuanwu import XuanWu  # noqa: E402

DEMOS = {
    # name: (chip name, firmware, instructions to run)
    "blinky": ("stm32f411", REPO_ROOT / "tests/firmware/stm32f411/Blink_m4.ino.elf", 500_000),
    "sampled": ("sam3x8e", REPO_ROOT / "tests/firmware/sam3x8e/Blink_m3.ino.elf", 500_000),
}


def main(argv):
    name = argv[1] if len(argv) > 1 else "blinky"
    if name not in DEMOS:
        print(f"unknown demo {name!r}; choose from {', '.join(DEMOS)}")
        return 2
    chip, firmware, default_steps = DEMOS[name]
    steps = int(argv[2]) if len(argv) > 2 else default_steps

    # The chip can be given by name (resolved inside the installed package) or by path.
    device = XuanWu(chip, str(firmware))
    device.reset()
    print(f"chip      : {chip}")
    print(f"firmware  : {firmware.name}")
    print(f"reset     : pc=0x{device.reg.pc:08x}  msp=0x{device.reg.msp:08x}")
    device.mem.show_map()

    device.run(count=steps)
    print(f"after {steps:,} instructions: pc=0x{device.reg.pc:08x}  sp=0x{device.reg.sp:08x}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))

# -*- coding: utf-8 -*-

"""Get a bare-metal ``printf`` onto the host console with Arm semihosting.

Semihosting is the "syscall" convention for firmware with no operating system: the
guest executes ``BKPT 0xAB`` with a request code in r0 and arguments in r1, and the
debugger (here: the simulator) does the work.  That means a firmware built with the
semihosting spec files can print without a UART, a serial bridge or any wiring.

    python examples/semihosting.py

This example does not need a compiler: it pokes an eight-byte program into RAM and
runs it.

    movs r0, #4    SYS_WRITE0 -- r1 points at a NUL-terminated string
    adr  r1, #252  ... which is the string 0x100 bytes further on
    bkpt 0xab      the trap
    b    .         park
"""

import io
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from xuanwu import XuanWu  # noqa: E402
from xuanwu.backends import SemiHosting  # noqa: E402

CHIP = "stm32f411"
FIRMWARE = REPO_ROOT / "tests/firmware/stm32f411/Blink_m4.ino.elf"

CODE = 0x2001_F000  # scratch RAM in the STM32F411 map
STRING = 0x2001_F100

PROGRAM = bytes.fromhex(
    "0420"  # movs r0, #4        SYS_WRITE0
    "3fa1"  # adr  r1, #252      -> STRING
    "abbe"  # bkpt 0xab          the semihosting trap
    "fee7"  # b .                park here
)


def main() -> int:
    stream = io.StringIO()
    device = XuanWu(CHIP, str(FIRMWARE), semihosting=SemiHosting(output=stream))
    device.reset()

    device.mem.write(CODE, PROGRAM)
    device.mem.write(STRING, b"hello from semihosting\n\x00")
    device.reg.pc_t = CODE  # | 0x1: bit 0 selects Thumb

    device.run(count=3)  # movs, adr, bkpt -- the trap is serviced and stepped over
    print(f"the guest printed: {stream.getvalue()!r}")
    print(f"pc after the trap: 0x{device.reg.pc:08x} (the BKPT is behind us)")

    device.run(count=1)  # the `b .` loop, to show execution continues
    print(f"pc after the parked loop ran: 0x{device.reg.pc:08x} (it branches to itself)")

    if stream.getvalue() != "hello from semihosting\n":
        print("semihosting did not deliver the string")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

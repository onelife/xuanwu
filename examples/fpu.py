# -*- coding: utf-8 -*-

"""Floating point, and why the extended exception frame matters.

``tests/firmware/stm32f411/fpu_test`` is a bare-metal Cortex-M4F program that keeps
four values in ``s0``-``s3`` across three SysTick interrupts.  Its handler
deliberately overwrites those very registers, so the values only survive if the
exception stacked and restored the floating-point frame.

    python examples/fpu.py

The second run disables that stacking on purpose, to show what the bug looks like:
the values come back as whatever the handler left behind.
"""

import logging
import struct
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from xuanwu import XuanWu  # noqa: E402

CHIP = "stm32f411"
FIRMWARE = REPO_ROOT / "tests/firmware/stm32f411/fpu_test/fpu_test.elf"

RESULT = 0x2000_0000
PARKED = 0x2000_0044
FIELDS = [
    "magic", "sum_ok", "product_ok", "convert_ok", "mvfr0", "mvfr1", "fpccr",
    "irq_count", "a", "b", "c", "d",
]
HELD_IN_S0_TO_S3 = ("a", "b", "c", "d")


def run(disable_fp_stacking: bool = False) -> dict:
    logging.disable(logging.CRITICAL)
    device = XuanWu(CHIP, str(FIRMWARE))
    device.reset()

    if disable_fp_stacking:
        # Pretend the core never records that the FPU was in use, which is what
        # the model did before floating-point stacking existed.  This is the
        # negative control for the test above.
        push_context = device.hw.push_context

        def without_fp_frame(exception):
            device.reg.write("control", device.reg.read("control") & ~0x4)  # CONTROL.FPCA
            return push_context(exception)

        device.hw.push_context = without_fp_frame

    for _ in range(40):
        device.run(count=20_000)
        if device.mem.read_word(PARKED):
            break

    values = {}
    for index, name in enumerate(FIELDS):
        raw = device.mem.read_word(RESULT + 4 * index)
        if name in HELD_IN_S0_TO_S3:
            values[name] = struct.unpack("<f", struct.pack("<I", raw))[0]
        else:
            values[name] = raw
    values["parked"] = device.mem.read_word(PARKED)
    values["device"] = device
    return values


def show(title: str, values: dict) -> None:
    print(f"--- {title}")
    print(f"    magic=0x{values['magic']:08x}  add={values['sum_ok']} mul={values['product_ok']} "
          f"cvt={values['convert_ok']}  interrupts={values['irq_count']}")
    print(f"    MVFR0=0x{values['mvfr0']:08x}  MVFR1=0x{values['mvfr1']:08x}  FPCCR=0x{values['fpccr']:08x}")
    print(f"    values held in s0-s3: {[(name, values[name]) for name in HELD_IN_S0_TO_S3]}")


def main() -> int:
    stacked = run()
    show("with the floating-point frame stacked", stacked)
    device = stacked["device"]
    systick = device.hw.perif["systick"]
    print(f"    simulated time: {systick.elapsed_ms:.2f} ms over {systick.cycles:.0f} cycles")

    clobbered = run(disable_fp_stacking=True)
    show("with stacking disabled (the values are destroyed)", clobbered)

    expected = {"a": 1.5, "b": 2.25, "c": -3.75, "d": 100.0}
    for name, value in expected.items():
        if stacked[name] != value:
            print(f"s{name} came back as {stacked[name]}, expected {value}")
            return 1
    if clobbered["a"] == expected["a"]:
        print("the negative control did not lose the values, so the test proves nothing")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

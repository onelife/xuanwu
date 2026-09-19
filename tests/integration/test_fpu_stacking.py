# -*- coding: utf-8 -*-

"""End-to-end floating-point acceptance test.

It runs a purpose-built Cortex-M4F firmware (`tests/firmware/stm32f411/fpu_test`)
that:

* performs single-precision arithmetic and checks the results itself;
* reads the floating-point extension registers;
* holds four values in S0-S3 across several SysTick interrupts whose handler
  deliberately overwrites those very registers.

The last part is the one that matters: the values can only survive if the
exception actually stacked and restored the floating-point frame.  A negative
control run disables that stacking and asserts the values are destroyed, so the
test cannot pass vacuously.
"""

import logging
import struct

import pytest

from xuanwu import XuanWu

pytestmark = pytest.mark.integration

FIRMWARE = "tests/firmware/stm32f411/fpu_test/fpu_test.elf"
CHIP = "stm32f411"

MAGIC = 0x58575546
RESULT = 0x20000000
IRQ_COUNT = 0x20000040
PARKED = 0x20000044

FIELDS = {
    "magic": 0,
    "sum_ok": 1,
    "product_ok": 2,
    "convert_ok": 3,
    "mvfr0": 4,
    "mvfr1": 5,
    "fpccr": 6,
    "irq_count": 7,
    "a": 8,
    "b": 9,
    "c": 10,
    "d": 11,
}

HELD = {"a": 1.5, "b": 2.25, "c": -3.75, "d": 100.0}
CLOBBERED = {"a": 9.0, "b": 9.0, "c": 8.0, "d": 8.0}


def as_float(bits: int) -> float:
    return struct.unpack("<f", struct.pack("<I", bits))[0]


def run_firmware(disable_fp_stacking: bool = False):
    """Boot the firmware and return its published results."""
    logging.disable(logging.CRITICAL)
    device = XuanWu(CHIP, FIRMWARE)
    device.reset()

    if disable_fp_stacking:
        # Pretend the core never records that the FPU was in use.  This is the
        # behaviour before FP stacking existed, and the control for the test.
        original = device.hw.push_context

        def without_fp_frame(exception):
            device.reg.write("control", device.reg.read("control") & ~0x4)
            return original(exception)

        device.hw.push_context = without_fp_frame

    for _ in range(40):
        device.run(count=20_000)
        if device.mem.read_word(PARKED):
            break

    values = {name: device.mem.read_word(RESULT + 4 * offset) for name, offset in FIELDS.items()}
    values["parked"] = device.mem.read_word(PARKED)
    values["live_irq_count"] = device.mem.read_word(IRQ_COUNT)
    values["device"] = device
    return values


@pytest.fixture(scope="module")
def results():
    return run_firmware()


@pytest.fixture(scope="module")
def clobbered():
    """The same firmware with floating-point stacking disabled."""
    return run_firmware(disable_fp_stacking=True)


class TestFirmwareRuns:
    def test_it_reached_the_end(self, results):
        assert results["parked"] == 1
        assert results["magic"] == MAGIC

    def test_it_executed_floating_point(self, results):
        assert results["sum_ok"] == 1, "1.5f + 2.25f should be 3.75f"
        assert results["product_ok"] == 1, "1.5f * 2.0f should be 3.0f"
        assert results["convert_ok"] == 1, "integer to float conversion failed"

    def test_the_extension_registers_were_readable(self, results):
        assert results["mvfr0"] == 0x10110021
        assert results["mvfr1"] == 0x11000011
        assert results["fpccr"] == 0xC0000000


class TestInterruptsStackedTheFpFrame:
    def test_interrupts_actually_happened(self, results):
        assert results["live_irq_count"] >= 3, "the SysTick handler never ran"
        assert results["irq_count"] >= 3

    @pytest.mark.parametrize("name", sorted(HELD))
    def test_the_held_value_survived(self, results, name):
        value = as_float(results[name])
        assert value == HELD[name], (
            f"S{list(sorted(HELD)).index(name)} held {value} after the interrupts; "
            "the floating-point frame was not stacked and restored"
        )

    def test_control_register_fpca_is_still_set(self, results):
        # thread mode with FPCA set, which is what makes the FP frame necessary
        assert results["device"].reg.read("control") & 0x4


class TestNegativeControl:
    """Without FP stacking the same firmware must lose the values."""

    def test_the_firmware_still_runs(self, clobbered):
        assert clobbered["parked"] == 1
        assert clobbered["live_irq_count"] >= 3

    @pytest.mark.parametrize("name", sorted(CLOBBERED))
    def test_the_held_value_is_destroyed(self, clobbered, name):
        value = as_float(clobbered[name])
        assert value == CLOBBERED[name], (
            f"S{list(sorted(CLOBBERED)).index(name)} still held {value}; "
            "the test would pass even without floating-point stacking"
        )

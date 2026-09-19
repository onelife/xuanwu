# -*- coding: utf-8 -*-

"""Execution slices: the scheduler, the time base, and what a slice may not break.

Sliced execution replaced a Python callback per instruction with "run to the next
deadline", which is worth ~160x in throughput.  These tests pin down the
properties that made that safe: the slice length is exact, the time base follows
the instructions actually executed, and nothing about the exception frame changes.
"""

import pytest
from unicorn import arm_const as uc_arm

from xuanwu import XuanWu
from xuanwu.arch.base import NEVER

CODE = 0x2001_F000
SPIN = bytes.fromhex("fee7")  # b .
NOPS = bytes.fromhex("00bf") * 32


@pytest.fixture
def device(stm32f411_path, stm32f411_firmware):
    dev = XuanWu(str(stm32f411_path), str(stm32f411_firmware))
    dev.reset()
    dev.mem.write(CODE, NOPS)
    dev.reg.pc_t = CODE
    return dev


@pytest.fixture
def spinning(device):
    """A device that stays put however many instructions are executed."""
    device.mem.write(CODE, SPIN)
    device.reg.pc_t = CODE
    return device


class FakeTimed:
    """Stands in for a peripheral that has to be told how much time passed."""

    def __init__(self, deadline: int = NEVER) -> None:
        self.total = 0
        self.slices = 0
        self.deadline = deadline

    def advance(self, instructions: int) -> None:
        self.total += instructions
        self.slices += 1

    def next_deadline(self) -> int:
        return self.deadline


class TestSliceLengthIsExact:
    def test_the_time_base_advances_by_the_instructions_executed(self, spinning):
        for count in (1, 7, 100, 9_999, 10_001, 33_333):
            before = spinning.hw.perif["systick"].cycles
            spinning.run(count=count)
            after = spinning.hw.perif["systick"].cycles
            assert after - before == count, f"ran {count} instructions, time base moved {after - before}"

    def test_a_model_with_advance_sees_every_instruction_once(self, spinning):
        fake = FakeTimed()
        spinning.hw.register_timed(fake)
        spinning.run(count=25_000)
        assert fake.total == 25_000
        assert fake.slices > 1, "a run this long spans several slices"

    def test_the_pc_advances_by_the_slice_length(self, device):
        device.run(count=16)
        assert device.reg.pc == CODE + 32  # 16 two-byte instructions

    def test_run_until_stops_on_the_address(self, device):
        device.run(until=CODE + 0x10)
        assert device.reg.pc == CODE + 0x10
        # eight two-byte instructions, and the time base must agree
        assert device.hw.perif["systick"].cycles == 8

    def test_the_systick_deadline_is_never_the_only_thing_that_stops_a_run(self, device):
        # a model with no deadline of its own must not shorten the slice
        fake = FakeTimed()
        device.hw.register_timed(fake)
        assert device.hw.next_slice(10_000) == 10_000


class TestTheSchedulerAsksForTheRightSlice:
    def test_a_deadline_shortens_the_slice(self, device):
        fake = FakeTimed(deadline=100)
        device.hw.register_timed(fake)
        assert device.hw.next_slice(10_000) == 100

    def test_the_maximum_slice_bounds_it(self, device):
        assert device.hw.next_slice() == device.hw._max_slice  # noqa: SLF001 - the point of the test

    def test_a_remaining_budget_bounds_it(self, device):
        assert device.hw.next_slice(remaining=37) == 37

    def test_a_slice_never_asks_for_zero(self, device):
        # asking for zero instructions would make run() spin forever
        assert device.hw.next_slice(remaining=0) >= 1

    def test_the_systick_deadline_matches_its_period(self, device):
        systick = device.hw.perif["systick"]
        device.mem.write(0xE000E014, (99).to_bytes(4, "little"))  # RVR
        device.mem.write(0xE000E018, (0).to_bytes(4, "little"))  # CVR
        device.mem.write(0xE000E010, (1).to_bytes(4, "little"))  # ENABLE
        assert systick.next_deadline() == 100
        device.run(count=99)
        assert systick.ticks == 0
        device.run(count=1)
        assert systick.ticks == 1


class TestStackAlignment:
    """SPREALIGN is bit 9 of the stacked xPSR -- it was written as bit 11.

    Bit 11 is IT[3], so every exception corrupted the Thumb IT state of the
    restored context; a conditional branch after the return was then rejected as
    an invalid instruction.  Found by running the RT-Thread firmware under sliced
    execution, which changed which boundary the corruption landed on.
    """

    def frame_of(self, device) -> int:
        sp = device.reg.read("msp")
        return int.from_bytes(device.mem.read(sp, 4), "little")

    def test_strealign_is_bit_9(self, device):
        assert device.hw.perif["scb"].read_register("CCR") & (1 << 9), "STKALIGN resets to 1"
        device.reg.write("msp", 0x2001_0004)  # bit 2 set: alignment is forced
        device.hw.push_context(0)
        xpsr = int.from_bytes(device.mem.read(device.reg.read("msp") + 28, 4), "little")
        assert xpsr & (1 << 9), "SPREALIGN must be reported in bit 9"
        assert not xpsr & (1 << 11), "bit 11 is IT[3], not SPREALIGN"

    def test_the_frame_is_pushed_to_an_eight_byte_boundary(self, device):
        device.reg.write("msp", 0x2001_0004)
        device.hw.push_context(0)
        assert device.reg.read("msp") == 0x2000_FFE0  # (0x...004 & ~4) - 0x20

    def test_exception_return_undoes_the_alignment(self, device):
        device.reg.write("msp", 0x2001_0004)
        device.hw.push_context(0)
        exc_return = device.reg.read("lr")
        assert exc_return == 0xFFFF_FFF9
        device.hw.pop_context(device.reg.read("msp"), exc_return)
        assert device.reg.read("msp") == 0x2001_0004


class TestStaleItStateRepair:
    """A slice that ends inside a Thumb IT block leaves Unicorn's IT state behind."""

    def it_bits(self, device) -> int:
        return device.hw.it_state()

    def set_it(self, device, value: int) -> None:
        epsr = device.reg.read(uc_arm.UC_ARM_REG_EPSR)
        device.reg.write(uc_arm.UC_ARM_REG_EPSR, (epsr & ~0x0000FC00) | (value << 10))

    def test_a_stale_state_with_no_it_instruction_is_cleared(self, device):
        device.mem.write(CODE, NOPS)
        device.reg.pc_t = CODE + 8
        self.set_it(device, 0x02)
        assert self.it_bits(device) == 0x02, "writing the EPSR must reach the IT state"
        assert device.hw.repair_stale_it_state() is True
        assert self.it_bits(device) == 0

    def test_a_real_it_block_is_left_alone(self, device):
        # it eq ; then the PC sits inside the block
        device.mem.write(CODE, bytes.fromhex("0020" "0028" "08bf" "0121"))
        device.reg.pc_t = CODE + 6
        self.set_it(device, 0x02)
        assert device.hw.repair_stale_it_state() is False, "a genuine IT block must not be touched"
        assert self.it_bits(device) == 0x02

    def test_a_clean_state_is_not_repaired(self, device):
        assert self.it_bits(device) == 0
        assert device.hw.repair_stale_it_state() is False


class TestTheEngineStillWorksWithoutHooks:
    """The interrupt engine no longer runs from a UC_HOOK_CODE callback.

    A per-instruction Python callback costs ~200x, so this keeps an eye on it: the
    simulator may be slower than bare Unicorn, but not by two orders of magnitude.
    """

    def test_only_the_exception_hook_is_registered(self, device):
        assert device.hw._max_slice > 0  # noqa: SLF001 - slice size is the contract
        assert device.hw.executed == 0, "nothing has run yet"

    def test_a_run_is_not_one_python_callback_per_instruction(self, spinning):
        import time

        start = time.perf_counter()
        spinning.run(count=200_000)
        elapsed = time.perf_counter() - start
        # 0.85 M instructions/s (the per-instruction-hook speed) would be ~0.24 s;
        # anything under a tenth of that means the bulk path is being used.
        assert elapsed < 0.05, f"200k instructions took {elapsed:.3f}s -- per-instruction hook?"

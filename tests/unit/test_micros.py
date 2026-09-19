# -*- coding: utf-8 -*-

"""`micros()` has to be right, because drivers use it as a timeout.

The Arduino SAM core computes it from registers the guest reads, not from a library
call:

.. code-block:: c

    return ((count + pend) * 1000)
         + (((SysTick->LOAD - ticks) * (1048576 / (F_CPU / 1000000))) >> 20);

where ``count`` is the millisecond counter, ``ticks`` is ``SysTick->VAL`` and
``pend`` is ``SCB->ICSR.PENDSTSET | SCB->SHCSR.SYSTICKACT``.  The SAM Wire library
polls its status bits in a loop that gives up after a ``micros()`` deadline, so a
model where these registers do not move turns a timeout into a hang.
"""

import pytest

from xuanwu import XuanWu

SYSTICK_CSR = 0xE000_E010
SYSTICK_RVR = 0xE000_E014
SYSTICK_CVR = 0xE000_E018
SCB_ICSR = 0xE000_ED04
SCB_SHCSR = 0xE000_ED24

CSR_ENABLE = 1 << 0
CSR_TICKINT = 1 << 1
PENDSTSET = 1 << 26
SYSTICKACT = 1 << 11

CLOCK = 84_000_000  # the Due's core clock, as the chip description declares it
PERIOD = 84_000  # one millisecond at 84 MHz

CODE = 0x2000_F000  # inside SRAM0 of the SAM3X8E (0x20000000..0x20010000)
SPIN = bytes.fromhex("fee7")


@pytest.fixture
def device(sam3x8e_path, sam3x8e_firmware):
    """A device spinning in place, so nothing else touches SysTick.

    Running the bundled firmware here would be a mistake: the Arduino core
    configures SysTick itself, and the registers under test would be its, not the
    ones this file set up.
    """
    dev = XuanWu(str(sam3x8e_path), str(sam3x8e_firmware), hardware_options={"bridge": "none"})
    dev.reset()
    dev.mem.write(CODE, SPIN)
    dev.reg.pc_t = CODE
    dev.mem.write(SYSTICK_RVR, (PERIOD - 1).to_bytes(4, "little"))
    dev.mem.write(SYSTICK_CVR, (0).to_bytes(4, "little"))
    dev.mem.write(SYSTICK_CSR, CSR_ENABLE.to_bytes(4, "little"))
    return dev


def micros(device) -> int:
    """The Arduino SAM core's formula, evaluated on the registers the guest reads."""
    ticks = device.mem.read_word(SYSTICK_CVR)
    load = device.mem.read_word(SYSTICK_RVR)
    pending = 0
    if device.mem.read_word(SCB_ICSR) & PENDSTSET or device.mem.read_word(SCB_SHCSR) & SYSTICKACT:
        pending = 1
    millis = device.hw.perif["systick"].ticks
    scale = 1048576 // (CLOCK // 1_000_000)
    return ((millis + pending) * 1000) + (((load - ticks) * scale) >> 20)


class TestTheTimeBaseIsReadable:
    def test_the_load_value_is_whatever_the_guest_wrote(self, device):
        assert device.mem.read_word(SYSTICK_RVR) == PERIOD - 1

    def test_the_counter_counts_down_from_the_reload_value(self, device):
        device.run(count=1000)
        assert device.mem.read_word(SYSTICK_CVR) == PERIOD - 1000

    def test_micros_matches_the_cycles_executed(self, device):
        # a whole period lands exactly on the wrap, so the sub-millisecond part is 0
        device.run(count=PERIOD)
        assert device.hw.perif["systick"].ticks == 1
        assert micros(device) == 1000

        device.run(count=PERIOD // 2)
        assert 1498 <= micros(device) <= 1500, "half a period in, micros() must say so"

    def test_micros_is_monotonic_across_slices(self, device):
        seen = []
        for _ in range(20):
            device.run(count=PERIOD // 10)
            seen.append(micros(device))
        assert seen == sorted(seen), seen
        assert seen[-1] - seen[0] >= 1900, "ten slices of 100 us must add up"

    def test_a_masked_tick_still_shows_as_pending(self, device):
        """What ``pend`` in the formula is for: the tick is due but not yet taken."""
        device.mem.write(SYSTICK_CSR, (CSR_ENABLE | CSR_TICKINT).to_bytes(4, "little"))
        device.reg.write("primask", 1)  # mask it, so the engine leaves it pending
        device.run(count=PERIOD + 10)
        assert device.mem.read_word(SCB_ICSR) & PENDSTSET, "a due-but-masked tick is pending"
        assert micros(device) >= 1000

    def test_a_taken_tick_sets_the_active_bit(self, device):
        device.mem.write(SYSTICK_CSR, (CSR_ENABLE | CSR_TICKINT).to_bytes(4, "little"))
        # one instruction past the deadline: the tick has been taken and the
        # handler has only just started, so the exception is still active.
        device.run(count=PERIOD + 1)
        assert device.reg.read("ipsr") == 15, "we should be in the SysTick handler"
        assert device.mem.read_word(SCB_SHCSR) & SYSTICKACT

    def test_the_active_bit_clears_when_the_handler_returns(self, device):
        device.mem.write(SYSTICK_CSR, (CSR_ENABLE | CSR_TICKINT).to_bytes(4, "little"))
        device.run(count=PERIOD + 2000)
        # the handler has finished (and whatever it pended is running now), so
        # SysTick is no longer the active exception
        assert not device.mem.read_word(SCB_SHCSR) & SYSTICKACT

    def test_the_cycle_count_and_the_clock_agree(self, device):
        device.run(count=PERIOD * 3)
        systick = device.hw.perif["systick"]
        assert systick.cycles == PERIOD * 3
        assert systick.elapsed_ms == pytest.approx(3.0, abs=1e-6)

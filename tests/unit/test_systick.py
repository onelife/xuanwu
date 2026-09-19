# -*- coding: utf-8 -*-

"""SysTick's time base.

Unicorn does not report how long an instruction took, so the model advances the
counter one instruction at a time, scaled by ``cycles_per_instruction``.  That is
an approximation and cannot be more; what *can* be exact is the arithmetic around
it: a period is ``RVR + 1`` cycles, ``CALIB`` agrees with the configured core
clock, and the number of elapsed periods follows from the instructions executed.
"""

import pytest

from xuanwu import XuanWu

CSR_ENABLE = 1 << 0
CSR_TICKINT = 1 << 1
CSR_COUNTFLAG = 1 << 16

SYSTICK_CSR = 0xE000E010
SYSTICK_RVR = 0xE000E014
SYSTICK_CVR = 0xE000E018
SYSTICK_CALIB = 0xE000E01C

# Scratch RAM in the STM32F411 map, and a two-byte ``b .`` loop.
CODE = 0x2001_F000
SPIN = bytes.fromhex("fee7")


@pytest.fixture
def device(stm32f411_path, stm32f411_firmware):
    """A device spinning in place, so one instruction is one hook call."""
    dev = XuanWu(str(stm32f411_path), str(stm32f411_firmware))
    dev.reset()
    dev.mem.write(CODE, SPIN)
    dev.reg.pc_t = CODE
    return dev


@pytest.fixture
def systick(device):
    return device.hw.perif["systick"]


def start_counting(systick, reload: int) -> None:
    systick.write(SYSTICK_RVR, 4, reload)
    systick.write(SYSTICK_CVR, 4, 0)
    systick.write(SYSTICK_CSR, 4, CSR_ENABLE)


class TestTheCounter:
    def test_a_period_is_rvr_plus_one_cycles(self, device, systick):
        start_counting(systick, 99)
        device.run(count=100)
        assert systick.cycles == 100
        assert systick.ticks == 1

    def test_ticks_follow_the_instruction_count(self, device, systick):
        start_counting(systick, 9)  # ten cycles per period
        device.run(count=250)
        assert systick.ticks == 25

    def test_the_counter_value_counts_down(self, device, systick):
        start_counting(systick, 99)
        # A write to CVR clears it; the reload value is loaded on the next clock.
        assert systick.read(SYSTICK_CVR, 4) == 0
        device.run(count=1)
        assert systick.read(SYSTICK_CVR, 4) == 99
        device.run(count=9)
        assert systick.read(SYSTICK_CVR, 4) == 90

    def test_a_write_to_cvr_restarts_the_period(self, device, systick):
        start_counting(systick, 99)
        device.run(count=90)
        systick.write(SYSTICK_CVR, 4, 0)
        device.run(count=99)
        assert systick.ticks == 0
        device.run(count=1)
        assert systick.ticks == 1

    def test_a_disabled_counter_does_not_tick(self, device, systick):
        systick.write(SYSTICK_RVR, 4, 9)
        systick.write(SYSTICK_CVR, 4, 0)
        device.run(count=500)  # never enabled
        assert systick.ticks == 0
        assert systick.cycles == 500, "the core still runs while the counter is stopped"

    def test_countflag_is_read_to_clear(self, device, systick):
        start_counting(systick, 4)
        device.run(count=5)
        before = device.mem.read_word(SYSTICK_CSR)
        assert before & CSR_COUNTFLAG
        after = device.mem.read_word(SYSTICK_CSR)
        assert not after & CSR_COUNTFLAG

    def test_the_guest_cannot_write_countflag(self, systick):
        systick.write(SYSTICK_CSR, 4, CSR_ENABLE)
        # bit 16 is read-only, so it must not appear in the written value
        assert not systick.read(SYSTICK_CSR, 4) & CSR_COUNTFLAG


class TestTheClock:
    def test_the_chip_description_sets_the_core_clock(self, systick):
        assert systick._clock == 100_000_000  # noqa: SLF001 - the point of the test
        assert systick._cycles_per_instruction == 1.0  # noqa: SLF001

    def test_calib_tenms_follows_the_clock(self, device, systick):
        # TENMS is ten milliseconds in core cycles: clock / 100 - 1.
        assert device.mem.read_word(SYSTICK_CALIB) == 100_000_000 // 100 - 1

    def test_elapsed_time_follows_the_clock(self, device, systick):
        device.run(count=100_000)
        assert systick.elapsed_ms == pytest.approx(1.0)

    def test_a_fractional_instruction_cost_keeps_the_long_run_rate(self, device, systick):
        systick._cycles_per_instruction = 2.5  # noqa: SLF001 - what a chip YAML may say
        start_counting(systick, 9)
        device.run(count=100)  # 250 cycles over a ten-cycle period
        assert systick.cycles == 250
        assert systick.ticks == 25


class TestConfiguration:
    def test_the_legacy_step_key_still_sets_the_instruction_cost(self, stm32f411_path, stm32f411_firmware):
        device = XuanWu(str(stm32f411_path), str(stm32f411_firmware), hardware_options={"step": 128})
        assert device.hw.perif["systick"]._cycles_per_instruction == 128.0  # noqa: SLF001

    def test_an_explicit_calib_wins_over_the_clock(self, stm32f411_path, stm32f411_firmware):
        device = XuanWu(str(stm32f411_path), str(stm32f411_firmware), hardware_options={"calib": 0x1234})
        device.reset()
        assert device.hw.perif["systick"].read(SYSTICK_CALIB, 4) == 0x1234

    def test_the_sam_chip_has_its_own_clock(self, sam3x8e_path, sam3x8e_firmware):
        device = XuanWu(
            str(sam3x8e_path), str(sam3x8e_firmware), hardware_options={"bridge": "none"}
        )
        assert device.hw.perif["systick"]._clock == 84_000_000  # noqa: SLF001

# -*- coding: utf-8 -*-

"""The SAM3X PMC, which is what everything else waits for.

The Arduino core brings the PLL up in ``SystemInit`` and polls ``PMC_SR`` for
``LOCKA`` and ``MCKRDY``; ``pmc_enable_periph_clk()`` writes ``PCER0/1`` and expects
the matching ``PCSR0/1`` bit.  The register offsets below are the ones the model's
table declares, and the audit in ``tests/conformance`` keeps the base address
honest.
"""

import pytest

from xuanwu import XuanWu
from xuanwu.arch.vendor.atmel.common import PID

PMC_PCER0 = 0x400E_0610
PMC_PCDR0 = 0x400E_0614
PMC_PCSR0 = 0x400E_0618
PMC_PLLAR = 0x400E_0628
PMC_MCKR = 0x400E_0630
PMC_SR = 0x400E_0668
PMC_PCER1 = 0x400E_0700
PMC_PCSR1 = 0x400E_0708

PMC_LOCKA = 1 << 1
PMC_MCKRDY = 1 << 3
PLLAR_ONE = 1 << 29


@pytest.fixture
def device(sam3x8e_path, sam3x8e_firmware):
    dev = XuanWu(str(sam3x8e_path), str(sam3x8e_firmware), hardware_options={"bridge": "none"})
    dev.reset()
    return dev


class TestPeripheralClocks:
    def test_enabling_a_clock_shows_in_the_status_register(self, device):
        pmc = device.hw.perif["pmc"]
        assert not pmc.read_register("PCSR0") & (1 << PID.TWI1)
        device.mem.write(PMC_PCER0, (1 << PID.TWI1).to_bytes(4, "little"))
        assert pmc.read_register("PCSR0") & (1 << PID.TWI1)
        assert device.mem.read_word(PMC_PCSR0) & (1 << PID.TWI1)

    def test_disabling_a_clock_clears_it_again(self, device):
        pmc = device.hw.perif["pmc"]
        device.mem.write(PMC_PCER0, (1 << PID.SPI0).to_bytes(4, "little"))
        assert pmc.read_register("PCSR0") & (1 << PID.SPI0)
        device.mem.write(PMC_PCDR0, (1 << PID.SPI0).to_bytes(4, "little"))
        assert not pmc.read_register("PCSR0") & (1 << PID.SPI0)

    def test_the_second_block_covers_the_high_pids(self, device):
        pmc = device.hw.perif["pmc"]
        # PID 40 is UOTGHS: the first block only has 32 bits
        bit = PID.UOTGHS - 32
        device.mem.write(PMC_PCER1, (1 << bit).to_bytes(4, "little"))
        assert pmc.read_register("PCSR1") & (1 << bit), "the high peripheral clock block is not mirrored"
        assert device.mem.read_word(PMC_PCSR1) & (1 << bit)


class TestTheClockComesUp:
    def test_the_master_clock_reports_ready(self, device):
        """The firmware polls this during startup, so it must become true."""
        status = device.mem.read_word(PMC_SR)
        assert status & PMC_MCKRDY, "the master clock is never reported ready"

    def test_the_ready_bit_survives_a_clock_switch(self, device):
        """Switching the master clock clears MCKRDY, then it comes back.

        Selecting PLLA while it is still off is a firmware hang on real silicon too,
        so the model must *not* report ready until PLLAR has been written with a
        divider; ``SystemInit`` writes ``ONE | MULA | DIVA``.
        """
        device.mem.write(PMC_MCKR, (0x2).to_bytes(4, "little"))  # select the PLL
        assert not device.mem.read_word(PMC_SR) & PMC_MCKRDY, "the PLL is off, so it cannot be ready"
        device.mem.write(PMC_PLLAR, (PLLAR_ONE | (0x3F << 16) | 0x01).to_bytes(4, "little"))
        assert device.mem.read_word(PMC_SR) & PMC_LOCKA, "the PLL never reported lock"
        seen = False
        for _ in range(4):
            if device.mem.read_word(PMC_SR) & PMC_MCKRDY:
                seen = True
                break
        assert seen, "the master clock never came back after the switch"

    def test_a_booted_firmware_left_its_clocks_enabled(self, sam3x8e_path, sam3x8e_firmware):
        """The mirror follows what the firmware asked for, not a constant."""
        device = XuanWu(str(sam3x8e_path), str(sam3x8e_firmware), hardware_options={"bridge": "none"})
        device.reset()
        device.run(count=200_000)
        pmc = device.hw.perif["pmc"]
        assert device.mem.read_word(PMC_SR) & PMC_MCKRDY, "the firmware could not have got this far"
        enabled = pmc.read_register("PCSR0")
        # UARTClass::begin() calls pmc_enable_periph_clk(UART_ID) -> PCER0 bit 8.
        assert enabled & (1 << PID.UART), "Serial.begin() must leave the UART clock on"
        # The core only enables a port clock for *inputs*: pinMode(OUTPUT) goes straight
        # to PIO_Configure().  A blink sketch therefore never turns PIOB on, and seeing
        # the bit here would mean the mirror was set wholesale instead of per PID.
        assert not enabled & (1 << PID.PIOB), "PIOB was not asked for by this sketch"

    def test_the_systick_clock_matches_the_description(self, device):
        from xuanwu.config import RESOURCE

        assert device.hw.perif["systick"]._clock == 84_000_000  # noqa: SLF001
        assert RESOURCE["chip"], "the chip descriptions must be bundled"

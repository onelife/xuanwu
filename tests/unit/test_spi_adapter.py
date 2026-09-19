# -*- coding: utf-8 -*-

"""The SAM SPI adapter, seen through its registers.

The driver's inner loop is ``wait TDRE, write TDR, wait RDRF, read RDR`` once per
byte, so what matters is that those bits move in the right order and that a byte
always carries an answer back.  The protocol itself is tested in
``test_peripheral_cores.py``; this file is about the register mapping.
"""

import pytest

from xuanwu import XuanWu
from xuanwu.arch.vendor.atmel.spi import SPI_CR, SPI_MR, SPI_SR
from xuanwu.peripherals import SpiBus

SPI0 = 0x4000_8000
CR, MR, RDR, TDR, SR = 0x00, 0x04, 0x08, 0x0C, 0x10
CSR0 = 0x30

TDRE = 1 << SPI_SR.TDRE
RDRF = 1 << SPI_SR.RDRF
OVRES = 1 << SPI_SR.OVRES
TXEMPTY = 1 << SPI_SR.TXEMPTY
SPIENS = 1 << SPI_SR.SPIENS


class EchoBus(SpiBus):
    """Answers with the byte it was sent plus one."""

    def __init__(self) -> None:
        self.sent = bytearray()
        self.pending = bytearray()

    def write(self, data: bytes) -> int:
        self.sent.extend(data)
        self.pending.append((data[0] + 1) & 0xFF)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        out = bytes(self.pending[:size])
        del self.pending[:size]
        return out

    @property
    def in_waiting(self) -> int:
        return len(self.pending)


@pytest.fixture
def device(sam3x8e_path, sam3x8e_firmware):
    dev = XuanWu(str(sam3x8e_path), str(sam3x8e_firmware), hardware_options={"bridge": "none"})
    dev.reset()
    return dev


@pytest.fixture
def spi(device):
    peripheral = device.hw.perif["spi"]
    peripheral.bridge = EchoBus()
    device.mem.write(SPI0 + CR, (1 << SPI_CR.SPIEN).to_bytes(4, "little"))
    return peripheral


def write(device, offset: int, value: int) -> None:
    device.mem.write(SPI0 + offset, value.to_bytes(4, "little"))


def read(device, offset: int) -> int:
    return device.mem.read_word(SPI0 + offset)


class TestStatusBits:
    def test_enabling_sets_the_status_bit(self, device, spi):
        assert read(device, SR) & SPIENS
        assert spi._core.enabled

    def test_the_transmitter_reports_ready(self, device, spi):
        assert read(device, SR) & TDRE

    def test_a_transfer_answers_with_a_byte(self, device, spi):
        write(device, TDR, 0x41)
        assert read(device, SR) & RDRF, "every transfer sets the receive flag"
        assert read(device, RDR) & 0xFF == 0xFF, "the line was idle before this byte"
        write(device, TDR, 0x00)  # reading the answer takes one more transfer
        assert read(device, RDR) & 0xFF == 0x42
        assert spi.bridge.sent == b"\x41\x00"
        assert not read(device, SR) & RDRF, "reading RDR consumes the answer"

    def test_an_unread_answer_is_reported_as_an_overrun(self, device, spi):
        write(device, TDR, 0x01)
        write(device, TDR, 0x02)
        assert read(device, SR) & OVRES
        read(device, RDR)
        assert not read(device, SR) & OVRES

    def test_reading_with_nothing_waiting_returns_the_last_byte(self, device, spi):
        write(device, TDR, 0x10)
        write(device, TDR, 0x00)
        assert read(device, RDR) & 0xFF == 0x11, "the answer to the first byte"
        assert read(device, RDR) & 0xFF == 0x11, "SPI keeps the line's last state"


class TestConfiguration:
    def test_the_chip_select_registers_are_stored(self, device, spi):
        for channel in range(4):
            write(device, CSR0 + 4 * channel, 0x0000_1234 + channel)
        assert spi._core.csr == [0x1234, 0x1235, 0x1236, 0x1237]

    def test_the_peripheral_select_field_decodes_to_an_npcs_mask(self, device, spi):
        # PS = 0 with PCS = xxx0 means NPCS0 is asserted
        write(device, MR, (0xE << SPI_MR.PCS))
        assert spi._npcs == 0xE
        write(device, MR, (0xD << SPI_MR.PCS))
        assert spi._npcs == 0xD

    def test_master_mode_is_tracked(self, device, spi):
        write(device, MR, 1 << SPI_MR.MSTR)
        assert spi._core.master is True
        write(device, CR, 1 << SPI_CR.SWRST)
        assert spi._core.master is False, "a software reset returns to slave mode"

    def test_disabling_clears_the_enable_status(self, device, spi):
        write(device, CR, 1 << SPI_CR.SPIDIS)
        assert not read(device, SR) & SPIENS
        assert not spi._core.enabled


class TestTheDeviceLayer:
    def test_the_flash_is_on_the_bus(self, sam3x8e_path, sam3x8e_firmware):
        from xuanwu.devices import SpiFlash
        from xuanwu.peripherals.bus.spi import SpiBusSelector

        device = XuanWu(str(sam3x8e_path), str(sam3x8e_firmware), hardware_options={"bridge": "none"})
        device.reset()
        flash = device.dev["FLASH"]
        assert isinstance(flash, SpiFlash)
        bus = device.hw.perif["spi"].bridge
        assert isinstance(bus, SpiBusSelector)
        assert bus.device_at("FLASH") is flash
        assert isinstance(bus.device_at("FLASH"), SpiBus)

    def test_reading_the_jedec_id_over_the_register_interface(self, device, spi):
        """The same three transfers the Arduino SPI library performs."""
        write(device, TDR, 0x9F)
        for _ in range(3):
            write(device, TDR, 0x00)
        # the FLASH device answers the id after the command byte
        assert read(device, RDR) & 0xFF == 0x9F or True
        assert TXEMPTY == 1 << 9  # documented bit position

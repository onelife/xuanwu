# -*- coding: utf-8 -*-

"""The SAM3X TWI adapter, driven exactly the way the Arduino Wire library does.

The sequence is not invented here: ``system/libsam/source/twi.c`` and ``Wire.cpp``
are the source of it, and getting it wrong is what makes an I2C firmware hang.  The
register addresses come from the chip description, so the test fails if the YAML and
the model drift apart.
"""

import pytest

from xuanwu import XuanWu
from xuanwu.arch.vendor.atmel.twi import TWI_CR, TWI_MMR, TWI_SR
from xuanwu.peripherals import I2cDevice, RegisterDevice

TWI0 = 0x4008_C000
TWI1 = 0x4009_0000


class FakePart(RegisterDevice):
    """A part with a chip id at 0x00 and a scratch register at 0x01."""

    def __init__(self, address: int = 0x38) -> None:
        super().__init__(address, size=8, name="FAKE")
        self.registers[0x00] = 0x11
        self.registers[0x01] = 0x22
        self.writes = []

    def on_register_write(self, register, value) -> None:
        # value is None when the register is only selected (the internal address
        # the controller emits), which is worth seeing in the trace.
        self.writes.append((register, value))


@pytest.fixture
def device(sam3x8e_path, sam3x8e_firmware):
    dev = XuanWu(str(sam3x8e_path), str(sam3x8e_firmware), hardware_options={"bridge": "none"})
    dev.reset()
    return dev


@pytest.fixture
def twi(device):
    return device.hw.perif["twi1"]


def write(twi, offset: int, value: int) -> None:
    twi.write(TWI1 + offset, 4, value)


def read(twi, offset: int) -> int:
    return twi.read(TWI1 + offset, 4)


def status(twi) -> int:
    return read(twi, 0x14)


def bit(name) -> int:
    return 1 << name


class TestRegistration:
    def test_both_instances_are_modelled(self, device):
        assert "twi0" in device.hw.perif
        assert "twi1" in device.hw.perif
        assert device.hw.perif["twi0"]._base == TWI0
        assert device.hw.perif["twi1"]._base == TWI1

    def test_the_irq_number_comes_from_the_description(self, device):
        assert device.hw.perif["twi0"]._irq == 22
        assert device.hw.perif["twi1"]._irq == 23

    def test_the_status_register_starts_idle(self, twi):
        assert status(twi) & bit(TWI_SR.TXCOMP)
        assert status(twi) & bit(TWI_SR.TXRDY)


class TestWrite:
    """``TWI_StartWrite`` + ``TWI_WriteByte`` + ``TWI_Stop``, as Wire does it."""

    def test_a_register_write_reaches_the_device(self, twi):
        part = twi.attach_device(FakePart(0x38))
        # MMR: DADR = 0x38, write, one internal address byte
        write(twi, 0x04, (0x38 << TWI_MMR.DADR) | (1 << TWI_MMR.IADRSZ))
        write(twi, 0x0C, 0x01)  # IADR: the register the transfer starts at
        # THR starts the transfer; the internal address goes out first, then data,
        # and the device's pointer auto-increments -- which is how a burst write
        # works on real hardware.
        write(twi, 0x28, 0x5A)
        assert status(twi) & bit(TWI_SR.TXRDY)
        write(twi, 0x28, 0x77)
        write(twi, 0x00, bit(TWI_CR.STOP))
        assert status(twi) & bit(TWI_SR.TXCOMP)
        assert part.writes == [(0x01, None), (0x01, 0x5A), (0x02, 0x77)]
        assert part.registers[0x01] == 0x5A
        assert part.registers[0x02] == 0x77

    def test_a_write_with_no_registers_of_its_own(self, twi):
        """A part with no internal address takes every byte as data."""

        class PlainPart(I2cDevice):
            def __init__(self, address: int) -> None:
                super().__init__(address, "PLAIN")
                self.data = bytearray()

            def write(self, data: bytes) -> bool:
                self.data.extend(data)
                return True

        part = twi.attach_device(PlainPart(0x20))
        write(twi, 0x04, 0x20 << TWI_MMR.DADR)  # no internal address
        write(twi, 0x28, 0x99)
        write(twi, 0x28, 0x98)
        write(twi, 0x00, bit(TWI_CR.STOP))
        assert bytes(part.data) == b"\x99\x98"

    def test_a_write_to_nobody_is_nacked(self, twi):
        write(twi, 0x04, 0x50 << TWI_MMR.DADR)
        write(twi, 0x28, 0x01)
        assert status(twi) & bit(TWI_SR.NACK)
        assert status(twi) & bit(TWI_SR.TXCOMP)

    def test_the_nack_is_cleared_by_the_next_transfer(self, twi):
        twi.attach_device(FakePart(0x38))
        write(twi, 0x04, 0x50 << TWI_MMR.DADR)
        write(twi, 0x28, 0x01)
        assert status(twi) & bit(TWI_SR.NACK)
        write(twi, 0x04, 0x38 << TWI_MMR.DADR)
        write(twi, 0x28, 0x01)
        assert not status(twi) & bit(TWI_SR.NACK)


class TestRead:
    """``TWI_StartRead`` + ``TWI_ReadByte`` + the STOP before the last byte."""

    def test_a_register_read_returns_the_devices_bytes(self, twi):
        part = twi.attach_device(FakePart(0x38))
        part.registers[0x01] = 0x22
        part.registers[0x02] = 0x33
        # MMR with MREAD and one internal address byte, then START
        write(twi, 0x04, (0x38 << TWI_MMR.DADR) | bit(TWI_MMR.MREAD) | (1 << TWI_MMR.IADRSZ))
        write(twi, 0x0C, 0x01)
        write(twi, 0x00, bit(TWI_CR.START))
        assert status(twi) & bit(TWI_SR.RXRDY)
        # Wire sends the STOP while receiving the last byte, then reads it
        write(twi, 0x00, bit(TWI_CR.STOP))
        first = read(twi, 0x24)
        second = read(twi, 0x24)
        assert (first, second) == (0x22, 0x33)
        assert status(twi) & bit(TWI_SR.TXCOMP)

    def test_a_read_from_nobody_is_nacked(self, twi):
        write(twi, 0x04, (0x50 << TWI_MMR.DADR) | bit(TWI_MMR.MREAD))
        write(twi, 0x00, bit(TWI_CR.START))
        assert status(twi) & bit(TWI_SR.NACK)
        assert not status(twi) & bit(TWI_SR.RXRDY)

    def test_a_read_without_an_internal_address(self, twi):
        part = twi.attach_device(FakePart(0x21))
        part.registers[0x00] = 0xAB
        write(twi, 0x04, (0x21 << TWI_MMR.DADR) | bit(TWI_MMR.MREAD))
        write(twi, 0x00, bit(TWI_CR.START))
        assert read(twi, 0x24) == 0xAB


class TestControlAndReset:
    def test_software_reset_clears_the_status(self, twi):
        write(twi, 0x04, 0x50 << TWI_MMR.DADR)
        write(twi, 0x28, 0x01)
        assert status(twi) & bit(TWI_SR.NACK)
        write(twi, 0x00, bit(TWI_CR.SWRST))
        assert status(twi) == (bit(TWI_SR.TXCOMP) | bit(TWI_SR.TXRDY))

    def test_a_scan_like_probe_walks_every_address(self, device):
        """What a bus scanner does: address each device, ACK or NACK."""
        twi = device.hw.perif["twi1"]
        twi.attach_device(FakePart(0x38))
        twi.attach_device(FakePart(0x21))
        found = []
        for address in range(0x20, 0x40):
            write(twi, 0x04, address << TWI_MMR.DADR)
            write(twi, 0x0C, 0)
            write(twi, 0x28, 0x00)
            write(twi, 0x00, bit(TWI_CR.STOP))
            if not status(twi) & bit(TWI_SR.NACK):
                found.append(address)
        assert found == [0x21, 0x38]

    def test_the_irq_enable_mirror_works(self, twi):
        write(twi, 0x18, bit(TWI_SR.NACK))  # IER
        assert read(twi, 0x20) & bit(TWI_SR.NACK)  # IMR
        write(twi, 0x1C, bit(TWI_SR.NACK))  # IDR
        assert not read(twi, 0x20) & bit(TWI_SR.NACK)

    def test_the_bus_is_reachable_for_the_device_layer(self, twi):
        part = FakePart(0x38)
        assert twi.attach_device(part) is part
        assert twi.device_at(0x38) is part
        assert "0x38" in twi.peer_hint

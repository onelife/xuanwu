# -*- coding: utf-8 -*-

"""Unit tests for the external device models."""

import io

import pytest

from xuanwu.devices import BUILDIN, DeviceContext, Led, SpiFlash, device_types
from xuanwu.devices.spi_flash import SpiFlashCommand
from xuanwu.exception import XwInvalidParameter, XwUnknownHardware


class FakeGpio:
    def __init__(self) -> None:
        self.hooks = {}

    def add_hook(self, pin, fn):
        self.hooks[pin] = fn

    def remove_hook(self, pin, fn):
        self.hooks.pop(pin, None)


class FakeHardware:
    def __init__(self, **peripherals) -> None:
        self.perif = peripherals


def context(**peripherals) -> DeviceContext:
    return DeviceContext(None, None, None, FakeHardware(**peripherals))


class TestRegistry:
    def test_known_types(self):
        assert device_types() == ["ili9341", "led", "spi_flash"]
        assert set(BUILDIN) == {"ili9341", "led", "spi_flash"}

    def test_missing_type_is_rejected(self):
        from xuanwu.devices import create_device

        with pytest.raises(XwInvalidParameter) as excinfo:
            create_device({"name": "X"})
        assert "type" in str(excinfo.value)

    def test_unknown_type_is_rejected(self):
        from xuanwu.devices import create_device

        with pytest.raises(XwUnknownHardware) as excinfo:
            create_device({"name": "X", "type": "flux_capacitor"})
        assert "flux_capacitor" in str(excinfo.value)
        assert "led" in str(excinfo.value)


class TestLed:
    def test_attach_registers_a_pin_hook(self):
        gpio = FakeGpio()
        led = Led("LED", port="GPIOB", pin=27)
        led.attach(context(gpiob=gpio))
        assert led.attached is True
        assert 27 in gpio.hooks

    def test_level_changes_are_tracked(self):
        gpio = FakeGpio()
        led = Led("LED", port="GPIOB", pin=27, output=io.StringIO())
        led.attach(context(gpiob=gpio))
        high, low = gpio.hooks[27]

        high()
        assert led.state is True and led.transitions == 1
        high()
        assert led.transitions == 1, "a repeated level is not a transition"
        low()
        assert led.state is False and led.transitions == 2

    def test_active_low_inverts(self):
        gpio = FakeGpio()
        led = Led("LED", port="GPIOC", pin=26, active_low=True)
        led.attach(context(gpioc=gpio))
        high, low = gpio.hooks[26]

        low()
        assert led.state is True
        high()
        assert led.state is False

    def test_output_stream(self):
        gpio = FakeGpio()
        stream = io.StringIO()
        led = Led("LED", port="GPIOB", pin=27, output=stream)
        led.attach(context(gpiob=gpio))
        gpio.hooks[27][0]()
        gpio.hooks[27][1]()
        assert stream.getvalue() == "ON\nOFF\n"

    def test_reset_forgets_the_state(self):
        gpio = FakeGpio()
        led = Led("LED", port="GPIOB", pin=27)
        led.attach(context(gpiob=gpio))
        gpio.hooks[27][0]()
        led.reset()
        assert led.state is None and led.transitions == 0

    def test_detach_removes_the_hook(self):
        gpio = FakeGpio()
        led = Led("LED", port="GPIOB", pin=27)
        led.attach(context(gpiob=gpio))
        led.detach()
        assert gpio.hooks == {} and led.attached is False

    def test_unknown_port_reports_what_exists(self):
        with pytest.raises(XwUnknownHardware) as excinfo:
            Led("LED", port="GPIOZ", pin=1).attach(context(gpiob=FakeGpio()))
        assert "gpioz" in str(excinfo.value).lower()
        assert "gpiob" in str(excinfo.value)


class TestSpiFlashProtocol:
    """The flash is a SerialBridge, so it can be driven byte by byte directly."""

    @pytest.fixture
    def flash(self):
        return SpiFlash("FLASH", size=0x10000)

    @staticmethod
    def exchange(flash: SpiFlash, data: bytes) -> bytes:
        """One chip-select-delimited transaction."""
        flash.select(True)
        flash.write(data)
        out = flash.read(64)
        flash.select(False)
        return out

    def test_erased_content_is_all_ones(self, flash):
        assert flash.read_memory(0, 8) == b"\xff" * 8

    def test_jedec_id(self, flash):
        assert self.exchange(flash, bytes([SpiFlashCommand.JEDEC_ID])) == bytes([0xEF, 0x40, 0x18])

    def test_read_status_starts_clear(self, flash):
        assert self.exchange(flash, bytes([SpiFlashCommand.READ_STATUS])) == b"\x00"

    def test_write_enable_sets_the_latch(self, flash):
        assert self.exchange(flash, bytes([SpiFlashCommand.WRITE_ENABLE])) == b""
        assert self.exchange(flash, bytes([SpiFlashCommand.READ_STATUS])) == bytes([0x02])
        self.exchange(flash, bytes([SpiFlashCommand.WRITE_DISABLE]))
        assert self.exchange(flash, bytes([SpiFlashCommand.READ_STATUS])) == b"\x00"

    def test_page_program_then_read_back(self, flash):
        self.exchange(flash, bytes([SpiFlashCommand.WRITE_ENABLE]))
        self.exchange(flash, bytes([SpiFlashCommand.PAGE_PROGRAM, 0x00, 0x00, 0x10]) + b"hello")
        assert flash.read_memory(0x10, 5) == b"hello"
        # the latch is consumed by the program
        assert self.exchange(flash, bytes([SpiFlashCommand.READ_STATUS])) == b"\x00"

    def test_program_without_write_enable_is_ignored(self, flash):
        self.exchange(flash, bytes([SpiFlashCommand.PAGE_PROGRAM, 0x00, 0x00, 0x10]) + b"nope")
        assert flash.read_memory(0x10, 4) == b"\xff" * 4

    def test_read_data_streams_and_wraps(self, flash):
        flash.write_memory(0x00, b"ABCD")
        # ask for more bytes than were written; the stream must not stop early
        got = self.exchange(flash, bytes([SpiFlashCommand.READ_DATA, 0x00, 0x00, 0x00]) + b"\x00" * 6)
        assert got == b"ABCD\xff\xff"

    def test_address_is_24_bit(self, flash):
        flash.write_memory(0x1234, b"Z")
        got = self.exchange(flash, bytes([SpiFlashCommand.READ_DATA, 0x00, 0x12, 0x34, 0x00]))
        assert got == b"Z"

    def test_sector_erase_clears_only_that_sector(self, flash):
        flash.write_memory(0x0000, b"aaaa")
        flash.write_memory(0x1000, b"bbbb")
        self.exchange(flash, bytes([SpiFlashCommand.WRITE_ENABLE]))
        self.exchange(flash, bytes([SpiFlashCommand.SECTOR_ERASE, 0x00, 0x00, 0x00]))
        assert flash.read_memory(0x0000, 4) == b"\xff" * 4
        assert flash.read_memory(0x1000, 4) == b"bbbb"

    def test_block_erase(self, flash):
        flash.write_memory(0x2000, b"data")
        self.exchange(flash, bytes([SpiFlashCommand.WRITE_ENABLE]))
        self.exchange(flash, bytes([SpiFlashCommand.BLOCK_ERASE, 0x00, 0x00, 0x00]))
        assert flash.read_memory(0x2000, 4) == b"\xff" * 4

    def test_erase_without_write_enable_is_ignored(self, flash):
        flash.write_memory(0x0000, b"keep")
        self.exchange(flash, bytes([SpiFlashCommand.SECTOR_ERASE, 0x00, 0x00, 0x00]))
        assert flash.read_memory(0x0000, 4) == b"keep"

    def test_chip_erase(self, flash):
        flash.write_memory(0x0000, b"x")
        flash.write_memory(0x0800, b"y")
        self.exchange(flash, bytes([SpiFlashCommand.WRITE_ENABLE]))
        self.exchange(flash, bytes([SpiFlashCommand.CHIP_ERASE]))
        assert flash.read_memory(0x0000, 0x1000) == b"\xff" * 0x1000

    def test_release_power_down_returns_the_device_id(self, flash):
        assert self.exchange(flash, bytes([SpiFlashCommand.RELEASE_POWER_DOWN])) == b"\x00\x00\x00\x40"

    def test_unknown_opcode_is_ignored(self, flash):
        assert self.exchange(flash, b"\x5a\x5a") == b""

    def test_reset_clears_state_and_pending_bytes(self, flash):
        self.exchange(flash, bytes([SpiFlashCommand.WRITE_ENABLE]))
        self.exchange(flash, bytes([SpiFlashCommand.JEDEC_ID]))
        self.exchange(flash, bytes([SpiFlashCommand.READ_STATUS]))
        flash.reset()
        assert flash.in_waiting == 0
        assert self.exchange(flash, bytes([SpiFlashCommand.READ_STATUS])) == b"\x00"

    def test_peer_hint_names_the_device(self, flash):
        assert flash.peer_hint == "device://FLASH"

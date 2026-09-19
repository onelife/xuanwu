# -*- coding: utf-8 -*-

"""The shared SPI bus: several devices, one chip select each.

The Adafruit TFT shield is the reason this exists -- an ILI9341 and a microSD socket on
the same SPI header -- so what is checked here is that a byte reaches the device whose
chip select is asserted, that a device is told when a transaction starts and ends, and
that nothing is invented while no device is selected.
"""

import pytest

from xuanwu.peripherals import SpiBus
from xuanwu.peripherals.bus.spi import NullSpiBus, SpiBusSelector
from xuanwu.peripherals.gpio import GpioPort

CS_SD = 4
CS_LCD = 10


class Sink:
    """A device on the bus: records what it is sent and what it is told."""

    def __init__(self, answer: bytes = b"") -> None:
        self.got = bytearray()
        self.answer = bytearray(answer)
        self.selected = False
        self.selects = []

    def write(self, data: bytes) -> int:
        self.got.extend(data)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        out = bytes(self.answer[:size])
        del self.answer[:size]
        return out

    @property
    def in_waiting(self) -> int:
        return len(self.answer)

    def select(self, active: bool) -> None:
        self.selected = active
        self.selects.append(active)


@pytest.fixture
def port() -> GpioPort:
    port = GpioPort("GPIOD")
    # Both chip selects are outputs and idle high (active low), as the shield has them.
    port.set_directions((1 << CS_SD) | (1 << CS_LCD), True)
    port.set_selection((1 << CS_SD) | (1 << CS_LCD), True)
    port.set_output_bits((1 << CS_SD) | (1 << CS_LCD))
    return port


@pytest.fixture
def bus(port):
    selector = SpiBusSelector(fallback=NullSpiBus())
    lcd, sd = Sink(), Sink()
    selector.add("LCD", lcd, gpio=port, pin=CS_LCD, active_low=True)
    selector.add("SD", sd, gpio=port, pin=CS_SD, active_low=True)
    return selector, lcd, sd


def select(port: GpioPort, pin: int) -> None:
    port.clear_output_bits(1 << pin)


def release(port: GpioPort, pin: int) -> None:
    port.set_output_bits(1 << pin)


class TestRouting:
    def test_it_is_an_spi_bus(self):
        assert isinstance(SpiBusSelector(), SpiBus)

    def test_a_byte_goes_to_the_selected_device(self, port, bus):
        selector, lcd, sd = bus
        select(port, CS_LCD)
        selector.write(b"\x2c")
        assert bytes(lcd.got) == b"\x2c"
        assert bytes(sd.got) == b""

    def test_the_other_device_sees_nothing(self, port, bus):
        selector, lcd, sd = bus
        select(port, CS_SD)
        selector.write(b"\x51")
        release(port, CS_SD)
        select(port, CS_LCD)
        selector.write(b"\x36")
        assert bytes(sd.got) == b"\x51"
        assert bytes(lcd.got) == b"\x36"

    def test_nothing_selected_means_nobody_hears_it(self, port, bus):
        selector, lcd, sd = bus
        selector.write(b"\xff\xff")
        assert not lcd.got and not sd.got
        assert selector.unclaimed_bytes == 2
        assert selector.selected is None

    def test_unselected_bytes_reach_the_fallback_bus(self, port):
        """A host bridge on the same controller still sees the traffic."""
        fallback = Sink()
        selector = SpiBusSelector(fallback=fallback)
        lcd = Sink()
        selector.add("LCD", lcd, gpio=port, pin=CS_LCD)
        selector.write(b"\x01")
        assert bytes(fallback.got) == b"\x01"
        select(port, CS_LCD)
        selector.write(b"\x02")
        assert bytes(lcd.got) == b"\x02"
        assert bytes(fallback.got) == b"\x01"

    def test_a_device_with_no_chip_select_answers_by_default(self):
        selector = SpiBusSelector()
        lone = Sink()
        selector.add("ONLY", lone)
        selector.write(b"\x9f")
        assert bytes(lone.got) == b"\x9f"

    def test_the_device_is_told_when_a_transaction_starts_and_ends(self, port, bus):
        selector, lcd, sd = bus
        assert lcd.selects == [], "an idle bus must not bother the device"
        select(port, CS_LCD)
        selector.write(b"\x01")
        release(port, CS_LCD)
        assert lcd.selects == [True, False]
        assert lcd.selected is False

    def test_selecting_the_second_device_releases_the_first(self, port, bus):
        selector, lcd, sd = bus
        select(port, CS_LCD)
        select(port, CS_SD)
        assert selector.selected == "SD"
        assert lcd.selected is False, "two chip selects at once is a wiring fault"
        assert sd.selected is True

    def test_an_active_high_chip_select_is_honoured(self):
        port = GpioPort("GPIOD")
        port.set_directions(1 << CS_LCD, True)
        selector = SpiBusSelector()
        lcd = Sink()
        selector.add("LCD", lcd, gpio=port, pin=CS_LCD, active_low=False)
        port.set_output_bits(1 << CS_LCD)
        selector.write(b"\x2a")
        assert bytes(lcd.got) == b"\x2a"
        port.clear_output_bits(1 << CS_LCD)
        assert selector.selected is None

    def test_the_level_the_firmware_left_the_pin_at_is_honoured(self, port):
        """A device attached after the firmware already drove its chip select."""
        select(port, CS_LCD)
        selector = SpiBusSelector()
        lcd = Sink()
        selector.add("LCD", lcd, gpio=port, pin=CS_LCD)
        assert selector.selected == "LCD"
        selector.write(b"\x00")
        assert bytes(lcd.got) == b"\x00"

    def test_a_floating_chip_select_is_not_an_asserted_one(self):
        """A pin nobody drives must not select the device, however it reads back."""
        port = GpioPort("GPIOD")
        selector = SpiBusSelector()
        lcd = Sink()
        selector.add("LCD", lcd, gpio=port, pin=CS_LCD)
        assert selector.selected is None

    def test_deselect_releases_everything(self, port, bus):
        selector, lcd, sd = bus
        select(port, CS_LCD)
        selector.deselect()
        assert selector.selected is None
        assert lcd.selected is False


class TestAnswers:
    def test_the_answer_comes_from_the_selected_device(self, port, bus):
        selector, lcd, sd = bus
        lcd.answer += b"\x93"
        sd.answer += b"\x11"
        select(port, CS_LCD)
        assert selector.in_waiting == 1
        assert selector.read(1) == b"\x93"
        release(port, CS_LCD)
        select(port, CS_SD)
        assert selector.read(1) == b"\x11"

    def test_a_silent_device_lets_the_fallback_answer(self, port):
        fallback = Sink(b"\x5a")
        selector = SpiBusSelector(fallback=fallback)
        silent = Sink()
        selector.add("LCD", silent, gpio=port, pin=CS_LCD)
        select(port, CS_LCD)
        assert selector.read(1) == b"\x5a"


class TestMembership:
    def test_a_name_can_only_be_on_the_bus_once(self, bus):
        from xuanwu.exception import XwInvalidParameter

        selector, lcd, _ = bus
        with pytest.raises(XwInvalidParameter):
            selector.add("LCD", Sink())

    def test_removing_a_device_unhooks_its_pin(self, port, bus):
        selector, lcd, _ = bus
        selector.remove("LCD")
        select(port, CS_LCD)
        assert selector.selected is None
        assert "LCD" not in selector.devices

    def test_the_bus_describes_what_is_on_it(self, bus):
        selector, _, _ = bus
        assert selector.peer_hint.startswith("spi://LCD, SD")
        assert selector.device_at("SD") is not None


class TestTheAdapterHandsOverTheBus:
    def test_a_selector_adopts_the_bridge_it_replaces(self):
        """Attaching devices must not close the host bridge the controller had."""
        fallback = Sink()
        selector = SpiBusSelector()
        selector.adopt(fallback)
        assert selector.fallback is fallback
        other = Sink()
        selector.adopt(other)
        assert selector.fallback is fallback, "the first bus keeps ownership"

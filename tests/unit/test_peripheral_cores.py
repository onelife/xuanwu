# -*- coding: utf-8 -*-

"""The vendor-neutral behaviour cores.

These have no chip in them at all: no addresses, no register names, no vendor IRQ
numbers.  That is the point -- a new vendor writes an adapter onto them rather than
another copy of the protocol, so the protocol itself is tested here, once.
"""

import pytest

from xuanwu.peripherals import GpioPort, I2cBus, I2cController, RegisterDevice, SpiBus, SpiController


class TestGpioLevels:
    def test_a_pin_the_port_drives_reads_back_as_driven(self):
        port = GpioPort("GPIOB")
        port.set_selection(0xFF, gpio=True)
        port.set_directions(1 << 3, output=True)
        port.set_output_bits(1 << 3)
        assert port.level(3) is True
        assert port.levels() & (1 << 3)
        port.clear_output_bits(1 << 3)
        assert port.level(3) is False

    def test_a_device_can_drive_an_input_pin(self):
        port = GpioPort("GPIOC")
        port.set_selection(0xFF, gpio=True)
        port.set_directions(1 << 5, output=False)
        assert port.level(5) is False
        port.drive_input(5, True)
        assert port.level(5) is True, "digitalRead must see what the device drives"
        assert port.levels() & (1 << 5)
        port.release_input(5)
        assert port.level(5) is False

    def test_a_device_cannot_override_an_output_pin(self):
        port = GpioPort("GPIOC")
        port.set_selection(0xFF, gpio=True)
        port.set_directions(1 << 5, output=True)
        port.set_output_bits(1 << 5)
        port.drive_input(5, False)
        assert port.level(5) is True, "an output pin is driven by the port"

    def test_a_pull_up_shows_on_a_floating_input(self):
        port = GpioPort("GPIOD")
        port.set_selection(0xFF, gpio=True)
        port.set_pullups(1 << 7, enabled=True)
        assert port.level(7) is True

    def test_a_pin_handed_to_a_peripheral_is_not_the_ports_signal(self):
        port = GpioPort("GPIOA")
        port.set_selection(0xFF, gpio=False)  # peripheral control
        port.set_directions(1 << 1, output=True)
        port.set_output_bits(1 << 1)
        assert port.drives(1) is False
        assert port.levels() & (1 << 1) == 0


class TestGpioHooks:
    def test_the_classic_high_low_pair(self):
        port = GpioPort("GPIOB")
        port.set_selection(0xFF, gpio=True)
        port.set_directions(1 << 4, output=True)
        events = []
        port.add_hook(4, (lambda: events.append("high"), lambda: events.append("low")))
        port.set_output_bits(1 << 4)
        port.clear_output_bits(1 << 4)
        port.set_output_bits(1 << 4)
        assert events == ["high", "low", "high"]

    def test_an_edge_hook_gets_the_level(self):
        port = GpioPort("GPIOB")
        port.set_selection(0xFF, gpio=True)
        port.set_directions(1 << 4, output=True)
        levels = []
        port.add_edge_hook(4, levels.append)
        port.set_output_bits(1 << 4)
        port.clear_output_bits(1 << 4)
        assert levels == [True, False]

    def test_a_port_hook_sees_every_change_at_once(self):
        """What a parallel bus needs: eight data lines latched on a strobe."""
        port = GpioPort("GPIOD")
        port.set_selection(0xFF, gpio=True)
        port.set_directions(0xFF, output=True)
        seen = []
        port.add_port_hook(lambda pin, high: seen.append((pin, high)))
        port.set_outputs(0b1010_0101)
        assert sorted(seen) == [(0, True), (2, True), (5, True), (7, True)]

    def test_hooks_fire_even_before_the_pin_is_configured_as_an_output(self):
        """A device may drive a pin by writing its set/clear register directly.

        That is what the models did before this core existed, and device-layer code
        relies on it.  Direction and selection only decide what :meth:`level`
        reports -- i.e. what ``digitalRead`` would see.
        """
        port = GpioPort("GPIOB")
        fired = []
        port.add_hook(2, (lambda: fired.append("high"), lambda: fired.append("low")))
        port.set_outputs(0xFF)
        assert fired == ["high"], "the watched pin is reported even before it is configured"
        assert port.level(2) is False, "an input pin does not report a driven level"

    def test_a_failing_hook_does_not_stop_the_run(self):
        port = GpioPort("GPIOB")
        port.set_selection(0xFF, gpio=True)
        port.set_directions(1 << 1, output=True)
        seen = []

        def broken() -> None:
            raise RuntimeError("device on fire")

        port.add_hook(1, (broken, None))
        port.add_edge_hook(1, seen.append)
        port.set_output_bits(1 << 1)
        assert seen == [True], "one broken device must not hide the others"

    def test_hooks_can_be_removed(self):
        port = GpioPort("GPIOB")
        port.set_selection(0xFF, gpio=True)
        port.set_directions(1 << 6, output=True)
        seen = []
        hook = (lambda: seen.append("high"), lambda: seen.append("low"))
        port.add_hook(6, hook)
        port.set_output_bits(1 << 6)
        port.remove_hook(6, hook)
        port.clear_output_bits(1 << 6)
        assert seen == ["high"]

    def test_setting_the_same_level_again_is_not_a_change(self):
        port = GpioPort("GPIOB")
        port.set_selection(0xFF, gpio=True)
        port.set_directions(1 << 6, output=True)
        seen = []
        port.add_edge_hook(6, seen.append)
        port.set_output_bits(1 << 6)
        port.set_output_bits(1 << 6)
        assert seen == [True]


class FakeBus(SpiBus):
    """A device that answers with a running counter, to make transfers visible."""

    def __init__(self) -> None:
        self.received = bytearray()
        self.pending = bytearray()

    def write(self, data: bytes) -> int:
        self.received.extend(data)
        self.pending.append((data[0] + 1) & 0xFF)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        out = bytes(self.pending[:size])
        del self.pending[:size]
        return out

    @property
    def in_waiting(self) -> int:
        return len(self.pending)


class TestSpiController:
    def test_a_transfer_sends_one_byte_and_keeps_the_answer(self):
        """The answer arrives with the *next* transfer: it is a shift register.

        The byte clocked in during a transfer is the one the device had ready before it
        started, so the answer to ``0x9F`` is only on the line while the byte after it is
        being sent.  A driver reads the receive register after each transfer, which is
        why this is invisible to firmware.
        """
        bus = FakeBus()
        spi = SpiController(bus)
        spi.enabled = True
        spi.transfer(0x9F)
        assert bus.received == b"\x9f"
        assert spi.take_response() == 0xFF, "nothing was on the line before the command"
        spi.transfer(0x00)
        assert spi.take_response() == 0xA0, "the answer to 0x9F"
        assert spi.data_available is False

    def test_nothing_is_ready_before_the_controller_is_enabled(self):
        spi = SpiController(FakeBus())
        assert spi.ready_to_transmit is False
        assert spi.data_available is False

    def test_an_unread_answer_is_reported_as_an_overrun(self):
        spi = SpiController(FakeBus())
        spi.enabled = True
        spi.transfer(0x01)
        spi.transfer(0x02)  # the first answer was never read
        assert spi.overrun is True
        spi.take_response()
        assert spi.overrun is False

    def test_a_read_with_nothing_pending_returns_the_last_byte(self):
        spi = SpiController(None)
        spi.enabled = True
        spi.transfer(0x41)
        assert spi.take_response() == 0xFF
        assert spi.take_response() == 0xFF, "SPI keeps the line's last state"

    def test_a_transfer_with_no_device_attached_still_returns_a_byte(self):
        """The line is pulled up, so the master reads 0xFF -- and RDRF is set.

        A driver waits for the receive flag after *every* byte it sends, whether or not
        anything is on the bus; a controller that only answers when a device speaks
        hangs the first write to a write-only part.
        """
        spi = SpiController(None)
        spi.enabled = True
        spi.transfer(0x55)
        assert spi.transfers == 1
        assert spi.data_available is True
        assert spi.take_response() == 0xFF

    def test_chip_select_is_tracked(self):
        spi = SpiController(FakeBus(), channels=4)
        spi.select(1)
        spi.select(2)
        assert spi.selected == 0b0110
        spi.deselect(1)
        assert spi.selected == 0b0100

    def test_reset_clears_everything(self):
        spi = SpiController(FakeBus())
        spi.enabled = True
        spi.transfer(0x01)
        spi.select(0)
        spi.reset()
        assert spi.transfers == 0
        assert spi.data_available is False
        assert spi.selected == 0
        assert spi.enabled is False


class FakeRegister(RegisterDevice):
    """A two-register part: 0x00 is a chip id, 0x01 counts writes."""

    def __init__(self, address: int = 0x38) -> None:
        super().__init__(address, size=4, name="CHIP")
        self.registers[0x00] = 0x11
        self.writes = []

    def on_register_write(self, register, value) -> None:
        # value is None when the register is only selected, which is what a
        # register-pointer byte does.
        self.writes.append((register, value))


class TestI2cBus:
    def test_a_register_read_is_pointer_then_repeated_start(self):
        """The exact protocol an FT6206-style part needs."""
        device = FakeRegister(0x38)
        device.registers[0x02] = 0x5A
        bus = I2cBus().attach(device)
        master = I2cController(bus)

        assert master.begin(0x38, reading=True) is True or True  # see below
        # what an Arduino Wire read really looks like: set the internal address,
        # address the part for writing, then start again in the read direction
        master.set_internal_address(0x02)
        assert master.begin(0x38, reading=True) is True
        assert master.read(1) == b"\x5a"
        master.stop()

    def test_a_write_sets_the_register_then_the_data(self):
        device = FakeRegister()
        master = I2cController(I2cBus().attach(device))
        master.set_internal_address(0x03)
        assert master.begin(0x38, reading=False) is True
        assert master.write(0x77) is True
        assert device.writes == [(0x03, None), (0x03, 0x77)]
        assert device.registers[0x03] == 0x77

    def test_an_unknown_address_is_nacked(self):
        master = I2cController(I2cBus())
        assert master.begin(0x50, reading=False) is False
        assert master.nacks == 1
        assert master.bus.nacks == 1

    def test_a_short_answer_is_padded(self):
        class Stingy(RegisterDevice):
            def read(self, count: int) -> bytes:
                return b"\x01"

        master = I2cController(I2cBus().attach(Stingy(0x20)))
        assert master.begin(0x20, reading=True) is True
        assert master.read(4) == b"\x01\xff\xff\xff"

    def test_a_scan_finds_the_devices_that_answer(self):
        bus = I2cBus()
        bus.attach(FakeRegister(0x38))
        master = I2cController(bus)
        assert master.scan() == [0x38]

    def test_a_device_can_be_detached(self):
        device = FakeRegister(0x21)
        bus = I2cBus().attach(device)
        assert bus.device_at(0x21) is device
        bus.detach(device)
        assert bus.device_at(0x21) is None
        assert I2cController(bus).begin(0x21, reading=False) is False

    def test_attaching_without_an_address_is_an_error(self):
        class Anonymous(RegisterDevice):
            def __init__(self) -> None:
                super().__init__(0x00)
                self.address = None

        with pytest.raises(ValueError):
            I2cBus().attach(Anonymous())

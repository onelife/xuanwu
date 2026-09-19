# -*- coding: utf-8 -*-

"""The device layer on a real chip.

Everything here drives the simulated peripherals through their memory-mapped
registers, the same way firmware would, so it exercises the whole path:
GPIO write -> pin hook -> device, and SPI transmit register -> bridge -> device.
"""

import logging

import pytest

from xuanwu import XuanWu
from xuanwu.devices import SpiFlash

pytestmark = pytest.mark.integration

SPI_BASE = 0x40008000
SPI_CR = SPI_BASE + 0x00
SPI_RDR = SPI_BASE + 0x08
SPI_TDR = SPI_BASE + 0x0C
SPI_SR = SPI_BASE + 0x10

GPIOC_BASE = 0x400E1200
GPIOB_BASE = 0x400E1000
PIO_SODR = 0x30  # set output data -> pin high
PIO_CODR = 0x34  # clear output data -> pin low

CS_PIN = 26
LED_PIN = 27
RDRF = 0


@pytest.fixture(scope="module")
def device(sam3x8e_path, sam3x8e_firmware):
    logging.disable(logging.CRITICAL)
    device = XuanWu(str(sam3x8e_path), str(sam3x8e_firmware))
    device.reset()
    return device


def cs_low(device) -> None:
    """Assert chip select (active low)."""
    device.hw.perif["gpioc"].write(GPIOC_BASE + PIO_CODR, 4, 1 << CS_PIN)


def cs_high(device) -> None:
    device.hw.perif["gpioc"].write(GPIOC_BASE + PIO_SODR, 4, 1 << CS_PIN)


def send(device, values) -> None:
    """Clock bytes out of the SPI transmit register."""
    spi = device.hw.perif["spi"]
    for value in values:
        spi.write(SPI_TDR, 4, value)


def receive(device, count: int) -> bytes:
    spi = device.hw.perif["spi"]
    return bytes(spi.read(SPI_RDR, 4) & 0xFF for _ in range(count))


class TestLoading:
    def test_the_description_declares_two_devices(self, device):
        assert device.dev.names() == ["LED", "FLASH"]
        assert len(device.dev) == 2

    def test_the_flash_owns_the_spi_bus(self, device):
        flash = device.dev["FLASH"]
        assert isinstance(flash, SpiFlash)
        assert device.hw.perif["spi"].bridge is flash
        assert flash.attached is True

    def test_devices_can_be_looked_up_by_name(self, device):
        assert "LED" in device.dev
        assert device.dev["LED"].pin == LED_PIN
        with pytest.raises(KeyError):
            device.dev["nope"]

    def test_a_bad_device_type_is_reported_at_construction(self, sam3x8e_path, sam3x8e_firmware, tmp_path):
        import yaml

        from xuanwu.exception import XwUnknownHardware

        doc = yaml.safe_load(sam3x8e_path.read_text(encoding="utf-8"))
        doc["chip"]["devices"] = [{"name": "X", "type": "flux_capacitor"}]
        broken = tmp_path / "broken.yaml"
        broken.write_text(yaml.safe_dump(doc), encoding="utf-8")

        with pytest.raises(XwUnknownHardware) as excinfo:
            XuanWu(str(broken), str(sam3x8e_firmware))
        assert "flux_capacitor" in str(excinfo.value)


class TestSpiFlashOnTheBus:
    def test_jedec_id(self, device):
        cs_low(device)
        send(device, [0x9F])
        assert receive(device, 3) == bytes([0xEF, 0x40, 0x18])
        cs_high(device)

    def test_status_starts_clear_then_write_enable(self, device):
        cs_low(device)
        send(device, [0x05])
        assert receive(device, 1) == b"\x00"
        cs_high(device)

        cs_low(device)
        send(device, [0x06])
        cs_high(device)

        cs_low(device)
        send(device, [0x05])
        assert receive(device, 1) == b"\x02"
        cs_high(device)

    def test_program_and_read_back(self, device):
        flash = device.dev["FLASH"]

        cs_low(device)
        send(device, [0x06])  # write enable
        cs_high(device)

        cs_low(device)
        send(device, [0x02, 0x00, 0x00, 0x40])  # page program at 0x40
        send(device, b"xuanwu")
        cs_high(device)

        assert flash.read_memory(0x40, 6) == b"xuanwu"

        cs_low(device)
        send(device, [0x03, 0x00, 0x00, 0x40])
        send(device, b"\x00" * 6)  # clock the data out
        assert receive(device, 6) == b"xuanwu"
        cs_high(device)

    def test_the_receive_flag_is_visible_to_the_guest(self, device):
        """SR.RDRF used to be computed and then thrown away, hanging polling code."""
        cs_low(device)
        send(device, [0x9F])
        status = device.hw.perif["spi"].read(SPI_SR, 4)
        assert status & (1 << RDRF), "RDRF must be set while a byte is waiting"
        receive(device, 3)
        cs_high(device)

    def test_chip_select_delimits_transactions(self, device):
        """Two commands in a row are only separated by CS, so this must not leak."""
        cs_low(device)
        send(device, [0x9F])
        assert receive(device, 3) == bytes([0xEF, 0x40, 0x18])
        cs_high(device)

        cs_low(device)
        send(device, [0x05])  # a fresh transaction: a new command
        assert receive(device, 1) == b"\x00"
        cs_high(device)


class TestLedOnAPin:
    def test_driving_the_pin_lights_the_led(self, device):
        led = device.dev["LED"]
        gpiob = device.hw.perif["gpiob"]
        led.reset()

        gpiob.write(GPIOB_BASE + PIO_SODR, 4, 1 << LED_PIN)
        assert led.state is True
        assert led.transitions == 1

        gpiob.write(GPIOB_BASE + PIO_CODR, 4, 1 << LED_PIN)
        assert led.state is False
        assert led.transitions == 2

    def test_other_pins_do_not_move_the_led(self, device):
        led = device.dev["LED"]
        gpiob = device.hw.perif["gpiob"]
        led.reset()

        gpiob.write(GPIOB_BASE + PIO_SODR, 4, 1 << 12)
        assert led.transitions == 0

    def test_a_real_firmware_blinks_it(self, sam3x8e_path, sam3x8e_firmware):
        """Blink toggles LED_BUILTIN, which on the Due is PB27."""
        device = XuanWu(str(sam3x8e_path), str(sam3x8e_firmware))
        device.reset()
        led = device.dev["LED"]
        led.reset()

        for _ in range(20):
            device.run(count=200_000)
            if led.transitions >= 2:
                break

        assert led.transitions >= 2, "the firmware never toggled the LED pin"

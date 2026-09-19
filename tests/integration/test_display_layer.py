# -*- coding: utf-8 -*-

"""The TFT shield on a real Due, driven through memory-mapped registers.

Everything here goes the long way round on purpose: a write to ``GPIOD_CODR`` selects
the panel, a write to the data/command pin says what the next ``SPI_TDR`` write means,
and the pixels end up in a framebuffer.  That is the whole path -- PIO adapter, GPIO
core, chip-select fan-out, SPI adapter, ILI9341 -- so it is also the test that would
catch a broken pin name, a swapped offset or a bus that routes to the wrong device.
"""

import logging

import pytest

from xuanwu import XuanWu
from xuanwu.devices import Ili9341Device, SpiFlash
from xuanwu.peripherals.bus.spi import SpiBusSelector
from xuanwu.peripherals.display import rgb565

pytestmark = pytest.mark.integration

SPI_BASE = 0x40008000
SPI_CR = SPI_BASE + 0x00
SPI_RDR = SPI_BASE + 0x08
SPI_TDR = SPI_BASE + 0x0C

GPIO_BASE = {"gpioc": 0x400E1200}
PIO_PER = 0x00
PIO_OER = 0x10
PIO_SODR = 0x30
PIO_CODR = 0x34

# The shield's Arduino pin numbers, as the Due's variant table maps them.
LCD_CS = 29  # D10 -> PC29
LCD_DC = 21  # D9  -> PC21
FLASH_CS = 26  # D4 -> PC26, the microSD socket
SPARE_PIN = 25  # D5, with nothing wired to it

RED = rgb565(255, 0, 0)
GREEN = rgb565(0, 255, 0)
BLUE = rgb565(0, 0, 255)
BLACK = rgb565(0, 0, 0)


@pytest.fixture(scope="module")
def device(sam3x8e_tft_path, sam3x8e_firmware):
    logging.disable(logging.CRITICAL)
    device = XuanWu(str(sam3x8e_tft_path), str(sam3x8e_firmware), hardware_options={"bridge": "none"})
    device.reset()
    # What pinMode() + the SPI library's begin() would have done before any drawing:
    # the panel's chip select and D/C become outputs, CS idles high, SPI is enabled.
    pins = (1 << LCD_CS) | (1 << LCD_DC)
    gpio_write(device, "gpioc", PIO_PER, pins)
    gpio_write(device, "gpioc", PIO_OER, pins)
    gpio_write(device, "gpioc", PIO_SODR, 1 << LCD_CS)
    device.hw.perif["spi"].write(SPI_CR, 4, 1 << 0)  # SPIEN
    return device


def gpio_write(device, port: str, offset: int, value: int) -> None:
    device.hw.perif[port].write(GPIO_BASE[port] + offset, 4, value)


def cs_low(device) -> None:
    gpio_write(device, "gpioc", PIO_CODR, 1 << LCD_CS)


def cs_high(device) -> None:
    gpio_write(device, "gpioc", PIO_SODR, 1 << LCD_CS)


def dc_command(device) -> None:
    gpio_write(device, "gpioc", PIO_CODR, 1 << LCD_DC)


def dc_data(device) -> None:
    gpio_write(device, "gpioc", PIO_SODR, 1 << LCD_DC)


def send(device, values) -> None:
    spi = device.hw.perif["spi"]
    for value in values:
        spi.write(SPI_TDR, 4, value)


def send_command(device, value: int, params=()) -> None:
    """One command byte with D/C low, then its parameters with D/C high."""
    cs_low(device)
    dc_command(device)
    send(device, [value])
    dc_data(device)
    send(device, params)
    cs_high(device)


def fill_rect(device, x: int, y: int, width: int, height: int, color: int) -> None:
    send_command(device, 0x2A, [x >> 8, x & 0xFF, (x + width - 1) >> 8, (x + width - 1) & 0xFF])
    send_command(device, 0x2B, [y >> 8, y & 0xFF, (y + height - 1) >> 8, (y + height - 1) & 0xFF])
    cs_low(device)
    dc_command(device)
    send(device, [0x2C])  # RAMWR
    dc_data(device)
    high, low = (color >> 8) & 0xFF, color & 0xFF
    for _ in range(width * height):
        send(device, [high, low])
    cs_high(device)


def panel(device) -> Ili9341Device:
    return device.dev["LCD"]


@pytest.fixture(autouse=True)
def release_bus(device):
    """Every test leaves the bus idle, so the next one starts from a known state."""
    yield
    cs_high(device)
    dc_data(device)


class TestLoading:
    def test_the_shield_adds_the_panel_to_the_due(self, device):
        assert device.dev.names() == ["LED", "FLASH", "LCD"]
        assert isinstance(panel(device), Ili9341Device)

    def test_both_spi_devices_share_one_controller(self, device):
        bus = device.hw.perif["spi"].bridge
        assert isinstance(bus, SpiBusSelector)
        assert set(bus.devices) == {"FLASH", "LCD"}
        assert isinstance(bus.device_at("FLASH"), SpiFlash)

    def test_the_panel_starts_black_and_unselected(self, device):
        assert panel(device).core.gram.count(BLACK) == 240 * 320
        assert device.hw.perif["spi"].bridge.selected is None


class TestWiring:
    def test_a_command_reaches_the_panel(self, device):
        send_command(device, 0x29)  # DISPON
        assert panel(device).core.display_on is True

    def test_a_parameter_goes_where_the_command_left_off(self, device):
        send_command(device, 0x36, [0x48])  # MADCTL
        assert panel(device).core.madctl == 0x48
        assert device.hw.perif["spi"].bridge.selected is None, "CS is released at the end"

    def test_the_chip_select_decides_who_hears_the_bytes(self, device):
        bus = device.hw.perif["spi"].bridge
        gpio_write(device, "gpioc", PIO_CODR, 1 << SPARE_PIN)  # a pin with no device
        routed, unclaimed = bus.routed_bytes, bus.unclaimed_bytes
        send(device, [0x9F])
        assert bus.selected is None
        assert bus.routed_bytes == routed, "no device is selected, so nobody hears it"
        assert bus.unclaimed_bytes == unclaimed + 1
        gpio_write(device, "gpioc", PIO_SODR, 1 << SPARE_PIN)

    def test_the_flash_still_answers_on_the_same_controller(self, device):
        bus = device.hw.perif["spi"].bridge
        spi = device.hw.perif["spi"]
        commands_before = len(panel(device).core.commands)

        gpio_write(device, "gpioc", PIO_CODR, 1 << FLASH_CS)
        answer = bytearray()
        for value in (0x9F, 0x00, 0x00, 0x00):  # JEDEC id, read like a driver does
            spi.write(SPI_TDR, 4, value)
            answer.append(spi.read(SPI_RDR, 4) & 0xFF)
        assert bus.selected == "FLASH"
        gpio_write(device, "gpioc", PIO_SODR, 1 << FLASH_CS)

        assert bytes(answer[1:]) == bytes([0xEF, 0x40, 0x18])
        assert len(panel(device).core.commands) == commands_before, "the panel saw nothing"


class TestDrawing:
    def test_a_filled_rectangle_lands_where_it_was_asked_for(self, device):
        fill_rect(device, 10, 20, 3, 2, RED)
        surface = panel(device).surface
        assert surface.pixel(10, 20) == RED
        assert surface.pixel(12, 21) == RED
        assert surface.pixel(13, 21) == BLACK
        assert surface.count(RED) == 6

    def test_a_whole_screen_fill(self, device):
        fill_rect(device, 0, 0, 240, 320, BLUE)
        assert panel(device).core.gram.count(BLUE) == 240 * 320

    def test_the_rotation_the_library_sets_is_honoured(self, device):
        send_command(device, 0x36, [0x28])  # MV | BGR: landscape
        assert panel(device).size == (320, 240)
        fill_rect(device, 5, 6, 2, 2, GREEN)
        surface = panel(device).surface
        assert surface.size == (320, 240)
        assert surface.pixel(5, 6) == GREEN
        assert surface.pixel(6, 7) == GREEN
        assert surface.count(GREEN) == 4
        send_command(device, 0x36, [0x48])  # back to portrait

    def test_the_panel_can_be_saved_as_a_png(self, device, tmp_path):
        fill_rect(device, 0, 0, 4, 4, RED)
        path = panel(device).save(str(tmp_path / "panel.png"))
        assert open(path, "rb").read()[:8] == b"\x89PNG\r\n\x1a\n"

    def test_a_viewer_sees_the_frames(self, device):
        viewer = panel(device).viewer
        before = viewer.updates
        panel(device).refresh()
        assert viewer.updates == before + 1
        assert viewer.frame.size == (240, 320)

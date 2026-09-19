# -*- coding: utf-8 -*-

"""The TFT shield with Adafruit's library actually drawing on it.

`Tft_smoke_m3.ino` uses the real library -- `Adafruit_ILI9341`, `Adafruit_GFX`,
`Adafruit_BusIO` and the SAM core's `SPI` -- on a small, known set of shapes, so
everything it paints can be checked pixel by pixel from the host.  The whole path is
under test here: ``SPI.begin()``, the register reads the library performs at start-up,
the initialisation sequence, window addressing, the pixel stream from
``SPI.transfer(buf, len)``, rotation, and the digitiser-free subset of the shield (the
microSD stays silent because its chip select is never asserted).

The full `graphicstest` run -- megabytes of pixels, minutes of host time -- is
``test_due_tft_milestone.py``.
"""

import logging
from types import SimpleNamespace

import pytest

from xuanwu import XuanWu
from xuanwu.peripherals.display import rgb565

pytestmark = [pytest.mark.integration, pytest.mark.slow]

RED = rgb565(255, 0, 0)
GREEN = rgb565(0, 255, 0)
BLUE = rgb565(0, 0, 255)
CYAN = rgb565(0, 255, 255)
YELLOW = rgb565(255, 255, 0)
WHITE = rgb565(255, 255, 255)
BLACK = rgb565(0, 0, 0)

# Where the library puts things, in the orientation the sketch drew them.
RECT_X, RECT_Y, RECT_W, RECT_H = 10, 20, 30, 40
FRAME_X, FRAME_Y, FRAME_W, FRAME_H = 50, 60, 40, 30
MARKER_X, MARKER_Y = 5, 6  # drawn in landscape, checked in portrait further down
TEXT_X, TEXT_Y = 4, 200

# The register values the library reads back before it draws anything.
POWER_MODE = 0x9C
MADCTL_PORTRAIT = 0x48  # MX | BGR, which is what setRotation(0) writes
PIXEL_FORMAT = 0x55  # 16 bpp


@pytest.fixture(scope="module")
def tft(sam3x8e_tft_path, due_tft_smoke_firmware):
    """Run the sketch once; every test below looks at that one run."""
    logging.disable(logging.CRITICAL)
    device = XuanWu(
        str(sam3x8e_tft_path), str(due_tft_smoke_firmware), hardware_options={"bridge": "loopback"}
    )
    device.reset()
    bridge = device.hw.perif["uart"]._bridge
    panel = device.dev["LCD"]

    text = b""
    for _ in range(120):
        device.run(count=5_000_000)
        text += bridge.drain()
        if b"smoke done" in text:
            break
    else:
        pytest.fail(f"the sketch never reached its end: {text.decode('utf-8', 'replace')!r}")

    logging.disable(logging.NOTSET)
    return SimpleNamespace(device=device, panel=panel, text=text.decode("utf-8", "replace"))


def lines(tft) -> list:
    return [line.strip() for line in tft.text.splitlines() if line.strip()]


class TestBringUp:
    def test_the_sketch_runs_to_the_end(self, tft):
        assert lines(tft)[0] == "TFT smoke"
        assert lines(tft)[-1] == "smoke done"

    def test_the_library_reads_back_the_registers_it_wrote(self, tft):
        """`Adafruit_ILI9341::readcommand8()` against the panel model."""
        assert f"power mode {POWER_MODE:X}" in lines(tft)
        assert f"madctl {MADCTL_PORTRAIT:X}" in lines(tft)
        assert f"pixfmt {PIXEL_FORMAT:X}" in lines(tft)

    def test_the_library_sees_the_size_it_rotated_to(self, tft):
        assert "size 240x320" in lines(tft), "portrait"
        assert "size 320x240" in lines(tft), "landscape"

    def test_the_panel_was_initialised_and_left_on(self, tft):
        panel = tft.panel.core
        assert panel.display_on is True
        assert not panel.sleeping
        assert panel.colmod == PIXEL_FORMAT
        # The sequence the library sends, in order.
        commands = panel.commands
        assert commands[0] == 0x01, "a software reset comes first"
        assert commands[-1] != 0x01
        assert 0x11 in commands and 0x29 in commands, "sleep out and display on"
        assert 0x36 in commands, "MADCTL, for the rotation"
        assert 0x3A in commands, "COLMOD, for the pixel format"

    def test_the_panel_got_the_bytes_and_nothing_else_did(self, tft):
        bus = tft.device.hw.perif["spi"].bridge
        assert tft.panel.bytes_received > 100_000
        assert bus.unclaimed_bytes == 0, "every byte had a device to go to"


class TestWhatWasDrawn:
    def test_the_filled_rectangle_is_exactly_where_it_was_asked_for(self, tft):
        surface = tft.panel.surface
        assert surface.count(RED) == RECT_W * RECT_H
        assert surface.pixel(RECT_X, RECT_Y) == RED
        assert surface.pixel(RECT_X + RECT_W - 1, RECT_Y + RECT_H - 1) == RED
        assert surface.pixel(RECT_X - 1, RECT_Y) == BLACK
        assert surface.pixel(RECT_X + RECT_W, RECT_Y + RECT_H - 1) == BLACK

    def test_the_outline_is_a_frame_and_not_a_block(self, tft):
        surface = tft.panel.surface
        outline = 2 * FRAME_W + 2 * FRAME_H - 4
        assert surface.count(GREEN) == outline, "only the border is drawn"
        assert surface.pixel(FRAME_X, FRAME_Y) == GREEN
        assert surface.pixel(FRAME_X + FRAME_W - 1, FRAME_Y + FRAME_H - 1) == GREEN
        assert surface.pixel(FRAME_X + FRAME_W // 2, FRAME_Y + FRAME_H // 2) == BLACK, "hollow"

    def test_the_fast_line_covers_a_whole_row(self, tft):
        surface = tft.panel.surface
        assert surface.count(BLUE) == surface.width
        assert surface.pixel(0, 0) == BLUE
        assert surface.pixel(surface.width - 1, 0) == BLUE
        assert surface.pixel(0, 1) == BLACK

    def test_a_single_pixel_stayed_a_single_pixel(self, tft):
        surface = tft.panel.surface
        assert surface.pixel(200, 300) == WHITE
        assert surface.count(WHITE) == 1

    def test_the_text_landed_in_its_cell(self, tft):
        surface = tft.panel.surface
        drawn = [
            (x, y)
            for y in range(TEXT_Y, TEXT_Y + 16)
            for x in range(TEXT_X, TEXT_X + 6 * 12)
            if surface.pixel(x, y) == YELLOW
        ]
        assert drawn, "no text pixels at all"
        assert surface.count(YELLOW) == len(drawn)
        assert min(x for x, _ in drawn) >= TEXT_X
        assert min(y for _, y in drawn) >= TEXT_Y

    def test_nothing_was_drawn_outside_the_expected_shapes(self, tft):
        """Two colours are enough to say the frame is not a smear."""
        surface = tft.panel.surface
        histogram = surface.histogram()
        assert set(histogram) <= {BLACK, BLUE, RED, GREEN, CYAN, YELLOW, WHITE}
        assert histogram[BLACK] > surface.width * surface.height // 2


class TestRotation:
    def test_a_marker_drawn_in_landscape_comes_back_rotated(self, tft):
        """The same memory, read through the other orientation.

        Landscape is ``MV``: logical (x, y) is stored at column y, row x.  Back in
        portrait ``MX`` mirrors the column, so 2x2 at (5, 6) is the 2x2 at
        (239 - 7 .. 239 - 6, 5 .. 6) = (232 .. 233, 5 .. 6).
        """
        surface = tft.panel.surface
        expected = {
            (239 - MARKER_Y, MARKER_X),
            (239 - MARKER_Y - 1, MARKER_X),
            (239 - MARKER_Y, MARKER_X + 1),
            (239 - MARKER_Y - 1, MARKER_X + 1),
        }
        found = {(x, y) for y in range(4, 8) for x in range(230, 236) if surface.pixel(x, y) == CYAN}
        assert found == expected
        assert surface.count(CYAN) == 4

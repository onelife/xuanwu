# -*- coding: utf-8 -*-

"""The ILI9341 controller, driven the way a display library drives it.

A library sends a command byte with D/C low and parameters with D/C high, sets an
address window with ``CASET``/``PASET``, and then streams pixels into ``RAMWR``.  What is
checked here is that the pixels land where the sketch asked for, in all four
orientations, and that the registers a library reads back say what it just wrote.
"""

import pytest

from xuanwu.peripherals.display import DisplaySurface, Ili9341, rgb565

RED = rgb565(255, 0, 0)
GREEN = rgb565(0, 255, 0)
BLUE = rgb565(0, 0, 255)
BLACK = rgb565(0, 0, 0)
WHITE = rgb565(255, 255, 255)


class Library:
    """What Adafruit_ILI9341 does, reduced to the calls that move pixels."""

    def __init__(self, panel: Ili9341 = None) -> None:
        self.panel = panel or Ili9341()

    def command(self, value: int, *params: int) -> None:
        self.panel.write_command(value)
        for param in params:
            self.panel.write_data(param)

    def set_rotation(self, rotation: int) -> None:
        """The library's own table, BGR included."""
        table = {0: 0x48, 1: 0x28, 2: 0x88, 3: 0xE8}
        self.command(0x36, table[rotation % 4])

    def begin(self) -> None:
        self.command(0x11)  # SLPOUT
        self.command(0x3A, 0x55)  # COLMOD: 16 bpp
        self.command(0x36, 0x48)  # MADCTL: MX | BGR
        self.command(0x29)  # DISPON

    def set_window(self, x: int, y: int, width: int, height: int) -> None:
        self.command(0x2A, x >> 8, x & 0xFF, (x + width - 1) >> 8, (x + width - 1) & 0xFF)
        self.command(0x2B, y >> 8, y & 0xFF, (y + height - 1) >> 8, (y + height - 1) & 0xFF)

    def fill_rect(self, x: int, y: int, width: int, height: int, color: int) -> None:
        self.set_window(x, y, width, height)
        self.command(0x2C)
        high, low = (color >> 8) & 0xFF, color & 0xFF
        for _ in range(width * height):
            self.panel.write_data(high)
            self.panel.write_data(low)

    def fill_screen(self, color: int) -> None:
        width, height = self.panel.size
        self.fill_rect(0, 0, width, height, color)


@pytest.fixture
def library() -> Library:
    display = Library()
    display.begin()
    return display


class TestInitialisation:
    def test_a_reset_panel_is_asleep_with_the_display_off(self):
        panel = Ili9341()
        assert panel.sleeping
        assert not panel.display_on
        assert panel.size == (240, 320)
        assert panel.rotation == 0

    def test_waking_up_and_turning_the_display_on(self, library):
        assert not library.panel.sleeping
        assert library.panel.display_on

    def test_the_colour_format_is_read_back(self, library):
        assert library.panel.colmod == 0x55
        assert library.panel.colour_bits == 16

    def test_the_madctl_value_is_read_back(self, library):
        library.panel.write_command(0x0B)  # RDDMADCTL
        answer = library.panel.take_response()
        assert answer == bytes([0x00, 0x48])

    def test_the_identification_is_read_back(self, library):
        library.panel.write_command(0x04)  # RDDID
        assert library.panel.take_response() == bytes([0x00, 0x93, 0x41])

    def test_an_unknown_command_swallows_its_parameters(self, library):
        """The undocumented init commands must not be mistaken for pixel data."""
        library.command(0xEF, 0x03, 0x80, 0x02)
        library.command(0xCF, 0x00, 0xC1, 0x30)
        assert library.panel.gram.count(BLACK) == 240 * 320, "nothing should have been drawn"


class TestPixels:
    def test_filling_the_screen_leaves_every_pixel_the_colour(self, library):
        library.fill_screen(RED)
        assert library.panel.gram.count(RED) == 240 * 320
        assert library.panel.pixel(0, 0) == RED
        assert library.panel.pixel(239, 319) == RED

    def test_a_window_only_touches_its_own_rectangle(self, library):
        library.fill_screen(BLACK)
        library.fill_rect(10, 20, 3, 2, GREEN)
        assert library.panel.pixel(10, 20) == GREEN
        assert library.panel.pixel(12, 21) == GREEN
        assert library.panel.pixel(13, 21) == BLACK
        assert library.panel.pixel(9, 20) == BLACK
        assert library.panel.gram.count(GREEN) == 6

    def test_the_address_counter_wraps_inside_the_window(self, library):
        library.fill_screen(BLACK)
        library.set_window(0, 0, 2, 2)
        library.command(0x2C)
        for color in (RED, GREEN, BLUE, WHITE, RED):
            library.panel.write_data((color >> 8) & 0xFF)
            library.panel.write_data(color & 0xFF)
        assert library.panel.pixel(0, 0) == RED, "the fifth pixel wraps back to the start"
        assert library.panel.pixel(1, 0) == GREEN
        assert library.panel.pixel(0, 1) == BLUE
        assert library.panel.pixel(1, 1) == WHITE

    def test_half_a_pixel_is_kept_until_its_second_byte(self, library):
        library.set_window(0, 0, 1, 1)
        library.command(0x2C)
        library.panel.write_data(0xF8)
        assert library.panel.pixel(0, 0) == BLACK, "one byte is not a pixel"
        library.panel.write_data(0x00)
        assert library.panel.pixel(0, 0) == RED

    def test_the_continue_command_keeps_the_window(self, library):
        library.fill_screen(BLACK)
        library.set_window(0, 0, 1, 2)
        library.command(0x2C)
        for _ in range(1):
            library.panel.write_data(0xF8)
            library.panel.write_data(0x00)
        library.command(0x3C)  # RAMWRC
        library.panel.write_data(0x07)
        library.panel.write_data(0xE0)
        assert library.panel.pixel(0, 0) == RED
        assert library.panel.pixel(0, 1) == GREEN

    def test_a_whole_screen_fill_counts_as_one_dirty_box(self, library):
        library.panel.gram.clear_dirty()
        library.fill_screen(BLUE)
        assert library.panel.gram.dirty == (0, 0, 240, 320)


class TestOrientation:
    @pytest.mark.parametrize(
        "rotation, size",
        [(0, (240, 320)), (1, (320, 240)), (2, (240, 320)), (3, (320, 240))],
    )
    def test_the_size_the_firmware_sees(self, library, rotation, size):
        library.set_rotation(rotation)
        assert library.panel.size == size
        assert library.panel.rotation == rotation

    @pytest.mark.parametrize("rotation", [0, 1, 2, 3])
    def test_a_pixel_lands_where_the_sketch_drew_it(self, library, rotation):
        library.set_rotation(rotation)
        library.fill_screen(BLACK)
        width, height = library.panel.size
        library.fill_rect(5, 6, 1, 1, RED)
        snapshot = library.panel.snapshot()
        assert snapshot.pixel(5, 6) == RED, "the snapshot must be in the same coordinates"
        assert snapshot.size == (width, height)
        assert snapshot.count(RED) == 1

    def test_rotation_swaps_the_gram_axes(self, library):
        library.set_rotation(1)
        library.fill_screen(BLACK)
        library.fill_rect(5, 6, 1, 1, RED)
        column, row = library.panel.gram_coords(5, 6)
        assert (column, row) == (6, 5), "landscape exchanges the axes in the panel memory"
        assert library.panel.gram.pixel(6, 5) == RED

    @pytest.mark.parametrize(
        "rotation, expected",
        [(0, (239, 0)), (1, (0, 0)), (2, (0, 319)), (3, (239, 319))],
    )
    def test_the_gram_corner_each_rotation_starts_from(self, library, rotation, expected):
        library.set_rotation(rotation)
        assert library.panel.gram_coords(0, 0) == expected

    def test_the_colour_order_bits_do_not_move_a_pixel(self, library):
        """BGR/ML/MH change how the panel is scanned, not where a pixel is stored."""
        library.fill_screen(BLACK)
        library.command(0x36, 0x40)  # MX only
        library.fill_rect(3, 4, 1, 1, RED)
        column, row = library.panel.gram_coords(3, 4)
        assert library.panel.gram.pixel(column, row) == RED


class TestReadBack:
    def test_reading_a_pixel_returns_it_after_a_dummy_byte(self, library):
        library.fill_screen(BLACK)
        library.fill_rect(2, 3, 1, 1, GREEN)
        library.command(0x2A, 0x00, 0x02, 0x00, 0x02)
        library.command(0x2B, 0x00, 0x03, 0x00, 0x03)
        library.panel.write_command(0x2E)  # RAMRD
        answer = library.panel.take_response()
        assert answer[0] == 0x00, "the first byte of a read is a dummy"
        assert (answer[1] << 8) | answer[2] == GREEN

    def test_the_status_says_whether_the_display_is_on(self, library):
        library.panel.write_command(0x0E)  # RDDSM
        answer = library.panel.take_response()
        assert answer[1] & 0x04, "DISPON was sent, so the status bit must be set"


class TestFramebuffer:
    def test_the_snapshot_is_a_surface_of_the_right_size(self, library):
        image = library.panel.snapshot()
        assert isinstance(image, DisplaySurface)
        assert image.size == (240, 320)
        assert image.count(BLACK) == 240 * 320

    def test_the_hash_follows_the_pixels(self, library):
        library.fill_screen(RED)
        first = library.panel.snapshot().digest()
        library.fill_screen(GREEN)
        second = library.panel.snapshot().digest()
        assert first != second
        library.fill_screen(RED)
        assert library.panel.snapshot().digest() == first, "the same drawing hashes the same"

    def test_the_bit_map_is_visible_in_the_gram(self, library):
        """The panel memory is 240x320 whatever the orientation."""
        assert library.panel.gram.size == (240, 320)
        library.set_rotation(1)
        assert library.panel.gram.size == (240, 320)

    def test_the_snapshot_follows_a_rotation_that_draws_nothing(self, library):
        """The frame buffer is cached, so a rotation has to invalidate it.

        Otherwise a firmware that rotates at the end of a frame -- which is exactly what
        `graphicstest` does -- would be read back in the orientation it left behind.
        """
        library.fill_screen(RED)
        library.set_rotation(0)
        assert library.panel.snapshot().size == (240, 320)
        library.set_rotation(1)
        assert library.panel.snapshot().size == (320, 240)
        assert library.panel.snapshot().count(RED) == 240 * 320

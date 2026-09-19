# -*- coding: utf-8 -*-

"""The framebuffer: pixels, dirty regions, hashes and the PNG writer.

Everything a test asserts about a display comes through here, so it has to be exact
and cheap.  The PNG writer is checked by decoding what it wrote, because "the file
exists" is not the same as "the picture is right".
"""

import struct
import zlib

import pytest

from xuanwu.peripherals.display import DisplaySurface, rgb565

RED = rgb565(255, 0, 0)
GREEN = rgb565(0, 255, 0)
BLUE = rgb565(0, 0, 255)
BLACK = rgb565(0, 0, 0)
WHITE = rgb565(255, 255, 255)
GREY = rgb565(128, 128, 128)


class TestColours:
    def test_the_packing_is_the_panel_order(self):
        assert RED == 0xF800
        assert GREEN == 0x07E0
        assert BLUE == 0x001F
        assert WHITE == 0xFFFF
        assert BLACK == 0x0000

    def test_the_low_bits_are_dropped_the_way_the_panel_drops_them(self):
        # 8-bit 0x07 has nothing left in the top five bits of red.
        assert rgb565(0x07, 0x00, 0x00) == 0x0000
        # 0xF8 keeps all five.
        assert rgb565(0xF8, 0x00, 0x00) == 0xF800


class TestPixels:
    def test_a_new_surface_is_black(self):
        surface = DisplaySurface(4, 3)
        assert surface.size == (4, 3)
        assert surface.count(BLACK) == 12

    def test_a_pixel_can_be_written_and_read(self):
        surface = DisplaySurface(4, 3)
        surface.set_pixel(2, 1, RED)
        assert surface.pixel(2, 1) == RED
        assert surface.count(RED) == 1

    def test_writing_outside_the_surface_is_ignored(self):
        surface = DisplaySurface(4, 3)
        surface.set_pixel(4, 0, RED)
        surface.set_pixel(-1, 0, RED)
        surface.set_pixel(0, 3, RED)
        assert surface.count(RED) == 0

    def test_a_fill_covers_everything(self):
        surface = DisplaySurface(3, 2)
        surface.fill(BLUE)
        assert surface.count(BLUE) == 6

    def test_a_rectangle_only_covers_itself(self):
        surface = DisplaySurface(8, 8)
        surface.fill_rect(2, 3, 3, 2, GREEN)
        assert surface.count(GREEN) == 6
        assert surface.pixel(2, 3) == GREEN
        assert surface.pixel(4, 4) == GREEN
        assert surface.pixel(5, 4) == BLACK

    def test_a_rectangle_is_clipped_to_the_surface(self):
        surface = DisplaySurface(4, 4)
        surface.fill_rect(3, 3, 4, 4, WHITE)
        assert surface.count(WHITE) == 1
        assert surface.pixel(3, 3) == WHITE


class TestDirtyRegions:
    def test_a_fresh_surface_has_nothing_dirty(self):
        assert DisplaySurface(4, 4).dirty is None

    def test_one_pixel_is_a_one_pixel_box(self):
        surface = DisplaySurface(4, 4)
        surface.set_pixel(1, 2, RED)
        assert surface.dirty == (1, 2, 2, 3)

    def test_two_pixels_give_the_box_around_both(self):
        surface = DisplaySurface(8, 8)
        surface.set_pixel(1, 1, RED)
        surface.set_pixel(5, 6, RED)
        assert surface.dirty == (1, 1, 6, 7)

    def test_a_fill_covers_the_whole_surface(self):
        surface = DisplaySurface(4, 4)
        surface.fill(BLUE)
        assert surface.dirty == (0, 0, 4, 4)

    def test_the_box_can_be_cleared(self):
        surface = DisplaySurface(4, 4)
        surface.set_pixel(1, 1, RED)
        surface.clear_dirty()
        assert surface.dirty is None
        surface.set_pixel(2, 2, RED)
        assert surface.dirty == (2, 2, 3, 3)

    def test_writing_the_same_value_still_counts_as_a_change(self):
        """The panel cannot know the pixel was already that colour."""
        surface = DisplaySurface(4, 4, BLACK)
        surface.set_pixel(0, 0, BLACK)
        assert surface.dirty == (0, 0, 1, 1)


class TestComparing:
    def test_a_copy_is_independent(self):
        surface = DisplaySurface(4, 4)
        surface.set_pixel(1, 1, RED)
        copy = surface.copy()
        assert copy.pixel(1, 1) == RED
        copy.set_pixel(1, 1, GREEN)
        assert surface.pixel(1, 1) == RED

    def test_the_hash_follows_the_content(self):
        one, two = DisplaySurface(4, 4), DisplaySurface(4, 4)
        assert one.digest() == two.digest()
        two.set_pixel(0, 0, WHITE)
        assert one.digest() != two.digest()

    def test_the_histogram_counts_every_value(self):
        surface = DisplaySurface(4, 4)
        surface.fill_rect(0, 0, 4, 3, RED)
        histogram = surface.histogram()
        assert histogram[RED] == 12
        assert histogram[BLACK] == 4

    def test_the_bounding_box_of_a_colour(self):
        surface = DisplaySurface(8, 8)
        surface.fill_rect(2, 3, 3, 2, GREEN)
        assert surface.bounding_box(GREEN) == (2, 3, 5, 5)
        assert surface.bounding_box(RED) is None

    def test_the_writes_counter_notices_activity(self):
        surface = DisplaySurface(4, 4)
        before = surface.writes
        surface.set_pixel(0, 0, RED)
        assert surface.writes == before + 1


class TestPng:
    def test_the_header_says_what_the_image_is(self, tmp_path):
        surface = DisplaySurface(3, 2)
        path = surface.to_png(str(tmp_path / "image.png"))
        data = open(path, "rb").read()
        assert data[:8] == b"\x89PNG\r\n\x1a\n"
        length = struct.unpack(">I", data[8:12])[0]
        assert data[12:16] == b"IHDR"
        width, height, depth, colour = struct.unpack(">IIBB", data[16 : 16 + 10])
        assert (width, height, depth, colour) == (3, 2, 8, 2)
        assert length == 13

    def test_the_pixels_survive_the_round_trip(self, tmp_path):
        surface = DisplaySurface(4, 4)
        surface.set_pixel(0, 0, RED)
        surface.set_pixel(1, 0, GREEN)
        surface.set_pixel(2, 0, BLUE)
        surface.set_pixel(3, 3, WHITE)
        path = surface.to_png(str(tmp_path / "pixels.png"))
        pixels = _decode_png(path)
        assert pixels[(0, 0)] == (255, 0, 0)
        assert pixels[(1, 0)] == (0, 255, 0)
        assert pixels[(2, 0)] == (0, 0, 255)
        assert pixels[(3, 3)] == (255, 255, 255)
        assert pixels[(0, 1)] == (0, 0, 0)

    def test_the_file_is_a_real_png(self, tmp_path):
        path = DisplaySurface(2, 2).to_png(str(tmp_path / "small.png"))
        assert len(_decode_png(path)) == 4


def _decode_png(path: str):
    """The smallest PNG reader that can prove the writer right."""
    data = open(path, "rb").read()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    offset = 8
    width = height = 0
    idat = b""
    while offset < len(data):
        length = struct.unpack(">I", data[offset : offset + 4])[0]
        tag = data[offset + 4 : offset + 8]
        payload = data[offset + 8 : offset + 8 + length]
        if tag == b"IHDR":
            width, height = struct.unpack(">II", payload[:8])
        elif tag == b"IDAT":
            idat += payload
        offset += 12 + length
    raw = zlib.decompress(idat)
    pixels = {}
    stride = width * 3
    for y in range(height):
        row = raw[y * (stride + 1) : (y + 1) * (stride + 1)]
        assert row[0] == 0, "every row is written with filter type 0"
        for x in range(width):
            red, green, blue = row[1 + x * 3 : 4 + x * 3]
            pixels[(x, y)] = (red, green, blue)
    return pixels


class TestGuards:
    @pytest.mark.parametrize("size", [(0, 1), (1, 0), (-1, 2)])
    def test_a_surface_needs_a_size(self, size):
        with pytest.raises(ValueError):
            DisplaySurface(*size)

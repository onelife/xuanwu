# -*- coding: utf-8 -*-

"""An RGB565 framebuffer with dirty tracking, a hash and a PNG writer.

The buffer is the *displayed* image, in the orientation the firmware drew in, so a
test can compare pixels against the coordinates the sketch used.  Everything here is
plain Python: a viewer is an optional extra, not a requirement to be able to look at
what a firmware painted.
"""

import hashlib
import struct
import zlib
from typing import Dict, Optional, Tuple

__all__ = ["DisplaySurface", "rgb565"]

Rect = Tuple[int, int, int, int]


def rgb565(red: int, green: int, blue: int) -> int:
    """Pack 8-bit components the way the panels do: 5 bits red, 6 green, 5 blue."""
    return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue & 0xF8) >> 3)


def _expand565(value: int) -> Tuple[int, int, int]:
    red = (value >> 11) & 0x1F
    green = (value >> 5) & 0x3F
    blue = value & 0x1F
    return (red << 3) | (red >> 2), (green << 2) | (green >> 4), (blue << 3) | (blue >> 2)


class DisplaySurface:
    """A width x height RGB565 image, with the pixels in row order.

    ``pixels`` is a ``bytearray`` of ``width * height * 2`` bytes, little-endian per
    pixel, which is what the panels, the tests and a blit all want; the accessors take
    and return the 16-bit value.
    """

    def __init__(self, width: int, height: int, color: int = 0x0000) -> None:
        if width <= 0 or height <= 0:
            raise ValueError(f"a surface needs a positive size, got {width}x{height}")
        self.width = int(width)
        self.height = int(height)
        self.pixels = bytearray(self.width * self.height * 2)
        if color:
            self.fill(color)
        self.dirty: Optional[Rect] = None
        """Bounding box of what changed since :meth:`clear_dirty`, or None."""
        self.writes = 0
        """How many pixels have been written; a cheap activity counter."""

    # -- geometry ---------------------------------------------------------

    @property
    def size(self) -> Tuple[int, int]:
        return self.width, self.height

    def contains(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    # -- pixels -----------------------------------------------------------

    def pixel(self, x: int, y: int) -> int:
        offset = (y * self.width + x) * 2
        return self.pixels[offset] | (self.pixels[offset + 1] << 8)

    def set_pixel(self, x: int, y: int, color: int) -> None:
        if not self.contains(x, y):
            return
        offset = (y * self.width + x) * 2
        self.pixels[offset] = color & 0xFF
        self.pixels[offset + 1] = (color >> 8) & 0xFF
        self._touch(x, y)
        self.writes += 1

    def row(self, y: int) -> bytes:
        """One row's raw bytes (for a blit or a PNG)."""
        start = y * self.width * 2
        return bytes(self.pixels[start : start + self.width * 2])

    def fill(self, color: int) -> None:
        self.pixels[:] = bytes([color & 0xFF, (color >> 8) & 0xFF]) * (self.width * self.height)
        self.dirty = (0, 0, self.width, self.height)
        self.writes += self.width * self.height

    def fill_rect(self, x: int, y: int, width: int, height: int, color: int) -> None:
        left, top = max(x, 0), max(y, 0)
        right, bottom = min(x + width, self.width), min(y + height, self.height)
        if right <= left or bottom <= top:
            return
        row = bytes([color & 0xFF, (color >> 8) & 0xFF]) * (right - left)
        for row_index in range(top, bottom):
            start = (row_index * self.width + left) * 2
            self.pixels[start : start + (right - left) * 2] = row
        self._touch_rect(left, top, right - left, bottom - top)
        self.writes += (right - left) * (bottom - top)

    def copy(self) -> "DisplaySurface":
        other = DisplaySurface.__new__(DisplaySurface)
        other.width = self.width
        other.height = self.height
        other.pixels = bytearray(self.pixels)
        other.dirty = None
        other.writes = 0
        return other

    # -- what a viewer needs ----------------------------------------------

    def clear_dirty(self) -> None:
        self.dirty = None

    def _touch(self, x: int, y: int) -> None:
        self._touch_rect(x, y, 1, 1)

    def _touch_rect(self, x: int, y: int, width: int, height: int) -> None:
        left, top = max(x, 0), max(y, 0)
        right, bottom = min(x + width, self.width), min(y + height, self.height)
        if right <= left or bottom <= top:
            return
        if self.dirty is None:
            self.dirty = (left, top, right, bottom)
            return
        x0, y0, x1, y1 = self.dirty
        self.dirty = (min(x0, left), min(y0, top), max(x1, right), max(y1, bottom))

    # -- comparing ---------------------------------------------------------

    def digest(self) -> str:
        """A hash of every pixel: the golden value a test pins down."""
        return hashlib.sha256(bytes(self.pixels)).hexdigest()

    def count(self, color: int) -> int:
        """How many pixels have this exact value."""
        low, high = color & 0xFF, (color >> 8) & 0xFF
        return sum(
            1
            for offset in range(0, len(self.pixels), 2)
            if self.pixels[offset] == low and self.pixels[offset + 1] == high
        )

    def histogram(self) -> Dict[int, int]:
        """Pixel value -> count, for a quick look at what was drawn."""
        counts: Dict[int, int] = {}
        for offset in range(0, len(self.pixels), 2):
            value = self.pixels[offset] | (self.pixels[offset + 1] << 8)
            counts[value] = counts.get(value, 0) + 1
        return counts

    def bounding_box(self, color: int) -> Optional[Rect]:
        """The smallest box containing every pixel of ``color``."""
        box = None
        for y in range(self.height):
            for x in range(self.width):
                if self.pixel(x, y) == color:
                    x0, y0, x1, y1 = box if box else (x, y, x + 1, y + 1)
                    box = (min(x0, x), min(y0, y), max(x1, x + 1), max(y1, y + 1))
        return box

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"<DisplaySurface {self.width}x{self.height} writes={self.writes}>"

    # -- output ------------------------------------------------------------

    def to_png(self, path: str) -> str:
        """Write the image as a truecolour PNG (no image library needed)."""
        raw = bytearray()
        for y in range(self.height):
            raw.append(0)  # filter: none
            for x in range(self.width):
                red, green, blue = _expand565(self.pixel(x, y))
                raw += bytes((red, green, blue))

        def chunk(tag: bytes, data: bytes) -> bytes:
            return (
                struct.pack(">I", len(data))
                + tag
                + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
            )

        header = struct.pack(">IIBBBBB", self.width, self.height, 8, 2, 0, 0, 0)
        png = (
            b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", header)
            + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
            + chunk(b"IEND", b"")
        )
        with open(path, "wb") as handle:
            handle.write(png)
        return path

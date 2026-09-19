# -*- coding: utf-8 -*-

"""The ILI9341 display controller: command set, address counter and GRAM.

This is the part of a TFT panel that firmware actually talks to, with no transport in
it: bytes go in, a pin already told us whether each one is a command or a parameter,
and the pixels land in a :class:`DisplaySurface`.  The SPI wiring, the D/C pin and the
viewer belong to the device layer, which is what makes it possible to drive this model
from a test by poking bytes in, or from a Nucleo instead of a Due.

Modelled: the initialisation commands (accepted), ``CASET``/``PASET``/``RAMWR`` window
addressing with wrap-around, ``MADCTL`` rotation (the address counter *and* the image
the user sees), ``COLMOD`` (16 bpp), sleep/display/inversion state, the read commands a
library uses for identification and pixel read-back, and the ``0x2C``/``0x3C``
continue variants.  Not modelled: gamma, frame rate, power levels, tearing and the
partial/scroll modes -- they are stored or ignored, and none of them change a pixel.
"""

from enum import IntEnum
from typing import Dict, List, Optional, Tuple

from ...config import logger
from .surface import DisplaySurface

__all__ = ["Ili9341", "Ili9341Command", "MADCTL"]


class Ili9341Command(IntEnum):
    NOP = 0x00
    SWRESET = 0x01
    RDDID = 0x04
    RDDST = 0x09
    RDDPM = 0x0A
    RDDMADCTL = 0x0B
    RDDCOLMOD = 0x0C
    RDDIM = 0x0D
    RDDSM = 0x0E
    RDDSDR = 0x0F
    SLPIN = 0x10
    SLPOUT = 0x11
    PTLON = 0x12
    NORON = 0x13
    INVOFF = 0x20
    INVON = 0x21
    GAMMASET = 0x26
    DISPOFF = 0x28
    DISPON = 0x29
    CASET = 0x2A
    PASET = 0x2B
    RAMWR = 0x2C
    RAMRD = 0x2E
    PTLAR = 0x30
    VSCRDEF = 0x33
    MADCTL = 0x36
    VSCRSADD = 0x37
    IDMOFF = 0x38
    IDMON = 0x39
    COLMOD = 0x3A
    RAMWRC = 0x3C
    RAMRDC = 0x3E
    SET_INDEX = 0xD9


class MADCTL(IntEnum):
    MH = 2  # horizontal refresh order
    BGR = 3  # colour filter order
    ML = 4  # vertical refresh order
    MV = 5  # exchange row and column
    MX = 6  # column address order
    MY = 7  # row address order


# How many parameter bytes each command takes.  A command that is not here is either
# handled per byte (RAMWR and friends) or ignored -- an unknown command swallows its
# parameters, just as an unknown command on the real chip does nothing.
PARAM_COUNTS: Dict[int, int] = {
    Ili9341Command.RDDID: 0,
    Ili9341Command.RDDST: 0,
    Ili9341Command.RDDPM: 0,
    Ili9341Command.RDDMADCTL: 0,
    Ili9341Command.RDDCOLMOD: 0,
    Ili9341Command.RDDIM: 0,
    Ili9341Command.RDDSM: 0,
    Ili9341Command.RDDSDR: 0,
    Ili9341Command.GAMMASET: 1,
    Ili9341Command.CASET: 4,
    Ili9341Command.PASET: 4,
    Ili9341Command.PTLAR: 4,
    Ili9341Command.VSCRDEF: 6,
    Ili9341Command.MADCTL: 1,
    Ili9341Command.VSCRSADD: 2,
    Ili9341Command.COLMOD: 1,
    Ili9341Command.SET_INDEX: 1,
}

# Commands that act on their own, with no parameter byte at all.
IMMEDIATE = frozenset(
    {
        Ili9341Command.NOP,
        Ili9341Command.SWRESET,
        Ili9341Command.SLPIN,
        Ili9341Command.SLPOUT,
        Ili9341Command.PTLON,
        Ili9341Command.NORON,
        Ili9341Command.INVOFF,
        Ili9341Command.INVON,
        Ili9341Command.DISPOFF,
        Ili9341Command.DISPON,
        Ili9341Command.IDMOFF,
        Ili9341Command.IDMON,
    }
)

READ_COMMANDS = frozenset(
    {
        Ili9341Command.RDDID,
        Ili9341Command.RDDST,
        Ili9341Command.RDDPM,
        Ili9341Command.RDDMADCTL,
        Ili9341Command.RDDCOLMOD,
        Ili9341Command.RDDIM,
        Ili9341Command.RDDSM,
        Ili9341Command.RDDSDR,
    }
)

WRITE_DATA_COMMANDS = frozenset({Ili9341Command.RAMWR, Ili9341Command.RAMWRC})
READ_DATA_COMMANDS = frozenset({Ili9341Command.RAMRD, Ili9341Command.RAMRDC})

# What the identification registers answer; an ILI9341 reports 0x93, 0x41.
ID = (0x00, 0x93, 0x41)


class Ili9341:
    """One panel's worth of controller state, driven byte by byte."""

    GRAM_WIDTH = 240
    GRAM_HEIGHT = 320

    def __init__(self, width: int = GRAM_WIDTH, height: int = GRAM_HEIGHT) -> None:
        if (width, height) != (self.GRAM_WIDTH, self.GRAM_HEIGHT):
            raise ValueError(f"an ILI9341 is {self.GRAM_WIDTH}x{self.GRAM_HEIGHT}, not {width}x{height}")
        self.gram = DisplaySurface(self.GRAM_WIDTH, self.GRAM_HEIGHT)
        """The panel memory, always in its own 240x320 orientation."""

        self.command: Optional[int] = None
        """The command the parameters and data currently belong to."""
        self.params: List[int] = []
        self.rx = bytearray()
        """Bytes the controller wants to send back (read commands)."""
        self.commands: List[int] = []
        """Every command byte seen, in order -- a firmware's fingerprint."""
        self.bytes_in = 0

        self.expected: Optional[int] = None
        self._pixel_high: Optional[int] = None
        self._snapshot: Optional[DisplaySurface] = None
        self._snapshot_key: Optional[Tuple[int, int]] = None
        self._window: Tuple[int, int, int, int] = (0, self.GRAM_WIDTH - 1, 0, self.GRAM_HEIGHT - 1)
        self._x = 0
        self._y = 0
        self.reset()

    # -- state -------------------------------------------------------------

    def reset(self) -> None:
        """Hardware reset: the command and data latches, not the panel memory."""
        self.madctl = 0
        self.colmod = 0x00
        self.read_index: Optional[int] = None
        """The ``0xD9`` read index the driver selected, if any (see ``_read_register``)."""
        self.sleeping = True
        self.display_on = False
        self.inverted = False
        self.partial = False
        self.idle = False
        self.scroll_start = 0
        self.command = None
        self.params.clear()
        self.rx.clear()
        self.expected = None
        self._pixel_high = None
        self._snapshot = None
        self._snapshot_key = None
        self._window = (0, self.GRAM_WIDTH - 1, 0, self.GRAM_HEIGHT - 1)
        self._x, self._y = 0, 0

    @property
    def size(self) -> Tuple[int, int]:
        """The size the *firmware* sees, which MADCTL swaps in landscape."""
        if self.madctl & (1 << MADCTL.MV):
            return self.GRAM_HEIGHT, self.GRAM_WIDTH
        return self.GRAM_WIDTH, self.GRAM_HEIGHT

    @property
    def rotation(self) -> int:
        """Which of the four orientations MADCTL selects, as a library would number it."""
        exchange = bool(self.madctl & (1 << MADCTL.MV))
        flip_x = bool(self.madctl & (1 << MADCTL.MX))
        flip_y = bool(self.madctl & (1 << MADCTL.MY))
        if not exchange:
            return 2 if flip_y else 0
        return 3 if (flip_x and flip_y) else 1

    @property
    def colour_bits(self) -> int:
        """Pixel depth from COLMOD: 0x55 is the 16 bpp everything uses."""
        return {0x55: 16, 0x65: 18, 0x66: 18}.get(self.colmod, 16)

    # -- the byte stream ---------------------------------------------------

    def write_command(self, value: int) -> None:
        """A byte with D/C low."""
        self.bytes_in += 1
        value &= 0xFF
        self.command = value
        self.params.clear()
        self._pixel_high = None
        self.commands.append(value)

        if value in WRITE_DATA_COMMANDS:
            self.expected = None
            if value == Ili9341Command.RAMWR:
                # A new memory write starts at the top left of the window; the
                # "continue" variant carries on from wherever the counter is.
                self._x, self._y = self._window[0], self._window[2]
            return
        if value in READ_DATA_COMMANDS:
            self.expected = None
            self._read_pixel()
            return
        if value in READ_COMMANDS:
            self.expected = 0
            self._read_register(value)
            return
        self.expected = PARAM_COUNTS.get(value)
        if value in IMMEDIATE:
            self._apply(value, ())
            self.expected = None

    def write_data(self, value: int) -> None:
        """A byte with D/C high: a parameter, or pixel data."""
        self.bytes_in += 1
        value &= 0xFF
        command = self.command
        if command is None:
            return
        if command in WRITE_DATA_COMMANDS:
            self._pixel(value)
            return
        if command in READ_DATA_COMMANDS:
            return
        if self.expected is None:
            # An unknown command: its parameters go nowhere, as on the chip.
            return
        self.params.append(value)
        if len(self.params) >= self.expected:
            self._apply(command, tuple(self.params))
            self.params.clear()
            if command not in WRITE_DATA_COMMANDS:
                # A one-shot command does not accumulate a second round of parameters.
                self.command = None
                self.expected = None

    def write(self, data: bytes, command: bool = False) -> None:
        """Feed a run of bytes; ``command`` says which pin level they arrived with."""
        handler = self.write_command if command else self.write_data
        for value in data:
            handler(value)

    def take_response(self) -> bytes:
        """Bytes the controller has queued for the host, oldest first."""
        answer = bytes(self.rx)
        self.rx.clear()
        return answer

    @property
    def in_waiting(self) -> int:
        return len(self.rx)

    # -- pixels ------------------------------------------------------------

    def gram_coords(self, x: int, y: int) -> Tuple[int, int]:
        """Map a coordinate the firmware used onto the panel's own memory."""
        if self.madctl & (1 << MADCTL.MV):
            column, row = y, x
        else:
            column, row = x, y
        if self.madctl & (1 << MADCTL.MX):
            column = self.GRAM_WIDTH - 1 - column
        if self.madctl & (1 << MADCTL.MY):
            row = self.GRAM_HEIGHT - 1 - row
        return column, row

    def pixel(self, x: int, y: int) -> int:
        """One pixel, in the orientation the firmware is drawing in."""
        column, row = self.gram_coords(x, y)
        return self.gram.pixel(column, row)

    def set_pixel(self, x: int, y: int, color: int) -> None:
        column, row = self.gram_coords(x, y)
        self.gram.set_pixel(column, row, color)

    def snapshot(self) -> DisplaySurface:
        """The image as the user sees it, in the firmware's orientation.

        Built by walking the panel memory, so it is cached until something that changes
        the image does: a pixel write, or the rotation, which re-reads the same memory
        through the other orientation.  A test that looks at the frame buffer several
        times pays for it once.
        """
        key = (self.gram.writes, self.madctl)
        if self._snapshot is not None and self._snapshot_key == key:
            return self._snapshot
        width, height = self.size
        image = DisplaySurface(width, height)
        for y in range(height):
            for x in range(width):
                column, row = self.gram_coords(x, y)
                image.set_pixel(x, y, self.gram.pixel(column, row))
        self._snapshot = image
        self._snapshot_key = key
        return image

    def _pixel(self, value: int) -> None:
        if self._pixel_high is None:
            self._pixel_high = value
            return
        color = (self._pixel_high << 8) | value
        self._pixel_high = None
        self.set_pixel(self._x, self._y, color)
        self._advance()

    def _advance(self) -> None:
        """Step the address counter, wrapping inside the window like the chip does."""
        column_start, column_end, page_start, page_end = self._window
        self._x += 1
        if self._x > column_end:
            self._x = column_start
            self._y += 1
            if self._y > page_end:
                self._y = page_start

    def _read_pixel(self) -> None:
        """RAMRD: a dummy byte, then the pixel at the address counter."""
        color = self.pixel(self._x, self._y)
        self.rx += bytes([0x00, (color >> 8) & 0xFF, color & 0xFF])
        self._advance()

    def _read_register(self, command: int) -> None:
        """Answer a read command.

        A plain read returns a dummy byte first, which is what the panel does and what
        a driver discards.  ``Adafruit_ILI9341::readcommand8()`` first writes the index
        register (``0xD9``) and then reads the register in one go, so in that mode the
        value comes back immediately -- which is why the diagnostics it prints are real
        register values and not a wall of zeroes.
        """
        dummy = b"" if self.read_index is not None else b"\x00"
        if command == Ili9341Command.RDDID:
            self.rx += bytes(ID)
        elif command == Ili9341Command.RDDMADCTL:
            self.rx += dummy + bytes([self.madctl])
        elif command == Ili9341Command.RDDCOLMOD:
            self.rx += dummy + bytes([self.colmod])
        elif command == Ili9341Command.RDDPM:
            # Booster on, display on, normal mode -- what a working panel reports.
            self.rx += dummy + bytes([0x9C if self.display_on else 0x08])
        elif command == Ili9341Command.RDDSM:
            state = (self.sleeping and 0x01 or 0x00) | (self.display_on and 0x04 or 0x00)
            self.rx += dummy + bytes([state])
        else:
            # RDIMGFMT, RDDSDR and the rest: no fault bits set.
            self.rx += dummy + bytes([0x00])

    # -- commands with parameters -----------------------------------------

    def _apply(self, command: int, params: Tuple[int, ...]) -> None:
        if command == Ili9341Command.CASET:
            start = (params[0] << 8) | params[1]
            end = (params[2] << 8) | params[3]
            if end < start:
                start, end = end, start
            self._window = (start, end, self._window[2], self._window[3])
        elif command == Ili9341Command.PASET:
            start = (params[0] << 8) | params[1]
            end = (params[2] << 8) | params[3]
            if end < start:
                start, end = end, start
            self._window = (self._window[0], self._window[1], start, end)
        elif command == Ili9341Command.MADCTL:
            self.madctl = params[0]
        elif command == Ili9341Command.COLMOD:
            self.colmod = params[0]
            if self.colour_bits != 16:
                logger.warning(f"[ILI9341 ]: COLMOD 0x{params[0]:02x} is not 16 bpp; pixels are still 16 bpp")
        elif command == Ili9341Command.SET_INDEX:
            self.read_index = params[0]
        elif command == Ili9341Command.SWRESET:
            self.reset()
        elif command == Ili9341Command.SLPIN:
            self.sleeping = True
        elif command == Ili9341Command.SLPOUT:
            self.sleeping = False
        elif command == Ili9341Command.DISPON:
            self.display_on = True
        elif command == Ili9341Command.DISPOFF:
            self.display_on = False
        elif command == Ili9341Command.INVON:
            self.inverted = True
        elif command == Ili9341Command.INVOFF:
            self.inverted = False
        elif command == Ili9341Command.PTLON:
            self.partial = True
        elif command == Ili9341Command.NORON:
            self.partial = False
        elif command == Ili9341Command.IDMON:
            self.idle = True
        elif command == Ili9341Command.IDMOFF:
            self.idle = False
        elif command == Ili9341Command.VSCRSADD:
            self.scroll_start = (params[0] << 8) | params[1]
        elif command in (
            Ili9341Command.PTLAR,
            Ili9341Command.VSCRDEF,
            Ili9341Command.GAMMASET,
        ):
            pass  # stored by the caller if it matters; none of these move a pixel
        elif command in (Ili9341Command.NOP,) or command in READ_COMMANDS:
            pass
        else:
            logger.debug(f"[ILI9341 ]: ignoring command 0x{command:02x}")

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        width, height = self.size
        return (
            f"<Ili9341 {width}x{height} rotation={self.rotation} madctl=0x{self.madctl:02x} "
            f"display={'on' if self.display_on else 'off'} commands={len(self.commands)}>"
        )

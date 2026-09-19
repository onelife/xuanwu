# -*- coding: utf-8 -*-

"""Displays: the panel-side behaviour, independent of how bytes reach it.

``DisplaySurface`` is a plain RGB565 framebuffer; ``Ili9341`` is the controller that
turns a command/data byte stream into pixels on one.  Neither knows about SPI, GPIO or
a viewer, so the same code runs under any vendor's controller and under a test that
pokes bytes in directly.
"""

from .ili9341 import Ili9341, Ili9341Command
from .surface import DisplaySurface, rgb565

__all__ = ["DisplaySurface", "rgb565", "Ili9341", "Ili9341Command"]

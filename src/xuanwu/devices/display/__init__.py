# -*- coding: utf-8 -*-

"""Display devices: the panel model plus how it is wired and looked at.

The controller itself lives in :mod:`xuanwu.peripherals.display`, because it is not a
device on a bus but a chip a firmware drives; here it gets an SPI port, a data/command
pin and a viewer.
"""

from .ili9341 import Ili9341Device
from .viewers import Viewer, create_viewer

__all__ = ["Ili9341Device", "Viewer", "create_viewer"]

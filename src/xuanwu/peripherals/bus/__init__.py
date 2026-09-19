# -*- coding: utf-8 -*-

"""The buses a peripheral model can be attached to.

``SpiBus``/``I2cBus`` are what the *device* sees; ``SpiController``/``I2cController``
are what the *register adapter* drives.  Keeping them apart is what lets one chip
model (an ILI9341, a serial flash, an FT6206) run on any vendor's controller, and
lets one controller serve any device.
"""

from .i2c import I2cBus, I2cController, I2cDevice, RegisterDevice
from .spi import NullSpiBus, SpiBus, SpiBusSelector, SpiController

__all__ = [
    "SpiBus",
    "SpiBusSelector",
    "SpiController",
    "NullSpiBus",
    "I2cBus",
    "I2cController",
    "I2cDevice",
    "RegisterDevice",
]

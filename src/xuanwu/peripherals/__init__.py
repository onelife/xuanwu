# -*- coding: utf-8 -*-

"""Vendor-neutral behaviour cores.

A peripheral model in ``arch/vendor/`` is a register table plus the mapping from
register bits onto one of these cores.  The core owns the *behaviour* -- how a GPIO
port tracks levels and tells devices about changes, how an SPI master moves a byte,
how an I2C master runs a transaction -- while the adapter owns the *register
layout*.  Supporting another vendor is then a matter of writing an adapter rather
than reimplementing the protocol.

Everything here is free of chip specifics: no base addresses, no register names, no
vendor IRQ numbers.
"""

from .bus.i2c import I2cBus, I2cController, I2cDevice, RegisterDevice
from .bus.spi import SpiBus, SpiController
from .gpio import GpioPort

__all__ = [
    "GpioPort",
    "SpiBus",
    "SpiController",
    "I2cBus",
    "I2cController",
    "I2cDevice",
    "RegisterDevice",
]

# -*- coding: utf-8 -*-

"""External device framework.

A *device* is something wired to the simulated MCU: an LED on a pin, a flash
chip on an SPI bus, a sensor on an I2C bus.  Devices are declared in the chip
description (``devices:``) and attached once the peripherals exist, so they can
hook GPIO pins and take over a peripheral's byte stream.
"""

from abc import ABC, abstractmethod
from typing import Any

from ..exception import XwUnknownHardware

__all__ = ["Device", "DeviceContext"]


class DeviceContext:
    """What a device is handed when it attaches."""

    def __init__(self, box: Any, reg: Any, mem: Any, hw: Any) -> None:
        self.box = box
        self.reg = reg
        self.mem = mem
        self.hw = hw

    def peripheral(self, name: str) -> Any:
        """Return a core peripheral model by its chip-description name."""
        key = name.lower()
        if key not in self.hw.perif:
            known = ", ".join(sorted(self.hw.perif)) or "none"
            raise XwUnknownHardware(f"No peripheral {name!r} on this chip (have: {known})")
        return self.hw.perif[key]


class Device(ABC):
    """Base class for everything attached to the simulated MCU."""

    type = "device"

    def __init__(self, name: str, **options: Any) -> None:
        self.name = name
        self.options = options
        self.attached = False

    @abstractmethod
    def attach(self, ctx: DeviceContext) -> None:
        """Wire the device to the peripherals it needs."""

    def detach(self) -> None:
        """Undo :meth:`attach`; safe to call when not attached."""
        self.attached = False

    def reset(self) -> None:
        """Return the device to its power-on state."""

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"<{type(self).__name__} {self.name!r}>"

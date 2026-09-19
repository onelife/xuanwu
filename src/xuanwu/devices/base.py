# -*- coding: utf-8 -*-

"""External device framework.

A *device* is something wired to the simulated MCU: an LED on a pin, a flash
chip on an SPI bus, a sensor on an I2C bus.  Devices are declared in the chip
description (``devices:``) and attached once the peripherals exist, so they can
hook GPIO pins and take over a peripheral's byte stream.
"""

from abc import ABC, abstractmethod
from typing import Any, Dict

from ..exception import XwUnknownHardware
from ..peripherals.bus.spi import SpiBusSelector

__all__ = ["Device", "DeviceContext"]


class DeviceContext:
    """What a device is handed when it attaches."""

    def __init__(self, box: Any, reg: Any, mem: Any, hw: Any) -> None:
        self.box = box
        self.reg = reg
        self.mem = mem
        self.hw = hw
        self._spi_buses: Dict[str, SpiBusSelector] = {}

    def peripheral(self, name: str) -> Any:
        """Return a core peripheral model by its chip-description name."""
        key = name.lower()
        if key not in self.hw.perif:
            known = ", ".join(sorted(self.hw.perif)) or "none"
            raise XwUnknownHardware(f"No peripheral {name!r} on this chip (have: {known})")
        return self.hw.perif[key]

    def gpio(self, name: str) -> Any:
        """The pin-level view of a GPIO port.

        A device cares about pins, not about the registers an adapter keeps them in, so
        this hands back the behaviour core (:class:`xuanwu.peripherals.gpio.GpioPort`)
        when the vendor adapter has one.  An adapter that is still register-only is
        returned as it is, which is what a device would have got anyway.
        """
        peripheral = self.peripheral(name)
        return getattr(peripheral, "port", peripheral)

    def spi_bus(self, port: str) -> SpiBusSelector:
        """The shared bus on an SPI port, created on first use.

        Devices that share a bus -- a display and an SD socket on one controller, each
        with its own chip select -- register with this instead of taking the
        peripheral's byte stream for themselves.  The bus the controller was using
        before (a host bridge, say) is kept as the fallback for unselected traffic.
        """
        peripheral = self.peripheral(port)
        existing = self._spi_buses.get(port.lower())
        if existing is not None:
            return existing
        bridge = peripheral.bridge
        if isinstance(bridge, SpiBusSelector):
            self._spi_buses[port.lower()] = bridge
            return bridge
        selector = SpiBusSelector(fallback=bridge)
        peripheral.bridge = selector  # the adapter hands the old bus to the selector
        self._spi_buses[port.lower()] = selector
        return selector


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

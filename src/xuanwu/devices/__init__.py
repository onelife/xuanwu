# -*- coding: utf-8 -*-

"""External devices, one module each, plus the registry that builds them."""

from typing import Any, Dict, List

from ..exception import XwInvalidParameter, XwUnknownHardware
from .base import Device, DeviceContext
from .display import Ili9341Device
from .led import Led
from .spi_flash import SpiFlash, SpiFlashCommand

__all__ = [
    "Device",
    "DeviceContext",
    "Ili9341Device",
    "Led",
    "SpiFlash",
    "SpiFlashCommand",
    "BUILDIN",
    "create_device",
]


BUILDIN: Dict[str, Any] = {
    "led": Led,
    "spi_flash": SpiFlash,
    "ili9341": Ili9341Device,
}
"""Device models keyed by the ``type`` used in the chip description."""


def create_device(spec: Dict[str, Any]) -> Device:
    """Build one device from its chip-description entry."""
    spec = dict(spec)
    kind = spec.pop("type", None)
    name = spec.pop("name", None)
    if kind is None:
        raise XwInvalidParameter(f"Every device needs a 'type'; got {spec!r}")
    if kind not in BUILDIN:
        raise XwUnknownHardware(f"Unknown device type {kind!r}; available: {', '.join(sorted(BUILDIN))}")
    return BUILDIN[kind](name or kind, **spec)


def device_types() -> List[str]:
    """Names of the device models that can be declared in a chip description."""
    return sorted(BUILDIN)

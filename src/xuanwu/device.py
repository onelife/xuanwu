# -*- coding: utf-8 -*-

"""External device layer.

Builds the devices a chip description declares and attaches them to the
peripherals once those exist.  See :mod:`xuanwu.devices` for the device models
themselves and :class:`~xuanwu.devices.base.DeviceContext` for what a device is
handed when it attaches.
"""

from typing import Any, Dict, Iterator, List

from .config import logger
from .devices import Device, DeviceContext, create_device

__all__ = ["DeviceController"]


class DeviceController(object):
    """Owns the external devices of one simulation."""

    def __init__(self) -> None:
        self.devices: List[Device] = []
        self._by_name: Dict[str, Device] = {}

    def load(self, chip: Dict[str, Any], box: Any, reg: Any, mem: Any, hw: Any) -> None:
        """Instantiate and attach everything in the chip's ``devices:`` list."""
        specs = list(chip.get("devices") or [])
        if "device" in chip:
            logger.warning(
                "The chip description uses the obsolete singular 'device:' key; "
                "it is ignored. Use a 'devices:' list with a 'type' per entry."
            )
        if not specs:
            return

        context = DeviceContext(box, reg, mem, hw)
        for spec in specs:
            device = create_device(spec)
            device.attach(context)
            self.devices.append(device)
            self._by_name[device.name] = device
        logger.debug(f"Attached devices: {', '.join(self._by_name) or 'none'}")

    # -- collection access ----------------------------------------------

    def __getitem__(self, name: str) -> Device:
        return self._by_name[name]

    def __contains__(self, name: object) -> bool:
        return name in self._by_name

    def __iter__(self) -> Iterator[Device]:
        return iter(self.devices)

    def __len__(self) -> int:
        return len(self.devices)

    def names(self) -> List[str]:
        """Names of the attached devices."""
        return list(self._by_name)

    # -- lifecycle -------------------------------------------------------

    def reset(self) -> None:
        for device in self.devices:
            device.reset()

    def detach(self) -> None:
        """Detach and forget every device."""
        for device in self.devices:
            device.detach()
        self.devices.clear()
        self._by_name.clear()

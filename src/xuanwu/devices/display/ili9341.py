# -*- coding: utf-8 -*-

"""The Adafruit 2.8" TFT shield's panel, wired to a Due.

The shield is an ILI9341 on the SPI header with the data/command pin on D9 and chip
select on D10, which is exactly what ``Adafruit_ILI9341`` is written for.  This class
is the wiring: it takes bytes off a shared SPI bus, looks at the D/C pin to decide
whether each one is a command, and keeps a viewer up to date.  The controller itself is
:class:`xuanwu.peripherals.display.Ili9341` and knows none of this.

The SPI port is *shared* -- the same shield also carries a microSD socket on D4 -- so
the panel goes through the bus selector instead of taking the controller over.
"""

from typing import Any, Dict, List, Optional, Tuple

from ...config import logger
from ...exception import XwInvalidParameter
from ...peripherals.bus.spi import SpiBusSelector
from ...peripherals.display import DisplaySurface, Ili9341
from ..base import Device, DeviceContext
from .viewers import Viewer, create_viewer

__all__ = ["Ili9341Device"]


class Ili9341Device(Device):
    """An ILI9341 panel on a shared SPI bus, with a D/C pin."""

    type = "ili9341"

    def __init__(
        self,
        name: str,
        port: str = "SPI",
        cs: Optional[Dict[str, Any]] = None,
        dc: Optional[Dict[str, Any]] = None,
        reset: Optional[Dict[str, Any]] = None,
        viewer: str = "headless",
        scale: int = 1,
        dump: Optional[str] = None,
        **options: Any,
    ) -> None:
        super().__init__(name, **options)
        if not dc:
            raise XwInvalidParameter(f"{name}: an ILI9341 needs a 'dc' pin to tell commands from data")
        self.port = port
        self.cs = dict(cs or {})
        self.dc = dict(dc)
        self.reset_pin = dict(reset or {})
        self.viewer_kind = viewer
        self.scale = int(scale)
        self.dump = dump

        self.core = Ili9341()
        self.viewer: Optional[Viewer] = None
        self._bus: Optional[SpiBusSelector] = None
        self._dc_port = None
        self._dc_pin: Optional[int] = None
        self._dc_high = True
        self._rx = bytearray()
        self._selected = False
        self.bytes_received = 0

    # -- wiring ------------------------------------------------------------

    def attach(self, ctx: DeviceContext) -> None:
        port = ctx.gpio(self.dc["port"])
        self._dc_port = port
        self._dc_pin = int(self.dc["pin"])
        self._dc_high = bool(port.level(self._dc_pin))
        port.add_edge_hook(self._dc_pin, self._on_dc)

        if self.reset_pin and "port" in self.reset_pin:
            # A reset line the firmware can pull: the panel comes back blank.
            ctx.gpio(self.reset_pin["port"]).add_edge_hook(
                int(self.reset_pin["pin"]), self._on_reset
            )

        bus = ctx.spi_bus(self.port)
        self._bus = bus
        pin = int(self.cs["pin"]) if self.cs else None
        gpio = ctx.gpio(self.cs["port"]) if self.cs else None
        bus.add(self.name, self, gpio=gpio, pin=pin, active_low=bool(self.cs.get("active_low", True)))

        if self.viewer_kind != "none":
            self.viewer = create_viewer(self.viewer_kind, title=f"xuanwu:{self.name}", scale=self.scale, dump=self.dump)

        self.attached = True
        where = f", CS {self.cs['port']}.{pin}" if pin is not None else ""
        logger.info(f"[{self.name:8s}]: ILI9341 on {self.port}, DC {self.dc['port']}.{self._dc_pin}{where}")

    def detach(self) -> None:
        if self._dc_port is not None and self._dc_pin is not None:
            self._dc_port.remove_edge_hook(self._dc_pin, self._on_dc)
        if self._bus is not None:
            self._bus.remove(self.name)
        if self.viewer is not None:
            self.viewer.close()
        self._bus = None
        self._dc_port = None
        super().detach()

    def reset(self) -> None:
        self.core.reset()
        self._rx.clear()
        self._selected = False
        self.bytes_received = 0

    # -- pins --------------------------------------------------------------

    def _on_dc(self, high: bool) -> None:
        self._dc_high = bool(high)

    def _on_reset(self, high: bool) -> None:
        if not high:
            self.core.reset()

    def select(self, active: bool) -> None:
        """Told by the bus selector; a panel starts a fresh frame on chip select."""
        self._selected = bool(active)

    # -- SpiBus: what the controller sees ----------------------------------

    @property
    def peer_hint(self) -> str:
        return f"device://{self.name}"

    @property
    def in_waiting(self) -> int:
        return len(self._rx)

    def read(self, size: int = 1) -> bytes:
        answer = bytes(self._rx[:size])
        del self._rx[:size]
        return answer

    def write(self, data: bytes) -> int:
        for value in data:
            if self._dc_high:
                self.core.write_data(value)
            else:
                self.core.write_command(value)
            answer = self.core.take_response()
            if answer:
                self._rx.extend(answer)
        self.bytes_received += len(data)
        return len(data)

    def close(self) -> None:
        if self.viewer is not None:
            self.viewer.close()

    # -- looking at it -----------------------------------------------------

    @property
    def surface(self) -> DisplaySurface:
        """The image as it is on the panel, in the firmware's orientation."""
        return self.core.snapshot()

    @property
    def size(self) -> Tuple[int, int]:
        return self.core.size

    def pixel(self, x: int, y: int) -> int:
        return self.core.pixel(x, y)

    def refresh(self, force: bool = False) -> None:
        """Push the current image to the viewer, if there is one."""
        if self.viewer is not None:
            self.viewer.update(self.surface, force=force)

    def save(self, path: str) -> str:
        """Write the current image to a PNG; handy without any GUI at all."""
        return self.surface.to_png(path)

    def command_log(self) -> List[int]:
        """Every command byte the firmware sent, in order."""
        return list(self.core.commands)

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        width, height = self.size
        return f"<Ili9341Device {self.name!r} {width}x{height} bytes={self.bytes_received}>"

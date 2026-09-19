# -*- coding: utf-8 -*-

"""An LED (or any logic probe) on a GPIO pin."""

from typing import Any, Optional, TextIO

from ..config import logger
from .base import Device, DeviceContext

__all__ = ["Led"]


class Led(Device):
    """Watches one GPIO pin and records its level.

    The GPIO model calls the first hook when a pin is driven high and the second
    when it is driven low, so this only needs to translate that into "lit" and
    remember the transitions -- which makes it a useful probe in tests as well
    as a visible output in an example.
    """

    type = "led"

    def __init__(
        self,
        name: str,
        port: str = "GPIOB",
        pin: int = 27,
        active_low: bool = False,
        output: Optional[TextIO] = None,
        **options: Any,
    ) -> None:
        super().__init__(name, **options)
        self.port = port
        self.pin = int(pin)
        self.active_low = bool(active_low)
        self.state: Optional[bool] = None
        """``True`` when lit, ``None`` before the first level change."""
        self.transitions = 0
        self._output = output
        self._gpio = None

    def attach(self, ctx: DeviceContext) -> None:
        self._gpio = ctx.peripheral(self.port)
        self._gpio.add_hook(self.pin, (self._on_high, self._on_low))
        self.attached = True
        logger.info(f"[{self.name:8s}]: LED on {self.port}.{self.pin}")

    def detach(self) -> None:
        if self._gpio is not None:
            self._gpio.remove_hook(self.pin, (self._on_high, self._on_low))
            self._gpio = None
        super().detach()

    def reset(self) -> None:
        self.state = None
        self.transitions = 0

    # the GPIO model calls hooks[0] for a high level and hooks[1] for a low one
    def _on_high(self) -> None:
        self._level(True)

    def _on_low(self) -> None:
        self._level(False)

    def _level(self, high: bool) -> None:
        lit = (not high) if self.active_low else high
        if lit == self.state:
            return
        self.state = lit
        self.transitions += 1
        if self._output is not None:
            self._output.write("ON\n" if lit else "OFF\n")
            self._output.flush()
        logger.debug(f"[{self.name:8s}]: {'on' if lit else 'off'}")

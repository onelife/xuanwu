# -*- coding: utf-8 -*-

"""GPIO port behaviour, without a register layout.

The port keeps four things that firmware and devices both care about:

* **direction** -- is the pin an output;
* **selection** -- does the GPIO peripheral own the pin, or has it been handed to a
  peripheral (UART, SPI, TWI ...);
* **the level the port drives**;
* **the level a device drives** on an input pin, which is what ``digitalRead``
  returns and what an interrupt or a card-detect line is made of.

Devices subscribe to changes rather than polling.  Three granularities are
available because different devices need different things: a single pin with the
classic "high"/"low" callbacks, a single pin with the resulting level as an
argument, and every change of the port with ``(pin, level)`` -- the last one is what
a parallel bus needs to latch eight data lines on a strobe.
"""

from collections import defaultdict
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

from ..config import logger

__all__ = ["GpioPort"]

LevelHook = Callable[[int, bool], None]
EdgeHook = Callable[[bool], None]
PinHook = Tuple[Optional[Callable[[], None]], Optional[Callable[[], None]]]


class GpioPort:
    """One port's worth of pins, and the devices watching them."""

    def __init__(self, name: str = "GPIO", width: int = 32) -> None:
        self.name = name
        self.width = width
        self._mask = (1 << width) - 1
        self.outputs = 0
        """Levels the port drives (the ODSR equivalent)."""
        self.directions = 0
        """Bit set means the pin is an output."""
        self.selection = 0
        """Bit set means the GPIO peripheral owns the pin (the PER equivalent)."""
        self.pullups = 0
        self.peripheral_function = 0
        """Which alternate function each pin is on (the ABSR equivalent)."""
        self.opens = 0
        """Open-drain pins (the OWER equivalent)."""
        self.multi_drive = 0
        self.input_filter = 0
        self.externals: Dict[int, bool] = {}
        """Levels driven by devices on input pins."""
        self.revision = 0
        """Bumped by anything that can change a pin level.

        A device that needs the *level* of a pin rather than its edges -- a chip select
        sampled when a byte arrives, a parallel bus latched on a strobe -- can compare
        this against what it last saw and skip re-reading every pin.  It counts writes,
        not changes: a write that drives a pin to the level it already had still bumps
        it, because the device cannot tell the difference from the outside.
        """
        self.written = 0
        """Pins whose output register has been written by the guest at least once.

        Without this, a pin nobody ever touched is indistinguishable from one the guest
        drove low, and a device watching the level (a chip select) would answer on a bus
        it was never selected on.
        """
        self._pin_hooks: Dict[int, Set[PinHook]] = defaultdict(set)
        self._edge_hooks: Dict[int, Set[EdgeHook]] = defaultdict(set)
        self._port_hooks: Set[LevelHook] = set()

    # -- state the guest can see -----------------------------------------

    def is_output(self, pin: int) -> bool:
        return bool(self.directions >> pin & 1)

    def is_gpio(self, pin: int) -> bool:
        return bool(self.selection >> pin & 1)

    def drives(self, pin: int) -> bool:
        """True when the port itself decides the level on this pin."""
        return self.is_output(pin) and self.is_gpio(pin)

    def level(self, pin: int) -> bool:
        """The level on the pin, whoever is driving it."""
        if self.drives(pin):
            return bool(self.outputs >> pin & 1)
        if pin in self.externals:
            return bool(self.externals[pin])
        return bool(self.pullups >> pin & 1)

    def levels(self) -> int:
        """Every pin's level at once (the PDSR equivalent)."""
        values = self.pullups & ~self.directions & self._mask
        values = (values & ~self.selection) | (self.outputs & self.selection & self.directions)
        driven = self.externals
        for pin, high in driven.items():
            if not self.drives(pin):
                values = (values | (1 << pin)) if high else (values & ~(1 << pin))
        return values & self._mask

    # -- state the adapter sets ------------------------------------------

    def set_outputs(self, values: int) -> None:
        """Replace the driven levels (a write to ODSR)."""
        values &= self._mask
        # A write to the whole register decides every pin, including the ones it drives
        # low -- which is what makes them different from pins nobody touched.
        self.written |= self._mask
        self._apply(self.outputs ^ values, values)

    def set_output_bits(self, mask: int) -> None:
        """Drive the masked pins high, leaving the rest (a write to SODR)."""
        mask &= self._mask
        self.written |= mask
        values = (self.outputs | mask) & self._mask
        self._apply(self.outputs ^ values, values)

    def clear_output_bits(self, mask: int) -> None:
        """Drive the masked pins low, leaving the rest (a write to CODR)."""
        mask &= self._mask
        self.written |= mask
        values = (self.outputs & ~mask) & self._mask
        self._apply(self.outputs ^ values, values)

    def set_directions(self, mask: int, output: bool) -> None:
        if output:
            self.directions |= mask & self._mask
        else:
            self.directions &= ~mask & self._mask
        self.revision += 1

    def set_selection(self, mask: int, gpio: bool) -> None:
        if gpio:
            self.selection |= mask & self._mask
        else:
            self.selection &= ~mask & self._mask
        self.revision += 1

    def set_pullups(self, mask: int, enabled: bool) -> None:
        if enabled:
            self.pullups |= mask & self._mask
        else:
            self.pullups &= ~mask & self._mask
        self.revision += 1

    def reset(self) -> None:
        self.outputs = 0
        self.directions = 0
        self.selection = 0
        self.pullups = 0
        self.peripheral_function = 0
        self.opens = 0
        self.multi_drive = 0
        self.input_filter = 0
        self.externals.clear()
        self.written = 0
        self.revision += 1

    # -- state a device sets ---------------------------------------------

    def drive_input(self, pin: int, high: bool) -> None:
        """A device pulls an input pin to a level."""
        changed = self.externals.get(pin) != high
        self.externals[pin] = bool(high)
        self.revision += 1
        if changed and not self.drives(pin):
            self._notify(1 << pin)

    def release_input(self, pin: int) -> None:
        """A device stops driving a pin, which floats to its pull."""
        if pin in self.externals:
            del self.externals[pin]
            self.revision += 1
            if not self.drives(pin):
                self._notify(1 << pin)

    # -- hooks ------------------------------------------------------------

    def add_hook(self, pin: int, fn: Tuple[Any, Any]) -> None:
        """Watch one pin with a ``(on_high, on_low)`` pair."""
        self._pin_hooks[pin].add((fn[0], fn[1]))

    def remove_hook(self, pin: int, fn: Tuple[Any, Any]) -> None:
        self._pin_hooks[pin].discard((fn[0], fn[1]))

    def add_edge_hook(self, pin: int, fn: EdgeHook) -> None:
        """Watch one pin, called with the new level: ``fn(high)``."""
        self._edge_hooks[pin].add(fn)

    def remove_edge_hook(self, pin: int, fn: EdgeHook) -> None:
        self._edge_hooks[pin].discard(fn)

    def add_port_hook(self, fn: LevelHook) -> None:
        """Watch the whole port, called as ``fn(pin, high)`` per change."""
        self._port_hooks.add(fn)

    def remove_port_hook(self, fn: LevelHook) -> None:
        self._port_hooks.discard(fn)

    def clear_hooks(self) -> None:
        self._pin_hooks.clear()
        self._edge_hooks.clear()
        self._port_hooks.clear()

    def watching(self) -> List[int]:
        """Pins that have at least one hook; useful for diagnostics."""
        return sorted(set(self._pin_hooks) | set(self._edge_hooks))

    # -- internals ---------------------------------------------------------

    def _apply(self, changed: int, values: int) -> None:
        self.revision += 1
        if not changed:
            # The level did not move, but a device watching the *level* (a chip select)
            # may still need to be told, so the revision above is not conditional.
            return
        self.outputs = ((self.outputs & ~changed) | (values & changed)) & self._mask
        self._notify(changed)

    def _notify(self, changed: int) -> None:
        """Tell the hooks about the pins whose level actually changed.

        Every changed bit of the output register is reported, even for a pin that
        has not been configured as a GPIO output.  That is what the models did
        before this core existed, and device-layer code (and tests) rely on being
        able to drive a pin by writing its set/clear register directly, without
        going through the direction registers first.  Direction and selection only
        decide what :meth:`level` reports, i.e. what ``digitalRead`` would see.
        """
        changed &= self._mask
        while changed:
            bit = changed & -changed
            changed ^= bit
            pin = bit.bit_length() - 1
            high = bool(self.outputs & bit)
            for on_high, on_low in self._pin_hooks.get(pin, ()):
                hook = on_high if high else on_low
                if hook is None:
                    continue
                try:
                    hook()
                except Exception as error:  # noqa: BLE001 - a device must not kill the run
                    logger.error(f"[{self.name}]: hook for pin {pin} failed: {error!r}")
            for edge_hook in self._edge_hooks.get(pin, ()):
                edge_hook(high)
            for port_hook in self._port_hooks:
                port_hook(pin, high)

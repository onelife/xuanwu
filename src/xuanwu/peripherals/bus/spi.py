# -*- coding: utf-8 -*-

"""SPI master behaviour, without a register layout.

SPI is full duplex: every clocked byte sends one out and brings one in.  Firmware
therefore writes the transmit register and then reads the receive register, and a
model that only forwarded bytes on write would leave the receive path empty.  The
controller here fetches the answer at the moment the byte is written and keeps it
until the guest reads it, which is both faithful and cheaper -- firmware polls the
status register before *and* after every byte.

The device end is a :class:`SpiBus`: the existing ``SerialBridge`` implementations
and the device models already speak that shape (``write(bytes)``/``read(n)``/
``in_waiting``), so a device can be attached unchanged.
"""

from collections import OrderedDict
from functools import partial
from typing import Any, Dict, Optional, Protocol, Tuple, runtime_checkable

from ...config import logger
from ...exception import XwInvalidParameter

__all__ = ["SpiBus", "NullSpiBus", "SpiController", "SpiBusSelector"]

IDLE_LINE = 0xFF
"""What the data line reads when no device drives it (pulled up, as SPI idle is)."""


@runtime_checkable
class SpiBus(Protocol):
    """What a device on an SPI bus has to provide.

    Structural on purpose: the serial bridges in ``backends`` already have this
    shape (``write(bytes)`` / ``read(n)`` / ``in_waiting``), so a host bridge and an
    on-chip device model are the same thing to the controller, without either of
    them inheriting from the other.
    """

    def write(self, data: bytes) -> int:
        """Accept bytes clocked towards the device."""

    def read(self, size: int = 1) -> bytes:
        """Bytes the device wants to send back, oldest first."""

    @property
    def in_waiting(self) -> int:
        """How many bytes are waiting to be read."""


class NullSpiBus:
    """A bus with nothing on it: nothing is accepted and zeros come back."""

    def write(self, data: bytes) -> int:
        return len(data)

    def read(self, size: int = 1) -> bytes:
        return b"\x00" * size

    @property
    def in_waiting(self) -> int:
        return 0

    @property
    def peer_hint(self) -> str:
        return "spi://none"

    def close(self) -> None:  # pragma: no cover - nothing to close
        pass


class SpiController:
    """Master side of an SPI bus: one byte out, one byte in, per transfer.

    The controller knows nothing about the register that holds the byte or the bit
    that reports readiness; the adapter asks for the state it needs.
    """

    def __init__(self, bus: Optional[SpiBus] = None, channels: int = 4) -> None:
        self.channels = channels
        self.csr = [0] * channels
        """Per-chip-select configuration (mode, divider, bit order ...)."""
        self.bus: Optional[SpiBus] = bus
        self.enabled = False
        self.master = True
        self.config = 0
        """The mode register, kept verbatim for the adapter's benefit."""
        self.selected = 0
        """Chip-select lines currently asserted, as a bitmask."""
        self.last_received = 0
        """The last byte the device sent, which is what a read returns."""
        self.transfers = 0
        self._pending = False
        self._overflowed = False

    # -- configuration -----------------------------------------------------

    def attach(self, bus: Optional[SpiBus]) -> None:
        self.bus = bus

    def reset(self) -> None:
        self.csr = [0] * self.channels
        self.enabled = False
        self.master = True
        self.config = 0
        self.selected = 0
        self.last_received = 0
        self.transfers = 0
        self._pending = False
        self._overflowed = False

    def select(self, channel: int) -> None:
        self.selected |= 1 << channel

    def deselect(self, channel: int) -> None:
        self.selected &= ~(1 << channel)

    # -- semantic status the adapter maps onto its own bits ----------------

    @property
    def ready_to_transmit(self) -> bool:
        return self.enabled and self.bus is not None

    @property
    def data_available(self) -> bool:
        """A byte is waiting in the receive register."""
        return self._pending

    @property
    def overrun(self) -> bool:
        """A transfer completed before the previous answer was read."""
        return self._overflowed

    @property
    def busy(self) -> bool:
        return False

    # -- the transfer itself -----------------------------------------------

    def transfer(self, value: int) -> None:
        """Clock one byte out and keep the byte that came back during the transfer.

        That byte is the one the device put on the line *before* this transfer started --
        its answer to the previous byte -- because a shift register cannot answer a byte
        while it is still receiving it.  Sampling first is also what a driver does: the
        SAM ``SPIClass::transfer()`` reads the receive register and then writes the
        transmit register again.

        A transfer always yields a byte, even when nothing on the bus is driving the data
        line: the master shifts in the idle level, and a driver that waits for the
        receive flag after every byte would otherwise hang on the first write to a
        write-only device such as a display.
        """
        if self._pending:
            # The previous answer was never read: that is the overrun the status
            # register reports.
            self._overflowed = True
        self.last_received = self._sample()
        if self.bus is not None:
            self.bus.write(bytes([value & 0xFF]))
        self.transfers += 1
        self._pending = True

    def _sample(self) -> int:
        """The byte on the data line: what the device queued, or the idle level."""
        if self.bus is not None and self.bus.in_waiting > 0:
            return int.from_bytes(self.bus.read(1), "little")
        return IDLE_LINE

    def take_response(self) -> int:
        """Read the receive register: the last byte that was clocked in.

        Reading it twice gives the same byte -- nothing new can have arrived without a
        transfer -- and clearing the flag is what an overrun is measured from.
        """
        if self._pending:
            self._pending = False
            self._overflowed = False
        return self.last_received

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return (
            f"<SpiController enabled={self.enabled} master={self.master} "
            f"selected=0x{self.selected:x} transfers={self.transfers}>"
        )


class SpiBusSelector:
    """Several devices on one SPI controller, one chip-select pin each.

    A controller has a single byte stream, but a board may put more than one device on
    it and pick between them with an ordinary output pin -- the Adafruit TFT shield has
    an ILI9341 on one and a microSD socket on another, both on the Due's SPI header.
    The controller is handed *this* instead of a device: every byte goes to whichever
    device has its chip select asserted, and to nobody when none has, which is what the
    bus itself does.

    A device is told when it is selected and released, because on real hardware that is
    where a transaction begins and ends (a flash chip resets its command latch, an SD
    card ends its command frame).  Devices that do not care simply do not implement
    ``select()``.
    """

    def __init__(self, fallback: Optional[SpiBus] = None) -> None:
        self.fallback = fallback
        """Where bytes go while no device is selected (a host bridge, usually)."""
        self.devices: "OrderedDict[str, Any]" = OrderedDict()
        self.chip_selects: Dict[str, Tuple[int, bool]] = {}
        """Device name -> (pin, active_low)."""
        self._ports: Dict[int, Any] = {}
        self._hooks: Dict[int, Any] = {}
        self._sources: list = []
        """(port, pin, name, active_low) for every chip select, for level sampling."""
        self._revisions: Dict[int, int] = {}
        self._selected: Optional[str] = None
        self.routed_bytes = 0
        self.unclaimed_bytes = 0

    # -- membership --------------------------------------------------------

    def add(
        self,
        name: str,
        device: Any,
        gpio: Any = None,
        pin: Optional[int] = None,
        active_low: bool = True,
    ) -> Any:
        """Put a device on the bus; returns the device, so calls can be chained."""
        if name in self.devices:
            raise XwInvalidParameter(f"{name!r} is already on this SPI bus")
        self.devices[name] = device
        if pin is None:
            logger.warning(
                f"[{name:8s}]: no chip-select pin -- the bus cannot be shared, so this "
                "device answers whenever no other device is selected"
            )
            if self._selected is None:
                self._select(name)
            return device
        pin = int(pin)
        self.chip_selects[name] = (pin, bool(active_low))
        self._ports[pin] = gpio
        self._sources.append((gpio, pin, name, bool(active_low)))
        if gpio is not None:
            # The edge hook tells the device the moment its transaction starts or ends;
            # the level is sampled again per byte (see _refresh), because firmware often
            # writes the pin to the level it already had, which raises no edge at all.
            hook = partial(self._on_chip_select, name)
            self._hooks[pin] = hook
            gpio.add_edge_hook(pin, hook)
            self._on_chip_select(name, self._level(gpio, pin, bool(active_low)))
        return device

    @staticmethod
    def _level(port: Any, pin: int, active_low: bool) -> bool:
        """What the chip select reads as, to the best of the port's knowledge.

        A pin the guest has never written is *not* an asserted chip select: reading a
        pull resistor as one would put a device on a bus nobody selected it on -- which
        is exactly what happens on the TFT shield, where two devices share one
        controller and only one of them is being talked to.

        A pin the guest *has* written is taken at face value, from the output register:
        a chip select is an output by definition, and simple firmware (and the device
        tests) drive one without configuring the direction registers first.
        """
        written = getattr(port, "written", None)
        if written is not None and not (written >> pin) & 1:
            return active_low
        driven = getattr(port, "drives", None)
        if written is None and driven is not None and not driven(pin):
            return active_low
        outputs = getattr(port, "outputs", None)
        if outputs is not None:
            return bool((outputs >> pin) & 1)
        return bool(port.level(pin))

    def _refresh(self) -> None:
        """Re-read every chip select, but only when its port has been written to."""
        for port, pin, name, active_low in self._sources:
            revision = getattr(port, "revision", None)
            if revision is not None:
                key = id(port)
                if self._revisions.get(key) == revision:
                    continue
                self._revisions[key] = revision
            self._on_chip_select(name, self._level(port, pin, active_low))

    def adopt(self, bus: Optional[SpiBus]) -> None:
        """Take over a bus the controller was using before devices were attached."""
        if self.fallback is None:
            self.fallback = bus

    def remove(self, name: str) -> None:
        device = self.devices.pop(name, None)
        if name in self.chip_selects:
            pin, _ = self.chip_selects.pop(name)
            port, hook = self._ports.pop(pin, None), self._hooks.pop(pin, None)
            if port is not None and hook is not None:
                port.remove_edge_hook(pin, hook)
            self._sources = [source for source in self._sources if source[2] != name]
        if self._selected == name:
            self._release(name)
        if device is not None:
            self._notify(device, False)

    def device_at(self, name: str) -> Any:
        return self.devices[name]

    @property
    def selected(self) -> Optional[str]:
        """The name of the device currently answering, if any."""
        return self._selected

    def deselect(self) -> None:
        """Release every chip select -- as if the firmware had finished a transaction."""
        for name in list(self.devices):
            self._release(name)

    # -- chip select -------------------------------------------------------

    def _on_chip_select(self, name: str, high: bool) -> None:
        pin, active_low = self.chip_selects[name]
        if bool(high) != active_low:
            self._select(name)
        else:
            self._release(name)

    def _select(self, name: str) -> None:
        if self._selected == name:
            return
        if self._selected is not None:
            # Two chip selects asserted at once is a wiring fault on real hardware:
            # both parts would drive the data line.  The newest wins here.
            logger.debug(f"[SPI     ]: {name} selected while {self._selected} still was")
            self._release(self._selected)
        self._selected = name
        self._notify(self.devices[name], True)

    def _release(self, name: str) -> None:
        if self._selected != name:
            return
        self._selected = None
        self._notify(self.devices[name], False)

    @staticmethod
    def _notify(device: Any, active: bool) -> None:
        select = getattr(device, "select", None)
        if select is not None:
            select(active)

    # -- SpiBus: the controller's end of the bus ---------------------------

    def write(self, data: bytes) -> int:
        self._refresh()
        name = self._selected
        if name is None:
            self.unclaimed_bytes += len(data)
            if self.fallback is not None:
                self.fallback.write(data)
            return len(data)
        self.routed_bytes += len(data)
        self.devices[name].write(data)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        self._refresh()
        name = self._selected
        if name is None:
            if self.fallback is not None:
                return self.fallback.read(size)
            return b"\x00" * size
        answer = self.devices[name].read(size)
        if not answer and self.fallback is not None:
            # A device with nothing to say does not stop the host bridge from
            # answering: one of the two is the byte the controller is waiting for.
            return self.fallback.read(size)
        return answer

    @property
    def in_waiting(self) -> int:
        self._refresh()
        name = self._selected
        waiting = self.devices[name].in_waiting if name is not None else 0
        if not waiting and self.fallback is not None:
            waiting = self.fallback.in_waiting
        return waiting

    @property
    def peer_hint(self) -> str:
        names = ", ".join(self.devices) or "none"
        hint = f"spi://{names}"
        if self.fallback is not None:
            hint += f" (+{self.fallback.peer_hint})"
        return hint

    def close(self) -> None:
        if self.fallback is not None:
            close = getattr(self.fallback, "close", None)
            if close is not None:
                close()

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return (
            f"<SpiBusSelector devices={list(self.devices)} selected={self._selected!r} "
            f"routed={self.routed_bytes} unclaimed={self.unclaimed_bytes}>"
        )

# -*- coding: utf-8 -*-

"""I2C master behaviour and the device end of the bus.

An I2C transfer is a sequence, not a stream: a START with the slave address and the
direction bit, zero or more data bytes each acknowledged by the receiver, and a
STOP.  A repeated START in the middle is how a register read works -- write the
register pointer, then start again in the read direction.

So the controller exposes exactly that sequence, and a device implements it from the
other side.  :class:`RegisterDevice` saves the common case -- a chip whose first
written byte selects a register -- from being written out again for every part.
"""

from collections import OrderedDict
from typing import Dict, Iterable, List, Optional

from ...config import logger

__all__ = ["I2cBus", "I2cController", "I2cDevice", "RegisterDevice"]


class I2cDevice(object):
    """A device on the bus, addressed by the controller.

    Subclasses implement the wire events.  Returning ``False`` from :meth:`start`
    or :meth:`write` is a NACK, which is how a controller learns that nobody is
    listening.
    """

    address: Optional[int] = None

    def __init__(self, address: Optional[int] = None, name: str = "I2C") -> None:
        if address is not None:
            self.address = address
        self.name = name

    # -- wire events -------------------------------------------------------

    def start(self, address: int, reading: bool) -> bool:
        """Addressed as ``address``, expecting ``reading`` data.  True = ACK."""
        return True

    def write(self, data: bytes) -> bool:
        """Bytes the master sent.  True = ACK."""
        return True

    def read(self, count: int) -> bytes:
        """Bytes to send back; a short answer is padded by the bus."""
        return b"\xff" * count

    def stop(self) -> None:
        """The master released the bus."""

    def reset(self) -> None:
        """Return to the power-on state; devices with state override this."""

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        address = f"0x{self.address:02x}" if self.address is not None else "?"
        return f"<{type(self).__name__} {self.name!r} @{address}>"


class RegisterDevice(I2cDevice):
    """A device whose first written byte after a START selects a register.

    ``registers`` is the backing store.  Subclasses override
    :meth:`on_register_read` / :meth:`on_register_write` when a register has
    behaviour of its own (a status register that clears when read, a command
    register, a chip id).
    """

    def __init__(self, address: int, size: int = 256, name: str = "I2C") -> None:
        super().__init__(address, name)
        self.registers = bytearray(size)
        self.pointer: Optional[int] = None
        self.reading = False
        self._expect_pointer = True

    def reset(self) -> None:
        self.pointer = None
        self.reading = False
        self._expect_pointer = True

    # -- wire events -------------------------------------------------------

    def start(self, address: int, reading: bool) -> bool:
        if address != self.address:
            return False
        self.reading = reading
        # A write starts with the register pointer; a read continues from wherever
        # the pointer was left, which is the repeated-start case.
        self._expect_pointer = not reading
        return True

    def write(self, data: bytes) -> bool:
        for value in data:
            if self._expect_pointer:
                self.pointer = value & 0xFF
                self._expect_pointer = False
                self.on_register_write(self.pointer, None)
                continue
            register = self.pointer if self.pointer is not None else 0
            self.registers[register] = value & 0xFF
            self.on_register_write(register, value & 0xFF)
            self.pointer = (register + 1) % len(self.registers)
        return True

    def read(self, count: int) -> bytes:
        if self.pointer is None:
            self.pointer = 0
        out = bytearray()
        for _ in range(count):
            out.append(self.on_register_read(self.pointer) & 0xFF)
            self.pointer = (self.pointer + 1) % len(self.registers)
        return bytes(out)

    # -- hooks -------------------------------------------------------------

    def on_register_read(self, register: int) -> int:
        """Return the value a read of ``register`` produces."""
        return self.registers[register]

    def on_register_write(self, register: int, value: Optional[int]) -> None:
        """Called when ``register`` is selected (``value`` is None) or written."""


class I2cBus(object):
    """Routes the wire events to whichever device answers the address."""

    def __init__(self) -> None:
        self.devices: "OrderedDict[int, I2cDevice]" = OrderedDict()
        self.active: Optional[I2cDevice] = None
        self.transactions = 0
        self.nacks = 0

    def attach(self, device: I2cDevice) -> "I2cBus":
        """Put a device on the bus; returns the bus so calls can be chained."""
        if device.address is None:
            raise ValueError(f"{device!r} has no I2C address")
        self.devices[device.address] = device
        logger.debug(f"I2C: {device.name} at 0x{device.address:02x}")
        return self

    def detach(self, device: I2cDevice) -> None:
        self.devices.pop(device.address, None)
        if self.active is device:
            self.active = None

    def device_at(self, address: int) -> Optional[I2cDevice]:
        return self.devices.get(address)

    def reset(self) -> None:
        self.active = None
        self.transactions = 0
        self.nacks = 0
        for device in self.devices.values():
            device.reset()

    # -- wire events -------------------------------------------------------

    def start(self, address: int, reading: bool) -> bool:
        device = self.devices.get(address)
        self.active = device
        self.transactions += 1
        if device is None:
            self.nacks += 1
            logger.debug(f"I2C: no device at 0x{address:02x}")
            return False
        return bool(device.start(address, reading))

    def write(self, data: Iterable[int]) -> bool:
        if self.active is None:
            self.nacks += 1
            return False
        return bool(self.active.write(bytes(data)))

    def read(self, count: int) -> bytes:
        if self.active is None:
            return b"\xff" * count
        data = bytes(self.active.read(count))
        if len(data) < count:
            data += b"\xff" * (count - len(data))
        return data

    def stop(self) -> None:
        """End the transfer.

        The addressed device stays reachable: a master sends the STOP while it is
        clocking the *last* byte in, so that byte is still read from the same
        device afterwards.  The next START addresses whoever comes next.
        """
        if self.active is not None:
            self.active.stop()

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        addresses = ", ".join(f"0x{address:02x}" for address in self.devices)
        return f"<I2cBus [{addresses}]>"


class I2cController(object):
    """Master side of an I2C bus: the START/byte/ACK/STOP sequence.

    The internal address register that some controllers have is modelled the way the
    hardware uses it: those bytes go out right after the address, and a read follows
    with a repeated START -- which is exactly the register-read protocol.
    """

    def __init__(self, bus: Optional[I2cBus] = None) -> None:
        self.bus = bus if bus is not None else I2cBus()
        self.internal_address: Optional[bytes] = None
        self.transfers = 0
        self.nacks = 0

    def attach(self, bus: I2cBus) -> None:
        self.bus = bus

    def reset(self) -> None:
        self.internal_address = None
        self.transfers = 0
        self.nacks = 0

    def set_internal_address(self, value: int, size: int = 1) -> None:
        """Select the register a transfer starts from (the IADR equivalent)."""
        self.internal_address = (value & ((1 << (8 * size)) - 1)).to_bytes(size, "big")

    def clear_internal_address(self) -> None:
        self.internal_address = None

    # -- transactions ------------------------------------------------------

    def begin(self, address: int, reading: bool) -> bool:
        """Address a device, emitting any pending internal address first."""
        pending = self.internal_address
        if reading and pending is None:
            return self._start(address, True)
        if not self._start(address, False):
            return False
        if pending is not None:
            if not self.bus.write(pending):
                self.nacks += 1
                return False
            if reading and not self._start(address, True):
                return False
        self.transfers += 1
        return True

    def _start(self, address: int, reading: bool) -> bool:
        if not self.bus.start(address, reading):
            self.nacks += 1
            return False
        return True

    def write(self, value: int) -> bool:
        return self.write_bytes([value & 0xFF])

    def write_bytes(self, data: Iterable[int]) -> bool:
        if not self.bus.write(data):
            self.nacks += 1
            return False
        return True

    def read(self, count: int = 1) -> bytes:
        return self.bus.read(count)

    def stop(self) -> None:
        self.bus.stop()

    def scan(self) -> List[int]:
        """Address every device on the bus, the way a bus scanner does."""
        found = []
        for address in sorted(self.bus.devices):
            if self.bus.start(address, reading=False):
                found.append(address)
            self.bus.stop()
        return found

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"<I2cController devices={len(self.bus.devices)} transfers={self.transfers}>"


def device_map(devices: Iterable[I2cDevice]) -> Dict[int, I2cDevice]:
    """Index devices by address; useful when building a bus from a YAML list."""
    return {device.address: device for device in devices if device.address is not None}

# -*- coding: utf-8 -*-

"""Two-wire interface (TWI0/TWI1), the SAM3X I2C master.

This is an *adapter*: the register block and its status bits live here, the transfer
sequence lives in :class:`xuanwu.peripherals.bus.i2c.I2cController`.

The sequence is the one the Arduino ``Wire`` library drives, which the ASF helpers in
``system/libsam/source/twi.c`` make explicit:

* a **write** sets ``MMR`` (address, direction, internal-address size) and ``IADR``,
  then the *first* ``THR`` write starts the transfer -- START is implicit, there is
  no ``CR.START`` -- and each further byte waits for ``SR.TXRDY``; ``CR.STOP`` ends
  it and ``SR.TXCOMP`` reports completion;
* a **read** sets ``MMR`` with ``MREAD`` and then ``CR.START``; the master clocks
  bytes out of ``RHR`` while ``SR.RXRDY`` is set, sends ``CR.STOP`` before the last
  one, and waits for ``SR.TXCOMP``.

``Wire`` polls those bits with a ``micros()`` timeout, so the model only has to make
them true in the right order -- no interrupt is raised.  ``SR.NACK`` is what tells
the library that nobody answered, which is how a bus scan finds its devices.
"""

from enum import IntEnum
from typing import Any, Optional

from ....config import logger
from ....peripherals.bus.i2c import I2cBus, I2cController, I2cDevice
from ...base import ArmHardwareBase, Register
from .common import PID

__all__ = ["ArmSamTwi"]


class TWI_CR(IntEnum):
    START = 0
    STOP = 1
    MSEN = 2
    MSDIS = 3
    SVEN = 4
    SVDIS = 5
    QUICK = 6
    SWRST = 7


class TWI_SR(IntEnum):
    TXCOMP = 0
    RXRDY = 1
    TXRDY = 2
    SVREAD = 3
    SVACC = 4
    GACC = 5
    OVRE = 6
    NACK = 7
    ARBLST = 8
    SCL_WS = 9
    EOSACC = 10


class TWI_MMR(IntEnum):
    IADRSZ = 8
    MREAD = 12
    DADR = 16


class ArmSamTwi(ArmHardwareBase):
    """Two-wire interface, master mode."""

    NAME = "TWI"
    REGISTERS = (
        ("CR", "I", 0x000000FF),
        ("MMR", "I", 0x007F1F00),
        ("SMR", "I", 0x00000000),
        ("IADR", "I", 0x00FFFFFF),
        ("CWGR", "I", 0x0007FFFF),
        ("SR", "I", 0x00000000),
        ("IER", "I", 0xFFFFFFFF),
        ("IDR", "I", 0xFFFFFFFF),
        ("IMR", "I", 0x00000000),
        ("RHR", "I", 0x00000000),
        ("THR", "I", 0x000000FF),
        ("RESERVED0", "53I", 0x00000000),  # up to the PDC window at +0x100
    )

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        # The IRQ number comes from the chip description; the vendor constant is
        # only the fallback for descriptions written before that field existed.
        self._irq = kwargs.get("irq", PID.TWI0)
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write
        self.bus = I2cBus()
        self._core = I2cController(self.bus)
        self._mmr = 0
        self._address = 0
        self._internal = 0
        self._internal_size = 0
        self._reading = False
        self._active = False
        self._sr = (1 << TWI_SR.TXCOMP) | (1 << TWI_SR.TXRDY)

    # -- the device layer's view of the bus --------------------------------

    def attach_device(self, device: I2cDevice) -> I2cDevice:
        """Put a device on this controller's bus."""
        self.bus.attach(device)
        return device

    @property
    def peer_hint(self) -> str:
        addresses = ", ".join(f"0x{address:02x}" for address in self.bus.devices)
        return f"i2c://[{addresses}]" if addresses else "i2c://empty"

    def device_at(self, address: int) -> Optional[I2cDevice]:
        return self.bus.device_at(address)

    # -- internals ---------------------------------------------------------

    def _set_status(self, bit: int, state: bool) -> None:
        if state:
            self._sr |= 1 << bit
        else:
            self._sr &= ~(1 << bit)
        self.write_register("SR", self._sr)

    def _begin(self, reading: bool) -> bool:
        """Address the device and, for a read, clock the internal address out."""
        if self._internal_size:
            self._core.set_internal_address(self._internal, self._internal_size)
        else:
            self._core.clear_internal_address()
        self._reading = reading
        self._active = True
        self._set_status(TWI_SR.TXCOMP, False)
        self._set_status(TWI_SR.NACK, False)
        ok = self._core.begin(self._address, reading)
        if not ok:
            self._set_status(TWI_SR.NACK, True)
            self._set_status(TWI_SR.TXCOMP, True)
            self._active = False
            logger.debug(f"[{self.NAME:8s}]: no answer from 0x{self._address:02x}")
        return ok

    def _write_byte(self, value: int) -> None:
        if not self._active:
            if not self._begin(reading=False):
                return
        if not self._core.write(value):
            self._set_status(TWI_SR.NACK, True)
            self._active = False
        # The transmitter accepted the byte and is ready for the next one.
        self._set_status(TWI_SR.TXRDY, True)

    def _read_byte(self) -> int:
        data = self._core.read(1)
        return data[0] if data else 0xFF

    def _stop(self) -> None:
        self._core.stop()
        self._active = False
        self._set_status(TWI_SR.TXRDY, False)
        self._set_status(TWI_SR.RXRDY, False)
        # Transfer complete.  A read has to keep the last byte in RHR until the
        # guest reads it, which is why this does not touch the receive path.
        self._set_status(TWI_SR.TXCOMP, True)

    def reset(self):
        super().reset()
        self._mmr = 0
        self._address = 0
        self._internal = 0
        self._internal_size = 0
        self._reading = False
        self._active = False
        self._sr = (1 << TWI_SR.TXCOMP) | (1 << TWI_SR.TXRDY)
        self._core.reset()
        self.bus.reset()
        for name in ("IER", "IDR", "IMR", "RHR", "THR", "SMR", "CWGR"):
            self.write_register(name, 0)
        self.write_register("SR", self._sr)

    # -- register hooks ----------------------------------------------------

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        if name == "SR":
            data = self._sr
        elif name == "RHR":
            data = self._read_byte()
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name == "CR":
            if data & (1 << TWI_CR.SWRST):
                self.reset()
                # SWRST is self-clearing
                data &= ~(1 << TWI_CR.SWRST)
            if data & (1 << TWI_CR.MSEN):
                self._sr |= (1 << TWI_SR.TXCOMP) | (1 << TWI_SR.TXRDY)
            if data & (1 << TWI_CR.START):
                self._begin(reading=True)
                if self._reading and not (self._sr & (1 << TWI_SR.NACK)):
                    self._set_status(TWI_SR.RXRDY, True)
            if data & (1 << TWI_CR.STOP):
                self._stop()
            self.write_register("SR", self._sr)
        elif name == "MMR":
            self._mmr = data
            self._address = (data >> TWI_MMR.DADR) & 0x7F
            self._internal_size = (data >> TWI_MMR.IADRSZ) & 0x3
        elif name == "IADR":
            self._internal = data & 0xFFFFFF
        elif name == "THR":
            self._write_byte(data & 0xFF)
        elif name == "IER":
            val = self.read_register("IMR") | data
            self.write_register("IMR", val)
            data = 0
        elif name == "IDR":
            val = self.read_register("IMR") & ~data
            self.write_register("IMR", val)
        elif name == "CWGR":
            pass  # the bit rate is a timing question, not a behavioural one
        elif name == "SMR":
            logger.debug(f"[{name_:16s}]: slave mode is not modelled")
        return data

# -*- coding: utf-8 -*-

"""Serial peripheral interface.

This is an *adapter*: the register layout and its bit meanings live here, the
protocol lives in :class:`xuanwu.peripherals.bus.spi.SpiController`.  Everything the
guest sees -- chip-select decoding, the status bits, the transmit/receive pair --
is expressed in terms of that core.
"""

from enum import IntEnum
from typing import Any, Optional

from ....backends import create_bridge
from ....config import logger
from ....peripherals.bus.spi import SpiBus, SpiController
from ...base import ArmHardwareBase, Register

__all__ = ["ArmSamSpi"]


class SPI_CR(IntEnum):
    SPIEN = 0
    SPIDIS = 1
    SWRST = 7
    LASTXFER = 24


class SPI_MR(IntEnum):
    MSTR = 0
    PS = 1
    PCSDEC = 2
    MODFDIS = 4
    WDRBT = 5
    LLB = 7
    PCS = 16  # PCS = xxx0 => NPCS[3:0] = 1110
    DLYBCS = 24  # If DLYBCS is less than or equal to six, six MCK periods will be inserted by default.


class SPI_SR(IntEnum):
    RDRF = 0
    TDRE = 1
    MODF = 2
    OVRES = 3
    NSSR = 8
    TXEMPTY = 9
    UNDES = 10  # Slave Mode Only
    SPIENS = 16


class ArmSamSpi(ArmHardwareBase):
    """Serial Peripheral Interface"""
    # TODO: >8 bits

    NAME = "SPI"
    REGISTERS = (
        ("CR", "I", 0xFFFFFFFF),
        ("MR", "I", 0xFFFFFFFF),
        ("RDR", "I", 0x00000000),
        ("TDR", "I", 0xFFFFFFFF),
        ("SR", "I", 0x00000000),
        ("IER", "I", 0xFFFFFFFF),
        ("IDR", "I", 0xFFFFFFFF),
        ("IMR", "I", 0x00000000),
        ("RESERVED0", "4I", 0x00000000),
        ("CSR0", "I", 0xFFFFFFFF),
        ("CSR1", "I", 0xFFFFFFFF),
        ("CSR2", "I", 0xFFFFFFFF),
        ("CSR3", "I", 0xFFFFFFFF),
        ("RESERVED1", "38I", 0x00000000),
        ("WPMR", "I", 0xFFFFFFFF),
        ("WPSR", "I", 0x00000000),
    )

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._npcs = 0xf
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write
        # 'bridge' may be set in the chip YAML: auto | socat | tcp | loopback
        bridge = create_bridge(
            kwargs.get("bridge", "auto"),
            baudrate=kwargs.get("baudrate", 115200),
            prefix="spi",
        )
        self._core = SpiController(bus=bridge, channels=4)
        logger.info(f"Serial device (SPI): {bridge.peer_hint}")

    def __del__(self):
        core = getattr(self, "_core", None)
        # A device model on the bus (a flash chip, say) has nothing to close.
        close = getattr(core.bus, "close", None) if core is not None else None
        if close is not None:
            close()

    def reset(self):
        super().reset()
        self._core.reset()
        self.write_register("MR", 0x00000000)
        self.write_register("RDR", 0x00000000)
        self.write_register("SR", 0x00000000)
        for channel in range(4):
            self.write_register(f"CSR{channel}", 0x00000000)
        self.write_register("WPMR", 0x00000000)
        self.write_register("WPSR", 0x00000000)

    # -- the device layer's view of the bus --------------------------------

    @property
    def peer_hint(self) -> str:
        """Where an external program should attach (a pty path, or tcp://host:port)."""
        return self._core.bus.peer_hint if self._core.bus is not None else "spi://none"

    @property
    def bridge(self) -> Optional[SpiBus]:
        """The other end of this peripheral's byte stream."""
        return self._core.bus

    @bridge.setter
    def bridge(self, bridge: Optional[SpiBus]) -> None:
        # Lets the device layer take over the bus (or hand it back).
        previous = self._core.bus
        if previous is not None:
            adopt = getattr(bridge, "adopt", None)
            if adopt is not None:
                # A bus that can host several devices keeps the old one as its
                # fallback instead of having it closed under it.
                adopt(previous)
            else:
                close = getattr(previous, "close", None)
                if close is not None:
                    close()
        self._core.attach(bridge)

    # -- register hooks ----------------------------------------------------

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        # name_ = ".".join([self.NAME, name])
        if name == "SR":
            # The status bits come from the core, which already fetched the answer
            # to the last transfer -- firmware polls SR before *and* after every
            # byte, so this is the hottest register of all.  The derived bits are
            # replaced, not merged: leaving a stale RDRF behind makes a driver
            # believe a byte is waiting when it has already been read.
            derived = (1 << SPI_SR.RDRF) | (1 << SPI_SR.OVRES)
            sr = (data & ~derived) | (1 << SPI_SR.TXEMPTY)
            if self._core.data_available:
                sr |= 1 << SPI_SR.RDRF
            if self._core.overrun:
                sr |= 1 << SPI_SR.OVRES
            if sr != data:
                self.write_register("SR", sr)
            data = sr
        elif name == "RDR":
            data = self._core.take_response()
            mr = self.read_register("MR")
            if mr & (1 << SPI_MR.MSTR):
                # Master mode: echo the PCS field along with the received data.
                data |= mr & 0x000F0000
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name == "CR":
            sr = self.read_register("SR")
            if data & (1 << SPI_CR.SPIDIS):
                self._core.enabled = False
                sr &= ~(1 << SPI_SR.SPIENS)
                sr &= ~(1 << SPI_SR.TDRE)
            elif data & (1 << SPI_CR.SPIEN):
                self._core.enabled = True
                sr |= 1 << SPI_SR.SPIENS
                sr |= 1 << SPI_SR.TDRE
            if data & (1 << SPI_CR.SWRST):
                self._core.reset()
                # The reset leaves MR at zero, so the core has to be told as well:
                # writing the register directly does not go through the hook above.
                self._core.master = False
                self._core.config = 0
                self.write_register("MR", 0)
            if data & (1 << SPI_CR.LASTXFER):
                self._npcs = 0xF
            self.write_register("SR", sr)
        elif name == "MR":
            self._core.master = bool(data & (1 << SPI_MR.MSTR))
            self._core.config = data
            if data & (1 << SPI_MR.PS) == 0x0:
                self._npcs = self._decode_pcs(data)
        elif name == "TDR":
            mr = self.read_register("MR")
            if mr & (1 << SPI_MR.PS):
                self._npcs = self._decode_pcs(data)
                if data & (1 << SPI_CR.LASTXFER):
                    self._npcs = 0xF
            self._core.transfer(data & 0xFF)
        elif name.startswith("CSR"):
            wpmr = self.read_register("WPMR")
            if wpmr != 0x0:
                data = data_orig
                logger.warning(f"[{name_:16s}]: Ignore write")
            else:
                channel = int(name[3:])
                if 0 <= channel < self._core.channels:
                    self._core.csr[channel] = data
        elif name in ["IER"]:
            reg = name[:-2] + "M" + name[-1:]
            val = self.read_register(reg)
            new_val = val | data
            self.write_register(reg, new_val)
            data = 0
        elif name in ["IDR"]:
            reg = name[:-2] + "M" + name[-1:]
            val = self.read_register(reg)
            new_val = val & ~data
            self.write_register(reg, new_val)
        elif name == "WPMR":
            if (data & 0xFFFFFF00) != 0x53504900:
                logger.warning(f"[{name_:16s}]: Invalid WPKEY, 0x{data:08x}")
                data = data_orig
            else:
                data &= 0x01
        return data

    @staticmethod
    def _decode_pcs(data: int) -> int:
        """Turn the PCS field of MR/TDR into the NPCS mask it selects."""
        pcs = (data >> SPI_MR.PCS) & 0xF
        if pcs & 0x1 == 0x0:
            return 0xE
        if pcs & 0x3 == 0x1:
            return 0xD
        if pcs & 0x7 == 0x3:
            return 0xB
        if pcs & 0xF == 0x7:
            return 0x7
        return 0xF

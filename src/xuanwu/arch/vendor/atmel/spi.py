# -*- coding: utf-8 -*-

"""Serial peripheral interface."""

from enum import IntEnum
from typing import Any, Optional

from ....backends import create_bridge
from ....config import logger
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
    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._npcs = 0xf
        self._last_rx = 0
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write
        # 'bridge' may be set in the chip YAML: auto | socat | tcp | loopback
        self._bridge = create_bridge(
            kwargs.get("bridge", "auto"),
            baudrate=kwargs.get("baudrate", 115200),
            prefix="spi",
        )
        logger.info(f"Serial device (SPI): {self._bridge.peer_hint}")

    def __del__(self):
        bridge = getattr(self, "_bridge", None)
        if bridge is not None:
            bridge.close()

    def reset(self):
        super().reset()
        self.write_register("MR", 0x00000000)
        self.write_register("RDR", 0x00000000)
        self.write_register("SR", 0x00000000)
        self.write_register("CSR0", 0x00000000)
        self.write_register("CSR1", 0x00000000)
        self.write_register("CSR2", 0x00000000)
        self.write_register("CSR3", 0x00000000)
        self.write_register("WPMR", 0x00000000)
        self.write_register("WPSR", 0x00000000)

    @property
    def peer_hint(self) -> str:
        """Where an external program should attach (a pty path, or tcp://host:port)."""
        return self._bridge.peer_hint

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        # name_ = ".".join([self.NAME, name])
        if name == "SR":
            sr = data
            if self._bridge.in_waiting > 0:
                sr |= 1 << SPI_SR.RDRF
            sr |= 1 << SPI_SR.TXEMPTY
            if sr != data:
                self.write_register("SR", data)
            if self._bridge.in_waiting > 1:
                data |= 1 << SPI_SR.OVRES
        elif name == "RDR":
            if self._bridge.in_waiting > 0:
                # self._last_rx = int.from_bytes(self._bridge.read(self._bridge.in_waiting + 10)[-1], "little")
                self._last_rx = int.from_bytes(self._bridge.read(1), "little")
            data = self._last_rx
            mr = self.read_register("MR")
            if mr & (1 << SPI_MR.MSTR):
                # Master mode: echo the PCS field along with the received data.
                data |= mr & 0x000F0000
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name == "CR":
            sr = self.read_register("SR")
            # sr_orig = sr
            if data & (1 << SPI_CR.SPIDIS):
                sr &= ~(1 << SPI_SR.SPIENS)
                sr &= ~(1 << SPI_SR.TDRE)
            elif data & (1 << SPI_CR.SPIEN):
                sr |= 1 << SPI_SR.SPIENS
                sr |= 1 << SPI_SR.TDRE
            if data & (1 << SPI_CR.SWRST):
                mr = self.read_register("MR")
                mr &= ~(1 << SPI_MR.MSTR)
                self.write_register("MR", mr)
            if data & (1 << SPI_CR.LASTXFER):
                self._npcs = 0xF
            self.write_register("SR", sr)
        elif name == "MR":
            if data & (1 << SPI_MR.PS) == 0x0:
                pcs = (data >> SPI_MR.PCS) & 0xF
                if data & (1 << SPI_MR.PCSDEC):
                    self._npcs = pcs
                elif pcs & 0x1 == 0x0:
                    self._npcs = 0xE
                elif pcs & 0x3 == 0x1:
                    self._npcs = 0xD
                elif pcs & 0x7 == 0x3:
                    self._npcs = 0xB
                elif pcs & 0xf == 0x7:
                    self._npcs = 0x7
        elif name == "TDR":
            mr = self.read_register("MR")
            if mr & (1 << SPI_MR.PS):
                pcs = (data >> SPI_MR.PCS) & 0xF
                if mr & (1 << SPI_MR.PCSDEC):
                    self._npcs = pcs
                elif pcs & 0x1 == 0x0:
                    self._npcs = 0xE
                elif pcs & 0x3 == 0x1:
                    self._npcs = 0xD
                elif pcs & 0x7 == 0x3:
                    self._npcs = 0xB
                elif pcs & 0xf == 0x7:
                    self._npcs = 0x7
                if data & (1 << SPI_CR.LASTXFER):
                    self._npcs = 0xF
            # else:
            #     pcs = (mr >> SPI_MR.PCS) & 0xF
            self._bridge.write((data & 0xFF).to_bytes(1, "little"))
            # logger.debug(f'[{name_:16s}]: Output "{(data & 0xFF).to_bytes(1, "little")}"')
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
        elif name.startswith("CSR"):
            wpmr = self.read_register("WPMR")
            if wpmr != 0x0:
                data = data_orig
                logger.warning(f"[{name_:16s}]: Ignore write")
        elif name == "WPMR":
            if (data & 0xFFFFFF00) != 0x53504900:
                logger.warning(f"[{name_:16s}]: Invalid WPKEY, 0x{data:08x}")
                data = data_orig
            else:
                data &= 0x01
        return data

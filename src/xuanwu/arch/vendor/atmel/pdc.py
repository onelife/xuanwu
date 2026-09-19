# -*- coding: utf-8 -*-

"""Peripheral DMA controller."""

from enum import IntEnum
from typing import Any

from ....config import logger
from ...base import ArmHardwareBase, Register

__all__ = ["ArmSamPdc"]


class PDC_PTCR(IntEnum):
    RXTEN = 0
    RXTDIS = 1
    TXTEN = 8
    TXTDIS = 9


class PDC_PTSR(IntEnum):
    RXTEN = 0
    TXTEN = 8


class ArmSamPdc(ArmHardwareBase):
    """Peripheral DMA Controller """

    NAME = "PDC"
    REGISTERS = (
        ("RPR", "I", 0xFFFFFFFF),
        ("RCR", "I", 0x0000FFFF),
        ("TPR", "I", 0xFFFFFFFF),
        ("TCR", "I", 0x0000FFFF),
        ("RNPR", "I", 0xFFFFFFFF),
        ("RNCR", "I", 0x0000FFFF),
        ("TNPR", "I", 0xFFFFFFFF),
        ("TNCR", "I", 0x0000FFFF),
        ("PTCR", "I", 0x00000303),
        ("PTSR", "I", 0x00000101),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._fix_before_write = self.fix_before_write

    def reset(self):
        super().reset()
        self.write_register("RPR", 0x00000000)
        self.write_register("RCR", 0x00000000)
        self.write_register("TPR", 0x00000000)
        self.write_register("TCR", 0x00000000)
        self.write_register("RNPR", 0x00000000)
        self.write_register("RNCR", 0x00000000)
        self.write_register("TNPR", 0x00000000)
        self.write_register("TNCR", 0x00000000)
        self.write_register("PTCR", 0x00000000)
        self.write_register("PTSR", 0x00000000)

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self._name, name])
        # update status register
        if name == "PTCR":
            ptsr = self.read_register("PTSR")
            if data & (1 << PDC_PTCR.RXTEN):
                ptsr |= 1 << PDC_PTSR.RXTEN
                logger.info(f"[{name_:16s}]: Enable TX")
            if data & (1 << PDC_PTCR.RXTDIS):
                ptsr &= ~(1 << PDC_PTSR.RXTEN)
                logger.info(f"[{name_:16s}]: Disable TX")
            if data & (1 << PDC_PTCR.TXTEN):
                ptsr |= 1 << PDC_PTSR.TXTEN
                logger.info(f"[{name_:16s}]: Enable RX")
            if data & (1 << PDC_PTCR.TXTDIS):
                ptsr &= ~(1 << PDC_PTSR.TXTEN)
                logger.info(f"[{name_:16s}]: Disable RX")
            self.write_register("PTSR", ptsr)
            data = 0
        return data

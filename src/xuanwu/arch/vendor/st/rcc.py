# -*- coding: utf-8 -*-

"""Reset and clock control."""

from typing import Any

from ....config import logger
from ...base import ArmHardwareBase, Register

__all__ = ["ArmStmRcc"]


class ArmStmRcc(ArmHardwareBase):
    """Reset and clock control"""

    NAME = "RCC"
    REGISTERS = (
        ("CR", "I", 0x050D00F9),
        ("PLLCFGR", "I", 0x0F437FFF),
        ("CFGR", "I", 0xFFFFFCF3),
        ("CIR", "I", 0x00BF3F00),
        ("AHB1RSTR", "I", 0x0060109F),
        ("AHB2RSTR", "I", 0x00000080),
        ("RESERVED0", "2I", 0x00000000),
        ("APB1RSTR", "I", 0x10E2C80F),
        ("APB2RSTR", "I", 0x00177931),
        ("RESERVED1", "2I", 0x00000000),
        ("AHB1ENR", "I", 0x0060109F),
        ("AHB2ENR", "I", 0x00000080),
        ("RESERVED2", "2I", 0x00000000),
        ("APB1ENR", "I", 0x10E2C80F),
        ("APB2ENR", "I", 0x00177931),
        ("RESERVED3", "2I", 0x00000000),
        ("AHB1LPENR", "I", 0x0061909F),
        ("AHB2LPENR", "I", 0x00000080),
        ("RESERVED4", "2I", 0x00000000),
        ("APB1LPENR", "I", 0x10E2C80F),
        ("APB2LPENR", "I", 0x00177931),
        ("RESERVED5", "2I", 0x00000000),
        ("BDCR", "I", 0x0001830D),
        ("CSR", "I", 0x01000001),
        ("RESERVED6", "2I", 0x00000000),
        ("SSCGR", "I", 0xCFFFFFFF),
        ("PLLI2SCFGR", "I", 0x70007FFF),
        ("RESERVED7", "I", 0x00000000),
        ("DCKCFGR", "I", 0x01000000),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._fix_before_write = self.fix_before_write

    def reset(self):
        super().reset()
        self.write_register("CR", 0x00000081)
        self.write_register("PLLCFGR", 0x24003010)
        self.write_register("CFGR", 0x0)
        self.write_register("CIR", 0x0)
        self.write_register("AHB1RSTR", 0x0)
        self.write_register("AHB2RSTR", 0x0)
        self.write_register("APB1RSTR", 0x0)
        self.write_register("APB2RSTR", 0x0)
        self.write_register("AHB1ENR", 0x0)
        self.write_register("AHB2ENR", 0x0)
        self.write_register("APB1ENR", 0x0)
        self.write_register("APB2ENR", 0x0)
        self.write_register("AHB1LPENR", 0x0061900F)
        self.write_register("AHB2LPENR", 0x00000080)
        self.write_register("APB1LPENR", 0x10E2C80F)
        self.write_register("APB2LPENR", 0x00077930)
        self.write_register("BDCR", 0x00077930)
        self.write_register("CSR", 0x0E000000)
        self.write_register("SSCGR", 0x0)
        self.write_register("PLLI2SCFGR", 0x24003000)
        self.write_register("DCKCFGR", 0x0)

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name == "CR":
            # set ready, if enabled clock
            clock_mask = 0x05010001
            status_mask = clock_mask << 1
            clock = data & clock_mask
            logger.debug(f"[{name_:16s}]: Enable clock 0x{clock:08x}")
            data = (data & ~status_mask) | (clock << 1)
        elif name == "CFGR":
            # set status, if selected clock
            clock_mask = 0x00000003
            status_mask = clock_mask << 2
            clock = data & clock_mask
            # clock = data & 0x00000003
            logger.debug(f"[{name_:16s}]: Select clock 0x{clock:08x}")
            data = (data & ~status_mask) | (clock << 2)
        return data

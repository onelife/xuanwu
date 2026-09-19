# -*- coding: utf-8 -*-

"""Analog-to-digital converter."""

from enum import IntEnum
from typing import Any

from ....config import logger
from ...base import ArmHardwareBase, Register

__all__ = ["ArmSamAdc"]


class ADC_CR(IntEnum):
    SWRST = 0
    START = 1


class ADC_MR(IntEnum):
    USEQ = 31


class ADC_ISR(IntEnum):
    DRDY = 24
    GOVRE = 25
    COMPE = 26
    ENDRX = 27
    RXBUFF = 28


class ADC_EMR(IntEnum):
    TAG = 24


class ArmSamAdc(ArmHardwareBase):
    """Analog-to-Digital Converter"""
    # TODO: CLK selection?

    NAME = "ADC"
    REGISTERS = (
        ("CR", "I", 0x00000003),
        ("MR", "I", 0xFFFFFFFF),
        ("SEQR1", "I", 0xFFFFFFFF),
        ("SEQR2", "I", 0xFFFFFFFF),
        ("CHER", "I", 0x0000FFFF),
        ("CHDR", "I", 0x0000FFFF),
        ("CHSR", "I", 0x00000000),
        ("RESERVED0", "I", 0x00000000),
        ("LCDR", "I", 0x00000000),
        ("IER", "I", 0xFFFFFFFF),
        ("IDR", "I", 0xFFFFFFFF),
        ("IMR", "I", 0x00000000),
        ("ISR", "I", 0x00000000),
        ("RESERVED1", "2I", 0x00000000),
        ("OVER", "I", 0x00000000),
        ("EMR", "I", 0xFFFFFFFF),
        ("CWR", "I", 0xFFFFFFFF),
        ("CGR", "I", 0xFFFFFFFF),
        ("COR", "I", 0xFFFFFFFF),
        ("CDR0", "I", 0x00000000),
        ("CDR1", "I", 0x00000000),
        ("CDR2", "I", 0x00000000),
        ("CDR3", "I", 0x00000000),
        ("CDR4", "I", 0x00000000),
        ("CDR5", "I", 0x00000000),
        ("CDR6", "I", 0x00000000),
        ("CDR7", "I", 0x00000000),
        ("CDR8", "I", 0x00000000),
        ("CDR9", "I", 0x00000000),
        ("CDR10", "I", 0x00000000),
        ("CDR11", "I", 0x00000000),
        ("CDR12", "I", 0x00000000),
        ("CDR13", "I", 0x00000000),
        ("CDR14", "I", 0x00000000),
        ("CDR15", "I", 0x00000000),
        ("RESERVED2", "I", 0x00000000),
        ("ACR", "I", 0xFFFFFFFF),
        ("RESERVED3", "19I", 0x00000000),
        ("WPMR", "I", 0xFFFFFFFF),
        ("WPSR", "I", 0x00000000),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write

    def reset(self):
        super().reset()
        self.write_register("CR", 0x0)
        self.write_register("MR", 0x0)
        self.write_register("SEQR1", 0x0)
        self.write_register("SEQR2", 0x0)
        self.write_register("CHSR", 0x0)
        self.write_register("LCDR", 0x0)
        self.write_register("IMR", 0x0)
        self.write_register("ISR", 0x0)
        self.write_register("OVER", 0x0)
        self.write_register("EMR", 0x0)
        self.write_register("CWR", 0x0)
        self.write_register("CGR", 0x0)
        self.write_register("COR", 0x0)
        self.write_register("CDR0", 0x000000)
        self.write_register("CDR1", 0x000101)
        self.write_register("CDR2", 0x000202)
        self.write_register("CDR3", 0x000303)
        self.write_register("CDR4", 0x000404)
        self.write_register("CDR5", 0x000505)
        self.write_register("CDR6", 0x00606)
        self.write_register("CDR7", 0x000707)
        self.write_register("CDR8", 0x000808)
        self.write_register("CDR9", 0x000909)
        self.write_register("CDR10", 0x000A0A)
        self.write_register("CDR11", 0x000B0B)
        self.write_register("CDR12", 0x000C0C)
        self.write_register("CDR13", 0x000D0D)
        self.write_register("CDR14", 0x000E0E)
        self.write_register("CDR15", 0x000F0F)
        self.write_register("ACR", 0x0)
        self.write_register("WPMR", 0x0)
        self.write_register("WPSR", 0x0)

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name == "OVER":
            # read to clear
            self.write_register(name, 0)
        elif name == "LCDR":
            logger.debug(f"[{name_:16s}]: Last ADC result, 0x{data & 0xFFF:08x}")
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name == "CR":
            if data & (1 << ADC_CR.START):
                chsr = self.read_register("CHSR")
                if chsr != 0:
                    isr = chsr | (1 << ADC_ISR.DRDY)
                    self.write_register("ISR", isr)
                    enabled = []
                    for i in range(16):
                        if chsr & 0x01:
                            enabled.append(i)
                        chsr >>= 1
                    mr = self.read_register("MR")
                    if mr & (1 << ADC_MR.USEQ):
                        seq = []
                        seqr1 = self.read_register("SEQR1")
                        seqr2 = self.read_register("SEQR2")
                        for seqr in [seqr1, seqr2]:
                            for i in range(8):
                                ch = seqr & 0x0F
                                if ch in enabled:
                                    seq.append(ch)
                                seqr >>= 4
                        last_ch = seq[-1]
                    else:
                        last_ch = enabled[-1]
                    cdr = self.read_register(f"CDR{last_ch}")
                    emr = self.read_register("EMR")
                    lcdr = cdr
                    if emr & (1 << ADC_EMR.TAG):
                        lcdr |= last_ch << 12
                    self.write_register("LCDR", lcdr)
        elif name in ["CHER"]:
            wpmr = self.read_register("WPMR")
            if wpmr == 0x0:
                reg = name[:-2] + "S" + name[-1:]
                val = self.read_register(reg)
                new_val = val | data
                self.write_register(reg, new_val)
                diff = val ^ new_val
                for i in range(32):
                    if diff & 0x01:
                        logger.debug(f"[{name_:16s}]: Enable {self.NAME}{i}")
                    diff >>= 1
            else:
                logger.warning(f"[{name_:16s}]: Ignore write")
            data = 0
        elif name in ["CHDR"]:
            wpmr = self.read_register("WPMR")
            if wpmr == 0x0:
                reg = name[:-2] + "S" + name[-1:]
                val = self.read_register(reg)
                new_val = val & ~data
                self.write_register(reg, new_val)
                diff = val ^ new_val
                for i in range(32):
                    if diff & 0x01:
                        logger.debug(f"[{name_:16s}]: Disable {self.NAME}{i}")
                    diff >>= 1
            else:
                logger.warning(f"[{name_:16s}]: Ignore write")
            data = 0
        elif name in ["MR", "SEQR1", "SEQR2", "EMR", "CWR", "CGR", "COR", "ACR"]:
            wpmr = self.read_register("WPMR")
            if wpmr != 0x0:
                data = data_orig
                logger.warning(f"[{name_:16s}]: Ignore write")
        elif name == "WPMR":
            if (data & 0xFFFFFF00) != 0x41444300:
                logger.warning(f"[{name_:16s}]: Invalid WPKEY, 0x{data:08x}")
                data = data_orig
            else:
                data &= 0x01
        return data

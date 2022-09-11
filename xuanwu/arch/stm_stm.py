# -*- coding: utf-8 -*-

from typing import Any

from ..config import logger
from .armv7m import Register, ArmHardwareBase


__all__ = ["BUILDIN"]


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


class ArmStmGpio(ArmHardwareBase):
    """General-purpose I/Os"""

    NAME = "GPIO"
    REGISTERS = (
        ("MODER", "I", 0xFFFFFFFF),
        ("OTYPER", "I", 0x0000FFFF),
        ("OSPEEDR", "I", 0xFFFFFFFF),
        ("PUPDR", "I", 0xFFFFFFFF),
        ("IDR", "I", 0x00000000),  # word access only
        ("ODR", "I", 0x0000FFFF),
        ("BSRR", "I", 0xFFFFFFFF),
        ("LCKR", "I", 0x0001FFFF),
        ("AFRL", "I", 0xFFFFFFFF),
        ("AFRH", "I", 0xFFFFFFFF),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.NAME = self._name.upper()
        self._fix_before_write = self.fix_before_write
        self._lock_seq = 0
        self._lock_val = 0

    def reset(self):
        super().reset()
        if self.NAME.endswith("A"):
            self.write_register("MODER", 0xA8000000)
            self.write_register("OSPEEDR", 0x0C000000)
            self.write_register("PUPDR", 0x64000000)
        if self.NAME.endswith("B"):
            self.write_register("MODER", 0x00000280)
            self.write_register("OSPEEDR", 0x000000C0)
            self.write_register("PUPDR", 0x00000100)
        else:
            self.write_register("MODER", 0x0)
            self.write_register("OSPEEDR", 0x0)
            self.write_register("PUPDR", 0x0)
        self.write_register("OTYPER", 0x0)
        self.write_register("IDR", 0x0)
        self.write_register("ODR", 0x0)
        self.write_register("BSRR", 0x0)
        self.write_register("LCKR", 0x0)
        self.write_register("AFRL", 0x0)
        self.write_register("AFRH", 0x0)

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if self._lock_seq >= 3:
            logger.debug(f"[{name_:16s}]: Config is locked, 0x{self._lock_val:08x}")
            if name in ["OTYPER"]:
                mask = self._lock_val & 0xFFFF
                data = (data & ~mask) | (data_orig & mask)
            elif name in ["MODER", "OSPEEDR", "PUPDR"]:
                val = self._lock_val
                mask = 0
                for i in range(16):
                    val_ = val & 0x1
                    if val_:
                        mask |= 0x3
                    val >>= 1
                    mask <<= 2
                data = (data & ~mask) | (data_orig & mask)
            elif name in ["AFRL", "AFRH"]:
                val = self._lock_val
                mask = 0
                for i in range(16):
                    val_ = val & 0x1
                    if val_:
                        mask |= 0xF
                    val >>= 1
                    mask <<= 4
                if name == "AFRH":
                    mask >>= 32
                mask &= 0xFFFFFFFF
                data = (data & ~mask) | (data_orig & mask)
        if name == "ODR":
            odr = data_orig
            diff = odr ^ data
            for i in range(16):
                diff_ = diff & 0x01
                odr_ = odr & 0x01
                if diff_:
                    if odr_:
                        logger.debug(f"[{name_:16s}]: Reset P{self.NAME[-1]}{i}")
                    else:
                        logger.debug(f"[{name_:16s}]: Set P{self.NAME[-1]}{i}")
                diff >>= 1
                odr >>= 1
        elif name == "BSRR":
            logger.debug(f"[{name_:16s}]: BSRR <= 0x{data:08x}")
            br = (data >> 16) & 0xFFFF
            bs = data & 0xFFFF
            odr = self.read_register("ODR")
            odr = ((odr & ~br) | bs) & 0xFFFF
            self.write_register("ODR", odr)
            data = 0x0
            for i in range(16):
                br_ = br & 0x01
                bs_ = bs & 0x01
                if bs_:
                    logger.debug(f"[{name_:16s}]: Set P{self.NAME[-1]}{i}")
                elif br_:
                    logger.debug(f"[{name_:16s}]: Reset P{self.NAME[-1]}{i}")
                br >>= 1
                bs >>= 1
        elif name == "LCKR":
            if self._lock_seq <= 0 and (data & 0x10000) != 0x0:
                self._lock_seq = 1
                self._lock_val = data
                data = 0
            elif self._lock_seq in [1, 2]:
                if (data ^ self._lock_val) & 0x1FFFF == 0x10000:
                    self._lock_seq += 1
                    self._lock_val = data
                else:
                    self._lock_seq = 0
                    self._lock_val = 0
                    data = 0
            elif self._lock_seq >= 4:
                logger.warning(f"[{name_:16s}]: LCKR is already locked")
                data = data_orig
            logger.debug(f"[{name_:16s}]: LCKR write sequence: {self._lock_seq}")
            if self._lock_seq == 3:
                self._lock_seq += 1
        return data


BUILDIN = {
    "rcc": ArmStmRcc,
    "gpio": ArmStmGpio,
}

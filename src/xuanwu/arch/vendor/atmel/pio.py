# -*- coding: utf-8 -*-

"""Parallel I/O controller (PIOA..PIOF)."""

from typing import Any, Callable, Tuple, Union

from ....config import logger
from ...base import ArmHardwareBase, Register

__all__ = ["ArmSamGpio"]


class ArmSamGpio(ArmHardwareBase):
    """General-purpose I/Os"""

    NAME = "GPIO"
    REGISTERS = (
        ("PER", "I", 0xFFFFFFFF),
        ("PDR", "I", 0xFFFFFFFF),
        ("PSR", "I", 0x00000000),
        ("RESERVED0", "I", 0x00000000),
        ("OER", "I", 0xFFFFFFFF),
        ("ODR", "I", 0xFFFFFFFF),
        ("OSR", "I", 0x00000000),
        ("RESERVED1", "I", 0x00000000),
        ("IFER", "I", 0xFFFFFFFF),
        ("IFDR", "I", 0xFFFFFFFF),
        ("IFSR", "I", 0x00000000),
        ("RESERVED2", "I", 0x00000000),
        ("SODR", "I", 0xFFFFFFFF),
        ("CODR", "I", 0xFFFFFFFF),
        ("ODSR", "I", 0xFFFFFFFF),
        ("PDSR", "I", 0x00000000),
        ("IER", "I", 0xFFFFFFFF),
        ("IDR", "I", 0xFFFFFFFF),
        ("IMR", "I", 0x00000000),
        ("ISR", "I", 0x00000000),
        ("MDER", "I", 0xFFFFFFFF),
        ("MDDR", "I", 0xFFFFFFFF),
        ("MDSR", "I", 0x00000000),
        ("RESERVED3", "I", 0x00000000),
        ("PUDR", "I", 0xFFFFFFFF),
        ("PUER", "I", 0xFFFFFFFF),
        ("PUSR", "I", 0x00000000),
        ("RESERVED4", "I", 0x00000000),
        ("ABSR", "I", 0xFFFFFFFF),
        ("RESERVED5", "3I", 0x00000000),
        ("SCIFSR", "I", 0xFFFFFFFF),
        ("DIFSR", "I", 0xFFFFFFFF),
        ("IFDGSR", "I", 0x00000000),
        ("SCDR", "I", 0x00003FFF),
        ("RESERVED6", "4I", 0x00000000),
        ("OWER", "I", 0xFFFFFFFF),
        ("OWDR", "I", 0xFFFFFFFF),
        ("OWSR", "I", 0x00000000),
        ("RESERVED7", "I", 0x00000000),
        ("AIMER", "I", 0xFFFFFFFF),
        ("AIMDR", "I", 0xFFFFFFFF),
        ("AIMMR", "I", 0x00000000),
        ("RESERVED8", "I", 0x00000000),
        ("ESR", "I", 0xFFFFFFFF),
        ("LSR", "I", 0xFFFFFFFF),
        ("ELSR", "I", 0x00000000),
        ("RESERVED9", "I", 0x00000000),
        ("FELLSR", "I", 0xFFFFFFFF),
        ("REHLSR", "I", 0xFFFFFFFF),
        ("FRLHSR", "I", 0x00000000),
        ("RESERVED10", "I", 0x00000000),
        ("LOCKSR", "I", 0x00000000),
        ("WPMR", "I", 0xFFFFFFFF),
        ("WPSR", "I", 0x00000000),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.NAME = self._name.upper()
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write
        self._hook = [set() for _ in range(32)]

    def add_hook(self, pin: int, fn: Tuple[Union[Callable, None]]) -> None:
        self._hook[pin].add((fn[0], fn[1]))

    def remove_hook(self, pin: int, fn: Tuple[Union[Callable, None]]) -> None:
        self._hook[pin].remove((fn[0], fn[1]))

    def reset(self):
        super().reset()
        self.write_register("PSR", 0x0)
        self.write_register("OSR", 0x0)
        self.write_register("IFSR", 0x0)
        self.write_register("ODSR", 0x0)
        self.write_register("PDSR", 0x0)
        self.write_register("IMR", 0x0)
        self.write_register("ISR", 0x0)
        self.write_register("MDSR", 0x0)
        self.write_register("PUSR", 0x0)
        self.write_register("ABSR", 0x0)
        self.write_register("IFDGSR", 0x0)
        self.write_register("SCDR", 0x0)
        self.write_register("OWSR", 0x0)
        self.write_register("AIMMR", 0x0)
        self.write_register("ELSR", 0x0)
        self.write_register("FRLHSR", 0x0)
        self.write_register("LOCKSR", 0x0)
        self.write_register("WPMR", 0x0)
        self.write_register("WPSR", 0x0)

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        if name == "ISR":
            self.write_register(name, 0)
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name in ["PER", "OER", "IFER", "MDER", "PUDR", "OWER"]:
            wpmr = self.read_register("WPMR")
            if wpmr == 0x0:
                reg = name[:-2] + "S" + name[-1:]
                val = self.read_register(reg)
                new_val = val | data
                self.write_register(reg, new_val)
                if name == "OER":
                    diff = val ^ new_val
                    for i in range(32):
                        if diff & 0x01:
                            logger.debug(f"[{name_:16s}]: Enable P{self.NAME[-1]}{i}")
                        diff >>= 1
            else:
                logger.warning(f"[{name_:16s}]: Ignore write")
            data = 0
        elif name in ["PDR", "ODR", "IFDR", "MDDR", "PUER", "OWDR"]:
            wpmr = self.read_register("WPMR")
            if wpmr == 0x0:
                reg = name[:-2] + "S" + name[-1:]
                val = self.read_register(reg)
                new_val = val & ~data
                self.write_register(reg, new_val)
                if name == "ODR":
                    diff = val ^ new_val
                    for i in range(32):
                        if diff & 0x01:
                            logger.debug(f"[{name_:16s}]: Disable P{self.NAME[-1]}{i}")
                        diff >>= 1
            else:
                logger.warning(f"[{name_:16s}]: Ignore write")
            data = 0
        elif name == "ABSR":
            wpmr = self.read_register("WPMR")
            if wpmr != 0x0:
                data = data_orig
                logger.warning(f"[{name_:16s}]: Ignore write")
        elif name in ["IER", "AIMER"]:
            reg = name[:-2] + "M" + name[-1:]
            val = self.read_register(reg)
            val |= data
            self.write_register(reg, val)
            data = 0
        elif name in ["IDR", "AIMDR"]:
            reg = name[:-2] + "M" + name[-1:]
            val = self.read_register(reg)
            val &= ~data
            self.write_register(reg, val)
        elif name == "SODR":
            reg = "ODSR"
            val = self.read_register(reg)
            new_val = val | data
            self.write_register(reg, new_val)
            # update PDSR
            psr = self.read_register("PSR")
            osr = self.read_register("OSR")
            pdsr = self.read_register("PDSR")
            mask = psr & osr
            pdsr = (pdsr & ~mask) | (new_val & mask)
            self.write_register("PDSR", pdsr)
            data = 0
            diff = val ^ new_val
            for i in range(32):
                if diff & 0x01:
                    logger.debug(f"[{name_:16s}]: Set P{self.NAME[-1]}{i}")
                    for hooks in self._hook[i]:
                        if hooks[0]:
                            _ = hooks[0]()
                diff >>= 1
        elif name == "CODR":
            reg = "ODSR"
            val = self.read_register(reg)
            new_val = val & ~data
            self.write_register(reg, new_val)
            # update PDSR
            psr = self.read_register("PSR")
            osr = self.read_register("OSR")
            pdsr = self.read_register("PDSR")
            mask = psr & osr
            pdsr = (pdsr & ~mask) | (new_val & mask)
            self.write_register("PDSR", pdsr)
            data = 0
            diff = val ^ new_val
            for i in range(32):
                if diff & 0x01:
                    logger.debug(f"[{name_:16s}]: Reset P{self.NAME[-1]}{i}")
                    for hooks in self._hook[i]:
                        if hooks[1]:
                            _ = hooks[1]()
                diff >>= 1
        elif name == "ODSR":
            owsr = self.read_register("OWSR")
            data = (data & owsr) | (data_orig & ~owsr)
            # update PDSR
            psr = self.read_register("PSR")
            osr = self.read_register("OSR")
            pdsr = self.read_register("PDSR")
            mask = psr & osr
            pdsr = (pdsr & ~mask) | (data & mask)
            self.write_register("PDSR", pdsr)
            diff = data ^ data_orig
            data_ = data
            for i in range(32):
                if diff & 0x01:
                    if data_ & 0x01:
                        logger.debug(f"[{name_:16s}]: Set P{self.NAME[-1]}{i}")
                    else:
                        logger.debug(f"[{name_:16s}]: Reset P{self.NAME[-1]}{i}")
                diff >>= 1
                data_ >>= 1
        elif name == "DIFSR":
            reg = "IFDGSR"
            val = self.read_register(reg)
            val |= data
            self.write_register(reg, val)
            data = 0
        elif name == "SCIFSR":
            reg = "IFDGSR"
            val = self.read_register(reg)
            val &= ~data
            self.write_register(reg, val)
            data = 0
        elif name == "LSR":
            reg = "ELSR"
            val = self.read_register(reg)
            val |= data
            self.write_register(reg, val)
            data = 0
        elif name == "ESR":
            reg = "ELSR"
            val = self.read_register(reg)
            val &= ~data
            self.write_register(reg, val)
            data = 0
        elif name == "REHLSR":
            reg = "FRLHSR"
            val = self.read_register(reg)
            val |= data
            self.write_register(reg, val)
            data = 0
        elif name == "FELLSR":
            reg = "FRLHSR"
            val = self.read_register(reg)
            val &= ~data
            self.write_register(reg, val)
            data = 0
        elif name == "WPMR":
            if (data & 0xFFFFFF00) != 0x50494F00:
                logger.warning(f"[{name_:16s}]: Invalid WPKEY, 0x{data:08x}")
                data = data_orig
            else:
                data &= 0x01
        return data

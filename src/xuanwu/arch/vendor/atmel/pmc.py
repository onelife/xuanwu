# -*- coding: utf-8 -*-

"""Power management controller (clock tree and peripheral clocks)."""

from enum import IntEnum
from typing import Any

from ....config import logger
from ...base import ArmHardwareBase, Register
from .common import PID

__all__ = ["ArmSamPmc"]


class PMC_MOR(IntEnum):
    MOSCXTEN = 0
    MOSCXTBY = 1
    MOSCRCEN = 3
    MOSCSEL = 24
    CFDEN = 25


class PMC_SR(IntEnum):
    MOSCXTS = 0
    LOCKA = 1
    MCKRDY = 3
    LOCKU = 6
    MOSCSELS = 16
    MOSCRCS = 17
    CFDEV = 18
    CFDS = 19
    FOS = 20


class PMC_PLLAR(IntEnum):
    ONE = 29


class PMC_UCKR(IntEnum):
    UPLLEN = 16


class MCKR_CSS(IntEnum):
    SLOW_CLK = 0
    MAIN_CLK = 1
    PLLA_CLK = 2
    UPLL_CLK = 3


class ArmSamPmc(ArmHardwareBase):
    """Power management controller"""

    NAME = "PMC"
    REGISTERS = (
        ("SCER", "I", 0xFFFFFFFF),
        ("SCDR", "I", 0xFFFFFFFF),
        ("SCSR", "I", 0x00000000),
        ("RESERVED0", "I", 0x00000000),
        ("PCER0", "I", 0xFFFFFFFC),
        ("PCDR0", "I", 0xFFFFFFFC),
        ("PCSR0", "I", 0x00000000),
        ("UCKR", "I", 0xFFFFFFFF),
        ("MOR", "I", 0xFFFFFFFF),
        ("MCFR", "I", 0x00000000),
        ("PLLAR", "I", 0xFFFFFFFF),
        ("RESERVED1", "I", 0x00000000),
        ("MCKR", "I", 0xFFFFFFFF),
        ("RESERVED2", "I", 0x00000000),
        ("USB", "I", 0xFFFFFFFF),
        ("RESERVED3", "I", 0x00000000),
        ("PCK0", "I", 0xFFFFFFFF),
        ("PCK1", "I", 0xFFFFFFFF),
        ("PCK2", "I", 0xFFFFFFFF),
        ("RESERVED4", "5I", 0x00000000),
        ("IER", "I", 0xFFFFFFFF),
        ("IDR", "I", 0xFFFFFFFF),
        ("SR", "I", 0x00000000),
        ("IMR", "I", 0x00000000),
        ("FSMR", "I", 0xFFFFFFFF),
        ("FSPR", "I", 0xFFFFFFFF),
        ("FOCR", "I", 0xFFFFFFFF),
        ("RESERVED5", "26I", 0x00000000),
        ("WPMR", "I", 0xFFFFFFFF),
        ("WPSR", "I", 0x00000000),
        ("RESERVED6", "5I", 0x00000000),
        ("PCER1", "I", 0xFFFFFFFF),
        ("PCDR1", "I", 0xFFFFFFFF),
        ("PCSR1", "I", 0x00000000),
        ("PCR", "I", 0x00000000),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write

    def reset(self):
        super().reset()
        self.write_register("SCSR", 0x00000001)
        self.write_register("PCSR0", 0x00000000)
        self.write_register("UCKR", 0x10200800)
        self.write_register("MOR", 0x00000001)
        self.write_register("MCFR", 0x000107D0)  # main: 4M, slow: 32K
        self.write_register("PLLAR", 0x00003F00)
        self.write_register("MCKR", 0x00000001)
        self.write_register("USB", 0x00000000)
        self.write_register("PCK0", 0x00000000)
        self.write_register("PCK1", 0x00000000)
        self.write_register("PCK2", 0x00000000)
        self.write_register("SR", 0x00000008)
        self.write_register("IMR", 0x00000000)
        self.write_register("FSMR", 0x00000000)
        self.write_register("FSPR", 0x00000000)
        self.write_register("WPMR", 0x00000000)
        self.write_register("WPSR", 0x00000000)
        self.write_register("PCSR1", 0x00000000)
        self.write_register("PCR", 0x00000000)

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        if name == "SR":
            data_ = data
            if data_ & (1 << PMC_SR.MOSCSELS):
                # clear MOSCSELS for next read
                data_ &= ~(1 << PMC_SR.MOSCSELS)
                self.write_register(name, data_)
            if data_ & (1 << PMC_SR.MOSCRCS) == 0:
                mor = self.read_register("MOR")
                if mor & (1 << PMC_MOR.MOSCRCEN):
                    # set MOSCRCEN for next read
                    data_ |= (1 << PMC_SR.MOSCRCS)
                    self.write_register(name, data_)
            if data_ & (1 << PMC_SR.MCKRDY) == 0:
                mckr = self.read_register("MCKR")
                css = mckr & 0x03
                # set MCKRDY for next read
                if css == MCKR_CSS.PLLA_CLK:
                    if data & (1 << PMC_SR.LOCKA):
                        data_ |= (1 << PMC_SR.MCKRDY)
                        self.write_register(name, data_)
                elif css == MCKR_CSS.UPLL_CLK:
                    if data & (1 << PMC_SR.LOCKU):
                        data_ |= (1 << PMC_SR.MCKRDY)
                        self.write_register(name, data_)
                else:
                    data_ |= (1 << PMC_SR.MCKRDY)
                    self.write_register(name, data_)
        # elif name == "IMR":
        #     self.write_register(name, 0)
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name in ["SCER", "PCER0", "IER", "PCER1"]:
            wpmr = self.read_register("WPMR")
            if wpmr == 0x0:
                # update status register
                if name == "IER":
                    reg = "SR"
                elif name[-1] in "1234567890":
                    reg = name[:-3] + "S" + name[-2:]
                else:
                    reg = name[:-2] + "S" + name[-1:]
                val = self.read_register(reg)
                new_val = val | data
                self.write_register(reg, new_val)
                # update PCKRDYx
                if name == "SCER" and (data & 0x00000700):
                    sr = self.read_register("SR")
                    sr |= data & 0x00000700
                    self.write_register("SR", sr)
                elif name.startswith("PCER"):
                    pid = 32 if name == "PCER1" else 0
                    diff = (val ^ new_val) & data
                    for pid_ in range(32):
                        if diff & 0x01:
                            logger.debug(f"[{name_:16s}]: Enable {PID(pid + pid_).name}")
                        diff >>= 1
            else:
                logger.warning(f"[{name_:16s}]: Ignore write")
            data = 0
        elif name in ["SCDR", "PCDR0", "IDR", "PCDR1"]:
            wpmr = self.read_register("WPMR")
            if wpmr == 0x0:
                # update status register
                reg = name
                if name == "IDR":
                    reg = "SR"
                elif name[-1] in "1234567890":
                    reg = name[:-3] + "S" + name[-2:]
                else:
                    reg = name[:-2] + "S" + name[-1:]
                val = self.read_register(reg)
                new_val = val & ~data
                self.write_register(reg, new_val)
                # update PCKRDYx
                if name == "SCDR" and (data & 0x00000700):
                    sr = self.read_register("SR")
                    sr &= ~(data & 0x00000700)
                    self.write_register("SR", sr)
                elif name.startswith("PCER"):
                    pid = 32 if name == "PCER1" else 0
                    diff = (val ^ new_val) & data
                    for pid_ in range(32):
                        if diff & 0x01:
                            logger.debug(f"[{name_:16s}]: Disable {PID(pid + pid_).name}")
                        diff >>= 1
            else:
                logger.warning(f"[{name_:16s}]: Ignore write")
            data = 0
        elif name in ["UCKR", "MOR", "PLLAR", "MCKR", "USB", "PCK0", "PCK1", "PCK2", "FSMR", "FSPR"]:
            wpmr = self.read_register("WPMR")
            if wpmr == 0x0:
                if name == "MOR":
                    if (data & 0x00FF0000) != 0x00370000:
                        logger.warning(f"[{name_:16s}]: Invalid KEY, 0x{data:08x}")
                        data = data_orig
                    else:
                        data &= ~0x00FF0000
                        sr = self.read_register("SR")
                        if data & (1 << PMC_MOR.MOSCXTEN):
                            data &= ~(1 << PMC_MOR.MOSCXTBY)
                            sr |= 1 << PMC_SR.MOSCXTS
                        else:
                            sr &= ~(1 << PMC_SR.MOSCXTS)
                        if data & (1 << PMC_MOR.MOSCXTBY):
                            sr |= (1 << PMC_SR.MOSCXTS)
                        if data & (1 << PMC_MOR.MOSCRCEN) or (data ^ data_orig) & 0x000000F0:
                            # delay set
                            sr &= ~(1 << PMC_SR.MOSCRCS)
                        if data & (1 << PMC_MOR.MOSCSEL):
                            # delay clear
                            sr |= 1 << PMC_SR.MOSCSELS
                        if data & (1 << PMC_MOR.CFDEN):
                            sr |= 1 << PMC_SR.FOS
                        else:
                            sr &= ~(1 << PMC_SR.FOS)
                        self.write_register("SR", sr)
                elif name == "MCKR":
                    if (data ^ data_orig) & 0x000000FF:
                        sr = self.read_register("SR")
                        # delay set
                        sr &= ~(1 << PMC_SR.MCKRDY)
                        self.write_register("SR", sr)
                elif name == "PLLAR":
                    if data & (1 << PMC_PLLAR.ONE) == 0:
                        logger.warning(f"[{name_:16s}]: Invalid ONE, 0x{data:08x}")
                        data = data_orig
                    else:
                        data &= ~(1 << PMC_PLLAR.ONE)
                    sr = self.read_register("SR")
                    if data & 0xFFFF0000:
                        sr |= 1 << PMC_SR.LOCKA
                    else:
                        sr &= ~(1 << PMC_SR.LOCKA)
                    self.write_register("SR", sr)
                if name == "UCKR":
                    sr = self.read_register("SR")
                    if data & (1 << PMC_UCKR.UPLLEN):
                        sr |= 1 << PMC_SR.LOCKU
                    else:
                        sr &= ~(1 << PMC_SR.LOCKU)
                    self.write_register("SR", sr)
            else:
                data = data_orig
                logger.warning(f"[{name_:16s}]: Ignore write")
        elif name in ["FOCR"]:
            data = 0
        elif name == "WPMR":
            if (data & 0xFFFFFF00) != 0x504D4300:
                logger.warning(f"[{name_:16s}]: Invalid WPKEY, 0x{data:08x}")
                data = data_orig
            else:
                data &= 0x01
        elif name == "PCR":
            pid = data & 0x3F
            cmd = (data >> 12) & 0x01
            div = (data >> 16) & 0x03
            en = (data >> 28) & 0x01
            if pid not in list(PID):
                logger.warning(f"[{name_:16s}]: Invalid PID, 0x{data:08x}")
            elif cmd and div and pid not in [PID.CAN0, PID.CAN1]:
                logger.warning(f"[{name_:16s}]: {PID(pid).name} doesn't support set DIV, 0x{data:08x}")
            elif cmd:
                # write
                if pid >= 32:
                    reg = "PCSR1"
                    pid_ = pid - 32
                else:
                    reg = "PCSR0"
                    pid_ = pid
                val = self.read_register(reg)
                if en:
                    new_val = val | (1 << pid_)
                else:
                    new_val = val & ~(1 << pid_)
                if val != new_val:
                    logger.debug(f"[{name_:16s}]: {'Enable' if en else 'Disable'} {PID(pid).name}")
                    self.write_register(reg, new_val)
                if div:
                    # TODO: CAN
                    pass
            else:
                # read
                if pid >= 32:
                    reg = "PCSR1"
                    pid_ = pid - 32
                else:
                    reg = "PCSR0"
                    pid_ = pid
                val = self.read_register(reg)
                if val & (1 << pid_):
                    data |= 1 << 28
                else:
                    data &= ~(1 << 28)
                if pid in [PID.CAN0, PID.CAN1]:
                    # TODO: CAN
                    pass
        return data

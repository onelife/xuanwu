# -*- coding: utf-8 -*-

from enum import IntEnum
import os
from typing import Any

from unicorn import Uc
# from unicorn.arm_const import *
from serial import Serial

from ..config import logger
from .armv7m import Register, ArmHardwareBase


__all__ = ["BUILDIN"]  # , "ArmSamPmc", "ArmSamGpio", "ArmSamAdc", "ArmSamUart", "ArmSamPwm", "ArmSamUotghs"]


class PID(IntEnum):
    # SUPC = 0
    # RSTC = 1
    RTC = 2
    RTT = 3
    WDG = 4
    PMC = 5
    EEFC0 = 6
    EEFC1 = 7
    UART = 8
    SMC_SDRAMC = 9
    SDRAMC = 10
    PIOA = 11
    PIOB = 12
    PIOC = 13
    PIOD = 14
    PIOE = 15
    PIOF = 16
    USART0 = 17
    USART1 = 18
    USART2 = 19
    USART3 = 20
    HSMCI = 21
    TWI0 = 22
    TWI1 = 23
    SPI0 = 24
    SPI1 = 25
    SSC = 26
    TC0 = 27
    TC1 = 28
    TC2 = 29
    TC3 = 30
    TC4 = 31
    TC5 = 32
    TC6 = 33
    TC7 = 34
    TC8 = 35
    PWM = 36
    ADC = 37
    DACC = 38
    DMAC = 39
    UOTGHS = 40
    TRNG = 41
    EMAC = 42
    CAN0 = 43
    CAN1 = 44


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


class UART_CR(IntEnum):
    RSTRX = 2
    RSTTX = 3
    RXEN = 4
    RXDIS = 5
    TXEN = 6
    TXDIS = 7
    RSTSTA = 8


class UART_SR(IntEnum):
    RXRDY = 0
    TXRDY = 1
    ENDRX = 3
    ENDTX = 4
    OVRE = 5
    FRAME = 6
    PARE = 7
    TXEMPTY = 9
    TXBUFE = 11
    RXBUFF = 12


class ArmSamUart(ArmHardwareBase):
    """Universal Asynchronous Receiver Transceiver"""

    NAME = "UART"
    REGISTERS = (
        ("CR", "I", 0xFFFFFFFF),
        ("MR", "I", 0xFFFFFFFF),
        ("IER", "I", 0xFFFFFFFF),
        ("IDR", "I", 0xFFFFFFFF),
        ("IMR", "I", 0x00000000),
        ("SR", "I", 0x00000000),
        ("RHR", "I", 0x00000000),
        ("THR", "I", 0xFFFFFFFF),
        ("BRGR", "I", 0xFFFFFFFF),
    )
    COMMAND = "socat -d -d pty,link={uart},raw,echo=0 pty,link={tty},raw,echo=0"

    def __init__(self, *args, **kwargs: Any) -> None:
        from tempfile import mkstemp
        import shlex
        from subprocess import Popen, PIPE, TimeoutExpired

        super().__init__(*args, **kwargs)
        self._irq = PID.UART
        self._last_rx = 0
        self._last_rx_new = False
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write
        # ref: https://stackoverflow.com/questions/2291772/virtual-serial-device-in-python
        uart_ = mkstemp(prefix=b"uart_")
        tty_ = mkstemp(prefix=b"tty_")
        os.close(uart_[0])
        os.close(tty_[0])
        self._uart = uart_[1].decode("utf-8")
        self._tty = tty_[1].decode("utf-8")
        self._proc = Popen(shlex.split(self.COMMAND.format(uart=self._uart, tty=self._tty)), stdout=PIPE, stderr=PIPE)
        try:
            _ = self._proc.communicate(timeout=1)
        except TimeoutExpired:
            pass
        self._serial = Serial(self._uart, kwargs.get("baudrate", 115200))
        logger.info(f"Serial device (UART): {self._tty}")

    def __del__(self):
        self._proc.kill()
        os.remove(self._uart)
        os.remove(self._tty)

    def reset(self):
        super().reset()
        self.write_register("MR", 0x00000000)
        self.write_register("IMR", 0x00000000)
        self.write_register("SR", 0x00000000)
        self.write_register("RHR", 0x00000000)
        self.write_register("BRGR", 0x00000000)

    @property
    def tty_device(self):
        return self._tty

    def system_clock_callback(self, box: Uc, address: int, size: int, user_data: Any) -> None:
        cr = self.read_register("CR")
        sr = data = self.read_register("SR")
        if self._serial.in_waiting > 0:
            if cr & (1 << UART_CR.RXEN):
                if not self._last_rx_new:
                    # self._last_rx = int.from_bytes(self._serial.read(self._serial.in_waiting + 10)[-1], "little")
                    rx = self._serial.read(1)
                    self._last_rx = int.from_bytes(rx, "little")
                    # logger.debug(f"UART RX: {rx}")
                    self._last_rx_new = True
                data |= 1 << UART_SR.RXRDY
            else:
                _ = self._serial.read(self._serial.in_waiting + 128)
        # if self._serial.in_waiting > 1:
        #     sr |= 1 << UART_SR.OVRE
        if cr & (1 << UART_CR.TXEN):
            # if self._serial.out_waiting <= 1:
            #     sr |= 1 << UART_SR.TXRDY
            data |= 1 << UART_SR.TXRDY
        if sr != data:
            self.write_register("SR", data)
        imr = self.read_register("IMR")
        if imr & data and not self._ctl.is_irq_pending_or_active(self._irq):
            self._ctl.set_irq_pending(self._irq)

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        # name_ = ".".join([self.NAME, name])
        if name == "RHR":
            sr = data_ = self.read_register("SR")
            data_ &= ~(1 << UART_SR.RXRDY)
            if sr != data_:
                self.write_register("SR", data_)
            data = self._last_rx
            self._last_rx_new = False
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name == "CR":
            sr = self.read_register("SR")
            sr_orig = sr
            if data & ((1 << UART_CR.RSTRX) | (1 << UART_CR.RXDIS)):
                sr &= ~(1 << UART_SR.RXRDY)
                if data & (1 << UART_CR.RSTRX):
                    self._serial.reset_input_buffer()
            if data & ((1 << UART_CR.RSTTX) | (1 << UART_CR.TXDIS)):
                sr &= ~(1 << UART_SR.TXRDY)
                if data & (1 << UART_CR.RSTTX):
                    self._serial.reset_output_buffer()
            if data & (1 << UART_CR.RSTSTA):
                sr &= ~((1 << UART_SR.OVRE) | (1 << UART_SR.FRAME) | (1 << UART_SR.PARE))
            self.write_register("SR", sr)
            diff = sr_orig ^ sr
            if diff & (1 << UART_SR.RXRDY):
                if sr & (1 << UART_SR.RXRDY):
                    logger.debug(f"[{name_:16s}]: Enable RX")
                else:
                    logger.debug(f"[{name_:16s}]: Disable RX")
            if diff & (1 << UART_SR.TXRDY):
                if sr & (1 << UART_SR.TXRDY):
                    logger.debug(f"[{name_:16s}]: Enable TX")
                else:
                    logger.debug(f"[{name_:16s}]: Disable TX")
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
        elif name == "THR":
            cr = self.read_register("CR")
            if cr & (1 << UART_CR.TXEN):
                # if self._serial.out_waiting <= 1:
                #     self._serial.write(data.to_bytes(1, "little"))
                self._serial.write((data & 0xFF).to_bytes(1, "little"))
                # logger.debug(f'[{name_:16s}]: Output "{(data & 0xFF).to_bytes(1, "little")}"')
        return data


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
    COMMAND = "socat -d -d pty,link={spi},raw,echo=0 pty,link={tty},raw,echo=0"

    def __init__(self, *args, **kwargs: Any) -> None:
        from tempfile import mkstemp
        import shlex
        from subprocess import Popen, PIPE, TimeoutExpired

        super().__init__(*args, **kwargs)
        self._npcs = 0xf
        self._last_rx = 0
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write
        # ref: https://stackoverflow.com/questions/2291772/virtual-serial-device-in-python
        spi_ = mkstemp(prefix=b"spi_")
        tty_ = mkstemp(prefix=b"tty_")
        os.close(spi_[0])
        os.close(tty_[0])
        self._spi = spi_[1].decode("utf-8")
        self._tty = tty_[1].decode("utf-8")
        self._proc = Popen(shlex.split(self.COMMAND.format(spi=self._spi, tty=self._tty)), stdout=PIPE, stderr=PIPE)
        try:
            _ = self._proc.communicate(timeout=1)
        except TimeoutExpired:
            pass
        self._serial = Serial(self._spi, kwargs.get("baudrate", 115200))
        logger.info(f"Serial device (SPI): {self._tty}")

    def __del__(self):
        self._proc.kill()
        os.remove(self._spi)
        os.remove(self._tty)

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
    def tty_device(self):
        return self._tty

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        # name_ = ".".join([self.NAME, name])
        if name == "SR":
            sr = data
            if self._serial.in_waiting > 0:
                sr |= 1 << SPI_SR.RDRF
            sr |= 1 << SPI_SR.TXEMPTY
            if sr != data:
                self.write_register("SR", data)
            if self._serial.in_waiting > 1:
                data |= 1 << SPI_SR.OVRES
        elif name == "RDR":
            if self._serial.in_waiting > 0:
                # self._last_rx = int.from_bytes(self._serial.read(self._serial.in_waiting + 10)[-1], "little")
                self._last_rx = int.from_bytes(self._serial.read(1), "little")
            data = self._last_rx
            mr = self.read_register("MR")
            if mr & (1 << SPI_MR.SPI_MR):
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
            self._serial.write((data & 0xFF).to_bytes(1, "little"))
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


class ArmSamPwm(ArmHardwareBase):
    """Pulse Width Modulation"""
    # TODO: CLK selection?

    NAME = "PWM"
    REGISTERS = (
        ("CLK", "I", 0xFFFFFFFF),
        ("ENA", "I", 0xFFFFFFFF),
        ("DIS", "I", 0xFFFFFFFF),
        ("SR", "I", 0x00000000),
        ("IER1", "I", 0xFFFFFFFF),
        ("IDR1", "I", 0xFFFFFFFF),
        ("IMR1", "I", 0x00000000),
        ("ISR1", "I", 0x00000000),
        ("SCM", "I", 0xFFFFFFFF),
        ("RESERVED0", "I", 0x00000000),
        ("SCUC", "I", 0xFFFFFFFF),
        ("SCUP", "I", 0xFFFFFFFF),
        ("SCUPUPD", "I", 0xFFFFFFFF),
        ("IER2", "I", 0xFFFFFFFF),
        ("IDR2", "I", 0xFFFFFFFF),
        ("IMR2", "I", 0x00000000),
        ("ISR2", "I", 0x00000000),
        ("OOV", "I", 0xFFFFFFFF),
        ("OS", "I", 0xFFFFFFFF),
        ("OSS", "I", 0xFFFFFFFF),
        ("OSC", "I", 0xFFFFFFFF),
        ("OSSUPD", "I", 0xFFFFFFFF),
        ("OSCUPD", "I", 0xFFFFFFFF),
        ("FMR", "I", 0xFFFFFFFF),
        ("FSR", "I", 0x00000000),
        ("FCR", "I", 0xFFFFFFFF),
        ("FPV", "I", 0xFFFFFFFF),
        ("FPE1", "I", 0xFFFFFFFF),
        ("FPE2", "I", 0xFFFFFFFF),
        ("RESERVED1", "2I", 0x00000000),
        ("ELMR0", "I", 0xFFFFFFFF),
        ("ELMR1", "I", 0xFFFFFFFF),
        ("RESERVED2", "11I", 0x00000000),
        ("SMMR", "I", 0xFFFFFFFF),
        ("RESERVED3", "12I", 0x00000000),
        ("WPCR", "I", 0xFFFFFFFF),
        ("WPSR", "I", 0x00000000),
        ("RESERVED4", "17I", 0x00000000),
        ("CMPV0", "I", 0xFFFFFFFF),
        ("CMPVUPD0", "I", 0xFFFFFFFF),
        ("CMPM0", "I", 0xFFFFFFFF),
        ("CMPMUPD0", "I", 0xFFFFFFFF),
        ("CMPV1", "I", 0xFFFFFFFF),
        ("CMPVUPD1", "I", 0xFFFFFFFF),
        ("CMPM1", "I", 0xFFFFFFFF),
        ("CMPMUPD1", "I", 0xFFFFFFFF),
        ("CMPV2", "I", 0xFFFFFFFF),
        ("CMPVUPD2", "I", 0xFFFFFFFF),
        ("CMPM2", "I", 0xFFFFFFFF),
        ("CMPMUPD2", "I", 0xFFFFFFFF),
        ("CMPV3", "I", 0xFFFFFFFF),
        ("CMPVUPD3", "I", 0xFFFFFFFF),
        ("CMPM3", "I", 0xFFFFFFFF),
        ("CMPMUPD3", "I", 0xFFFFFFFF),
        ("CMPV4", "I", 0xFFFFFFFF),
        ("CMPVUPD4", "I", 0xFFFFFFFF),
        ("CMPM4", "I", 0xFFFFFFFF),
        ("CMPMUPD4", "I", 0xFFFFFFFF),
        ("CMPV5", "I", 0xFFFFFFFF),
        ("CMPVUPD5", "I", 0xFFFFFFFF),
        ("CMPM5", "I", 0xFFFFFFFF),
        ("CMPMUPD5", "I", 0xFFFFFFFF),
        ("CMPV6", "I", 0xFFFFFFFF),
        ("CMPVUPD6", "I", 0xFFFFFFFF),
        ("CMPM6", "I", 0xFFFFFFFF),
        ("CMPMUPD6", "I", 0xFFFFFFFF),
        ("CMPV7", "I", 0xFFFFFFFF),
        ("CMPVUPD7", "I", 0xFFFFFFFF),
        ("CMPM7", "I", 0xFFFFFFFF),
        ("CMPMUPD7", "I", 0xFFFFFFFF),
        ("RESERVED5", "20I", 0x00000000),
        ("CMR0", "I", 0xFFFFFFFF),
        ("CDTY0", "I", 0xFFFFFFFF),
        ("CDTYUPD0", "I", 0xFFFFFFFF),
        ("CPRD0", "I", 0x0000FFFF),
        ("CPRDUPD0", "I", 0x0000FFFF),
        ("CCNT0", "I", 0x00000000),
        ("DT0", "I", 0xFFFFFFFF),
        ("DTUPD0", "I", 0xFFFFFFFF),
        ("CMR1", "I", 0xFFFFFFFF),
        ("CDTY1", "I", 0xFFFFFFFF),
        ("CDTYUPD1", "I", 0xFFFFFFFF),
        ("CPRD1", "I", 0x0000FFFF),
        ("CPRDUPD1", "I", 0x0000FFFF),
        ("CCNT1", "I", 0x00000000),
        ("DT1", "I", 0xFFFFFFFF),
        ("DTUPD1", "I", 0xFFFFFFFF),
        ("CMR2", "I", 0xFFFFFFFF),
        ("CDTY2", "I", 0xFFFFFFFF),
        ("CDTYUPD2", "I", 0xFFFFFFFF),
        ("CPRD2", "I", 0x0000FFFF),
        ("CPRDUPD2", "I", 0x0000FFFF),
        ("CCNT2", "I", 0x00000000),
        ("DT2", "I", 0xFFFFFFFF),
        ("DTUPD2", "I", 0xFFFFFFFF),
        ("CMR3", "I", 0xFFFFFFFF),
        ("CDTY3", "I", 0xFFFFFFFF),
        ("CDTYUPD3", "I", 0xFFFFFFFF),
        ("CPRD3", "I", 0x0000FFFF),
        ("CPRDUPD3", "I", 0x0000FFFF),
        ("CCNT3", "I", 0x00000000),
        ("DT3", "I", 0xFFFFFFFF),
        ("DTUPD3", "I", 0xFFFFFFFF),
        ("CMR4", "I", 0xFFFFFFFF),
        ("CDTY4", "I", 0xFFFFFFFF),
        ("CDTYUPD4", "I", 0xFFFFFFFF),
        ("CPRD4", "I", 0x0000FFFF),
        ("CPRDUPD4", "I", 0x0000FFFF),
        ("CCNT4", "I", 0x00000000),
        ("DT4", "I", 0xFFFFFFFF),
        ("DTUPD4", "I", 0xFFFFFFFF),
        ("CMR5", "I", 0xFFFFFFFF),
        ("CDTY5", "I", 0xFFFFFFFF),
        ("CDTYUPD5", "I", 0xFFFFFFFF),
        ("CPRD5", "I", 0x0000FFFF),
        ("CPRDUPD5", "I", 0x0000FFFF),
        ("CCNT5", "I", 0x00000000),
        ("DT5", "I", 0xFFFFFFFF),
        ("DTUPD5", "I", 0xFFFFFFFF),
        ("CMR6", "I", 0xFFFFFFFF),
        ("CDTY6", "I", 0xFFFFFFFF),
        ("CDTYUPD6", "I", 0xFFFFFFFF),
        ("CPRD6", "I", 0x0000FFFF),
        ("CPRDUPD6", "I", 0x0000FFFF),
        ("CCNT6", "I", 0x00000000),
        ("DT6", "I", 0xFFFFFFFF),
        ("DTUPD6", "I", 0xFFFFFFFF),
        ("CMR7", "I", 0xFFFFFFFF),
        ("CDTY7", "I", 0xFFFFFFFF),
        ("CDTYUPD7", "I", 0xFFFFFFFF),
        ("CPRD7", "I", 0x0000FFFF),
        ("CPRDUPD7", "I", 0x0000FFFF),
        ("CCNT7", "I", 0x00000000),
        ("DT7", "I", 0xFFFFFFFF),
        ("DTUPD7", "I", 0xFFFFFFFF),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._step = kwargs.get("step", 1)
        self._ccnt = [0] * 8
        self._cprd = [0] * 8
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write

    def reset(self):
        super().reset()
        self.write_register("CLK", 0x0)
        self.write_register("SR", 0x0)
        self.write_register("IMR1", 0x0)
        self.write_register("ISR1", 0x0)
        self.write_register("SCM", 0x0)
        self.write_register("SCUC", 0x0)
        self.write_register("SCUP", 0x0)
        self.write_register("SCUPUPD", 0x0)
        self.write_register("IMR2", 0x0)
        self.write_register("ISR2", 0x0)
        self.write_register("OOV", 0x0)
        self.write_register("OS", 0x0)
        self.write_register("FMR", 0x0)
        self.write_register("FSR", 0x0)
        self.write_register("FPV", 0x0)
        self.write_register("FPE1", 0x0)
        self.write_register("FPE2", 0x0)
        self.write_register("ELMR0", 0x0)
        self.write_register("ELMR1", 0x0)
        self.write_register("SMMR", 0x0)
        self.write_register("WPSR", 0x0)
        self.write_register("CMPV0", 0x0)
        self.write_register("CMPM0", 0x0)
        self.write_register("CMPV1", 0x0)
        self.write_register("CMPM1", 0x0)
        self.write_register("CMPV2", 0x0)
        self.write_register("CMPM2", 0x0)
        self.write_register("CMPV3", 0x0)
        self.write_register("CMPM3", 0x0)
        self.write_register("CMPV4", 0x0)
        self.write_register("CMPM4", 0x0)
        self.write_register("CMPV5", 0x0)
        self.write_register("CMPM5", 0x0)
        self.write_register("CMPV6", 0x0)
        self.write_register("CMPM6", 0x0)
        self.write_register("CMPV7", 0x0)
        self.write_register("CMPM7", 0x0)
        # self.write_register("CMR", 0x0)
        # self.write_register("CDTY", 0x0)
        # self.write_register("CPRD", 0x0)
        # self.write_register("CCNT", 0x0)
        # self.write_register("DT", 0x0)
        self._ccnt = [0] * 8
        self._cprd = [0] * 8

    def system_clock_callback(self, box: Uc, address: int, size: int, user_data: Any) -> None:
        for i in range(8):
            self._ccnt[i] += self._step
            if self._ccnt[i] >= self._cprd[i]:
                self._ccnt[i] = 0
                # TODO: trigger interrupt
                # csr |= 1 << CSR.COUNTFLAG
                # if csr & (1 << CSR.TICKINT):
                #     self._ctl.set_irq_pending(self._irq)
                # self.write_register("CSR", csr)

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        # name_ = ".".join([self.NAME, name])
        if name == "WPSR":
            data_ = data & 0x0000FF7F
            # read to clear
            self.write_register(name, data_)
        # elif name == "LCDR":
        #     logger.debug(f"[{name_:16s}]: Last ADC result, 0x{data & 0xFFF:08x}")
        elif name[:-1] == "CCNT":
            data = self._ccnt[int(name[-1])]
            self.write_register(f"CCNT{name[-1]}", self._ccnt[int(name[-1])])
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name in ["ENA", "IER1", "IER2"]:
            if name == "ENA":
                reg = "SR"
            elif name[-1] in "1234567890":
                reg = name[:-3] + "M" + name[-2:]
            else:
                reg = name[:-2] + "M" + name[-1:]
            val = self.read_register(reg)
            new_val = val | data
            self.write_register(reg, new_val)
            if name == "ENA":
                diff = val ^ new_val
                # 8 channels
                for i in range(8):
                    if diff & 0x01:
                        # clear counter
                        self.write_register(f"CCNT{i}", 0)
                        logger.debug(f"[{name_:16s}]: Enable {self.NAME}{i}")
                    diff >>= 1
            data = 0
        elif name in ["DIS", "IDR1", "IDR2"]:
            if name == "DIS":
                wpsr = self.read_register("WPSR")
                wps1 = (wpsr | (wpsr >> 8)) & 0x02
                if wps1:
                    logger.warning(f"[{name_:16s}]: Ignore write")
                    reg = None
                else:
                    reg = "SR"
            elif name[-1] in "1234567890":
                reg = name[:-3] + "M" + name[-2:]
            else:
                reg = name[:-2] + "M" + name[-1:]
            if reg:
                val = self.read_register(reg)
                new_val = val & ~data
                self.write_register(reg, new_val)
                if name == "DIS":
                    diff = val ^ new_val
                    for i in range(32):
                        if diff & 0x01:
                            logger.debug(f"[{name_:16s}]: Disable {self.NAME}{i}")
                        diff >>= 1
            data = 0
        elif name == "CLK":
            wpsr = self.read_register("WPSR")
            wps0 = (wpsr | (wpsr >> 8)) & 0x01
            if wps0:
                logger.warning(f"[{name_:16s}]: Ignore write")
                data = data_orig
        elif name in ["SCM", "SMMR"] or name[:-1] in ["CMR"]:
            wpsr = self.read_register("WPSR")
            wps2 = (wpsr | (wpsr >> 8)) & 0x04
            if wps2:
                logger.warning(f"[{name_:16s}]: Ignore write")
                data = data_orig
        elif name[:-1] in ["CPRD", "CPRDUPD"]:
            wpsr = self.read_register("WPSR")
            wps3 = (wpsr | (wpsr >> 8)) & 0x08
            if wps3:
                logger.warning(f"[{name_:16s}]: Ignore write")
                data = data_orig
            else:
                self._cprd[int(name[-1])] = data
                if name[:-1] == "CPRDUPD":
                    self.write_register(f"CPRD{name[-1]}", data)
                    data = 0
        elif name[:-1] in ["DT", "DTUPD"]:
            wpsr = self.read_register("WPSR")
            wps4 = (wpsr | (wpsr >> 8)) & 0x10
            if wps4:
                logger.warning(f"[{name_:16s}]: Ignore write")
                data = data_orig
            elif name[:-1] == "DTUPD":
                self.write_register(f"DT{name[-1]}", data)
                data = 0
        elif name in ["FMR", "FPV", "FPE1", "FPE2"]:
            wpsr = self.read_register("WPSR")
            wps5 = (wpsr | (wpsr >> 8)) & 0x20
            if wps5:
                logger.warning(f"[{name_:16s}]: Ignore write")
                data = data_orig
        elif name == "SCUPUPD":
            scup = self.read_register("SCUP")
            scup = (scup & 0xFFFFFFF0) | (data & 0x0000000F)
            self.write_register("SCUP", scup)
            data = 0
        elif name == "WPCR":
            if (data & 0xFFFFFF00) != 0x50574D00:
                logger.warning(f"[{name_:16s}]: Invalid WPKEY, 0x{data:08x}")
                data = data_orig
            else:
                data &= 0xFF
                wpcmd = data & 0x03
                wprg = (data >> 2) & 0x3F
                wpsr = self.read_register("WPSR")
                if wpcmd == 0:
                    wpsr &= ~wprg
                elif wpcmd == 1:
                    wpsr |= wprg
                elif wpcmd == 2:
                    wpsr |= (wprg << 8)
                self.write_register("WPSR", wpsr)
        return data


class UOTGHS_CTRL(IntEnum):
    FRZCLK = 14
    USBE = 15


class UOTGHS_SR(IntEnum):
    CLKUSABLE = 14


class ArmSamUotghs(ArmHardwareBase):
    """USB On-The-Go interface"""

    NAME = "UOTGHS"
    REGISTERS = (
        ("CTRL", "I", 0xFFFFFFFF),
        ("SR", "I", 0x00000000),
        ("SCR", "I", 0xFFFFFFFF),
        ("SFR", "I", 0xFFFFFFFF),
        ("RESERVED0", "8I", 0x00000000),
        ("FSM", "I", 0x00000000),

    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        # self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write

    def reset(self):
        super().reset()
        self.write_register("CTRL", 0x03004000)
        self.write_register("SR", 0x00000400)
        self.write_register("FSM", 0x00000009)

    # def fix_after_read(self, name: str, register: Register, data: int) -> int:
    #     if name == "SR":

    #     return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        # name_ = ".".join([self.NAME, name])
        if name == "CTRL":
            if data & (1 << UOTGHS_CTRL.FRZCLK):
                sr = self.read_register("SR")
                sr |= (1 << UOTGHS_SR.CLKUSABLE)
                self.write_register("SR", sr)
        return data


BUILDIN = {
    "pmc": ArmSamPmc,
    "gpio": ArmSamGpio,
    "adc": ArmSamAdc,
    "pwm": ArmSamPwm,
    "uart": ArmSamUart,
    "spi": ArmSamSpi,
    "uotghs": ArmSamUotghs,
}

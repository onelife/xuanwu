# -*- coding: utf-8 -*-

"""Pulse width modulation controller."""

from typing import Any

from ....config import logger
from ...base import ArmHardwareBase, Register

__all__ = ["ArmSamPwm"]


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

    def advance(self, instructions: int) -> None:
        """Advance the eight channel counters by a whole execution slice."""
        step = self._step * instructions
        for i in range(8):
            self._ccnt[i] += step
            if self._ccnt[i] >= self._cprd[i]:
                self._ccnt[i] %= self._cprd[i] or 1
                # TODO: trigger interrupt

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

# -*- coding: utf-8 -*-

"""SysTick system timer."""

from typing import Any

from unicorn import Uc

from ...config import logger
from ..base import ArmHardwareBase, Register
from .constants import CSR

__all__ = ["ArmHardwareSystick"]


class ArmHardwareSystick(ArmHardwareBase):
    """System timer"""
    # TODO: CLK selection?

    NAME = "SYSTICK"
    REGISTERS = (
        ("CSR", "I", 0x00000007),
        ("RVR", "I", 0x00FFFFFF),
        ("CVR", "I", 0x00FFFFFF),
        ("CALIB", "I", 0x00000000),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._step = kwargs.get("step", 1)  # count down step
        self._calib = kwargs.get("calib", 0x00002904)  # IMPLEMENTATION DEFINED, overridable per chip
        self._cvr = 0
        self._rvr = 0
        self._irq = -1
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write

    def reset(self):
        super().reset()
        self.write_register("CSR", 0x0)
        self.write_register("RVR", 0x0)
        self.write_register("CVR", 0x0)
        # The SysTick calibration value is IMPLEMENTATION DEFINED; override it per chip via YAML.
        self.write_register("CALIB", self._calib)
        self._csr = 0
        self._cvr = 0
        self._rvr = 0

    def system_clock_callback(self, box: Uc, address: int, size: int, user_data: Any) -> None:
        # csr = self.read_register("CSR")
        if self._csr & (1 << CSR.ENABLE) == 0:
            return
        # cvr = self.read_register("CVR")
        self._cvr -= self._step
        if self._cvr <= 0:
            self._cvr = self._rvr
            # cvr = self.read_register("RVR")
            self._csr |= 1 << CSR.COUNTFLAG
            # trigger interrupt
            if self._csr & (1 << CSR.TICKINT):
                self._ctl.set_irq_pending(self._irq)
            self.write_register("CSR", self._csr)
        # self.write_register("CVR", self._cvr)

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        name_ = ".".join([self.NAME, name])
        # csr = self.read_register("CSR")
        if name == "CSR":
            data = self._csr
            # read to clear
            self._csr &= ~(1 << CSR.COUNTFLAG)
            self.write_register("CSR", self._csr)
            logger.debug(f"[{name_:16s}]: Clear CSR.COUNTFLAG")
        elif name == "CVR":
            data = self._cvr
            self.write_register("CVR", self._cvr)
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        # csr = self.read_register("CSR")
        if name == "CSR":
            self._csr = data
            # TODO: CLKSOURCE, If no external clock is provided, this bit reads as 1 and ignores writes.
            pass
        elif name == "CVR":
            # set status, if selected clock
            self._cvr = 0
            data = 0
            self._csr &= ~(1 << CSR.COUNTFLAG)
            self.write_register("CSR", self._csr)
            logger.debug(f"[{name_:16s}]: Clear CVR and CSR.COUNTFLAG")
        elif name == "RVR":
            self._rvr = data
        return data

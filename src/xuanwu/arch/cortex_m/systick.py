# -*- coding: utf-8 -*-

"""SysTick system timer.

Unicorn does not report how many cycles an instruction took, so the counter is
driven one *instruction* at a time and advanced by ``cycles_per_instruction``.
That factor is the model's only handle on instructions-per-cycle: leaving it at
the default of 1 means "one cycle per instruction", which is the honest
approximation, while a larger value makes a firmware's millisecond tick cost
fewer emulated instructions (a speed/fidelity trade-off, not a measurement).

``clock`` is the core frequency (Hz) the time base is derived from. It feeds
``CALIB.TENMS`` and the :attr:`elapsed_ms` accessor, so a firmware that computes
its reload value from the real clock and one that reads ``CALIB`` agree.
"""

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
        ("CSR", "I", 0x00000007),  # COUNTFLAG is read-only
        ("RVR", "I", 0x00FFFFFF),
        ("CVR", "I", 0x00FFFFFF),
        ("CALIB", "I", 0x00000000),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        # ``step`` is the historical name for this factor and is still accepted.
        self._cycles_per_instruction = float(kwargs.get("cycles_per_instruction", kwargs.get("step", 1)))
        self._clock = int(kwargs.get("clock", 1_000_000))  # core clock, Hz
        self._calib = kwargs.get("calib")  # IMPLEMENTATION DEFINED, derived from the clock if absent
        self._cvr = 0
        self._rvr = 0
        self._irq = -1
        self._cycles_left = 0
        self._cycles = 0.0
        self._ticks = 0
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write

    @property
    def cycles(self) -> float:
        """Simulated cycles executed since the last reset."""
        return self._cycles

    @property
    def ticks(self) -> int:
        """Number of times the counter wrapped, that is, SysTick periods elapsed."""
        return self._ticks

    @property
    def elapsed_ms(self) -> float:
        """Simulated time since the last reset, in milliseconds."""
        return self._cycles / self._clock * 1000.0

    @property
    def calib(self) -> int:
        if self._calib is not None:
            return self._calib
        # NOREF = 0 (a reference clock is provided), SKEW = 0, TENMS = one 10 ms
        # period in core cycles.  A clock below 100 Hz cannot express that.
        tenms = max(0, min(self._clock // 100 - 1, 0xFFFFFF))
        return tenms

    def reset(self):
        super().reset()
        self.write_register("CSR", 0x0)
        self.write_register("RVR", 0x0)
        self.write_register("CVR", 0x0)
        self.write_register("CALIB", self.calib)
        self._csr = 0
        self._cvr = 0
        self._rvr = 0
        self._cycles_left = 0
        self._cycles = 0.0
        self._ticks = 0

    def system_clock_callback(self, box: Uc, address: int, size: int, user_data: Any) -> None:
        """Advance the time base by one instruction."""
        self._cycles += self._cycles_per_instruction
        period = self._rvr + 1
        if self._csr & (1 << CSR.ENABLE) == 0:
            # Disabled: the counter is held at the reload value.
            self._cycles_left = period
        else:
            self._cycles_left -= self._cycles_per_instruction
            if self._cycles_left <= 0:
                # A period is RVR + 1 cycles.  Counting the periods in one step
                # keeps this exact (and O(1)) even when one instruction is longer
                # than a whole period; the pending interrupt is a single flag, so
                # missing several periods collapses into one.
                missed = int(-self._cycles_left // period) + 1
                self._cycles_left += missed * period
                self._ticks += missed
                self._csr |= 1 << CSR.COUNTFLAG
                if self._csr & (1 << CSR.TICKINT):
                    self._ctl.set_irq_pending(self._irq)
                self.write_register("CSR", self._csr)
        # CVR is the number of cycles still to go before the counter wraps; the
        # reload value is RVR, so the value just after a wrap reads as RVR.
        self._cvr = min(int(self._cycles_left), self._rvr)

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        name_ = ".".join([self.NAME, name])
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
        if name == "CSR":
            # Bit 16 is the read-only COUNTFLAG; keep whatever the model has.
            self._csr = (self._csr & (1 << CSR.COUNTFLAG)) | (data & ~(1 << CSR.COUNTFLAG))
            data = self._csr
            # TODO: CLKSOURCE, If no external clock is provided, this bit reads as 1 and ignores writes.
            if self._csr & (1 << CSR.ENABLE):
                # Starting the counter loads the reload value on the next clock.
                if self._cycles_left <= 0:
                    self._cycles_left = self._rvr + 1
                    self._cvr = self._rvr
        elif name == "CVR":
            # A write clears the counter; it reloads from RVR and restarts counting.
            self._cvr = 0
            self._cycles_left = self._rvr + 1
            data = 0
            self._csr &= ~(1 << CSR.COUNTFLAG)
            self.write_register("CSR", self._csr)
            logger.debug(f"[{name_:16s}]: Clear CVR and CSR.COUNTFLAG")
        elif name == "RVR":
            self._rvr = data
            if self._cycles_left > self._rvr + 1 or not self._csr & (1 << CSR.ENABLE):
                self._cycles_left = self._rvr + 1
        return data

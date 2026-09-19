# -*- coding: utf-8 -*-

"""Floating-point extension registers.

These sit in the system control space just above the debug registers and tell
software what the FPU can do (``MVFR0``/``MVFR1``) and control how the FP
context is stacked (``FPCCR``/``FPCAR``/``FPDSCR``).

The exception frame itself is handled by
:class:`~xuanwu.arch.cortex_m.controller.ArmHardwareController`; this model is
only the register interface.  Lazy stacking is not implemented, so ``LSPACT``
always reads as zero.
"""

from typing import Any

from ...config import logger
from ..base import ArmHardwareBase, Register

__all__ = ["ArmHardwareFpu"]


FPCCR_RESET = 0xC0000000
"""``ASPEN`` and ``LSPEN`` set, which is what a Cortex-M4 resets to."""

MVFR0_RESET = 0x10110021
MVFR1_RESET = 0x11000011
"""Cortex-M4F feature report: single precision only, no double precision."""


class ArmHardwareFpu(ArmHardwareBase):
    """Floating-point extension registers."""

    NAME = "FPU"
    REGISTERS = (
        ("RESERVED0", "I", 0x00000000),
        ("FPCCR", "I", 0xFFFFFFFE),  # LSPACT is owned by the hardware
        ("FPCAR", "I", 0xFFFFFFF8),
        ("FPDSCR", "I", 0x07000000),
        ("MVFR0", "I", 0x00000000),  # read-only
        ("MVFR1", "I", 0x00000000),
        ("MVFR2", "I", 0x00000000),
    )

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._fpccr = kwargs.get("fpccr", FPCCR_RESET)
        self._mvfr0 = kwargs.get("mvfr0", MVFR0_RESET)
        self._mvfr1 = kwargs.get("mvfr1", MVFR1_RESET)
        self._fix_after_read = self.fix_after_read

    def reset(self):
        super().reset()
        self.write_register("RESERVED0", 0x00000000)
        self.write_register("FPCCR", self._fpccr)
        self.write_register("FPCAR", 0x00000000)
        self.write_register("FPDSCR", 0x00000000)
        self.write_register("MVFR0", self._mvfr0)
        self.write_register("MVFR1", self._mvfr1)
        self.write_register("MVFR2", 0x00000000)

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        if name == "FPCCR":
            # Nothing is ever deferred, so report "no lazy state active".
            data &= ~0x1
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name == "FPCCR":
            if data != data_orig:
                logger.debug(f"[{name_:16s}]: 0x{data_orig:08x} => 0x{data:08x}")
        return data

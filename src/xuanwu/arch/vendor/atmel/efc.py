# -*- coding: utf-8 -*-

"""Flash controller (EEFC0/EEFC1).

Only what firmware touches: the flash mode register that ``SystemInit`` writes to
set the wait states, and a status register that reports "ready" so a polling write
sequence does not spin.  The write/erase sequences themselves are not modelled --
a simulated flash that actually reprogrammed itself would only make
``unclaimed_accesses()`` quieter.
"""

from enum import IntEnum
from typing import Any

from ....config import logger
from ...base import ArmHardwareBase, Register

__all__ = ["ArmSamEfc"]


class EEFC_FSR(IntEnum):
    FRDY = 0
    FCMDE = 1
    FLOCKE = 2
    FLERR = 3


class ArmSamEfc(ArmHardwareBase):
    """Embedded flash controller."""

    NAME = "EFC"
    REGISTERS = (
        ("FMR", "I", 0x00000000),
        ("FCR", "I", 0x00000000),
        ("FSR", "I", 0x00000000),
        ("FRR", "I", 0x00000000),
        ("FVR", "I", 0x00000000),
    )

    FSR_READY = 1 << EEFC_FSR.FRDY | (1 << 9)
    """``FRDY`` plus the field that says the whole flash is ready."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._fix_after_read = self.fix_after_read
        self._fix_before_write = self.fix_before_write

    def reset(self):
        super().reset()
        self.write_register("FMR", 0x00000000)
        self.write_register("FCR", 0x00000000)
        self.write_register("FSR", self.FSR_READY)
        self.write_register("FRR", 0x00000000)
        self.write_register("FVR", 0x00000000)

    def fix_after_read(self, name: str, register: Register, data: int) -> int:
        if name == "FSR":
            # Nothing is ever in progress: a firmware polling FRDY proceeds.
            data = self.FSR_READY
            self.write_register("FSR", data)
        return data

    def fix_before_write(self, name: str, register: Register, data: int, data_orig: int) -> int:
        name_ = ".".join([self.NAME, name])
        if name == "FCR":
            logger.debug(f"[{name_:16s}]: flash command 0x{data:08x} acknowledged")
            self.write_register("FCR", 0x00000000)
            data = 0
        return data

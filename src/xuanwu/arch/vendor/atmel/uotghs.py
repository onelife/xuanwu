# -*- coding: utf-8 -*-

"""USB On-The-Go interface."""

from enum import IntEnum
from typing import Any

from ...base import ArmHardwareBase, Register

__all__ = ["ArmSamUotghs"]


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

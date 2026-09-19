# -*- coding: utf-8 -*-

"""System control and ID registers not held in the SCB."""

from typing import Any

from ..base import ArmHardwareBase

__all__ = ["ArmHardwareScid"]


class ArmHardwareScid(ArmHardwareBase):
    """system control and ID registers not in the SCB"""

    NAME = "SCID"
    REGISTERS = (
        ("RESERVED0", "I", 0x00000000),
        ("ICTR", "I", 0x00000000),
        ("ACTLR", "I", 0x00000307),
        ("RESERVED1", "I", 0x00000000),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._interrupt_lines = kwargs.get("interrupt_lines", 7)

    def reset(self):
        super().reset()
        self.write_register("ICTR", self._interrupt_lines)
        self.write_register("ACTLR", 0x0)

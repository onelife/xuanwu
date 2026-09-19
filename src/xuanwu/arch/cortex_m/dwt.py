# -*- coding: utf-8 -*-

"""Data watchpoint and trace unit."""

from typing import Any

from ..base import ArmHardwareBase

__all__ = ["ArmHardwareDwt"]


class ArmHardwareDwt(ArmHardwareBase):
    """data watchpoint trigger"""

    NAME = "DWT"
    REGISTERS = (
        ("CTRL", "I", 0xFF7F1FFF),
        ("CYCCNT", "I", 0xFFFFFFFF),
        ("CPICNT", "I", 0x000000FF),
        ("EXCCNT", "I", 0x000000FF),
        ("SLEEPCNT", "I", 0x000000FF),
        ("LSUCNT", "I", 0x000000FF),
        ("FOLDCNT", "I", 0x000000FF),
        ("PCSR", "I", 0x00000000),
        ("COMP0", "I", 0xFFFFFFFF),
        ("MASK0", "I", 0x0000001F),
        ("FUNCTION0", "I", 0x010FFFAF),
        ("RESERVED0", "I", 0x00000000),
        ("COMP1", "I", 0xFFFFFFFF),
        ("MASK1", "I", 0x0000001F),
        ("FUNCTION1", "I", 0x010FFF2F),
        ("RESERVED1", "I", 0x00000000),
        ("COMP2", "I", 0xFFFFFFFF),
        ("MASK2", "I", 0x0000001F),
        ("FUNCTION2", "I", 0x010FFF2F),
        ("RESERVED2", "I", 0x00000000),
        ("COMP3", "I", 0xFFFFFFFF),
        ("MASK3", "I", 0x0000001F),
        ("FUNCTION3", "I", 0x010FFF2F),
        ("RESERVED3", "989I", 0x00000000),
        ("PID4", "I", 0x00000000),
        ("PID5", "I", 0x00000000),
        ("PID6", "I", 0x00000000),
        ("PID7", "I", 0x00000000),
        ("PID0", "I", 0x00000000),
        ("PID1", "I", 0x00000000),
        ("PID2", "I", 0x00000000),
        ("PID3", "I", 0x00000000),
        ("CID0", "I", 0x00000000),
        ("CID1", "I", 0x00000000),
        ("CID2", "I", 0x00000000),
        ("CID3", "I", 0x00000000),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

    def reset(self):
        super().reset()
        # unknown reset value
        self.write_register("CTRL", 0x0F000000)

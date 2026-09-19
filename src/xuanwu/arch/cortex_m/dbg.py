# -*- coding: utf-8 -*-

"""Debug system registers."""

from typing import Any

from ..base import ArmHardwareBase

__all__ = ["ArmHardwareDbg"]


class ArmHardwareDbg(ArmHardwareBase):
    """Debug system"""

    NAME = "DBG"
    REGISTERS = (
        ("DHCSR", "I", 0xFFFF002F),
        ("DCRSR", "I", 0x0001007F),
        ("DCRDR", "I", 0xFFFFFFFF),
        ("DEMCR", "I", 0x010F07F1),
    )

    def __init__(self, *args, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

    def reset(self):
        super().reset()
        self.write_register("DHCSR", 0x0)
        self.write_register("DCRSR", 0x0)
        self.write_register("DCRDR", 0x0)
        self.write_register("DEMCR", 0x0)
